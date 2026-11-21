// Boot handoff trace for the primary plane (pipe A plane 1), cursor, PSR and the
// eDP link. Diagnoses the ~1 s garbled boot logo seen when the native
// framebuffer takes over the firmware scanout. Samples MMIO every 5 ms for 25 s
// from just before native framebuffer start, stores an entry only on change,
// and publishes the entries as WEXPlaneTrace in the registry root.
// Opt out with -wex-noplanetrace (checked by the caller). Never writes hardware state.
#pragma once
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <kern/thread_call.h>
#include <Headers/kern_util.hpp>

namespace WEXPlaneTrace {

struct ConsoleInfo {
    unsigned int v_height, v_width, v_depth, v_rowbytes;
    unsigned long v_baseaddr;
    unsigned int v_type;
    char v_name[32];
    uint64_t v_physaddr;
    unsigned int v_rows, v_columns, v_rowscanbytes, v_scale, v_rotate, v_reserved[3];
};

struct Entry {
    uint64_t ns;          // nanoseconds since mach absolute time zero
    uint32_t mark;        // 1 before native start, 2 after native start returned
    uint32_t ctl;         // PLANE_CTL_1_A 0x70180 (bit31 enable, 12:10 tiling)
    uint32_t stride;      // PLANE_STRIDE_1_A 0x70188
    uint32_t size;        // PLANE_SIZE_1_A 0x70190
    uint32_t surf;        // PLANE_SURF_1_A 0x7019C
    uint32_t live;        // PLANE_SURFLIVE_1_A 0x701AC
    uint32_t psr;         // EDP_PSR_CTL transcoder A 0x60800
    uint32_t psrStat;     // EDP_PSR_STATUS transcoder A 0x60840
    uint32_t psr2;        // EDP_PSR2_CTL transcoder A 0x60900
    uint32_t psr2Stat;    // EDP_PSR2_STATUS transcoder A 0x60940
    uint32_t curCtl;      // CUR_CTL_A 0x70080
    uint32_t curBase;     // CUR_BASE_A 0x70084
    uint32_t ddiBuf;      // DDI_BUF_CTL_A 0x64000
    uint64_t dpTp;        // DP_TP_CTL transcoder A 0x60540
};

inline constexpr unsigned MaxEntries=200;
inline Entry entries[MaxEntries];
inline volatile uint32_t count;
inline volatile uint32_t mark;
inline IOMemoryMap *map;
inline thread_call_t call;
inline ConsoleInfo *console;
inline bool guardEnabled;
// Guard record: state (0 idle, 1 suppressed, 2 released), suppress count, release reason
// (1 steady flips, 2 timeout), ns at first suppress, ns at release, surface changes seen, boot control,
// longest gap between guard polls while active (ns) and when it ended (ns).
inline uint64_t guardRecord[9];

inline void setConsole(void *vinfo) {console=static_cast<ConsoleInfo *>(vinfo);}

inline void publish() {
    const uint32_t n=__atomic_load_n(&count,__ATOMIC_ACQUIRE);
    auto data=OSData::withBytes(entries,static_cast<unsigned int>(n*sizeof(Entry)));
    if(data) {IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneTrace",data);data->release();}
    if(guardEnabled)IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneGuard",guardRecord,sizeof(guardRecord));
}

inline void work(thread_call_param_t,thread_call_param_t) {
    auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
    auto rd=[base](uint32_t reg){return *reinterpret_cast<volatile uint32_t *>(base+reg);};
    auto wr=[base](uint32_t reg,uint32_t v){*reinterpret_cast<volatile uint32_t *>(base+reg)=v;};
    Entry last {};
    bool first=true;
    uint32_t bootSurf=rd(0x7019C),bootCtl=rd(0x70180);
    const bool bootLinear=((bootCtl>>10)&7)==0 && (bootCtl&0x80000000U);
    guardRecord[6]=bootCtl;
    uint32_t state=0;uint64_t suppressNs=0;
    uint32_t lastSurf=bootSurf;uint64_t flips[4] {};unsigned nflips=0;
    uint64_t startNs=0,lastPoll=0,lastSample=0;absolutetime_to_nanoseconds(mach_absolute_time(),&startNs);
    for(unsigned i=0;__atomic_load_n(&count,__ATOMIC_RELAXED)<MaxEntries;++i) {
        Entry e {};
        uint64_t now=mach_absolute_time(),ns=0;absolutetime_to_nanoseconds(now,&ns);
        if(ns-startNs>25000000000ULL)break;
        const bool guarding=guardEnabled && bootLinear && state!=2 && ns-startNs<15000000000ULL;
        if(guarding && lastPoll && ns-lastPoll>guardRecord[7]) {guardRecord[7]=ns-lastPoll;guardRecord[8]=ns;}
        lastPoll=ns;
        if(guarding) {
            // Native start switches plane 1A to Y-tiling before it points the plane at its
            // own surface; the firmware's linear image is then scanned out as tiles (the
            // white bands). The first native surfaces are not drawn yet either (white lines
            // with the cursor), so keep the plane dark until the boot progress animation
            // flips steadily: 4 surface changes within 150 ms. Timeout 6 s.
            const uint32_t ctl=rd(0x70180),surf=rd(0x7019C);
            const bool tiled=((ctl>>10)&7)!=0;
            if(state==1 && surf!=lastSurf) {
                lastSurf=surf;
                flips[nflips%4]=ns;++nflips;
                guardRecord[5]=nflips;
            }
            const bool steady=nflips>=4 && ns-flips[(nflips-4)%4]<150000000ULL;
            if(state==1 && (steady || ns-suppressNs>6000000000ULL)) {
                if(!(ctl&0x80000000U)) {wr(0x70180,ctl|0x80000000U);wr(0x7019C,surf);}
                state=2;guardRecord[2]=steady?1:2;guardRecord[4]=ns;
            } else if(tiled && (ctl&0x80000000U) && (state==1 || surf==bootSurf) && guardRecord[1]<512) {
                wr(0x70180,ctl&~0x80000000U);wr(0x7019C,surf);
                if(state==0)suppressNs=ns,guardRecord[3]=ns;
                state=1;++guardRecord[1];
            }
            guardRecord[0]=state;
        }
        if(guarding && ns-lastSample<5000000ULL && i) {IOSleep(1);continue;}
        lastSample=ns;
        e.ns=ns;e.mark=mark;
        e.ctl=rd(0x70180);e.stride=rd(0x70188);e.size=rd(0x70190);e.surf=rd(0x7019C);
        e.live=rd(0x701AC);e.psr=rd(0x60800);e.psrStat=rd(0x60840);e.psr2=rd(0x60900);
        e.psr2Stat=rd(0x60940);e.curCtl=rd(0x70080);e.curBase=rd(0x70084);e.ddiBuf=rd(0x64000);
        e.dpTp=rd(0x60540);
        // PSR status registers also carry counters; compare only their state fields.
        const bool changed=first || e.mark!=last.mark || e.ctl!=last.ctl || e.stride!=last.stride ||
            e.size!=last.size || e.surf!=last.surf || e.live!=last.live || e.psr!=last.psr ||
            (e.psrStat>>29)!=(last.psrStat>>29) || e.psr2!=last.psr2 ||
            (e.psr2Stat>>28)!=(last.psr2Stat>>28) || e.curCtl!=last.curCtl || e.curBase!=last.curBase ||
            e.ddiBuf!=last.ddiBuf || e.dpTp!=last.dpTp;
        if(changed) {
            const uint32_t n=__atomic_load_n(&count,__ATOMIC_RELAXED);
            entries[n]=e;__atomic_store_n(&count,n+1,__ATOMIC_RELEASE);
            last=e;first=false;
        }
        if((i%200)==199)publish();
        IOSleep(guarding?1:5);
    }
    publish();
    IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneTraceDone",true);
}

// Call just before the native framebuffer controller start.
inline void start(IOPCIDevice *pci,bool guard) {
    if(call || !pci)return;
    guardEnabled=guard;
    map=pci->mapDeviceMemoryWithRegister(0x10,kIOMapInhibitCache);
    if(!map || map->getLength()<0x80000) {OSSafeReleaseNULL(map);return;}
    call=thread_call_allocate_with_priority(work,nullptr,THREAD_CALL_PRIORITY_HIGH);
    if(!call) {OSSafeReleaseNULL(map);return;}
    mark=1;
    IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneTraceEntrySize",static_cast<unsigned long long>(sizeof(Entry)),32);
    thread_call_enter(call);
    IOSleep(15); // let the first sample record the firmware scanout before native start
}

inline void started() {mark=2;}

}
