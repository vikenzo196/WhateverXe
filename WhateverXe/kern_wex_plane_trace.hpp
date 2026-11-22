// Display handoff guards and trace for the primary plane (pipe A plane 1).
// The native framebuffer switches plane 1A to Y-tiling while it still scans out
// the linear firmware surface (and again on the way down), which shows the
// firmware image as white bands. The guards keep plane 1A dark in those windows:
// at boot until the native boot animation flips steadily, after boot whenever
// surface 0 is scanned out Y-tiled, and after the power-off/restart notice.
// The trace samples plane, cursor, PSR and eDP link registers every 5 ms for
// 25 s from just before native framebuffer start, stores an entry only on
// change, and publishes WEXPlaneTrace/WEXPlaneGuard in the registry root.
// Opt out: -wex-noplanetrace (all), -wex-noplaneguard (boot guard),
// -wex-noshutguard (surface 0 watch and power-off guard).
#pragma once
#include <IOKit/IOLib.h>
#include <libkern/c++/OSSerialize.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <kern/thread_call.h>
#include <IOKit/IOMessage.h>
#include <IOKit/pwr_mgt/RootDomain.h>
#include <IOKit/IODeviceTreeSupport.h>
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
inline bool watchEnabled;
inline void startWatch();
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
    uint32_t lastSurf=bootSurf;uint64_t flips[8] {};unsigned nflips=0;
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
            // flips steadily: 8 surface changes within 250 ms (the first native surfaces
            // change 3-4 times in a short burst before they are drawn). Timeout 6 s.
            const uint32_t ctl=rd(0x70180),surf=rd(0x7019C);
            const bool tiled=((ctl>>10)&7)!=0;
            if(state==1 && surf!=lastSurf) {
                lastSurf=surf;
                flips[nflips%8]=ns;++nflips;
                guardRecord[5]=nflips;
            }
            const bool steady=nflips>=8 && ns-flips[(nflips-8)%8]<250000000ULL;
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
    if(watchEnabled)startWatch();
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


// On-demand GT power sample (read-only, always-on/mirror range, no forcewake):
// WEXGTPower = [ns, GEN12_RPSTAT1 0x1381B4 (CAGF [19:11] x 50/3 MHz), GEN6_GT_GFX_RC6
// 0x138108 (1.28 us ticks), 0x145960, GT_PERF_STATUS 0x145948, RP_STATE_CAP 0x145998,
// host RPS frequency queries, answered queries, last encoded request].
// Each registry read of the property takes a fresh sample.
inline bool gtPowerSerialize(void *,void *,OSSerialize *serializer) {
    extern uint64_t tglHostRPSCalls,tglHostRPSAnswered;extern uint32_t tglHostRPSLast;
    uint64_t q[9] {};
    q[6]=__atomic_load_n(&tglHostRPSCalls,__ATOMIC_RELAXED);q[7]=__atomic_load_n(&tglHostRPSAnswered,__ATOMIC_RELAXED);
    q[8]=__atomic_load_n(&tglHostRPSLast,__ATOMIC_RELAXED);
    if(map && map->getLength()>=0x146000) {
        auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
        absolutetime_to_nanoseconds(mach_absolute_time(),q);
        const uint32_t regs[] {0x1381B4,0x138108,0x145960,0x145948,0x145998};
        for(unsigned i=0;i<5;++i)q[1+i]=*reinterpret_cast<volatile uint32_t *>(base+regs[i]);
    }
    auto data=OSData::withBytes(q,sizeof(q));if(!data)return false;
    const bool ok=data->serialize(serializer);data->release();return ok;
}
inline void publishGTPower() {
    if(auto s=OSSerializer::forTarget(nullptr,gtPowerSerialize)) {
        IORegistryEntry::getRegistryRoot()->setProperty("WEXGTPower",s);s->release();
    }
}

inline void started() {mark=2;publishGTPower();}

// Shutdown/restart: the native teardown scans out stale content through the
// Y-tiled plane for 2-3 s (bands). After the kernel announces power-off/restart
// keep plane 1A dark until the machine goes down. Record what was seen in
// NVRAM (wex-shutguard) since the registry does not survive the power cycle:
// [message, ctl at notice, surf at notice, suppressions, ms to last suppression].
inline thread_call_t shutCall;
inline IONotifier *shutNotifier;
inline uint32_t shutRecord[5];

inline void shutSave() {
    if(auto opts=IORegistryEntry::fromPath("/options",gIODTPlane)) {
        if(auto d=OSData::withBytes(shutRecord,sizeof(shutRecord))) {opts->setProperty("wex-shutguard",d);d->release();}
        opts->release();
    }
}

inline void shutWork(thread_call_param_t,thread_call_param_t) {
    auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
    auto rd=[base](uint32_t reg){return *reinterpret_cast<volatile uint32_t *>(base+reg);};
    auto wr=[base](uint32_t reg,uint32_t v){*reinterpret_cast<volatile uint32_t *>(base+reg)=v;};
    shutRecord[1]=rd(0x70180);shutRecord[2]=rd(0x7019C);
    uint64_t start=mach_absolute_time(),startNs=0;absolutetime_to_nanoseconds(start,&startNs);
    uint32_t saved=~0U;
    for(unsigned i=0;i<60000;++i) {
        const uint32_t ctl=rd(0x70180);
        if(ctl&0x80000000U) {
            wr(0x70180,ctl&~0x80000000U);wr(0x7019C,rd(0x7019C));
            uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
            ++shutRecord[3];shutRecord[4]=static_cast<uint32_t>((ns-startNs)/1000000ULL);
        }
        if(shutRecord[3]!=saved && (i%50)==0) {saved=shutRecord[3];shutSave();}
        IOSleep(1);
    }
}

// After boot, a light watcher (every 20 ms) keeps plane 1A dark whenever the
// native driver scans out the firmware surface 0 with Y-tiling: this never
// happens in normal use, only on the way down (before the power-off notice),
// where it shows the stale linear image as bands. Record in NVRAM wex-surf0guard:
// [suppressions, ctl seen, uptime s at first, uptime s at last].
inline thread_call_t watchCall;
inline uint32_t watchRecord[4];
inline void watchSave() {
    if(auto opts=IORegistryEntry::fromPath("/options",gIODTPlane)) {
        if(auto d=OSData::withBytes(watchRecord,sizeof(watchRecord))) {opts->setProperty("wex-surf0guard",d);d->release();}
        opts->release();
    }
}
inline void watchWork(thread_call_param_t,thread_call_param_t) {
    auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
    const uint32_t ctl=*reinterpret_cast<volatile uint32_t *>(base+0x70180);
    const uint32_t surf=*reinterpret_cast<volatile uint32_t *>(base+0x7019C);
    if((ctl&0x80000000U) && ((ctl>>10)&7) && surf==0) {
        *reinterpret_cast<volatile uint32_t *>(base+0x70180)=ctl&~0x80000000U;
        *reinterpret_cast<volatile uint32_t *>(base+0x7019C)=surf;
        uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
        const uint32_t sec=static_cast<uint32_t>(ns/1000000000ULL);
        if(!watchRecord[0])watchRecord[2]=sec;
        ++watchRecord[0];watchRecord[1]=ctl;watchRecord[3]=sec;
        if(watchRecord[0]<=4 || (watchRecord[0]%64)==0)watchSave();
    }
    uint64_t deadline=0;clock_interval_to_deadline(20,kMillisecondScale,&deadline);
    thread_call_enter_delayed(watchCall,deadline);
}
inline void startWatch() {
    if(watchCall || !map)return;
    watchCall=thread_call_allocate_with_priority(watchWork,nullptr,THREAD_CALL_PRIORITY_HIGH);
    if(watchCall)thread_call_enter(watchCall);
}

inline IOReturn shutHandler(void *,void *,UInt32 type,IOService *,void *,vm_size_t) {
    if((type==kIOMessageSystemWillPowerOff || type==kIOMessageSystemWillRestart) && shutCall && map && !shutRecord[0]) {
        shutRecord[0]=type==kIOMessageSystemWillPowerOff?1:2;
        shutSave();
        thread_call_enter(shutCall);
    }
    return kIOReturnSuccess;
}

// Call after native framebuffer start; opt out with -wex-noshutguard (checked by the caller).
inline void armShutdown() {
    if(shutNotifier || !map)return;
    watchEnabled=true;
    shutCall=thread_call_allocate_with_priority(shutWork,nullptr,THREAD_CALL_PRIORITY_HIGH);
    if(shutCall)shutNotifier=registerPrioritySleepWakeInterest(shutHandler,nullptr,nullptr);
}

}
