// Read-only boot handoff trace for the primary plane (pipe A plane 1) and the
// kernel console. Diagnoses the ~1 s garbled boot logo seen when the native
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
    uint32_t offset;      // PLANE_OFFSET_1_A 0x701A4
    uint32_t ctl2;        // PLANE_CTL_2_A 0x70280
    uint32_t surf2;       // PLANE_SURF_2_A 0x7029C
    uint32_t vrow, vwidth, vheight, vdepth;
    uint64_t vbase;       // kernel console v_baseaddr
};

inline constexpr unsigned MaxEntries=96;
inline Entry entries[MaxEntries];
inline volatile uint32_t count;
inline volatile uint32_t mark;
inline IOMemoryMap *map;
inline thread_call_t call;
inline ConsoleInfo *console;

inline void setConsole(void *vinfo) {console=static_cast<ConsoleInfo *>(vinfo);}

inline void publish() {
    const uint32_t n=__atomic_load_n(&count,__ATOMIC_ACQUIRE);
    auto data=OSData::withBytes(entries,static_cast<unsigned int>(n*sizeof(Entry)));
    if(data) {IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneTrace",data);data->release();}
}

inline void work(thread_call_param_t,thread_call_param_t) {
    auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
    auto rd=[base](uint32_t reg){return *reinterpret_cast<volatile uint32_t *>(base+reg);};
    Entry last {};
    bool first=true;
    for(unsigned i=0;i<5000 && __atomic_load_n(&count,__ATOMIC_RELAXED)<MaxEntries;++i) {
        Entry e {};
        uint64_t now=mach_absolute_time(),ns=0;absolutetime_to_nanoseconds(now,&ns);
        e.ns=ns;e.mark=mark;
        e.ctl=rd(0x70180);e.stride=rd(0x70188);e.size=rd(0x70190);e.surf=rd(0x7019C);
        e.live=rd(0x701AC);e.offset=rd(0x701A4);e.ctl2=rd(0x70280);e.surf2=rd(0x7029C);
        if(console) {
            e.vrow=console->v_rowbytes;e.vwidth=console->v_width;e.vheight=console->v_height;
            e.vdepth=console->v_depth;e.vbase=console->v_baseaddr;
        }
        const bool changed=first || e.mark!=last.mark || e.ctl!=last.ctl || e.stride!=last.stride ||
            e.size!=last.size || e.surf!=last.surf || e.live!=last.live || e.offset!=last.offset ||
            e.ctl2!=last.ctl2 || e.surf2!=last.surf2 || e.vrow!=last.vrow || e.vbase!=last.vbase ||
            e.vwidth!=last.vwidth || e.vdepth!=last.vdepth;
        if(changed) {
            const uint32_t n=__atomic_load_n(&count,__ATOMIC_RELAXED);
            entries[n]=e;__atomic_store_n(&count,n+1,__ATOMIC_RELEASE);
            last=e;first=false;
        }
        if((i%200)==199)publish();
        IOSleep(5);
    }
    publish();
    IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneTraceDone",true);
}

// Call just before the native framebuffer controller start.
inline void start(IOPCIDevice *pci) {
    if(call || !pci)return;
    map=pci->mapDeviceMemoryWithRegister(0x10,kIOMapInhibitCache);
    if(!map || map->getLength()<0x80000) {OSSafeReleaseNULL(map);return;}
    call=thread_call_allocate(work,nullptr);
    if(!call) {OSSafeReleaseNULL(map);return;}
    mark=1;
    IORegistryEntry::getRegistryRoot()->setProperty("WEXPlaneTraceEntrySize",static_cast<unsigned long long>(sizeof(Entry)),32);
    thread_call_enter(call);
    IOSleep(15); // let the first sample record the firmware scanout before native start
}

inline void started() {mark=2;}

}
