// On-demand flip timing trace (diagnostics, read-only MMIO). Include from one TU only.
// `sysctl -w debug.wexflip.arm=<seconds>` (1-120) starts a capture: a high priority
// thread polls pipe A every ~200 us and records each change of PLANE_SURF (flip
// programmed) and PLANE_SURFLIVE (flip latched by the hardware) for planes 1A-3A,
// with the pipe frame counter and scanline. The result is the registry root
// property WEXFlipTrace (16-byte entries), read with ioreg or a small tool.
// Comparing programmed vs latched frames tells whether a frame held one refresh
// longer was submitted late (compositor) or latched late (driver/hardware).
#pragma once
#include <sys/sysctl.h>
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <libkern/c++/OSSerialize.h>
#include <kern/thread_call.h>

namespace WEXFlipTrace {

struct Entry {
    uint32_t us;      // microseconds since capture start
    uint16_t frame;   // PIPE_FRMCOUNT_A 0x70040 (low 16 bits)
    uint16_t line;    // PIPEDSL_A 0x70000 scanline
    uint32_t surf;    // new surface address
    uint8_t plane;    // 1..3
    uint8_t kind;     // 0 programmed (PLANE_SURF), 1 latched (PLANE_SURFLIVE)
    uint16_t pad;
};

static constexpr unsigned MaxEntries=16384;
static Entry entries[MaxEntries];
static volatile uint32_t count;
static volatile uint32_t running;
static int armSeconds;
static IOMemoryMap *flipMap;
static thread_call_t flipCall;

static void work(thread_call_param_t,thread_call_param_t) {
    auto base=reinterpret_cast<volatile uint8_t *>(flipMap->getVirtualAddress());
    auto rd=[base](uint32_t reg){return *reinterpret_cast<volatile uint32_t *>(base+reg);};
    const uint32_t surfReg[3] {0x7019C,0x7029C,0x7039C},liveReg[3] {0x701AC,0x702AC,0x703AC};
    uint32_t lastSurf[3],lastLive[3];
    for(unsigned p=0;p<3;++p) {lastSurf[p]=rd(surfReg[p]);lastLive[p]=rd(liveReg[p]);}
    uint64_t start=mach_absolute_time(),startNs=0,limit=static_cast<uint64_t>(armSeconds)*1000000000ULL;
    absolutetime_to_nanoseconds(start,&startNs);
    __atomic_store_n(&count,0,__ATOMIC_RELEASE);
    for(;;) {
        uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);ns-=startNs;
        if(ns>limit)break;
        for(unsigned p=0;p<3;++p) {
            const uint32_t s=rd(surfReg[p]),l=rd(liveReg[p]);
            for(unsigned k=0;k<2;++k) {
                const uint32_t v=k?l:s;uint32_t &last=k?lastLive[p]:lastSurf[p];
                if(v==last)continue;
                last=v;
                const uint32_t n=__atomic_load_n(&count,__ATOMIC_RELAXED);
                if(n>=MaxEntries)continue;
                entries[n]={static_cast<uint32_t>(ns/1000),static_cast<uint16_t>(rd(0x70040)),
                    static_cast<uint16_t>(rd(0x70000)&0x1FFF),v,static_cast<uint8_t>(p+1),static_cast<uint8_t>(k),0};
                __atomic_store_n(&count,n+1,__ATOMIC_RELEASE);
            }
        }
        IODelay(200);
    }
    __atomic_store_n(&running,0,__ATOMIC_RELEASE);
}

static bool serialize(void *,void *,OSSerialize *serializer) {
    const uint32_t n=__atomic_load_n(&count,__ATOMIC_ACQUIRE);
    auto data=OSData::withBytes(entries,static_cast<unsigned int>(n*sizeof(Entry)));
    if(!data)return false;
    const bool ok=data->serialize(serializer);data->release();return ok;
}

static int armHandler(struct sysctl_oid *oidp,void *arg1,int arg2,struct sysctl_req *req) {
    int value=armSeconds;
    const int error=sysctl_handle_int(oidp,&value,0,req);
    if(error || !req->newptr)return error;
    if(value<1 || value>120 || !flipCall || !flipMap)return EINVAL;
    if(__atomic_exchange_n(&running,1,__ATOMIC_ACQ_REL))return EBUSY;
    armSeconds=value;
    thread_call_enter(flipCall);
    return 0;
}

SYSCTL_NODE(_debug, OID_AUTO, wexflip, CTLFLAG_RW, 0, "WhateverXe flip timing trace");
SYSCTL_PROC(_debug_wexflip, OID_AUTO, arm, CTLTYPE_INT|CTLFLAG_RW, nullptr, 0, armHandler, "I", "capture seconds");
SYSCTL_UINT(_debug_wexflip, OID_AUTO, count, CTLFLAG_RD, (unsigned int *)&count, 0, "entries captured");

// Call once after native framebuffer start; opt out with -wex-noflipsysctl (caller).
static void install(IOMemoryMap *map) {
    if(flipCall || !map)return;
    flipMap=map;flipMap->retain();
    flipCall=thread_call_allocate_with_priority(work,nullptr,THREAD_CALL_PRIORITY_HIGH);
    if(!flipCall)return;
    sysctl_register_oid(&sysctl__debug_wexflip);
    sysctl_register_oid(&sysctl__debug_wexflip_arm);
    sysctl_register_oid(&sysctl__debug_wexflip_count);
    if(auto s=OSSerializer::forTarget(nullptr,serialize)) {
        IORegistryEntry::getRegistryRoot()->setProperty("WEXFlipTrace",s);s->release();
    }
}

}
