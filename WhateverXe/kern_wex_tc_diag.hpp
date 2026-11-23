// Type-C / external DP diagnostics (read-only). Include from one TU only.
// - WEXAUXTrace (registry root, on demand): last 128 donor AUX transactions
//   (native and I2C-over-AUX) with the DDI, whether the fork let them through,
//   the donor command/length, the first packet word before/after and results.
// - WEXTCRegs (registry root, on demand): a snapshot of HPD, FIA, DDI buffer,
//   transcoder and power-well registers, read when ioreg asks for it.
// - NVRAM wex-tcboot: how far Display 13 framebuffer start got on this boot,
//   with the firmware's transcoder/DDI state at entry. Kept in NVRAM so a boot
//   that ends in a black screen can be read on the next one.
// Opt out with -wex-notcdiag.
#pragma once
#include <IOKit/IOLib.h>
#include <IOKit/IOMemoryDescriptor.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <libkern/c++/OSSerialize.h>

namespace WEXTCDiag {

struct AUXEntry {
    uint32_t seq;
    uint8_t kind;      // 1 native AUX, 2 I2C-over-AUX
    uint8_t ddi;       // donor port DDI (field 0x120), 0xFF if unknown
    uint8_t admitted;  // fork readiness check before the call
    uint8_t complete;  // fork readiness check after the call
    uint32_t command;
    uint32_t length;
    uint32_t before;   // first packet word before the call
    uint32_t after;    // first packet word after the call
    uint32_t before1;  // second packet word before the call (write payload)
    uint32_t after1;   // second packet word after the call (read data)
    uint32_t us;       // microseconds since boot
    uint32_t donor;    // donor return value (0 when refused by the fork)
    uint32_t result;   // value returned to the donor
};

static constexpr unsigned AUXMax=1024;
inline AUXEntry aux[AUXMax];
inline volatile uint32_t auxSeq;
inline volatile uint32_t mstHidden;
inline bool enabled;
inline IOMemoryMap *regMap;

inline void init() {enabled=!checkKernelArgument("-wex-notcdiag");}

inline uint32_t peek(const uint32_t *packet) {return packet ? *packet : 0;}
inline uint32_t peek1(const uint32_t *packet,uint32_t length) {return packet && length>4 ? packet[1] : 0;}

// The internal eDP (DDI 0) is not recorded: its backlight writes would flood the ring.
inline void recordAUX(uint8_t kind,void *port,bool admitted,bool complete,uint32_t command,uint32_t length,
                      uint32_t before,uint32_t after,uint32_t donor,uint32_t result,uint32_t before1=0,uint32_t after1=0) {
    if(!enabled)return;
    uint8_t ddi=0xFF;
    if(port)ddi=static_cast<uint8_t>(*reinterpret_cast<const uint32_t *>(static_cast<const uint8_t *>(port)+0x120));
    if(ddi==0)return;
    const uint32_t n=__atomic_add_fetch(&auxSeq,1,__ATOMIC_ACQ_REL);
    uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
    aux[(n-1)%AUXMax]={n,kind,ddi,admitted,complete,command,length,before,after,before1,after1,
        static_cast<uint32_t>(ns/1000),donor,result};
}

inline bool serializeAUX(void *,void *,OSSerialize *serializer) {
    const uint32_t n=__atomic_load_n(&auxSeq,__ATOMIC_ACQUIRE);
    const uint32_t first=n>AUXMax ? n-AUXMax : 0;
    auto data=OSData::withCapacity(static_cast<unsigned int>((n-first)*sizeof(AUXEntry)));
    if(!data)return false;
    for(uint32_t i=first;i<n;++i)data->appendBytes(&aux[i%AUXMax],sizeof(AUXEntry));
    const bool ok=data->serialize(serializer);data->release();return ok;
}


// Hook results on the training path (external ports only).
struct HookEntry {uint32_t seq,us;uint8_t id,ddi,pad0,pad1;uint32_t result,arg,byte115_117_10_11A;};
static constexpr unsigned HookMax=512;
inline HookEntry hooks[HookMax];
inline volatile uint32_t hookSeq;

inline void recordHook(uint8_t id,void *object,uint32_t result,uint32_t arg,uint32_t isPath) {
    if(!enabled || !object)return;
    auto o=static_cast<OSObject *>(object);
    if(o->getMetaClass()->getClassSize()<(isPath?0x3730U:0x624U))return;
    auto p=static_cast<const uint8_t *>(object);
    if(isPath) {
        auto port=*reinterpret_cast<OSObject * const *>(p+0x3648);
        if(!port || port->getMetaClass()->getClassSize()<0x624U)return;
        p=reinterpret_cast<const uint8_t *>(port);
    }
    const uint32_t ddi=*reinterpret_cast<const uint32_t *>(p+0x120);
    if(ddi==0 || ddi>8)return;
    const uint32_t flags=p[0x115]|(p[0x117]<<8)|(p[0x10]<<16)|(static_cast<uint32_t>(p[0x11A])<<24);
    const uint32_t n=__atomic_add_fetch(&hookSeq,1,__ATOMIC_ACQ_REL);
    uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);
    hooks[(n-1)%HookMax]={n,static_cast<uint32_t>(ns/1000),id,static_cast<uint8_t>(ddi),0,0,result,arg,flags};
}

inline bool serializeHooks(void *,void *,OSSerialize *serializer) {
    const uint32_t n=__atomic_load_n(&hookSeq,__ATOMIC_ACQUIRE);
    const uint32_t first=n>HookMax ? n-HookMax : 0;
    auto data=OSData::withCapacity(static_cast<unsigned int>((n-first)*sizeof(HookEntry)));
    if(!data)return false;
    for(uint32_t i=first;i<n;++i)data->appendBytes(&hooks[i%HookMax],sizeof(HookEntry));
    const bool ok=data->serialize(serializer);data->release();return ok;
}

// Registers: HPD (north TC bank), TC hotplug control, FIA1/FIA2 lane and mode
// state, DDI_BUF_CTL A..I, AUX_CH_CTL A..I, TRANS_DDI_FUNC_CTL A..D,
// TRANSCONF A..D, power well request/status.
static constexpr uint32_t regList[]={
    0x44470,0x44474,0x44478,0x4447C,0x44038,0xC4000,0xC4030,
    0x163880,0x163890,0x163894,0x1638A0,0x1638C0,
    0x16E880,0x16E890,0x16E894,0x16E8A0,0x16E8C0,
    0x64000,0x64100,0x64200,0x64300,0x64400,0x64500,0x64600,0x64700,0x64800,
    0x64010,0x64110,0x64210,0x64310,0x64410,0x64510,0x64610,0x64710,0x64810,
    0x60400,0x61400,0x62400,0x63400,
    0x70008,0x71008,0x72008,0x73008,
    0x45400,0x45404,0x45410,0x45414,0x45440,0x45444,
};
static constexpr unsigned RegCount=sizeof(regList)/sizeof(regList[0]);

inline void readRegs(IOMemoryMap *map,uint32_t *out) {
    auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
    const auto length=map->getLength();
    for(unsigned i=0;i<RegCount;++i)
        out[i]=regList[i]+4<=length ? *reinterpret_cast<volatile uint32_t *>(base+regList[i]) : 0xDEADBEEF;
}

inline bool serializeRegs(void *,void *,OSSerialize *serializer) {
    uint32_t pairs[RegCount*2];
    uint32_t values[RegCount];
    if(regMap)readRegs(regMap,values);
    for(unsigned i=0;i<RegCount;++i){pairs[2*i]=regList[i];pairs[2*i+1]=regMap ? values[i] : 0;}
    auto data=OSData::withBytes(pairs,sizeof(pairs));
    if(!data)return false;
    const bool ok=data->serialize(serializer);data->release();return ok;
}

// NVRAM boot record: [version, step, ms since boot at step, bootReady, transcoders
// A..D func ctl, transconf A..D, DDI_BUF_CTL TC1..TC4, DE_HPD_ISR, TC_HOTPLUG_CTL].
inline uint32_t bootRecord[19];

inline void bootSave() {
    if(auto opts=IORegistryEntry::fromPath("/options",gIODTPlane)) {
        if(auto d=OSData::withBytes(bootRecord,sizeof(bootRecord))) {opts->setProperty("wex-tcboot",d);d->release();}
        opts->release();
    }
}

inline uint32_t uptimeMs() {uint64_t ns=0;absolutetime_to_nanoseconds(mach_absolute_time(),&ns);return static_cast<uint32_t>(ns/1000000);}

inline void bootStep(uint32_t step) {
    if(!enabled)return;
    bootRecord[1]=step;bootRecord[2]=uptimeMs();bootSave();
}

// Keep the previous boot's record (a boot that hung never overwrote it with step 5).
inline void keepPrevious() {
    if(auto opts=IORegistryEntry::fromPath("/options",gIODTPlane)) {
        if(auto old=OSDynamicCast(OSData,opts->getProperty("wex-tcboot"))) {
            if(auto d=OSData::withData(old)){opts->setProperty("wex-tcboot-prev",d);d->release();}
        }
        opts->release();
    }
}

// Framebuffer start entry: capture what the firmware left lit, then save.
inline void bootEntry(IOPCIDevice *pci,bool bootReady) {
    if(!enabled || !pci)return;
    keepPrevious();
    bootRecord[0]=1;bootRecord[3]=bootReady;
    if(auto map=pci->mapDeviceMemoryWithIndex(0)) {
        auto base=reinterpret_cast<volatile uint8_t *>(map->getVirtualAddress());
        auto rd=[base](uint32_t r){return *reinterpret_cast<volatile uint32_t *>(base+r);};
        const uint32_t trans[4]={0x60400,0x61400,0x62400,0x63400},conf[4]={0x70008,0x71008,0x72008,0x73008};
        for(unsigned i=0;i<4;++i){bootRecord[4+i]=rd(trans[i]);bootRecord[8+i]=rd(conf[i]);bootRecord[12+i]=rd(0x64300+0x100*i);}
        bootRecord[16]=rd(0x44470);bootRecord[17]=rd(0x44038);bootRecord[18]=0;
        map->release();
    }
    bootStep(1);
}

// After native start (and the plane-trace mapping exists): publish on-demand views.
inline void install(IOMemoryMap *map) {
    if(!enabled)return;
    if(map && !regMap){regMap=map;regMap->retain();}
    auto root=IORegistryEntry::getRegistryRoot();
    if(auto s=OSSerializer::forTarget(nullptr,serializeAUX)){root->setProperty("WEXAUXTrace",s);s->release();}
    if(auto s=OSSerializer::forTarget(nullptr,serializeRegs)){root->setProperty("WEXTCRegs",s);s->release();}
    if(auto s=OSSerializer::forTarget(nullptr,serializeHooks)){root->setProperty("WEXTCHooks",s);s->release();}
    if(auto s=OSSerializer::forTarget(nullptr,[](void *,void *,OSSerialize *z){
        auto n=OSNumber::withNumber(__atomic_load_n(&mstHidden,__ATOMIC_RELAXED),32);if(!n)return false;
        const bool ok=n->serialize(z);n->release();return ok;})){root->setProperty("WEXTCMSTHidden",s);s->release();}
}

}
