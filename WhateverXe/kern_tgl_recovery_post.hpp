// Fork diagnostics: render-engine state after the native TGL engine reset.
// Read-only MMIO samples, taken inside the recovery callback right after
// resetAndResume and later from a thread call (+20ms, +200ms, +1s, +3s), each
// under the donor's own counted SafeForceWake(true/false, 7) pair.
// Published as WEXRecoveryPost. No writes, no changed recovery behaviour.
#ifndef kern_tgl_recovery_post_hpp
#define kern_tgl_recovery_post_hpp
#include <stdint.h>

namespace TGLRecoveryPost {
constexpr uint32_t Registers[] {
 0x2030,0x2034,0x2038,0x203c, // RCS ring tail/head/start/ctl
 0x2064,0x2068,0x206c,0x2074, // IPEIR, IPEHR, INSTDONE, ACTHD
 0x2080,0x209c,0x20c0,0x2140, // HWS_PGA, MI_MODE, INSTPM, BBADDR
 0x2234,0x2238,0x2244,0x229c, // EXECLIST_STATUS lo/hi, CTX ctl, GFX_MODE
 0x23a0,0x2510,0x2514,0x2550, // CSB pointers, ELSQ lo/hi, EXECLIST_CONTROL
 0x941c,0xd84,0x20b0,0x2580}; // GDRST, render FW ack, EIR, ccs/ctx mode
constexpr unsigned RegisterCount=sizeof(Registers)/sizeof(Registers[0]);
constexpr unsigned Samples=5, Recoveries=8;
struct Record {
 uint64_t valid, recovery, ring, mmio, accelerator; // valid = sample bitmask
 uint64_t reserved;
 uint64_t time[Samples];                    // mach absolute time per sample
 uint32_t value[Samples][RegisterCount];
};
static_assert(sizeof(Record)%8==0,"WEXRecoveryPost layout");
}
#endif
