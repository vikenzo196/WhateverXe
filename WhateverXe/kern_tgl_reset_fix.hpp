// Fork: render-engine register restore after the native TGL engine reset.
//
// Observed on Raptor Lake-H (GB3U): the first engine reset restores the
// donor's RCS reset-register list (HWS_PGA 0x2080, GFX_MODE 0x229c, ...),
// later resets leave every RCS register at its power-on value. Without
// HWS_PGA and GFX_MODE (bit 3, 12-entry CSB) the command streamer cannot
// report completion and the GPU stays hung until a forced reboot.
//
// This keeps the donor's own list layout (IGVector<resetRegisterInfo>:
// count at +0, entries at +0x10, stride 0x24, offset +0, masked flag +8),
// remembers the live values while the engine is configured, and replays them
// with the donor's restore semantics (masked -> value | 0xffff0000) only when
// HWS_PGA reads 0 after a reset. No allocation, no new locking.
#ifndef kern_tgl_reset_fix_hpp
#define kern_tgl_reset_fix_hpp
#include <stdint.h>

namespace TGLResetFix {
constexpr uint32_t HWSPGA=0x2080;
constexpr unsigned MaxRegs=64, EntryStride=0x24;

struct Cache {
	uint32_t count, valid, refreshes, replays;
	uint32_t off[MaxRegs], val[MaxRegs];
	uint8_t masked[MaxRegs];
};

inline uint32_t read(volatile uint8_t *mmio,uint32_t off) {
	return *reinterpret_cast<volatile uint32_t *>(mmio+off);
}
inline void write(volatile uint8_t *mmio,uint32_t off,uint32_t value) {
	*reinterpret_cast<volatile uint32_t *>(mmio+off)=value;
}

// Copy the register layout once. Rejects anything that is not an RCS list
// with a sane size, sane MMIO offsets, and HWS_PGA among its entries.
inline bool loadLayout(Cache &c,const void *list) {
	if (c.count || !list) return c.count!=0;
	auto p=static_cast<const uint8_t *>(list);
	const auto count=*reinterpret_cast<const uint64_t *>(p);
	auto data=*reinterpret_cast<const uint8_t *const *>(p+0x10);
	if (!count || count>MaxRegs || !data) return false;
	bool hws=false;
	for (unsigned i=0;i<count;++i) {
		const auto off=*reinterpret_cast<const uint32_t *>(data+i*EntryStride);
		if (!off || off>=0x200000 || (off&3)) return false;
		hws|=off==HWSPGA;
	}
	if (!hws) return false;
	for (unsigned i=0;i<count;++i) {
		c.off[i]=*reinterpret_cast<const uint32_t *>(data+i*EntryStride);
		c.masked[i]=data[i*EntryStride+8]!=0;
	}
	c.count=static_cast<uint32_t>(count);
	return true;
}

// Remember live values, only while the engine is configured.
inline bool refresh(Cache &c,volatile uint8_t *mmio) {
	if (!c.count || !read(mmio,HWSPGA)) return false;
	for (unsigned i=0;i<c.count;++i) c.val[i]=read(mmio,c.off[i]);
	c.valid=1;++c.refreshes;
	return true;
}

inline bool needsReplay(const Cache &c,volatile uint8_t *mmio) {
	return c.valid && c.count && !read(mmio,HWSPGA);
}

// Donor restore semantics. Returns the number of registers written.
inline unsigned replay(Cache &c,volatile uint8_t *mmio) {
	if (!needsReplay(c,mmio)) return 0;
	for (unsigned i=0;i<c.count;++i)
		write(mmio,c.off[i],c.masked[i] ? (c.val[i]|0xFFFF0000U) : c.val[i]);
	++c.replays;
	return c.count;
}
}
#endif
