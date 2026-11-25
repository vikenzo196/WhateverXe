//  XE-PM, from JulioNectar's NootedGreen fork (Hackintosh-XPS13Plus-IrisXe-Tahoe,
//  commit 0492b29, src/NootedGreen/kern_xepm.cpp), shared with the hackintosh
//  community. Adapted for WhateverXe: boot-arg -wex-pm, EPP fixed at 0 (sysctl
//  debug.xepm.epp at runtime), log prefix WEX.
//  XE-PM: CPU performance and idle power profile for hybrid Alder Lake / Raptor Lake (and Tiger Lake) under XCPM.
//  Opt-in with -wex-pm. Everything is applied 20 s after the patcher loads (system up), re-checked at 60 s.
//
//  1. HWP units (hybrid P-cores). XCPM writes IA32_HWP_REQUEST (0x774) max/desired in 100 MHz ratio units (max = the
//     TURBO_RATIO_LIMIT value, 50 on an i7-1360P). On hybrid P-cores one HWP performance unit is ~78.7 MHz (Linux
//     intel_pstate hybrid scaling factor 78741; HWP_CAPABILITIES highest = 64 for 5.0 GHz), so max 50 caps them at
//     ~3.9 GHz. The static XCPM routine at _xcpm_enable_pstate_ctrs+0x1190 (macOS 26.7.1 25G241) takes the target
//     multiplier and writes the local CPU's request; it is routed, and after Apple's write the request is rescaled on
//     cores whose highest perf exceeds the requested max (max -> highest, desired -> desired * highest / max). The
//     route is installed only after the prologue and the write site match byte for byte.
//  2. EPP. XCPM's frequency vector asks EPP 0x80. On ADL/RPL mobile parts that clips one-core turbo (Intel, Linux 5.17
//     intel_pstate: EPP <= 102 needed). The profile writes EPP 0 (sysctl debug.xepm.epp to change).
//  3. Energy-efficient turbo off (MSR_POWER_CTL 0x1FC bit 19).
//  4. XCPM idle table (_xcpm_cstate_table): for the spoofed CPU it has C8 (0x40) and C10 (0x60) disabled and C3 (0x10,
//     not an ADL/RPL/TGL state) enabled, so cores stop at C6 or fall back to C1. C8/C10 are enabled and C3 disabled,
//     only if the three entries carry exactly those hints.
//  5. D3hot for PCI functions macOS has no driver for (IPU camera, GNA, ISH, DPTF participant, Thunderbolt/USB4 DMA),
//     by device id, only when nothing is attached (or only a client that never matched).
//  Measured on a Dell XPS 13 Plus 9320 (i7-1360P): Geekbench 7 single-core 1269 -> 1697, multi-core 5859 -> 6148,
//  one-thread clock 3.3 -> 4.3-4.85 GHz, idle package ~4.6 -> ~3.3 W.
#include <Headers/kern_api.hpp>
#include <Headers/kern_patcher.hpp>
#include <IOKit/IOLib.h>
#include <IOKit/pci/IOPCIDevice.h>
#include <kern/thread_call.h>
#include <sys/sysctl.h>
#include <IOKit/IOMessage.h>
#include <IOKit/acpi/IOACPIPlatformDevice.h>
#include <IOKit/pwr_mgt/RootDomain.h>

extern "C" void mp_rendezvous_no_intrs(void (*fn)(void *), void *arg);

static inline uint64_t xpRdmsr(uint32_t m) { uint32_t lo, hi; asm volatile("rdmsr" : "=a"(lo), "=d"(hi) : "c"(m)); return ((uint64_t)hi << 32) | lo; }
static inline void xpWrmsr(uint32_t m, uint64_t v) { asm volatile("wrmsr" : : "c"(m), "a"((uint32_t)v), "d"((uint32_t)(v >> 32))); }

static bool gXpOn = false;
static int gXpHwp = 0, gXpEpp = -1;
static int gXpDes = -1, gXpMin = -1;                 // sysctl debug.xepm.desired / .min (root), -1 = XCPM's
static uint64_t gXpLastReq = 0;                      // last IA32_HWP_REQUEST written (any CPU)                 // sysctl debug.xepm.hwp / .epp (root)
static int gXpEet = 0, gXpCst = 0, gXpD3 = 0, gXpRoute = 0;   // status
static mach_vm_address_t gXpOrgSetMult = 0, gXpCstPtr = 0;

// ─── 1/2: XCPM HWP request ───────────────────────────────────────────────────────────────────────────────────────
static uint64_t xpSetMult(uint64_t mult, uint64_t a1, uint64_t a2, uint64_t a3) {   // only edi is used by Apple
	const uint64_t r = FunctionCast(xpSetMult, gXpOrgSetMult)(mult, a1, a2, a3);
	// XCPM calls this before it enables HWP; the HWP MSRs #GP until IA32_PM_ENABLE bit 0 is set.
	if (gXpHwp && (xpRdmsr(0x770) & 1)) {
		const uint64_t v = xpRdmsr(0x774);
		if (v & (1ULL << 42)) return r;   // package control: leave it alone
		const uint32_t hi = (uint32_t)(xpRdmsr(0x771) & 0xff), max = (uint32_t)(v >> 8) & 0xff, des = (uint32_t)(v >> 16) & 0xff;
		uint64_t nv = v;
		uint32_t mx = max, d = des, mn = (uint32_t)v & 0xff;
		if (max && hi > max && hi < 0xff) {
			d = des ? (des * hi + max / 2) / max : 0;
			if (d > hi) d = hi;
			mx = hi;
		}
		// desired 0 = autonomous HWP: the core picks its own frequency, steered by EPP.
		if (gXpDes >= 0) d = (uint32_t)gXpDes > mx ? mx : (uint32_t)gXpDes;
		if (gXpMin >= 0) mn = (uint32_t)gXpMin > mx ? mx : (uint32_t)gXpMin;
		nv = (nv & ~0xFFFFFFULL) | mn | ((uint64_t)mx << 8) | ((uint64_t)d << 16);
		if (gXpEpp >= 0) nv = (nv & ~0xFF000000ULL) | ((uint64_t)gXpEpp << 24);
		if (nv != v) xpWrmsr(0x774, nv);
		gXpLastReq = nv;
	}
	return r;
}

// ─── 3: energy-efficient turbo ───────────────────────────────────────────────────────────────────────────────────
static void xpEetOne(void *) {
	const uint64_t v = xpRdmsr(0x1FC);
	if (!(v & (1ULL << 19))) xpWrmsr(0x1FC, v | (1ULL << 19));
}

// ─── 4: XCPM idle table ──────────────────────────────────────────────────────────────────────────────────────────
static int xpCstFix() {
	uint8_t *tab = gXpCstPtr ? *reinterpret_cast<uint8_t **>(gXpCstPtr) : nullptr;
	if (!tab) return 2;
	auto hint = [&](int i) { return *reinterpret_cast<volatile uint32_t *>(tab + i * 24 + 4); };
	auto en = [&](int i) { return reinterpret_cast<volatile uint32_t *>(tab + i * 24 + 8); };
	if (hint(0) != 0x60 || hint(2) != 0x40 || hint(6) != 0x10) return 3;
	if (*en(0) == 1 && *en(2) == 1 && *en(6) == 0) return 1;
	if (MachInfo::setKernelWriting(true, KernelPatcher::kernelWriteLock) != KERN_SUCCESS) return 4;
	*en(0) = 1; *en(2) = 1; *en(6) = 0;
	MachInfo::setKernelWriting(false, KernelPatcher::kernelWriteLock);
	return 1;
}

// ─── 5: D3hot for driverless functions ───────────────────────────────────────────────────────────────────────────
static int xpD3() {
	static const uint16_t ids[] = {
		0xa75d, 0x465d, 0x9a19,           // IPU6 camera (RPL-P, ADL-P, TGL)
		0xa74f, 0x464f, 0x9a11,           // GNA
		0x51fc, 0x54fc, 0xa0fc,           // ISH (ADP-P, ADP-N, TGP-LP)
		0xa71d, 0x461d, 0x9a03,           // DPTF processor participant
		0xa73e, 0xa76d, 0x463e, 0x466d, 0x9a1b, 0x9a1d,   // Thunderbolt 4 / USB4 DMA (macOS has no host driver here)
		0x51d8, 0x51d9,                   // ADP-P LPSS I2C6/I2C7 (the THC IP slot): left in D0 without a client it keeps a 517 ns LTR
	};
	int n = 0;
	OSIterator *it = IOService::getMatchingServices(IOService::serviceMatching("IOPCIDevice"));
	if (!it) return 0;
	while (auto *o = it->getNextObject()) {
		auto *d = OSDynamicCast(IOPCIDevice, o);
		if (!d) continue;
		if (IOService *cl = d->getClient())   // e.g. a USBToolBox instance on the TBT DMA that never matched
			if (cl->getState() & (kIOServiceMatchedState | kIOServiceRegisteredState)) continue;
		const uint32_t id = d->configRead32(0);
		if ((id & 0xffff) != 0x8086) continue;
		bool want = false;
		for (uint16_t w : ids) if (w == (id >> 16)) want = true;
		uint8_t pm = 0;
		if (!want || !d->findPCICapability(kIOPCIPowerManagementCapability, &pm) || !pm) continue;
		const uint16_t cs = d->configRead16(pm + 4);
		if ((cs & 3) != 3) { d->configWrite16(pm + 4, (uint16_t)(cs | 3)); n++; }
	}
	it->release();
	return n;
}

// ─── S0ixSleep (separate kext) owning the system sleep: boot-arg -wex-s0ixsleep ─────────────────────────────────
// XE-PM then leaves acpi_sleep_kernel to S0ixSleep, and the display code follows S0ixSleep's sleep state through the
// structure it publishes on IOResources ("S0ixSleep": OSData with its address). The pointer is looked up from thread
// context (xpApply at 20/60 s, s2stat) and only read elsewhere. From version 2 the structure also lets a driver add a
// hook that S0ixSleep calls inside its wait (phase 0 entering, 1 leaving, interrupts off, one CPU): the GT reset that
// XE-PM's own wait does at the end runs there.
typedef void (*XpS0ixHook)(void *context, uint32_t phase, uint32_t reason);
struct XpS0ixShared {
	uint32_t version; volatile uint32_t enabled, sleeping, sleeps, reason;
	uint32_t hookSlots;                                // version >= 2
	bool (*addHook)(XpS0ixHook hook, void *context);   // version >= 2, thread context
};
static void xpS0ixRegisterHooks(const XpS0ixShared *sh);
static bool gXpS0ixMode = false;
static const XpS0ixShared *gXpS0ix = nullptr;
static void xpFindS0ix() {
	if (!gXpS0ixMode || __atomic_load_n(&gXpS0ix, __ATOMIC_ACQUIRE)) return;
	IOService *res = IOService::getResourceService();
	OSData *d = res ? OSDynamicCast(OSData, res->getProperty("S0ixSleep")) : nullptr;
	if (!d || d->getLength() != 8) return;
	uint64_t a = 0;
	memcpy(&a, d->getBytesNoCopy(), sizeof(a));
	auto *sh = reinterpret_cast<const XpS0ixShared *>(a);
	if (!sh || sh->version < 1 || sh->version > 99) return;
	__atomic_store_n(&gXpS0ix, sh, __ATOMIC_RELEASE);
	if (sh->version >= 2) xpS0ixRegisterHooks(sh);
}
static inline const XpS0ixShared *xpS0ix() { return __atomic_load_n(&gXpS0ix, __ATOMIC_ACQUIRE); }
static void xpApply(thread_call_param_t, thread_call_param_t) {
	xpFindS0ix();
	gXpCst = xpCstFix();
	gXpD3 += xpD3();
	if (!gXpEet) { mp_rendezvous_no_intrs(xpEetOne, nullptr); gXpEet = (int)((xpRdmsr(0x1FC) >> 19) & 1); }
	if (gXpOrgSetMult) gXpHwp = 1;   // takes effect at XCPM's next request on each CPU
	uint64_t st[] {1,(uint64_t)gXpHwp,(uint64_t)gXpRoute,(uint64_t)(int64_t)gXpEpp,(uint64_t)gXpEet,(uint64_t)gXpCst,(uint64_t)gXpD3};
	IORegistryEntry::getRegistryRoot()->setProperty("WEXPowerProfile",st,sizeof(st));
	IOLog("WEX PM: profile on: HWP rescale %d (route %d), EPP %d, EE turbo off %d, C8/C10 %d, D3 %d\n",
	      gXpHwp, gXpRoute, gXpEpp, gXpEet, gXpCst, gXpD3);
}

SYSCTL_NODE(_debug, OID_AUTO, xepm, CTLFLAG_RW, 0, "WhateverXe XE-PM");
SYSCTL_INT(_debug_xepm, OID_AUTO, hwp, CTLFLAG_RW, &gXpHwp, 0, "HWP rescale on hybrid P-cores");
SYSCTL_INT(_debug_xepm, OID_AUTO, epp, CTLFLAG_RW, &gXpEpp, 0, "EPP override, -1 = XCPM's");
SYSCTL_INT(_debug_xepm, OID_AUTO, desired, CTLFLAG_RW, &gXpDes, 0, "HWP desired perf override, 0 = autonomous, -1 = XCPM's");
SYSCTL_INT(_debug_xepm, OID_AUTO, min, CTLFLAG_RW, &gXpMin, 0, "HWP minimum perf override, -1 = XCPM's");
SYSCTL_QUAD(_debug_xepm, OID_AUTO, last_req, CTLFLAG_RD, &gXpLastReq, "last IA32_HWP_REQUEST written");
SYSCTL_INT(_debug_xepm, OID_AUTO, eet_off, CTLFLAG_RD, &gXpEet, 0, "");
SYSCTL_INT(_debug_xepm, OID_AUTO, cst, CTLFLAG_RD, &gXpCst, 0, "1 = C8/C10 on");
SYSCTL_INT(_debug_xepm, OID_AUTO, d3, CTLFLAG_RD, &gXpD3, 0, "");
SYSCTL_INT(_debug_xepm, OID_AUTO, route, CTLFLAG_RD, &gXpRoute, 0, "1 = HWP route installed");

// ─── 6: s2idle-style system sleep (experimental, off by default) ───────────────────────────────────────────────
// This firmware has no working S3 (it is Modern Standby only, FACP LowPowerS0Idle): entering S3 resets the machine.
// With sysctl debug.xepm.s2idle=1, acpi_sleep_kernel keeps its whole path (drivers already powered down, other CPUs
// off, _PTS run by the platform) but its final callback, which would write SLP_EN, is replaced: the boot CPU polls
// the ACPI event registers and returns when a new GPE0 status bit (lid, power button, EC, ...) or the RTC alarm
// appears. XNU then runs its normal resume path ("returns if sleep failed"). Ports from FACP: PM1a_EVT 0x1800,
// GPE0_BLK 0x1860 (32 bytes: 16 status + 16 enable).
typedef void (*xpAcpiSleepCallback)(void *);
static mach_vm_address_t gXpOrgAcpiSleep = 0;
static int gXpSleepMode = 0, gXpSleepRoute = 0;
static uint32_t gXpSleepCount = 0, gXpSleepReason = 0;
static uint64_t gXpSleepLoops = 0;
static inline uint8_t xpInb(uint16_t port) { uint8_t v; asm volatile("inb %1, %0" : "=a"(v) : "Nd"(port)); return v; }
static inline uint16_t xpInw(uint16_t port) { uint16_t v; asm volatile("inw %1, %0" : "=a"(v) : "Nd"(port)); return v; }
static inline void xpOutw(uint16_t port, uint16_t v) { asm volatile("outw %0, %1" : : "a"(v), "Nd"(port)); }
static inline void xpOutb(uint16_t port, uint8_t v) { asm volatile("outb %0, %1" : : "a"(v), "Nd"(port)); }
// The EC's SCI GPE (H_EC._GPE) ends the wait for every EC event: battery, AC, thermal, fan, USB-C,
// Samsung SCAI, as well as the key that should wake us. When it rises with no keyboard byte waiting
// in the 8042 (port 0x64 OBF), read the EC event (ACPI EC QR_EC 0x84 on 0x66/0x62): the firmware's
// non-wake events (_Q51-_Q54 AC/battery, _Q57/_Q58/_Q5B, _Q60/_Q61/_Q65/_Q66 battery, _Q72-_Q76
// thermal/fan/UCSI, _Q7C SCAI) are consumed and the wait goes on; power button (_Q50), lid (_Q5E/_Q5F),
// no pending event or anything else still wakes. -1: EC GPE unknown, no filtering.
static int gXpEcGpe = -1;
static uint32_t gXpEcIgnored = 0, gXpEcLast[4] = {0x1FF, 0x1FF, 0x1FF, 0x1FF};
static bool xpEcWait(bool obf) {
	for (unsigned i = 0; i < 20000; i++) {
		const uint8_t st = xpInb(0x66);
		if (obf ? (st & 1) : !(st & 2)) return true;
		for (unsigned k = 0; k < 50; k++) __builtin_ia32_pause();
	}
	return false;
}
static uint32_t xpEcQuery() {
	if (!(xpInb(0x66) & 0x20)) return 0x100;   // SCI_EVT clear: no event pending
	if (!xpEcWait(false)) return 0x101;
	xpOutb(0x66, 0x84);
	if (!xpEcWait(true)) return 0x102;
	return xpInb(0x62);
}
static bool xpEcIgnorable(uint32_t q) {
	switch (q) {
		case 0x51: case 0x52: case 0x53: case 0x54: case 0x57: case 0x58: case 0x5B:
		case 0x60: case 0x61: case 0x65: case 0x66: case 0x72: case 0x73: case 0x74: case 0x75: case 0x76: case 0x7C:
			return true;
	}
	return false;
}
static void xpFindEcGpe() {
	if (gXpEcGpe >= 0) return;
	if (OSIterator *it = IOService::getMatchingServices(IOService::nameMatching("PNP0C09"))) {
		while (auto *o = it->getNextObject()) {
			// No IOACPIFamily link dependency: string metaCast, then the class's virtual evaluateInteger.
			auto *dev = o->metaCast("IOACPIPlatformDevice") ? static_cast<IOACPIPlatformDevice *>(o) : nullptr;
			UInt32 gpe = 0;
			if (dev && dev->evaluateInteger("_GPE", &gpe) == kIOReturnSuccess && gpe < 128) { gXpEcGpe = (int)gpe; break; }
		}
		it->release();
	}
}
static constexpr uint16_t xpPm1Sts = 0x1800, xpGpe0Sts = 0x1860;
// Wait inside the sleep callback. XNU has already turned the other CPUs off and shut this CPU's
// local APIC down (acpi_sleep_kernel: pmCPUExitHaltToOff, lapic_shutdown), so no interrupt can end
// an MWAIT here: the wake sources are polled. s2wait 1 (default): the core's HWP ceiling is set to
// its floor for the wait (the request is restored exactly; EPP untouched) and each poll slice is a
// timed TPAUSE (C0.2, WAITPKG) when the CPU has it; s2wait 0: the original PAUSE spin.
// s2safety: seconds after which the wait ends by itself (0 = never; the test script sets it).
// The package energy counter (RAPL) is read around the wait: s2stat reports the average power.
static int gXpSleepWait = 1, gXpSleepSafety = 0;
static uint64_t gXpTscPerMs = 0, gXpSleepMs = 0, gXpSleepPkgMw = 0, gXpSleepCoreMw = 0, gXpSleepHwp = 0;
static int gXpWaitPkg = -1;
// Measurement around the wait (read only): package C-state residency (PC2..PC10, per mille of the wait),
// graphics (PP1) and platform (PSYS) energy, the package C-state limit, GT RC6 residency and the display
// pipe/plane/DC state when the wait starts. The MSRs are probed with rdmsr_carefully (no #GP on a missing
// MSR) when s2idle is armed; the wait reads only those that answered.
static mach_vm_address_t gXpRdmsrCarefully = 0;
static constexpr uint32_t xpResMsr[7] = {0x60D, 0x3F8, 0x3F9, 0x3FA, 0x630, 0x631, 0x632};   // PC2 PC3 PC6 PC7 PC8 PC9 PC10
static uint32_t gXpMsrOk = 0;   // bits 0-6 xpResMsr, 8 PP1 0x641, 9 PSYS 0x64D, 10 PKG_CST_CONFIG 0xE2, 11/12 core C6/C7, 31 probed
static uint32_t gXpResPm[7];
static uint64_t gXpPp1Mw = 0, gXpPsysMw = 0, gXpPkgCst = 0;
static uint32_t gXpRc6 = 0, gXpGtCore[2], gXpDisp[5];   // RC6 counter delta; GT core status start/end; display regs
static void xpProbeMsrs() {
	if (!gXpRdmsrCarefully || (gXpMsrOk & 0x80000000U)) return;
	auto rd = reinterpret_cast<int (*)(uint32_t, uint32_t *, uint32_t *)>(gXpRdmsrCarefully);
	uint32_t lo = 0, hi = 0, ok = 0x80000000U;
	for (unsigned i = 0; i < 7; i++) if (rd(xpResMsr[i], &lo, &hi) == 0) ok |= 1U << i;
	if (rd(0x641, &lo, &hi) == 0) ok |= 1U << 8;
	if (rd(0x64D, &lo, &hi) == 0) ok |= 1U << 9;
	if (rd(0xE2, &lo, &hi) == 0) { ok |= 1U << 10; gXpPkgCst = ((uint64_t)hi << 32) | lo; }
	if (rd(0x3FD, &lo, &hi) == 0) ok |= 1U << 11;   // core C6 residency
	if (rd(0x3FE, &lo, &hi) == 0) ok |= 1U << 12;   // core C7 residency
	gXpMsrOk = ok;
}
static inline uint64_t xpRdtsc() { uint32_t lo, hi; asm volatile("rdtsc" : "=a"(lo), "=d"(hi)); return ((uint64_t)hi << 32) | lo; }
// TPAUSE ecx: bit 0 = 0 selects C0.2; ends at the EDX:EAX TSC deadline (or on an external event).
static inline void xpTpause(uint64_t deadline) {
	asm volatile(".byte 0x66, 0x0f, 0xae, 0xf1" : : "c"(0U), "a"((uint32_t)deadline), "d"((uint32_t)(deadline >> 32)) : "cc", "memory");
}
static void xpCalibrate() {
	xpFindEcGpe();
	xpProbeMsrs();
	if (gXpWaitPkg < 0) {
		uint32_t a = 7, b, c = 0, d;
		asm volatile("cpuid" : "+a"(a), "=b"(b), "+c"(c), "=d"(d));
		gXpWaitPkg = (c >> 5) & 1;
	}
	if (gXpTscPerMs) return;
	const uint64_t n0 = mach_absolute_time(), t0 = xpRdtsc();
	IODelay(20000);
	const uint64_t n1 = mach_absolute_time(), t1 = xpRdtsc();
	uint64_t ns = 0; absolutetime_to_nanoseconds(n1 - n0, &ns);
	if (ns) gXpTscPerMs = (t1 - t0) * 1000000ULL / ns;
}
static constexpr unsigned xpGpe0Len = 16;
static uint8_t gXpGpeEn[16];
// The render engine (GT) keeps its state through an s2idle sleep, but the
// accelerator's sleep/wake path expects the cold GT of a real S3 resume. Before
// returning, reset the GT domains like a power cycle would (GEN6_GDRST 0x941C,
// GRDOM_FULL; not in the GT power well, no forcewake needed - Linux
// gen6_hw_domain_reset). The display engine is not part of this reset.
static volatile uint8_t *gXpGpuMmio = nullptr;
static uint32_t gXpGdrst = 0;
static void xpGtReset() {
	if (!gXpGpuMmio) return;
	// Hold forcewake across the reset like Linux __intel_gt_reset (render, GT, media domains: request/ack).
	static constexpr uint32_t fw[4][2] = {{0xA278, 0x0D84}, {0xA188, 0x130044}, {0xA540, 0x0D50}, {0xA560, 0x0D70}};
	auto reg = [](uint32_t r) { return reinterpret_cast<volatile uint32_t *>(gXpGpuMmio + r); };
	for (auto &f : fw) {
		*reg(f[0]) = 0x00010001U;
		for (unsigned spins = 0; !(*reg(f[1]) & 1U) && spins < 200; spins++) for (unsigned k = 0; k < 2000; k++) __builtin_ia32_pause();
	}
	auto gdrst = reinterpret_cast<volatile uint32_t *>(gXpGpuMmio + 0x941C);
	*gdrst = 1U;   // GRDOM_FULL
	uint32_t spins = 0;
	while ((*gdrst & 1U) && spins < 2000) { for (unsigned k = 0; k < 2000; k++) __builtin_ia32_pause(); spins++; }
	gXpGdrst = (*gdrst & 1U) ? 0x80000000U | spins : spins + 1;
	for (auto &f : fw) *reg(f[0]) = 0x00010000U;
}
// S0ixSleep wait hook (its interrupts-off, single-CPU context): the GT reset on the way out.
static uint32_t gXpS0ixHookCalls[2] = {0, 0};
static int gXpS0ixHooked = 0;   // 1 added, -1 refused (no free slot)
static void xpS0ixHook(void *, uint32_t phase, uint32_t) {
	if (phase < 2) gXpS0ixHookCalls[phase]++;
	if (phase == 1) xpGtReset();
}
static void xpS0ixRegisterHooks(const XpS0ixShared *sh) {
	if (gXpS0ixHooked || !sh->addHook) return;
	gXpS0ixHooked = sh->addHook(xpS0ixHook, nullptr) ? 1 : -1;
}
void xePmSetGpuMmio(volatile uint8_t *mmio) { gXpGpuMmio = mmio; }
bool xePmS2idleActive() { if (const XpS0ixShared *s = xpS0ix()) return s->enabled != 0; return gXpSleepMode == 1 && gXpOrgAcpiSleep; }
// True from kIOMessageSystemWillSleep to kIOMessageSystemHasPoweredOn of a system sleep taken with s2idle armed:
// the display/GT changes for a power-kept sleep apply only inside that window, never at runtime.
static volatile bool gXpSystemSleeping = false;
bool xePmS2idleSleeping() { if (const XpS0ixShared *s = xpS0ix()) return __atomic_load_n(&s->sleeping, __ATOMIC_ACQUIRE) != 0; return __atomic_load_n(&gXpSystemSleeping, __ATOMIC_ACQUIRE); }
// An s2idle wake always comes back as a dark wake (no wake reason XNU recognises as user input);
// the first key or the power button is consumed by the wait (EC query) and macOS then waits for
// another HID event to light the display. After a wake caused by the user (PWRBTN, a non-ignored
// GPE), ask for full wake the way XNU's own dark-wake promotion does (requestUserActive), from a
// thread call shortly after the first capability message. Not for the safety timer or the RTC.
static volatile bool gXpUserWakePending = false;
static thread_call_t gXpUserWakeCall = nullptr;
static uint32_t gXpUserWakeCount = 0;
// IOPMrootDomain::requestUserActive(IOService *, const char *), solved at install from the kernel
// (no link-time dependency on a symbol the KPI sets may not export).
static mach_vm_address_t gXpRequestUserActive = 0;
static void xpUserWakeWork(thread_call_param_t, thread_call_param_t) {
	IOPMrootDomain *root = IOService::getPMRootDomain();
	if (root && gXpRequestUserActive) {
		reinterpret_cast<void (*)(IOPMrootDomain *, IOService *, const char *)>(gXpRequestUserActive)(root, root, "s2idle user wake");
		gXpUserWakeCount++;
	}
}
static IOReturn xpSleepInterest(void *, void *, UInt32 type, IOService *, void *, vm_size_t) {
	if ((type == kIOMessageSystemCapabilityChange || type == kIOMessageSystemWillPowerOn || type == kIOMessageSystemHasPoweredOn) &&
	    gXpUserWakeCall && __atomic_exchange_n(&gXpUserWakePending, false, __ATOMIC_ACQ_REL)) {
		// Late enough for the dark wake's framebuffer Doze transitions (FB0 held at Sleep while the first
		// native FB restores the saved display controller): a full wake requested at once sends every FB
		// from Sleep straight to Wake and that restore is skipped.
		uint64_t deadline; clock_interval_to_deadline(1000, kMillisecondScale, &deadline);
		thread_call_enter_delayed(gXpUserWakeCall, deadline);
	}
	if (type == kIOMessageSystemWillSleep && xePmS2idleActive()) __atomic_store_n(&gXpSystemSleeping, true, __ATOMIC_RELEASE);
	else if (type == kIOMessageSystemHasPoweredOn) __atomic_store_n(&gXpSystemSleeping, false, __ATOMIC_RELEASE);
	return kIOReturnSuccess;
}
uint32_t xePmS2idleSleeps() { if (const XpS0ixShared *s = xpS0ix()) return __atomic_load_n(&s->sleeps, __ATOMIC_ACQUIRE); return __atomic_load_n(&gXpSleepCount, __ATOMIC_ACQUIRE); }
// ─── MWAIT wait machinery (tested at runtime by debug.xepm.mwtest, used by the sleep wait with s2wait 2) ─────────
// The TPAUSE wait is C0, so the package stays in PC0 (~2.9 W). For a deep MWAIT the core needs a wake event: the
// LAPIC timer in TSC deadline mode, pointed at vector 0xF2 of a copy of the kernel's IDT (gate: EOI + iretq), with
// the task priority at 0xE0 so that no kernel vector (<= 0xDF) can be delivered. xAPIC (MMIO page) or x2APIC (MSRs).
extern "C" {
__attribute__((used)) volatile uint32_t gXpMwIrq = 0;
__attribute__((used)) volatile uint64_t gXpMwEoi = 0;   // xAPIC: virtual address of the EOI register; 0: x2APIC
void xpMwStub();
void xpMwSpur();
}
asm(R"(
	.section __TEXT,__text,regular,pure_instructions
	.p2align 4
	.private_extern _xpMwStub
_xpMwStub:
	.byte 0xf3, 0x0f, 0x1e, 0xfa
	pushq %rax
	pushq %rcx
	pushq %rdx
	lock incl _gXpMwIrq(%rip)
	movq _gXpMwEoi(%rip), %rax
	testq %rax, %rax
	jz 1f
	movl $0, (%rax)
	jmp 2f
1:
	movl $0x80B, %ecx
	xorl %eax, %eax
	xorl %edx, %edx
	wrmsr
2:
	popq %rdx
	popq %rcx
	popq %rax
	iretq
	.p2align 4
	.private_extern _xpMwSpur
_xpMwSpur:
	.byte 0xf3, 0x0f, 0x1e, 0xfa
	iretq
)");
static constexpr uint8_t xpMwVector = 0xF2, xpMwSpurious = 0xFF;
alignas(4096) static uint8_t gXpMwIdt[4096];
alignas(64) static volatile uint64_t gXpMwMonitor[8];
static IOMemoryMap *gXpMwApicMap = nullptr;
static volatile uint32_t *gXpMwApic = nullptr;   // xAPIC MMIO page; nullptr: x2APIC MSRs
static inline uint32_t xpApicRd(unsigned off) { return gXpMwApic ? gXpMwApic[off / 4] : (uint32_t)xpRdmsr(0x800 + off / 16); }
static inline void xpApicWr(unsigned off, uint32_t v) { if (gXpMwApic) gXpMwApic[off / 4] = v; else xpWrmsr(0x800 + off / 16, v); }
struct __attribute__((packed)) XpIdtr { uint16_t limit; uint64_t base; };
static void xpMwGate(uint8_t vec, uint64_t h, uint16_t cs) {
	uint8_t *g = gXpMwIdt + vec * 16;
	g[0] = (uint8_t)h; g[1] = (uint8_t)(h >> 8); g[2] = (uint8_t)cs; g[3] = (uint8_t)(cs >> 8);
	g[4] = 0; g[5] = 0x8E;   // IST 0; present, DPL 0, 64-bit interrupt gate
	g[6] = (uint8_t)(h >> 16); g[7] = (uint8_t)(h >> 24);
	for (unsigned i = 0; i < 4; i++) g[8 + i] = (uint8_t)(h >> (32 + 8 * i));
	g[12] = g[13] = g[14] = g[15] = 0;
}
// XNU sets the IDT limit to 0x1000 + the CPU number (sidt gives user code its CPU number): the 256 gates are the
// first 4096 bytes; callers keep the limit as it is.
static void xpMwBuildIdt(const XpIdtr &kidt) {
	memcpy(gXpMwIdt, reinterpret_cast<const void *>(kidt.base), sizeof(gXpMwIdt));
	uint16_t cs;
	asm volatile("mov %%cs, %0" : "=r"(cs));
	xpMwGate(xpMwVector, reinterpret_cast<uint64_t>(&xpMwStub), cs);
	xpMwGate(xpMwSpurious, reinterpret_cast<uint64_t>(&xpMwSpur), cs);
}
// Done at runtime (sysctl s2wait=2): the CPU features the wait needs (MWAIT, TSC deadline timer, an APIC timer that
// keeps running in deep C-states: ARAT) and the APIC page mapping, which cannot be made in the sleep callback.
static bool gXpMwReady = false;
static int xpMwPrepare() {
	uint32_t a = 1, b, c = 0, d;
	asm volatile("cpuid" : "+a"(a), "=b"(b), "+c"(c), "=d"(d));
	const bool hasMwait = (c >> 3) & 1, hasDl = (c >> 24) & 1;
	a = 6; c = 0;
	asm volatile("cpuid" : "+a"(a), "=b"(b), "+c"(c), "=d"(d));
	const bool arat = (a >> 2) & 1;
	if (!hasMwait || !hasDl || !arat) return ENOTSUP;
	const uint64_t apic = xpRdmsr(0x1B);
	if (!(apic & 0x800)) return ENXIO;
	const bool x2 = (apic & 0x400) != 0;
	if (!x2 && !gXpMwApicMap) {
		if (IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(apic & 0xFFFFFF000ULL, 0x1000, kIODirectionInOut)) {
			gXpMwApicMap = md->map(kIOMapInhibitCache);
			md->release();
		}
		if (!gXpMwApicMap) return ENOMEM;
	}
	gXpMwApic = x2 ? nullptr : reinterpret_cast<volatile uint32_t *>(gXpMwApicMap->getVirtualAddress());
	gXpMwEoi = x2 ? 0 : reinterpret_cast<uint64_t>(gXpMwApic) + 0xB0;
	gXpMwReady = true;
	return 0;
}
// Sleep wait (interrupts off, other CPUs off). acpi_sleep_kernel has run lapic_shutdown: timer/LINT0/error/perf LVTs
// masked, software disabled, possibly globally disabled (APIC_BASE enable clear). The APIC is enabled in the mode it
// was prepared in, the timer LVT pointed at 0xF2, everything else left as found; xpMwLeave puts back the exact state.
struct XpMwSave { XpIdtr kidt; uint64_t apicBase, dl; uint32_t svr, lvt, tpr; };
static uint64_t gXpMwFoundBase = 0;
static uint32_t gXpMwFoundSvr = 0, gXpMwFail = 0;
static bool xpMwEnter(XpMwSave &s) {
	s.apicBase = xpRdmsr(0x1B);
	gXpMwFoundBase = s.apicBase;
	const bool x2 = gXpMwApic == nullptr;
	if (s.apicBase & 0x800) {
		if (((s.apicBase & 0x400) != 0) != x2) { gXpMwFail = 1; return false; }
	} else {
		xpWrmsr(0x1B, (s.apicBase | 0x800) & ~0x400ULL);   // disabled -> xAPIC (-> x2APIC)
		if (x2) xpWrmsr(0x1B, s.apicBase | 0xC00);
	}
	asm volatile("sidt %0" : "=m"(s.kidt));
	if ((size_t)s.kidt.limit + 1 < sizeof(gXpMwIdt)) {
		if (xpRdmsr(0x1B) != s.apicBase) xpWrmsr(0x1B, s.apicBase);
		gXpMwFail = 2;
		return false;
	}
	s.svr = xpApicRd(0xF0);
	s.lvt = xpApicRd(0x320);
	s.tpr = xpApicRd(0x80);
	s.dl = xpRdmsr(0x6E0);
	gXpMwFoundSvr = s.svr;
	xpMwBuildIdt(s.kidt);
	XpIdtr ours {s.kidt.limit, reinterpret_cast<uint64_t>(gXpMwIdt)};
	asm volatile("lidt %0" : : "m"(ours) : "memory");
	xpApicWr(0xF0, (s.svr & ~0xFFU) | 0x100U | xpMwSpurious);   // software enabled, spurious vector 0xFF
	xpApicWr(0x80, 0xE0);
	xpApicWr(0x320, (2U << 17) | xpMwVector);   // TSC deadline mode, unmasked
	gXpMwFail = 0;
	return true;
}
static void xpMwLeave(const XpMwSave &s) {
	xpWrmsr(0x6E0, 0);
	asm volatile("sti\n\tnop\n\tnop\n\tcli" : : : "memory");   // a deadline that fired after the last cli lands here
	xpApicWr(0x320, s.lvt);
	xpApicWr(0x80, s.tpr);
	xpApicWr(0xF0, s.svr);
	asm volatile("lidt %0" : : "m"(s.kidt) : "memory");
	if (s.dl) xpWrmsr(0x6E0, s.dl);
	if (xpRdmsr(0x1B) != s.apicBase) xpWrmsr(0x1B, s.apicBase);
}
static int gXpSleepPeriodMs = 50;
static uint32_t gXpMwSleepIrq = 0, gXpMwUsed = 0;
static uint64_t gXpMwSleepC6 = 0, gXpMwSleepC7 = 0;
static void xpFakeSleep(void *) {
	uint8_t snap[xpGpe0Len];
	// Only GPEs the OS armed for wake (GPE0_EN, 0x1870) may end the wait; others (EC chatter) are ignored.
	for (unsigned i = 0; i < xpGpe0Len; i++) { snap[i] = xpInb(xpGpe0Sts + i); gXpGpeEn[i] = xpInb(xpGpe0Sts + 16 + i); }
	xpOutw(xpPm1Sts, (1U << 8) | (1U << 10));   // clear PWRBTN_STS and RTC_STS (write 1 to clear)
	uint32_t reason = 0; uint64_t loops = 0;
	const int wait = gXpSleepWait;
	const uint64_t tpms = gXpTscPerMs, t0 = xpRdtsc();
	const uint64_t safety = gXpSleepSafety > 0 && tpms ? (uint64_t)gXpSleepSafety * 1000ULL * tpms : 0;
	const uint64_t e0 = xpRdmsr(0x611) & 0xFFFFFFFFULL, c0 = xpRdmsr(0x639) & 0xFFFFFFFFULL;
	const uint32_t mok = gXpMsrOk;
	uint64_t r0[7] = {}, g0 = 0, p0 = 0;
	for (unsigned i = 0; i < 7; i++) if (mok & (1U << i)) r0[i] = xpRdmsr(xpResMsr[i]);
	if (mok & (1U << 8)) g0 = xpRdmsr(0x641) & 0xFFFFFFFFULL;
	if (mok & (1U << 9)) p0 = xpRdmsr(0x64D) & 0xFFFFFFFFULL;
	uint32_t rc0 = 0;
	if (gXpGpuMmio) {
		auto mm = [](uint32_t r) { return *reinterpret_cast<volatile uint32_t *>(gXpGpuMmio + r); };
		rc0 = mm(0x138108);          // GEN6_GT_GFX_RC6 residency
		gXpGtCore[0] = mm(0x138060); // GEN6_GT_CORE_STATUS
		gXpDisp[0] = mm(0x70008);    // TRANSCONF A
		gXpDisp[1] = mm(0x70180);    // PLANE_CTL_1_A
		gXpDisp[2] = mm(0x45504);    // DC_STATE_EN
		gXpDisp[3] = mm(0x45404);    // PWR_WELL_CTL2 (driver requests)
		gXpDisp[4] = mm(0x70030);    // PIPE_MISC A
	}
	uint64_t hwp = 0; bool hwpSet = false;
	if (wait >= 1 && (xpRdmsr(0x770) & 1)) {
		hwp = xpRdmsr(0x774);
		if (!(hwp & (1ULL << 42))) {   // package control: leave it alone
			uint64_t floor = hwp & 0xFF;
			if (!floor) floor = (xpRdmsr(0x771) >> 24) & 0xFF;   // IA32_HWP_CAPABILITIES lowest performance
			if (floor) { xpWrmsr(0x774, (hwp & ~0xFFFFFFULL) | floor | (floor << 8)); hwpSet = true; }
		}
	}
	gXpSleepHwp = hwpSet ? hwp : 0;
	// s2wait 2: between polls the core sleeps in MWAIT C10 until the LAPIC timer (s2period ms) or another break event.
	XpMwSave mws {};
	const bool mw = wait == 2 && gXpMwReady && tpms && xpMwEnter(mws);
	const uint64_t mwPeriod = tpms * (uint64_t)gXpSleepPeriodMs;
	const uint32_t mwIrq0 = gXpMwIrq;
	const uint64_t mc6 = mw && (mok & (1U << 11)) ? xpRdmsr(0x3FD) : 0, mc7 = mw && (mok & (1U << 12)) ? xpRdmsr(0x3FE) : 0;
	while (!reason) {
		const uint16_t pm1 = xpInw(xpPm1Sts);
		if (pm1 & (1U << 8)) reason = 1;
		else if (pm1 & (1U << 10)) reason = 2;
		else for (unsigned i = 0; i < xpGpe0Len; i++) {
			const uint8_t g = xpInb(xpGpe0Sts + i), rise = g & ~snap[i] & gXpGpeEn[i];
			if (rise) {
				const unsigned gpe = i * 8 + __builtin_ctz(rise);
				if ((int)gpe == gXpEcGpe && rise == (1U << (gpe & 7)) && !(xpInb(0x64) & 1)) {
					const uint32_t q = xpEcQuery();
					gXpEcLast[3] = gXpEcLast[2]; gXpEcLast[2] = gXpEcLast[1]; gXpEcLast[1] = gXpEcLast[0]; gXpEcLast[0] = q;
					if (xpEcIgnorable(q)) {
						gXpEcIgnored++;
						xpOutb(xpGpe0Sts + i, (uint8_t)rise);   // write-1-to-clear this GPE only
						snap[i] = g & (uint8_t)~rise;
						continue;
					}
				}
				reason = 0x100 + gpe; break;
			}
			snap[i] = g;
		}
		if (mw) {
			if (reason) break;
			xpWrmsr(0x6E0, xpRdtsc() + mwPeriod);
			asm volatile(".byte 0x0f, 0x01, 0xc8" : : "a"(gXpMwMonitor), "c"(0U), "d"(0U) : "memory");   // MONITOR
			asm volatile("sti\n\t.byte 0x0f, 0x01, 0xc9\n\tcli" : : "a"(0x60U), "c"(1U) : "memory");   // MWAIT C10
		} else if (wait >= 1 && gXpWaitPkg == 1 && tpms) xpTpause(xpRdtsc() + tpms);   // ~1 ms
		else for (unsigned k = 0; k < 2000; k++) __builtin_ia32_pause();
		++loops;
		if (!reason && safety && xpRdtsc() - t0 >= safety) reason = 3;
	}
	if (mw) {
		gXpMwSleepIrq = gXpMwIrq - mwIrq0;
		gXpMwSleepC6 = (mok & (1U << 11)) && tpms ? (xpRdmsr(0x3FD) - mc6) / tpms : 0;
		gXpMwSleepC7 = (mok & (1U << 12)) && tpms ? (xpRdmsr(0x3FE) - mc7) / tpms : 0;
		xpMwLeave(mws);
	}
	gXpMwUsed = mw ? 1 : (wait == 2 ? 2 : 0);   // 2: asked for, not entered (see mwfail)
	if (hwpSet) xpWrmsr(0x774, hwp);
	{
		const uint64_t t1 = xpRdtsc(), e1 = xpRdmsr(0x611) & 0xFFFFFFFFULL, c1 = xpRdmsr(0x639) & 0xFFFFFFFFULL;
		const uint32_t esu = (uint32_t)(xpRdmsr(0x606) >> 8) & 0x1F;
		const uint64_t ms = tpms ? (t1 - t0) / tpms : 0;
		gXpSleepMs = ms;
		// mW = (counts / 2^esu J) / (ms / 1000 s) * 1000
		gXpSleepPkgMw = ms ? (((e1 - e0) & 0xFFFFFFFFULL) * 1000000ULL >> esu) / ms : 0;
		gXpSleepCoreMw = ms ? (((c1 - c0) & 0xFFFFFFFFULL) * 1000000ULL >> esu) / ms : 0;
		const uint64_t dt = t1 - t0;
		for (unsigned i = 0; i < 7; i++) gXpResPm[i] = (mok & (1U << i)) && dt ? (uint32_t)((xpRdmsr(xpResMsr[i]) - r0[i]) * 1000ULL / dt) : 0;
		gXpPp1Mw = (mok & (1U << 8)) && ms ? ((((xpRdmsr(0x641) & 0xFFFFFFFFULL) - g0) & 0xFFFFFFFFULL) * 1000000ULL >> esu) / ms : 0;
		gXpPsysMw = (mok & (1U << 9)) && ms ? ((((xpRdmsr(0x64D) & 0xFFFFFFFFULL) - p0) & 0xFFFFFFFFULL) * 1000000ULL >> esu) / ms : 0;
		if (gXpGpuMmio) {
			gXpRc6 = *reinterpret_cast<volatile uint32_t *>(gXpGpuMmio + 0x138108) - rc0;
			gXpGtCore[1] = *reinterpret_cast<volatile uint32_t *>(gXpGpuMmio + 0x138060);
		}
	}
	if (reason == 1) xpOutw(xpPm1Sts, 1U << 8);
	xpGtReset();
	gXpSleepReason = reason; gXpSleepLoops = loops; gXpSleepCount++;
	if (reason != 2 && reason != 3) __atomic_store_n(&gXpUserWakePending, true, __ATOMIC_RELEASE);
}
static void xpAcpiSleepKernel(xpAcpiSleepCallback func, void *refcon) {
	if (gXpSleepMode == 1) func = xpFakeSleep;
	FunctionCast(xpAcpiSleepKernel, gXpOrgAcpiSleep)(func, refcon);
}
static int xpSleepHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	int value = gXpSleepMode;
	const int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || !req->newptr) return error;
	if (value < 0 || value > 1) return EINVAL;
	if (!gXpOrgAcpiSleep) return ENODEV;
	if (value == 1) xpCalibrate();
	gXpSleepMode = value;
	return 0;
}
static int xpSleepWaitHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	int value = gXpSleepWait;
	const int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || !req->newptr) return error;
	if (value < 0 || value > 2) return EINVAL;
	if (value == 2) {
		xpCalibrate();
		if (const int e = xpMwPrepare()) return e;
	}
	gXpSleepWait = value;
	return 0;
}
static int xpSleepPeriodHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	int value = gXpSleepPeriodMs;
	const int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || !req->newptr) return error;
	if (value < 5 || value > 1000) return EINVAL;
	gXpSleepPeriodMs = value;
	return 0;
}
static int xpSleepSafetyHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	int value = gXpSleepSafety;
	const int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || !req->newptr) return error;
	if (value < 0 || value > 86400) return EINVAL;
	gXpSleepSafety = value;
	return 0;
}
static char gXpSleepText[960];
static int xpSleepStatHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	xpFindS0ix();
	char en[40]; size_t u = 0;
	for (unsigned i = 0; i < 16 && u + 3 < sizeof(en); i++) u += snprintf(en + u, sizeof(en) - u, "%02x", gXpGpeEn[i]);
	snprintf(gXpSleepText, sizeof(gXpSleepText), "route=%d mode=%d sleeps=%u reason=0x%x loops=%llu gdrst=0x%x gpe_en=%s"
		" wait=%d waitpkg=%d safety=%d tsc_ms=%llu ms=%llu pkg_mw=%llu core_mw=%llu hwp=0x%llx"
		" ecgpe=%d ec_ign=%u ec_q=%x,%x,%x,%x userwake=%u"
		" pc=2:%u,3:%u,6:%u,7:%u,8:%u,9:%u,10:%u pp1_mw=%llu psys_mw=%llu pkgcst=0x%llx msrok=0x%x"
		" rc6=%u gtcore=0x%x/0x%x trans=0x%x plane=0x%x dc=0x%x pwell=0x%x pipemisc=0x%x"
		" mw=%u period_ms=%d mwirq=%u mw_c6_ms=%llu mw_c7_ms=%llu apic_found=0x%llx svr_found=0x%x mwfail=%u s0ix=%d/%d"
		" s0ixhook=%d/%u/%u",
		gXpSleepRoute, gXpSleepMode, gXpSleepCount, gXpSleepReason, gXpSleepLoops, gXpGdrst, en,
		gXpSleepWait, gXpWaitPkg, gXpSleepSafety, gXpTscPerMs, gXpSleepMs, gXpSleepPkgMw, gXpSleepCoreMw, gXpSleepHwp,
		gXpEcGpe, gXpEcIgnored, gXpEcLast[0], gXpEcLast[1], gXpEcLast[2], gXpEcLast[3], gXpUserWakeCount,
		gXpResPm[0], gXpResPm[1], gXpResPm[2], gXpResPm[3], gXpResPm[4], gXpResPm[5], gXpResPm[6],
		gXpPp1Mw, gXpPsysMw, gXpPkgCst, gXpMsrOk,
		gXpRc6, gXpGtCore[0], gXpGtCore[1], gXpDisp[0], gXpDisp[1], gXpDisp[2], gXpDisp[3], gXpDisp[4],
		gXpMwUsed, gXpSleepPeriodMs, gXpMwSleepIrq, gXpMwSleepC6, gXpMwSleepC7, gXpMwFoundBase, gXpMwFoundSvr, gXpMwFail,
		gXpS0ixMode ? 1 : 0, xpS0ix() ? 1 : 0, gXpS0ixHooked, gXpS0ixHookCalls[0], gXpS0ixHookCalls[1]);
	return sysctl_handle_string(oidp, gXpSleepText, sizeof(gXpSleepText), req);
}
SYSCTL_PROC(_debug_xepm, OID_AUTO, s2idle, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, xpSleepHandler, "I", "1 = sleep without S3");
SYSCTL_PROC(_debug_xepm, OID_AUTO, s2stat, CTLTYPE_STRING | CTLFLAG_RD, nullptr, 0, xpSleepStatHandler, "A", "s2idle status");
SYSCTL_PROC(_debug_xepm, OID_AUTO, s2wait, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, xpSleepWaitHandler, "I", "s2idle wait: 0 spin, 1 TPAUSE, 2 MWAIT + APIC timer");
SYSCTL_PROC(_debug_xepm, OID_AUTO, s2period, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, xpSleepPeriodHandler, "I", "s2wait 2: APIC timer period (ms)");
SYSCTL_PROC(_debug_xepm, OID_AUTO, s2safety, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, xpSleepSafetyHandler, "I", "s2idle wake after N s (0 never)");

// ─── 7: MWAIT wait test (runtime, opt-in) ──────────────────────────────────────────────────────────────────────
// The s2idle wait polls in TPAUSE C0.2, which is C0: with one core in C0 the package never leaves PC0 (measured:
// PC2..PC10 all 0, ~2.9 W). A deep MWAIT needs a wake event, and in the sleep callback the local APIC is off.
// This test checks, while the system runs, the mechanism the wait would use: on the current CPU, interrupts off,
// a copy of the kernel's IDT with one extra gate (vector 0xF2, EOI + iretq) is loaded, the task priority is raised
// so that only vectors >= 0xF0 are delivered, the LAPIC timer (TSC deadline mode, as XNU programs it) is pointed
// at 0xF2, and N times: deadline = now + period, then sti; MWAIT C10; cli. Everything is put back as it was
// (IDT, TPR, LVT timer, XNU's own deadline). The LAPIC is reached through its MMIO page in xAPIC mode (XNU's mode
// here, APIC_BASE 0xFEE00800) or through MSRs in x2APIC mode. sysctl debug.xepm.mwtest=N (1..50) runs it,
// debug.xepm.mwperiod sets the period in us (N * period <= 100 ms), debug.xepm.mwstat has the result.
static int gXpMwPeriodUs = 2000, gXpMwLast = 0;
static char gXpMwText[440];
static void xpMwTest(int iters) {
	int code = 0;
	uint64_t apic = 0, dl = 0, t0 = 0, t1 = 0, c6 = 0, c7 = 0, minUs = ~0ULL, maxUs = 0;
	uint32_t irq = 0, lvt = 0, tpr = 0, apicId = 0, irr = 0, svr = 0, idtLimit = 0;
	const uint64_t tpms = gXpTscPerMs, period = tpms * (uint64_t)gXpMwPeriodUs / 1000ULL;
	uint32_t a = 1, b, c = 0, d;
	asm volatile("cpuid" : "+a"(a), "=b"(b), "+c"(c), "=d"(d));
	const bool hasMwait = (c >> 3) & 1, hasDl = (c >> 24) & 1;
	const uint32_t mok = gXpMsrOk;
	apic = xpRdmsr(0x1B);
	const bool x2 = (apic & 0xC00) == 0xC00;
	if (!hasMwait || !hasDl) code = -1;
	else if (!period) code = -2;
	else if (!(apic & 0x800)) code = -3;   // APIC globally disabled
	else if (!x2 && !gXpMwApicMap) {
		if (IOMemoryDescriptor *md = IOMemoryDescriptor::withPhysicalAddress(apic & 0xFFFFFF000ULL, 0x1000, kIODirectionInOut)) {
			gXpMwApicMap = md->map(kIOMapInhibitCache);
			md->release();
		}
		if (!gXpMwApicMap) code = -6;
	}
	if (!code) {
		gXpMwApic = x2 ? nullptr : reinterpret_cast<volatile uint32_t *>(gXpMwApicMap->getVirtualAddress());
		gXpMwEoi = x2 ? 0 : reinterpret_cast<uint64_t>(gXpMwApic) + 0xB0;
		const boolean_t ints = ml_set_interrupts_enabled(FALSE);
		XpIdtr kidt {};
		asm volatile("sidt %0" : "=m"(kidt));
		idtLimit = kidt.limit;
		if (xpRdmsr(0x1B) != apic) code = -7;
		// XNU sets the IDT limit to 0x1000 + the CPU number (sidt gives user code its CPU number): the 256 gates are
		// the first 4096 bytes, the limit is kept as it is.
		else if ((size_t)kidt.limit + 1 < sizeof(gXpMwIdt)) code = -4;
		else {
			lvt = xpApicRd(0x320);
			svr = xpApicRd(0xF0);
			if (((lvt >> 17) & 3) != 2) code = -5;   // XNU's timer not in TSC deadline mode: leave it alone
			else if (!(svr & 0x100)) code = -8;     // APIC software disabled
			else {
				apicId = x2 ? xpApicRd(0x20) : xpApicRd(0x20) >> 24;
				tpr = xpApicRd(0x80);
				dl = xpRdmsr(0x6E0);
				xpMwBuildIdt(kidt);
				XpIdtr ours {kidt.limit, reinterpret_cast<uint64_t>(gXpMwIdt)};
				asm volatile("lidt %0" : : "m"(ours) : "memory");
				xpApicWr(0x80, 0xE0);
				xpApicWr(0x320, (lvt & ~0x100FFU) | xpMwVector);   // unmask, our vector, mode kept
				const uint64_t c6a = (mok & (1U << 11)) ? xpRdmsr(0x3FD) : 0, c7a = (mok & (1U << 12)) ? xpRdmsr(0x3FE) : 0;
				const uint32_t irq0 = gXpMwIrq;
				t0 = xpRdtsc();
				for (int i = 0; i < iters; i++) {
					const uint64_t st = xpRdtsc();
					xpWrmsr(0x6E0, st + period);
					asm volatile(".byte 0x0f, 0x01, 0xc8" : : "a"(gXpMwMonitor), "c"(0U), "d"(0U) : "memory");   // MONITOR
					asm volatile("sti\n\t.byte 0x0f, 0x01, 0xc9\n\tcli" : : "a"(0x60U), "c"(1U) : "memory");   // MWAIT C10
					const uint64_t us = (xpRdtsc() - st) * 1000ULL / tpms;
					if (us < minUs) minUs = us;
					if (us > maxUs) maxUs = us;
				}
				t1 = xpRdtsc();
				irq = gXpMwIrq - irq0;
				c6 = (mok & (1U << 11)) ? xpRdmsr(0x3FD) - c6a : 0;
				c7 = (mok & (1U << 12)) ? xpRdmsr(0x3FE) - c7a : 0;
				xpWrmsr(0x6E0, 0);
				asm volatile("sti\n\tnop\n\tnop\n\tcli" : : : "memory");   // a deadline that fired after the last cli lands here
				irr = xpApicRd(0x270);   // IRR 0xE0..0xFF
				xpApicWr(0x320, lvt);
				xpApicWr(0x80, tpr);
				asm volatile("lidt %0" : : "m"(kidt) : "memory");
				if (dl) xpWrmsr(0x6E0, dl);   // XNU's deadline (fires at once if it already passed)
			}
		}
		ml_set_interrupts_enabled(ints);
	}
	gXpMwLast = code ? code : iters;
	const uint64_t tot = tpms && t1 > t0 ? (t1 - t0) * 1000ULL / tpms : 0;
	snprintf(gXpMwText, sizeof(gXpMwText),
		"code=%d iters=%d period_us=%d irq=%u total_us=%llu min_us=%llu max_us=%llu core_c6_us=%llu core_c7_us=%llu"
		" irr_e0=0x%x apic=0x%llx mode=%s apicid=%u lvt=0x%x svr=0x%x tpr=0x%x dl_was=%d mwait=%d tscdl=%d idt_limit=0x%x",
		code, iters, gXpMwPeriodUs, irq, tot, minUs == ~0ULL ? 0 : minUs, maxUs,
		tpms ? c6 * 1000ULL / tpms : 0, tpms ? c7 * 1000ULL / tpms : 0,
		irr, apic, x2 ? "x2apic" : "xapic", apicId, lvt, svr, tpr, dl ? 1 : 0, hasMwait, hasDl, idtLimit);
}
static int xpMwTestHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	int value = gXpMwLast;
	const int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || !req->newptr) return error;
	if (value < 1 || value > 50 || (int64_t)value * gXpMwPeriodUs > 100000) return EINVAL;
	xpMwTest(value);
	return 0;
}
static int xpMwPeriodHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	int value = gXpMwPeriodUs;
	const int error = sysctl_handle_int(oidp, &value, 0, req);
	if (error || !req->newptr) return error;
	if (value < 100 || value > 10000) return EINVAL;
	gXpMwPeriodUs = value;
	return 0;
}
static int xpMwStatHandler(struct sysctl_oid *oidp, void *, int, struct sysctl_req *req) {
	return sysctl_handle_string(oidp, gXpMwText, sizeof(gXpMwText), req);
}
SYSCTL_PROC(_debug_xepm, OID_AUTO, mwtest, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, xpMwTestHandler, "I", "run the MWAIT wait test N times");
SYSCTL_PROC(_debug_xepm, OID_AUTO, mwperiod, CTLTYPE_INT | CTLFLAG_RW, nullptr, 0, xpMwPeriodHandler, "I", "MWAIT test period (us)");
SYSCTL_PROC(_debug_xepm, OID_AUTO, mwstat, CTLTYPE_STRING | CTLFLAG_RD, nullptr, 0, xpMwStatHandler, "A", "MWAIT test result");

void xePmInstall(KernelPatcher &patcher) {
	if (!checkKernelArgument("-wex-pm")) return;
	gXpOn = true;
	gXpEpp = 0;
	// Autonomous HWP (desired 0): XCPM's desired hint kept busy P-cores near 1.5 GHz, so
	// WindowServer missed 120 Hz vblanks. sysctl debug.xepm.desired=-1 restores XCPM's value.
	gXpDes = 0;
	gXpCstPtr = patcher.solveSymbol(KernelPatcher::KernelID, "_xcpm_cstate_table");
	patcher.clearError();
	const mach_vm_address_t base = patcher.solveSymbol(KernelPatcher::KernelID, "_xcpm_enable_pstate_ctrs");
	patcher.clearError();
	if (base) {
		const mach_vm_address_t fn = base + 0x1190;
		static const uint8_t pro[] = {0x55, 0x48, 0x89, 0xE5, 0x53, 0x50, 0x89, 0xFB};   // push rbp; mov rbp,rsp; push rbx; push rax; mov ebx,edi
		static const uint8_t wr[] = {0x89, 0xF8, 0xC1, 0xEA, 0x18, 0xB9, 0x74, 0x07, 0x00, 0x00, 0x0F, 0x30};   // +0x168: ...; mov ecx,0x774; wrmsr
		auto *p = reinterpret_cast<const uint8_t *>(fn);
		bool ok = true;
		for (size_t i = 0; i < sizeof(pro); i++) if (p[i] != pro[i]) ok = false;
		for (size_t i = 0; i < sizeof(wr); i++) if (p[0x168 + i] != wr[i]) ok = false;
		if (ok) {
			gXpOrgSetMult = patcher.routeFunction(fn, reinterpret_cast<mach_vm_address_t>(xpSetMult), true);
			if (patcher.getError() != KernelPatcher::Error::NoError) { gXpOrgSetMult = 0; patcher.clearError(); }
		}
		gXpRoute = gXpOrgSetMult ? 1 : (ok ? 5 : 3);
	} else gXpRoute = 2;
	sysctl_register_oid(&sysctl__debug_xepm);
	sysctl_register_oid(&sysctl__debug_xepm_hwp);
	sysctl_register_oid(&sysctl__debug_xepm_epp);
	sysctl_register_oid(&sysctl__debug_xepm_desired);
	sysctl_register_oid(&sysctl__debug_xepm_min);
	sysctl_register_oid(&sysctl__debug_xepm_last_req);
	sysctl_register_oid(&sysctl__debug_xepm_eet_off);
	sysctl_register_oid(&sysctl__debug_xepm_cst);
	sysctl_register_oid(&sysctl__debug_xepm_d3);
	sysctl_register_oid(&sysctl__debug_xepm_route);
	{
		gXpS0ixMode = checkKernelArgument("-wex-s0ixsleep");
		const mach_vm_address_t fn = patcher.solveSymbol(KernelPatcher::KernelID, "_acpi_sleep_kernel");
		patcher.clearError();
		static const uint8_t pro[] = {0x55, 0x48, 0x89, 0xE5, 0x41, 0x57, 0x41, 0x56};   // push rbp; mov rbp,rsp; push r15; push r14
		bool ok = fn != 0 && !gXpS0ixMode;
		for (size_t i = 0; ok && i < sizeof(pro); i++) if (reinterpret_cast<const uint8_t *>(fn)[i] != pro[i]) ok = false;
		if (ok) {
			gXpOrgAcpiSleep = patcher.routeFunction(fn, reinterpret_cast<mach_vm_address_t>(xpAcpiSleepKernel), true);
			if (patcher.getError() != KernelPatcher::Error::NoError) { gXpOrgAcpiSleep = 0; patcher.clearError(); }
		}
		gXpSleepRoute = gXpOrgAcpiSleep ? 1 : (gXpS0ixMode ? 4 : (fn ? 3 : 2));   // 4: left to S0ixSleep
		sysctl_register_oid(&sysctl__debug_xepm_s2idle);
		if (gXpOrgAcpiSleep) {
			gXpRequestUserActive = patcher.solveSymbol(KernelPatcher::KernelID, "__ZN14IOPMrootDomain17requestUserActiveEP9IOServicePKc");
			patcher.clearError();
			gXpUserWakeCall = gXpRequestUserActive ? thread_call_allocate(xpUserWakeWork, nullptr) : nullptr;
			gXpRdmsrCarefully = patcher.solveSymbol(KernelPatcher::KernelID, "_rdmsr_carefully");
			patcher.clearError();
			registerPrioritySleepWakeInterest(xpSleepInterest, nullptr, nullptr);
		}
		sysctl_register_oid(&sysctl__debug_xepm_s2stat);
		sysctl_register_oid(&sysctl__debug_xepm_s2wait);
		sysctl_register_oid(&sysctl__debug_xepm_s2safety);
		sysctl_register_oid(&sysctl__debug_xepm_s2period);
		sysctl_register_oid(&sysctl__debug_xepm_mwtest);
		sysctl_register_oid(&sysctl__debug_xepm_mwperiod);
		sysctl_register_oid(&sysctl__debug_xepm_mwstat);
	}
	uint64_t dl;
	thread_call_t t20 = thread_call_allocate(xpApply, nullptr), t60 = thread_call_allocate(xpApply, nullptr);
	clock_interval_to_deadline(20, kSecondScale, &dl);
	if (t20) thread_call_enter_delayed(t20, dl);
	clock_interval_to_deadline(60, kSecondScale, &dl);
	if (t60) thread_call_enter_delayed(t60, dl);
	IOLog("WEX PM: -wex-pm, HWP route %d, idle table %s; profile in 20 s\n", gXpRoute, gXpCstPtr ? "found" : "missing");
}
