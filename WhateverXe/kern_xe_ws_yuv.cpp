// SPDX-License-Identifier: BSD-3-Clause
#include "kern_xe_ws_yuv.hpp"

#include <Headers/kern_api.hpp>
#include <Headers/kern_patcher.hpp>
#include <Headers/kern_user.hpp>
#include <Headers/kern_util.hpp>
#include <IOKit/IORegistryEntry.h>
#include <libkern/OSAtomic.h>
#include <libkern/c++/OSNumber.h>
#include <libkern/c++/OSSerialize.h>
#include <sys/param.h>
#include <string.h>

namespace {
// SkyLight::supported_detachable_format(WSCompositeDestination*, unsigned int, CGSize, CGSize, bool)
// After RGB formats (BGRA, l10r, RGhA) return early, non-RGB (YUV) formats execute:
//   84 DB                test bl, bl         ; isIntel
//   74 29                je   +0x29          ; if (!isIntel) return false -> Metal YCbCr path
//   44 89 F7             mov  edi, r14d
//   BE 41 52 47 42       mov  esi, 0x42475241 ('BGRA')
//   F2 0F 10 45 E8       movsd xmm0, [rbp-0x18]
//   F2 0F 10 4D E0       movsd xmm1, [rbp-0x20]
// Patching byte +2 (0x74 -> 0xEB) makes Intel Xe take the non-Intel Metal YCbCr path for YUV
// while keeping WSGVAInitialize, gGVASupportedFormats, and RGB layer detachment intact.
static const uint8_t kSupportedDetachFind[] {
    0x84, 0xDB, 0x74, 0x29,
    0x44, 0x89, 0xF7,
    0xBE, 0x41, 0x52, 0x47, 0x42,
    0xF2, 0x0F, 0x10, 0x45, 0xE8,
    0xF2, 0x0F, 0x10, 0x4D, 0xE0
};

// Known file offsets of the signature inside the main shared cache file. Pages
// that contain the bytes anywhere else are left untouched and only recorded.
//   25G342 (macOS 26.7.3): dyld_shared_cache_x86_64h + 0x9545EA3 (SkyLight __TEXT)
static const uint64_t kSupportedDetachOffsets[] {0x9545EA3};
static const char kSharedCacheMain[] = "/dyld_shared_cache_x86_64h";

using GetPathFn = int (*)(void *vnode, char *pathbuf, int *len);

GetPathFn getPath = nullptr;
mach_vm_address_t orgCsValidatePage = 0;
mach_vm_address_t orgCsValidateRange = 0;
volatile SInt32 hitCount = 0;
volatile SInt32 missCount = 0;
volatile uint64_t lastMissOffset = 0;

inline uint8_t *findPattern(const void *data, size_t size) {
    if (!data || size < sizeof(kSupportedDetachFind)) return nullptr;
    const auto *bytes = static_cast<const uint8_t *>(data);
    const size_t limit = size - sizeof(kSupportedDetachFind);
    for (size_t i = 0; i <= limit; ++i) {
        if (UNLIKELY(bytes[i] == 0x84 && bytes[i + 1] == 0xDB &&
                     bytes[i + 2] == 0x74 && bytes[i + 3] == 0x29 &&
                     !memcmp(bytes + i, kSupportedDetachFind, sizeof(kSupportedDetachFind)))) {
            return const_cast<uint8_t *>(bytes + i);
        }
    }
    return nullptr;
}

__attribute__((noinline)) void applyIfSharedCache(void *vp, uint8_t *match, uint64_t fileOffset) {
    if (!vp || !match || !getPath) return;
    char path[MAXPATHLEN];
    int length = static_cast<int>(sizeof(path));
    if (getPath(vp, path, &length) != 0) return;
    if (!UserPatcher::matchSharedCachePath(path)) return;
    const size_t pathLength = strnlen(path, sizeof(path));
    const size_t suffixLength = sizeof(kSharedCacheMain) - 1;
    bool known = pathLength >= suffixLength && !strcmp(path + pathLength - suffixLength, kSharedCacheMain);
    if (known) {
        known = false;
        for (auto offset : kSupportedDetachOffsets) known |= offset == fileOffset;
    }
    if (!known) {
        lastMissOffset = fileOffset;
        if (OSIncrementAtomic(&missCount) < 4)
            SYSLOG("wex", "SkyLight YUV detach signature at unknown offset 0x%llx, left unpatched", fileOffset);
        return;
    }
    if (match[2] != 0x74) return;
    match[2] = 0xEB;
    const SInt32 prev = OSIncrementAtomic(&hitCount);
    if (prev < 4) {
        SYSLOG("wex", "SkyLight supported_detachable_format YUV detach patched (hit=%d)",
               static_cast<int>(prev + 1));
    }
}

void patchedCsValidatePage(void *vp, void *pager, uint64_t pageOffset, const void *data,
                           int *validatedP, int *taintedP, int *nxP) {
    FunctionCast(patchedCsValidatePage, orgCsValidatePage)(
        vp, pager, pageOffset, data, validatedP, taintedP, nxP);
    if (!vp || !data || !validatedP || *validatedP == 0 || (taintedP && *taintedP != 0)) return;
    if (uint8_t *match = findPattern(data, PAGE_SIZE)) {
        applyIfSharedCache(vp, match, pageOffset + static_cast<uint64_t>(match - static_cast<const uint8_t *>(data)));
    }
}

boolean_t patchedCsValidateRange(void *vp, void *pager, uint64_t rangeOffset, const void *data,
                                 vm_size_t size, unsigned *tainted) {
    const boolean_t res = FunctionCast(patchedCsValidateRange, orgCsValidateRange)(
        vp, pager, rangeOffset, data, size, tainted);
    if (!res || (tainted && *tainted != 0) || !vp || !data || size < sizeof(kSupportedDetachFind) || size > (64U * PAGE_SIZE)) {
        return res;
    }
    if (uint8_t *match = findPattern(data, static_cast<size_t>(size))) {
        applyIfSharedCache(vp, match, rangeOffset + static_cast<uint64_t>(match - static_cast<const uint8_t *>(data)));
    }
    return res;
}

bool serializeHits(void *, void *, OSSerialize *serializer) {
    const uint64_t count = static_cast<uint32_t>(__atomic_load_n(&hitCount, __ATOMIC_RELAXED)) |
        (static_cast<uint64_t>(static_cast<uint32_t>(__atomic_load_n(&missCount, __ATOMIC_RELAXED))) << 32);
    auto num = OSNumber::withNumber(count, 64);
    if (!num) return false;
    const bool ok = num->serialize(serializer);
    num->release();
    return ok;
}

void onPatcherLoad(void *, KernelPatcher &patcher) {
    getPath = reinterpret_cast<GetPathFn>(patcher.solveSymbol(KernelPatcher::KernelID, "_vn_getpath"));
    patcher.clearError();
    bool pageRouted = false;
    bool rangeRouted = false;
    if (getPath) {
        KernelPatcher::RouteRequest pageRoute {"_cs_validate_page", patchedCsValidatePage, orgCsValidatePage};
        pageRouted = patcher.routeMultipleLong(KernelPatcher::KernelID, &pageRoute, 1);
        patcher.clearError();
        KernelPatcher::RouteRequest rangeRoute {"_cs_validate_range", patchedCsValidateRange, orgCsValidateRange};
        rangeRouted = patcher.routeMultipleLong(KernelPatcher::KernelID, &rangeRoute, 1);
        patcher.clearError();
    }
    const bool installed = pageRouted || rangeRouted;
    auto root = IORegistryEntry::getRegistryRoot();
    root->setProperty("WhateverXeWSYUVDetachPatched", installed);
    if (auto hits = OSSerializer::forTarget(nullptr, serializeHits)) {
        root->setProperty("WhateverXeWSYUVDetachHits", hits);
        hits->release();
    }
    if (auto miss = OSSerializer::forTarget(nullptr, [](void *, void *, OSSerialize *serializer) {
            auto num = OSNumber::withNumber(__atomic_load_n(&lastMissOffset, __ATOMIC_RELAXED), 64);
            if (!num) return false;
            const bool ok = num->serialize(serializer);
            num->release();
            return ok;
        })) {
        root->setProperty("WhateverXeWSYUVDetachMissOffset", miss);
        miss->release();
    }
    SYSLOG("wex", "SkyLight YUV detach cs_validate hook installed=%d (page=%d range=%d)",
           installed ? 1 : 0, pageRouted ? 1 : 0, rangeRouted ? 1 : 0);
}
} // namespace

void XeWSYuv::init() {
    if (checkKernelArgument("-wex-ws-gva")) {
        IORegistryEntry::getRegistryRoot()->setProperty("WhateverXeWSYUVDetachPatched", false);
        SYSLOG("wex", "WindowServer YUV detach patch disabled by -wex-ws-gva");
        return;
    }
    lilu.onPatcherLoadForce(onPatcherLoad, nullptr);
}
