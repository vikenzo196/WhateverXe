#!/usr/bin/env python3
"""WindowServer YUV detach patch: guards in source, signature against the installed cache."""
from pathlib import Path
import mmap
import re

root = Path(__file__).resolve().parents[1]
src = (root / 'WhateverXe/kern_xe_ws_yuv.cpp').read_text()
start = (root / 'WhateverXe/kern_start.cpp').read_text()

# Opt-out is checked before any hook is requested.
init = src[src.index('void XeWSYuv::init()'):]
assert init.index('checkKernelArgument("-wex-ws-gva")') < init.index('onPatcherLoadForce')
assert 'XeWSYuv::init();' in start and '#include "kern_xe_ws_yuv.hpp"' in start

# The native validator runs first; only validated, untainted data is touched.
page = src[src.index('void patchedCsValidatePage'):src.index('boolean_t patchedCsValidateRange')]
assert page.index('orgCsValidatePage') < page.index('findPattern')
assert '*validatedP == 0' in page and '*taintedP != 0' in page
rng = src[src.index('boolean_t patchedCsValidateRange'):src.index('bool serializeHits')]
assert rng.index('orgCsValidateRange') < rng.index('findPattern')
assert '!res' in rng and '*tainted != 0' in rng

# Patching requires the main shared cache file and a known file offset.
apply = src[src.index('void applyIfSharedCache'):src.index('void patchedCsValidatePage')]
assert apply.index('matchSharedCachePath') < apply.index('match[2] = 0xEB')
assert apply.index('kSupportedDetachOffsets') < apply.index('match[2] = 0xEB')
assert apply.count('match[2] = 0xEB') == 1 and 'match[2] != 0x74' in apply

pattern = bytes(int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]{2})\b',
    re.search(r'kSupportedDetachFind\[\] \{(.*?)\};', src, re.S).group(1)))
offsets = [int(x, 16) for x in re.findall(r'0x([0-9A-Fa-f]+)',
    re.search(r'kSupportedDetachOffsets\[\] \{(.*?)\};', src, re.S).group(1))]
assert len(pattern) == 22 and pattern[2] == 0x74 and pattern[3] == 0x29 and offsets

cache = Path('/System/Volumes/Preboot/Cryptexes/OS/System/Library/dyld/dyld_shared_cache_x86_64h')
if cache.exists():
    with cache.open('rb') as f, mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ) as m:
        found = [x.start() for x in re.finditer(re.escape(pattern), m)]
        if found:
            assert len(found) == 1 and found[0] in offsets, [hex(x) for x in found]
            o = found[0]
            # je +0x29 lands on "xor eax,eax" (return false), and the pattern stays in one page.
            assert m[o + 4 + 0x29:o + 4 + 0x29 + 2] == b'\x31\xc0'
            assert o // 4096 == (o + len(pattern) - 1) // 4096
            print('ws_yuv: signature at 0x%x in installed cache' % o)
        else:
            print('ws_yuv: signature absent in installed cache (patch is a no-op here)')
