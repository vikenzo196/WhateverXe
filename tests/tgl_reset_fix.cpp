#include "../WhateverXe/kern_tgl_reset_fix.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>

namespace {
alignas(16) uint8_t mmio[0x40000];
alignas(16) uint8_t list[0x20], entries[TGLResetFix::EntryStride*8];
uint32_t r(uint32_t o) {return TGLResetFix::read(mmio,o);}
void setEntry(unsigned i,uint32_t off,bool masked) {
	memcpy(entries+i*TGLResetFix::EntryStride,&off,4);
	entries[i*TGLResetFix::EntryStride+8]=masked;
}
void setList(uint64_t count) {
	memset(list,0,sizeof(list));memcpy(list,&count,8);
	auto p=reinterpret_cast<uint8_t *>(entries);memcpy(list+0x10,&p,sizeof(p));
}
}
int main() {
	using namespace TGLResetFix;
	unsigned cases=0;
	// Bad lists are rejected.
	{Cache c {};assert(!loadLayout(c,nullptr));++cases;}
	{Cache c {};memset(entries,0,sizeof(entries));setEntry(0,0x2134,false);setList(1);assert(!loadLayout(c,list) && !c.count);++cases;}
	{Cache c {};setEntry(0,0x2080,false);setList(MaxRegs+1);assert(!loadLayout(c,list));++cases;}
	{Cache c {};setEntry(0,0x2081,false);setList(1);assert(!loadLayout(c,list));++cases;}
	// RCS-like list: HWS_PGA plain, GFX_MODE masked, one more plain.
	memset(entries,0,sizeof(entries));
	setEntry(0,0x2080,false);setEntry(1,0x229c,true);setEntry(2,0x20c0,false);setList(3);
	Cache c {};
	assert(loadLayout(c,list) && c.count==3 && !c.masked[0] && c.masked[1]);++cases;
	// Layout is copied once.
	setEntry(0,0x9999,false);assert(loadLayout(c,list) && c.off[0]==0x2080);setEntry(0,0x2080,false);++cases;
	// No values while the engine is unconfigured; no replay without values.
	memset(mmio,0,sizeof(mmio));
	assert(!refresh(c,mmio) && !c.valid);assert(!replay(c,mmio));++cases;
	// Configured engine: remember.
	write(mmio,0x2080,0x40004000);write(mmio,0x229c,0x8);write(mmio,0x20c0,0x1234);
	assert(refresh(c,mmio) && c.valid && c.val[0]==0x40004000 && c.val[1]==8 && c.val[2]==0x1234);++cases;
	// Healthy engine after reset: nothing written.
	write(mmio,0x20c0,0x5555);
	assert(!needsReplay(c,mmio) && !replay(c,mmio) && r(0x20c0)==0x5555);++cases;
	// Wiped engine: replay with masked semantics.
	memset(mmio,0,sizeof(mmio));
	assert(replay(c,mmio)==3 && r(0x2080)==0x40004000 && r(0x229c)==0xFFFF0008 && r(0x20c0)==0x1234 && c.replays==1);++cases;
	// After replay HWS_PGA is set: a second call does nothing.
	assert(!replay(c,mmio) && c.replays==1);++cases;
	// A refresh on a wiped engine keeps the old good values.
	memset(mmio,0,sizeof(mmio));assert(!refresh(c,mmio) && c.val[0]==0x40004000);++cases;
	printf("%u reset-fix cases passed: layout checks, remember, replay, masked restore, idempotence\n",cases);
}
