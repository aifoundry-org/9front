#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

#define INITMAP	(ROUND((uintptr)end + BY2PG, PGLSZ(1)))

void
mmu0init(uintptr *l1)
{
	print("mmu0init l1 %p\n", l1);
#ifdef XXX
	uintptr va, pa, pe, attr;
	/* VDRAM */
	attr = PTEREAD | PTEWRITE | PTEEXEC;
	pe = -KZERO;
	for(pa = VDRAM - KZERO; pa < pe; pa += PGLSZ(PTLEVELS-1))
		l1[PTLX(pa, PTLEVELS-1)] = PA2PTE(pa) | PTEVALID | attr;

	/* DRAM - INITMAP */
	pe = INITMAP;
	for(pa = VDRAM - KZERO, va = VDRAM; pa < pe; pa += PGLSZ(1), va += PGLSZ(1))
		l1[PTL1X(va, 1)] = PA2PTE(pa) | PTEVALID | attr;

	/* VIRTIO */
	attr = PTEREAD | PTEWRITE | PTEDEVICE;
	pe = PHYSIOEND;
	for(pa = PHYSIO, va = VIRTIO; pa < pe; pa += PGLSZ(1), va += PGLSZ(1)){
		if(((pa|va) & PGLSZ(1)-1) != 0){
			l1[PTL1X(va, 1)] = PA2PTE((uintptr)l1) | PTEVALID;
			for(; pa < pe && ((va|pa) & PGLSZ(1)-1) != 0; pa += PGLSZ(0), va += PGLSZ(0)){
//				assert(l1[PTLX(va, 0)] == 0);
				l1[PTLX(va, 0)] = PA2PTE(pa) | PTEVALID | attr;
			}
			break;
		}
		l1[PTL1X(va, 1)] = PA2PTE(pa) | PTEVALID | attr;
	}

	if(PTLEVELS > 2)
	for(va = KSEG0; va != 0; va += PGLSZ(2))
		l1[PTLX(va, 2)] = PA2PTE((uintptr)&l1[L1TABLEX(va, 1)]) | PTEVALID;
#endif
}

uintptr uzero = 0x1000000000ULL;	/* until meminit */

/*
 * User space starts above all RAM, on a root table slot boundary
 * and never below 4GiB.  *uzero moves it; RAM above it is not used.
 */
static void
setuzero(void)
{
	Confmem *cm;
	uvlong top, min;
	char *p;

	min = 4ULL*GiB;
	if(min < PADDR(end))
		min = PADDR(end);
	top = min;
	for(cm = conf.mem; cm < &conf.mem[nelem(conf.mem)]; cm++)
		if(cm->limit > top)
			top = cm->limit;
	if(p = getconf("*uzero"))
		top = strtoull(p, 0, 0);
	if(top < min)
		top = min;
	uzero = ROUND(top, PGLSZ(PTLEVELS-1));
	if(uzero >= USTKTOP - USTKSIZE)
		panic("meminit: uzero %#p beyond user space", uzero);
	print("meminit: uzero %#p\n", uzero);
	for(cm = conf.mem; cm < &conf.mem[nelem(conf.mem)]; cm++){
		if(cm->limit > uzero){
			print("meminit: ram %#p-%#p above uzero not used\n",
				cm->base > uzero ? cm->base : uzero, cm->limit);
			cm->limit = uzero;
		}
		if(cm->limit < cm->base)
			cm->limit = cm->base;
		cm->npage = (cm->limit - cm->base)/BY2PG;
		if(cm->npage != 0)
			print("meminit: bank %#p-%#p\n", cm->base, cm->limit);
	}
}

/*
 * RAM is what the DTB's /memory nodes describe, or 2GiB from VDRAM
 * without them, less the reserved regions, anything at or above
 * *maxmem and everything below the end of the kernel's area.
 * Carve conf.mem from it in address order, as pc does.
 */
void
meminit(void)
{
	Confmem *cm;
	uvlong base, size, top;
	char *p;
	extern u64int *sv57, *sv48, *sv39, *pGiB;

	top = -1;
	if(p = getconf("*maxmem"))
		top = strtoull(p, 0, 0);
	if(memmapnext(-1, MemRAM) == -1)
		memmapadd(VDRAM, (top != -1 ? top : VDRAM + 2ULL*GiB) - VDRAM, MemRAM);
	if(top != -1)
		memmapadd(top, -top, MemReserved);
	/* firmware, Mach pages, kernel and the boot page tables after end */
	memmapadd(0, ROUND(PGROUND((uintptr)end - KTZERO) + (uintptr)(VDRAM+0x2fffff), 0x200000), MemReserved);

	cm = conf.mem;
	for(base = memmapnext(-1, MemRAM); base != -1; base = memmapnext(base, MemRAM)){
		size = memmapsize(base, BY2PG) & ~(uvlong)(BY2PG-1);
		if(size == 0)
			continue;
		if(cm >= &conf.mem[nelem(conf.mem)]){
			print("meminit: out of entries, losing %#llux-%#llux\n", base, base+size);
			continue;
		}
		base = memmapalloc(base, size, BY2PG, MemRAM);
		if(base == -1)
			continue;
		cm->base = base;
		cm->limit = base + size;
		cm->npage = size/BY2PG;
		cm++;
	}
	if(conf.mem[0].npage == 0)
		panic("meminit: no memory");
	setuzero();

	sv57 = (void *)PGROUND((uintptr)end);
	sv48 = (void *)PGROUND((uintptr)sv57 + 1);
	sv39 = (void *)PGROUND((uintptr)sv48 + 1);
	pGiB = (void *)PGROUND((uintptr)sv39 + 1);
	memset(sv57, 0, 16384);
	kmapram();
}
