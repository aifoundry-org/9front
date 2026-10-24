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

void
meminit(void)
{
	char *p;
	uintptr l = VDRAM + 2ULL * GiB;
	extern u64int *sv57, *sv48, *sv39, *pGiB;
	extern uintptr klimit;

	if(p = getconf("*maxmem"))
		l = strtoull(p, 0, 0);

	// leave a big hole after end, for now. We need it for this and that.
	conf.mem[0].base = ROUND(PGROUND((uintptr)end - KTZERO) + (uintptr)(VDRAM+0x2fffff), 0x200000);
	if(l > KLIMIT)
		l = KLIMIT;
	conf.mem[0].limit = l;
	if(l <= conf.mem[0].base)
		panic("meminit: kernel does not fit in RAM");
	sv57 = (void *)PGROUND((uintptr)end);
	sv48 = (void *)PGROUND((uintptr)sv57 + 1);
	sv39 = (void *)PGROUND((uintptr)sv48 + 1);
	pGiB = (void *)PGROUND((uintptr)sv39 + 1);
	memset(sv57, 0, 16384);
	kmapram(conf.mem[0].base, l);

	conf.mem[0].npage = (conf.mem[0].limit - conf.mem[0].base)/BY2PG;
}
