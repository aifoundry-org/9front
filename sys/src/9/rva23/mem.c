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

/*
 * Map RAM at or above UZERO, where the identity map would fall in
 * user space, at KHIGH in the upper half of the Sv39 space.  The
 * kernel reaches it only through kaddr (kmap of user pages and page
 * tables); cankaddr keeps xalloc out of it.  2MiB leaves cover just
 * the bank: the rest of its GiB may alias other memory.
 */
static void
kmaphigh(uintptr base, uintptr limit)
{
	extern uintptr khighlimit;
	extern u64int *sv39;
	u64int *l2, *pte;
	uintptr pa, va;

	for(pa = base & -PGLSZ(1); pa < limit; pa += PGLSZ(1)){
		va = KHIGH + (pa - UZERO);
		pte = &sv39[PTLX(va, 2)];
		if((*pte & PTEVALID) == 0)
			*pte = PA2PTE((uintptr)bootptpage()) | PTEVALID;
		l2 = (u64int*)(uintptr)PTE2PA(*pte);
		l2[PTLX(va, 1)] = PA2PTE(pa) | 0xcf;
	}
	if(limit > khighlimit)
		khighlimit = limit;
}

static void
addbank(uvlong base, uvlong limit)
{
	Confmem *cm;

	base = PGROUND(base);
	limit &= ~(uvlong)(BY2PG-1);
	if(limit <= base || limit - base < MiB)
		return;
	if(base >= UZERO && limit - UZERO > KHIGHSIZE){
		print("meminit: bank %#llux-%#llux beyond KHIGH, not used\n", base, limit);
		return;
	}
	for(cm = &conf.mem[1]; cm < &conf.mem[nelem(conf.mem)]; cm++)
		if(cm->npage == 0)
			break;
	if(cm == &conf.mem[nelem(conf.mem)]){
		print("meminit: no room for bank %#llux-%#llux\n", base, limit);
		return;
	}
	cm->base = base;
	cm->limit = limit;
	cm->npage = (limit - base)/BY2PG;
}

/* add [base, limit) less the reserved regions from the r'th on */
static void
addrange(uvlong base, uvlong limit, int r)
{
	uvlong rb, rl;

	for(; dtbmemregion(1, r, &rb, &rl) == 0; r++){
		if(rl <= base || rb >= limit)
			continue;
		if(rb > base)
			addrange(base, rb, r+1);
		base = rl;
		if(base >= limit)
			return;
	}
	addbank(base, limit);
}

/*
 * RAM banks in the DTB other than the one the kernel was loaded
 * in, which is conf.mem[0].  Returns the highest limit below UZERO,
 * for kmapram.
 */
static uintptr
dtbbanks(uintptr l)
{
	uvlong b, e;
	Confmem *cm;
	int i;

	for(i = 0; dtbmemregion(0, i, &b, &e) == 0; i++)
		if(VDRAM < b || VDRAM >= e)
			addrange(b, e, 0);
	for(cm = &conf.mem[1]; cm < &conf.mem[nelem(conf.mem)]; cm++){
		if(cm->npage == 0)
			continue;
		if(cm->base >= UZERO)
			kmaphigh(cm->base, cm->limit);
		else if(cm->limit > l)
			l = cm->limit;
		print("meminit: bank %#p-%#p\n", cm->base, cm->limit);
	}
	return l;
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
	conf.mem[0].limit = l;

	if(l > KLIMIT)
		l = KLIMIT;
	sv57 = (void *)PGROUND((uintptr)end);
	sv48 = (void *)PGROUND((uintptr)sv57 + 1);
	sv39 = (void *)PGROUND((uintptr)sv48 + 1);
	pGiB = (void *)PGROUND((uintptr)sv39 + 1);
	memset(sv57, 0, 16384);
	kmapram(conf.mem[0].base, dtbbanks(l));

	conf.mem[0].npage = (conf.mem[0].limit - conf.mem[0].base)/BY2PG;
}
