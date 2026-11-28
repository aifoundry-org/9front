/*
 * The kernel's view of physical memory: the boot page tables that
 * identity-map it (kmapram), each hart's root table (mmu1init), device
 * mappings (vmap) and uncached DMA memory (mmuuncache).  A board
 * directory with a different address map supplies its own memmap.c.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

enum {
	Spew = 0,
};

extern u64int *sv57, *sv48, *sv39, *pGiB;

void
mmu1init(void)
{
	m->mmutop = mallocalign(L1TOPSIZE, BY2PG, 0, 0);
	if(m->mmutop == nil)
		panic("mmu1init: no memory for mmutop");
	memset(m->mmutop, 0, L1TOPSIZE);
	memmove(m->mmutop, sv39, L1TOPSIZE);
	mmuswitch(nil, 0);
}

static int
ramslot(uintptr pa)
{
	Confmem *cm;

	for(cm = conf.mem; cm < &conf.mem[nelem(conf.mem)]; cm++)
		if(cm->npage != 0 && cm->base < pa + PGLSZ(2) && cm->limit > pa)
			return 1;
	return 0;
}

/*
 * Identity-map [0, UZERO): GiB slots holding RAM through L2 tables
 * of 2MiB leaves, the first 6 slots (devices) with 1GiB leaves; other
 * slots stay unmapped.
 */
void
kmapram(void)
{
	u64int i, j, pa;
	u64int *l2;

	sv57[0] = ((((u64int)sv48)>>2)) | 1;
	sv48[0] = ((((u64int)sv39)>>2)) | 1;
	for(i = 0; i < UZERO/PGLSZ(2); i++){
		pa = i*PGLSZ(2);
		if(ramslot(pa)){
			/*
			 * Every hart's mmutop is a by-value copy of sv39, so
			 * the shared L2 table means a later mmuuncache() that
			 * edits an L2/L3 entry is seen by all harts with only
			 * a TLB shootdown (see mmuuncache).
			 */
			l2 = bootptpage();
			for(j = 0; j < 512; j++)
				l2[j] = PA2PTE(pa + j*PGLSZ(1)) | 0xcf;
			sv39[i] = PA2PTE((uintptr)l2) | PTEVALID;
		} else if(i < 6)
			sv39[i] = PA2PTE(pa) | 0xcf;
	}

	wsatp(((uintptr)sv39>>12)|(8ULL<<60));
}

void*
vmap(uvlong pa, vlong size)
{
	uvlong a;

	if(size <= 0)
		size = 1;
	if(pa >= UZERO || size > UZERO - pa)
		return nil;
	if(sv39 == nil)
		return (void*)pa;	/* paging still off */
	for(a = pa; a < pa+size; a += PGLSZ(2) - a%PGLSZ(2))
		if(sv39[a/PGLSZ(2)] == 0)
			return nil;
	return (void*)pa;
}

/*
 * Flush a kernel VA range from every hart's TLB.  kmapram() maps DRAM through
 * shared L2 tables, so a page-table edit is already visible to all harts; only
 * stale TLB entries need clearing.  Online harts are 0..nonline-1 with
 * machno == hartid (see archrv.c).
 */
static void
uncacheshootdown(uintptr va, usize size)
{
	uvlong mask;

	flushalltlb();				/* this hart */
	if(nosbi || sys->nonline <= 1)
		return;
	mask = ((1ULL << sys->nonline) - 1) & ~(1ULL << m->machno);
	if(mask)
		sbiremotesfencevma(&mask, va, size);
}

/*
 * Make an identity-mapped kernel range non-cacheable (Svpbmt NC) so it can
 * hold DMA buffers on this non-coherent SoC.  v must be page aligned and lie
 * in DRAM, which kmapram() mapped with 2MiB leaves under a shared L2 table.
 * We split the covering 2MiB leaves down to 4KiB and set PTEUNCACHED on the
 * pages of [v, v+size); the shared tables mean all harts see the new PTEs, so
 * only a TLB shootdown is needed.  Helper for ucalloc(); returns v, or nil if
 * v isn't in such a range.
 */
void*
mmuuncache(void *v, usize size)
{
	uintptr va, e, pa;
	u64int *l2, *l3, l2e;
	int j, l1x, l2x;

	va = (uintptr)v;
	e = va + size;
	if((va & (BY2PG-1)) != 0 || size == 0)
		return nil;
	for(; va < e; va += BY2PG){
		l1x = PTLX(va, 2);
		if((sv39[l1x] & PTEVALID) == 0 ||
		   (sv39[l1x] & (PTEREAD|PTEWRITE|PTEEXEC)) != 0)
			return nil;		/* not a shared L2, i.e. not DRAM */
		l2 = (u64int*)PTE2PA(sv39[l1x]);
		l2x = PTLX(va, 1);
		l2e = l2[l2x];
		if((l2e & (PTEREAD|PTEWRITE|PTEEXEC)) != 0){
			/* 2MiB leaf: split into a shared L3 table of 4KiB leaves */
			pa = PTE2PA(l2e);
			l3 = mallocalign(BY2PG, BY2PG, 0, 0);
			if(l3 == nil)
				return nil;
			for(j = 0; j < 512; j++)
				l3[j] = PA2PTE(pa + j*BY2PG) | (l2e & 0x3FF);
			coherence();
			l2[l2x] = PA2PTE((uintptr)l3) | PTEVALID;
		} else
			l3 = (u64int*)PTE2PA(l2e);
		l3[PTLX(va, 0)] = (l3[PTLX(va, 0)] & ~PTEPBMT) | PTEUNCACHED;
	}
	coherence();
	/*
	 * The range was cacheable until now, so this hart (the only one to
	 * have touched freshly allocated memory) may hold dirty lines that
	 * would later evict and clobber DMA data.  Write them back and drop
	 * them before the mapping goes live as non-cacheable.
	 */
	for(va = (uintptr)v & ~(Cachelinesz-1); va < e; va += Cachelinesz)
		cboflush((void*)va);
	uncacheshootdown((uintptr)v, size);
	return v;
}
