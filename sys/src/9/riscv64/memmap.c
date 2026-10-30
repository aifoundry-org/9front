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
	mmuswitch(nil);
}

void
kmapram(uintptr, uintptr limit)
{
	u64int i, j, n, pa;
	u64int *l2;

	sv57[0] = ((((u64int)sv48)>>2)) | 1;
	if (Spew)print("%p is %p\n", sv57, sv57[0]);
	sv48[0] = ((((u64int)sv39)>>2)) | 1;
	/*
	 * cover every GiB slot up to the end of RAM, not a fixed few:
	 * pages the kernel can only reach through this identity map
	 * (kaddr) fault if their slot is missing.  Keep at least the
	 * first 6 slots for the low MMIO leaves, and stop at UZERO,
	 * where user space begins.
	 * TODO: handle more complex memory topologies.
	 */
	n = (limit + 0x40000000ULL-1)/0x40000000ULL;
	if(n < 6)
		n = 6;
	if(n > UZERO/0x40000000ULL)
		n = UZERO/0x40000000ULL;
	for(i = 0; i < n; i++){
		pa = 0x40000000ULL*i;
		/*
		 * A slot counts as "DRAM" if it overlaps [VDRAM, limit) at
		 * all, not just when its aligned start is >= VDRAM: VDRAM
		 * (where the bootloader actually placed the kernel) is not
		 * itself GiB-aligned in general, so the slot straddling it
		 * would otherwise be misclassified as a plain 1GiB leaf --
		 * even though conf.mem[0].base (and hence the very first
		 * pages mainmem ever hands out) lives inside that same
		 * slot, right above VDRAM.
		 */
		if(pa + 0x40000000ULL > VDRAM && pa < limit){
			/*
			 * DRAM: map with 2MiB leaves reached through a shared L2
			 * table rather than a single 1GiB leaf.  Every hart's mmutop
			 * is a by-value copy of sv39, so the shared L2 pointer means
			 * a later mmuuncache() that edits an L2/L3 entry is seen by
			 * all harts with only a TLB shootdown (see mmuuncache).
			 */
			l2 = bootptpage();
			for(j = 0; j < 512; j++)
				l2[j] = PA2PTE(pa + j*PGLSZ(1)) | 0xcf;
			sv39[i] = PA2PTE((uintptr)l2) | PTEVALID;
		} else
			/* below/above DRAM (includes MMIO): keep the 1GiB leaf */
			sv39[i] = PA2PTE(pa) | 0xcf;
		if (Spew)print("sv39:%p is %p\n", &sv39[i], sv39[i]);
	}

	wsatp(((uintptr)sv39>>12)|(8ULL<<60));

}

void*
vmap(uvlong pa, vlong)
{
	return (void *)pa;
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
