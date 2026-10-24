/*
 * ET-SoC-1 Maxion memory map, replacing ../riscv64/memmap.c.  RAM, and
 * this kernel, sit at 0x8600000000, out of Sv39's reach, so the harts
 * run in Sv48 with this kernel's existing 64..128 GiB user ABI and
 * three-level user tables: each Mach's low sv39 table hangs from slot 0
 * of a four-level root whose other slots are shared kernel mappings.
 * Only the owned RAM window, the PLIC and the console UART are mapped,
 * and vmap refuses anything else.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

extern u64int *sv57, *sv48, *sv39, *pGiB;

/* each Mach's own low sv39 table under a copy of the shared root */
void
mmu1init(void)
{
	m->mmutop = mallocalign(L1TOPSIZE, BY2PG, 0, 0);
	if(m->mmutop == nil)
		panic("mmu1init: no memory for mmutop");
	memset(m->mmutop, 0, L1TOPSIZE);
	memmove(m->mmutop, sv39, L1TOPSIZE);
	m->mmuroot = mallocalign(BY2PG, BY2PG, 0, 0);
	if(m->mmuroot == nil)
		panic("mmu1init: no memory for Sv48 root");
	memmove(m->mmuroot, sv48, BY2PG);
	m->mmuroot[0] = PA2PTE((uintptr)m->mmutop) | PTEVALID;
	mmuswitch(nil);
}

/* Hardware Sv48 indices, unlike PTLX's three-level user-subtree indices. */
static void
maxionleaf(uintptr pa, int level, uintptr attr)
{
	u64int *table, pte;
	int l, x;

	table = sv48;
	for(l = 3; l > level; l--){
		x = (pa >> (PGSHIFT + l*PTSHIFT)) & 511;
		pte = table[x];
		if(pte == 0){
			pte = PA2PTE((uintptr)bootptpage()) | PTEVALID;
			table[x] = pte;
		}
		if(pte & (PTEREAD|PTEWRITE|PTEEXEC))
			panic("maxionleaf: overlapping mapping");
		table = (u64int*)PTE2PA(pte);
	}
	x = (pa >> (PGSHIFT + level*PTSHIFT)) & 511;
	table[x] = PA2PTE(pa) | attr;
}

/*
 * Map only the owned RAM, PLIC and console UART, switch
 * to Sv48 and prove the top of RAM is reachable.  The DT is parsed in
 * Bare mode; its firmware-owned relocated copy need not remain mapped.
 */
void
kmapram(uintptr, uintptr limit)
{
	uintptr pa, base, attr, satp;
	volatile uvlong *probe;
	uvlong saved;
	Dtbuart *du;
	int i;

	sv48[0] = PA2PTE((uintptr)sv39) | PTEVALID;
	attr = PTEVALID|PTEREAD|PTEWRITE|PTEACCESSED|PTEDIRTY;
	maxionleaf(VDRAM, 2, attr|PTEEXEC);
	base = (uintptr)soc.plic;
	if(base != 0){
		if((base & (PGLSZ(1)-1)) != 0)
			panic("maxion: unaligned PLIC");
		for(pa = base; pa < base + 32*MiB; pa += PGLSZ(1))
			maxionleaf(pa, 1, attr);
	}
	for(i = 0; i < dtbnuart(); i++){
		du = dtbuart(i);
		if(du != nil && du->ok && du->isconsole)
			maxionleaf(du->base & ~(BY2PG-1), 0, attr);
	}
	mmumode = 9ULL<<60;
	satp = ((uintptr)sv48>>12) | mmumode;
	coherence();
	wsatp(satp);
	if(rsatp() != satp)
		panic("maxion: Sv48 root rejected; refusing userspace");
	probe = (volatile uvlong*)(limit - BY2PG);
	saved = *probe;
	*probe = 0x4d4158494f4e3950ULL;
	coherence();
	if(*probe != 0x4d4158494f4e3950ULL)
		panic("maxion: top-of-RAM mapping failed");
	*probe = saved;
	coherence();
	print("maxion: Sv48 active, satp %#p, RAM %#p..%#p\n", satp, VDRAM, limit);
	print("maxion: top-of-RAM write/read/restore passed at %#p\n", (uintptr)probe);
}

/* only what kmapram mapped: owned RAM, the PLIC and the console UART */
void*
vmap(uvlong pa, vlong size)
{
	Dtbuart *du;
	uvlong e, base;
	int i;

	if(size <= 0 || pa + size < pa)
		return nil;
	e = pa + size;
	if(pa >= VDRAM && e <= KLIMIT)
		return (void*)pa;
	base = (uintptr)soc.plic;
	if(base != 0 && pa >= base && e <= base + 32*MiB)
		return (void*)pa;
	for(i = 0; i < dtbnuart(); i++){
		du = dtbuart(i);
		if(du != nil && du->ok && du->isconsole &&
		   pa >= du->base && e <= du->base + BY2PG)
			return (void*)pa;
	}
	return nil;
}

/* No Svpbmt or cache-block instructions; DMA allocation fails closed. */
void*
mmuuncache(void*, usize)
{
	return nil;
}
