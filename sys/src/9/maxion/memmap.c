/*
 * ET-SoC-1 Maxion memory map, replacing ../rva23/memmap.c.
 *
 * OpenSBI starts hart 0 of the Maxion cluster in an owned, 1GiB
 * aligned RAM window at 0x8600000000, out of Sv39's reach, with the
 * kernel and a device tree there.  So this kernel's page tables have
 * four levels (EVASHIFT in mem.h) and the harts run in Sv48 (wsatp in
 * fns.h); user space starts at the root slot above the window and each
 * Mach's root is a copy of the kernel's.  Only the RAM window, the
 * PLIC, the console UART and the packet mailbox page are mapped, and
 * vmap refuses anything else.  The harts have neither Sstc, Zicbom nor
 * Svpbmt and DMA is not coherent: no PCI, storage or DMA devices.  In
 * particular, the QEMU ECAM address is a real Maxion mailbox, so no
 * PCI scanner is ever linked.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "maxion.h"

extern u64int *sv57, *sv48, *sv39, *pGiB;

static uintptr ramlo, ramhi;	/* the RAM window */

#undef wsatp
void
wsatp48(u64int satp)
{
	wsatp(satp & ~(0xfULL<<60) | 9ULL<<60);
}

void
mmu1init(void)
{
	m->mmutop = mallocalign(L1TOPSIZE, BY2PG, 0, 0);
	if(m->mmutop == nil)
		panic("mmu1init: no memory for mmutop");
	memmove(m->mmutop, sv48, L1TOPSIZE);
	mmuswitch(nil, 0);
}

static void
maxionleaf(uintptr pa, int level, uintptr attr)
{
	u64int *table, pte;
	int l;

	if(pa >= uzero)
		panic("maxionleaf: %#p in user space", pa);
	table = sv48;
	for(l = PTLEVELS-1; l > level; l--){
		pte = table[PTLX(pa, l)];
		if(pte == 0){
			pte = PA2PTE((uintptr)bootptpage()) | PTEVALID;
			table[PTLX(pa, l)] = pte;
		}
		if(pte & (PTEREAD|PTEWRITE|PTEEXEC))
			panic("maxionleaf: overlapping mapping");
		table = (u64int*)PTE2PA(pte);
	}
	table[PTLX(pa, level)] = PA2PTE(pa) | attr;
}

/*
 * Fail closed: only an explicit, aligned, 1GiB loader window will do,
 * and it is the only RAM used.  It is not a host reservation.
 */
static void
window(void)
{
	Confmem *cm;

	ramlo = VDRAM;
	ramhi = conf.mem[0].limit;
	if((ramlo & (GiB-1ULL)) != 0 || ramhi <= ramlo
	|| ramhi - ramlo > GiB || (ramhi & (GiB-1ULL)) != 0)
		panic("maxion: require aligned 1 GiB RAM window");
	if(ramhi > uzero)
		panic("maxion: RAM window above user space");
	for(cm = &conf.mem[1]; cm < &conf.mem[nelem(conf.mem)]; cm++)
		if(cm->npage != 0){
			print("maxion: ram %#p-%#p not used\n", cm->base, cm->limit);
			cm->limit = cm->base;
			cm->npage = 0;
		}
}

/*
 * Map only the RAM window, PLIC, console UART and packet page, switch
 * to Sv48 and prove the top of RAM is reachable.  The DT is read in
 * Bare mode; its firmware-owned relocated copy need not remain mapped.
 */
void
kmapram(void)
{
	uintptr pa, base, attr, satp;
	volatile uvlong *probe;
	uvlong saved;
	Dtbuart *du;
	Dtbmxp *dm;
	char *p;
	int i;

	if(!dtbsocmatch("esperanto,et-soc-1")
	&& ((p = getconf("*soc")) == nil || strcmp(p, "maxion") != 0))
		panic("maxion: not an ET-SoC-1 device tree (*soc=maxion overrides)");
	print("maxion: physical entry %#p\n", (uintptr)_start);
	window();
	maxiondt();
	attr = PTEVALID|PTEREAD|PTEWRITE|PTEACCESSED|PTEDIRTY;
	maxionleaf(ramlo, 2, attr|PTEEXEC);
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
	/*
	 * The packet mailbox is one page of SP SRAM, uncached by PMA.
	 * Map it only when the bootstrap described it; never the ROM
	 * boot marker page below it, and never RAM or user space.
	 */
	dm = dtbmxp();
	if(dm != nil){
		if(dm->base + BY2PG > uzero
		|| dm->base < ramhi && dm->base + BY2PG > ramlo){
			print("maxion: packet page %#llux in RAM or user space; disabled\n", dm->base);
			dm->ok = 0;
		}else
			maxionleaf(dm->base, 0, attr);
	}
	satp = ((uintptr)sv48>>12) | 9ULL<<60;
	coherence();
	wsatp(satp);
	if(rsatp() != satp)
		panic("maxion: Sv48 root rejected; refusing userspace");
	probe = (volatile uvlong*)(ramhi - BY2PG);
	saved = *probe;
	*probe = 0x4d4158494f4e3950ULL;
	coherence();
	if(*probe != 0x4d4158494f4e3950ULL)
		panic("maxion: top-of-RAM mapping failed");
	*probe = saved;
	coherence();
	print("maxion: Sv48 active, satp %#p, RAM %#p..%#p\n", satp, ramlo, ramhi);
}

/* only what kmapram mapped: the RAM window, the PLIC and the console UART */
void*
vmap(uvlong pa, vlong size)
{
	Dtbuart *du;
	uvlong e, base;
	int i;

	if(size <= 0 || pa + size < pa)
		return nil;
	e = pa + size;
	if(pa >= ramlo && e <= ramhi)
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
