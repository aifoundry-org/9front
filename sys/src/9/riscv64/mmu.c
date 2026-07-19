#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

enum {
	Spew = 0,
};

#define INITMAP	(ROUND((uintptr)end + BY2PG, PGLSZ(1))-KZERO)

// kmax will be computed dynamically in future; for now it is
// UZERO.
uintptr klimit = UZERO;

void
mmu1init(void)
{
	extern u64int *sv57, *sv48, *sv39, *pGiB;
	m->mmutop = mallocalign(L1TOPSIZE, BY2PG, 0, 0);
	if(m->mmutop == nil)
		panic("mmu1init: no memory for mmutop");
	memset(m->mmutop, 0, L1TOPSIZE);
	memmove(m->mmutop, sv39, L1TOPSIZE);
	mmuswitch(nil);
}

uintptr
paddr(void *va)
{
	if((uintptr)va < KLIMIT)
		return (uintptr)va;
	panic("paddr: va=%#p pc=%#p", va, getcallerpc(&va));
}

uintptr
cankaddr(uintptr pa)
{
//	print("cankaddr %p, -KZ %p\n", pa,(uintptr)-KZERO );
	if(pa < (uintptr)KLIMIT)
		return KLIMIT - pa;
	return 0;
}

void*
kaddr(uintptr pa)
{
	if(pa < (uintptr)KLIMIT)
		return (void*)(pa);
	panic("kaddr: pa=%#p pc=%#p", pa, getcallerpc(&pa));
}

static void*
kmapaddr(uintptr pa)
{
	return kaddr(pa);
}

KMap*
kmap(Page *p)
{
	return kmapaddr(p->pa);
}

void
kunmap(KMap*)
{
}

void
kmapinval(void)
{
}

static void*
rampage(void)
{
	uintptr pa;

	if(conf.npage)
		return mallocalign(BY2PG, BY2PG, 0, 0);

	pa = conf.mem[0].base;
	assert((pa % BY2PG) == 0);
	conf.mem[0].base += BY2PG;
	return KADDR(pa);
}

// nothing to do.
static void
l1map(uintptr va, uintptr pa, uintptr pe, uintptr attr)
{
	USED(va);
	USED(pa);
	USED(pe);
	USED(attr);
}

u64int *sv57, *sv48, *sv39, *pGiB;
u64int mmumode;

/*
 * Carve one page-table page out of the front of the physical pool at boot,
 * before conf.npage is known.  Nothing has zeroed the pool, so zero the
 * page here.  Identity mapped, so va == pa.
 */
static u64int*
bootptpage(void)
{
	uintptr pa;

	pa = conf.mem[0].base;
	assert((pa & (BY2PG-1)) == 0);
	conf.mem[0].base += BY2PG;
	memset((void*)pa, 0, BY2PG);
	return (u64int*)pa;
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
		if(pa >= VDRAM && pa < limit){
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

uintptr
mmukmap(uintptr va, uintptr pa, usize size)
{
	uintptr attr = 0, off;

	if(va == 0)
		return 0;

	off = pa & BY2PG-1;

//	TODO: When we support memory access cache flags, need to do a copy here
//	attr = va & PTEMA(7);
	attr |= PTEWRITE | PTEREAD;

	va &= -BY2PG;
	pa &= -BY2PG;

	l1map(va, pa, pa + off + size, attr);
	flushtlb();

	return va + off;
}

void*
vmap(uvlong pa, vlong size)
{
	return (void *)pa;
}

void
vunmap(void *, vlong)
{
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

/* That macro hackery is just too much for me to look at, and kenc should
 * inline this function anyway. (it does not, but it could) */
static u64int vpn(uintptr va, int level)
{
	int shift = 12 + 9*level;
	uintptr val = (va>>shift)&0x1ff;
	if (0)print("<<<vpn(%p,%d)[shift %d]=%p>>>", va, level, shift, val);
	return val;
}

static uintptr 
ptephys(uintptr pte)
{
	return (pte & ~(0xFFFFULL<<48 | BY2PG-1))<<2;
}

/* there are still a few nasty assumptions in here about
 * pte not needing left shifts etc. riscv really fucked up
 * the pte format. Intel did better.
 */
static uintptr*
mmuwalk(uintptr va, int level)
{
	uintptr *table, pte;
	Page *pg;
	int i, x;
// In future, PTLEVELS will be dynamic.
	if (0)print("mmuwalk: va %p, walk to level %d starting from %d,", va, level, PTLEVELS);
	x = vpn(va, PTLEVELS-1);
	// N.B.: the assumption here is that mmutop was already set from 
	// p->mmutop. mmutop[x] will never be non-zero. If it is, it's a bug.
	table = m->mmutop;
	if (0)print("MMUWALK m is %p MMUTOP is %p\n", m,  m->mmutop);
	for(i = PTLEVELS-2; i >= level; i--){
		if (0)print("%d: table %p, index %d, pte %p: ", i, table, x, table[x]);
		pte = table[x];
		if (0)print(" %p %s, points to %p,", pte, pte & PTEVALID ? "valid" : "invalid", (pte>>10)<<12);
		if(pte & PTEVALID) {
			if (0){
				if(pte & (0xFFFFULL<<48))
					iprint("strange pte %#p va %#p, ", pte, va);
				pte &= ~(0xFFFFULL<<48 | BY2PG-1);
				pte <<= 2;
			}
			pte = (pte >>10)<<12;
		} else {
			pg = up->mmufree;
			if(pg == nil){
				if (0)print("up->mmufree is empty, return nil\n");
				return nil;
			}
			up->mmufree = pg->next;
			pg->va = va & -PGLSZ(i+1);
			if (0)print("page for pte is %p, ", pg->va);
			if((pg->next = up->mmuhead[i+1]) == nil)
				up->mmutail[i+1] = pg;
			up->mmuhead[i+1] = pg;
			if (0)print("SET up->mmuhead[%d] = %p, pte@ %p\n", i+1, pg, pg->pa);
			pte = pg->pa;
			memset(kmapaddr(pg->pa), 0, BY2PG);
			coherence();
			if (0)print(": SET table %p[%x]@%p = addr %p val%llx\n", table, x, &table[x], pte, ((pte>>12)<<10) | PTEVALID);
			table[x] = ((pte>>12)<<10) | PTEVALID;	// XXX: Does this need PA2PTE
		}
		table = kmapaddr(pte);
		if (0)print("kmapaddr of %p is %p, ", pte, table);
		x = vpn(va, (uintptr)i);
		if (0)print("\n");
	}
if (0)print("RETURN &%p[0x%x] = ", table, x);
if (0)print("%p\n", &table[x]);
	return &table[x];
}

u64int *
userpte(void *v)
{
	uintptr *pte = mmuwalk((uvlong)v, 0);
	if (v == nil){
		print("%p in up %p: not mapped\n", v, up);
		error("not mapped");
	}
	if (Spew)print("userpte %p -> pte %p contains %p phys %p v %p\n", v, pte, *pte, ptephys(*pte), kmapaddr(ptephys(*pte)));
	return pte;
}

void *
usertokernel(void *v)
{
	uintptr *pte = mmuwalk((uvlong)v, 0);
	if (v == nil){
		if (Spew)print("%p in up %p: not mapped\n", v, up);
		error("not mapped");
	}
	if (Spew)print("usertokernel %p -> pte %p contains %p phys %p v %p\n", v, pte, *pte, ptephys(*pte), kmapaddr(ptephys(*pte)));
	return kmapaddr((uintptr)pte);
}

static Proc *asidlist[256];

static int
allocasid(Proc *p)
{
	static Lock lk;
	Proc *x;
	int a;

	lock(&lk);
	a = p->asid;
	if(a < 0)
		a = -a;
	if(a == 0)
		a = p->pid;
	for(;; a++){
		a %= nelem(asidlist);
		if(a == 0)
			continue;	// reserved
		x = asidlist[a];
		if(x == p || x == nil || (x->asid < 0 && x->mach == nil))
			break;
	}
	p->asid = a;
	asidlist[a] = p;
	unlock(&lk);

	return x != p;
}

static void
freeasid(Proc *p)
{
	int a;

	a = p->asid;
	if(a < 0)
		a = -a;
	if(a > 0 && asidlist[a] == p)
		asidlist[a] = nil;
	p->asid = 0;
}

void
putasid(Proc *p)
{
	/*
	 * Prevent the following scenario:
	 *	pX sleeps on cpuA, leaving its page tables in mmutop
	 *	pX wakes up on cpuB, and exits, freeing its page tables
	 *  pY on cpuB allocates a freed page table page and overwrites with data
	 *  cpuA takes an interrupt, and is now running with bad page tables
	 * In theory this shouldn't hurt because only user address space tables
	 * are affected, and mmuswitch will clear mmutop before a user process is
	 * dispatched.  But empirically it correlates with weird problems, eg
	 * resetting of the core clock at 0x4000001C which confuses local timers.
	 */
	if(conf.nmach > 1)
		mmuswitch(nil);

	if(p->asid > 0)
		p->asid = -p->asid;
}

void
putmmu(uintptr va, uintptr pa, Page *pg)
{
	uintptr *pte, old;
	int s;
	uintptr pteattr = PTE2ATTR(pa) | PTEUSER | PTEACCESSED | PTEEXEC; // hack.

	if (pteattr & PTEWRITE)
		pteattr |= PTEDIRTY | PTEREAD;

if (Spew)print("pid %lud putmmu(%p, %p, %p)\n", up ? up->pid : 0, va, pa, pg);
	s = splhi();
	while((pte = mmuwalk(va, 0)) == nil){
		spllo();
		up->mmufree = newpage(0, nil);
		if (0)print("putmmu: get a page %p, try again\n", up->mmufree);
		splhi();
	}
	if (up->pid == 5) soft();
	old = *pte;
	*pte = 0;
	if((old & PTEVALID) != 0)
		flushasidvall((uvlong)up->asid<<48 | va>>12);
	else
		flushasidva((uvlong)up->asid<<48 | va>>12);
	*pte = PA2PTE(pa) | pteattr;
	if (Spew)print("va %p pa %p pte %p *pte %p\n", va, pa, pte, *pte);
	if(needtxtflush(pg)){
		cachedwbinvse(kmap(pg), BY2PG);
		cacheiinvse((void*)va, BY2PG);
		donetxtflush(pg);
	}
	flushalltlb();
	flushvatlb(va);
	wsatp(rsatp());
	if (up->pid == 5) soft();
	splx(s);
	if (0)print("putmmu done\n");
}

static void
mmufree(Proc *p)
{
	int i;

	freeasid(p);

	for(i=1; i<PTLEVELS; i++){
		if(p->mmuhead[i] == nil)
			break;
		p->mmutail[i]->next = p->mmufree;
		p->mmufree = p->mmuhead[i];
		p->mmuhead[i] = p->mmutail[i] = nil;
	}
}

void
mmuswitch(Proc *p)
{
	uintptr va;
	Page *t;
if (0)	print("SWITCH MMUTOP IS %p, @ 100 is %p\n", m->mmutop, m->mmutop[0x100]);
	for(va = UZERO; va < USTKTOP; va += PGLSZ(PTLEVELS-1))
		m->mmutop[PTLX(va, PTLEVELS-1)] = 0;

if (0)	print("p %p for setting tbr?\n", p);
	if(p == nil){
		// maybe flush the user mode entries? probably
		if(Spew)print("mmuswitch p is nil what to do?\n");
		wsatp(((uintptr)m->mmutop>>12)|(8ULL<<60));
		return;
	}

	if(p->newtlb){
		mmufree(p);
		p->newtlb = 0;
	}
if (0)	print("allocasid(p) %d\n", allocasid(p));
	if(allocasid(p)){
		if (Spew)print("NOT messing with ASID\n");
		flushasid((uvlong)p->asid<<48);
	}

	//print("set tbr to %llx\n", (uvlong)p->asid<<48 | PADDR(m->mmutop));
	//setttbr((uvlong)p->asid<<48 | PADDR(m->mmutop));

	for(t = p->mmuhead[PTLEVELS-1]; t != nil; t = t->next){
		va = t->va;
		u64int pte = ((t->pa)>>12) << 10 | PTEVALID; //| PTEUSER;
if (0)		print("Set mmutop[%llxx] to %llx\n", PTLX(va, PTLEVELS-1), pte );
		m->mmutop[PTLX(va, PTLEVELS-1)] = pte;
	}
	wsatp(((uintptr)m->mmutop>>12)|(8ULL<<60));
}

void
mmurelease(Proc *p)
{
	mmuswitch(nil);
	mmufree(p);
	freepages(p->mmufree, nil, 0);
	p->mmufree = nil;
}

void
flushmmu(void)
{
	int x;

	x = splhi();
	up->newtlb = 1;
	mmuswitch(up);
	splx(x);
}

void
checkmmu(uintptr, uintptr)
{
}
