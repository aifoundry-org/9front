#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"
#include "ureg.h"
#include "riscv64.h"

/*
 * RISC-V AIA: an APLIC domain in MSI delivery mode feeding the
 * per-hart IMSIC interrupt files (the K3's arrangement, and qemu-virt
 * with aia=aplic-imsic).  The APLIC converts wired source i into an
 * MSI carrying external-interrupt identity i; the target hart's IMSIC
 * latches it and raises a supervisor external interrupt, which we
 * claim by swapping 0 into CSR stopei.  We number identities the same
 * as sources, so the vctl index is just the DTB irq number, as with
 * the PLIC.
 *
 * Addresses come from the DTB (bootargs.c).  M-mode firmware already
 * programmed the domain's MSI address decode (smsiaddrcfg); those
 * registers are read-only from here, so the only routing we choose is
 * the hart index and identity in each source's target register.  All
 * sources go to the boot hart for now, like the PLIC driver's single
 * S context.
 *
 * IMSIC enable bits (eie) are CSRs of the local hart only, and
 * intrenable can be called from any cpu, so aplicinit enables every
 * identity up front on the boot hart and delivery is gated per source
 * at the APLIC.
 */

enum {
	/* APLIC domain memory map, byte offsets */
	Domaincfg	= 0x0000,
	Sourcecfg	= 0x0004,	/* + 4*(i-1) */
	Setipnum	= 0x1cdc,
	Clripnum	= 0x1ddc,
	Setienum	= 0x1edc,
	Clrienum	= 0x1fdc,
	Genmsi		= 0x3000,
	Target		= 0x3004,	/* + 4*(i-1) */

	Domainsize	= 0x8000,

	/* domaincfg */
	Ie		= 1<<8,		/* domain interrupt enable */
	Dm		= 1<<2,		/* delivery mode: MSI */

	/* sourcecfg source modes */
	Sminactive	= 0,
	Smdetached	= 1,
	Smedgerise	= 4,
	Smedgefall	= 5,
	Smlevelhigh	= 6,
	Smlevellow	= 7,

	/* target, MSI delivery mode */
	Thartshift	= 18,

	/* IMSIC interrupt-file registers, via siselect */
	Eidelivery	= 0x70,
	Eithreshold	= 0x72,
	Eie0		= 0xc0,
};

extern uintptr setsie(uintptr);

static Lock vctllock;
static Vctl *vctl[Ngintr];
static uchar srcmode[Ngintr];
static u32int *regs;
static uint nsrc;
static int mhart;		/* APLIC hart index all MSIs are routed to */

static u32int
rd(uint off)
{
	return regs[off/4];
}

static void
wr(uint off, u32int v)
{
	regs[off/4] = v;
}

void
aplicinit(void)
{
	uint i;

	regs = vmap((uintptr)soc.aplic, soc.aplicsize? soc.aplicsize: Domainsize);
	if(regs == nil)
		panic("aplicinit: aplic vmap");
	nsrc = soc.naplicsrc;
	if(nsrc == 0 || nsrc >= Ngintr)
		nsrc = Ngintr - 1;

	wr(Domaincfg, 0);
	for(i = Firstirq; i <= nsrc; i++){
		wr(Sourcecfg + 4*(i-1), Sminactive);
		wr(Clrienum, i);
	}
	mhart = m->machno;

	/*
	 * this hart's interrupt file: deliver any enabled identity,
	 * no priority threshold, all identities enabled (see above).
	 */
	for(i = 0; i <= nsrc/64; i++)
		imsicswr(Eie0 + 2*i, i == 0? ~1ULL: ~0ULL);	/* identity 0 doesn't exist */
	imsicswr(Eithreshold, 0);
	imsicswr(Eidelivery, 1);

	wr(Domaincfg, Ie|Dm);
	coherence();
	if((rd(Domaincfg) & Dm) == 0)
		panic("aplicinit: domain can't do MSI delivery");
	print("aplic: %ud sources, domaincfg %ux\n", nsrc, rd(Domaincfg));

	/* enable S-mode external interrupts in SIE CSR */
	setsie(Seie);
}

/*
 *  called via intr.c to handle supervisor external interrupts.
 *  returns 0; external interrupts are never clock interrupts.
 *
 *  Iteration is bounded, as in plic.c: a source that immediately
 *  re-raises can otherwise pin the cpu here with sstatus.SIE=0,
 *  silently stopping the clock.
 */
int
aplicirq(Ureg *ureg)
{
	Vctl *v;
	uint id;
	int trips;

	m->intr++;
	trips = 64;
	while((id = stopeiclaim() >> 16) != 0){
		v = id < Ngintr? vctl[id]: nil;
		if(v == nil){
			if(id >= Firstirq && id <= nsrc){
				wr(Sourcecfg + 4*(id-1), Sminactive);
				coherence();
			}
			iprint("aplic: spurious irq %ud, masking\n", id);
		} else {
			for(; v != nil; v = v->next)
				v->f(ureg, v->a);
			/*
			 * MSI delivery clears a level source's pending bit when
			 * the MSI is sent, and only a fresh edge would set it
			 * again.  If the handler left the device asserted (more
			 * work arrived), poking setipnum re-raises the source;
			 * if the line dropped, it's a no-op.
			 */
			if(id <= nsrc && srcmode[id] >= Smlevelhigh)
				wr(Setipnum, id);
		}
		if(--trips <= 0){
			if(id >= Firstirq && id <= nsrc){
				wr(Sourcecfg + 4*(id-1), Sminactive);
				coherence();
			}
			iprint("aplic: irq %ud storm, masking source\n", id);
			break;
		}
	}
	idlewake();
	return 0;
}

void
aplicintrenable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *name)
{
	Vctl *v;

	if(BUSTYPE(tbdf) == BusPCI){
		pciintrenable(tbdf, f, a);
		return;
	}

	if(tbdf != BUSUNKNOWN)
		return;
	if((uint)irq < Firstirq || (uint)irq > nsrc || (uint)irq >= Ngintr)
		return;

	v = mallocz(sizeof(Vctl), 1);
	if(v == nil)
		panic("aplicintrenable: no mem");
	v->isintr = 1;
	v->irq = irq;
	v->vno = irq;
	v->tbdf = tbdf;
	v->f = f;
	v->a = a;
	v->cpu = -1;
	strncpy(v->name, name, KNAMELEN-1);

	lock(&vctllock);
	v->next = vctl[irq];
	vctl[irq] = v;

	if(srcmode[irq] == Sminactive)
		srcmode[irq] = Smlevelhigh;	/* qemu-virt peripherals; TODO: modes from DTB interrupt flags */
	/*
	 * order matters: target[i] is read-only zero (writes discarded)
	 * while source i is inactive, so sourcecfg must go live first.
	 */
	wr(Sourcecfg + 4*(irq-1), srcmode[irq]);
	wr(Target + 4*(irq-1), (u32int)mhart<<Thartshift | irq);
	wr(Setienum, irq);
	coherence();
	/*
	 * read back for the boot log: both are WARL, and a sourcecfg
	 * stuck at 0 (delegation missing) or a target stuck at 0 is
	 * exactly what a "boots but no interrupts" report needs to show.
	 */
	print("aplic: src %d for %s: cfg %ux target %ux\n", irq, name,
		rd(Sourcecfg + 4*(irq-1)), rd(Target + 4*(irq-1)));
	unlock(&vctllock);
}

void
aplicintrdisable(int, void (*f)(Ureg*, void*), void *a, int tbdf, char*)
{
	if(BUSTYPE(tbdf) == BusPCI){
		pciintrdisable(tbdf, f, a);
		return;
	}
}
