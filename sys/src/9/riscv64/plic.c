#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"
#include "ureg.h"
#include "sysreg.h"
#include "riscv64.h"
#include "../port/error.h"

enum {
	PLICSIZE	= 0x04000000,

	/*
	 * S-mode context for the boot hart.  Both QEMU-virt (hart 0) and the
	 * SpacemiT K1 (boot hart phandle 0x10, the first entry in the PLIC's
	 * interrupts-extended) put M-mode at context 0 and S-mode at context 1.
	 * If we ever boot a kernel on a non-zero hart this will need to come
	 * from socidentify() or the DTB.
	 */
	Suctxt		= 1,
	Defprio		= 1,
};

extern uintptr setsie(uintptr);

static Lock vctllock;
static Vctl *vctl[Ngintr];
static Plic *plic;

void
intrinit(void)
{
	uint i;
	uintptr base;

	/*
	 * soc.plic was set by socidentify() to the SoC's PLIC physical base.
	 * vmap is currently identity, but call it so the type / API survives a
	 * future MMU rework.
	 */
	base = (uintptr)soc.plic;
	if(base == 0)
		panic("intrinit: soc.plic not set");
	plic = (Plic*)vmap(base, PLICSIZE);
	if(plic == nil)
		panic("intrinit: plic vmap");
	soc.plic = (uchar*)plic;

	/* mask every source */
	for(i = Firstirq; i < Ngintr; i++){
		plic->prio[i] = 0;
		plic->enabits[Suctxt][i/32] &= ~(1u << (i & 31));
	}
	plic->context[Suctxt].priothresh = 0;
	coherence();

	/* enable S-mode external interrupts in SIE CSR */
	setsie(Seie);
}

/*
 *  called by trap.c to handle external interrupts from the plic.
 *  returns 0; the plic never delivers clock interrupts.
 *
 *  Iteration is bounded so a stuck/spurious source can't pin the cpu
 *  in the claim/complete loop forever (which would also keep sstatus.SIE=0
 *  and silently stop the clock).  Per the PLIC spec the gateway re-forwards
 *  a level-triggered source after each completion as long as the source
 *  line stays high, so a misbehaving device drives this loop indefinitely
 *  unless we break out and disable the source.
 */
int
irq(Ureg *ureg)
{
	Vctl *v;
	uint id;
	int trips;

	m->intr++;
	trips = 64;
	while((id = plic->context[Suctxt].claimcompl) != 0){
		if(id < Ngintr){
			v = vctl[id];
			if(v == nil){
				/*
				 * Spurious: a source asserted with no handler.  If
				 * we just complete and loop, the gateway will re-
				 * forward it forever.  Mask the source so the cpu
				 * can keep running and the report below is visible.
				 */
				plic->prio[id] = 0;
				plic->enabits[Suctxt][id/32] &= ~(1u << (id & 31));
				coherence();
				iprint("plic: spurious irq %ud, masking\n", id);
			} else
				for(; v != nil; v = v->next)
					v->f(ureg, v->a);
		}
		plic->context[Suctxt].claimcompl = id;
		coherence();
		if(--trips <= 0){
			/*
			 * Either a real storm or a handler that isn't actually
			 * de-asserting the source.  Mask this source and bail so
			 * the rest of the system can keep running long enough to
			 * print and reboot.
			 */
			if(id < Ngintr){
				plic->prio[id] = 0;
				plic->enabits[Suctxt][id/32] &= ~(1u << (id & 31));
				coherence();
			}
			iprint("plic: irq %ud storm, masking source\n", id);
			break;
		}
	}
	/*
	 * a handler may have readied a proc that another (idle) cpu should
	 * run; poke idle cpus so they don't wait for their next clock tick.
	 */
	idlewake();
	return 0;
}

void
intrenable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *name)
{
	Vctl *v;

	if(BUSTYPE(tbdf) == BusPCI){
		pciintrenable(tbdf, f, a);
		return;
	}

	if(tbdf != BUSUNKNOWN)
		return;
	if((uint)irq < Firstirq || (uint)irq >= Ngintr)
		return;

	v = mallocz(sizeof(Vctl), 1);
	if(v == nil)
		panic("intrenable: no mem");
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

	plic->prio[irq] = Defprio;
	plic->enabits[Suctxt][irq/32] |= 1u << (irq & 31);
	coherence();
	unlock(&vctllock);
}

void
intrdisable(int, void (*f)(Ureg*, void*), void *a, int tbdf, char*)
{
	if(BUSTYPE(tbdf) == BusPCI){
		pciintrdisable(tbdf, f, a);
		return;
	}
}
