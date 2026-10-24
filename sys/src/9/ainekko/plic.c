/*
 * The ET-SoC-1 PLIC, shared with the service and management
 * processors: one priority array for everyone, sources 1..41, and
 * contexts in the order S0, M0, S1, M1, ...  This kernel owns hart 0's
 * S-mode context (0) and only the console UART's source (15).  It
 * never writes a priority it did not set, another context, or a
 * source above 41.  intr.c does not call it at all with *uartpoll.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"
#include "riscv64.h"
#include "../port/error.h"

enum {
	Plicbase	= 0x10000000,
	Plicsize	= 0x02000000,
	Ctxt		= 0,		/* hart 0, S-mode */
	Nsrc		= 42,		/* source IDs 1..41 */
	Uartirq		= 15,		/* UART1, the console */
	Defprio		= 1,
};

extern uintptr setsie(uintptr);

static Lock vctllock;
static Vctl *vctl[Nsrc];
static Plic *plic;

/* shared priorities must survive spurious/storm masking */
static void
plicmask(uint id)
{
	plic->enabits[Ctxt][id/32] &= ~(1u << (id & 31));
}

void
plicinit(void)
{
	uint i;
	uintptr base;

	/* soc.plic is the PLIC's physical base, from the device tree */
	base = (uintptr)soc.plic;
	if(base == 0)
		panic("intrinit: soc.plic not set");
	if(m->hartid != 0 || m->machno != 0 || conf.nmach != 1)
		panic("plicinit: Maxion requires hart 0 only");
	if(base != Plicbase)
		panic("plicinit: unexpected Maxion PLIC base %#p", base);
	plic = (Plic*)vmap(base, Plicsize);
	if(plic == nil)
		panic("intrinit: plic vmap");
	soc.plic = (uchar*)plic;

	/*
	 * Mask only our context.  Preserve all shared priorities,
	 * including UART1 until it is explicitly enabled, and never visit
	 * another hart, SP/MM context, or a source above 41.
	 */
	for(i = Firstirq; i < Nsrc; i++)
		plicmask(i);
	plic->context[Ctxt].priothresh = 0;
	coherence();
	print("plic: Maxion S0 context 0, UART1 irq 15 only; shared priorities preserved\n");

	/* enable S-mode external interrupts in SIE CSR */
	setsie(Seie);
}

/*
 *  called via intr.c to handle external interrupts from the plic.
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
plicirq(Ureg *ureg)
{
	Vctl *v;
	uint id, i;
	int trips;

	m->intr++;
	trips = 64;
	while((id = plic->context[Ctxt].claimcompl) != 0){
		if(id >= Nsrc){
			/*
			 * Do not index or complete an invalid source.  Fail
			 * closed using only implemented enables in our context.
			 */
			for(i = Firstirq; i < Nsrc; i++)
				plicmask(i);
			coherence();
			iprint("plic: invalid Maxion irq %ud, masking S0 enables\n", id);
			break;
		}
		v = vctl[id];
		if(id != Uartirq)
			v = nil;
		if(v == nil){
			/*
			 * Spurious: a source asserted with no handler.  If
			 * we just complete and loop, the gateway will re-
			 * forward it forever.  Mask the source so the cpu
			 * can keep running and the report below is visible.
			 */
			plicmask(id);
			coherence();
			iprint("plic: spurious irq %ud, masking\n", id);
		} else
			for(; v != nil; v = v->next)
				v->f(ureg, v->a);
		plic->context[Ctxt].claimcompl = id;
		coherence();
		if(--trips <= 0){
			/*
			 * Either a real storm or a handler that isn't actually
			 * de-asserting the source.  Mask this source and bail so
			 * the rest of the system can keep running long enough to
			 * print and reboot.
			 */
			plicmask(id);
			coherence();
			iprint("plic: irq %ud storm, masking source\n", id);
			break;
		}
	}
	idlewake();
	return 0;
}

void
plicintrenable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *name)
{
	Vctl *v;

	/* there is no PCI here: the QEMU ECAM address is a Maxion mailbox */
	if(tbdf != BUSUNKNOWN || irq != Uartirq){
		print("plic: refusing Maxion irq %d tbdf %#ux for %s\n", irq, tbdf, name);
		return;
	}

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
	plic->enabits[Ctxt][irq/32] |= 1u << (irq & 31);
	coherence();
	unlock(&vctllock);
}

void
plicintrdisable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char*)
{
	Vctl *v, **l;
	Mpl pl;

	if(tbdf != BUSUNKNOWN || irq != Uartirq)
		return;
	/* UP: exclude dispatch while unlinking and freeing a handler. */
	pl = splhi();
	lock(&vctllock);
	for(l = &vctl[irq]; (v = *l) != nil; l = &v->next)
		if(v->f == f && v->a == a){
			*l = v->next;
			break;
		}
	if(vctl[irq] == nil){
		plicmask(irq);
		coherence();
	}
	unlock(&vctllock);
	free(v);
	splx(pl);
}
