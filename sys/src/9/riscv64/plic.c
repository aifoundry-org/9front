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

	/*
	 * sources this driver touches (prio/enable writes at init and the
	 * spurious/storm masking).  221 is the bound the K1 and qemu-virt
	 * PLICs have been run with; Ngintr is larger now for the K3 APLIC,
	 * and poking a PLIC's registers beyond what the silicon decodes
	 * is not worth the risk.
	 */
	Nplicsrc	= 221,

	/* ET-SoC-1: shared priorities, but only hart 0 S-mode is ours. */
	Maxionctxt	= 0,
	Maxionnsrc	= 42,		/* source IDs 1..41 */
	Maxionuart	= 15,
	Maxionplicsize	= 0x02000000,
};

extern uintptr setsie(uintptr);

static Lock vctllock;
static Vctl *vctl[Nplicsrc];
static Plic *plic;
static uint plicctxt = Suctxt;
static uint plicnsrc = Nplicsrc;

static void
plicmask(uint id)
{
	/* Shared Maxion priorities must survive spurious/storm masking. */
	if(!maxion)
		plic->prio[id] = 0;
	plic->enabits[plicctxt][id/32] &= ~(1u << (id & 31));
}

void
plicinit(void)
{
	uint i;
	uintptr base, size;

	/*
	 * soc.plic was set by socidentify() to the SoC's PLIC physical base.
	 * Keep the physical address separate from its kernel mapping.
	 */
	base = (uintptr)soc.plic;
	if(base == 0)
		panic("intrinit: soc.plic not set");
	size = PLICSIZE;
	if(maxion){
		if(m->hartid != 0 || m->machno != 0 || conf.nmach != 1)
			panic("plicinit: Maxion requires hart 0 only");
		if(base != 0x10000000)
			panic("plicinit: unexpected Maxion PLIC base %#p", base);
		plicctxt = Maxionctxt;
		plicnsrc = Maxionnsrc;
		size = Maxionplicsize;
	}
	plic = (Plic*)vmap(base, size);
	if(plic == nil)
		panic("intrinit: plic vmap");
	soc.plic = (uchar*)plic;

	/*
	 * Mask only our context.  On Maxion preserve all shared priorities,
	 * including UART1 until it is explicitly enabled, and never visit
	 * another hart, SP/MM context, or a source above 41.
	 */
	for(i = Firstirq; i < plicnsrc; i++)
		plicmask(i);
	plic->context[plicctxt].priothresh = 0;
	coherence();
	if(maxion)
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
	while((id = plic->context[plicctxt].claimcompl) != 0){
		if(maxion && id >= plicnsrc){
			/*
			 * Do not index or complete an invalid source.  Fail
			 * closed using only implemented enables in our context.
			 */
			for(i = Firstirq; i < plicnsrc; i++)
				plicmask(i);
			coherence();
			iprint("plic: invalid Maxion irq %ud, masking S0 enables\n", id);
			break;
		}
		if(id < plicnsrc){
			v = vctl[id];
			if(maxion && id != Maxionuart)
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
		}
		plic->context[plicctxt].claimcompl = id;
		coherence();
		if(--trips <= 0){
			/*
			 * Either a real storm or a handler that isn't actually
			 * de-asserting the source.  Mask this source and bail so
			 * the rest of the system can keep running long enough to
			 * print and reboot.
			 */
			if(id < plicnsrc){
				plicmask(id);
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
plicintrenable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *name)
{
	Vctl *v;

	/* Reject before the PCI path: Maxion mailbox space is not ECAM. */
	if(maxion && (tbdf != BUSUNKNOWN || irq != Maxionuart)){
		print("plic: refusing Maxion irq %d tbdf %#ux for %s\n", irq, tbdf, name);
		return;
	}

	if(BUSTYPE(tbdf) == BusPCI){
		pciintrenable(tbdf, f, a);
		return;
	}

	if(tbdf != BUSUNKNOWN)
		return;
	if((uint)irq < Firstirq || (uint)irq >= plicnsrc)
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
	plic->enabits[plicctxt][irq/32] |= 1u << (irq & 31);
	coherence();
	unlock(&vctllock);
}

void
plicintrdisable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char*)
{
	Vctl *v, **l;
	Mpl pl;

	if(maxion){
		if(tbdf != BUSUNKNOWN || irq != Maxionuart)
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
		return;
	}
	if(BUSTYPE(tbdf) == BusPCI){
		pciintrdisable(tbdf, f, a);
		return;
	}
}
