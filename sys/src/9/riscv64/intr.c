#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"

/*
 * One kernel image boots SoCs with either style of external interrupt
 * controller: a PLIC (K1, default qemu-virt) or an AIA APLIC+IMSIC
 * pair (K3, qemu-virt with aia=).  bootargsinit/socidentify decide
 * which one the machine has; commit to the matching driver here, once,
 * so the per-interrupt path pays only an indirect call.
 */

static int (*irqfn)(Ureg*);
static void (*enablefn)(int, void (*)(Ureg*, void*), void*, int, char*);
static void (*disablefn)(int, void (*)(Ureg*, void*), void*, int, char*);

void
intrinit(void)
{
	if(soc.plic != nil){
		irqfn = plicirq;
		enablefn = plicintrenable;
		disablefn = plicintrdisable;
		plicinit();
	} else if(soc.aplic != nil){
		irqfn = aplicirq;
		enablefn = aplicintrenable;
		disablefn = aplicintrdisable;
		aplicinit();
	} else
		panic("intrinit: no interrupt controller");
}

int
irq(Ureg *ureg)
{
	return irqfn(ureg);
}

void
intrenable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *name)
{
	enablefn(irq, f, a, tbdf, name);
}

void
intrdisable(int irq, void (*f)(Ureg*, void*), void *a, int tbdf, char *name)
{
	disablefn(irq, f, a, tbdf, name);
}
