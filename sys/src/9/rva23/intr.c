#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"

/*
 * bootargsinit/socidentify figure out whether the external interrupt controller is
 * a PLIC (K1, default qemu-virt) or an AIA APLIC+IMSIC pair (K3, qemu-virt with aia=).
 * Commit to the matching driver here, once; but there's still a per-interrupt indirection.
 */

static int (*irqfn)(Ureg*);
static void (*enablefn)(int, void (*)(Ureg*, void*), void*, int, char*);
static void (*disablefn)(int, void (*)(Ureg*, void*), void*, int, char*);

/*
 * last resort when neither a PLIC nor an APLIC was identified: boot anyway with
 * external interrupts dead, complaining per device, so the console (with
 * *uartpoll=1) stays usable for diagnosis.
 */
static int
noneirq(Ureg*)
{
	iprint("irq: external interrupt with no interrupt controller\n");
	return 0;
}

static void
noneenable(int irq, void (*)(Ureg*, void*), void*, int, char *name)
{
	print("intrenable: no interrupt controller for %s irq %d\n", name, irq);
}

static void
nonedisable(int, void (*)(Ureg*, void*), void*, int, char*)
{
}

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
	} else {
		print("intrinit: no interrupt controller in DTB; external interrupts are dead\n");
		irqfn = noneirq;
		enablefn = noneenable;
		disablefn = nonedisable;
	}
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
