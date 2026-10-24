/*
 * ET-SoC-1 Maxion external interrupts, replacing ../riscv64/intr.c:
 * the shared PLIC (plic.c) is the only controller.  With *uartpoll the
 * kernel leaves it untouched, runs with external interrupts off and
 * polls the console, the safe default while the service and management
 * processors share the PLIC.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"
#include "riscv64.h"

static int (*irqfn)(Ureg*);
static void (*enablefn)(int, void (*)(Ureg*, void*), void*, int, char*);
static void (*disablefn)(int, void (*)(Ureg*, void*), void*, int, char*);

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
	if(getconf("*uartpoll") != nil){
		clrsie(Seie);
		irqfn = noneirq;
		enablefn = noneenable;
		disablefn = nonedisable;
		print("maxion: polling console; external controller untouched\n");
	}else if(soc.plic != nil){
		irqfn = plicirq;
		enablefn = plicintrenable;
		disablefn = plicintrdisable;
		plicinit();
	}else{
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
