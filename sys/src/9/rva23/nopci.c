/*
 * For kernels with no PCI bus: plic.c and aplic.c hand
 * interrupts with a PCI tbdf to these.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
pciintrenable(int tbdf, void (*)(Ureg*, void*), void*)
{
	print("pciintrenable: no PCI, tbdf %#ux\n", tbdf);
}

void
pciintrdisable(int, void (*)(Ureg*, void*), void*)
{
}
