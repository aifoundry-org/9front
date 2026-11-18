/*
 * Microchip PolarFire SoC board hooks: no SpacemiT GPIO, thermal,
 * display or USB to bring up.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
boardinit(void)
{
}

/* no PCI bus yet: plic.c routes PCI interrupts here */
void
pciintrenable(int tbdf, void (*)(Ureg*, void*), void*)
{
	print("pciintrenable: no PCI, tbdf %#ux\n", tbdf);
}

void
pciintrdisable(int, void (*)(Ureg*, void*), void*)
{
}
