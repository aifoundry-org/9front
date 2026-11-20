/*
 * Microchip PolarFire SoC board hooks: no SpacemiT GPIO, thermal,
 * display or USB to bring up.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

static uintptr ncnext = NCDMABASE;

/*
 * Zeroed memory from the non-cached DMA window, for device rings
 * and bounce buffers; never freed.
 */
void*
ncalloc(usize n, usize align)
{
	uintptr p;

	p = ROUND(ncnext, align);
	if(p + n > NCDMABASE + NCDMASIZE)
		panic("ncalloc: out of non-cached memory");
	ncnext = p + n;
	memset((void*)p, 0, n);
	return (void*)p;
}

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
