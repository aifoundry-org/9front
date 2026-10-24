/*
 * Maxion has no Zicbom, so this l2init stands in for ../rva23/cbo.c's
 * and leaves soc.havecbo clear: cachedwbse, cachedwbinvse and
 * cachedinvse then only order earlier stores (coherence()).  That does
 * not clean or invalidate caches, so it does not make noncoherent DMA
 * safe; this port has no DMA devices.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
l2init(void)
{
	soc.havecbo = 0;
	if(m->machno == 0)
		print("l2init: no Zicbom; fences only, not DMA-safe\n");
}
