/*
 * cache-block management: whether the Zicbom cbo.* instructions that
 * cachedwbse, cachedwbinvse and cachedinvse use are there.  Without
 * them those only order earlier stores, which is not enough for
 * noncoherent DMA.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

/* called by each hart */
void
l2init(void)
{
	if(m->machno != 0)
		return;
	soc.havecbo = (dtbisa() & Isazicbom) != 0;
	if(soc.havecbo)
		print("l2init: Zicbom\n");
	else
		print("l2init: no Zicbom; fences only, not DMA-safe\n");
}
