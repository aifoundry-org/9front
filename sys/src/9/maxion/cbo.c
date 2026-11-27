/*
 * In place of ../rva23/cbo.c, as the Maxion lacks Zicbom: soc.havecbo
 * stays 0, so cachedwbse, cachedwbinvse and cachedinvse only order
 * earlier stores.  That does not make noncoherent DMA safe.
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
	if(m->machno == 0)
		print("l2init: no Zicbom; fences only, not DMA-safe\n");
}
