/*
 * Microchip PolarFire SoC (BeagleV-Fire and kin)
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "mpfsfns.h"

/*
 * DDR the HSS maps non-cached at 0xc0000000: the DTB's
 * non-cached-low-buffer, kept out of the RAM banks by its
 * alloc-ranges.  The U54s have no cache-block operations, so
 * device DMA goes through here.
 */
enum {
	Ncdmabase	= 0xc4000000ULL,
	Ncdmasize	= 64*MiB,
};

static uintptr ncnext = Ncdmabase;

/*
 * Zeroed memory from the non-cached DMA window, for device rings
 * and bounce buffers; never freed.
 */
void*
ncalloc(usize n, usize align)
{
	uintptr p;

	p = ROUND(ncnext, align);
	if(p + n > Ncdmabase + Ncdmasize)
		panic("ncalloc: out of non-cached memory");
	ncnext = p + n;
	memset((void*)p, 0, n);
	return (void*)p;
}

Socmod socmpfs = {
	"mpfs",
	"microchip,mpfs",
};
