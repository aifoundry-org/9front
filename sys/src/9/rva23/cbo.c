/*
 * cache-block management: whether the Zicbom cbo.* instructions that
 * cachedwbse, cachedwbinvse and cachedinvse use are there.  A board
 * whose harts lack Zicbom provides its own l2init in cbo.c instead.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

/* configure l2 & l3 caches; called by each hart in case of l2 caches */
/* don't bother until we know we need it. */
void
l2init(void)
{
	/*
	 * this kernel targets the RVA23 profile, which mandates Zicbom
	 * (cache-block management: cbo.clean/cbo.flush/cbo.inval) as a
	 * required extension.  so, unlike Geoff's kernel (which supports
	 * older harts that may lack Zicbom and fall back to soc-specific
	 * l2/l3 cache-controller flushes), we can just assume it's present
	 * rather than probe for it at runtime.
	 *
	 * if this kernel is ever ported to a non-RVA23 hart, replace this
	 * with a real probe (e.g. read the "riscv,isa"/"riscv,isa-extensions"
	 * dtb string for "zicbom") before relying on cachedwbse/cachedwbinvse/
	 * /cachedinvse, which silently no-op (coherence() only) when
	 * soc.havecbo is 0.
	 */
	if (m->machno == 0) {
		soc.havecbo = 1;
		print("l2init: assuming Zicbom (RVA23-mandated cache-block ops)\n");
	}
}
