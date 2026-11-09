/*
 * Maxion has no Sstc, so there is no stimecmp CSR: these stand in for
 * ../rva23/timecmp.s and program the timer through SBI TIME.  The
 * deadline last set is kept in m->timecmp, which is what rdstimecmp
 * returns; timerset publishes the same value there for idlewake.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
wrstimecmp(u64int t)
{
	m->timecmp = t;
	sbisettimer(t);
}

u64int
rdstimecmp(void)
{
	return m->timecmp;
}
