/*
 * In place of ../rva23/timecmp.s, as the Maxion lacks Sstc: with no
 * stimecmp CSR the timer is set through SBI TIME, and the deadline
 * last set is kept in m->timecmp for rdstimecmp.
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
