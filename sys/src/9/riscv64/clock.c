#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "ureg.h"
#include "sysreg.h"

/*
 * CLINT tick rate: default is 10 Mhz for qemu; otherwise set from DTB
 * /cpus/timebase-frequency in bootargs.c.  Copied here so that
 * fastticks() works even if called before clockinit runs.
 */
static uvlong freq = 10000000;

extern uvlong rdtime(void);

void
clockinit(void)
{
	if(timebase != 0)
		freq = timebase;
}

uvlong
fastticks(uvlong *hz)
{
	if(hz)
		*hz = freq;
	return rdtime();
}

ulong
µs(void)
{
	uvlong hz;
	uvlong t = fastticks(&hz);
	return (t * 1000000ULL) / hz;
}

void
microdelay(int n)
{
	ulong now;

	now = µs();
	while(µs() - now < n);
}

void
delay(int n)
{
	while(--n >= 0)
		microdelay(1000);
}
