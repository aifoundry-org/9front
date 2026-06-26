/*
 * Spacemit K1 SoC reset via the on-chip watchdog.
 *
 * SBI SRST is silently ignored on this board, so we self-reset by
 * configuring the WDT for the shortest possible timeout and letting it
 * fire.  Each WDT register write must be preceded by the unlock magic
 * (0xbaba to WDT_WFAR, 0xeb10 to WDT_WSAR).  Bit 2 of MPMU_WDTPCR holds
 * the WDT in reset when set, so we must clear it before touching the
 * WDT registers.  Bit 4 (WDTR) of MPMU_APRR enables the WDT-driven
 * SoC reset; it is normally already set by u-boot.
 *
 * Installed via rvreset, which exit() calls after SBI SRST returns.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

enum {
	WDT_BASE	= 0xD4080000ull,
	WDT_SIZE	= 0x100,
	WDT_WFAR	= 0xb0,	/* write-access magic 1: 0xbaba */
	WDT_WSAR	= 0xb4,	/* write-access magic 2: 0xeb10 */
	WDT_WMER	= 0xb8,	/* match enable */
	WDT_WMR		= 0xbc,	/* match value */
	WDT_WSR		= 0xc0,	/* status */
	WDT_WCR		= 0xc8,	/* counter reset */

	MPMU_BASE	= 0xD4050000ull,
	MPMU_SIZE	= 0x2000,
	MPMU_WDTPCR	= 0x200,	/* WDT power/clock/reset gate */
	MPMU_APRR	= 0x1020,	/* APRR_WDTR (bit 4) enables reset */
};

static void
k1wdtwrite(uchar *wdt, int off, ulong val)
{
	*(ulong*)(wdt + WDT_WFAR) = 0xbaba;
	*(ulong*)(wdt + WDT_WSAR) = 0xeb10;
	*(ulong*)(wdt + off) = val;
	coherence();
}

static void
k1reset(void)
{
	uchar *wdt, *mpmu;
	ulong v;

	wdt = vmap(WDT_BASE, WDT_SIZE);
	mpmu = vmap(MPMU_BASE, MPMU_SIZE);
	if (wdt == nil || mpmu == nil)
		return;

	/* clock-enable + reset-deassert (bits 0-1 set, bit 2 clear) */
	v = *(ulong*)(mpmu + MPMU_WDTPCR);
	*(ulong*)(mpmu + MPMU_WDTPCR) = (v | 0x3) & ~0x4;
	coherence();

	k1wdtwrite(wdt, WDT_WSR,  0x0);
	k1wdtwrite(wdt, WDT_WMR,  0x1);	/* shortest timeout */
	k1wdtwrite(wdt, WDT_WMER, 0x3);	/* enable counter + reset */
	k1wdtwrite(wdt, WDT_WCR,  0x1);	/* clear/restart counter */

	*(ulong*)(mpmu + MPMU_APRR) |= (1u<<4);
	coherence();

	delay(5000);
}

void (*rvreset)(void) = k1reset;
