/*
 * Spacemit K1 SoC reset.
 *
 * SBI SRST is silently ignored on this board.  The trustworthy reset
 * is a reset request to the P1 power-management chip (bit 1 of its
 * power control register 2, at address 0x41 on TWSI8), which restarts
 * the SoC the same way the reset button does; the boot ROM then finds
 * every block in its power-on state.  We drive the TWSI unit by
 * polling, one byte per TB round, per the I2C chapter of the K1 user
 * manual.
 *
 * If the PMIC can't be reached we fall back to the on-chip watchdog:
 * shortest timeout, let it fire.  Each WDT register write must be
 * preceded by the unlock magic (0xbaba to WDT_WFAR, 0xeb10 to
 * WDT_WSAR).  Bit 2 of MPMU_WDTPCR holds the WDT in reset when set;
 * bit 4 (WDTR) of MPMU_APRR enables the WDT-driven SoC reset.  The
 * WDT reset is not a full power-on reset — peripheral state survives
 * it and the ROM may fail to boot from SD — so the SD hosts are put
 * back to their power-on state first to improve the odds.
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

	APMU_BASE	= 0xD4282800ull,
	APMU_SIZE	= 0x400,
	APMU_SDH0	= 0x54,		/* SDH0 clock/reset control */
	APMU_SDH2	= 0xe0,		/* SDH2 clock/reset control */
	SDH_PORVAL	= 0x100,	/* reset value: block in reset, clocks off, div 1 */

	APBC_BASE	= 0xD4015000ull,
	APBC_SIZE	= 0x100,
	APBC_TWSI8	= 0x20,		/* write-only: bit 2 reset, 1 fnclk, 0 apbclk */

	TWSI8_BASE	= 0xD401D800ull,
	TWSI8_SIZE	= 0x40,
	TWSI_ICR	= 0x00,
	 Iue		= 1<<14,	/* unit enable */
	 Scle		= 1<<13,	/* drive SCL (master) */
	 Tb		= 1<<3,		/* transfer byte */
	 Acknak		= 1<<2,		/* NAK after received byte */
	 Stop		= 1<<1,
	 Start		= 1<<0,
	TWSI_ISR	= 0x04,
	 Irf		= 1<<20,	/* IDBR receive full (W1C) */
	 Ite		= 1<<19,	/* IDBR transmit empty (W1C) */
	TWSI_IDBR	= 0x0c,

	PMIC_ADDR	= 0x41,		/* P1 power-management chip on TWSI8 */
	PMIC_PWRCTRL2	= 0x7e,
	 PMIC_SHUTDOWN	= 1<<2,		/* shutdown request */
	 PMIC_RSTREQ	= 1<<1,		/* reset request */
};

static int
twsiwait(uchar *t, ulong bit)
{
	int i;

	for(i = 0; i < 10000; i++){
		if(*(ulong*)(t + TWSI_ISR) & bit){
			*(ulong*)(t + TWSI_ISR) = bit;
			return 0;
		}
		microdelay(10);
	}
	return -1;
}

static int
twsiput(uchar *t, int b, ulong flags)
{
	*(ulong*)(t + TWSI_IDBR) = b;
	*(ulong*)(t + TWSI_ICR) = Iue|Scle|flags|Tb;
	coherence();
	return twsiwait(t, Ite);
}

static int
twsiget(uchar *t, ulong flags)
{
	*(ulong*)(t + TWSI_ICR) = Iue|Scle|flags|Tb;
	coherence();
	if(twsiwait(t, Irf) < 0)
		return -1;
	return *(ulong*)(t + TWSI_IDBR) & 0xFF;
}

static uchar*
k1twsi(void)
{
	static uchar *t;
	uchar *apbc;

	if(t != nil)
		return t;
	apbc = vmap(APBC_BASE, APBC_SIZE);
	t = vmap(TWSI8_BASE, TWSI8_SIZE);
	if(apbc == nil || t == nil){
		t = nil;
		return nil;
	}

	/* the clock register is write-only: pulse reset, leave clocks on */
	*(ulong*)(apbc + APBC_TWSI8) = 0x7;
	coherence();
	delay(1);
	*(ulong*)(apbc + APBC_TWSI8) = 0x3;
	coherence();
	delay(1);
	vunmap(apbc, APBC_SIZE);

	*(ulong*)(t + TWSI_ICR) = Iue|Scle;
	coherence();
	return t;
}

int
k1pmicread(int reg)
{
	uchar *t;

	t = k1twsi();
	if(t == nil)
		return -1;
	if(twsiput(t, PMIC_ADDR<<1, Start) < 0)
		return -1;
	if(twsiput(t, reg, 0) < 0)
		return -1;
	if(twsiput(t, PMIC_ADDR<<1|1, Start) < 0)
		return -1;
	return twsiget(t, Stop|Acknak);
}

int
k1pmicwrite(int reg, int val)
{
	uchar *t;

	t = k1twsi();
	if(t == nil)
		return -1;
	if(twsiput(t, PMIC_ADDR<<1, Start) < 0)
		return -1;
	if(twsiput(t, reg, 0) < 0)
		return -1;
	return twsiput(t, val, Stop);
}

static void
k1pmicreset(void)
{
	int v;

	v = k1pmicread(PMIC_PWRCTRL2);
	if(v < 0)
		return;
	k1pmicwrite(PMIC_PWRCTRL2, v | PMIC_RSTREQ);
	delay(1000);
}

static void
k1poweroff(void)
{
	int v;

	v = k1pmicread(PMIC_PWRCTRL2);
	if(v < 0)
		return;
	k1pmicwrite(PMIC_PWRCTRL2, v | PMIC_SHUTDOWN);
	delay(1000);
}

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
	uchar *wdt, *mpmu, *apmu;
	ulong v;

	k1pmicreset();
	print("pmic reset failed; trying wdt\n");

	wdt = vmap(WDT_BASE, WDT_SIZE);
	mpmu = vmap(MPMU_BASE, MPMU_SIZE);
	if (wdt == nil || mpmu == nil)
		return;

	/*
	 * the WDT reset is not a full power-on reset: the SD hosts keep
	 * whatever state the kernel left, and the boot ROM then fails to
	 * load the SPL from a card that boots fine from power-on.  Put
	 * the SDH clock/reset controls back to their power-on values so
	 * the ROM finds the world it expects.
	 */
	apmu = vmap(APMU_BASE, APMU_SIZE);
	if (apmu != nil){
		*(ulong*)(apmu + APMU_SDH0) = SDH_PORVAL;
		*(ulong*)(apmu + APMU_SDH2) = SDH_PORVAL;
		coherence();
	}

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
void (*rvpoweroff)(void) = k1poweroff;
