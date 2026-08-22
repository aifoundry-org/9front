/*
 * Spacemit K1 reset
 *
 * SBI SRST is silently ignored. The trustworthy reset is a reset request to the
 * P1 power-management chip (bit 1 of its power control register 2, at address
 * 0x41 on TWSI8), which restarts the SoC the same way the reset button does;
 * the boot ROM then finds every block in its power-on state. We drive the TWSI
 * by polling, one byte per TB round, per the I2C chapter of the K1 user manual.
 *
 * If the PMIC can't be reached, fall back to the on-chip watchdog: shortest
 * timeout, let it fire. Each WDT register write must be preceded by the unlock
 * magic (0xbaba to WdtWfar, 0xeb10 to WdtWsar). Bit 2 of MpmuWdtpcr holds
 * the WDT in reset when set; bit 4 (WDTR) of MpmuAprr enables the WDT reset.
 * TODO maybe remove the fallback if we never need it?
 *
 * Installed via rvreset, which exit() calls after SBI SRST returns.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k1.h"

static int
twsiwait(uchar *t, ulong bit)
{
	int i;

	for(i = 0; i < 10000; i++){
		if(*(ulong*)(t + TwsiIsr) & bit){
			*(ulong*)(t + TwsiIsr) = bit;
			return 0;
		}
		microdelay(10);
	}
	return -1;
}

static int
twsiput(uchar *t, int b, ulong flags)
{
	*(ulong*)(t + TwsiIdbr) = b;
	*(ulong*)(t + TwsiIcr) = Iue|Scle|flags|Tb;
	coherence();
	return twsiwait(t, Ite);
}

static int
twsiget(uchar *t, ulong flags)
{
	*(ulong*)(t + TwsiIcr) = Iue|Scle|flags|Tb;
	coherence();
	if(twsiwait(t, Irf) < 0)
		return -1;
	return *(ulong*)(t + TwsiIdbr) & 0xFF;
}

static uchar*
k1twsi(void)
{
	static uchar *t;
	uchar *apbc;

	if(t != nil)
		return t;
	apbc = vmap(ApbcPhys, ApbcSize);
	t = vmap(Twsi8Phys, Twsi8Size);
	if(apbc == nil || t == nil){
		t = nil;
		return nil;
	}

	/* the clock register is write-only: pulse reset, leave clocks on */
	*(ulong*)(apbc + ApbcTwsi8) = 0x7;
	coherence();
	delay(1);
	*(ulong*)(apbc + ApbcTwsi8) = 0x3;
	coherence();
	delay(1);
	vunmap(apbc, ApbcSize);

	*(ulong*)(t + TwsiIcr) = Iue|Scle;
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
	if(twsiput(t, PmicAddr<<1, Start) < 0)
		return -1;
	if(twsiput(t, reg, 0) < 0)
		return -1;
	if(twsiput(t, PmicAddr<<1|1, Start) < 0)
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
	if(twsiput(t, PmicAddr<<1, Start) < 0)
		return -1;
	if(twsiput(t, reg, 0) < 0)
		return -1;
	return twsiput(t, val, Stop);
}

static void
k1pmicreset(void)
{
	int v;

	v = k1pmicread(PmicPwrctrl2);
	if(v < 0)
		return;
	k1pmicwrite(PmicPwrctrl2, v | PmicRstreq);
	delay(1000);
}

static void
k1poweroff(void)
{
	int v;

	if(!dtbsocmatch("spacemit,k1"))
		return;
	v = k1pmicread(PmicPwrctrl2);
	if(v < 0)
		return;
	k1pmicwrite(PmicPwrctrl2, v | PmicShutdown);
	delay(1000);
}

static void
k1wdtwrite(uchar *wdt, int off, ulong val)
{
	*(ulong*)(wdt + WdtWfar) = 0xbaba;
	*(ulong*)(wdt + WdtWsar) = 0xeb10;
	*(ulong*)(wdt + off) = val;
	coherence();
}

static void
k1reset(void)
{
	uchar *wdt, *mpmu, *apmu;
	ulong v;

	/*
	 * every register this file touches (PMIC I2C, WDT, MPMU, APMU) sits at a
	 * K1 address; on any other chip, fall through to exit()'s remaining SBI attempts.
	 */
	if(!dtbsocmatch("spacemit,k1"))
		return;
	k1pmicreset();
	print("pmic reset failed; trying wdt\n");

	wdt = vmap(WdtPhys, WdtSize);
	mpmu = vmap(MpmuPhys, MpmuSize);
	if (wdt == nil || mpmu == nil)
		return;

	/*
	 * WDT reset is not a full power-on reset. Put the SDH clock/reset
	 * controls back to their power-on values so the ROM sees what it expects
	 * and might be able to load the SPL.
	 */
	apmu = vmap(ApmuPhys, ApmuSize);
	if (apmu != nil){
		*(ulong*)(apmu + ApmuSdh0) = SdhPorval;
		*(ulong*)(apmu + ApmuSdh2) = SdhPorval;
		coherence();
	}

	/* clock-enable + reset-deassert (bits 0-1 set, bit 2 clear) */
	v = *(ulong*)(mpmu + MpmuWdtpcr);
	*(ulong*)(mpmu + MpmuWdtpcr) = (v | 0x3) & ~0x4;
	coherence();

	k1wdtwrite(wdt, WdtWsr,  0x0);
	k1wdtwrite(wdt, WdtWmr,  0x1);	/* shortest timeout */
	k1wdtwrite(wdt, WdtWmer, 0x3);	/* enable counter + reset */
	k1wdtwrite(wdt, WdtWcr,  0x1);	/* clear/restart counter */

	*(ulong*)(mpmu + MpmuAprr) |= (1u<<4);
	coherence();

	delay(5000);
}

void (*rvreset)(void) = k1reset;
void (*rvpoweroff)(void) = k1poweroff;
