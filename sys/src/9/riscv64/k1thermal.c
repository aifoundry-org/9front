/*
 * Spacemit K1 on-die temperature sensor and fan speed control.
 *
 * The K1 has a five-channel thermal sensor block at 0xD4018000 (soc,
 * package, gpu, cluster0, cluster1).  Its two gating clocks and reset
 * live in one APBC register at 0xD401506C: bit 0 is the bus clock, bit
 * 1 the core clock, bit 2 holds the block in reset while set.  Once
 * ungated we run the sensor in hardware auto mode and read each
 * channel's raw 16-bit code from the DATA registers; the temperature in
 * degrees Celsius is the code minus a fixed 278 offset (K1 thermal
 * chapter).  The two cluster channels are exported as #P/cputemp in the
 * usual "value±resolution" form that stats(1) reads from /dev/cputemp.
 *
 * The board fan is driven by RCPU PWM2 at 0xC0888300 (PWMCR/PWMDCR/
 * PWMPCR at +0/+4/+8, the PXA PWM layout).  Its clock and reset gate is
 * at 0xC0888008 (bit 1 clock, bit 0 releases reset, bit 2 asserts it);
 * the RCPU island is powered at reset so no power switch is needed.
 * The PWM output is pad GPIO_79, which must be muxed to function mode 2.
 * We program a fixed ~1024-count period and set the duty from a
 * percentage written to #P/fanpwm; the "disable full-duty" quirk means
 * the duty is capped one below the full-duty bit rather than using it.
 * Below about half duty the fan only stalls instead of turning, so that
 * whole range is treated as off.  u-boot leaves the fan running flat
 * out, so we quiet it to a safe default at boot that still spins up.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "k1.h"

enum {
	APBC_TSEN	= 0x6C,		/* bit0 bus clk, bit1 core clk, bit2 reset */

	/* thermal sensor block */
	TSEN_BASE	= 0xD4018000ull,
	TSEN_SIZE	= 0x100,
	TS_PCTRL	= 0x00,
	 PC_ENABLE	= 1<<0,
	 PC_TEMPMODE	= 1<<3,
	 PC_RAWSEL	= 1<<7,
	 PC_CTUNE	= 0xF<<8,
	 PC_SWCTRL	= 0xF<<18,
	 PC_AUTOMODE	= 1<<23,
	TS_EN		= 0x08,
	 EN_ALL		= 0x1F,		/* five sensors */
	TS_TIME		= 0x0C,
	 TIME_MASK	= 0xFFFFFF,
	 TIME_SET	= 0xF | 0xF0 | (3<<20),	/* ref cnt, adc rst, filter */
	TS_INTEN	= 0x14,
	TS_DATA		= 0x20,		/* + (id/2)*4; two 16-bit codes per word */
	TEMPOFF		= 278,		/* code - 278 = degrees Celsius */

	Cluster0	= 3,		/* the two CPU-cluster channels */
	Cluster1	= 4,

	/* RCPU2 syscon: fan PWM clock/reset gate */
	RCPU_BASE	= 0xC0888000ull,
	RCPU_SIZE	= 0x400,	/* covers gate (+0x08) and PWM (+0x300) */
	RCPU_PWM2	= 0x08,		/* bit0 reset-release, bit1 clk, bit2 reset */

	/* fan PWM block (RCPU PWM2), PXA layout */
	PWM_OFF		= 0x300,	/* from RCPU_BASE */
	PWMCR		= 0x00,
	 PWMCR_SD	= 1<<6,		/* stop counter when done */
	PWMDCR		= 0x04,		/* duty; bit10 = full duty (disabled here) */
	PWMPCR		= 0x08,		/* period value */
	PWMPV		= 1023,		/* 1024-count period */
	PWMFULL		= 1023,		/* cap duty below the full-duty bit */
	Fanmin		= 51,		/* below this the fan only stalls; treat as off */
	Fandefault	= 70,		/* percent duty at boot: reliably starts, still quiet */

	/* pin mux: GPIO_79 -> rcpu_pwm2 (mode 2) */
	AIB_BASE	= 0xD401E000ull,
	AIB_SIZE	= 0x280,
	GPIO79		= 80,		/* PINID(79); word index into the block */
	GPIO79_CONF	= 0xC442,	/* EDGE_NONE|PULL_UP|PAD_3V_DS4|MUX_MODE2 */
};

static uchar *tsen;
static uchar *rcpu;
static int fanpct = -1;

static ulong
rd(uchar *b, int off)
{
	return *(ulong*)(b + off);
}

static void
wr(uchar *b, int off, ulong v)
{
	*(ulong*)(b + off) = v;
	coherence();
}

/* raw code for sensor id, or -1 if the block isn't mapped */
static int
tempcode(int id)
{
	ulong v;

	if(tsen == nil)
		return -1;
	v = rd(tsen, TS_DATA + (id/2)*4);
	if(id & 1)
		v >>= 16;
	return v & 0xFFFF;
}

static long
cputempread(Chan*, void *a, long n, vlong off)
{
	char str[64];
	int t0, t1;

	t0 = tempcode(Cluster0);
	t1 = tempcode(Cluster1);
	if(t0 < 0 || t1 < 0)
		error(Eio);
	snprint(str, sizeof str, "%d±1\n%d±1\n", t0 - TEMPOFF, t1 - TEMPOFF);
	return readstr(off, a, n, str);
}

static void
setfan(int pct)
{
	int dc;

	if(rcpu == nil)
		return;
	if(pct < 0)
		pct = 0;
	if(pct > 100)
		pct = 100;
	/*
	 * the whole low range only makes the fan stall rather than turn,
	 * so anything below Fanmin means off.
	 */
	if(pct < Fanmin)
		dc = 0;
	else {
		dc = (PWMPV + 1) * pct / 100;
		if(dc > PWMFULL)
			dc = PWMFULL;
	}
	wr(rcpu, PWM_OFF + PWMCR, PWMCR_SD);
	wr(rcpu, PWM_OFF + PWMDCR, dc);
	wr(rcpu, PWM_OFF + PWMPCR, PWMPV);
	fanpct = pct;
}

static long
fanpwmread(Chan*, void *a, long n, vlong off)
{
	char str[16];

	snprint(str, sizeof str, "%d\n", fanpct);
	return readstr(off, a, n, str);
}

static long
fanpwmwrite(Chan*, void *a, long n, vlong)
{
	char buf[16];
	int m;

	m = n;
	if(m > sizeof buf - 1)
		m = sizeof buf - 1;
	memmove(buf, a, m);
	buf[m] = 0;
	setfan(atoi(buf));
	return n;
}

/* ungate and initialise the thermal sensor */
static void
thermalinit(void)
{
	uchar *apbc;
	ulong v;

	apbc = vmap(ApbcPhys, ApbcSize);
	tsen = vmap(TSEN_BASE, TSEN_SIZE);
	if(apbc == nil || tsen == nil){
		tsen = nil;
		return;
	}

	/* enable both clocks (bits 0,1), release reset (clear bit 2) */
	v = rd(apbc, APBC_TSEN);
	wr(apbc, APBC_TSEN, (v & ~0x4) | 0x3);
	vunmap(apbc, ApbcSize);

	wr(tsen, TS_INTEN, ~0);		/* mask all interrupts */
	v = rd(tsen, TS_TIME);
	wr(tsen, TS_TIME, (v & ~TIME_MASK) | TIME_SET);
	v = rd(tsen, TS_PCTRL);
	v &= ~(PC_SWCTRL | PC_CTUNE);
	v |= PC_RAWSEL | PC_TEMPMODE | PC_AUTOMODE | PC_ENABLE;
	wr(tsen, TS_PCTRL, v);
	wr(tsen, TS_EN, rd(tsen, TS_EN) | EN_ALL);

	addarchfile("cputemp", 0444, cputempread, nil);
}

/* ungate the fan PWM, mux its pad, and set a quiet default */
static void
faninit(void)
{
	uchar *aib;
	ulong v;

	rcpu = vmap(RCPU_BASE, RCPU_SIZE);
	aib = vmap(AIB_BASE, AIB_SIZE);
	if(rcpu == nil || aib == nil){
		rcpu = nil;
		return;
	}

	/* enable clock (bit1), release reset (bit0 set, bit2 clear) */
	v = rd(rcpu, RCPU_PWM2);
	wr(rcpu, RCPU_PWM2, (v & ~0x4) | 0x3);

	/* GPIO_79 -> rcpu_pwm2: replace the low config half, keep the rest */
	v = ((ulong*)aib)[GPIO79];
	((ulong*)aib)[GPIO79] = (v & ~0xFFFFul) | GPIO79_CONF;
	coherence();
	vunmap(aib, AIB_SIZE);

	setfan(Fandefault);
	addarchfile("fanpwm", 0664, fanpwmread, fanpwmwrite);
}

void
k1thermalinit(void)
{
	/*
	 * mvendorid 0x710 covers every SpacemiT part including the K3,
	 * whose APBC/AIB live elsewhere; only the DTB names the SoC
	 * precisely enough to touch the addresses above.
	 */
	if(soc.vendor != SpacemiT || !dtbsocmatch("spacemit,k1"))
		return;
	thermalinit();
	faninit();
}
