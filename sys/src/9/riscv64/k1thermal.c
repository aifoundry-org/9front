/*
 * Spacemit K1 on-die temperature sensor and fan speed control.
 *
 * The two CPU cluster temperatures are exported as #P/cputemp in the
 * usual "value±resolution" form that stats(1) reads from /dev/cputemp.
 * Fan control is expressed as percentage in #P/fanpwm; the fan stalls
 * at less than half though, so a value < 50 turns the fan off.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "k1.h"

enum {
	ApbcTsen	= 0x6C,		/* bit0 bus clk, bit1 core clk, bit2 reset */

	/* thermal sensor block */
	TsenPhys	= 0xD4018000ull,
	TsenSize	= 0x100,
	TsPctrl		= 0x00,
	 PcEnable	= 1<<0,
	 PcTempmode	= 1<<3,
	 PcRawsel	= 1<<7,
	 PcCtune	= 0xF<<8,
	 PcSwctrl	= 0xF<<18,
	 PcAutomode	= 1<<23,
	TsEn		= 0x08,
	 EnAll		= 0x1F,		/* five sensors */
	TsTime		= 0x0C,
	 TimeMask	= 0xFFFFFF,
	 TimeSet	= 0xF | 0xF0 | (3<<20),	/* ref cnt, adc rst, filter */
	TsInten		= 0x14,
	TsData		= 0x20,		/* + (id/2)*4; two 16-bit codes per word */
	Tempoff		= 278,		/* code - 278 = deg C (K1 thermal chapter) */

	Cluster0	= 3,		/* the two CPU-cluster channels */
	Cluster1	= 4,
	/* also available: soc, package, gpu */

	/* RCPU2 syscon: fan PWM clock/reset gate */
	RcpuPhys	= 0xC0888000ull,
	RcpuSize	= 0x400,	/* covers gate (+0x08) and PWM (+0x300) */
	RcpuPwm2	= 0x08,		/* bit0 reset-release, bit1 clk, bit2 reset */

	/* fan PWM block (RCPU PWM2), PXA layout */
	PwmOff		= 0x300,	/* from RcpuPhys */
	Pwmcr		= 0x00,
	 PwmcrSd	= 1<<6,		/* stop counter when done */
	Pwmdcr		= 0x04,		/* duty; bit10 = full duty (disabled here) */
	Pwmpcr		= 0x08,		/* period value */
	Pwmpv		= 1023,		/* 1024-count period */
	Pwmfull		= 1023,		/* cap duty below the full-duty bit */
	Fanmin		= 51,		/* below this the fan only stalls; treat as off */
	Fandefault	= 70,		/* percent duty at boot: reliably starts, still quiet */

	/* pin mux: GPIO_79 -> rcpu_pwm2 (mode 2) */
	AibPhys		= 0xD401E000ull,
	AibSize		= 0x280,
	Gpio79		= 80,		/* PINID(79); word index into the block */
	Gpio79Conf	= 0xC442,	/* EDGE_NONE|PULL_UP|PAD_3V_DS4|MUX_MODE2 */
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
	v = rd(tsen, TsData + (id/2)*4);
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
	snprint(str, sizeof str, "%d±1\n%d±1\n", t0 - Tempoff, t1 - Tempoff);
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
	/* avoid stalling */
	if(pct < Fanmin)
		dc = 0;
	else {
		dc = (Pwmpv + 1) * pct / 100;
		if(dc > Pwmfull)
			dc = Pwmfull;
	}
	wr(rcpu, PwmOff + Pwmcr, PwmcrSd);
	wr(rcpu, PwmOff + Pwmdcr, dc);
	wr(rcpu, PwmOff + Pwmpcr, Pwmpv);
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

/* ungate and initialize the thermal sensor */
static void
thermalinit(void)
{
	uchar *apbc;
	ulong v;

	apbc = vmap(ApbcPhys, ApbcSize);
	tsen = vmap(TsenPhys, TsenSize);
	if(apbc == nil || tsen == nil){
		tsen = nil;
		return;
	}

	/* enable both clocks (bits 0,1), release reset (clear bit 2) */
	v = rd(apbc, ApbcTsen);
	wr(apbc, ApbcTsen, (v & ~0x4) | 0x3);
	vunmap(apbc, ApbcSize);

	wr(tsen, TsInten, ~0);		/* mask all interrupts */
	v = rd(tsen, TsTime);
	wr(tsen, TsTime, (v & ~TimeMask) | TimeSet);
	v = rd(tsen, TsPctrl);
	v &= ~(PcSwctrl | PcCtune);
	v |= PcRawsel | PcTempmode | PcAutomode | PcEnable;
	wr(tsen, TsPctrl, v);
	wr(tsen, TsEn, rd(tsen, TsEn) | EnAll);

	addarchfile("cputemp", 0444, cputempread, nil);
}

/* ungate the fan PWM, mux its pad, and set a quiet default */
static void
faninit(void)
{
	uchar *aib;
	ulong v;

	rcpu = vmap(RcpuPhys, RcpuSize);
	aib = vmap(AibPhys, AibSize);
	if(rcpu == nil || aib == nil){
		rcpu = nil;
		return;
	}

	/* enable clock (bit1), release reset (bit0 set, bit2 clear) */
	v = rd(rcpu, RcpuPwm2);
	wr(rcpu, RcpuPwm2, (v & ~0x4) | 0x3);

	/* GPIO_79 -> rcpu_pwm2: replace the low config half, keep the rest */
	v = ((ulong*)aib)[Gpio79];
	((ulong*)aib)[Gpio79] = (v & ~0xFFFFul) | Gpio79Conf;
	coherence();
	vunmap(aib, AibSize);

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
