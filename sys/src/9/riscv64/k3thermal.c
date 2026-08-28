/*
 * Spacemit K3 on-die temperature sensor (TSEN_LITE block)
 *
 * Exports #P/cputemp as "name value±resolution" lines, one per enabled sensor
 *
 * Based on:
 *   9front k1thermal.c (k1 temperature sensor) - overall 9front driver architecture
 *   Linux K3 BSP driver - k3-thermal.c/.h register + descriptor layout reference
 *   k3.dtsi - Device tree source include / Common k3 pico itx & k3 com260
 *
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"

enum {
	/* confirmed: k3-pico-itx.dtb, system-controller@d4015000 */
	ApbcPhys	= 0xD4015000ull,
	ApbcSize	= 0x1000,
	/* confirmed: SpacemiT k3-syscon.h, APBC_TSEN_CLK_RST */
	ApbcTsen	= 0x6C,
	 ApbcBusEn	= 1<<0,		/* confirmed: CCU_GATE_DEFINE(tsen_bus_clk, ...) */
	 ApbcFuncEn	= 1<<1,		/* confirmed: CCU_GATE_DEFINE(tsen_clk, ...) */
	 ApbcReset	= 1<<2,		/* UNCONFIRMED: K1 convention, not independently checked for K3 */

	/* confirmed: k3-pico-itx.dtb, thermal@d4018000 */
	TsenPhys	= 0xD4018000ull,
	TsenSize	= 0x100,

	/* -- below: from the K3 BSP driver's register facts -- */
	TsCfg		= 0x00,
	 CfgCkdivMask	= 3<<2,		/* clock divider select */
	 CfgCkdivDiv4	= 0<<2,		/* /4 -- the only divider this driver sets */
	 CfgVrefMask	= 0xFF<<7,	/* vref trim field */
	 CfgAutozero	= 1<<25,	/* pulse: enable auto-zero calibration */
	 CfgOutsel	= 1<<26,	/* pulse: alternate output select */
	 CfgTsenEn	= 1<<0,		/* power up / enable the sensor block */
	TsCfg2		= 0x04,		/* write sensor index here to mux it in */
	TsIntenb	= 0x08,
	TsIntst		= 0x0C,
	TsIntclr	= 0x10,
	TsRebootThresh	= 0x14,
	TsTempData	= 0x18,
	 TempDataMask	= 0xFFF,	/* 12-bit sample, K3 UM 16.4.2 */
	TsVrefEfuse	= 0x1C,
	TsTempThresh	= 0x20,
	TsEcoIntvl	= 0x24,
	TsLpFixBypass	= 0x28,

	VrefCalibDefault = 0x9A,	/* fallback vref trim; real value lives in efuse */
	TempRawDiv	= 8,		/* raw code is in 1/8 degC units */
	Tempoff		= 274,		/* confirmed: k3-pico-itx.dtb temperature_offset=0x112 */

	NMux		= 8,		/* confirmed: sensor_range = <0 7> */
	/* confirmed: k3-pico-itx.dtb thermal-zones "thermal-sensors" cells */
	STop		= 0,
	SPkg		= 1,		/* confirmed disabled on this board (tsensor_map[1]=0) */
	SVpu		= 2,
	SGpu		= 3,
	SCluster0	= 4,
	SCluster1	= 5,
	SCluster2	= 6,
	SCluster3	= 7,
};

/* nil entries are muxable-but-unwired positions (confirmed via tsensor_map) */
static char *sensorname[NMux] = {
	[STop]		"top",
	[SPkg]		nil,
	[SVpu]		"vpu",
	[SGpu]		"gpu",
	[SCluster0]	"cluster0",
	[SCluster1]	"cluster1",
	[SCluster2]	"cluster2",
	[SCluster3]	"cluster3",
};

static uchar *tsen;
static QLock tsenlk;	/* the mux is a shared, select-then-read resource */

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

/* select sensor id on the mux, settle, and return its raw 12-bit code */
static int
tempcode(int id)
{
	ulong v;

	if(tsen == nil)
		return -1;
	qlock(&tsenlk);
	wr(tsen, TsCfg2, id);
	delay(1);
	v = rd(tsen, TsTempData) & TempDataMask;
	qunlock(&tsenlk);
	return v;
}

static long
cputempread(Chan*, void *a, long n, vlong off)
{
	char str[256], *p, *e;
	int i, code, degc;

	p = str;
	e = str + sizeof str;
	for(i = 0; i < NMux; i++){
		if(sensorname[i] == nil)
			continue;
		code = tempcode(i);
		if(code < 0)
			error(Eio);
		degc = code/TempRawDiv - Tempoff;
		p = seprint(p, e, "%s %d±1\n", sensorname[i], degc);
	}
	return readstr(off, a, n, str);
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

	/* enable both clocks (bus+func), release reset */
	v = rd(apbc, ApbcTsen);
	wr(apbc, ApbcTsen, (v & ~ApbcReset) | ApbcBusEn | ApbcFuncEn);
	vunmap(apbc, ApbcSize);

	wr(tsen, TsIntclr, ~0);
	wr(tsen, TsIntenb, ~0);		/* mask all interrupts */

	v = rd(tsen, TsCfg);
	v &= ~CfgCkdivMask;
	v |= CfgCkdivDiv4;
	v &= ~CfgVrefMask;
	v |= VrefCalibDefault<<7;
	wr(tsen, TsCfg, v);

	/* one-shot auto-zero calibration pulse */
	v = rd(tsen, TsCfg);
	wr(tsen, TsCfg, v | CfgAutozero | CfgOutsel);
	delay(1);
	v = rd(tsen, TsCfg);
	wr(tsen, TsCfg, v & ~(CfgAutozero | CfgOutsel));

	wr(tsen, TsCfg, rd(tsen, TsCfg) | CfgTsenEn);

	addarchfile("cputemp", 0444, cputempread, nil);
}

void
k3thermalinit(void)
{
	if(!dtbsocmatch("spacemit,k3"))
		return;
	thermalinit();
}
