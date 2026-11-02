/*
 * Spacemit K3 on-die temperature sensor (TSEN_LITE block)
 *
 * Exports #P/cputemp as "name value±resolution" lines, one per enabled sensor.
 * K3 UM 16.4 gives the block seven measurement points -- top, vpu, gpu and
 * one per cluster -- sampled to 12 bits, but documents no registers; the
 * layout below is what the block answers to at the address the DTB gives.
 *
 * The driver structure follows k1thermal.c, the K1 temperature sensor.
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
	ApbcTsen	= 0x6C/4,
	 ApbcBusEn	= 1<<0,		/* confirmed: CCU_GATE_DEFINE(tsen_bus_clk, ...) */
	 ApbcFuncEn	= 1<<1,		/* confirmed: CCU_GATE_DEFINE(tsen_clk, ...) */
	 ApbcReset	= 1<<2,		/* UNCONFIRMED: K1 convention, not independently checked for K3 */

	/* confirmed: k3-pico-itx.dtb, thermal@d4018000 */
	TsenPhys	= 0xD4018000ull,
	TsenSize	= 0x100,

	TsCfg		= 0x00/4,
	 CfgCkdivMask	= 3<<2,		/* clock divider select */
	 CfgCkdivDiv4	= 0<<2,		/* /4 -- the only divider this driver sets */
	 CfgVrefMask	= 0xFF<<7,	/* vref trim field */
	 CfgAutozero	= 1<<25,	/* pulse: enable auto-zero calibration */
	 CfgOutsel	= 1<<26,	/* pulse: alternate output select */
	 CfgTsenEn	= 1<<0,		/* power up / enable the sensor block */
	TsCfg2		= 0x04/4,		/* write sensor index here to mux it in */
	TsIntenb	= 0x08/4,
	TsIntst		= 0x0C/4,
	TsIntclr	= 0x10/4,
	TsRebootThresh	= 0x14/4,
	TsTempData	= 0x18/4,
	 TempDataMask	= 0xFFF,	/* 12-bit sample, K3 UM 16.4.2 */
	TsVrefEfuse	= 0x1C/4,
	TsTempThresh	= 0x20/4,
	TsEcoIntvl	= 0x24/4,
	TsLpFixBypass	= 0x28/4,

	VrefCalibDefault = 0xD8,	/* fallback vref trim; real value lives in efuse */
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

static u32int *tsen;
static QLock tsenlk;	/* the mux is a shared, select-then-read resource */

/* a register write has to be ordered against what follows it */
static void
wreg(u32int *base, uintptr reg, u32int v)
{
	base[reg] = v;
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
	wreg(tsen, TsCfg2, id);
	delay(1);
	v = tsen[TsTempData] & TempDataMask;
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
	u32int *apbc;
	u32int v;

	apbc = vmap(ApbcPhys, ApbcSize);
	tsen = vmap(TsenPhys, TsenSize);
	if(apbc == nil || tsen == nil){
		tsen = nil;
		return;
	}

	/* enable both clocks (bus+func), release reset */
	v = apbc[ApbcTsen];
	wreg(apbc, ApbcTsen, (v & ~ApbcReset) | ApbcBusEn | ApbcFuncEn);
	vunmap(apbc, ApbcSize);

	wreg(tsen, TsIntclr, ~0);
	wreg(tsen, TsIntenb, ~0);		/* mask all interrupts */

	v = tsen[TsCfg];
	v &= ~CfgCkdivMask;
	v |= CfgCkdivDiv4;
	v &= ~CfgVrefMask;
	v |= VrefCalibDefault<<7;
	wreg(tsen, TsCfg, v);

	/* one-shot auto-zero calibration pulse */
	v = tsen[TsCfg];
	wreg(tsen, TsCfg, v | CfgAutozero | CfgOutsel);
	delay(1);
	v = tsen[TsCfg];
	wreg(tsen, TsCfg, v & ~(CfgAutozero | CfgOutsel));

	wreg(tsen, TsCfg, tsen[TsCfg] | CfgTsenEn);

	addarchfile("cputemp", 0444, cputempread, nil);
}

void
k3thermalinit(void)
{
	if(!dtbsocmatch("spacemit,k3"))
		return;
	thermalinit();
}
