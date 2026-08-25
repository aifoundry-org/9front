/*
 * Spacemit K3 on-die temperature sensor (TSEN_LITE block).
 *
 * Exports #P/cputemp as "name value±resolution" lines, one per enabled
 * sensor, in the same shape as the K1 driver's #P/cputemp.
 *
 * PROVENANCE / CONFIDENCE NOTES:
 *
 *   - Register offsets/bitfields/algorithm (TsCfg, TsCfg2, TsTempData,
 *     the code/8 formula, the vref-calibration and auto-zero pulse
 *     sequence) come from a K3 vendor BSP Linux driver (k3-thermal.c/.h,
 *     SPDX GPL-2.0-only); only the hardware *facts* were carried over,
 *     the Plan9 code below is freshly written, not copied.
 *
 *   - Everything below is now cross-checked against the k3-pico-itx
 *     board's actual compiled device tree (k3-pico-itx.dtb), which is
 *     about as strong a source as this gets short of the silicon
 *     itself:
 *
 *       thermal@d4018000 {
 *         compatible = "spacemit,k3-tsensor";
 *         reg = <0x0 0xd4018000 0x0 0x100>;
 *         clocks = <&apbc CLK_APBC_TSEN &apbc CLK_APBC_TSEN_BUS>;
 *         clock-names = "func", "bus";
 *         resets = <&apbc 0x3d>;
 *         sensor_range = <0 7>;
 *         tsensor_map = <1 0 1 1 1 1 1 1>;
 *         temperature_offset = <0x112>;   // 274
 *       };
 *
 *     and, in the same dtb:
 *
 *       system-controller@d4015000 {
 *         compatible = "spacemit,k3-syscon-apbc";
 *         reg = <0x0 0xd4015000 0x0 0x1000>;
 *       };
 *
 *     This settles several things that were previously guessed:
 *       * TsenSize is 0x100, not the 0x800 the UM's address-map table
 *         reserves for it (the reg is just smaller than the reservation).
 *       * ApbcPhys/ApbcSize are now known: 0xD4015000 / 0x1000. ApbcTsen
 *         staying at offset 0x6C inside it (already confirmed from
 *         SpacemiT's k3-syscon.h) lines up with this base exactly.
 *       * Tempoff is 274 here, NOT the 278 the BSP header's
 *         TEMPERATURE_OFFSET falls back to -- this board's device tree
 *         overrides it per-unit via "temperature_offset", presumably
 *         a per-die or per-SKU trim value. Trust the DT value.
 *       * There are 8 physical mux positions (sensor_range = 0..7), not
 *         7 -- the K3 UM's "7 sensors" is this board's count of enabled
 *         ones. tsensor_map disables index 1 here.
 *       * The sensor-to-index mapping was WRONG in the previous version
 *         of this file. The board's thermal-zones nodes tie names to
 *         indices unambiguously: index 0 = top, 1 = (unused/disabled on
 *         this board -- likely a "package" sensor per K1's convention,
 *         where index 1 is also "package"), 2 = vpu, 3 = gpu, 4 =
 *         cluster0, 5 = cluster1, 6 = cluster2, 7 = cluster3. This is
 *         board data, not an SoC-level register default, so a
 *         different K3 board could in principle wire tsensor_map
 *         differently -- but absent a way to read the DT at this layer,
 *         this board's mapping is what's hardcoded below.
 *
 *   - Still not independently confirmed: that bit2 of ApbcTsen is the
 *     reset bit (carried over from K1's convention; the dtb's "resets"
 *     property confirms a reset line exists on this same apbc
 *     controller, cell "0x3d", but doesn't say which bit it maps to).
 *
 *   - Board matching: gated on dtbsocmatch("spacemit,k3") alone, same
 *     as the K3 dwqe Ethernet driver's etherpnp() does -- no soc.vendor
 *     check, no extra header needed. This file is now fully
 *     self-contained: no k3.h, no external Soc/vendor declarations.
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
