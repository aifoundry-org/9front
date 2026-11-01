/*
 * screenpicoitx.c -- Spacemit K3 Pico-ITX: DP1 (USB-C DisplayPort
 * Alt-Mode) display driver. Board-specific, not just SoC-specific --
 * named after the board, matching this tree's usbxhcipicoitx.c/
 * usbxhcibpif3.c/usbxhcicom260.c convention. Covers power domain,
 * Type-C orientation switch, FUSB301 CC-logic init, DSI4LN2 clock/
 * reset tree, DP1 controller init (PLLs/analog calibration),
 * framebuffer reservation, DPU1 mode-set, MSA, AUX channel, and real
 * DisplayPort link training.
 *
 * CONFIRMED WORKING on real hardware: DP1 shows a clean, stable
 * 1920x1080@60 image (verified live, September 2026).
 *
 * For the debugging history, design decisions and register-source
 * provenance behind this file (why each register/bit is what it is,
 * what was tried and reverted, and what a diff against a live
 * working Linux boot turned up), see k3-9front-display-investigation.md.
 * This file's own comments cover only what each part does, its
 * register source, and any non-obvious fact needed to not re-break it.
 *
 * Consolidates what were originally several separate files into one,
 * matching every other 9front architecture's own display driver
 * (a single screen.c per port). Section headers below note each
 * part's original filename for that history.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "../port/i2c.h"

/*
 * ADDENDUM: local prototypes for every function in this file that's
 * static -- i.e. everything except dpuk3() (fns.h has that one, and
 * screeninit(), the only two things anything outside this file ever
 * calls). Needed because dpuk3() (defined near the top, on purpose,
 * as this file's own single documented entry point) calls several of
 * these before their real definitions appear further down the file.
 */
static void dpuk3pwron(void);
static void dpuk3typecforce(int reverse);
static void dpuk3fusbtest(void);
static void dpuk3init(void);
static void dpuk3edppxclkmux(int dpu1);
static void dpuk3devinit(void);
static void dpuk3dp(void);
static int dpuk3linktrain(void);
static int dpuk3auxread(ulong dpcdaddr, uchar *buf, int len);
static int dpuk3auxwrite(ulong dpcdaddr, uchar *buf, int len);
static void dpuk3mode(uvlong addr, ulong stride);

/*
 * All enum blocks for this file, grouped here (matching the
 * convention used elsewhere in this tree, e.g. etherdwqe.c) instead
 * of scattered one per section. Each group is labelled with the
 * function(s) that use it; the detailed rationale for each register
 * and bit (confirmed sources, STATUS notes, etc.) stays as a comment
 * next to that function, further down this file.
 */

/* Power domain (dpuk3pwron) -- LCD1/DP1 power-domain APMU registers */
enum {
	/* confirmed: same block as dpuk3.c's ApmuPhysPw, k3-pico-itx.dtb
	 * system-controller@d4282800. Sized larger than dpuk3.c's copy
	 * (0x400) because CIU_LCD_CONFIG_REG_BASE is at offset 0x52C. */
	ApmuPhysPw	= 0xD4282800ull,
	ApmuSizePw	= 0x1000,

	/* one-time setup, done regardless of which domain(s) get powered */
	PmuaPwrBlkTmr	= 0x00DC,
	CiuLcdConfig	= 0x052C,
	 CiuLcdEdpMuxEn	= 1<<8,

	/* LCD1 (DPU1/DP1) domain descriptor -- k3_pm_domain_desc[K3_PMU_LCD1_PWR_DOMAIN] */
	Lcd1PwrCtrl	= 0x03F4,
	 Lcd1AutoPwrOn	= 1<<0,
	 Lcd1HwMode	= 1<<4,

	ApmuPowerStatus	= 0x00F0,
	 Lcd1PwrStat	= 1<<5,		/* "domain requested on" bit */
	 Lcd1HwPwrStat	= 1<<15,	/* "hw sequencer says actually on" bit */

	PwrOnDelayUs	= 310,		/* matches vendor driver's udelay(310) */
	PollTries	= 10000,
	PollDelayUs	= 6,
};

/* Type-C orientation override (dpuk3typecforce) -- APMU Type-C control register */
enum {
	/* confirmed: same block as dpuk3.c's ApmuPhysTy */
	ApmuPhysTy	= 0xD4282800ull,
	ApmuSizeTy	= 0x200,	/* must cover offset 0x110 */

	PmuaTypecCtrl	= 0x110,
	 TypecOrientFlip	= 1<<2,
	 TypecOrientOvrdEn	= 1<<3,
	 TypecOrientOvrd	= 1<<4,
};

/* FUSB301 USB-C CC-logic chip (dpuk3fusbtest) -- i2c1 device registers/bits */
enum {
	FusbAddr	= 0x25,

	RegDeviceId	= 0x01,
	RegModes	= 0x02,
	 ModesDrpAcc	= 1<<5,
	 ModesMask	= 0x3F,
	RegControl	= 0x03,
	 CtrlTgl35ms	= 0<<4,		/* bits[5:4] */
	 CtrlTglMask	= 3<<4,
	 CtrlCur1500ma	= 2<<1,		/* bits[2:1] */
	 CtrlCurMask	= 3<<1,
	 CtrlIntMask	= 1<<0,		/* 1=interrupt masked/disabled */
	RegManual	= 0x04,
	 ManualErrRecovery = 1<<0,
	 ManualMask	= 0xF,
	RegReset	= 0x05,
	 ResetSwRes	= 1<<0,

	RegStatus	= 0x11,
	 StatusOrientShift = 4,		/* bits[5:4] */
	 StatusOrientMask  = 3<<4,
	 StatusVbusOk	= 1<<3,
	 StatusAttach	= 1<<0,
	RegType		= 0x12,
	 TypeSnk	= 1<<4,
	 TypeSrc	= 1<<3,
	 TypePwrAcc	= 1<<2,
	 TypeDbgAcc	= 1<<1,
	 TypeAudAcc	= 1<<0,
};

/* DPU1/DP1 clock & reset tree (dpuk3init/dpuk3edppxclkmux) -- APMU clock-gate/reset registers */
enum {
	/* confirmed: k3-pico-itx.dtb, system-controller@d4282800 */
	ApmuPhysCk	= 0xD4282800ull,
	ApmuSizeCk	= 0x400,

	/* --- plain variant (CTRL1/CTRL2/CTRL5 bit16/15) -- likely DPU0/
	 * eDP0's, not DP1's; kept anyway, harmless. See doc. --- */
	ApmuLcdClkResCtrl1	= 0x44,
	 LcdSwRst	= 1<<4,
	 LcdHclkEn	= 1<<5,
	 LcdPxclkFc	= 1<<30,
	 LcdMclkFc	= 1<<29,

	ApmuLcdClkResCtrl2	= 0x4C,
	 LcdPxclkEn	= 1<<16,
	 LcdMclkEn	= 1<<0,
	 LcdMclkRst	= 1<<9,
	 LcdDscclkRst	= 1<<15,

	ApmuLcdClkResCtrl5	= 0x274,
	 DpuAclkEn	= 1<<16,	/* plain variant gate */
	 DpuAclkRst	= 1<<15,	/* plain variant reset */
	 DpuAclkFc	= 1<<31,	/* plain variant FC trigger, same reg */

	/* --- DSI4LN2 variant (CTRL3/CTRL4, CTRL5 bit0/1) -- what dpu1_crtc0
	 * (our actual DP1 pipeline) really uses, confirmed against its own
	 * DTB clock/reset IDs. This was the real bug behind DP1 staying
	 * dark for a long time -- see doc. --- */
	ApmuLcdClkResCtrl3	= 0x26C,
	 Dsi4ln2PxclkFc	= 1<<30,	/* FC trigger for dsi4ln2 pxclk, lives in CTRL3 */
	 Dsi4ln2MclkFc	= 1<<29,	/* FC trigger for dsi4ln2 mclk, lives in CTRL3 */
	 Dsi4ln2LcdSwRst	= 1<<4,	/* RESET_APMU_DSI4LN2_LCD_SW: 1=release */
	 Dsi4ln2EscclkRst = 1<<3,	/* RESET_DSI4LN2_ESCCLK: 1=release. Matches
					 * a working Linux's live regmap dump;
					 * escclk plausibly feeds the AUX
					 * channel's ~1MHz Manchester signaling. */
	 Dsi4ln2EscclkEn = 1<<2,	/* dsi4ln2_dsi_esc_clk gate, ccu-k3.c
					 * CCU_MUX_GATE_DEFINE(...,CTRL3,0,1,BIT(2),0) */

	ApmuLcdClkResCtrl4	= 0x270,
	 Dsi4ln2PxclkEn	= 1<<16,	/* gate only -- div/mux untouched, see STATUS */
	 Dsi4ln2PxclkMux_SHIFT = 21,	/* bits[23:21], 3 bits, 8 parents */
	 Dsi4ln2PxclkDiv_SHIFT = 17,	/* bits[19:17], 3 bits, value+1=divide */
	 Dsi4ln2PxclkMuxPll2d4 = 5,	/* index 5 in dsi4ln2_lcdpx_parents[]:
					 * {pll1_d4_614p4, pll1_d5_491p52, pll7_d5,
					 * pll6_d6, pll2_d7, pll2_d4, pll1_d48_51p2_ap,
					 * pll2_d8} -- confirmed ccu-k3.c */
	 Dsi4ln2PxclkDivBy5 = 4,	/* register value 4 -> divide by 5
					 * (val+1 convention, confirmed via
					 * divider_recalc_rate() with no
					 * special flags) */
	 Dsi4ln2MclkEn	= 1<<0,		/* gate only -- div/mux untouched, see STATUS */
	 Dsi4ln2MclkRst	= 1<<9,		/* RESET_APMU_DSI4LN2_LCD_MCLK: 1=release */
	 Dsi4ln2DscclkRst = 1<<15,	/* RESET_APMU_DSI4LN2_LCD_DSCCLK: 1=release */

	 Dsi4ln2AclkEn	= 1<<1,		/* dsi4ln2_dpu_aclk gate, CTRL5 bit1 (NOT bit16!) */
	 Dsi4ln2AclkRst	= 1<<0,		/* RESET_APMU_DSI4LN2_DPU_ACLK, CTRL5 bit0 (NOT bit15!) */
	 Dsi4ln2AclkFc	= 1<<30,	/* dsi4ln2_dpu_aclk FC trigger, same reg (CTRL5) */

	/* confirmed: K3 UM 17.4.2.4, cross-checked ccu-k3.c APMU_LCD_EDP_CTRL --
	 * separate small register, no plain/DSI4LN2 ambiguity here */
	ApmuLcdEdpCtrl	= 0x23C,
	 Edp0Prstn	= 1<<0,
	 Edp0PclkEn	= 1<<1,
	 Edp0PxclkMuxBit = 1<<2,	/* ADDENDUM: re-asserted every
					 * encoder_enable() in Linux, see
					 * dpuk3edppxclkmux() */
	 Edp1Prstn	= 1<<16,
	 Edp1PclkEn	= 1<<17,
	 Edp1PxclkMuxBit = 1<<18,	/* ADDENDUM: re-asserted every
					 * encoder_enable() in Linux, see
					 * dpuk3edppxclkmux() -- same bit
					 * dpuk3init() already sets once
					 * at boot */
};

/* DP1 controller init (dpuk3devinit) -- PLL lock and analog TX calibration registers */
enum {
	DpPhysRegDv		= 0xCAC88000ull,
	DpSizeRegDv		= 0x4000,

	/* 0x0018: framing/scramble/link-train config bits, shares the
	 * register with SCALE_DOWN_MODE/FORCE_HPD seen elsewhere */
	DpMiscCtrlRegDv	= 0x0018,
	 ScramblerDisableBitDv = 1<<31,
	 FastLinkTrainEnBit = 1<<29,
	 EnhanceFramingEnBit = 1<<30,

	/* 0x001c: controller/phy/hdcp/aux/video resets -- same register
	 * dpuk3dp.c already maps as DpResetCtrlRegDv, repeated here so this
	 * file is self-contained */
	DpResetCtrlRegDv	= 0x001C,
	 ControllerResetBitDv = 1<<31,
	 PhyResetBitDv	= 1<<30,
	 HdcpResetBitDv	= 1<<29,
	 AudioResetBit	= 1<<28,	/* released, as part of AllResetsMaskDv
					 * (tried leaving it asserted per a
					 * Linux diff; made corruption worse,
					 * reverted -- see doc) */
	 AuxResetBitDv	= 1<<27,
	 VideoResetMaskRegDv	= 0xF,
	 AllResetsMaskDv	= ControllerResetBitDv|PhyResetBitDv|HdcpResetBitDv|AudioResetBit|AuxResetBitDv|VideoResetMaskRegDv,

	DpAuxIntStaRegDv	= 0x0080,
	 AuxReplyEventDv	= 1<<16,

	/* interrupt mask registers -- confirmed against spacemit_inno_dp.h */
	AuxReplyMsk	= 0x0084,	AuxReplyMskBit = 1<<16,
	HpdIntMsk	= 0x0084,	HpdIntMskBit = 1<<17,
	HdcpIntMsk	= 0x0084,	HdcpIntMskBit = 1<<15,
	IllegalAuxMsk	= 0x0084,	IllegalAuxMskBit = 1<<14,
	TypeCEventMsk	= 0x0084,	TypeCEventMskBit = 1<<13,
	DscEventMsk	= 0x0084,	DscEventMskBit = 1<<12,
	SdpS3Msk	= 0x0084,	SdpS3MskBit = 1<<11,
	SdpS2Msk	= 0x0084,	SdpS2MskBit = 1<<10,
	SdpS1Msk	= 0x0084,	SdpS1MskBit = 1<<9,
	SdpS0Msk	= 0x0084,	SdpS0MskBit = 1<<8,
	FifoS3Msk	= 0x0084,	FifoS3MskBit = 1<<7,
	FifoS2Msk	= 0x0084,	FifoS2MskBit = 1<<6,
	FifoS1Msk	= 0x0084,	FifoS1MskBit = 1<<5,
	FifoS0Msk	= 0x0084,	FifoS0MskBit = 1<<4,
	SinkIrqMsk	= 0x008C,	SinkIrqMskBit = 1<<31,
	HotPlugMsk	= 0x008C,	HotPlugMskBit = 1<<29,
	HotUnplugMsk	= 0x008C,	HotUnplugMskBit = 1<<28,
	SinkUnplugErrMsk = 0x008C,	SinkUnplugErrMskBit = 1<<27,

	DpVideoCtrlRegDv	= 0x0200,
	 VideoStreamEnDv	= 1<<28,

	DpPhyifCtrlRegDv	= 0x0100,
	 PhyBusyBypBit	= 1<<31,
	 XmitEnableShiftDv = 17,
	 NumLanesShiftDv	= 5,
	 PhyRateShiftDv	= 0,

	/* pixel/core PLL: same offsets/fields as dpuk3phy.c, repeated
	 * here so this file's sequence is self-contained and matches
	 * the real order (reset pulse happens between power-down and
	 * power-up, not before both) */
	PrePll0RegDv		= 0x0190,
	 PllPdDv		= 1<<0,
	 PllOutputNormalDv = 1<<1,
	 PllDsmPdDv	= 1<<4,
	 PllDacPdDv	= 1<<5,
	 PllPrediv_SHIFTDv = 8,
	 PllFbdivHi_SHIFTDv = 16,
	 PllFbdivLo_SHIFTDv = 24,
	 PixelPllLockDv	= 1<<7,
	 PllDownspreadBitDv = 1<<20,	/* undocumented for this reg, but
					 * confirmed identical to CorePll0RegDv's
					 * ANA_MPLL_DOWNSPREAD via a live-Linux
					 * regmap diff */
	 PllDisableSscgBitDv = 1<<21,	/* same story, ANA_MPLL_DISABLE_SSCG
					 * ("0: enable SSC; 1: disable SSC") */
	PrePll1RegDv		= 0x0194,
	 PllDivm_SHIFTDv	= 16,
	 PllDivaux_SHIFTDv = 8,
	PrePll2RegDv		= 0x0198,
	 PllDiv5EnDv	= 1<<31,
	 PllDivp_SHIFTDv	= 24,
	 PrePllLockBypenDv = 1<<17,
	 PrePllDpEnBitDv	= 1<<8,		/* SOC_DPTX_ANA_PREPLL_DP_EN --
					 * "DP mode enable" for the pixel PLL */
	PrePll3RegDv		= 0x019C,

	CorePll0RegDv	= 0x0180,
	 MpllPdDv		= 1<<0,
	 MpllDsmPdDv	= 1<<4,
	 MpllDacPdDv	= 1<<5,
	 MpllPrediv_SHIFTDv = 8,
	 MpllFbdivHi_SHIFTDv = 16,
	 MpllFbdivLo_SHIFTDv = 24,
	 CorePllLockDv	= 1<<7,
	 MpllDownspreadBitDv = 1<<20,	/* ANA_MPLL_DOWNSPREAD, see PLL
					 * pixel-side twin above */
	 MpllDisableSscgBitDv = 1<<21,	/* ANA_MPLL_DISABLE_SSCG, see above */
	CorePll1RegDv	= 0x0184,
	CorePll2RegDv	= 0x0188,
	 MpllVcoclkDiv8EnDv = 1<<16,
	 MpllPostdivEnDv	= 1<<11,
	 MpllPostdiv_SHIFTDv = 8,
	CorePll3RegDv	= 0x01A0,
	 MpllClkdiv16m_SHIFTDv = 8,

	/* pixel PLL: 148.5MHz from 24MHz ref -- unchanged from dpuk3phy.c */
	PllPredivDv	= 1,
	PllFbdivDv	= 99,
	PllDacPdValDv	= 1,
	PllDsmPdValDv	= 1,
	PllDiv5EnValDv	= 0,
	PllDivmDv		= 0,
	PllDivauxDv	= 8,
	PllDivpDv		= 1,
	/* core PLL: HBR (2.7Gbps) from 24MHz ref -- unchanged from dpuk3phy.c */
	MpllPredivDv	= 2,
	MpllFbdivDv	= 225,
	MpllDacPdValDv	= 1,
	MpllDsmPdValDv	= 1,
	MpllPostdivRegDv	= 0,
	MpllPostdivEnValDv = 1,
	MpllVcoclkDiv8EnValDv = 1,
	MpllClkdiv16mDv	= 21,

	/* analog TX calibration block -- see file header, all new */
	AnaRtcal	= 0x01C0,
	 RtcalBgSel_SHIFT = 25,
	 RtcalFreqdivL_SHIFT = 16,
	 RtcalBypassBit	= 1<<15,
	 RtcalFreqdivH_SHIFT = 8,
	AnaTxRtm	= 0x01C4,
	 TxRtmD3_SHIFT = 24, TxRtmD2_SHIFT = 16, TxRtmD1_SHIFT = 8, TxRtmD0_SHIFT = 0,
	AnaTxIselDrvHi	= 0x01A4,	/* D3,D2 */
	 TxIselD3_SHIFT = 28, TxIselD2_SHIFT = 24,
	AnaTxMainselIselLo = 0x01A8,	/* mainsel D2,D3 + isel D1,D0 */
	 TxMainselD2_SHIFT = 24, TxMainselD3_SHIFT = 16,
	 TxIselD1_SHIFT = 4, TxIselD0_SHIFT = 0,
	AnaTxPostselMainsel = 0x01AC,	/* postsel D1,D0,D3,D2 + mainsel D0,D1 */
	 TxPostselD1_SHIFT = 28, TxPostselD0_SHIFT = 24,
	 TxPostselD3_SHIFT = 20, TxPostselD2_SHIFT = 16,
	 TxMainselD0_SHIFT = 8, TxMainselD1_SHIFT = 0,
	AnaTxModePresel	= 0x01B0,	/* mode D3..D0 + presel D1,D0,D3,D2 */
	 TxModeD3_SHIFT = 27, TxModeD2_SHIFT = 26, TxModeD1_SHIFT = 25, TxModeD0_SHIFT = 24,
	 TxPreselD1_SHIFT = 12, TxPreselD0_SHIFT = 8,
	 TxPreselD3_SHIFT = 4, TxPreselD2_SHIFT = 0,
	AnaTxAuxRxVsel	= 0x01D0,
	 AuxRxVsel_SHIFT = 12,

	/* per-lane analog enables + bandgap reference. Linux's own
	 * inno_dp_shutdown()->soc_dp_hw_disable() turns these off but
	 * nothing in soc_dp_dev_init() re-enables them (a genuine cold
	 * boot never sees them off) -- this driver's kexec path runs
	 * right after Linux's shutdown() already turned them off, so
	 * they need an explicit re-enable below. See doc. */
	AnaTxEnLdo	= 0x01A4,
	 EnLdoD3_SHIFT = 11, EnLdoD2_SHIFT = 10, EnLdoD1_SHIFT = 9, EnLdoD0_SHIFT = 8,
	 EnDrvD3_SHIFT = 3, EnDrvD2_SHIFT = 2, EnDrvD1_SHIFT = 1, EnDrvD0_SHIFT = 0,
	AnaTxEnP2s	= 0x01B0,	/* shares register with AnaTxModePresel */
	 EnP2sD3_SHIFT = 31, EnP2sD2_SHIFT = 30, EnP2sD1_SHIFT = 29, EnP2sD0_SHIFT = 28,
	AnaBgEn		= 0x01BC,
	 BgEn_SHIFT	= 2,
	AnaBgEnRcal_SHIFT = 27,		/* lives in AnaRtcal (0x01C0), same
					 * register already defined above */
	AnaTxPostselPreMode = 0x01DC,	/* postsel_pre D3..D0 + mode_de D3..D0 + mode_pre D3..D0 */
	 TxPostselPreD3_SHIFT = 20, TxPostselPreD2_SHIFT = 16,
	 TxPostselPreD1_SHIFT = 12, TxPostselPreD0_SHIFT = 8,
	 TxModeDeD3_SHIFT = 7, TxModeDeD2_SHIFT = 6, TxModeDeD1_SHIFT = 5, TxModeDeD0_SHIFT = 4,
	 TxModePreD3_SHIFT = 3, TxModePreD2_SHIFT = 2, TxModePreD1_SHIFT = 1, TxModePreD0_SHIFT = 0,

	/* Linux's own literal calibration constants -- see STATUS */
	MIsel		= 0x5,
	MMainsel	= 0x19,
	MPre		= 0x0,
	MPost		= 0x2,
	TxMode		= 0x1,
	TxPre		= 0x0,
	ClkDiv		= 24*1000/100,	/* 240 */
};

/* DP1 controller MSA + video stream enable (dpuk3dp) -- Main Stream Attributes registers */
enum {
	/* confirmed: k3-pico-itx.dtb, dp1@cac88000 */
	DpPhysRegDm		= 0xCAC88000ull,
	DpSizeRegDm		= 0x4000,

	/* 0x001c: controller/phy/hdcp/aux/video resets, all in one
	 * register. OPPOSITE polarity from every other reset in this
	 * codebase: here 0=normal(released), 1=reset(asserted). Always
	 * pulsed (assert, wait, deassert, wait) rather than trusting
	 * inherited state, matching spacemit_inno_dp.c. */
	DpResetCtrlRegDm	= 0x001C,
	 ControllerResetBitDm = 1<<31,
	 PhyResetBitDm	= 1<<30,
	 HdcpResetBitDm	= 1<<29,
	 AuxResetBitDm	= 1<<27,
	 VideoResetMaskRegDm	= 0xF,		/* bits[3:0], up to 4 streams */
	 AllResetsMaskDm	= ControllerResetBitDm|PhyResetBitDm|HdcpResetBitDm|AuxResetBitDm|VideoResetMaskRegDm,

	/* confirmed: SOC_DPTX_AUX_REPLY_EVENT_INT_STA, same as dpuk3aux.c */
	DpAuxIntStaRegDm	= 0x0080,
	 AuxReplyEventDm	= 1<<16,

	DpVideoCtrlRegDm	= 0x0200,
	 VideoMappingShift = 22,	/* bits[26:22] */
	 VideoMappingMask  = 0x1F<<22,
	  MapRgb8bit	= 1,		/* confirmed: SOC_DPTX_VIDEO_MAPPING enum, "1: RGB 8bits" */
	 VideoStreamEnDm	= 1<<28,

	/* HPD: a real physical/timing-dependent signal outside this
	 * driver's control -- checked, force-asserted if not seen. */
	DpHpdStatus	= 0x0088,
	 HpdInStatus	= 1<<26,
	DpHpdForce	= 0x0018,
	 ForceHpdBit	= 1<<27,

	DpPolarity	= 0x020C,
	 VsyncPolarity	= 1<<28,
	 HsyncPolarity	= 1<<29,

	/* PHY analog power state + per-lane transmitter enable */
	DpPhyifCtrlRegDm	= 0x0100,
	 PhyPowerdownMask = 0xF<<13,	/* bits[16:13]: 0x0=powered on, 0xC=powered down */
	 XmitEnableMask	= 0xF<<17,	/* bits[20:17]: one bit per lane */
	 NumLanesMask	= 0x3<<5,	/* bits[6:5] */
	 PhyRateMask	= 0x3<<0,	/* bits[1:0] */

	DpVactiveVblank	= 0x0210,	/* vactive bits[31:16], vblank bits[15:0] */
	DpHblankHactive	= 0x0214,	/* hblank bits[29:16], hactive bits[15:0] */
	DpFrontHsync	= 0x021C,	/* h_front_porch bits[31:16], h_sync_width bits[15:0] */
	DpFrontVsync	= 0x0220,	/* v_front_porch bits[31:16], v_sync_width bits[15:0] */
	DpStartxy	= 0x0224,	/* hstart bits[31:16], vstart bits[15:0] */
	DpMisc0		= 0x0228,	/* bits[7:0] */
	 Misc0Rgb8bit	= 0x20,		/* confirmed: spacemit_inno_dp.c soc_dp_hw_set_msa_and_enable_video() format table */
	DpNvidMisc1	= 0x022C,	/* nvid bits[31:8], misc1 bits[7:0] */

	/* 1920x1080@60 -- must match dpuk3mode.c exactly */
	ModeHactive	= 1920,
	ModeVactive	= 1080,
	ModeHfp		= 88,
	ModeHsync	= 44,
	ModeHbp		= 148,
	ModeVfp		= 4,
	ModeVsync	= 5,
	ModeVbp		= 36,
	ModeHtotal	= ModeHactive+ModeHfp+ModeHsync+ModeHbp,	/* 2200 */
	ModeVtotal	= ModeVactive+ModeVfp+ModeVsync+ModeVbp,	/* 1125 */

	/* soc_dp_encoder_enable() does a SECOND phy_power_off()->
	 * reconfigure->phy_power_on() cycle at mode-enable time, on top
	 * of what soc_dp_dev_init() did once at bind time (see
	 * dpuk3phycycle() below) -- this is that cycle's own copy of the
	 * same PLL/lane/rate constants dpuk3devinit.c already establishes
	 * (148.5MHz pixel, HBR 2.7Gbps core). */
	PrePll0RegDm		= 0x0190,
	 PllPdDm		= 1<<0,
	 PllOutputNormalDm = 1<<1,
	 PllDsmPdDm	= 1<<4,
	 PllDacPdDm	= 1<<5,
	 PllPrediv_SHIFTDm = 8,
	 PllFbdivHi_SHIFTDm = 16,
	 PllFbdivLo_SHIFTDm = 24,
	 PixelPllLockDm	= 1<<7,
	 PllDownspreadBitDm = 1<<20,	/* see dpuk3devinit.c's PllDownspreadBitDv */
	 PllDisableSscgBitDm = 1<<21,
	PrePll1RegDm		= 0x0194,
	 PllDivm_SHIFTDm	= 16,
	 PllDivaux_SHIFTDm = 8,
	PrePll2RegDm		= 0x0198,
	 PllDiv5EnDm	= 1<<31,
	 PllDivp_SHIFTDm	= 24,
	 PrePllLockBypenDm = 1<<17,
	 PrePllDpEnBitDm	= 1<<8,		/* SOC_DPTX_ANA_PREPLL_DP_EN */
	PrePll3RegDm		= 0x019C,

	CorePll0RegDm	= 0x0180,
	 MpllPdDm		= 1<<0,
	 MpllDsmPdDm	= 1<<4,
	 MpllDacPdDm	= 1<<5,
	 MpllPrediv_SHIFTDm = 8,
	 MpllFbdivHi_SHIFTDm = 16,
	 MpllFbdivLo_SHIFTDm = 24,
	 CorePllLockDm	= 1<<7,
	 MpllDownspreadBitDm = 1<<20,
	 MpllDisableSscgBitDm = 1<<21,
	CorePll1RegDm	= 0x0184,
	CorePll2RegDm	= 0x0188,
	 MpllVcoclkDiv8EnDm = 1<<16,
	 MpllPostdivEnDm	= 1<<11,
	 MpllPostdiv_SHIFTDm = 8,
	CorePll3RegDm	= 0x01A0,
	 MpllClkdiv16m_SHIFTDm = 8,

	PllPredivDm	= 1,
	PllFbdivDm	= 99,
	PllDacPdValDm	= 1,
	PllDsmPdValDm	= 1,
	PllDiv5EnValDm	= 0,
	PllDivmDm		= 0,
	PllDivauxDm	= 8,
	PllDivpDm		= 1,

	MpllPredivDm	= 2,
	MpllFbdivDm	= 225,
	MpllDacPdValDm	= 1,
	MpllDsmPdValDm	= 1,
	MpllPostdivRegDm	= 0,
	MpllPostdivEnValDm = 1,
	MpllVcoclkDiv8EnValDm = 1,
	MpllClkdiv16mDm	= 21,

	XmitEnableShiftDm	= 17,
	NumLanesShiftDm	= 5,
	PhyRateShiftDm	= 0,

	/* three more registers soc_dp_hw_set_msa_and_enable_video()
	 * writes. REG_VID_CLK_SEL is a SEPARATE register (0x0028, nowhere
	 * near the rest of this block) selecting external/internal pixel
	 * clock. */
	RegVidClkSel	= 0x0028,
	 RegVidClkSelBit = 1<<0,	/* 0=external pixel clock -- confirmed
					 * in both Linux and U-Boot */
	PhySscDisBit	= 1<<8,		/* lives in DpPhyifCtrlRegDm (0x0100),
					 * same register as lanes/rate/xmit */
	VidBistEn	= 0x0238,

	/* TU (Transfer Unit) fields, computed by hand for this driver's
	 * fixed case (148.5MHz pixel, HBR 2.7Gbps, 2 lanes, 24bpp RGB888,
	 * 280-cycle hblank), following U-Boot's formula (inno_dp.c):
	 *   tu = pixel_clk_khz*bpp*800 / (lanes*link_rate_khz), rounded
	 *      = 148500*24*800 / (2*2700000) = 528 -> tu_int=52 tu_frac=8
	 *   rd_thres = tu_int<6 ? 32 : (hblank<80 ? 12 : 16) = 16
	 *   hb_num = hblank*link_rate_khz / (40*pixel_clk_khz), rounded = 127
	 * Recompute these if the mode, lane count or link rate ever change. */
	DpTuReg		= 0x0218,
	 TuIntShift	= 25,		/* bits[31:25] */
	 TuFracShift	= 21,		/* bits[24:21] */
	 InitThreshShift = 10,		/* bits[18:10] */
	 TuInt		= 52,
	 TuFrac		= 8,
	 InitThresh	= 16,
	DpHblankInterval = 0x0230,
	 HblankIntervalShift = 16,	/* bits[31:16] */
	 HblankLinkCyc	= 127,
};

/* DP1 AUX channel / DPCD access (dpuk3auxread/dpuk3auxwrite) -- native AUX registers */
enum {
	/* confirmed: k3-pico-itx.dtb, dp1@cac88000 (same block as dpuk3dp.c) */
	DpPhysRegAx		= 0xCAC88000ull,
	DpSizeRegAx		= 0x4000,

	/* 0x001c: shared with PHY_RESET (bit30, untouched by this file) */
	DpResetCtrlRegAx	= 0x001C,
	 AuxResetBitAx	= 1<<27,

	DpAuxIntStaRegAx	= 0x0080,
	 AuxReplyEventAx	= 1<<16,	/* poll for 1 = done; write 1 to clear (W1C) */

	/* length[27:24], addr[23:4], cmd[3:0] -- one register, three fields */
	DpAuxCfg	= 0x0404,
	 AuxLenShift	= 24,
	 AuxAddrShift	= 4,
	 AuxCmdMask	= 0xF,

	DpAuxStart	= 0x0408,
	 AuxStartBit	= 1<<31,

	/* payload registers: NOTE reversed address/significance order,
	 * see STATUS above -- Data1 (first 4 bytes) is at the highest
	 * address, Data4 (last 4 bytes) at the lowest */
	DpAuxData4	= 0x040C,
	DpAuxData3	= 0x0410,
	DpAuxData2	= 0x0414,
	DpAuxData1	= 0x0418,

	DpAuxReply	= 0x0400,
	 AuxReplyStatusShift = 24,
	 AuxReplyStatusMask  = 0xFF<<24,
	  AuxAck	= 0,
	  AuxNack	= 1,
	  AuxDefer	= 2,
	 AuxErrCodeShift = 4,
	 AuxErrCodeMask  = 7<<4,

	AuxCmdWrite	= 8,	/* VESA DP AUX native write request type */
	AuxCmdRead	= 9,	/* VESA DP AUX native read request type */

	AuxMaxLen	= 16,	/* hardware payload limit, 4 x 32-bit registers */
	AuxTimeoutMs	= 200,	/* matches vendor driver's own retry timeout */
};

/* DP1 link training (dpuk3linktrain) -- clock recovery + channel equalization registers */
enum {
	DpPhysRegLt		= 0xCAC88000ull,
	DpSizeRegLt		= 0x4000,

	DpMiscCtrlRegLt	= 0x0018,
	 ScramblerDisableBitLt = 1<<31,

	DpPhyifCtrlRegLt	= 0x0100,
	 TpsSel_SHIFT	= 25,
	 TpsSelMask	= 0xF<<25,

	Lanes		= 2,	/* fixed, matches dpuk3devinit.c */

	/* DPCD offsets: plain VESA DisplayPort standard, not SoC/driver
	 * specific */
	DpcdTrainingAuxRdInterval = 0x0E,	/* within the 16-byte
						 * receiver-cap block read by
						 * dpuk3linktrain() -- re-read
						 * locally here */
	DpcdMaxLaneCount	= 0x02,
	 Tps3SupportedBit	= 1<<6,
	DpcdTrainingPatternSet	= 0x102,
	DpcdTrainingLane0Set	= 0x103,
	DpcdLane01Status	= 0x202,
	DpcdLaneAlignStatusUpdated = 0x204,
	 InterlaneAlignDoneBit	= 1<<0,
	DpcdAdjustRequestLane01 = 0x206,

	/* training_set byte layout (standard) */
	TrainVoltageSwingMask	= 0x3,
	TrainMaxSwingReached	= 1<<2,
	TrainPreEmphasisShift	= 3,
	TrainPreEmphasisMask	= 0x3<<3,
	TrainMaxPreEmphasisReached = 1<<5,

	/* lane0/1 status byte layout (standard), used per-nibble */
	StatusCrDoneBit		= 1<<0,
	StatusChannelEqDoneBit	= 1<<1,
	StatusSymbolLockedBit	= 1<<2,

	TpDisable	= 0,
	Tp1		= 1,
	Tp2		= 2,
	Tp3		= 3,

	MaxTrainRetries	= 8,
};

/* DPU1 mode-set (dpuk3mode) -- RDMA/composer/scene-control/timing-generator registers */
enum {
	/* confirmed: k3-pico-itx.dtb, display-subsystem-dp1 reg property */
	SaturnPhys	= 0xC0440000ull,
	SaturnSize	= 0x60000,

	/* diagnostic only, see dpuk3mode() -- dpu_top.h reg_0 */
	DpuTopId	= 0x0000,

	/* composer (CMP1, dev_id 1) -- composer_x.h */
	Cmp1Base	= 0x30000,
	 CmpEnable	= 0x00,		/* reg_0: module_enable */
	 CmpDstSize	= 0x04,		/* reg_1: dst_w[15:0] dst_h[31:16] */
	 CmpLayer00AreaLr = 0x20,	/* reg_8: layer00_area_left[15:0] right[31:16] */
	 CmpLayer00AreaTb = 0x24,	/* reg_9: layer00_area_top[15:0] bottom[31:16] */
	 CmpLayer00Ctrl	= 0x38,		/* reg_14: layer00_en[0] blend_sel[2:1] dma_id[7:4] alpha[15:8] */
	 PostPipeBypass	= 0x300,	/* CMP1_BASE_ADDR+0x300 = POST_PIPE_ADDR, not individually decoded */
	 PostPipeSize	= 0x334,

	/* RDMA0 (dev_id 1's layer-0 source) -- rdma_path.h, rdma_top.h */
	Rdma0Base	= 0x1000,
	 Rdma0Mode	= 0x00,		/* reg_0: layer_mode+outstanding_num+req_conti_num+
					 * layer_cmpsr_id+axi_port_sel+rdma_burst_len, packed;
					 * not individually decoded, see STATUS */
	 Rdma0Addr0Lo	= 0x24,		/* reg_9: base_addr0_low_ly0 */
	 Rdma0Addr0Hi	= 0x28,		/* reg_10: base_addr0_high_ly0[1:0] */
	 Rdma0Stride	= 0x3C,		/* reg_15: stride0[15:0] stride1[31:16] */
	 Rdma0ImgSize	= 0x40,		/* reg_16: width[15:0] height[31:16] */
	 Rdma0BboxStart	= 0x44,		/* reg_17 */
	 Rdma0BboxEnd	= 0x48,		/* reg_18 */
	 Rdma0Fmt	= 0x74,		/* reg_29: pixel_format[5:0] */
	  FmtXrgb8888	= 8,		/* confirmed: dpu_saturn.c primary_fmts[] */
	 Rdma0FbcMode	= 0x78,		/* reg_30 (rdma_path.h) --
					 * fbc_split_mode[0]/fbc_yuv_transform[1]/
					 * fbc_sb_layout[2]/fbc_tile_type[3].
					 * Forced to 0 (linear, no compression)
					 * -- an inherited nonzero value here
					 * made the RDMA decode our plain
					 * buffer as ARM AFBC-compressed,
					 * corrupting anything with real edges
					 * while flat colour survived by
					 * accident. See doc. */
	 Rdma0Fbc	= 0x7C,		/* reg_31: fbc_mem_size/base/map, not using FBC,
					 * U-Boot's tested value reproduced as-is */
	 DmaTopTune	= 0x284,	/* Rdma0Base + DMA_TOP_BASE_ADDR(0x280) + 0x4 =
					 * rdma_top.h reg_1, round-robin/QoS tuning,
					 * not individually decoded, see STATUS */

	/* timing generator (TMG0, dev_id 1) -- tmg.h */
	Tmg0Base	= 0x51200,
	 TmgCtrl	= 0x00,		/* reg_0: fm_timing_en[2] hsp[7] vsp[8] */
	  TmgCtrlVal	= (1<<2)|(1<<7)|(1<<8),
	 TmgUser	= 0x08,		/* reg_2: generic 4-bit "user" field[15:12],
					 * repurposed by software for output pixel format */
	  OutfmtRgb888	= 2,		/* confirmed: u-boot spacemit_dpu.h OUTFMT_RGB888 */
	 TmgHfp		= 0x0C,		/* reg_3: hfp[27:16] */
	 TmgHsBp		= 0x10,		/* reg_4: hsync_width[9:0] hbp[27:16] */
	 TmgVfpVs	= 0x14,		/* reg_5: vfp[15:0] vsync_width[25:16] */
	 TmgVbpHact	= 0x18,		/* reg_6: vbp[11:0] h_active[29:16] */
	 TmgVact	= 0x1C,		/* reg_7: v_active[13:0] */
	 TmgVsAlignHs	= 0x3C,		/* reg_15: vsync_align_with_hsync[0] */

	/* dpu_scene_ctl (per-pipeline enable/commit/start, dev_id 1) --
	 * dpu_scene_ctl.h. Written last, on purpose -- see dpuk3mode(). */
	SceneCtrlBase	= 0x340,
	 SceneCtrl0	= 0x00,		/* reg_0: nml_rch_en[0] nml_outctl_en[18] nml_frm_timing_en[25:23] */
	  SceneCtrl0Val	= 0x1040001,
	 SceneCtrl2	= 0x08,		/* reg_2: both_cfg_rdy[0] */
	 SceneCtrl3	= 0x0C,		/* reg_3: video_mod[0] timing_inter0/1 */
	  SceneCtrl3Val	= 0x821,
	 SceneCtrl4	= 0x10,		/* reg_4: sw_start[0] */

	/* 1920x1080@60, confirmed live on this board's DP-1 */
	ModeHactiveMd	= 1920,
	ModeVactiveMd	= 1080,
	ModeHfpMd		= 88,
	ModeHsyncMd	= 44,
	ModeHbpMd		= 148,
	ModeVfpMd		= 4,
	ModeVsyncMd	= 5,
	ModeVbpMd		= 36,

	/* XRGB8888: 4 bytes/pixel, no padding -- what dpuk3() passes as
	 * dpuk3mode()'s stride argument, and what FbReserve below (and
	 * dpuk3fbaddr()'s own comment) size the framebuffer for */
	ModeBppMd	= 4,
	ModeStrideMd	= ModeHactiveMd*ModeBppMd,
};

/* Framebuffer physical memory reservation (dpuk3fbaddr) -- reservation size constant */
enum {
	/* 1920x1080 XRGB8888 needs ~7.91MiB (1920*1080*4 = 8294400 bytes);
	 * round up to a clean, page-aligned reservation */
	FbReserve	= 8*1024*1024,
};

/*
 * ADDENDUM: single entry point for the whole DP1 pipeline, matching
 * how riscv64-video-branch's own devarch.c calls its dpuinit() --
 * one plain call, no condition visible in devarch.c at all, with the
 * board check living inside this function instead (same pattern as
 * that reference dpuinit()'s own
 * "if(!dtbsocmatch(...)) { print(...); return; }" at its very start).
 * devarch.c's own archinit() just calls dpuk3() then screeninit() in
 * sequence, unconditionally, exactly like that reference kernel does
 * with "dpuinit(); v2dinit(); screeninit();".
 */
void
dpuk3(void)
{
	uvlong fbaddr;

	if(!dtbsocmatch("spacemit,k3-pico-itx")){
		print("dpuk3: not a K3 Pico-ITX board, DP1 skipping\n");
		return;
	}

	dpuk3pwron();
	/* dpuk3hpdgpiotest() removed: GPIO2_24 is MUX_MODE6 ("dp1_hpd"
	 * dedicated alternate function per k3-pinctrl.dtsi), not plain
	 * GPIO mode, so gpiok3.c's generic GPIO controller isn't
	 * electrically connected to this pad while muxed. The only valid
	 * read of this signal is dpuk3dp.c's HpdInStatus register -- left
	 * defined in dpuk3hpdgpio.c, just not called.
	 */
	dpuk3fusbtest();
	dpuk3typecforce(0);
	/* dpuk3dpreset() removed: pulsing CONTROLLER_RESET/PHY_RESET here
	 * regressed a working DP controller to full silence -- it wipes
	 * controller state this sequence doesn't reproduce (the real
	 * driver does link-training/DPCD work after this pulse that we
	 * don't). Left defined in dpuk3dp.c, just not called.
	 */
	/* Order matches U-Boot's spacemit_dp_probe(): release every
	 * DSI4LN2 reset and enable every DSI4LN2 clock, then EDP1's own
	 * reset/dppxclk, before calling soc_dp_init() (what dpuk3devinit()
	 * replicates) -- so the DP controller's reset pulse/PLL bring-up
	 * happens after its bus clock (mclk/escclk) is already running.
	 */
	dpuk3init();

	/* eDP0/DPU0 pipeline: NOT CALLED. eDP0 has never shown real video
	 * on any software path tried on this board (its AUX channel never
	 * completes a transaction). The full sequence is kept, behind a
	 * single entry point dpuk3edp0() in screenpicoitxedp0.c,
	 * deliberately excluded from mkfile. To resume: add
	 * screenpicoitxedp0.$O to mkfile's OBJ list and call
	 * dpuk3edp0() here, before dpuk3devinit(). See
	 * k3-9front-display-investigation.md for the full history. */

	dpuk3devinit();
	fbaddr = dpuk3fbaddr();
	dpuk3mode(fbaddr, ModeStrideMd);
	dpuk3dp();
}


/*
 * ============================================================
 * Section: power domain (originally dpuk3pwr.c)
 * ============================================================
 */

/*
 * Spacemit K3 LCD1 (DPU1/DP1) power domain: turn it on via plain APMU
 * registers (0xD4282800), no RPMI/mailbox involved. Only LCD1/DP1 is
 * powered here, not LCD0/eDP0's sibling domain.
 *
 * Register source: SpacemiT U-Boot's drivers/power/domain/
 * k3-power-domain.c ("spacemit,k3-pm-domain", K3_PMU_LCD1_PWR_DOMAIN),
 * described here in our own words/code. See
 * k3-9front-display-investigation.md for why this exists and how it
 * was found.
 */

static uchar *apmupwr;

static ulong
rdpwr(int off)
{
	return *(ulong*)(apmupwr + off);
}

static void
wrpwr(int off, ulong v)
{
	*(ulong*)(apmupwr + off) = v;
	coherence();
}

/*
 * Power on the LCD1 (DPU1/DP1) domain. Safe to call more than once
 * (checks bit_pwr_stat first, like the vendor driver does). Call this
 * before dpuk3init()/dpuk3mode()/dpuk3dp() -- if the domain is off,
 * nothing written to DPU1/DP1 registers will stick.
 */
static void
dpuk3pwron(void)
{
	ulong v;
	int tries;

	apmupwr = vmap(ApmuPhysPw, ApmuSizePw);
	if(apmupwr == nil){
		print("dpuk3pwr: vmap %#llux failed\n", ApmuPhysPw);
		return;
	}

	/* one-time setup (harmless to redo if already done by firmware) */
	wrpwr(PmuaPwrBlkTmr, 0xFFFFFFFF);
	wrpwr(CiuLcdConfig, rdpwr(CiuLcdConfig) | CiuLcdEdpMuxEn);

	v = rdpwr(ApmuPowerStatus);
	if(v & Lcd1PwrStat)
		return;	/* LCD1 already on */

	v = rdpwr(Lcd1PwrCtrl);
	v |= Lcd1AutoPwrOn | Lcd1HwMode;
	wrpwr(Lcd1PwrCtrl, v);

	microdelay(PwrOnDelayUs);

	for(tries = PollTries; tries > 0; tries--){
		v = rdpwr(ApmuPowerStatus);
		if(v & Lcd1HwPwrStat)
			break;
		microdelay(PollDelayUs);
	}

	if(tries == 0)
		print("dpuk3pwr: LCD1 power-on TIMED OUT (ApmuPowerStatus %#lux)\n", v);
}

/*
 * ============================================================
 * Section: Type-C orientation switch (originally dpuk3typec.c)
 * ============================================================
 */

/*
 * Spacemit K3 internal Type-C orientation switch override. Forces the
 * SoC-internal Type-C orientation switch (PMUA_TYPEC_CTRL, offset
 * 0x110 in the same APMU block at 0xD4282800 used throughout
 * dpuk3.c), bypassing CC-driven auto-negotiation. Whether orientation
 * 0 or 1 is physically correct depends on the cable; this tries 0
 * (normal) first.
 *
 * Register source: Linux drivers/phy/spacemit/phy-k3-usb3.c
 * (spacemit-com/linux-6.18), PMUA_TYPEC_CTRL and its three bits.
 * See k3-9front-display-investigation.md for why this exists and
 * how it was found.
 */

static uchar *apmutypec;

static ulong
rdtypec(int off)
{
	return *(ulong*)(apmutypec + off);
}

static void
wrtypec(int off, ulong v)
{
	*(ulong*)(apmutypec + off) = v;
	coherence();
}

/*
 * Force the K3-internal Type-C orientation switch on or off.
 * reverse=0: normal orientation, override enabled.
 * reverse=1: reversed orientation, override enabled.
 * If this doesn't help (see file header CC-detection caveat), the
 * next thing to check is the FUSB301 itself, not this switch.
 */
static void
dpuk3typecforce(int reverse)
{
	ulong v;

	apmutypec = vmap(ApmuPhysTy, ApmuSizeTy);
	if(apmutypec == nil){
		print("dpuk3typec: vmap %#llux failed\n", ApmuPhysTy);
		return;
	}

	v = rdtypec(PmuaTypecCtrl);
	v &= ~(TypecOrientOvrd|TypecOrientOvrdEn);
	v |= TypecOrientOvrdEn;
	if(reverse)
		v |= TypecOrientOvrd;
	wrtypec(PmuaTypecCtrl, v);
}

/*
 * ============================================================
 * Section: FUSB301 CC-logic init (originally dpuk3fusb.c)
 * ============================================================
 */

/*
 * Spacemit K3 Pico-ITX: talk directly to the FUSB301 (USB-C CC logic
 * chip) over the K3's own i2c1 bus, address 0x25. Reads the chip's
 * current status, then runs the init sequence (reset pulse, set
 * power mode/toggle time/mode/state, clear interrupt mask).
 *
 * Register source: SpacemiT U-Boot (uboot-for-k3-display-verify,
 * k3-br-v1.0.y), drivers/usb/tcpm/fusb301.c -- register offsets, bit
 * positions and init sequence order, cross-checked against Linux's
 * fusb301_reset_device() for the reset-pulse-before-init step.
 *
 * fusbdev() must set I2Cdev->subaddr = 1: devi2c.c's putaddr() only
 * emits a register-address byte when dev->subaddr > 0, and FUSB301
 * uses 8-bit register addresses -- left at its zero default, every
 * read/write silently addresses whatever register the chip's
 * internal pointer last had, not the one requested.
 *
 * See k3-9front-display-investigation.md for why this exists and
 * how it was found.
 */

static I2Cdev*
fusbdev(void)
{
	I2Cbus *bus;
	I2Cdev *d;

	bus = i2cbus("i2c1");
	if(bus == nil){
		print("dpuk3fusb: i2c1 bus not found\n");
		return nil;
	}
	d = i2cdev(bus, FusbAddr);
	if(d == nil)
		return nil;

	/* FUSB301 uses single-byte (8-bit) register addresses; devi2c.c's
	 * putaddr() only emits the address byte when subaddr > 0. */
	d->subaddr = 1;
	return d;
}

static void
report(I2Cdev *d, char *when)
{
	int status, type, id;

	id = i2creadbyte(d, RegDeviceId);
	delay(2);
	status = i2creadbyte(d, RegStatus);
	delay(2);
	type = i2creadbyte(d, RegType);

	if(id < 0 || status < 0 || type < 0)
		print("dpuk3fusb: %s: read failed (id=%d status=%d type=%d)\n",
			when, id, status, type);
}

/*
 * Read the FUSB301's current status, run U-Boot's own init sequence
 * against it, and report status once more afterward. The chip's own
 * CC toggle-detect cycle runs autonomously in hardware ("TOGGLING
 * state"); a boot-time attach-wait loop was tried and removed, see
 * below.
 */
static void
dpuk3fusbtest(void)
{
	I2Cdev *d;

	d = fusbdev();
	if(d == nil)
		return;

	/* Linux and U-Boot both pulse a chip-level software reset before
	 * the mode/pwr/toggle/state sequence. */
	i2cwritebyte(d, RegReset, ResetSwRes);
	delay(10);

	i2cwritebyte(d, RegControl,
		(i2creadbyte(d, RegControl) & ~CtrlCurMask) | CtrlCur1500ma);
	delay(2);
	i2cwritebyte(d, RegControl,
		(i2creadbyte(d, RegControl) & ~CtrlTglMask) | CtrlTgl35ms);
	delay(2);
	i2cwritebyte(d, RegModes,
		(i2creadbyte(d, RegModes) & ~ModesMask) | ModesDrpAcc);
	delay(2);
	i2cwritebyte(d, RegManual,
		(i2creadbyte(d, RegManual) & ~ManualMask) | ManualErrRecovery);
	delay(2);

	/* Linux also clears the chip's own global interrupt mask here,
	 * as part of the same real init sequence -- we don't use the
	 * IRQ pin (polling instead), but reproduced for fidelity in case
	 * it affects internal state beyond just the physical INT line. */
	i2cwritebyte(d, RegControl, i2creadbyte(d, RegControl) & ~CtrlIntMask);
	delay(2);

	report(d, "after init sequence");

	/* A boot-time attach-wait loop was tried and removed: FUSB301
	 * attach state does not gate DP1 working on this board (confirmed
	 * against a running Linux kernel's own i2c1 traffic). */
}

/*
 * ============================================================
 * Section: DSI4LN2/plain clock and reset tree, pxclk mux (originally dpuk3.c)
 * ============================================================
 */

/*
 * Spacemit K3 display pipeline: clock/reset bring-up for the DPU1/DP1
 * path (scans out over USB-C DisplayPort Alt-Mode through the
 * on-board ANX7447).
 *
 * dpu1_crtc0's DTB clocks/resets properties select the "DSI4LN2"
 * variant clock/reset IDs, not the "plain" LCD_* ones:
 *   pxclk  -> CLK_APMU_DSI4LN2_LCD_PXCLK   (51)
 *   mclk   -> CLK_APMU_DSI4LN2_LCD_MCLK    (52)
 *   aclk   -> CLK_APMU_DSI4LN2_DPU_ACLK    (53)
 *   mclk_reset -> RESET_APMU_DSI4LN2_LCD_MCLK   (55)
 *   lcd_reset  -> RESET_APMU_DSI4LN2_LCD_SW     (54)
 *   aclk_reset -> RESET_APMU_DSI4LN2_DPU_ACLK   (57)
 *   dsc_reset  -> RESET_APMU_DSI4LN2_LCD_DSCCLK (56)
 * These live in APMU_LCD_CLK_RES_CTRL3/CTRL4 and part of CTRL5, a
 * separate register set from the "plain" LCD_* clocks (CTRL1/CTRL2,
 * CTRL5 bit 16/15), which most likely belong to the other pipeline
 * (DPU0/eDP0). This file sets both the plain bits (kept, harmless)
 * and the DSI4LN2 bits (load-bearing for DP1). EDP_CTRL's
 * EDP1_PRSTN/EDP1_PCLK_EN (bits 16/17) are untouched here.
 * pxclk/mclk/aclk divider and mux fields are left untouched in both
 * variants, inherited from the donor Linux kernel via kexec.
 *
 * Register sources:
 *   - k3-pico-itx.dtb: dpu1_crtc0's clocks/resets/clock-names/
 *     reset-names properties.
 *   - include/dt-bindings/clock/spacemit,k3-syscon.h
 *     (spacemit-com/linux-6.18): numeric ID -> symbolic name mapping.
 *   - drivers/clk/spacemit/ccu-k3.c: symbolic clock -> register/bit
 *     mapping.
 *   - drivers/reset/reset-spacemit-k3.c (SpacemiT U-Boot,
 *     uboot-for-k3-display-verify, k3-br-v1.0.y): symbolic reset ->
 *     register/bit mapping.
 *
 * EDP_CTRL bit 18 (edp1_pxclk mux select) is left untouched: a live
 * regmap dump from a working Linux boot on this board showed it SET
 * (selecting external_clk, not dsi4ln2_lcd_pxclk). See
 * k3-9front-display-investigation.md for why this exists and how it
 * was found.
 */

static uchar *apmuclk;

static ulong
rdclk(int off)
{
	return *(ulong*)(apmuclk + off);
}

static void
wrclk(int off, ulong v)
{
	*(ulong*)(apmuclk + off) = v;
	coherence();
}

static void
waitfc(int off, ulong bit)
{
	int i;

	for(i = 0; i < 10000 && (rdclk(off) & bit) != 0; i++)
		;
	if(i == 10000)
		print("dpuk3: waitfc TIMED OUT on reg %#x bit %#lux (still %#lux)\n",
			off, bit, rdclk(off));
}

/*
 * Bring up clocks/resets for the DPU1 -> DP1 chain. Sets both the
 * "plain" LCD_* bits (kept from the original version, likely
 * DPU0/eDP0's, harmless here) and the DSI4LN2_* bits that dpu1_crtc0
 * (our actual pipeline) uses per its own DTB clocks/resets
 * properties -- see file header for the correction this represents.
 * Leaves pxclk/mclk/aclk divider and mux fields untouched in both
 * variants. Safe to call more than once.
 */
static void
dpuk3init(void)
{
	ulong v;

	apmuclk = vmap(ApmuPhysCk, ApmuSizeCk);
	if(apmuclk == nil){
		print("dpuk3: vmap %#llux failed\n", ApmuPhysCk);
		return;
	}

	waitfc(ApmuLcdClkResCtrl1, LcdPxclkFc);
	waitfc(ApmuLcdClkResCtrl1, LcdMclkFc);
	waitfc(ApmuLcdClkResCtrl5, DpuAclkFc);
	waitfc(ApmuLcdClkResCtrl3, Dsi4ln2PxclkFc);
	waitfc(ApmuLcdClkResCtrl3, Dsi4ln2MclkFc);
	waitfc(ApmuLcdClkResCtrl5, Dsi4ln2AclkFc);

	/* plain variant -- kept, see file header */
	v = rdclk(ApmuLcdClkResCtrl1);
	v |= LcdSwRst | LcdHclkEn;
	wrclk(ApmuLcdClkResCtrl1, v);

	v = rdclk(ApmuLcdClkResCtrl2);
	v |= LcdPxclkEn | LcdMclkEn | LcdMclkRst | LcdDscclkRst;
	wrclk(ApmuLcdClkResCtrl2, v);

	v = rdclk(ApmuLcdClkResCtrl5);
	v |= DpuAclkEn | DpuAclkRst;
	wrclk(ApmuLcdClkResCtrl5, v);

	/* DSI4LN2 variant -- what DP1 actually needs. escclk bits included:
	 * the resulting value (0x80401c) matches a live regmap dump from
	 * a working Linux boot on this board. */
	v = rdclk(ApmuLcdClkResCtrl3);
	v |= Dsi4ln2LcdSwRst | Dsi4ln2EscclkRst | Dsi4ln2EscclkEn;
	wrclk(ApmuLcdClkResCtrl3, v);

	/* pxclk mux+divider, set explicitly rather than left inherited:
	 * pll2_d4 /5 = 150.000MHz, the closest achievable rate to the
	 * target 148.5MHz from any of the 8 available parents, matching
	 * U-Boot's spacemit_dp_probe() default clk_set_rate() target
	 * (dev_read_u32_default(dev, "pix-clk", 150000000)). */
	v = rdclk(ApmuLcdClkResCtrl4);
	v = (v & ~(7<<Dsi4ln2PxclkMux_SHIFT) & ~(7<<Dsi4ln2PxclkDiv_SHIFT))
		| (Dsi4ln2PxclkMuxPll2d4<<Dsi4ln2PxclkMux_SHIFT)
		| (Dsi4ln2PxclkDivBy5<<Dsi4ln2PxclkDiv_SHIFT)
		| Dsi4ln2PxclkEn | Dsi4ln2MclkEn | Dsi4ln2MclkRst | Dsi4ln2DscclkRst;
	wrclk(ApmuLcdClkResCtrl4, v);

	/* commit the mux/div change via the FC (frequency-change) trigger,
	 * same register/bit already waited-on at the top of this function */
	v = rdclk(ApmuLcdClkResCtrl3);
	wrclk(ApmuLcdClkResCtrl3, v | Dsi4ln2PxclkFc);
	waitfc(ApmuLcdClkResCtrl3, Dsi4ln2PxclkFc);

	v = rdclk(ApmuLcdClkResCtrl5);
	v |= Dsi4ln2AclkEn | Dsi4ln2AclkRst;
	wrclk(ApmuLcdClkResCtrl5, v);

	/* DP1/eDP1 APB reset release + pixel clock gate. Bit 18 (edp1_pxclk
	 * mux select) is deliberately left untouched: a working Linux
	 * boot's regmap dump shows EDP_CTRL = 0x70007, bit 18 SET
	 * (selecting external_clk, not dsi4ln2_lcd_pxclk). */
	v = rdclk(ApmuLcdEdpCtrl);
	v |= Edp1Prstn | Edp1PclkEn;
	wrclk(ApmuLcdEdpCtrl, v);

	print("dpuk3: clocks/resets up (Ctrl1 %#lux Ctrl2 %#lux Ctrl3 %#lux "
		"Ctrl4 %#lux Ctrl5 %#lux EdpCtrl %#lux)\n",
		rdclk(ApmuLcdClkResCtrl1), rdclk(ApmuLcdClkResCtrl2),
		rdclk(ApmuLcdClkResCtrl3), rdclk(ApmuLcdClkResCtrl4),
		rdclk(ApmuLcdClkResCtrl5), rdclk(ApmuLcdEdpCtrl));
}

/*
 * Linux's soc_dp_encoder_enable() re-writes this pxclk mux-select bit
 * every time the mode is enabled, not just once at probe time
 * (regmap_update_bits(SOC_DP_APMU_CLK_CTRL, BIT(2), BIT(2)) for
 * dpu_id==0/eDP0, BIT(18) for dpu_id==1/DP1) -- dpuk3init() only
 * writes it once, at boot. Call this from dpuk3phycycle(), right
 * before PLL/lane reconfiguration, to match Linux's call order.
 */
static void
dpuk3edppxclkmux(int dpu1)
{
	ulong before, v;

	if(apmuclk == nil)
		return;

	before = rdclk(ApmuLcdEdpCtrl);
	v = before;
	if(dpu1)
		v |= Edp1PxclkMuxBit;	/* bit18, DP1/dpu_id==1 */
	else
		v |= Edp0PxclkMuxBit;	/* bit2, eDP0/dpu_id==0 */
	wrclk(ApmuLcdEdpCtrl, v);

	print("dpuk3edppxclkmux: dpu_id=%d EdpCtrl before %#lux after %#lux "
		"(bit %s)\n", dpu1, before, rdclk(ApmuLcdEdpCtrl),
		before==rdclk(ApmuLcdEdpCtrl) ? "already set, no change" : "newly set");
}

/*
 * ============================================================
 * Section: DP1 controller init: reset, PLLs, analog TX calibration (originally dpuk3devinit.c)
 * ============================================================
 */

/*
 * Spacemit K3 DP1 controller: full, faithful replica of Linux's
 * soc_dp_dev_init() (spacemit_inno_dp.c), in the exact order it runs
 * there -- including the analog TX-driver calibration block (current
 * bias, main/pre/post-cursor levels, return-loss/termination
 * calibration) and SOC_DPTX_ANA_TX_AUX_RX_VSEL (AUX-receiver analog
 * voltage-select).
 *
 * Supersedes the piecemeal approach split across dpuk3phy.c
 * (pixel/core PLL only) and dpuk3dp.c's reset-pulse/XMIT_ENABLE logic:
 * this reproduces the real order (reset pulse BEFORE powering either
 * PLL back up, not after) plus the calibration block. Call this
 * INSTEAD of dpuk3phypll()/dpuk3phycorepll()/dpuk3dpreset(), before
 * dpuk3dp()'s MSA-description work.
 *
 * Register source: Linux drivers/gpu/drm/spacemit/spacemit_inno_dp.c
 * (spacemit-com/linux-6.18), soc_dp_dev_init() and the three
 * functions it calls (soc_dp_phy_config_lanes/config_rate/
 * power_on) -- offsets/fields from spacemit_inno_dp.h.
 *
 * The reset pulse here excludes PHY_RESET (resets only
 * controller/hdcp/audio/aux/video), matching U-Boot's soc_dp_init()
 * rather than Linux's soc_dp_dev_init(): U-Boot's AUX works from a
 * real cold boot on this board, and a kexec transition may not
 * reproduce the PHY-internal calibration state that re-asserting
 * PHY_RESET in software can't restore on its own.
 *
 * See k3-9front-display-investigation.md for why this exists and how
 * it was found.
 */

static uchar *dpDevinit;

static ulong
rdDevinit(int off)
{
	return *(ulong*)(dpDevinit + off);
}

static void
wrDevinit(int off, ulong v)
{
	*(ulong*)(dpDevinit + off) = v;
	coherence();
}

static void
setbitsDevinit(int off, ulong mask, ulong val)
{
	wrDevinit(off, (rdDevinit(off) & ~mask) | val);
}

/*
 * Full, faithful replica of soc_dp_dev_init(), in order. Replaces
 * dpuk3phypll()+dpuk3phycorepll()+dpuk3dpreset() as the thing called
 * before dpuk3mode()/dpuk3dp().
 */
static void
dpuk3devinit(void)
{
	int tries;
	ulong v;

	dpDevinit = vmap(DpPhysRegDv, DpSizeRegDv);
	if(dpDevinit == nil){
		print("dpuk3devinit: vmap %#llux failed\n", DpPhysRegDv);
		return;
	}

	/* 1: disable transmitter, power down both PLLs */
	setbitsDevinit(DpPhyifCtrlRegDv, 0xF<<XmitEnableShiftDv, 0);
	delay(2);
	wrDevinit(CorePll0RegDv, rdDevinit(CorePll0RegDv)|MpllPdDv);
	wrDevinit(PrePll0RegDv, rdDevinit(PrePll0RegDv)|PllPdDv);
	delay(2);

	/* 2: reset pulse, matching U-Boot's soc_dp_init() (not Linux's
	 * soc_dp_dev_init()): excludes PHY_RESET, resetting only
	 * controller/hdcp/audio/aux/video. See file header. */
	wrDevinit(DpResetCtrlRegDv, rdDevinit(DpResetCtrlRegDv) | (AllResetsMaskDv & ~PhyResetBitDv));
	delay(5);
	wrDevinit(DpResetCtrlRegDv, rdDevinit(DpResetCtrlRegDv) & ~(AllResetsMaskDv & ~PhyResetBitDv));
	delay(2);

	wrDevinit(DpAuxIntStaRegDv, AuxReplyEventDv);

	setbitsDevinit(DpMiscCtrlRegDv, FastLinkTrainEnBit, 0);
	setbitsDevinit(DpMiscCtrlRegDv, ScramblerDisableBitDv, 0);
	/* SCALE_DOWN_MODE lives in the same register as FORCE_HPD
	 * elsewhere in this codebase; Linux clears it here too, but
	 * this file doesn't touch FORCE_HPD's own bit -- see dpuk3dp.c */
	setbitsDevinit(DpPhyifCtrlRegDv, 0xF<<XmitEnableShiftDv, 0);

	setbitsDevinit(DpVideoCtrlRegDv, VideoStreamEnDv, 0);

	/* unmask interrupts (all the ones soc_dp_dev_init() unmasks) */
	setbitsDevinit(AuxReplyMsk, AuxReplyMskBit, 0);
	setbitsDevinit(IllegalAuxMsk, IllegalAuxMskBit, 0);
	setbitsDevinit(TypeCEventMsk, TypeCEventMskBit, 0);
	setbitsDevinit(DscEventMsk, DscEventMskBit, 0);
	setbitsDevinit(SdpS3Msk, SdpS3MskBit, 0);
	setbitsDevinit(SdpS2Msk, SdpS2MskBit, 0);
	setbitsDevinit(SdpS1Msk, SdpS1MskBit, 0);
	setbitsDevinit(SdpS0Msk, SdpS0MskBit, 0);
	setbitsDevinit(FifoS3Msk, FifoS3MskBit, 0);
	setbitsDevinit(FifoS2Msk, FifoS2MskBit, 0);
	setbitsDevinit(FifoS1Msk, FifoS1MskBit, 0);
	setbitsDevinit(FifoS0Msk, FifoS0MskBit, 0);
	setbitsDevinit(SinkIrqMsk, SinkIrqMskBit, 0);
	setbitsDevinit(HpdIntMsk, HpdIntMskBit, 0);
	setbitsDevinit(HotPlugMsk, HotPlugMskBit, 0);
	setbitsDevinit(HotUnplugMsk, HotUnplugMskBit, 0);
	setbitsDevinit(SinkUnplugErrMsk, SinkUnplugErrMskBit, 0);
	delay(2);

	/* bypass PHY busy state */
	wrDevinit(DpPhyifCtrlRegDv, rdDevinit(DpPhyifCtrlRegDv) | PhyBusyBypBit);

	setbitsDevinit(DpMiscCtrlRegDv, EnhanceFramingEnBit, EnhanceFramingEnBit);

	/* 3: configure lanes/rate registers explicitly (matches this
	 * board's already-inherited values, see STATUS, but Linux always
	 * writes them rather than trusting inherited state) */
	setbitsDevinit(DpPhyifCtrlRegDv, 3<<NumLanesShiftDv, 1<<NumLanesShiftDv);	/* 2 lanes */
	setbitsDevinit(DpPhyifCtrlRegDv, 3<<PhyRateShiftDv, 1<<PhyRateShiftDv);	/* HBR 2.7Gbps */

	/* 4: power both PLLs back up */
	v = (PllPredivDv<<PllPrediv_SHIFTDv)
		| ((PllFbdivDv & 0xFF)<<PllFbdivLo_SHIFTDv)
		| (((PllFbdivDv>>8) & 0xF)<<PllFbdivHi_SHIFTDv)
		| (PllDacPdValDv ? PllDacPdDv : 0)
		| (PllDsmPdValDv ? PllDsmPdDv : 0)
		| PllDownspreadBitDv | PllDisableSscgBitDv;
	wrDevinit(PrePll0RegDv, v);
	wrDevinit(PrePll1RegDv, (PllDivmDv<<PllDivm_SHIFTDv) | (PllDivauxDv<<PllDivaux_SHIFTDv));
	wrDevinit(PrePll2RegDv, (PllDiv5EnValDv ? PllDiv5EnDv : 0) | (PllDivpDv<<PllDivp_SHIFTDv) | PrePllDpEnBitDv);
	wrDevinit(PrePll3RegDv, 0);
	wrDevinit(PrePll0RegDv, v | PllOutputNormalDv);

	v = (MpllPredivDv<<MpllPrediv_SHIFTDv)
		| ((MpllFbdivDv & 0xFF)<<MpllFbdivLo_SHIFTDv)
		| (((MpllFbdivDv>>8) & 0xF)<<MpllFbdivHi_SHIFTDv)
		| (MpllDacPdValDv ? MpllDacPdDv : 0)
		| (MpllDsmPdValDv ? MpllDsmPdDv : 0)
		| MpllDownspreadBitDv | MpllDisableSscgBitDv;
	wrDevinit(CorePll0RegDv, v | MpllPdDv);
	delay(2);
	wrDevinit(CorePll1RegDv, 0);
	wrDevinit(CorePll2RegDv, (MpllVcoclkDiv8EnValDv ? MpllVcoclkDiv8EnDv : 0)
		| (MpllPostdivEnValDv ? MpllPostdivEnDv : 0)
		| (MpllPostdivRegDv<<MpllPostdiv_SHIFTDv));
	wrDevinit(CorePll3RegDv, MpllClkdiv16mDv<<MpllClkdiv16m_SHIFTDv);
	wrDevinit(PrePll2RegDv, rdDevinit(PrePll2RegDv) | PrePllLockBypenDv);
	wrDevinit(CorePll0RegDv, v);
	delay(2);

	/* 5: enable transmitter for 2 lanes, then confirm both PLLs lock */
	setbitsDevinit(DpPhyifCtrlRegDv, 0xF<<XmitEnableShiftDv, 3<<XmitEnableShiftDv);
	delay(2);

	for(tries = 1000; tries > 0; tries--){
		if((rdDevinit(PrePll0RegDv) & PixelPllLockDv) && (rdDevinit(CorePll0RegDv) & CorePllLockDv))
			break;
		microdelay(100);
	}
	if(tries == 0)
		print("dpuk3devinit: PLL lock TIMED OUT: pixel=%d core=%d\n",
			(rdDevinit(PrePll0RegDv)&PixelPllLockDv)!=0,
			(rdDevinit(CorePll0RegDv)&CorePllLockDv)!=0);

	/* 6: analog TX driver calibration, see file header. This clears
	 * AnaTxModePresel's MODE_Dx fields via one whole-register write
	 * to 0 rather than four individual field writes like Linux does;
	 * harmless since the setbitsDevinit() call below overwrites every
	 * field with its real value before anything reads it. */
	wrDevinit(AnaTxModePresel, 0);
	setbitsDevinit(AnaRtcal, 0x7F<<RtcalFreqdivH_SHIFT, ((ClkDiv>>8)&0x7F)<<RtcalFreqdivH_SHIFT);
	setbitsDevinit(AnaRtcal, RtcalBypassBit, RtcalBypassBit);
	setbitsDevinit(AnaRtcal, 0xFF<<RtcalFreqdivL_SHIFT, (ClkDiv&0xFF)<<RtcalFreqdivL_SHIFT);
	setbitsDevinit(AnaRtcal, 3<<RtcalBgSel_SHIFT, 0);

	wrDevinit(AnaTxRtm, 0);
	setbitsDevinit(AnaRtcal, RtcalBypassBit, RtcalBypassBit);
	setbitsDevinit(AnaRtcal, RtcalBypassBit, 0);
	delay(100);

	setbitsDevinit(AnaTxPostselPreMode,
		(1<<TxModePreD3_SHIFT)|(1<<TxModePreD2_SHIFT)|(1<<TxModePreD1_SHIFT)|(1<<TxModePreD0_SHIFT)
		|(1<<TxModeDeD3_SHIFT)|(1<<TxModeDeD2_SHIFT)|(1<<TxModeDeD1_SHIFT)|(1<<TxModeDeD0_SHIFT)
		|(TxPre<<TxPostselPreD3_SHIFT)|(TxPre<<TxPostselPreD2_SHIFT)
		|(TxPre<<TxPostselPreD1_SHIFT)|(TxPre<<TxPostselPreD0_SHIFT),
		(1<<TxModePreD3_SHIFT)|(1<<TxModePreD2_SHIFT)|(1<<TxModePreD1_SHIFT)|(1<<TxModePreD0_SHIFT)
		|(1<<TxModeDeD3_SHIFT)|(1<<TxModeDeD2_SHIFT)|(1<<TxModeDeD1_SHIFT)|(1<<TxModeDeD0_SHIFT)
		|(TxPre<<TxPostselPreD3_SHIFT)|(TxPre<<TxPostselPreD2_SHIFT)
		|(TxPre<<TxPostselPreD1_SHIFT)|(TxPre<<TxPostselPreD0_SHIFT));

	wrDevinit(AnaTxIselDrvHi, (MIsel<<TxIselD3_SHIFT)|(MIsel<<TxIselD2_SHIFT));
	setbitsDevinit(AnaTxMainselIselLo,
		(0x1F<<TxMainselD2_SHIFT)|(0x1F<<TxMainselD3_SHIFT)|(0xF<<TxIselD1_SHIFT)|(0xF<<TxIselD0_SHIFT),
		(MMainsel<<TxMainselD2_SHIFT)|(MMainsel<<TxMainselD3_SHIFT)|(MIsel<<TxIselD1_SHIFT)|(MIsel<<TxIselD0_SHIFT));
	setbitsDevinit(AnaTxPostselMainsel,
		(0xF<<TxPostselD1_SHIFT)|(0xF<<TxPostselD0_SHIFT)|(0xF<<TxPostselD3_SHIFT)|(0xF<<TxPostselD2_SHIFT)
		|(0x1F<<TxMainselD0_SHIFT)|(0x1F<<TxMainselD1_SHIFT),
		(MPost<<TxPostselD1_SHIFT)|(MPost<<TxPostselD0_SHIFT)|(MPost<<TxPostselD3_SHIFT)|(MPost<<TxPostselD2_SHIFT)
		|(MMainsel<<TxMainselD0_SHIFT)|(MMainsel<<TxMainselD1_SHIFT));
	setbitsDevinit(AnaTxModePresel,
		(1<<TxModeD3_SHIFT)|(1<<TxModeD2_SHIFT)|(1<<TxModeD1_SHIFT)|(1<<TxModeD0_SHIFT)
		|(7<<TxPreselD1_SHIFT)|(7<<TxPreselD0_SHIFT)|(7<<TxPreselD3_SHIFT)|(7<<TxPreselD2_SHIFT),
		(TxMode<<TxModeD3_SHIFT)|(TxMode<<TxModeD2_SHIFT)|(TxMode<<TxModeD1_SHIFT)|(TxMode<<TxModeD0_SHIFT)
		|(MPre<<TxPreselD1_SHIFT)|(MPre<<TxPreselD0_SHIFT)|(MPre<<TxPreselD3_SHIFT)|(MPre<<TxPreselD2_SHIFT));

	/* the one register this whole file exists for: AUX receiver
	 * analog voltage select, never touched anywhere before this */
	setbitsDevinit(AnaTxAuxRxVsel, 3<<AuxRxVsel_SHIFT, 0);

	/* Re-enable everything Linux's inno_dp_shutdown() turns off (LDO,
	 * output driver, serializer per lane, bandgap reference): a kexec
	 * from a Linux that already ran its own shutdown() leaves these
	 * disabled, unlike a genuine cold boot. */
	setbitsDevinit(AnaTxEnLdo,
		(1<<EnLdoD3_SHIFT)|(1<<EnLdoD2_SHIFT)|(1<<EnLdoD1_SHIFT)|(1<<EnLdoD0_SHIFT)
		|(1<<EnDrvD3_SHIFT)|(1<<EnDrvD2_SHIFT)|(1<<EnDrvD1_SHIFT)|(1<<EnDrvD0_SHIFT),
		(1<<EnLdoD3_SHIFT)|(1<<EnLdoD2_SHIFT)|(1<<EnLdoD1_SHIFT)|(1<<EnLdoD0_SHIFT)
		|(1<<EnDrvD3_SHIFT)|(1<<EnDrvD2_SHIFT)|(1<<EnDrvD1_SHIFT)|(1<<EnDrvD0_SHIFT));
	setbitsDevinit(AnaTxEnP2s,
		(1<<EnP2sD3_SHIFT)|(1<<EnP2sD2_SHIFT)|(1<<EnP2sD1_SHIFT)|(1<<EnP2sD0_SHIFT),
		(1<<EnP2sD3_SHIFT)|(1<<EnP2sD2_SHIFT)|(1<<EnP2sD1_SHIFT)|(1<<EnP2sD0_SHIFT));
	setbitsDevinit(AnaBgEn, 1<<BgEn_SHIFT, 1<<BgEn_SHIFT);
	setbitsDevinit(AnaRtcal, 1<<AnaBgEnRcal_SHIFT, 1<<AnaBgEnRcal_SHIFT);

	/* soc_dp_dev_init()'s caller (soc_dp_bind()) calls
	 * soc_dp_phy_power_off() immediately after it returns: the PHY
	 * goes back off here until dpuk3phycycle() powers it back on for
	 * real. This is the real soc_dp_phy_power_off() sequence: only
	 * PREPLL (pixel) is powered down, NOT the core/link MPLL; includes
	 * an AUX_RESET pulse and clears VIDEO_STREAM_ENABLE. */
	setbitsDevinit(DpVideoCtrlRegDv, VideoStreamEnDv, 0);
	setbitsDevinit(DpPhyifCtrlRegDv, 0xF<<XmitEnableShiftDv, 0);
	delay(2);
	wrDevinit(PrePll0RegDv, rdDevinit(PrePll0RegDv)|PllPdDv);
	delay(2);
	wrDevinit(DpResetCtrlRegDv, rdDevinit(DpResetCtrlRegDv)|AuxResetBitDv);
	delay(5);
	wrDevinit(DpResetCtrlRegDv, rdDevinit(DpResetCtrlRegDv)&~AuxResetBitDv);
	delay(2);
	print("dpuk3devinit: done\n");
}

/*
 * ============================================================
 * Section: DP1 MSA, phy cycle, TU config, real link training call (originally dpuk3dp.c)
 * ============================================================
 */

/*
 * Spacemit K3 DP1 controller: MSA (Main Stream Attributes) + video
 * stream enable, for the DP1 pipeline.
 *
 * "Inherit, don't retrain": assumes the DP1 link is already up --
 * lanes powered, clock-recovery and channel-equalization already
 * done -- since this board is brought up via kexec from a Linux that
 * already left DP-1 connected and scanning out real video. Does NOT
 * touch the DP PHY, CONTROLLER_RESET/PHY_RESET, run any AUX/DPCD
 * link-training sequence, or touch link rate/lane count -- all left
 * as the donor Linux configured them. A cold-boot path (no donor
 * Linux, full PLL/PHY setup and clock-recovery/channel-eq training
 * from scratch) is not implemented.
 *
 * Register source: Linux drivers/gpu/drm/spacemit/spacemit_inno_dp.h
 * (spacemit-com/linux-6.18), SOC_DPTX_* "offset,msb,lsb" macros.
 * Physical base from k3-pico-itx.dtb, dp1@cac88000.
 *
 * The mode is the same fixed 1920x1080@60 as dpuk3mode.c (must
 * match -- this only describes the identical timing the DPU/TMG side
 * is already generating, it does not resynthesize the pixel clock).
 * Hsync/Vsync polarity is written as 1/1 unconditionally, matching
 * soc_dp_hw_set_msa_and_enable_video()'s real runtime behavior (it
 * computes a mode-flag-dependent value first, then unconditionally
 * overwrites both bits to 1).
 */

static uchar *dpDpMain;

static ulong
rdDpMain(int off)
{
	return *(ulong*)(dpDpMain + off);
}

static void
wrDpMain(int off, ulong v)
{
	*(ulong*)(dpDpMain + off) = v;
	coherence();
}

/*
 * Describe the (already-running) 1920x1080@60 timing to the DP1
 * controller's MSA registers and (re-)enable the video stream.
 * Does not touch the PHY, link rate, lane count, or run any
 * training -- see file header.
 */
/*
 * Second PHY power-off -> reconfigure -> power-on cycle, matching
 * what soc_dp_encoder_enable() does at real mode-enable time, ON TOP
 * OF (not instead of) whatever dpuk3devinit()/soc_dp_dev_init()
 * already did once at bind time. Call this immediately before the
 * MSA/video-enable writes in dpuk3dp().
 */
static void
dpuk3phycycle(void)
{
	ulong v;
	int tries;

	/* re-assert the pxclk mux-select bit, matching
	 * soc_dp_encoder_enable()'s own re-write of it every mode-enable
	 * -- see dpuk3edppxclkmux()'s comment in dpuk3.c. */
	dpuk3edppxclkmux(1);	/* DP1 = dpu_id 1 */

	/* power-off phase matches the real soc_dp_phy_power_off(): only
	 * PREPLL (pixel) is powered down, NOT the core/link MPLL;
	 * VIDEO_STREAM_ENABLE is cleared first; an AUX_RESET pulse is
	 * included. */
	wrDpMain(DpVideoCtrlRegDm, rdDpMain(DpVideoCtrlRegDm) & ~VideoStreamEnDm);
	v = rdDpMain(DpPhyifCtrlRegDm);
	v &= ~(0xFul<<XmitEnableShiftDm);
	wrDpMain(DpPhyifCtrlRegDm, v);
	delay(2);
	wrDpMain(PrePll0RegDm, rdDpMain(PrePll0RegDm)|PllPdDm);
	delay(2);
	wrDpMain(DpResetCtrlRegDm, rdDpMain(DpResetCtrlRegDm)|AuxResetBitDm);
	delay(5);
	wrDpMain(DpResetCtrlRegDm, rdDpMain(DpResetCtrlRegDm)&~AuxResetBitDm);
	delay(2);

	/* reconfigure: same lane/rate fields, same PLL divider values as
	 * dpuk3devinit.c -- see that file for how these were derived */
	v = rdDpMain(DpPhyifCtrlRegDm);
	v = (v & ~(3<<NumLanesShiftDm)) | (1<<NumLanesShiftDm);	/* 2 lanes */
	v = (v & ~(3<<PhyRateShiftDm)) | (1<<PhyRateShiftDm);	/* HBR 2.7Gbps */
	wrDpMain(DpPhyifCtrlRegDm, v);

	v = (PllPredivDm<<PllPrediv_SHIFTDm)
		| ((PllFbdivDm & 0xFF)<<PllFbdivLo_SHIFTDm)
		| (((PllFbdivDm>>8) & 0xF)<<PllFbdivHi_SHIFTDm)
		| (PllDacPdValDm ? PllDacPdDm : 0)
		| (PllDsmPdValDm ? PllDsmPdDm : 0)
		| PllDownspreadBitDm | PllDisableSscgBitDm;
	wrDpMain(PrePll0RegDm, v);
	wrDpMain(PrePll1RegDm, (PllDivmDm<<PllDivm_SHIFTDm) | (PllDivauxDm<<PllDivaux_SHIFTDm));
	wrDpMain(PrePll2RegDm, (PllDiv5EnValDm ? PllDiv5EnDm : 0) | (PllDivpDm<<PllDivp_SHIFTDm) | PrePllDpEnBitDm);
	wrDpMain(PrePll3RegDm, 0);
	wrDpMain(PrePll0RegDm, v | PllOutputNormalDm);

	v = (MpllPredivDm<<MpllPrediv_SHIFTDm)
		| ((MpllFbdivDm & 0xFF)<<MpllFbdivLo_SHIFTDm)
		| (((MpllFbdivDm>>8) & 0xF)<<MpllFbdivHi_SHIFTDm)
		| (MpllDacPdValDm ? MpllDacPdDm : 0)
		| (MpllDsmPdValDm ? MpllDsmPdDm : 0)
		| MpllDownspreadBitDm | MpllDisableSscgBitDm;
	wrDpMain(CorePll0RegDm, v | MpllPdDm);
	delay(2);
	wrDpMain(CorePll1RegDm, 0);
	wrDpMain(CorePll2RegDm, (MpllVcoclkDiv8EnValDm ? MpllVcoclkDiv8EnDm : 0)
		| (MpllPostdivEnValDm ? MpllPostdivEnDm : 0)
		| (MpllPostdivRegDm<<MpllPostdiv_SHIFTDm));
	wrDpMain(CorePll3RegDm, MpllClkdiv16mDm<<MpllClkdiv16m_SHIFTDm);
	wrDpMain(PrePll2RegDm, rdDpMain(PrePll2RegDm) | PrePllLockBypenDm);
	wrDpMain(CorePll0RegDm, v);
	delay(2);

	/* power on: re-enable transmitter, confirm both PLLs lock again */
	v = rdDpMain(DpPhyifCtrlRegDm);
	v = (v & ~(0xFul<<XmitEnableShiftDm)) | (3<<XmitEnableShiftDm);	/* 2 lanes */
	wrDpMain(DpPhyifCtrlRegDm, v);
	delay(2);

	for(tries = 1000; tries > 0; tries--){
		if((rdDpMain(PrePll0RegDm) & PixelPllLockDm) && (rdDpMain(CorePll0RegDm) & CorePllLockDm))
			break;
		microdelay(100);
	}
	print("dpuk3dp: phy_cycle PLL re-lock: pixel=%d core=%d (%d tries left)\n",
		(rdDpMain(PrePll0RegDm)&PixelPllLockDm)!=0, (rdDpMain(CorePll0RegDm)&CorePllLockDm)!=0, tries);
}

static void
dpuk3dp(void)
{
	ulong v;

	dpDpMain = vmap(DpPhysRegDm, DpSizeRegDm);
	if(dpDpMain == nil){
		print("dpuk3dp: vmap %#llux failed\n", DpPhysRegDm);
		return;
	}

	{
		int tries;
		ulong hpd = 0;

		for(tries = 300; tries > 0; tries--){
			hpd = rdDpMain(DpHpdStatus) & HpdInStatus;
			if(hpd != 0)
				break;
			delay(10);
		}

		if(hpd == 0)
			wrDpMain(DpHpdForce, rdDpMain(DpHpdForce)|ForceHpdBit);
	}

	{
		ulong phyif = rdDpMain(DpPhyifCtrlRegDm);
		int nlanes;

		/* enable the transmitter for however many lanes are already
		 * configured (00=1, 01=2, 10=4): this board's PhyifCtrl has
		 * numlanes/rate/powerdown already set (survived from before
		 * kexec) but XMIT_ENABLE at 0, so the PHY is powered but not
		 * driving any lane. */
		switch((phyif&NumLanesMask)>>5){
		case 0: nlanes = 1; break;
		case 1: nlanes = 2; break;
		case 2: nlanes = 4; break;
		default: nlanes = 0; break;	/* reserved encoding, do nothing */
		}
		if(nlanes > 0){
			phyif |= ((1<<nlanes)-1)<<17;
			wrDpMain(DpPhyifCtrlRegDm, phyif);
		}
	}

	/* second phy power-off/reconfigure/power-on cycle, matching
	 * soc_dp_encoder_enable() -- see dpuk3phycycle()'s comment.
	 * Called here, immediately before MSA/video-enable, matching
	 * where Linux calls it (right after its own link-training
	 * attempt, which we skip since this cycle itself needs no AUX). */
	dpuk3phycycle();

	/* real link training, using dpuk3aux.c's AUX channel. Matches
	 * where Linux calls soc_dp_link_train() -- right after the phy
	 * cycle, before MSA. Like Linux, proceeds to MSA regardless of
	 * training's own success/failure: even a failed negotiation may
	 * leave the transmitter's analog registers in a better state
	 * (from responding to real ADJUST_REQUEST values) than the fixed
	 * defaults alone. */
	if(dpuk3linktrain() < 0)
		print("dpuk3dp: link training did not complete successfully, "
			"proceeding to MSA anyway (matches Linux's own behaviour)\n");

	/* VIDEO_MAPPING written here, as its own first write, matching
	 * soc_dp_hw_set_msa_and_enable_video()'s order (not combined with
	 * the final VIDEO_STREAM_ENABLE write). */
	wrDpMain(DpVideoCtrlRegDm, (rdDpMain(DpVideoCtrlRegDm) & ~VideoMappingMask)
		| (MapRgb8bit<<VideoMappingShift));

	wrDpMain(DpPolarity, (rdDpMain(DpPolarity) & ~(VsyncPolarity|HsyncPolarity))
		| VsyncPolarity | HsyncPolarity);

	wrDpMain(DpVactiveVblank, (ModeVactive<<16) | (ModeVtotal-ModeVactive));
	wrDpMain(DpHblankHactive, ((ModeHtotal-ModeHactive)<<16) | ModeHactive);
	wrDpMain(DpFrontHsync, (ModeHfp<<16) | ModeHsync);
	wrDpMain(DpFrontVsync, (ModeVfp<<16) | ModeVsync);
	wrDpMain(DpStartxy, ((ModeHsync+ModeHbp)<<16) | (ModeVsync+ModeVbp));

	wrDpMain(DpMisc0, Misc0Rgb8bit);
	wrDpMain(DpNvidMisc1, 0);	/* NVID=0, MISC1=0: confirmed constant in vendor driver */

	/* TU configuration -- see enum comment above for the formula and
	 * computed values. */
	wrDpMain(DpTuReg, (rdDpMain(DpTuReg) & ~((0x7Ful<<TuIntShift)|(0xFul<<TuFracShift)|(0x1FFul<<InitThreshShift)))
		| (TuInt<<TuIntShift) | (TuFrac<<TuFracShift) | (InitThresh<<InitThreshShift));
	wrDpMain(DpHblankInterval, (rdDpMain(DpHblankInterval) & ~(0xFFFFul<<HblankIntervalShift))
		| (HblankLinkCyc<<HblankIntervalShift));

	/* three more registers Linux writes here -- see enum comment
	 * above. REG_VID_CLK_SEL is a separate pixel-clock-source
	 * selector (0x0028), forced to "external" (0), matching what
	 * Linux always writes regardless of use_ext_pixel_clock. */
	wrDpMain(DpPhyifCtrlRegDm, rdDpMain(DpPhyifCtrlRegDm) | PhySscDisBit);
	wrDpMain(RegVidClkSel, rdDpMain(RegVidClkSel) & ~RegVidClkSelBit);
	wrDpMain(VidBistEn, 0);

	v = rdDpMain(DpVideoCtrlRegDm);
	v |= VideoStreamEnDm;
	wrDpMain(DpVideoCtrlRegDm, v);

	print("dpuk3dp: done, video stream enable set (DpVideoCtrlRegDm %#lux)\n", rdDpMain(DpVideoCtrlRegDm));
}

/*
 * ============================================================
 * Section: DP1 AUX channel transactions (originally dpuk3aux.c)
 * ============================================================
 */

/*
 * Spacemit K3 DP1 controller: native AUX channel read/write (DPCD
 * access). Foundation for real link training (PHY/PLL setup,
 * clock-recovery and channel-equalization training all read/write
 * DPCD over this same channel).
 *
 * Register source: Linux drivers/gpu/drm/spacemit/spacemit_inno_dp.h
 * (spacemit-com/linux-6.18), SOC_DPTX_AUX_* macros for offsets/bit
 * ranges. The transaction sequence (program length/address/cmd,
 * pulse AUX_START, poll the reply-event interrupt bit, read status,
 * unpack data) is modeled on soc_dp_aux_transfer() in
 * spacemit_inno_dp.c. AUX_CMD_TYPE values (native write = 8, native
 * read = 9) are plain VESA DisplayPort AUX protocol constants.
 *
 * The DATA1..DATA4 register-to-byte-range mapping is NOT in the
 * order the register addresses might suggest: DATA1 (bits[31:0] of
 * the 16-byte payload, the FIRST four bytes) sits at the HIGHEST
 * address (0x0418); DATA4 (the last four bytes) sits at the LOWEST
 * address (0x040c). Preserved exactly as found.
 *
 * dpuk3auxread()/dpuk3auxwrite() return -1 on any failure and leave
 * retry policy to the caller; nothing here auto-retries with an
 * AUX_RESET pulse the way the vendor driver's hw_reset() does.
 */

static uchar *dpAux;

static ulong
rdAux(int off)
{
	return *(ulong*)(dpAux + off);
}

static void
wrAux(int off, ulong v)
{
	*(ulong*)(dpAux + off) = v;
	coherence();
}

static int
mappedAux(void)
{
	if(dpAux != nil)
		return 1;
	dpAux = vmap(DpPhysRegAx, DpSizeRegAx);
	if(dpAux == nil)
		print("dpuk3aux: vmap %#llux failed\n", DpPhysRegAx);
	return dpAux != nil;
}

/*
 * Shared transfer core. cmd is AuxCmdWrite or AuxCmdRead. buf/len is
 * the payload to send (write) or the buffer to fill (read); len must
 * be 1..AuxMaxLen. addr is the 20-bit DPCD address.
 *
 * Returns 0 on ACK, -1 on NACK/DEFER/error/timeout -- the caller
 * decides whether/how to retry.
 */
static int
auxxfer(int cmd, ulong addr, uchar *buf, int len)
{
	ulong data[4], v;
	int i, tries;

	if(len < 1 || len > AuxMaxLen)
		return -1;
	if(!mappedAux())
		return -1;

	if(cmd == AuxCmdWrite){
		memset(data, 0, sizeof data);
		for(i = 0; i < len; i++)
			data[i/4] |= (ulong)buf[i] << ((i%4)*8);
		wrAux(DpAuxData1, data[0]);
		wrAux(DpAuxData2, data[1]);
		wrAux(DpAuxData3, data[2]);
		wrAux(DpAuxData4, data[3]);
	}

	/* Split into three separate register writes (LENGTH, then ADDR,
	 * then CMD_TYPE last), matching Linux's soc_dp_aux_transfer()
	 * exactly. All three subfields live in DpAuxCfg (0x0404): the
	 * hardware may latch/arm the transaction specifically on the
	 * CMD_TYPE write, so three separate bus writes are used instead
	 * of one combined 32-bit write. */
	wrAux(DpAuxCfg, (rdAux(DpAuxCfg) & ~(0xFul<<AuxLenShift)) | ((ulong)(len-1)<<AuxLenShift));
	wrAux(DpAuxCfg, (rdAux(DpAuxCfg) & ~(0xFFFFFul<<AuxAddrShift)) | (addr<<AuxAddrShift));
	wrAux(DpAuxCfg, (rdAux(DpAuxCfg) & ~AuxCmdMask) | (cmd&AuxCmdMask));

	wrAux(DpAuxStart, 0);
	wrAux(DpAuxStart, AuxStartBit);

	for(tries = AuxTimeoutMs*10; tries > 0; tries--){
		if(rdAux(DpAuxIntStaRegAx) & AuxReplyEventAx)
			break;
		microdelay(100);	/* matches vendor driver's usleep_range(100,110) */
	}
	if(tries == 0){
		print("dpuk3aux: timeout cmd %d addr %#lux len %d\n", cmd, addr, len);
		return -1;
	}

	v = rdAux(DpAuxReply);
	wrAux(DpAuxIntStaRegAx, AuxReplyEventAx);	/* W1C */

	switch((v & AuxReplyStatusMask) >> AuxReplyStatusShift){
	case AuxAck:
		break;
	default:
		print("dpuk3aux: reply status %#lux (errcode %#lux) cmd %d addr %#lux len %d\n",
			(v & AuxReplyStatusMask) >> AuxReplyStatusShift,
			(v & AuxErrCodeMask) >> AuxErrCodeShift, cmd, addr, len);
		return -1;
	}

	if(cmd == AuxCmdRead){
		data[0] = rdAux(DpAuxData1);
		data[1] = rdAux(DpAuxData2);
		data[2] = rdAux(DpAuxData3);
		data[3] = rdAux(DpAuxData4);
		for(i = 0; i < len; i++)
			buf[i] = (data[i/4] >> ((i%4)*8)) & 0xFF;
	}
	return 0;
}

static int
dpuk3auxread(ulong dpcdaddr, uchar *buf, int len)
{
	return auxxfer(AuxCmdRead, dpcdaddr, buf, len);
}

static int
dpuk3auxwrite(ulong dpcdaddr, uchar *buf, int len)
{
	return auxxfer(AuxCmdWrite, dpcdaddr, buf, len);
}

/*
 * ============================================================
 * Section: DP1 real DisplayPort link training, clock recovery + channel eq (originally dpuk3linktrain.c)
 * ============================================================
 */

/*
 * Spacemit K3 DP1 controller: real DisplayPort link training (clock
 * recovery + channel equalization), for DP1 only (2 lanes, HBR
 * 2.7Gbps, matching this codebase's other fixed assumptions).
 *
 * DECISION: follow Linux's protocol-only approach, not U-Boot's.
 * Linux's soc_dp_link_train_clock_recovery()/_channel_eq()
 * (spacemit_inno_dp.c) run the DPCD/AUX protocol but never touch the
 * ANA_TX_* transmitter registers in response to the sink's
 * ADJUST_REQUEST -- they only report a swing/pre-emphasis value to
 * the sink's own DPCD, leaving the transmitter's analog registers at
 * dpuk3devinit.c's fixed values. U-Boot's version does dynamically
 * reconfigure the transmitter via a vol_cfg_table[rate][swing]
 * [preemphasis] lookup, but U-Boot has never been shown to produce
 * real video through this adapter (it bails out on DP++ dongle
 * detection), while Linux is the only software confirmed working on
 * this exact hardware -- so this file matches Linux: clockrecovery()/
 * channeleq() below compute and report ADJUST_REQUEST back to the
 * sink's DPCD but never touch the local transmitter registers.
 *
 * DPCD offsets (DP_TRAINING_PATTERN_SET=0x102, DP_TRAINING_LANE0_SET=
 * 0x103, DP_LANE0_1_STATUS=0x202, DP_LANE_ALIGN_STATUS_UPDATED=0x204,
 * DP_ADJUST_REQUEST_LANE0_1=0x206) and their bit layouts are plain
 * VESA DisplayPort standard constants.
 *
 * TRAINING_AUX_RD_INTERVAL-based delay logic is modeled on U-Boot's
 * soc_dp_link_train_delay() (100us/400us default, or interval*4000us
 * if TRAINING_AUX_RD_INTERVAL is 1-4). Lane count is fixed at 2,
 * matching dpuk3devinit.c's fixed PHY_NUM_LANES=2 setup; this
 * codebase never negotiates lane count.
 *
 * See k3-9front-display-investigation.md for the full history.
 */

static uchar *dpLt;

static ulong
rdLt(int off)
{
	return *(ulong*)(dpLt + off);
}

static void
wrLt(int off, ulong v)
{
	*(ulong*)(dpLt + off) = v;
	coherence();
}

static int
mappedLt(void)
{
	if(dpLt != nil)
		return 1;
	dpLt = vmap(DpPhysRegLt, DpSizeRegLt);
	if(dpLt == nil)
		print("dpuk3linktrain: vmap %#llux failed\n", DpPhysRegLt);
	return dpLt != nil;
}

static void
settrainingpattern(int tp)
{
	uchar dpcdpat;
	ulong v;

	v = rdLt(DpPhyifCtrlRegLt);
	v = (v & ~TpsSelMask) | ((ulong)tp<<TpsSel_SHIFT);
	wrLt(DpPhyifCtrlRegLt, v);

	if(tp == TpDisable)
		wrLt(DpMiscCtrlRegLt, rdLt(DpMiscCtrlRegLt) & ~ScramblerDisableBitLt);
	else
		wrLt(DpMiscCtrlRegLt, rdLt(DpMiscCtrlRegLt) | ScramblerDisableBitLt);

	dpcdpat = tp;
	if(tp != TpDisable)
		dpcdpat |= 1<<5;	/* DP_LINK_SCRAMBLING_DISABLE, standard bit */
	dpuk3auxwrite(DpcdTrainingPatternSet, &dpcdpat, 1);
}

/* TRAINING_AUX_RD_INTERVAL-based delay, matching U-Boot's own
 * soc_dp_link_train_delay(). defaultus is 100 (clock recovery) or
 * 400 (channel eq). */
static void
traindelay(uchar rdinterval, int defaultus)
{
	int interval = rdinterval & 0x7F;

	if(interval == 0){
		microdelay(defaultus);
		return;
	}
	if(interval <= 4){
		microdelay(interval*4000);
		return;
	}
	microdelay(defaultus);
}

/*
 * Full clock-recovery training loop. dpcd is the 16-byte receiver
 * capability block already read by the caller (for
 * TRAINING_AUX_RD_INTERVAL). Returns 0 on success, -1 on failure.
 */
static int
clockrecovery(uchar *dpcd)
{
	uchar trainset[2] = {0, 0};
	uchar status[2];
	uchar v, p;
	int lane, retries;

	if(dpuk3auxwrite(DpcdTrainingLane0Set, trainset, Lanes) < 0)
		return -1;

	settrainingpattern(Tp1);

	for(retries = 0; retries < MaxTrainRetries; retries++){
		traindelay(dpcd[DpcdTrainingAuxRdInterval], 100);

		if(dpuk3auxread(DpcdLane01Status, status, 1) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}

		/* CR_DONE for both lanes, packed 2 lanes/byte, 4 bits each */
		if((status[0] & StatusCrDoneBit) && ((status[0]>>4) & StatusCrDoneBit)){
			return 0;
		}

		if(dpuk3auxread(DpcdAdjustRequestLane01, status, 1) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}

		for(lane = 0; lane < Lanes; lane++){
			uchar nibble = (lane==0) ? status[0]&0xF : (status[0]>>4)&0xF;

			v = nibble & 0x3;
			p = (nibble>>2) & 0x3;
			trainset[lane] = v | (p<<TrainPreEmphasisShift);
			/* NOT applying to the local transmitter (matches Linux,
			 * not U-Boot -- see file header). Only reported to the
			 * sink's own DPCD below. */
		}

		if(dpuk3auxwrite(DpcdTrainingLane0Set, trainset, Lanes) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}
	}

	print("dpuk3linktrain: clock recovery failed after %d retries\n", MaxTrainRetries);
	settrainingpattern(TpDisable);
	return -1;
}

/*
 * Full channel-equalization training loop, run after a successful
 * clockrecovery(). Uses TPS3 if the sink advertises support (DPCD
 * MAX_LANE_COUNT byte, TPS3_SUPPORTED bit), else TPS2. Returns 0 on
 * success, -1 on failure.
 */
static int
channeleq(uchar *dpcd)
{
	uchar trainset[2] = {0, 0};
	uchar status[3];	/* lane01_status, lane23_status(unused), align */
	uchar v, p;
	int lane, retries, tp;

	tp = (dpcd[DpcdMaxLaneCount] & Tps3SupportedBit) ? Tp3 : Tp2;
	print("dpuk3linktrain: channel eq using %s\n", tp==Tp3 ? "TPS3" : "TPS2");
	settrainingpattern(tp);

	for(retries = 0; retries < MaxTrainRetries; retries++){
		traindelay(dpcd[DpcdTrainingAuxRdInterval], 400);

		if(dpuk3auxread(DpcdLane01Status, status, 1) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}
		if(dpuk3auxread(DpcdLaneAlignStatusUpdated, &status[2], 1) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}

		if((status[0] & (StatusCrDoneBit|StatusChannelEqDoneBit|StatusSymbolLockedBit))
			== (StatusCrDoneBit|StatusChannelEqDoneBit|StatusSymbolLockedBit)
		   && ((status[0]>>4) & (StatusCrDoneBit|StatusChannelEqDoneBit|StatusSymbolLockedBit))
			== (StatusCrDoneBit|StatusChannelEqDoneBit|StatusSymbolLockedBit)
		   && (status[2] & InterlaneAlignDoneBit)){
			settrainingpattern(TpDisable);
			return 0;
		}

		if(dpuk3auxread(DpcdAdjustRequestLane01, status, 1) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}

		for(lane = 0; lane < Lanes; lane++){
			uchar nibble = (lane==0) ? status[0]&0xF : (status[0]>>4)&0xF;

			v = nibble & 0x3;
			p = (nibble>>2) & 0x3;
			trainset[lane] = v | (p<<TrainPreEmphasisShift);
			/* NOT applying to the local transmitter (matches Linux,
			 * not U-Boot -- see file header). Only reported to the
			 * sink's own DPCD below. */
		}

		if(dpuk3auxwrite(DpcdTrainingLane0Set, trainset, Lanes) < 0){
			settrainingpattern(TpDisable);
			return -1;
		}
	}

	print("dpuk3linktrain: channel eq failed after %d retries\n", MaxTrainRetries);
	settrainingpattern(TpDisable);
	return -1;
}

/*
 * Full link training entry point: reads the 16-byte receiver
 * capability block itself, then runs clock recovery followed by
 * channel equalization. Returns 0 on success, -1 on failure --
 * caller decides whether to proceed to MSA/video-enable regardless
 * (as Linux itself does).
 */
static int
dpuk3linktrain(void)
{
	uchar dpcd[16];

	if(!mappedLt())
		return -1;

	if(dpuk3auxread(0x0000, dpcd, sizeof dpcd) < 0){
		print("dpuk3linktrain: failed to read receiver cap block\n");
		return -1;
	}
	print("dpuk3linktrain: DPCD rev %#ux max_rate %#ux max_lanes %#ux\n",
		dpcd[0], dpcd[1], dpcd[2]&0x1F);

	if(clockrecovery(dpcd) < 0)
		return -1;
	if(channeleq(dpcd) < 0)
		return -1;

	print("dpuk3linktrain: link training successful\n");
	return 0;
}

/*
 * ============================================================
 * Section: DPU1 mode-set: RDMA/composer/TMG/scene-control (originally dpuk3mode.c)
 * ============================================================
 */

/*
 * Spacemit K3 DPU1 mode-set: program the RDMA read path, composer,
 * scene-control and timing generator for a single static video mode
 * on the DPU1/DP1 pipeline. Reproduces SpacemiT U-Boot's dpu_init()
 * (drivers/video/spacemit/spacemit_dpu.c) field-for-field and in the
 * same order: composer enable and TMG timing first, RDMA/composer
 * detail next, dpu_scene_ctl's enable+commit+start triggers last --
 * order matters here, not just the individual bits.
 *
 * Register source: Linux drivers/gpu/drm/spacemit/dpu/saturn_regs/
 * {rdma_path,composer_x,tmg,dpu_scene_ctl,rdma_top}.h (offsets/field
 * names/widths -- spacemit-com/linux-6.18) cross-checked against the
 * write sequence in SpacemiT's U-Boot dpu_init() (values and order --
 * spacemit-com/uboot-for-k3-display-verify, k3-br-v1.0.y).
 *
 * RDMA0's 0x1000 register (layer_mode/outstanding_num/req_conti_num/
 * layer_cmpsr_id/axi_port_sel/rdma_burst_len packed together) and
 * DMA_TOP's 0x1284 register (round-robin/QoS tuning) are reproduced
 * literally as U-Boot's tested values (0xf00217c, 0x840) rather than
 * individually decoded, likewise the two POST_PIPE bypass-path writes
 * at CMP1+0x300/+0x334.
 *
 * See k3-9front-display-investigation.md for why this exists and how
 * it was found.
 */

static uchar *saturn;

static void
wrMode(int off, ulong v)
{
	*(ulong*)(saturn + off) = v;
	coherence();
}

static void
dpuk3mode(uvlong addr, ulong stride)
{
	saturn = vmap(SaturnPhys, SaturnSize);
	if(saturn == nil){
		print("dpuk3mode: vmap %#llux failed\n", SaturnPhys);
		return;
	}

	/* 1. composer enable, first -- matches u-boot's order */
	wrMode(Cmp1Base+CmpEnable, 1);

	/* 2. TMG timing */
	wrMode(Tmg0Base+TmgHfp, ModeHfpMd<<16);
	wrMode(Tmg0Base+TmgHsBp, ModeHsyncMd | (ModeHbpMd<<16));
	wrMode(Tmg0Base+TmgVfpVs, ModeVfpMd | (ModeVsyncMd<<16));
	wrMode(Tmg0Base+TmgVbpHact, ModeVbpMd | (ModeHactiveMd<<16));
	wrMode(Tmg0Base+TmgVact, ModeVactiveMd);
	wrMode(Tmg0Base+TmgCtrl, TmgCtrlVal);
	wrMode(Tmg0Base+TmgUser, OutfmtRgb888<<12);
	wrMode(Tmg0Base+TmgVsAlignHs, 1);

	/* 3. DMA_TOP tuning (reproduced literally, see STATUS) */
	wrMode(Rdma0Base+DmaTopTune, 0x840);

	/* 4. RDMA0: mode/enable word, address, stride, size, crop, format, fbc */
	wrMode(Rdma0Base+Rdma0Mode, 0xf00217c);
	wrMode(Rdma0Base+Rdma0Addr0Lo, (ulong)addr);
	wrMode(Rdma0Base+Rdma0Addr0Hi, (addr>>32) & 3);
	wrMode(Rdma0Base+Rdma0Stride, stride & 0xFFFF);
	wrMode(Rdma0Base+Rdma0ImgSize, ModeHactiveMd | (ModeVactiveMd<<16));
	wrMode(Rdma0Base+Rdma0BboxStart, 0);
	wrMode(Rdma0Base+Rdma0BboxEnd, (ModeHactiveMd-1) | ((ModeVactiveMd-1)<<16));
	wrMode(Rdma0Base+Rdma0Fmt, FmtXrgb8888);
	wrMode(Rdma0Base+Rdma0FbcMode, 0);
	wrMode(Rdma0Base+Rdma0Fbc, 0x100001e0);

	/* 5. composer: output size, per-layer area/enable, post-pipe bypass */
	wrMode(Cmp1Base+CmpDstSize, ModeHactiveMd | (ModeVactiveMd<<16));
	wrMode(Cmp1Base+CmpLayer00AreaLr, (ModeHactiveMd-1)<<16);
	wrMode(Cmp1Base+CmpLayer00AreaTb, (ModeVactiveMd-1)<<16);
	wrMode(Cmp1Base+CmpLayer00Ctrl, 0xff03);
	wrMode(Cmp1Base+PostPipeBypass, 0);
	wrMode(Cmp1Base+PostPipeSize, ModeHactiveMd | (ModeVactiveMd<<16));

	/* 6. scene control: enable channel/output, then commit, then go --
	 * last, on purpose, matching u-boot's order */
	wrMode(SceneCtrlBase+SceneCtrl0, SceneCtrl0Val);
	wrMode(SceneCtrlBase+SceneCtrl3, SceneCtrl3Val);
	wrMode(SceneCtrlBase+SceneCtrl2, 1);
	wrMode(SceneCtrlBase+SceneCtrl4, 1);

	print("dpuk3mode: done, addr %#llux stride %lud mode %dx%d "
		"(hfp %d hs %d hbp %d vfp %d vs %d vbp %d)\n",
		addr, stride, ModeHactiveMd, ModeVactiveMd,
		ModeHfpMd, ModeHsyncMd, ModeHbpMd, ModeVfpMd, ModeVsyncMd, ModeVbpMd);
}

/*
 * ============================================================
 * Section: DP1 framebuffer physical memory reservation (originally dpuk3fb.c)
 * ============================================================
 */

/*
 * Spacemit K3 framebuffer physical memory: reserve a physically
 * contiguous chunk from the kernel's own xalloc pool for the DPU1
 * RDMA source buffer, instead of depending on any external kexec-time
 * maxmem coordination.
 *
 * xspanalloc()/xalloc() (port/port/xalloc.c, already linked into
 * every 9front kernel) carve their memory from the kernel-reserved
 * slice that xinit() splits off conf.mem at boot -- physically
 * contiguous and properly accounted, so nothing else can ever be
 * handed the same pages. This works identically across the 8/16/32GB
 * K3 Pico-ITX variants (xinit() already sizes the kernel pool
 * correctly via bootargs.c's normal *maxmem-from-DTB path), needing
 * no board-specific constant and no kexec-launcher cooperation.
 *
 * IMPORTANT: xspanalloc() does NOT return nil on failure -- it panics
 * the kernel directly (xalloc.c: "if(a==0) panic(...)"). So
 * dpuk3fbaddr() cannot fail gracefully; if the reservation can't be
 * satisfied, boot stops here with a panic, not a nil return.
 *
 * There is deliberately no separate mapping step for the framebuffer
 * itself: vmap() is only for physical ranges outside the kernel's
 * normal linear RAM mapping (device/MMIO registers). Memory from
 * xspanalloc() is ordinary kernel RAM, already KADDR-mapped the
 * moment it's allocated -- the pointer xspanalloc() returns is kept
 * (fbkaddr) alongside its PADDR() (fbaddr, for the DPU's base_addr
 * register), since writing pixel data needs the kernel-virtual side.
 */

static uvlong fbaddr;
static void *fbkaddr;

/*
 * Reserve the framebuffer's backing memory. Call once, early (from
 * archinit(), alongside dpuk3init()). Safe to call more than once --
 * later calls just return the address from the first.
 */
uvlong
dpuk3fbaddr(void)
{
	void *v;

	if(fbaddr != 0)
		return fbaddr;

	/* panics on failure -- see STATUS note above, not a nil return */
	v = xspanalloc(FbReserve, BY2PG, 0);

	/* Mark this range non-cacheable in place via mmuuncache() (mmu.c).
	 * The display DMAs from here and this SoC is not coherent: without
	 * this, every CPU write would need an explicit cachedwbinvse()
	 * before the DPU can see it -- too costly once devdraw/screeninit()
	 * are driving this buffer continuously. xspanalloc() above already
	 * aligns to BY2PG and FbReserve is BY2PG-aligned by construction,
	 * meeting mmuuncache()'s alignment requirement. */
	if(mmuuncache(v, FbReserve) == nil)
		print("dpuk3fb: mmuuncache failed, framebuffer stays cached "
			"(writes need an explicit cache flush)\n");

	fbkaddr = v;
	fbaddr = PADDR(v);
	print("dpuk3fb: reserved %d bytes, kaddr %#p paddr %#llux\n",
		FbReserve, v, fbaddr);
	return fbaddr;
}

/*
 * The same reservation's kernel-virtual address, for CPU read/write
 * of pixel data (step 8's test pattern, and later devdraw/softscreen
 * use). Must be called after dpuk3fbaddr() has run at least once;
 * returns nil otherwise rather than silently allocating out of order.
 */
void*
dpuk3fbkaddr(void)
{
	return fbkaddr;
}
