/*
 * espik3.c -- SpacemiT K3 eSPI controller driver, carrying a ChromeOS-EC
 * ("cros_ec") host-command tunnel to the CSCE2010 embedded controller,
 * for 9front.
 *
 * REVISION NOTE 3: every constant/struct field this file previously
 * had to guess at (marked "RECALLED, NOT CONFIRMED") has now been
 * checked against the real cros_ec_commands.h (Chromium OS Authors,
 * BSD-style license -- not GPL, its numeric ABI facts are used
 * directly rather than re-derived-and-flagged). Two were wrong:
 * MemmapId was 0x2C, should be 0x20; CmdProtocol3 was 0x88, should be
 * 0xDA. One was a real bug, not just unconfirmed: struct
 * ec_host_response is 8 bytes (struct_version, checksum, result,
 * data_len, AND a trailing 2-byte reserved field), not the 6 bytes
 * this file assumed -- every response payload read would have been
 * offset by 2 bytes and the checksum would never have validated.
 * StatusBusyMask (0x06) and EcCmdPwmSetFanTargetRpm (0x0021) turned
 * out to already be right. See each constant/struct below for where
 * it's now marked CONFIRMED instead of RECALLED.
 *
 * REVISION NOTE 2: the previous revision programmed SLAVE0_CONFIG with
 * a fixed, hand-picked value and skipped straight to using the PR
 * channel window. That was INCOMPLETE in a way that would likely have
 * silently hung or NO_RESPONSE'd on real hardware: per the eSPI Base
 * Spec, a master does not get to unilaterally decide the link is
 * configured just by writing its own CSRs -- it has to actually
 * perform the GET_CONFIGURATION / SET_CONFIGURATION handshake *on the
 * wire* (common eSPI opcodes, K3 UM �14.9.3.2 "Bus Protocol") so the
 * EC's own eSPI slave logic turns its channels on. Writing only the
 * K3 controller's local SLAVE0_CONFIG mirror does not do that -- the
 * slave (EC) side has to be told too, and that only happens via
 * DNCMD_TYPE=SET_CONFIGURATION transactions actually issued to it.
 *
 * This revision replaces espiinit() with a real negotiation: read the
 * EC's General Capabilities (GET_CONFIGURATION @0x08), intersect that
 * with what this board wants, SET_CONFIGURATION each of the four
 * per-channel config registers (@0x10/0x20/0x30/0x40) to turn them on
 * on the EC side, SET_CONFIGURATION the general capabilities register
 * to finalize, and only then mirror the negotiated result into the
 * local SLAVE0_CONFIG/GLOBAL_CONTROL_0/PR_BASE_ADDR_MEM_* CSRs. If any
 * step fails, fall back to a "master-only" config (program our own
 * CSRs to what we want and hope for the best) rather than getting
 * stuck -- same philosophy as before, just with a real handshake
 * attempted first instead of skipped.
 *
 * PROVENANCE: this revision's *facts* -- the per-channel config
 * register addresses and their magic "enable" values, the General
 * Capabilities bit layout used inside GET/SET_CONFIGURATION's 32-bit
 * payload (which is a *different* bit layout than SLAVE0_CONFIG's own
 * CSR, see the translation table below), the exact DN_TXHDR byte-index
 * packing formula, and this board's own negotiated parameters (freq,
 * io-mode, watchdog timeout, which channels, both PR_BASE_ADDR_MEM
 * values) -- were extracted from SpacemiT's real, GPL-2.0-only
 * `spacemit-k3-espi.c` platform driver for this exact SoC/board (the
 * one that owns the eSPI controller and exports
 * spacemit_k3_espi_recover() for cros_ec_espi.c to call). As before,
 * per this tree's copyright policy, none of that GPL code's
 * expression -- its control-flow shape, its struct layout, its
 * error-handling idiom, its variable/function names -- is reproduced
 * below; every function here is freshly written in this tree's own
 * style against the extracted facts (register addresses, bit
 * positions, magic constants, and the *sequence* of standard eSPI-
 * spec operations that sequence performs, which is dictated by the
 * eSPI Base Specification itself, not by this vendor's creativity).
 *
 * This board's specific negotiated parameters (HIGH CONFIDENCE, read
 * directly from k3-pico-itx.dtb's espi@cac8c000 node, cross-checked
 * against what the extracted parsing logic does with each property):
 *	operating-frequency = 20 (MHz)		-> freq select 0
 *	io-mode = "single"			-> io-mode select 0
 *	alert-mode = "io1", NOT "pin"		-> pin-mode alerting is OFF
 *	  (this one matters: the previous revision wrongly forced the
 *	  ALERT_MODE_SEL bit on unconditionally -- this board's own dtb
 *	  says it should stay off, since alert-mode is "io1" not "pin")
 *	watchdog-timeout = 0xffff
 *	crc-enable, peripheral/vwire/oob/flash-channel-enable, and
 *	  auto-gating-enable are all present (booleans default true when
 *	  the property is present at all)
 *	pr-mem-base0 = 0x84000000, pr-mem-base1 = 0xcac8c000
 *
 * Everything else (window layout inside the PR-mapped shared memory,
 * cros_ec host-command v3 framing) is unchanged from the previous
 * revision -- see that revision's notes, still accurate, reproduced
 * in condensed form below.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"

enum {
	/* eSPI controller CSR block -- confirmed, k3-pico-itx.dtb reg */
	EspiPhys	= 0xcac8c000ull,
	EspiSize	= 0x1000,

	/* EC shared-memory window, reached via the PR channel --
	 * confirmed, k3-pico-itx.dtb "ranges" == "pr-mem-base0" */
	EcWinPhys	= 0x84000000ull,
	EcWinSize	= 0x1000,
	PrMemBase1	= 0xcac8c000ull,	/* confirmed, dtb "pr-mem-base1" */

	/* -- eSPI controller CSR offsets, K3 UM 14.9.4.2 -- */
	DnTxHdr0	= 0x00,
	GlobalControl0	= 0x30,
	GlobalControl1	= 0x34,
	PrBaseAddrMem0	= 0x38,
	PrBaseAddrMem1	= 0x3C,
	Slave0Config	= 0x68,
	Slave0IntEn	= 0x6C,
	Slave0IntSts	= 0x70,

	/* -- EC shared-memory window layout, byte offsets -- */
	OffHostCmd	= 0x000,
	OffHostData	= 0x001,
	OffHostPacket	= 0x100,
	OffMemmap	= 0x200,

	/* DNCMD_TYPE, K3 UM 14.9.4.3 == extracted-fact-confirmed */
	DncmdSetConfig	= 0,
	DncmdGetConfig	= 1,
	DncmdInBandRst	= 2,
	DncmdTypeMask	= 7,
	DncmdEn		= 1<<3,

	/* GLOBAL_CONTROL_1 */
	G1EspiRstn	= 1<<5,
	G1SwRst		= 1<<0,

	/* GLOBAL_CONTROL_0 -- only the fields this driver touches */
	G0MstStopEn	= 1<<3,
	G0WdgEn		= 1<<0,
	G0WdgCntShift	= 8,
	G0WdgCntMask	= 0xFFFFu<<8,

	/* SLAVE0_CONFIG (local CSR mirror), K3 UM 14.9.4.3 */
	ScCrcCheckEn	= 1u<<31,
	ScAlertModeSel	= 1<<30,
	ScIoModeShift	= 28,
	ScIoModeMask	= 3<<28,
	ScFreqShift	= 25,
	ScFreqMask	= 7<<25,
	ScPrEn		= 1<<3,
	ScVwEn		= 1<<2,
	ScOobEn		= 1<<1,
	ScFlashEn	= 1<<0,

	/* Slave per-channel/general config register addresses, passed
	 * as the 16-bit "addr" argument to GET/SET_CONFIGURATION. These
	 * are the eSPI Base Spec's own standard configuration register
	 * map (not SpacemiT-specific), confirmed against the extracted
	 * driver's addressing. */
	CfgGeneral	= 0x08,
	CfgPeripheral	= 0x10,
	CfgVwire	= 0x20,
	CfgOob		= 0x30,
	CfgFlash	= 0x40,

	/* Per-channel SET_CONFIGURATION payload to turn each on --
	 * confirmed magic values, meaning not fully decoded but not
	 * needed to be: these are exactly what turns each channel on. */
	EnablePeripheral = 0x1115,
	EnableVwire	= 0x70701,
	EnableOob	= 0x111,
	EnableFlash	= 0x11125,

	/* General Capabilities register bit layout -- used ONLY inside
	 * the 32-bit GET/SET_CONFIGURATION(CfgGeneral,...) payload. This
	 * is a DIFFERENT bit layout than SLAVE0_CONFIG's local CSR
	 * (below is the translation table between the two): */
	GenCrcEnable	= 1u<<31,
	GenAlertModePin	= 1<<28,
	GenOpenDrainAlertSel = 1<<23,
	GenOpenDrainAlertSup = 1<<19,
	GenIoModeSelShift = 26,
	GenIoModeSelMask = 3<<26,
	GenIoModeSupShift = 24,
	GenIoModeSupMask = 3<<24,
	GenFreqSelShift	= 20,
	GenFreqSelMask	= 7<<20,
	GenFreqSupShift	= 16,
	GenFreqSupMask	= 7<<16,
	GenPeriChanSup	= 1<<0,
	GenVwireChanSup = 1<<1,
	GenOobChanSup	= 1<<2,
	GenFlashChanSup = 1<<3,

	FreqSel20MHz	= 0,		/* this board wants 20MHz, single IO */
	IoModeSingle	= 0,

	/* SLAVE0_INT_STS bits actually needed here, K3 UM 14.9.4.3
	 * (register-table facts, independent of either extracted file) */
	IntDncmd	= 1<<28,
	IntProtocolErr	= 1<<15,
	IntRxFlashOflow	= 1<<14,
	IntRxMsgOflow	= 1<<13,
	IntRxOobOflow	= 1<<12,
	IntIllegalLen	= 1<<11,
	IntIllegalTag	= 1<<10,
	IntUnsuccessCpl	= 1<<9,
	IntInvalidCtRsp	= 1<<8,
	IntUnknownRsp	= 1<<7,
	IntNonFatal	= 1<<6,
	IntFatalErr	= 1<<5,
	IntNoRsp	= 1<<4,
	IntCrcErr	= 1<<2,
	IntWaitTimeout	= 1<<1,
	IntBusErr	= 1<<0,

	IntProtocolMask	= IntProtocolErr|IntRxFlashOflow|IntRxMsgOflow|
			  IntRxOobOflow|IntIllegalLen|IntIllegalTag|
			  IntUnsuccessCpl|IntInvalidCtRsp|IntUnknownRsp|
			  IntCrcErr|IntWaitTimeout|IntBusErr,
	IntResponseMask	= IntNonFatal|IntFatalErr|IntNoRsp,
	IntAll		= 0xFFFFFFFFu,

	CfgTimeoutUs	= 50000,
	TxIdleTimeoutUs	= 1000,
	EspiTimeoutUs	= 50000,

	/* CONFIRMED against SpacemiT's real cros_ec_commands.h
	 * (BSD-style, Chromium OS Authors -- not GPL, safe to use its
	 * numeric ABI constants directly): */
	StatusBusyMask	= 0x06,		/* EC_LPC_STATUS_FROM_HOST|_PROCESSING */
	MemmapId	= 0x20,		/* EC_MEMMAP_ID: 'E','C' */
	MemmapSize	= 0xFF,		/* EC_MEMMAP_SIZE-1, for k3ecmemmapread bounds */
	CmdProtocol3	= 0xDA,		/* EC_COMMAND_PROTOCOL_3 */
};

static uchar *espi;	/* controller CSRs */
static uchar *ecwin;	/* EC shared-memory window */
static QLock eclk;

static ulong
erd(int off)
{
	return *(ulong*)(espi + off);
}

static void
ewr(int off, ulong v)
{
	*(ulong*)(espi + off) = v;
	coherence();
}

static uchar
wrd(int off)
{
	uchar v;

	v = *(ecwin + off);
	coherence();
	return v;
}

static void
wwr(int off, uchar v)
{
	*(ecwin + off) = v;
	coherence();
}

static void
wread(int off, uchar *dst, int n)
{
	int i;

	for(i = 0; i < n; i++)
		dst[i] = wrd(off + i);
}

static void
wwrite(int off, uchar *src, int n)
{
	int i;

	for(i = 0; i < n; i++)
		wwr(off + i, src[i]);
}

static uchar
sum8(uchar *p, int n)
{
	uchar s;
	int i;

	s = 0;
	for(i = 0; i < n; i++)
		s += p[i];
	return s;
}

/* ============================================================
 * Low-level DN_TXHDR byte access and common-command issuing.
 * K3 UM 14.9.4.3 register layout, packing formula fact-confirmed
 * against the extracted driver: for header byte index i (0..7),
 * it lives at DN_TXHDR_0 + (((i+1) & ~3)*1), byte-shift ((i+1)&3)*8.
 * (index0..2 pack into DN_TXHDR_0 bits[15:8..31:24], index3..6 into
 * DN_TXHDR_1, index7 into DN_TXHDR_2[7:0] -- matches the K3 UM's
 * register tables exactly once you work the arithmetic through.)
 * ============================================================ */
static void
txhdrwr(int idx, uchar v)
{
	int reg, shift;
	ulong d;

	reg = DnTxHdr0 + ((idx+1) & ~3);
	shift = ((idx+1) & 3) * 8;
	d = erd(reg);
	d &= ~(0xFFul << shift);
	d |= (ulong)v << shift;
	ewr(reg, d);
}

static uchar
txhdrrd(int idx)
{
	int reg, shift;

	reg = DnTxHdr0 + ((idx+1) & ~3);
	shift = ((idx+1) & 3) * 8;
	return (erd(reg) >> shift) & 0xFF;
}

static void
espitrigger(int type)
{
	ulong v;

	v = erd(DnTxHdr0);
	v &= ~(DncmdTypeMask | DncmdEn);
	v |= (type & DncmdTypeMask) | DncmdEn;
	ewr(DnTxHdr0, v);
}

static int
espiwaittxidle(void)
{
	int i;

	for(i = 0; i < TxIdleTimeoutUs; i++){
		if((erd(DnTxHdr0) & DncmdEn) == 0)
			return 0;
		microdelay(1);
	}
	return -1;
}

/*
 * Poll SLAVE0_INT_STS for `expected` to fire, clearing (RW1C) whatever
 * status shows up along the way (including a fresh pre-clear of just
 * the bits we're waiting for, so a stale prior occurrence can't give
 * a false positive). Returns 0 on `expected` firing, -1 on a
 * protocol/response-class error or on timeout.
 */
static int
espipollstatus(ulong expected)
{
	ulong status;
	int i;

	ewr(Slave0IntSts, expected);
	for(i = 0; i < CfgTimeoutUs/10; i++){
		status = erd(Slave0IntSts);
		if(status == 0){
			microdelay(10);
			continue;
		}
		ewr(Slave0IntSts, status);
		if(status & expected)
			return 0;
		if(status & (IntProtocolMask|IntResponseMask))
			return -1;
		microdelay(10);
	}
	return -1;
}

static int
espigetconfig(int addr, ulong *val)
{
	txhdrwr(0, 0);
	txhdrwr(1, addr>>8);
	txhdrwr(2, addr);
	espitrigger(DncmdGetConfig);
	if(espipollstatus(IntDncmd) < 0)
		return -1;
	*val = txhdrrd(3) | (ulong)txhdrrd(4)<<8 |
		(ulong)txhdrrd(5)<<16 | (ulong)txhdrrd(6)<<24;
	return 0;
}

static int
espisetconfig(int addr, ulong val)
{
	txhdrwr(0, 0);
	txhdrwr(1, addr>>8);
	txhdrwr(2, addr);
	txhdrwr(3, val);
	txhdrwr(4, val>>8);
	txhdrwr(5, val>>16);
	txhdrwr(6, val>>24);
	espitrigger(DncmdSetConfig);
	return espipollstatus(IntDncmd);
}

/* ============================================================
 * Controller/link bring-up.
 * ============================================================ */
static void
espihwreset(void)
{
	ewr(GlobalControl1, erd(GlobalControl1) & ~G1EspiRstn);
	ewr(GlobalControl1, erd(GlobalControl1) | G1EspiRstn | G1SwRst);
}

static void
espisendinbandreset(void)
{
	espitrigger(DncmdInBandRst);
	espiwaittxidle();
	delay(1);
}

#define FIELD(mask, shift, val) (((ulong)(val) << (shift)) & (mask))
#define FGET(mask, shift, v) (((v) & (mask)) >> (shift))

/* board-fixed "master wants" general-capabilities word, unnegotiated */
static ulong
espiwantedgencfg(void)
{
	ulong cfg;

	cfg = GenCrcEnable;
	/* alert-mode="io1" on this board, not "pin" -- leave
	 * GenAlertModePin OFF, matching what the extracted parser does
	 * for a non-"pin" alert-mode string */
	cfg |= FIELD(GenIoModeSelMask, GenIoModeSelShift, IoModeSingle);
	cfg |= FIELD(GenFreqSelMask, GenFreqSelShift, FreqSel20MHz);
	cfg |= GenPeriChanSup|GenVwireChanSup|GenOobChanSup|GenFlashChanSup;
	return cfg;
}

/* intersect what we want with what the slave says it supports */
static ulong
espinegotiate(ulong slavecaps)
{
	ulong cfg, iomode, freq;

	cfg = GenCrcEnable;
	iomode = IoModeSingle;
	if(FGET(GenIoModeSupMask, GenIoModeSupShift, slavecaps) < iomode)
		iomode = FGET(GenIoModeSupMask, GenIoModeSupShift, slavecaps);
	freq = FreqSel20MHz;
	if(FGET(GenFreqSupMask, GenFreqSupShift, slavecaps) < freq)
		freq = FGET(GenFreqSupMask, GenFreqSupShift, slavecaps);
	cfg |= FIELD(GenIoModeSelMask, GenIoModeSelShift, iomode);
	cfg |= FIELD(GenFreqSelMask, GenFreqSelShift, freq);
	if(slavecaps & GenPeriChanSup)
		cfg |= GenPeriChanSup;
	if(slavecaps & GenVwireChanSup)
		cfg |= GenVwireChanSup;
	if(slavecaps & GenOobChanSup)
		cfg |= GenOobChanSup;
	if(slavecaps & GenFlashChanSup)
		cfg |= GenFlashChanSup;
	return cfg;
}

static int
espiconfigurechannels(ulong cfg)
{
	if(cfg & GenPeriChanSup)
		if(espisetconfig(CfgPeripheral, EnablePeripheral) < 0)
			return -1;
	if(cfg & GenVwireChanSup)
		if(espisetconfig(CfgVwire, EnableVwire) < 0)
			return -1;
	if(cfg & GenOobChanSup)
		if(espisetconfig(CfgOob, EnableOob) < 0)
			return -1;
	if(cfg & GenFlashChanSup)
		if(espisetconfig(CfgFlash, EnableFlash) < 0)
			return -1;
	return 0;
}

/* translate a negotiated General-Capabilities word into the local
 * SLAVE0_CONFIG CSR's own (different!) bit layout */
static void
espiapplyconfig(ulong cfg)
{
	ulong v, iomode, freq;

	v = erd(Slave0Config);
	v &= ~(ScCrcCheckEn|ScAlertModeSel|ScIoModeMask|ScFreqMask|
		ScPrEn|ScVwEn|ScOobEn|ScFlashEn);

	if(cfg & GenCrcEnable)
		v |= ScCrcCheckEn;
	if(cfg & GenAlertModePin)
		v |= ScAlertModeSel;

	iomode = FGET(GenIoModeSelMask, GenIoModeSelShift, cfg);
	freq = FGET(GenFreqSelMask, GenFreqSelShift, cfg);
	v |= FIELD(ScIoModeMask, ScIoModeShift, iomode);
	v |= FIELD(ScFreqMask, ScFreqShift, freq);

	if(cfg & GenPeriChanSup)
		v |= ScPrEn;
	if(cfg & GenVwireChanSup)
		v |= ScVwEn;
	if(cfg & GenOobChanSup)
		v |= ScOobEn;
	if(cfg & GenFlashChanSup)
		v |= ScFlashEn;

	ewr(Slave0Config, v);
}

static void
espiprogramwindow(void)
{
	ulong v;

	v = erd(GlobalControl0);
	v &= ~(G0WdgCntMask | G0MstStopEn);
	v |= G0WdgEn;
	v |= FIELD(G0WdgCntMask, G0WdgCntShift, 0xFFFF);
	v |= G0MstStopEn;	/* auto-gating-enable present on this board */
	ewr(GlobalControl0, v);

	ewr(PrBaseAddrMem0, EcWinPhys);
	ewr(PrBaseAddrMem1, PrMemBase1);
}

static int
espiinit(void)
{
	ulong slavecaps, cfg;
	int ok;

	espi = vmap(EspiPhys, EspiSize);
	ecwin = vmap(EcWinPhys, EcWinSize);
	if(espi == nil || ecwin == nil){
		espi = ecwin = nil;
		return -1;
	}

	ewr(Slave0IntEn, IntAll);
	ewr(Slave0IntSts, IntAll);

	espihwreset();
	espisendinbandreset();

	ok = espigetconfig(CfgGeneral, &slavecaps) == 0;
	if(ok){
		cfg = espinegotiate(slavecaps);
		if(espiconfigurechannels(cfg) < 0 ||
		   espisetconfig(CfgGeneral, cfg) < 0){
			print("espik3: config handshake failed partway, "
				"falling back to master-only config\n");
			cfg = espiwantedgencfg();
		}
	} else {
		print("espik3: slave capability GET_CONFIGURATION failed "
			"(EC not answering yet?), using master-only config\n");
		cfg = espiwantedgencfg();
	}

	espiapplyconfig(cfg);
	espiprogramwindow();
	return 0;
}

/* ============================================================
 * cros_ec host-command protocol v3, over the PR-channel shared-memory
 * window -- struct layouts now CONFIRMED against the real
 * cros_ec_commands.h (BSD-style, Chromium OS Authors -- not GPL, safe
 * to use its numeric ABI facts directly). struct ec_host_response
 * turned out to have a trailing 2-byte `reserved` field after
 * data_len that the previous revision was missing (it assumed 6
 * bytes, real size is 8) -- that would have desynced every response
 * payload read by 2 bytes and made the checksum check always fail.
 * ============================================================ */
typedef struct EcReq EcReq;
struct EcReq {
	uchar version;
	uchar checksum;
	uchar cmdlo, cmdhi;
	uchar cmdver;
	uchar reserved;
	uchar lenlo, lenhi;
};	/* 8 bytes, matches struct ec_host_request */

typedef struct EcResp EcResp;
struct EcResp {
	uchar version;
	uchar checksum;
	uchar resultlo, resulthi;
	uchar lenlo, lenhi;
	uchar reserved0, reserved1;
};	/* 8 bytes, matches struct ec_host_response */

static int
ecwaitready(void)
{
	int i;

	for(i = 0; i < EspiTimeoutUs; i++){
		if((wrd(OffHostCmd) & StatusBusyMask) == 0)
			return 0;
		microdelay(1);
	}
	return -1;
}

/*
 * k3eccmd() -- send one cros_ec host command, protocol v3, over the
 * PR-channel shared-memory window. Public: any future consumer of
 * this EC (battery, keyboard, more sensors, ...) uses this, not just
 * the fan. See espik3fan.c for the one consumer that exists today.
 */
int
k3eccmd(int cmd, int ver, void *txp, int txlen, void *rxp, int rxlen)
{
	uchar buf[256], rbuf[256];
	EcReq req;
	EcResp resp;
	int n, result;

	if(espi == nil)
		return -1;
	if((int)sizeof(EcReq) + txlen > (int)sizeof buf)
		error("espik3: ec request too big");

	qlock(&eclk);
	if(waserror()){
		qunlock(&eclk);
		nexterror();
	}

	if(ecwaitready() < 0)
		error("espik3: EC busy, timed out waiting for previous command");

	req.version = 3;
	req.checksum = 0;
	req.cmdlo = cmd & 0xFF;
	req.cmdhi = (cmd>>8) & 0xFF;
	req.cmdver = ver;
	req.reserved = 0;
	req.lenlo = txlen & 0xFF;
	req.lenhi = (txlen>>8) & 0xFF;

	memmove(buf, &req, sizeof req);
	if(txlen > 0)
		memmove(buf+sizeof req, txp, txlen);
	buf[1] = -sum8(buf, sizeof req + txlen);

	wwrite(OffHostPacket, buf, sizeof req + txlen);
	wwr(OffHostCmd, CmdProtocol3);

	if(ecwaitready() < 0)
		error("espik3: EC response timeout");

	result = wrd(OffHostData);
	if(result != 0){
		qunlock(&eclk);
		poperror();
		return -1;
	}

	wread(OffHostPacket, rbuf, sizeof resp);
	memmove(&resp, rbuf, sizeof resp);
	n = resp.lenlo | resp.lenhi<<8;
	if(sizeof resp + n > sizeof rbuf)
		error("espik3: EC response too big");
	wread(OffHostPacket + sizeof resp, rbuf + sizeof resp, n);

	if(sum8(rbuf, sizeof resp + n) != 0)
		error("espik3: EC response checksum mismatch");

	if(n > rxlen)
		n = rxlen;
	if(n > 0 && rxp != nil)
		memmove(rxp, rbuf + sizeof resp, n);

	qunlock(&eclk);
	poperror();
	return n;
}


/*
 * k3ecmemmapread() -- generic read from the EC's memmap block
 * (EC_LPC_ADDR_MEMMAP-equivalent, offset OffMemmap within the window).
 * No command round-trip, no busy-wait -- the EC keeps this block live
 * on every plain PR read. `off` is relative to the memmap block
 * itself (e.g. MemmapFan==0x10 for fan speeds), not the window.
 * Public: any future consumer (battery status, switch states, more
 * sensors -- whatever else lives in EC_MEMMAP_*) uses this instead of
 * poking the window directly.
 */
int
k3ecmemmapread(int off, void *buf, int n)
{
	if(espi == nil || off < 0 || off+n > MemmapSize)
		return -1;
	wread(OffMemmap + off, buf, n);
	return n;
}

static int
ecidok(void)
{
	uchar b[2];

	wread(OffMemmap + MemmapId, b, 2);
	return b[0] == 'E' && b[1] == 'C';
}

/*
 * k3espiready() -- bring the eSPI link + EC up on first call (lazy,
 * idempotent, cached), board-gated. Public entry point for ANY
 * consumer of this EC -- fan, or whatever gets added later -- so the
 * board-matching/bring-up logic lives in exactly one place instead of
 * being duplicated per consumer.
 *
 * Board-specific gating, deliberately: dtbsocmatch() does a PREFIX
 * match against the root node's compatible list (bootargs.c), so
 * gating on the SoC-family string "spacemit,k3" would ALSO match
 * other K3 boards this tree knows about -- e.g.
 * "spacemit,k3-com260-ifx" (see i2ck3.c's own header) -- since that
 * string starts with "spacemit,k3" too. Every register/window value
 * in this file (the PR_BASE_ADDR_MEM_0 value, the physical windows,
 * this board's own negotiated eSPI parameters) was pulled from
 * k3-pico-itx.dtb specifically and is NOT known to hold on any other
 * board -- Com260 IFX in particular has no evidence in this tree of
 * wiring its EC the same way (or having a CSCE2010/eSPI EC at all).
 * Match the full board string, not just the SoC family, so this
 * driver only ever activates on k3-pico-itx.
 */
int
k3espiready(void)
{
	static int done, ready;

	if(done)
		return ready;
	done = 1;

	if(!dtbsocmatch("spacemit,k3-pico-itx"))
		return ready = 0;
	if(espiinit() < 0)
		return ready = 0;
	if(!ecidok()){
		print("espik3: no EC id at memmap+%#x -- link came up but EC "
			"isn't answering PR reads (wrong window base, PR "
			"channel not actually enabled on the EC side, or EC "
			"not booted yet)\n", MemmapId);
		return ready = 0;
	}
	return ready = 1;
}
