/*
 * sdufs.c - UFS Host Controller Interface driver for SpacemiT K3
 * skeleton for 9front, devsd/SDifc convention (cf. sdiahci.c)
 *
 * Register map: SpacemiT K3 User Manual ch. 9.7 (UFS Interface),
 * cross-checked against the real DT node in k3.dtsi:
 *   ufshc: ufshc@0xc0e00000 {
 *       compatible = "spacemit,k3-ufshcd";
 *       reg = <0x0 0xc0e00000 0x0 0x40000>;
 *       interrupt-parent = <&saplic>; interrupts = <135 0x4>;
 *       clocks = <&syscon_apmu CLK_APMU_UFS_ACLK>,
 *                <&syscon_apmu CLK_APMU_UFS_REFCLK>;
 *       resets = <&syscon_apmu RESET_APMU_UFS_ACLK>;
 *       freq-table-hz = <491520000 491520000 19200000 19200000>;
 *       lanes-per-direction = <2>;
 *   };
 *
 * M-PHY reference clock is 19.2MHz on this board
 */


#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/error.h"
#include "../port/sd.h"

/*
 * debug: when nonzero, print()s the full per-command/per-step trace
 * this driver used throughout its own bring-up. When zero (the default),
 * only the basics get printed during initialization and
 * anything that's a genuine error or warning, regardless
 * of this flag. Toggle at runtime with:
 *   echo debug 1 >/dev/sdXX/ctl	(enable)
 *   echo debug 0 >/dev/sdXX/ctl	(disable)
 */
static int debug = 0;

/* ---- physical bases from K3 UM 9.7.5, cross-checked against
 * k3.dtsi's "ufshc: ufshc@0xc0e00000" node ----------------------- */
enum {
	UfshcPhys	= 0xC0E00000,
	/* k3.dtsi: reg = <0x0 0xc0e00000 0x0 0x40000>; ONE range
	 * covering both UFSHC and UFS_MNG (UFS_MNG is at +0x1B00,
	 * well inside this window) -- so we vmap it once and reach
	 * UFS_MNG via pointer offset, not a second vmap. */
	UfshcSize	= 0x40000,
};

/* ---- interrupt from K3 UM ch.7.1 (Main CPU Interrupts) ------------ */
enum {
	UfsIrq = 135,	/* aplic_int[135] = ufs_intr */
};

/*
 * UfsMngOff must match the constant of the same name in bootargs.c's
 * Dtbufs handling: the UFS_MNG block sits this far past UFSHC's base
 * per K3 UM 9.7.5 (0xC0E01B00 - 0xC0E00000), used as a fallback when
 * the DTB's "reg" property gives only one address/size pair. Dtbufs
 * comes from dat.h, dtbnufs()/dtbufs() from fns.h.
 */
enum {
	UfsMngOff = 0x1B00,
};

/*
 * APMU (syscon_apmu) -- shared clock/reset syscon referenced by
 * k3.dtsi as "<&syscon_apmu ...>" for both this device and GMAC0.
 *
 * UFS CLOCK/RESET CONTROL REGISTER, K3 UM ch.17.4.2.4 (PMUAP),
 * PMU_UFS_CLK_RST_CTRL offset 0x268 -- this is
 * CLK_APMU_UFS_ACLK / RESET_APMU_UFS_ACLK from k3.dtsi's "clocks"/
 * "resets" properties (clock-names "ufs-aclk", reset-names
 * "ufs-aclk-rst").
 */
enum {
	ApmuPhys	= 0xD4282800,
	ApmuSize	= 0x400,
	Rpmuufsclkrst	= 0x268,
};
enum {
	UfsAclkFcreq	= 1<<8,		/* W1C: request SEL/DIV take effect */
	UfsAclkDivshift	= 5,		/* [7:5] */
	UfsAclkSelshift	= 2,		/* [4:2] */
	UfsAclkSel491M	= 0<<2,		/* matches clock-freq=491520000 in
					 * k3.dtsi (rounded to "491MHz" in
					 * the register description) */
	UfsAclkEn	= 1<<1,		/* 0=Disable 1=Enable */
	UfsAclkRstRel	= 1<<0,		/* 0=Reset 1=Release -- NOTE this
					 * polarity is the OPPOSITE of the
					 * APBCLOCK RST fields elsewhere in
					 * ch.17, which use 0=Release
					 * 1=Reset; don't copy-paste those
					 * idioms here. */
};

/* ---- UFSHC register offsets (9.7.5.1) ----------------------------- */
enum {
	Rhcap		= 0x00,	/* HOST_CONTROLLER_CAPABILITIES  RO */
	Rver		= 0x08,	/* UFS_VERSION                    RO */
	Rhcpid		= 0x10,	/* product id                     RO */
	Rhcmid		= 0x14,	/* manufacturer id                RO */
	Rahit		= 0x18,	/* AUTO_HIBERNATE_IDLE_TIMER      RW */
	Ris		= 0x20,	/* INTERRUPT_STATUS               RWC */
	Rie		= 0x24,	/* INTERRUPT_ENABLE               RWC */
	Rhcs		= 0x30,	/* HOST_CONTROLLER_STATUS         RO */
	Rhce		= 0x34,	/* HOST_CONTROLLER_ENABLE         RW */
	Ruecpa		= 0x38,	/* UIC error PHY adapter layer */
	Ruecdl		= 0x3C,	/* UIC error data link layer */
	Ruecn		= 0x40,	/* UIC error network layer */
	Ruect		= 0x44,	/* UIC error transport layer */
	Ruecdme		= 0x48,	/* UIC error DME */
	Rutriacr	= 0x4C,	/* transfer request interrupt aggregation ctl */
	Rutrlba		= 0x50,	/* UTRL base addr low  (1KB aligned) */
	Rutrlbau	= 0x54,	/* UTRL base addr high */
	Rutrldbr	= 0x58,	/* UTRL door bell */
	Rutrlclr	= 0x5C,	/* UTRL clear */
	Rutrlrsr	= 0x60,	/* UTRL run-stop */
	Rutrlcnr	= 0x64,	/* UTRL completion notification */
	Rutmrlba	= 0x70,	/* UTMRL base addr low (1KB aligned) */
	Rutmrlbau	= 0x74,	/* UTMRL base addr high */
	Rutmrldbr	= 0x78,	/* UTMRL door bell */
	Rutmrlclr	= 0x7C,	/* UTMRL clear */
	Rutmrlrsr	= 0x80,	/* UTMRL run-stop */
	Ruiccmd		= 0x90,	/* UIC command opcode */
	Ruiccmdarg1	= 0x94,
	Ruiccmdarg2	= 0x98,
	Ruiccmdarg3	= 0x9C,
	Rsys1clk1us	= 0xC0,
	Rtxsymclkns1us	= 0xC4,
	Rlocalcportid	= 0xC8,
	Rpaerrcode	= 0xCC,
	Rretrytimer	= 0xD0,
	Rpalinkstarttimer = 0xD8,
};

/* IS/IE bit numbers (9.7.5.1 IS register, same bits in IE) */
enum {
	ISutrcs		= 1<<0,	/* transfer request completion */
	ISudepri	= 1<<1,
	ISue		= 1<<2,	/* uic error */
	ISutms		= 1<<3,
	ISupms		= 1<<4,
	ISuhxs		= 1<<5,
	ISuhes		= 1<<6,
	ISulls		= 1<<7,
	ISulss		= 1<<8,
	ISutmrcs	= 1<<9,	/* task mgmt completion */
	ISuccs		= 1<<10,	/* uic command completion */
	ISdfes		= 1<<11,	/* device fatal error */
	ISutpes		= 1<<12,	/* utp error */
	IShcfes		= 1<<16,	/* host controller fatal error */
	ISsbfes		= 1<<17,	/* system bus fatal error */
};

/* HCS bits (9.7.5.1) */
enum {
	HCSdp		= 1<<0,	/* device present */
	HCSutrlrdy	= 1<<1,
	HCSutmrlrdy	= 1<<2,
	HCSucrdy	= 1<<3,
};

/*
 * UFS_MNG offsets (9.7.5.2) */
enum {
	Mphyrstctrl	= 0x00,
	Mphyputrl	= 0x04,
	Mphybackdoor	= 0x08,
	Mdeviceioctrl	= 0x0C,
};
enum {
	MPHYrstn	= 1<<0,
	MPHYrstncore	= 1<<1,
};
enum {
	Mpupll		= 1<<0,
	Mputx		= 3<<1,
	Mpurx		= 3<<3,
	Mrefclk26M	= 0<<10,
	Mrefclk19M2	= 1<<10,
	Mrefclk38M4	= 2<<10,
	Mrefclk52M	= 3<<10,
};

/*
 * Full-register values written to Mphyputrl during the M-PHY bring-up
 * handshake, taken from the vendor's own Linux driver for this exact
 * controller. Bits 0-6 = Mpupll|Mputx|Mpurx plus two further enable
 * bits (5,6), bits[11:10] = ref-clock-select = Mrefclk19M2 (0x400),
 * matching the board's real 19.2MHz reference clock (see file header).
 * MphyPuWithHb8Reset adds bits 8-9 on top of MphyPuAll, held briefly
 * as a pulse to an internal "ana_rx_hb8_reset" sub-block before
 * returning to MphyPuAll.
 */
enum {
	MphyPuAll		= 0x47f,
	MphyPuWithHb8Reset	= 0x77f,
};

/* Named constant for the ref-clock-select value baked into MphyPuAll/
 * MphyPuWithHb8Reset above */
#define UfsRefClkChoice	Mrefclk19M2

/*
 * How long to wait, after releasing MdevHwRstN, before touching
 * anything else link-related: time for the device's own embedded
 * flash controller firmware to come up. 200ms.
 */
#define UfsPowerOnWaitMs	200

/*
 * UFS_DEVICE_IO_CTRL bits (K3 UM 9.7.5.2, offset 0xC). Both default
 * to 0 at reset: REFCLK_OE=0 means no reference clock reaches the
 * device, and HW_RST_N=0 holds it in hardware reset permanently.
 * Neither is touched by the M-PHY lane/PLL bring-up, which only
 * concerns the host side of the link; the device side needs this too.
 */
enum {
	MdevReon	= 1<<8,	/* REFCLK_OE: 1 = enable REF_CLK to device */
	MdevBypassRstEarly = 1<<1,
	MdevHwRstN	= 1<<0,	/* 0 = reset asserted, 1 = reset deasserted */
};

/*
 * ---- UTP Transfer Request Descriptor (UTRD), 32 bytes ----
 *
 * Layout mandated by JEDEC JESD223 (UFSHCI) -- NOT specific to the
 * K3, and not in the excerpted manual pages. Field names/shapes here
 * are cross-checked against Linux's include/scsi/ufs/ufshci.h
 * (struct request_desc_header / struct utp_transfer_req_desc) purely
 * to confirm the wire layout the spec requires; this is a clean-room
 * reimplementation of that layout (same approach OpenBSD's ufshci.c
 * takes independently under its own BSD license), no GPL code used.
 *
 *   DW0: CT[31:28] (command type), DD[26:25] (data direction),
 *        Int[24] (interrupt on completion) -- see enum below
 *   DW1: reserved (crypto DUNL on some controllers, unused here --
 *        this host controller has no crypto support at all, per
 *        HOST_CONTROLLER_CAPABILITIES.CS=0/OODDS=0)
 *   DW2: OCS[7:0] -- written by hw on completion; sw preloads 0xF
 *   DW3: reserved (crypto DUNU on some controllers, same as DW1)
 *   DW4: UCDBA  -- command descriptor base addr, low 32, must be
 *                  128-byte (0x7f) aligned per spec (bits 6:0 = 0)
 *   DW5: UCDBAU -- command descriptor base addr, high 32
 *   DW6: RUO[15:0] | RUL[15:0]<<16   (offset/length of response UPIU
 *        inside the command descriptor, in units of dwords)
 *   DW7: PRDTO[15:0] | PRDTL[15:0]<<16 (offset/length of PRDT, dwords
 *        for offset, entry-count for length)
 */
typedef struct Utrd Utrd;
struct Utrd
{
	u32int	dw0;
	u32int	dw1;		/* reserved / crypto DUNL */
	u32int	dw2;		/* OCS after completion, bits[3:0], mask 0xF */
	u32int	dw3;		/* reserved / crypto DUNU */
	u32int	ucdba;		/* command descriptor base addr, 128B aligned */
	u32int	ucdbau;		/* command descriptor base addr upper */
	u32int	ru;		/* RUO[15:0] | RUL[15:0]<<16 */
	u32int	prdt;		/* PRDTO[15:0] | PRDTL[15:0]<<16 */
};

/* dw0 fields */
enum {
	UtrdCTufs	= 1<<28,	/* command type = UFS storage (value 1) */
	UtrdDDnone	= 0,
	UtrdDDwrite	= 1<<25,	/* host -> device */
	UtrdDDread	= 1<<26,	/* device -> host */
	UtrdIntr	= 1<<24,	/* interrupt on completion */
};

/* dw2 (OCS) values */
enum {
	Ocssuccess		= 0x0,
	Ocsinvcmdtable		= 0x1,
	Ocsinvprdtattr		= 0x2,
	Ocsmismatchbufsz	= 0x3,
	Ocsmismatchrupiusz	= 0x4,
	Ocspeercommfail		= 0x5,
	Ocsaborted		= 0x6,
	Ocsfatalerror		= 0x7,
	Ocsinvalid		= 0xf,	/* sw preload before ringing doorbell */
	Ocsmask			= 0xf,
};

/*
 * ---- PRDT entry, 16 bytes each ----
 * DW0 base_addr, DW1 upper_addr, DW2 reserved, DW3 size (byte count-1)
 */
typedef struct Prdt Prdt;
struct Prdt
{
	u32int	dbaddr;
	u32int	dbaddru;
	u32int	rsvd;
	u32int	dbc;		/* data byte count - 1, bits 17:0, must be
				 * a multiple of 4 minus 1 (dword granularity) */
};

/*
 * ---- UPIU basic header, 12 bytes, common to all transaction types ----
 * Wire format is big-endian / MSB-first per JEDEC UFS spec, but since
 * these are single-byte fields no swapping is needed when building
 * them with uchar stores on a little-endian host. Multi-byte fields
 * (exp. data transfer length, data segment length) below ARE
 * big-endian and must be stored MSB-first explicitly.
 */
typedef struct Upiuhdr Upiuhdr;
struct Upiuhdr
{
	uchar	transtype;	/* transaction code, see Upiu* enum below */
	uchar	flags;		/* per-transaction-type flag bits */
	uchar	lun;
	uchar	tasktag;
	uchar	iid_cmdset;	/* [7:4] initiator id, [3:0] command set type */
	uchar	query;		/* query task/function, transaction-specific */
	uchar	response;	/* response UPIU only: 0=success */
	uchar	status;		/* response UPIU only: SCSI status byte */
	uchar	totalehslen;
	uchar	devinfo;
	uchar	datasegcnt_hi;	/* big-endian u16 data segment length */
	uchar	datasegcnt_lo;
};

/* Command UPIU (request) continues after Upiuhdr with:
 *   DW3: expected data transfer length, big-endian u32
 *   DW4..: CDB, up to 16 bytes (longer CDBs need EHS, unsupported here)
 * offset 12 = DW3, offset 16 = CDB start. */
enum {
	OffExpXferLen	= 12,
	OffCdb		= 16,
	CdbMaxlen	= 16,
};

static void
putbe32(uchar *p, u32int v)
{
	p[0] = v>>24; p[1] = v>>16; p[2] = v>>8; p[3] = v;
}

enum {
	Maxutr		= 32,	/* NUTRS+1 from HCAP; K3 reset value 0x1f -> 32 slots */
	Maxutmr		= 8,
	Maxprd		= 1,	/* PRDT entries per command; 1 = single contiguous buffer */

	/* Command descriptor layout, chosen by us (offsets are told to
	 * hw via UTRD.RUO/RUL/PRDTO/PRDTL, so any 4-byte-aligned layout
	 * works -- this one mirrors the compact EDK2/OpenBSD style
	 * rather than Linux's fixed 512B/512B slots). */
	CmdUpiuOff	= 0,
	CmdUpiuLen	= 32,			/* hdr(12)+explen(4)+cdb(16) */
	RespUpiuOff	= CmdUpiuLen,		/* must be dword-aligned */
	/*
	 * Was 32 (8 dwords) -- too small. A Response UPIU is hdr(12) +
	 * residual xfer count(4) + reserved(4) + sense data length(2) +
	 * sense data(up to 18) = up to 40 bytes worst case (e.g. a CHECK
	 * CONDITION with full sense data, which is a completely routine
	 * first response from any SCSI/UFS device right after link/power
	 * up -- a "power on occurred" Unit Attention on the very first
	 * INQUIRY is standard, expected behavior, not a device fault).
	 * 32 bytes silently truncated that case, and the controller
	 * reports exactly that mismatch back as OCS=MISMATCH_RESPONSE_
	 * UPIU_SIZE (0x4) instead of executing the transfer -- confirmed
	 * against a real boot log. Bumped to 64 (16 dwords) for headroom.
	 */
	RespUpiuLen	= 64,
	PrdtOff		= RespUpiuOff + RespUpiuLen,
	Cdbsize		= PrdtOff + Maxprd*sizeof(Prdt),
	Scratchbufsize	= 4096,	/* see ufsrio()'s scratch-buffer mitigation */
};

typedef struct Ctlr Ctlr;
struct Ctlr
{
	Lock;
	uintptr	hcreg;		/* kernel VA mapped over UfshcPhys */
	uintptr	mngreg;		/* = hcreg + UfsMngOff, NOT a separate
				 * vmap (single physical reg range, see
				 * UfshcSize comment above) */
	uintptr	apmureg;	/* kernel VA mapped over ApmuPhys */

	Utrd	*utrl;		/* 1KB aligned, Maxutr entries */
	uchar	*cmddesc[Maxutr];	/* per-slot command descriptor memory */
	uchar	*scratchbuf;	/* page-sized DMA target for small reads --
				 * see the comment above Scratchbufsize */
	Rendez	slotr[Maxutr];
	uint	busy;		/* bitmap of slots pending */
	volatile int linereset;	/* set by ufsintr() on a UECPA "generic PHY
				 * adapter error" (LINERESET indication, per
				 * K3 UM 0x38 EC bit4/one-hot) -- the manual
				 * says M-PHY attributes must be restored
				 * after this; checked and acted on by
				 * ufsrio() before each command, see
				 * ufsrestorephy(). */

	SDev	*sdev;
};

#define HCR(c, r)	(*(u32int*)((c)->hcreg + (r)))
#define MNG(c, r)	(*(u32int*)((c)->mngreg + (r)))
#define APMU(c, r)	(*(u32int*)((c)->apmureg + (r)))
/*
 * ATOP ("analog top") block, per the vendor Linux driver: base offset
 * 0x1C00 from hcreg, i.e. mngreg+0x100 since mngreg=hcreg+UfsMngOff
 * (UfsMngOff=0x1B00, 0x1C00-0x1B00=0x100). Word-indexed once unlocked
 * via Mphybackdoor -- the vendor driver addresses entries as
 * ATOP_BASE + (index << 2), which this macro takes 'r' as already
 * being (index << 2) to keep call sites matching that convention.
 */
#define ATOP(c, r)	(*(u32int*)((c)->mngreg + 0x100 + (r)))

static void
ufsintr(Ureg*, void *arg)
{
	Ctlr *c = arg;
	u32int is;

	is = HCR(c, Ris);
	if(debug)
		print("ufs: ufsintr entered, IS=%#ux\n", is);
	if(is == 0)
		return;
	HCR(c, Ris) = is;		/* RWC: write-1-to-clear */

	if(is & ISutrcs){
		/* walk UTRLDBR vs our busy bitmap, wake completed slots */
		u32int db = HCR(c, Rutrldbr);
		uint done = c->busy & ~db;
		uint i;
		for(i = 0; i < Maxutr; i++)
			if(done & (1<<i)){
				c->busy &= ~(1<<i);
				wakeup(&c->slotr[i]);
			}
	}
	if(is & (ISue|ISdfes|ISutpes|IShcfes|ISsbfes)){
		/*
		 * Read UECPA/UECDL exactly ONCE each and reuse the stored
		 * value everywhere below: both are documented ROC (Read-to-
		 * Clear) in the K3 UM, so a second read anywhere else in this
		 * function would always see 0.
		 */
		u32int uecpa = HCR(c, Ruecpa);
		u32int uecdl = HCR(c, Ruecdl);
		print("ufs: error IS=%#ux UECPA=%#ux UECDL=%#ux UECN=%#ux UECT=%#ux "
			"HCS=%#ux\n", is, uecpa, uecdl, HCR(c, Ruecn),
			HCR(c, Ruect), HCR(c, Rhcs));
		/*
		 * K3 UM HOST_CONTROLLER_STATUS (offset 0x30), bits[31:12]
		 * UTP_ERROR_STATUS, valid only when UTPES is set: target LUN
		 * (31:24), task tag (23:16), UTP error code (15:12; only
		 * 1h=Invalid UPIU type is currently defined).
		 */
		if(is & ISutpes){
			u32int hcs = HCR(c, Rhcs);
			print("ufs: UTPES set -- HCS.UTP_ERROR_STATUS: "
				"target_lun=%#ux task_tag=%#ux utp_error_code=%#ux "
				"(0=Reserved 1=Invalid UPIU type 2-Fh=Reserved)\n",
				(hcs>>24)&0xff, (hcs>>16)&0xff, (hcs>>12)&0xf);
		}
		/*
		 * PA_ERROR_CODE (0xCC) gives further detail specifically when
		 * UECDL bit14 (PA_INIT_ERROR, EC=13 per the K3 UM's own error
		 * table -- "00-BAD PHY SYMBOL, 01-UNMAPPED_PHY_ESC_SYMBOL,
		 * 02-UNEXPECTED_PHY_ESC_SYMBOL, 03-BAD_PA_PARAM") is set.
		 */
		if((uecdl & 0x80000000) && (uecdl & (1<<13)))
			print("ufs: PA_ERROR_CODE=%#ux (0=BAD_PHY_SYMBOL "
				"1=UNMAPPED_PHY_ESC_SYMBOL 2=UNEXPECTED_PHY_ESC_SYMBOL "
				"3=BAD_PA_PARAM)\n", HCR(c, Rpaerrcode) & 0xf);
		{
			/*
			 * UECPA "generic PHY adapter error" (LINERESET
			 * indication) -- K3 UM's own EC field description
			 * for this bit says "This should be the LINERESET
			 * indication... Software is informed that MPHY has
			 * been reset and that all M-PHY attributes (that are
			 * not handled by UniPro) need to be restored in
			 * order to keep the link optimized." EC is very
			 * likely one-hot (bit4=code 4), not a sequential
			 * enum (per this file's own long-standing note on
			 * UECPA), so check both interpretations defensively:
			 * EC==4 (sequential) or bit4 set (one-hot). Uses the
			 * SAME uecpa value read once above -- see this
			 * block's comment.
			 */
			u32int ec = uecpa & 0x1f;
			if((uecpa & 0x80000000) && (ec == 4 || (ec & (1<<4)))){
				c->linereset = 1;
				coherence();	/* multicore visibility: this may
						 * run on a different hart than
						 * whichever one later calls
						 * ufsrio() and reads this flag */
			}
		}
	}
}

/*
 * Forward declaration: ufsuniproinit() is defined later in this file
 * (it uses ufsdmeset()/ufsdmesetsel(), which are defined after
 * ufsinit()), but must be called from within ufsinit()'s cold-init
 * path, right after HCE=1 succeeds.
 */
static int ufsuniproinit(Ctlr *c);

/*
 * 9.7.4.1 Host Controller Initialization, verbatim sequence from
 * the K3 manual:
 *   1. write HCE=1, wait for HCE to autoclear-then-read-1
 *   2. send DME_LINKSTARTUP UIC command
 *   3. poll IS.UCCS / HCS.DP
 */
static int
ufsinit(Ctlr *c)
{
	int i, warm;

	if(debug){
		print("ufs: ufsinit entry, hcreg=%#p mngreg=%#p\n", c->hcreg, c->mngreg);
		print("ufs: raw APMU Rpmuufsclkrst on entry = %#ux (En=%d RstRel=%d)\n",
			APMU(c, Rpmuufsclkrst),
			(APMU(c, Rpmuufsclkrst) & UfsAclkEn) != 0,
			(APMU(c, Rpmuufsclkrst) & UfsAclkRstRel) != 0);
	}

	/*
	 * WARM-BOOT PATH: if we got here via kexec from a Linux that was
	 * itself running off this same UFS device, the link is already
	 * trained and live (HCE=1, HCS.DP=1). Re-running the normal
	 * cold-init sequence (device reset, M-PHY reset, HCE toggle,
	 * DME_LINKSTARTUP) tears down a working link and produces
	 * persistent LINERESET errors, since it's fighting a live link
	 * rather than starting cold. When HCE and HCS.DP both read as
	 * already set, skip every electrical/link step and go straight
	 * to programming the UTRL base and allocating command
	 * descriptors, which this OS image still needs fresh regardless
	 * of whether the link itself was already up.
	 */
	/*
	 * UFS ACLK enable + reset release, K3 UM ch.17.4.2.4 (PMUAP),
	 * PMU_UFS_CLK_RST_CTRL @ 0x268 -- CLK_APMU_UFS_ACLK /
	 * RESET_APMU_UFS_ACLK from k3.dtsi. Without this the AXI/APB
	 * fabric to the UFSHC register block may not be clocked, so
	 * every HCR()/MNG() access -- including the warm-boot check
	 * below -- could read back garbage. Must run before the first
	 * HCR()/MNG() touch on both paths, since even a warm-boot kexec
	 * handoff can land here with APMU still in its power-on-reset
	 * state.
	 *
	 * Sequence: program SEL/DIV while still in reset, kick FC_REQ
	 * and wait for hw to clear it, enable the clock, then release
	 * reset. UFS_ACLK_RST is 0=Reset/1=Release -- the opposite
	 * polarity from the APBCLOCK RST fields used elsewhere in this
	 * manual chapter.
	 *
	 * Only run this if the clock isn't already enabled+released:
	 * UfsAclkFcreq is a live frequency-change handshake, and issuing
	 * it unconditionally reprograms/re-kicks the clock even when
	 * already running, which glitches the M-PHY/UniPro link on a
	 * warm-boot handoff (confirmed: drops HCE/HCS.DP back to 0).
	 */
	if((APMU(c, Rpmuufsclkrst) & (UfsAclkEn|UfsAclkRstRel)) !=
	    (UfsAclkEn|UfsAclkRstRel)){
		APMU(c, Rpmuufsclkrst) = (APMU(c, Rpmuufsclkrst) &
			~((7<<UfsAclkDivshift)|(7<<UfsAclkSelshift))) | UfsAclkSel491M;
		APMU(c, Rpmuufsclkrst) |= UfsAclkFcreq;
		for(i = 0; i < 1000 && (APMU(c, Rpmuufsclkrst) & UfsAclkFcreq); i++)
			microdelay(10);
		APMU(c, Rpmuufsclkrst) |= UfsAclkEn;
		APMU(c, Rpmuufsclkrst) |= UfsAclkRstRel;
		if(debug)
			print("ufs: Rpmuufsclkrst readback after our own enable sequence = "
				"%#ux (En=%d RstRel=%d) -- if En=0 here, the write to this "
				"register is being dropped (K3 DMSC resource firewall is "
				"the leading suspect)\n", APMU(c, Rpmuufsclkrst),
				(APMU(c, Rpmuufsclkrst) & UfsAclkEn) != 0,
				(APMU(c, Rpmuufsclkrst) & UfsAclkRstRel) != 0);
	}

	/*
	 * Timer calibration registers (K3 UM 9.7.5, not part of the base
	 * UFS HCI spec): internal host-controller timeouts, including
	 * whatever governs DME_LINKSTARTUP's response wait, are computed
	 * from these. Written unconditionally on both warm and cold
	 * paths -- pure informational constants with no reset semantics.
	 *
	 * Rsys1clk1us = ACLK rate / 1_000_000, truncating integer
	 * division per the vendor driver (491520000/1000000 = 491).
	 * Rtxsymclkns1us and Rpalinkstarttimer use the vendor's own
	 * derivation formulas:
	 *   Rtxsymclkns1us = round(1000/491) << 10 = 0x800
	 *   Rpalinkstarttimer = (491 * 100000) & ~0xf = 0x2ED34E0
	 */
	HCR(c, Rsys1clk1us) = 491;
	HCR(c, Rtxsymclkns1us) = 0x800;
	HCR(c, Rpalinkstarttimer) = 0x2ED34E0;
	if(debug)
		print("ufs: programmed Rsys1clk1us=%ud Rtxsymclkns1us=%#ux "
			"Rpalinkstarttimer=%#ux (all were dead/unwritten code before)\n",
			HCR(c, Rsys1clk1us), HCR(c, Rtxsymclkns1us),
			HCR(c, Rpalinkstarttimer));

	/*
	 * RETRY_TIMER (0xD0): count of automatic Data Link layer retries
	 * for NAC/replay-timer-expired frames. K3 UM reset default is 2;
	 * raised to 16 for extra margin against recoverable frame-level
	 * errors.
	 */
	HCR(c, Rretrytimer) = 16;
	if(debug)
		print("ufs: programmed Rretrytimer=%ud\n", HCR(c, Rretrytimer));

	warm = (HCR(c, Rhce) & 1) != 0 && (HCR(c, Rhcs) & HCSdp) != 0;
	if(debug){
		print("ufs: warm-boot check done, warm=%d Rhce=%#ux Rhcs=%#ux\n",
			warm, HCR(c, Rhce), HCR(c, Rhcs));
		if(warm)
			print("ufs: link already trained and up (warm boot via kexec) -- "
				"skipping cold-init, HCS=%#ux\n", HCR(c, Rhcs));
	}
	if(!warm){

	/*
	 * ACLK reset PULSE (assert, ~1us, deassert), per the vendor's
	 * Linux ufs_spacemit_platform_init(): done unconditionally at the
	 * start of every cold probe, before mphy_init. Resets internal
	 * stateful logic (sequencers, FIFOs) scoped to this clock domain
	 * that a plain "ensure released" write does not touch. Cold-path
	 * only: applying this on the warm (kexec-preserved-link) path
	 * would destroy the link state that path exists to preserve.
	 */
	APMU(c, Rpmuufsclkrst) &= ~UfsAclkRstRel;
	microdelay(1);
	APMU(c, Rpmuufsclkrst) |= UfsAclkRstRel;
	if(debug)
		print("ufs: ACLK reset pulsed (cold path), Rpmuufsclkrst=%#ux\n",
			APMU(c, Rpmuufsclkrst));

	/*
	 * M-PHY power-up handshake and post-lock ATOP calibration, per
	 * the vendor's Linux glue driver (ufs-spacemit) for this exact
	 * controller. Not a single register write:
	 *   1. assert PHY + core reset together
	 *   2. power up PLL+TX+RX (MphyPuAll = 0x47f, ref-clock-select =
	 *      19.2MHz -- see MphyPuAll's own comment for why this
	 *      changed from the original 0x87f/38.4MHz)
	 *   3. pulse an internal ana_rx_hb8_reset sub-block
	 *      (MphyPuWithHb8Reset = 0x77f, then back to MphyPuAll)
	 *   4. deassert device reset + enable REFCLK_OE (0x101)
	 *   5. wait for PLL_LOCK
	 * then unlock the ATOP ("analog top") backdoor register and zero
	 * two analog trim registers through it before re-locking --
	 * silicon calibration that DME_LINKSTARTUP depends on but never
	 * checks itself.
	 *
	 * Magic constants (MphyPuAll, MphyPuWithHb8Reset, 0x101) are
	 * vendor register values, not copied code; this implementation
	 * is original.
	 */
	MNG(c, Mphyrstctrl) = 0;			/* assert resets */
	for(i = 0; i < 1000; i++)
		microdelay(1);				/* vendor: mdelay(1) */
	MNG(c, Mphyrstctrl) = MPHYrstn|MPHYrstncore;	/* deassert */

	MNG(c, Mphyputrl) = MphyPuAll;			/* power up all, 0x47f */
	for(i = 0; i < 1000; i++)
		microdelay(1);
	MNG(c, Mphyputrl) = MphyPuWithHb8Reset;	/* assert ana_rx_hb8_reset, 0x77f */
	for(i = 0; i < 1000; i++)
		microdelay(1);
	MNG(c, Mphyputrl) = MphyPuAll;			/* deassert ana_rx_hb8_reset again */
	for(i = 0; i < 1000; i++)
		microdelay(1);

	MNG(c, Mdeviceioctrl) = MdevReon|MdevHwRstN;	/* 0x101: REFCLK_OE|HW_RST_N, no bypass bit */
	for(i = 0; i < 1000; i++)
		microdelay(1);

	if(debug)
		print("ufs: Mdeviceioctrl readback after corrected reset sequence = %#ux "
			"(REFCLK_OE=%d HW_RST_N=%d)\n", MNG(c, Mdeviceioctrl),
			(MNG(c, Mdeviceioctrl) & MdevReon) != 0,
			(MNG(c, Mdeviceioctrl) & MdevHwRstN) != 0);

	for(i = 0; i < 10000 && !(MNG(c, Mphyputrl) & (1<<31)); i++)
		microdelay(1);				/* MPHY_PLL_LOCK, bit31, up to 10ms */
	if(!(MNG(c, Mphyputrl) & (1<<31)))
		print("ufs: WARNING: M-PHY PLL lock timeout, MPHY_PU_CTRL=%#ux\n",
			MNG(c, Mphyputrl));

	MNG(c, Mphybackdoor) = 1;
	microdelay(20);
	ATOP(c, 0xC1<<2) = 0;
	ATOP(c, 0xC2<<2) = 0;
	microdelay(20);
	MNG(c, Mphybackdoor) = 0;
	microdelay(20);

	if(debug)
		print("ufs: M-PHY init sequence (reset/PU handshake/device-reset/"
			"PLL-lock/ATOP-cal) complete\n");

	for(i = 0; i < UfsPowerOnWaitMs; i++)
		microdelay(1000);		/* see UfsPowerOnWaitMs comment above */

	/* step 1: enable host controller */
	HCR(c, Rhce) = 1;
	for(i = 0; i < 10000 && (HCR(c, Rhce) & 1) == 0; i++)
		microdelay(100);
	if((HCR(c, Rhce) & 1) == 0){
		print("ufs: HCE never came ready\n");
		return -1;
	}

	/*
	 * PA-layer/M-PHY UniPro attribute tuning, per the vendor's
	 * spacemit_k3_ufs_unipro_init(): called right after HCE=1 takes
	 * effect and before DME_LINKSTARTUP is attempted below.
	 */
	if(ufsuniproinit(c) < 0)
		print("ufs: WARNING: ufsuniproinit failed, continuing anyway\n");

	} /* end of !warm cold-init block */

	if(debug)
		print("ufs: reached post cold/warm branch, about to alloc UTRL\n");

	/* program UTRL / UTMRL base addresses (must be 1KB aligned) */
	c->utrl = mallocalign(Maxutr*sizeof(Utrd), 1024, 0, 0);
	if(c->utrl == nil)
		return -1;
	if(debug)
		print("ufs: UTRL alloc ok at %#p (phys %#p), programming base regs\n",
			c->utrl, (void*)PADDR(c->utrl));
	HCR(c, Rutrlba) = (u32int)PADDR(c->utrl);
	HCR(c, Rutrlbau) = (u32int)(PADDR(c->utrl)>>32);
	/* UTMRL omitted here for brevity -- same pattern, 8 slots,
	 * 80 bytes/slot per spec */

	/* per-slot command descriptor buffers: must be 128-byte aligned
	 * (UCDBA bits 6:0 are hardwired 0) and DMA-coherent */
	for(i = 0; i < Maxutr; i++){
		c->cmddesc[i] = mallocalign(Cdbsize, 128, 0, 0);
		if(c->cmddesc[i] == nil){
			print("ufs: out of memory for command descriptors\n");
			return -1;
		}
	}

	/*
	 * Scratch buffer for the small-read cache-staleness mitigation
	 * (see ufsrio()'s use of Scratchbufsize): 4096 bytes is the
	 * smallest size confirmed reliable on real hardware -- every
	 * READ(10) request (always 4096 bytes in practice) has landed
	 * correctly, while 8-byte READ CAPACITY requests intermittently
	 * read back stale/never-written data.
	 *
	 * Root cause is NOT confirmed. One candidate considered was a
	 * real, physical L2 cache (per the K3 datasheet: 4MB/cluster,
	 * MOESI) that this kernel has no maintenance path for beyond
	 * Zicbom -- see archrv.c's disabled l2cachejustflush()/
	 * pl2cachedwbinvse()/memcacheflush(). That theory is weaker than
	 * first thought: per the RISC-V Zicbom spec, cbo.inval/clean/
	 * flush are defined to act on the full private cache hierarchy
	 * visible to the issuing hart, not just the nearest (L1) level --
	 * a conformant Zicbom implementation on the X100 should already
	 * reach its own L2, since that L2 sits inside the same coherence
	 * domain as L1 for that core, not behind some separate interface
	 * only reachable via vendor-specific registers. The disabled
	 * archrv.c scaffolding may instead target OTHER RISC-V SoCs in
	 * this kernel tree that don't implement Zicbom at all (pre-RVA23
	 * chips), not a genuine gap on THIS SoC. An untested alternative:
	 * DMA-write-ordering at the interconnect/AXI level between the
	 * UFS controller and the completion signal (UTRLDBR), independent
	 * of caching entirely. This works around the symptom empirically
	 * either way; it does not fix or confirm the actual cause, which
	 * remains open.
	 */
	c->scratchbuf = mallocalign(Scratchbufsize, 128, 0, 0);
	if(c->scratchbuf == nil){
		print("ufs: out of memory for scratch buffer\n");
		return -1;
	}

	/*
	 * Step 2/3: link startup via UIC command, with retries.
	 * Skipped entirely on the warm-boot path -- see the comment
	 * at the top of this function. LINERESET and similar
	 * PA-layer errors on the very first attempt are common if the
	 * device wasn't fully powered up yet despite the wait above
	 * -- this is the standard pattern real UFS host drivers use
	 * (e.g. Linux ufshcd retries DME_LINKSTARTUP up to a few
	 * times before giving up), not something specific to this
	 * driver's other bugs.
	 */
	if(!warm){
	int try, ok, pass;

	/*
	 * OUTER pass loop: per Linux's ufshcd_link_startup(), "If UFS
	 * device isn't active then we will have to issue link startup 2
	 * times to make sure the device state move to active." Neither
	 * U-Boot's nor the SpacemiT K3 vendor Linux driver opts out of
	 * this, so it applies here too. Cold path only -- warm inherits
	 * an already-active link.
	 */
	for(pass = 0; pass < 2; pass++){

	if(debug)
		print("ufs: entering DME_LINKSTARTUP retry loop (pass %d)\n", pass);
	ok = 0;
	for(try = 0; try < 3 && !ok; try++){
		if(try > 0){
			/* clear latched UIC error state before retrying;
			 * ERR bits are ROC (read-to-clear) per 9.7.5.1.
			 * Cast to void: these reads are for their clearing
			 * side effect only, result intentionally discarded. */
			(void)HCR(c, Ruecpa); (void)HCR(c, Ruecdl); (void)HCR(c, Ruecn);
			(void)HCR(c, Ruect); (void)HCR(c, Ruecdme);
			for(i = 0; i < 5; i++)
				microdelay(1000);	/* ~5ms between tries */
		}

		HCR(c, Ruiccmdarg1) = 0;
		HCR(c, Ruiccmdarg2) = 0;
		HCR(c, Ruiccmdarg3) = 0;
		HCR(c, Ruiccmd) = 0x16;	/* DME_LINKSTARTUP opcode, UFS HCI std */
		for(i = 0; i < 10000 && (HCR(c, Ris) & ISuccs) == 0; i++)
			microdelay(100);
		if((HCR(c, Ris) & ISuccs) == 0){
			if(debug)
				print("ufs: DME_LINKSTARTUP try %d timed out, HCS=%#ux\n",
					try, HCR(c, Rhcs));
			continue;
		}
		HCR(c, Ris) = ISuccs;
		ok = (HCR(c, Rhcs) & HCSdp) != 0;
	}

	if(!ok){
		/*
		 * IS.UCCS being set only means the host controller
		 * finished PROCESSING the UIC command -- it does not
		 * by itself mean the command's RESULT was success, and
		 * it says nothing about whether a device answered on
		 * the wire. Dump everything the K3 UM 9.7.5.1 exposes
		 * about UIC/PHY/link state so a real failure mode can
		 * be diagnosed instead of guessing blind:
		 *   - UICCMDARG2: generic UIC command result. Per the
		 *     JEDEC UFSHCI convention (not itself confirmed
		 *     against the excerpted K3 UM pages) bits[15:8] are
		 *     the config/generic result code: 0=SUCCESS,
		 *     1=FAILURE/INVALID_ATTR, 2=INVALID_ATTR_VALUE,
		 *     3=READ_ONLY_ATTR, 4=WRITE_ONLY_ATTR,
		 *     5=BAD_INDEX, 6=LOCKED_ATTR,
		 *     7=BAD_TEST_FEATURE_INDEX, 8=PEER_COMM_FAILURE,
		 *     9=BUSY, 0xA=DME_FAILURE.
		 *   - UECPA/UECDL/UECN/UECT/UECDME: latched UIC error
		 *     codes per layer (9.7.5.1 offsets 0x38-0x48); ERR
		 *     bit31 marks each one valid. NOTE: the EC sub-field
		 *     in UECPA is very likely a ONE-HOT bit mask, not a
		 *     binary-encoded enum, despite the manual listing
		 *     "00/01/02/03/04" like enum values -- e.g. EC=0x10
		 *     (bit4 set) means "Generic PHY Adapter error /
		 *     LINERESET", not literal value 16. This
		 *     reinterpretation is not independently confirmed.
		 *   - PA_ERROR_CODE (0xCC): detail when UECDL bit14
		 *     (PA_INIT_ERROR) is set.
		 *   - MPHY_PLL_LOCK (UFS_MNG 0x4 bit31): whether the
		 *     HOST side M-PHY PLL ever locked at all -- if this
		 *     is 0, the problem is upstream of link training
		 *     entirely (ref clock, power sequencing) and the
		 *     UIC layer diagnostics below are moot.
		 */
		print("ufs: no device present after %d link startup tries\n", try);
		print("ufs:   HCS=%#ux (DP=%d UTRLRDY=%d UTMRLRDY=%d UCRDY=%d)\n",
			HCR(c, Rhcs), HCR(c, Rhcs)&HCSdp,
			(HCR(c, Rhcs)&HCSutrlrdy)!=0, (HCR(c, Rhcs)&HCSutmrlrdy)!=0,
			(HCR(c, Rhcs)&HCSucrdy)!=0);
		print("ufs:   UICCMDARG2=%#ux (result code in bits[15:8])\n",
			HCR(c, Ruiccmdarg2));
		print("ufs:   UECPA=%#ux UECDL=%#ux UECN=%#ux UECT=%#ux UECDME=%#ux\n",
			HCR(c, Ruecpa), HCR(c, Ruecdl), HCR(c, Ruecn),
			HCR(c, Ruect), HCR(c, Ruecdme));
		print("ufs:   PA_ERROR_CODE=%#ux\n", HCR(c, Rpaerrcode));
		print("ufs:   MPHY_PLL_LOCK=%d MPHY_PU_CTRL=%#ux\n",
			(MNG(c, Mphyputrl)>>31)&1, MNG(c, Mphyputrl));
		return -1;
	}
	} /* end of for(pass...) */
	}

	if(debug)
		print("ufs: enabling interrupts and starting UTRL run\n");

	/* enable interrupts we care about */
	HCR(c, Rie) = ISutrcs|ISue|ISdfes|ISutpes|IShcfes|ISsbfes|ISutmrcs|ISuccs;
	if(debug)
		print("ufs: Rie readback = %#ux\n", HCR(c, Rie));

	/*
	 * K3 UM UTP_TRANSFER_REQUEST_LIST_RUN_STOP (offset 0x60):
	 * "This bit [UTRLRSR] shall only be set to '1' when
	 * HCS.UTRLRDY is set to '1'." UTRLRDY is the precondition for
	 * setting UTRLRSR, not a consequence of it -- wait for it first.
	 */
	for(i = 0; i < 10000 && (HCR(c, Rhcs) & HCSutrlrdy) == 0; i++)
		microdelay(100);
	if(debug)
		print("ufs: HCS.UTRLRDY before UTRLRSR=1 = %d (HCS=%#ux)\n",
			(HCR(c, Rhcs) & HCSutrlrdy) != 0, HCR(c, Rhcs));

	/* run-stop the transfer request list */
	HCR(c, Rutrlrsr) = 1;

	if(debug)
		print("ufs: ufsinit done\n");
	return 0;
}

/* forward decl: ufspnp() below references &sdufsifc, but the actual
 * SDifc sdufsifc definition sits at the bottom of this file (it
 * needs ufsrio() etc. already declared/defined first) */
extern SDifc sdufsifc;

/*
 * UPIU transaction codes (JEDEC UFS spec table 9.x). Request side
 * values 0x0-0x1f, response side has bit5 (0x20) set.
 */
enum {
	UpiuNopout	= 0x00,
	UpiuCommand	= 0x01,
	UpiuDataout	= 0x02,
	UpiuTaskmgmtreq	= 0x04,
	UpiuQueryreq	= 0x16,

	UpiuNopin	= 0x20,
	UpiuResponse	= 0x21,
	UpiuDatain	= 0x22,
	UpiuTaskmgmtrsp	= 0x24,
	UpiuQueryrsp	= 0x36,
	UpiuReject	= 0x3f,
};

/*
 * ---- Query Request/Response UPIU support ----
 *
 * Separate from ufsrio()'s SCSI Command UPIU path: Query UPIUs use a
 * completely different transaction-specific field layout. Facts below
 * (field order: opcode/idn/index/selector, then a big-endian 4-byte
 * value with a flag's actual bit in its LAST byte) were confirmed by
 * inspecting the generic Linux UFS core (ufshcd_init_query() and
 * ufshcd_query_flag()) -- not copied as code, just the field layout
 * facts, reimplemented independently here.
 * The specific numeric opcode/idn constants below are standard JEDEC
 * UFS values (not vendor-specific), used identically across every
 * public UFS implementation.
 */
enum {
	UpiuQfuncRead	= 0x01,		/* Standard READ Request */
	UpiuQfuncWrite	= 0x81,		/* Standard WRITE Request */

	QopNop		= 0x00,
	QopReadDesc	= 0x01,
	QopWriteDesc	= 0x02,
	QopReadAttr	= 0x03,
	QopWriteAttr	= 0x04,
	QopReadFlag	= 0x05,
	QopSetFlag	= 0x06,
	QopClearFlag	= 0x07,
	QopToggleFlag	= 0x08,

	FlagIdnDeviceInit = 0x01,	/* fDeviceInit */
};

/*
 * ufswaitslot: wait for a submitted transfer request to complete, per
 * K3 UM 9.7.4.2/9.7.4.3's documented polling-mode protocol: wait for
 * the slot's UTRLDBR bit to clear (hw sets it when the transfer
 * completes, success or error), then check/clear ISutrcs in IS.
 * Shared by every command path (ufsquery, ufsreaddesc, ufsrio).
 * Once this returns 0, it's safe to cachedinvse() and read the
 * UTRD/response UPIU.
 */
static int
ufswaitslot(Ctlr *c, int slot)
{
	int i;

	for(i = 0; i < 100000; i++){
		if((HCR(c, Rutrldbr) & (1<<slot)) == 0)
			break;
		microdelay(100);	/* up to ~10s total, same budget as before */
	}
	if(HCR(c, Rutrldbr) & (1<<slot))
		return -1;	/* timed out: slot never completed */

	/* K3 UM step 6: completion also sets ISutrcs in IS. Check/clear
	 * it defensively even though this driver polls UTRLDBR rather
	 * than relying on the interrupt. */
	if(HCR(c, Ris) & ISutrcs)
		HCR(c, Ris) = ISutrcs;		/* RWC */

	return 0;
}

/*
 * ufsslotalloc/ufsslotfree: allocate/release a UTP transfer request
 * slot. Shared by every command path.
 */
static int
ufsslotalloc(Ctlr *c)
{
	int slot;

	ilock(c);
	for(slot = 0; slot < Maxutr; slot++)
		if((c->busy & (1<<slot)) == 0)
			break;
	if(slot == Maxutr)
		slot = -1;
	else
		c->busy |= 1<<slot;
	iunlock(c);
	return slot;
}

static void
ufsslotfree(Ctlr *c, int slot)
{
	ilock(c);
	c->busy &= ~(1<<slot);
	iunlock(c);
}

/*
 * ufssubmit: fill in a no-PRDT UTRD for slot -- shared by every
 * command path that has no data phase of its own (NOP, Query,
 * descriptor reads). SDreq-driven SCSI commands with a real data
 * phase build their own UTRD+PRDT directly in ufsrio instead. Flushes
 * caches, rings the doorbell, waits for completion (ufswaitslot), and
 * releases the slot. On success, cd's Response UPIU has been freshly
 * invalidated and is ready to read. Callers distinguish the two
 * failure cases (via c->utrl[slot].dw2 for the OCS case) so they can
 * print their own operation-specific diagnostics:
 *   return 0   -- success
 *   return -1  -- timed out (ufswaitslot failed)
 *   return -2  -- OCS error (see c->utrl[slot].dw2)
 */
static int
ufssubmit(Ctlr *c, int slot)
{
	uchar *cd = c->cmddesc[slot];
	Utrd *ut = &c->utrl[slot];

	memset(ut, 0, sizeof *ut);
	ut->dw0 = UtrdCTufs | UtrdIntr | UtrdDDnone;
	ut->dw2 = Ocsinvalid;
	ut->ucdba = (u32int)PADDR(cd);
	ut->ucdbau = (u32int)(PADDR(cd)>>32);
	ut->ru = (RespUpiuLen/4) | ((RespUpiuOff/4)<<16);
	ut->prdt = 0;

	cachedwbse(cd, Cdbsize);
	cachedwbse(ut, sizeof *ut);
	coherence();
	HCR(c, Rutrldbr) = 1<<slot;

	if(ufswaitslot(c, slot) < 0){
		ufsslotfree(c, slot);
		return -1;
	}
	ufsslotfree(c, slot);
	cachedinvse(ut, sizeof *ut);
	if((ut->dw2 & Ocsmask) != Ocssuccess)
		return -2;
	cachedinvse(cd, Cdbsize);
	return 0;
}

/*
 * ufsquery: send one Query Request UPIU (opcode/idn/index/selector),
 * and return the big-endian 4-byte "value" field from the response
 * (flag ops only care about its last byte; descriptor/attribute ops
 * would need more of the response, not needed for the device-init
 * handshake below). Returns -1 on timeout/HCI-level failure, else 0
 * with *value filled.
 */
static int
ufsquery(Ctlr *c, uchar opcode, uchar idn, uchar index, uchar selector,
	u32int *value)
{
	int slot, r;
	uchar *cd;
	Upiuhdr *cmd, *rsp;
	u32int v;

	if((slot = ufsslotalloc(c)) < 0){
		print("ufs: ufsquery: no free slots\n");
		return -1;
	}
	cd = c->cmddesc[slot];
	memset(cd, 0, Cdbsize);

	cmd = (Upiuhdr*)(cd + CmdUpiuOff);
	cmd->transtype = UpiuQueryreq;
	cmd->query = (opcode == QopSetFlag || opcode == QopClearFlag ||
		opcode == QopToggleFlag || opcode == QopWriteAttr ||
		opcode == QopWriteDesc)? UpiuQfuncWrite : UpiuQfuncRead;
	/*
	 * Transaction-specific fields start at byte 12 (same start point
	 * as the Command UPIU's DW3, per the shared 12-byte basic header
	 * both transaction types begin with): opcode, idn, index,
	 * selector, 2 reserved, 2-byte length, then the 4-byte value.
	 */
	cd[CmdUpiuOff+12] = opcode;
	cd[CmdUpiuOff+13] = idn;
	cd[CmdUpiuOff+14] = index;
	cd[CmdUpiuOff+15] = selector;
	if(value != nil && opcode == QopWriteAttr)
		putbe32(cd + CmdUpiuOff + 20, *value);

	r = ufssubmit(c, slot);
	if(r == -1){
		print("ufs: ufsquery opcode=%#ux idn=%#ux TIMED OUT\n", opcode, idn);
		return -1;
	}
	if(r == -2){
		print("ufs: ufsquery opcode=%#ux idn=%#ux OCS error dw2=%#ux\n",
			opcode, idn, c->utrl[slot].dw2);
		return -1;
	}

	rsp = (Upiuhdr*)(cd + RespUpiuOff);
	v = (cd[RespUpiuOff+20]<<24) | (cd[RespUpiuOff+21]<<16) |
		(cd[RespUpiuOff+22]<<8) | cd[RespUpiuOff+23];
	if(debug)
		print("ufs: ufsquery opcode=%#ux idn=%#ux transtype=%#ux "
			"response=%#ux value=%#ux\n",
			opcode, idn, rsp->transtype, rsp->response, v);
	if(value != nil)
		*value = v;
	if(rsp->transtype != UpiuQueryrsp || rsp->response != 0)
		return -1;
	return 0;
}

/*
 * ufsnop: send a NOP OUT UPIU (transtype 0x00, the most basic UPIU
 * transaction type, per JEDEC UFS spec) and confirm a NOP IN response
 * (transtype 0x20). Per the generic Linux UFS core's device-init
 * sequence (ufshcd_device_init() -> ufshcd_verify_dev_init()), this
 * runs first, immediately after DME_LINKSTARTUP succeeds and before
 * fDeviceInit. NOP OUT/IN has no payload beyond the header; the
 * response has no data segment to parse, just transtype+response to
 * confirm the round trip worked.
 */
static int
ufsnop(Ctlr *c)
{
	int slot, r;
	uchar *cd;
	Upiuhdr *rsp;

	if((slot = ufsslotalloc(c)) < 0){
		print("ufs: ufsnop: no free slots\n");
		return -1;
	}
	cd = c->cmddesc[slot];
	memset(cd, 0, Cdbsize);
	/* every other header field (flags/lun/tasktag/iid_cmdset) stays
	 * zero -- NOP OUT carries no meaningful payload of its own */
	((Upiuhdr*)(cd + CmdUpiuOff))->transtype = UpiuNopout;

	r = ufssubmit(c, slot);
	if(r == -1){
		print("ufs: ufsnop: TIMED OUT\n");
		return -1;
	}
	if(r == -2){
		print("ufs: ufsnop: OCS error dw2=%#ux\n", c->utrl[slot].dw2);
		return -1;
	}

	rsp = (Upiuhdr*)(cd + RespUpiuOff);
	if(debug)
		print("ufs: ufsnop: transtype=%#ux response=%#ux\n",
			rsp->transtype, rsp->response);
	return rsp->transtype == UpiuNopin? 0 : -1;
}

/*
 * ufsreaddesc: Query READ_DESCRIPTOR -- structurally different from
 * ufsquery() above because descriptor reads return a variable-length
 * data segment. Copies up to 'len' descriptor bytes into 'buf'; returns
 * bytes actually available (may be less than 'len'), or -1 on failure.
 */
static int
ufsreaddesc(Ctlr *c, uchar idn, uchar index, uchar *buf, int len)
{
	int slot, r, rlen;
	uchar *cd;
	Upiuhdr *cmd, *rsp;

	if(len > RespUpiuLen - 32)
		len = RespUpiuLen - 32;	/* clamp to available response room */

	if((slot = ufsslotalloc(c)) < 0){
		print("ufs: ufsreaddesc: no free slots\n");
		return -1;
	}
	cd = c->cmddesc[slot];
	memset(cd, 0, Cdbsize);

	cmd = (Upiuhdr*)(cd + CmdUpiuOff);
	cmd->transtype = UpiuQueryreq;
	cmd->query = UpiuQfuncRead;
	cd[CmdUpiuOff+12] = QopReadDesc;
	cd[CmdUpiuOff+13] = idn;
	cd[CmdUpiuOff+14] = index;
	cd[CmdUpiuOff+15] = 0;			/* selector */
	cd[CmdUpiuOff+18] = len>>8;		/* Length field, big-endian u16 */
	cd[CmdUpiuOff+19] = len;

	r = ufssubmit(c, slot);
	if(r == -1){
		print("ufs: ufsreaddesc idn=%#ux TIMED OUT\n", idn);
		return -1;
	}
	if(r == -2){
		print("ufs: ufsreaddesc idn=%#ux OCS error dw2=%#ux\n",
			idn, c->utrl[slot].dw2);
		return -1;
	}

	rsp = (Upiuhdr*)(cd + RespUpiuOff);
	rlen = (cd[RespUpiuOff+18]<<8) | cd[RespUpiuOff+19];
	if(debug)
		print("ufs: ufsreaddesc idn=%#ux transtype=%#ux response=%#ux "
			"returned len=%d\n", idn, rsp->transtype, rsp->response, rlen);
	if(rsp->transtype != UpiuQueryrsp || rsp->response != 0)
		return -1;
	if(rlen > len)
		rlen = len;
	memmove(buf, cd + RespUpiuOff + 32, rlen);
	return rlen;
}

/*
 * Command UPIU flags byte: bit5=W(rite), bit6=R(ead), bits[2:0]=task
 * attr. Matches the parity of the transport-level UTRD DD field below
 * (UtrdDDwrite=1<<25, UtrdDDread=1<<26) and the JEDEC UFS Command UPIU
 * field reference. Both the SCSI-level Flags byte and the transport-
 * level UTRD.DD field must agree on data direction for every
 * data-phase command.
 */
enum {
	UpiuFlagWrite	= 1<<5,
	UpiuFlagRead	= 1<<6,
	UpiuFlagSimple	= 0,		/* task attribute: simple queueing */
};

/* iid_cmdset: command set type 0 = SCSI, in low nibble */
enum {
	CmdsetScsi	= 0,
};

/*
 * ---- Generic DME_SET + PA_PWRMODE change ----
 *
 * Facts (attribute IDs, DME_SET opcode 0x02, UPMCRS field at HCS
 * bits[10:8], PWR_LOCAL=1, and the DL_*_Default/PA_PWRMODEUSERDATA*
 * timeout programming sequence) confirmed directly from the real
 * public unipro.h/ufshci.h headers -- not copied as code, only the
 * numeric IDs/values, reimplemented independently here using this
 * driver's own polling-based UIC command dispatch (the same pattern
 * already proven working for DME_LINKSTARTUP).
 */
enum {
	UicDmeSet	= 0x02,

	AttrPaRxGear		= 0x1583,
	AttrPaActiveRxLanes	= 0x1580,
	AttrPaRxTermination	= 0x1584,
	AttrPaTxGear		= 0x1568,
	AttrPaActiveTxLanes	= 0x1560,
	AttrPaTxTermination	= 0x1569,
	AttrPaHsSeries		= 0x156A,
	AttrPaPwrMode		= 0x1571,
	AttrPaPwrModeUserData0	= 0x15B0,
	AttrDmeLocalFc0Prot	= 0xD041,
	AttrDmeLocalTc0Replay	= 0xD042,
	AttrDmeLocalAfc0Req	= 0xD043,

	DlFc0ProtDefault	= 8191,
	DlTc0ReplayDefault	= 65535,
	DlAfc0ReqDefault	= 32767,

	PwrFastMode		= 1,
	PwrFastAutoMode		= 4,
	PwrLocal		= 1,	/* UPMCRS success value */

	/* target config: matches ufs-spacemit.h's own recommended
	 * UFS_SPACEMIT_LIMIT_* values (HS Gear3, 2 lanes, HS Mode B) */
	TargetHsGear		= 3,
	TargetLanes		= 2,
	TargetHsSeries		= 2,	/* PA_HS_MODE_B */
};

/*
 * ufsuiccmd: shared dispatch for every local/peer DME_GET/DME_SET
 * variant below -- issue a UIC command (GenSelectorIndex packed into
 * ARG1 alongside the attribute ID), poll for completion, and (for GET
 * ops) return the result via *value. splhi()/splx() around dispatch+
 * poll avoids a real race confirmed on hardware: once interrupts are
 * enabled, ufsintr() can consume ISuccs before this function's own
 * poll loop sees it, making the poll spin to a false timeout even
 * though the command genuinely completed.
 */
static int
ufsuiccmd(Ctlr *c, u32int op, u32int attr, u32int sel, u32int arg3, u32int *value)
{
	int i, s;

	s = splhi();
	HCR(c, Ris) = ISuccs;	/* clear any stale completion bit from a
				 * previous UIC command before issuing a new
				 * one, or the poll below could return a
				 * false-instant "success" */
	HCR(c, Ruiccmdarg1) = (attr<<16) | sel;
	HCR(c, Ruiccmdarg2) = 0;
	HCR(c, Ruiccmdarg3) = arg3;
	HCR(c, Ruiccmd) = op;
	for(i = 0; i < 10000 && (HCR(c, Ris) & ISuccs) == 0; i++)
		microdelay(100);
	if((HCR(c, Ris) & ISuccs) == 0){
		splx(s);
		print("ufs: UIC cmd op=%#ux attr=%#ux sel=%#ux val=%#ux "
			"timed out\n", op, attr, sel, arg3);
		return -1;
	}
	HCR(c, Ris) = ISuccs;
	if(value != nil)
		*value = HCR(c, Ruiccmdarg3);
	splx(s);
	return 0;
}

/* ufsdmeset: plain local DME_SET, implicit selector 0. */
static int
ufsdmeset(Ctlr *c, u32int attr, u32int value)
{
	return ufsuiccmd(c, UicDmeSet, attr, 0, value, nil);
}

/*
 * ufsdmesetsel/ufsdmegetsel: local DME_SET/DME_GET with an explicit
 * GenSelectorIndex (needed for per-lane M-PHY attributes).
 *   UIC_ARG_MPHY_TX_GEN_SEL_INDEX(lane) = lane
 *   UIC_ARG_MPHY_RX_GEN_SEL_INDEX(lane) = PA_MAXDATALANES(4) + lane
 */
enum {
	UicDmeGet	= 0x01,
};

static int
ufsdmesetsel(Ctlr *c, u32int attr, u32int sel, u32int value)
{
	return ufsuiccmd(c, UicDmeSet, attr, sel, value, nil);
}

static int
ufsdmegetsel(Ctlr *c, u32int attr, u32int sel, u32int *value)
{
	return ufsuiccmd(c, UicDmeGet, attr, sel, 0, value);
}

/*
 * ufsdmesetselpeer/ufsdmegetselpeer: DME_PEER_SET/DME_PEER_GET --
 * same ARG1 packing as the local variants above, but targeting the
 * REMOTE device's attribute space instead of this host controller's
 * own. Opcodes confirmed standard UIC command values (DME_GET=0x01,
 * DME_SET=0x02, DME_PEER_GET=0x03, DME_PEER_SET=0x04 per the
 * UFS/UniPro spec, matching U-Boot's own UIC_CMD_DME_PEER_SET/
 * UIC_CMD_DME_PEER_GET constants). Needed for UFSHCD_QUIRK_BROKEN_LCC's
 * fix (disable TX_LCC_ENABLE on the DEVICE side), which requires
 * writing to the peer.
 */
enum {
	UicDmePeerGet	= 0x03,
	UicDmePeerSet	= 0x04,
};

static int
ufsdmesetselpeer(Ctlr *c, u32int attr, u32int sel, u32int value)
{
	return ufsuiccmd(c, UicDmePeerSet, attr, sel, value, nil);
}

static int
ufsdmegetselpeer(Ctlr *c, u32int attr, u32int sel, u32int *value)
{
	return ufsuiccmd(c, UicDmePeerGet, attr, sel, 0, value);
}

/*
 * ufsuniproinit: PA-layer/M-PHY UniPro attribute tuning, per the
 * vendor's spacemit_k3_ufs_unipro_init(). Vendor-specific attributes
 * governing HS-gear sync/prepare lengths, scrambling, TX skip, and
 * RX/TX capability values, none of which are guessable from the
 * generic UFS/UniPro spec. Runs after HCE=1, before DME_LINKSTARTUP.
 *
 * Standard PA_* attribute IDs are from the official U-Boot
 * drivers/ufs/unipro.h (standard MIPI UniPro attribute numbers, not
 * vendor-original expression). The vendor-proprietary M-PHY backdoor
 * attribute IDs (RX_LS_PRE_LEN_CAP, RX_LANE_HB8_BKDOOR_ATTR,
 * RX_PWRM_CLOSURE_LEN_CAP, RX_MIN_STALL_CAP, RX_LANE_SOF_BKDOOR_ATT,
 * ANA_EQ_CTRL_REG_ATTR, PA_MK2EXTENSIONGUARDBAND) and their tuning
 * values below are hardware facts -- the specific numeric attribute
 * ID SpacemiT's M-PHY vendor assigned each register, and the specific
 * value this silicon needs written to it -- cross-checked against
 * SpacemiT's own ufs-spacemit-k3.c purely to confirm those numbers;
 * no code structure, comments, or naming scheme from that source is
 * reproduced here.
 *
 * Selector encoding: RX0/TX0 = UIC_ARG_MPHY_*_GEN_SEL_INDEX(0),
 * RX1/TX1 = ..._INDEX(1); RX uses 4+lane per this driver's convention.
 */
static int
ufsuniproinit(Ctlr *c)
{
	ufsdmeset(c, 0x1552 /* PA_TxHsG1SyncLength */, 0x4f);
	ufsdmeset(c, 0x1553 /* PA_TxHsG1PrepareLength */, 0xf);
	ufsdmeset(c, 0x1554 /* PA_TxHsG2SyncLength */, 0x4f);
	ufsdmeset(c, 0x1555 /* PA_TxHsG2PrepareLength */, 0xf);
	ufsdmeset(c, 0x1556 /* PA_TxHsG3SyncLength */, 0x4f);
	ufsdmeset(c, 0x1557 /* PA_TxHsG3PrepareLength */, 0xf);
	ufsdmeset(c, 0x155A /* PA_TxMk2Extension */, 0x0);
	ufsdmeset(c, 0x155B /* PA_PeerScrambling */, 0x1);
	ufsdmeset(c, 0x155C /* PA_TxSkip */, 0x1);
	ufsdmeset(c, 0x155D /* PA_TxSkipPeriod */, 250);
	ufsdmeset(c, 0x155E /* PA_Local_TX_LCC_Enable */, 0x0);
	ufsdmeset(c, 0x155F /* PA_Peer_TX_LCC_Enable */, 0x0);
	ufsdmeset(c, 0x1585 /* PA_Scrambling */, 0x1);
	ufsdmeset(c, 0x15AA /* PA_Granularity */, 0x1);
	ufsdmeset(c, 0x15AB /* PA_MK2ExtensionGuardBand, vendor-defined */, 0x0);
	ufsdmeset(c, 0x15A8 /* PA_TActivate */, 0x64);
	ufsdmeset(c, 0x1564 /* PA_TxTrailingClocks */, 0x64);
	ufsdmeset(c, 0x15A3 /* PA_StallNoConfigTime */, 15);

	ufsdmesetsel(c, 0x008D /* RX_LS_PRE_LEN_CAP, vendor-defined */, 4+0, 0x0B);
	ufsdmesetsel(c, 0x008D, 4+1, 0x0B);
	ufsdmesetsel(c, 0x00F4 /* RX_LANE_HB8_BKDOOR_ATTR, vendor-defined */, 4+0, 0x9F);
	ufsdmesetsel(c, 0x00F4, 4+1, 0x9F);
	ufsdmesetsel(c, 0x008E /* RX_PWRM_CLOSURE_LEN_CAP, vendor-defined */, 4+0, 15);
	ufsdmesetsel(c, 0x008E, 4+1, 15);
	ufsdmesetsel(c, 0x0088 /* RX_MIN_STALL_CAP, vendor-defined */, 4+0, 0xFF);
	ufsdmesetsel(c, 0x0088, 4+1, 0xFF);
	ufsdmesetsel(c, 0x00CD /* ANA_EQ_CTRL_REG_ATTR, vendor-defined */, 0, 0x5);
	ufsdmesetsel(c, 0x00F2 /* RX_LANE_SOF_BKDOOR_ATT, vendor-defined */, 4+0, 0x9F);
	ufsdmesetsel(c, 0x00F2, 4+1, 0x9F);

	ufsdmeset(c, 0xFC /* vendor: "bypass B0 reduce phy power ECO" */, 0xFC);

	/*
	 * Timer registers, re-derived and written here to mirror the
	 * vendor's own unipro_init() writing them at this exact point
	 * (not just once earlier in ufsinit()) -- same formulas already
	 * used above, kept in sync deliberately.
	 */
	HCR(c, Rsys1clk1us) = 491;
	HCR(c, Rtxsymclkns1us) = 0x800;
	HCR(c, Rpalinkstarttimer) = 0x2ED34E0;

	if(debug)
		print("ufs: ufsuniproinit done\n");
	return 0;
}

/*
 * ufsdisabledevicetxlcc: fix for UFSHCD_QUIRK_BROKEN_LCC, set
 * explicitly by SpacemiT's own vendor U-Boot driver ("Mirror Linux
 * behavior: disable LCC for controller stability"). Per the generic
 * U-Boot/Linux UFS core's ufshcd_disable_tx_lcc(hba, peer=true): read
 * PA_CONNECTEDTXDATALANES (0x1561) from the peer to find how many TX
 * lanes are connected, then DME_PEER_SET TX_LCC_ENABLE (0x002C, an
 * M-TX config attribute, selected per-lane via
 * UIC_ARG_MPHY_TX_GEN_SEL_INDEX(lane)=lane) to 0 for each one. Must
 * run after DME_LINKSTARTUP succeeds and before any command/data
 * traffic.
 */
static void
ufsdisabledevicetxlcc(Ctlr *c)
{
	u32int txlanes;
	u32int i;

	if(ufsdmegetselpeer(c, 0x1561 /* PA_CONNECTEDTXDATALANES */, 0,
	    &txlanes) < 0){
		print("ufs: ufsdisabledevicetxlcc: couldn't read "
			"PA_CONNECTEDTXDATALANES, skipping\n");
		return;
	}
	if(debug)
		print("ufs: ufsdisabledevicetxlcc: PA_CONNECTEDTXDATALANES=%d\n",
			txlanes);
	for(i = 0; i < txlanes; i++){
		if(ufsdmesetselpeer(c, 0x002C /* TX_LCC_ENABLE */, i, 0) < 0)
			print("ufs: ufsdisabledevicetxlcc: failed on lane %d\n", i);
	}
	if(debug)
		print("ufs: ufsdisabledevicetxlcc: done (%d lane(s))\n", txlanes);
}

/*
 * ufspwrmode: negotiate the link out of its post-LINKSTARTUP default
 * (minimal) power mode into the real operational HS mode. Two-step
 * FASTAUTO_MODE-then-FAST_MODE is SpacemiT-specific, per the vendor's
 * spacemit_k3_ufs_set_power_mode() (a .set_power_mode hba_ops hook
 * that fully replaces the generic core's own power-mode-change
 * logic). Negotiated values (HS-Gear3, 2 lanes, FAST_MODE, HS-Mode-B)
 * match U-Boot's own successful negotiation on this hardware exactly.
 *
 * Retries the whole FASTAUTO+FAST sequence up to 3 times on failure
 */
static int
ufspwrmode(Ctlr *c)
{
	u32int mode;
	int status, i, retry;

	ufsdmeset(c, AttrPaRxGear, TargetHsGear);
	ufsdmeset(c, AttrPaActiveRxLanes, TargetLanes);
	ufsdmeset(c, AttrPaRxTermination, 1);	/* true: FAST/FASTAUTO mode */
	ufsdmeset(c, AttrPaTxGear, TargetHsGear);
	ufsdmeset(c, AttrPaActiveTxLanes, TargetLanes);
	ufsdmeset(c, AttrPaTxTermination, 1);
	ufsdmeset(c, AttrPaHsSeries, TargetHsSeries);

	ufsdmeset(c, AttrPaPwrModeUserData0+0, DlFc0ProtDefault);
	ufsdmeset(c, AttrPaPwrModeUserData0+1, DlTc0ReplayDefault);
	ufsdmeset(c, AttrPaPwrModeUserData0+2, DlAfc0ReqDefault);
	ufsdmeset(c, AttrPaPwrModeUserData0+3, DlFc0ProtDefault);
	ufsdmeset(c, AttrPaPwrModeUserData0+4, DlTc0ReplayDefault);
	ufsdmeset(c, AttrPaPwrModeUserData0+5, DlAfc0ReqDefault);
	ufsdmeset(c, AttrDmeLocalFc0Prot, DlFc0ProtDefault);
	ufsdmeset(c, AttrDmeLocalTc0Replay, DlTc0ReplayDefault);
	ufsdmeset(c, AttrDmeLocalAfc0Req, DlAfc0ReqDefault);

	status = 0;
	for(retry = 0; retry < 3; retry++){
		if(retry > 0){
			if(debug)
				print("ufs: PA_PWRMODE change retry %d\n", retry);
			for(i = 0; i < 10; i++)
				microdelay(1000);	/* 10ms between retries */
		}

		/* step 1: FASTAUTO both directions, per SpacemiT's own
		 * driver -- transitional step, result deliberately unchecked */
		mode = PwrFastAutoMode<<4 | PwrFastAutoMode;
		ufsuiccmd(c, UicDmeSet, AttrPaPwrMode, 0, mode, nil);
		microdelay(1000);	/* 1ms, matches vendor's mdelay(1) exactly */

		/* step 2: FAST both directions */
		mode = PwrFastMode<<4 | PwrFastMode;
		if(ufsuiccmd(c, UicDmeSet, AttrPaPwrMode, 0, mode, nil) < 0){
			print("ufs: PA_PWRMODE change timed out (retry %d)\n", retry);
			continue;
		}

		status = (HCR(c, Rhcs)>>8) & 0x7;
		if(debug)
			print("ufs: PA_PWRMODE change UPMCRS=%d (1=PWR_LOCAL success)\n", status);
		if(status == PwrLocal)
			return 0;
	}
	print("ufs: PA_PWRMODE change failed after 3 retries, UPMCRS=%d\n", status);
	return -1;
}

/*
 * ufsdevinit: mandatory post-LINKSTARTUP device-init handshake (JEDEC
 * UFS spec, not vendor-specific -- see the long comment at the call
 * site in ufspnp()). Set fDeviceInit, then poll Read Flag until the
 * device clears it, exactly mirroring the generic Linux UFS core's
 * ufshcd_complete_dev_init()/ufshcd_verify_dev_init() sequence (facts
 * only: set-then-poll-until-clear on this specific flag idn; this
 * implementation, loop structure, and comments are original).
 */
static int
ufsdevinit(Ctlr *c)
{
	int i;
	u32int val;

	/*
	 * DL_AFC0REQTIMEOUTVAL (0x2043, Data Link layer attribute --
	 * distinct from PA_PWRMODEUSERDATA2/DME_LocalAFC0ReqTimeOutVal
	 * already programmed in ufspwrmode()) set to max (0xFFFF), per
	 * the vendor's ufs_spacemit_link_startup_post_change(), called
	 * right after LINKSTARTUP succeeds and before the fDeviceInit
	 * handshake.
	 */
	ufsdmeset(c, 0x2043 /* DL_AFC0REQTIMEOUTVAL */, 0xFFFF);

	/*
	 * TC1 (Traffic Class 1) Data Link flow-control setup:
	 * DL_AFC1REQTIMEOUTVAL (0x2063) is separate from
	 * DL_AFC0REQTIMEOUTVAL (0x2043) above. Setting DL_PEERTC1PRESENT=1
	 * tells the local DL layer the peer supports TC1, needed if UFS's
	 * data-carrying CPort is assigned that traffic class. DL_PEERTC0
	 * PRESENT=1 set too, defensively. Timeout/replay defaults for TC1
	 * taken from unipro.h's own _Default constants, matching the TC0
	 * values already in use elsewhere in this file.
	 */
	ufsdmeset(c, 0x2046 /* DL_PEERTC0PRESENT */, 1);
	ufsdmeset(c, 0x2066 /* DL_PEERTC1PRESENT */, 1);
	ufsdmeset(c, 0x2061 /* DL_FC1PROTTIMEOUTVAL */, 8191);
	ufsdmeset(c, 0x2062 /* DL_TC1REPLAYTIMEOUTVAL */, 65535);
	ufsdmeset(c, 0x2063 /* DL_AFC1REQTIMEOUTVAL */, 0xFFFF);

	/*
	 * 3-step "dummy frame" poke (0x97->0xd7->0x17 into attribute 0xE8,
	 * TX lane 0), applied unconditionally per U-Boot's
	 * spacemit_k3_ufs_link_startup_notify(). Vendor comment: "add 0xe8
	 * make UFS2.1 run GEAR3+2Lane@409M". remotever/rx1fsm are read for
	 * diagnostic logging only.
	 */
	{
		u32int remotever, rx1fsm;

		remotever = 0;
		ufsdmegetsel(c, 0x15A0 /* PA_REMOTEVERINFO */, 0, &remotever);
		rx1fsm = 0;
		ufsdmegetsel(c, 0xC1, 4+1 /* RX lane 1 */, &rx1fsm);
		if(debug)
			print("ufs: remote UniPro ver=%#ux RX1 FSM status=%#ux\n",
				remotever, rx1fsm);
		ufsdmesetsel(c, 0xE8, 0 /* TX0 */, 0x97);
		ufsdmesetsel(c, 0xE8, 0, 0xd7);
		ufsdmesetsel(c, 0xE8, 0, 0x17);
	}

	/*
	 * Local-side TX_LCC_ENABLE disable (M-PHY attribute 0x002C, per
	 * unipro.h), for TX lanes 0 and 1 -- distinct from and in
	 * addition to both PA_Local/Peer_TX_LCC_Enable (already set in
	 * ufsuniproinit(), before LINKSTARTUP) and this driver's existing
	 * ufsdisabledevicetxlcc() (a DME_PEER_SET-based disable on the
	 * device side, from the generic UFSHCD_QUIRK_BROKEN_LCC
	 * mechanism).
	 */
	ufsdmesetsel(c, 0x002C /* TX_LCC_ENABLE */, 0, 0);
	ufsdmesetsel(c, 0x002C, 1, 0);

	/* TX_MIN_ACTIVATETIME (M-PHY attribute 0x0033), TX lanes 0 and 1 */
	ufsdmesetsel(c, 0x0033 /* TX_MIN_ACTIVATETIME */, 0, 0x0);
	ufsdmesetsel(c, 0x0033, 1, 0x0);

	/*
	 * Pre-set TX RATE/GEAR via the same ANA_HSGEAR_CTRL_ATTR (0x00C1)
	 * used as an ATOP backdoor array index in the M-PHY calibration
	 * step in ufsinit() -- here accessed as a genuine UIC attribute
	 * via a real DME_SET instead, to let the PLL lock at the target
	 * HS gear before the actual PA_PWRMODE switch. Followed by a
	 * second M-PHY PLL lock wait, since changing this can re-trigger
	 * PLL relock.
	 */
	ufsdmeset(c, 0x00C1 /* ANA_HSGEAR_CTRL_ATTR */, 0x25);
	{
		int j;
		for(j = 0; j < 10000 && !(MNG(c, Mphyputrl) & (1<<31)); j++)
			microdelay(1);
		if(!(MNG(c, Mphyputrl) & (1<<31)))
			print("ufs: WARNING: second M-PHY PLL lock (post ANA_HSGEAR_CTRL_ATTR) "
				"timed out, MPHY_PU_CTRL=%#ux\n", MNG(c, Mphyputrl));
	}

	/*
	 * NOP OUT/IN handshake, per the real generic core's own
	 * ufshcd_device_init() -> ufshcd_verify_dev_init() sequence:
	 * "Verify device initialization by sending NOP OUT UPIU", BEFORE
	 * fDeviceInit ("Initiate UFS initialization"). Retried a few
	 * times, matching the real core's NOP_OUT_RETRIES loop -- not
	 * fatal to the overall boot if it never succeeds, just logged.
	 */
	{
		int nopretry, nopok;

		nopok = 0;
		for(nopretry = 0; nopretry < 3 && !nopok; nopretry++){
			if(ufsnop(c) == 0)
				nopok = 1;
			else
				microdelay(1000);
		}
		if(!nopok)
			print("ufs: WARNING: NOP OUT/IN handshake never succeeded, "
				"continuing anyway\n");
	}

	if(ufsquery(c, QopSetFlag, FlagIdnDeviceInit, 0, 0, nil) < 0){
		print("ufs: ufsdevinit: SET_FLAG(fDeviceInit) failed\n");
		return -1;
	}
	for(i = 0; i < 100; i++){
		val = 0;
		if(ufsquery(c, QopReadFlag, FlagIdnDeviceInit, 0, 0, &val) < 0){
			print("ufs: ufsdevinit: READ_FLAG(fDeviceInit) failed\n");
			return -1;
		}
		if((val & 1) == 0){
			if(debug)
				print("ufs: ufsdevinit: fDeviceInit cleared after %d polls "
					"-- device init complete\n", i);
			{
				/*
				 * bRefClkFreq (QUERY_ATTR_IDN_REF_CLK_FREQ = 0x0A):
				 * the device's own record of its expected reference
				 * clock, expected to agree with the host's
				 * MPHY_REF_CLK_SEL=Mrefclk19M2 (19.2MHz, matching
				 * k3.dtsi's freq-table-hz).
				 */
				u32int refclk;

				refclk = 0xffffffff;
				if(ufsquery(c, QopReadAttr, 0x0A /* bRefClkFreq */,
				    0, 0, &refclk) < 0)
					print("ufs: bRefClkFreq read failed\n");
				else{
					if(debug)
						print("ufs: device bRefClkFreq=%d (0=19.2MHz "
							"1=26MHz 2=38.4MHz 3=52MHz) -- host side "
							"is now configured for 19.2MHz "
							"(MPHY_REF_CLK_SEL=Mrefclk19M2)\n", refclk);
					if(refclk != 0){
						u32int wval;

						wval = 0;
						print("ufs: MISMATCH -- correcting "
							"bRefClkFreq to 0 (19.2MHz)\n");
						if(ufsquery(c, QopWriteAttr, 0x0A, 0, 0, &wval) < 0)
							print("ufs: bRefClkFreq write failed\n");
						/*
						 * If we discover that the UFS reference clock was incorrect
						 * and need to correct it, the calibration we just performed
						 * is no longer valid. That’s why we return -2, to tell
						 * ufspnp() to completely restart the UFS initialization
						 * process once again.
						 */
						return -2;
					}
				}
			}
			{
				/*
				 * DIAGNOSTIC: read the Device Descriptor and check
				 * bNumberLU/bNumberWLU.
				 */
				uchar desc[16];
				int dlen = ufsreaddesc(c, 0 /*QUERY_DESC_IDN_DEVICE*/,
					0, desc, sizeof desc);
				if(dlen >= 8){
					if(debug)
						print("ufs: Device Descriptor: bLength=%d "
							"bDescriptorType=%#ux bDeviceType=%#ux "
							"bNumberLU=%d bNumberWLU=%d\n",
							desc[0], desc[1], desc[2], desc[6], desc[7]);
				} else
					print("ufs: Device Descriptor read failed or "
						"too short (dlen=%d)\n", dlen);
			}
			{
				/*
				 * Confirms LUN 0 is enabled (bLUEnable at Unit
				 * Descriptor offset 0x03)
				 */
				uchar udesc[16];
				int ulen = ufsreaddesc(c, 2 /*QUERY_DESC_IDN_UNIT*/,
					0, udesc, sizeof udesc);
				if(ulen > 3){
					if(debug)
						print("ufs: Unit Descriptor LUN0: bLength=%d "
							"bDescriptorType=%#ux bUnitIndex=%#ux "
							"bLUEnable=%d\n",
							udesc[0], udesc[1], udesc[2], udesc[3]);
				} else
					print("ufs: Unit Descriptor LUN0 read failed or "
						"too short (ulen=%d)\n", ulen);
			}
			{
				/*
				 * Power management in this driver is basic:
				 * link on (HCE=1, trained, running) or off (HCE=0).
				 * No Hibern8 entry/exit is ever attempted,
				 * automatic or manual. Rahit=0 (AUTO_HIBERNATE_IDLE_
				 * TIMER, offset 0x18) disables the controller's own
				 * auto-idle-into-Hibern8 behavior entirely.
				 */
				HCR(c, Rahit) = 0;
				if(debug)
					print("ufs: disabled Auto-Hibern8 (Rahit=0, "
						"UFSHCD_QUIRK_BROKEN_AUTO_HIBERN8 per vendor)\n");
			}
			{
				/*
				 * Even if the UFS link has started successfully,
				 * the Query/Descriptor operations may be leaving the
				 * M-PHY/PLL in a state that requires SpacemiT to reapply
				 * its specific attributes and wait for the PLL lock again.
				 */
				int j;

				ufsdmesetsel(c, 0x002C /* TX_LCC_ENABLE */, 0, 0);
				ufsdmesetsel(c, 0x002C, 1, 0);
				ufsdmesetsel(c, 0x0033 /* TX_MIN_ACTIVATETIME */, 0, 0x0);
				ufsdmesetsel(c, 0x0033, 1, 0x0);
				for(j = 0; j < 10000 && !(MNG(c, Mphyputrl) & (1<<31)); j++)
					microdelay(1);
				if(!(MNG(c, Mphyputrl) & (1<<31)))
					print("ufs: WARNING: third M-PHY PLL lock (post-"
						"device-descriptor, apply_dev_quirks-"
						"equivalent) timed out, MPHY_PU_CTRL=%#ux\n",
						MNG(c, Mphyputrl));
				else if(debug)
					print("ufs: apply_dev_quirks-equivalent done\n");
			}
			if(debug){
				/*
				 * Transport layer (T_*) diagnostic reads,
				 * read-only. Confirms whether UFS's data CPort is
				 * connected/opened at the Transport layer and
				 * which traffic class (TC0/TC1) it actually uses.
				 */
				u32int v;

				v = 0xffffffff;
				if(ufsdmegetsel(c, 0x4020 /* T_CONNECTIONSTATE */, 0, &v) == 0)
					print("ufs: T_CONNECTIONSTATE=%#ux\n", v);
				v = 0xffffffff;
				if(ufsdmegetsel(c, 0x4023 /* T_TRAFFICCLASS */, 0, &v) == 0)
					print("ufs: T_TRAFFICCLASS=%#ux\n", v);
				v = 0xffffffff;
				if(ufsdmegetsel(c, 0x4025 /* T_CPORTFLAGS */, 0, &v) == 0)
					print("ufs: T_CPORTFLAGS=%#ux\n", v);
				v = 0xffffffff;
				if(ufsdmegetsel(c, 0x4028 /* T_LOCALBUFFERSPACE */, 0, &v) == 0)
					print("ufs: T_LOCALBUFFERSPACE=%#ux\n", v);
				v = 0xffffffff;
				if(ufsdmegetsel(c, 0x4029 /* T_PEERBUFFERSPACE */, 0, &v) == 0)
					print("ufs: T_PEERBUFFERSPACE=%#ux\n", v);
				v = 0xffffffff;
				if(ufsdmegetsel(c, 0x402A /* T_CREDITSTOSEND */, 0, &v) == 0)
					print("ufs: T_CREDITSTOSEND=%#ux\n", v);
			}
			return ufspwrmode(c);
		}
		microdelay(10000);	/* 10ms between polls */
	}
	print("ufs: ufsdevinit: fDeviceInit never cleared after 100 polls\n");
	return -1;
}

static SDev*
ufspnp(void)
{
	Ctlr *c;
	SDev *sdev;
	Dtbufs *du;
	uvlong base, size;
	int irq, i, n;

	/*
	 * DTB-first attach, mirroring etherdwqe.c's etherpnp() pattern
	 * for this same port: dtbsocmatch() to confirm we're on a K3,
	 * then walk the "ufs"-class nodes bootargs.c now tracks
	 * (isufsnode()/Dtbufs, added alongside the existing Dtbether
	 * infrastructure), falling back to the manual's hardcoded
	 * addresses if there's no matching DTB entry or no DTB at all.
	 */
	du = nil;
	if(dtbsocmatch("spacemit,k3")){
		n = dtbnufs();
		for(i = 0; i < n; i++){
			Dtbufs *d = dtbufs(i);
			if(d != nil && d->ok){
				du = d;
				break;
			}
		}
	}
	if(du != nil){
		base = du->base;
		size = du->size? du->size : UfshcSize;
		irq = du->irq > 0? du->irq : UfsIrq;
		print("ufs: from DTB %s base=%#llux size=%#llux irq=%d\n",
			du->path, base, size, irq);
	} else {
		base = UfshcPhys;
		size = UfshcSize;
		irq = UfsIrq;
		print("ufs: no DTB entry, using K3 UM defaults\n");
	}

	c = malloc(sizeof(Ctlr));
	if(c == nil)
		return nil;
	/*
	 * Single vmap over the whole reg range, k3.dtsi confirms
	 * UFSHC and UFS_MNG share one physical window; UFS_MNG is
	 * reached by pointer offset, not a second vmap.
	 */
	c->hcreg = (uintptr)vmap(base, size);
	if(c->hcreg == 0){
		print("ufs: ERROR vmap failed base=%#llux size=%#llux\n", base, size);
		free(c);
		return nil;
	}
	c->mngreg = c->hcreg + UfsMngOff;

	/*
	 * APMU (syscon_apmu): shared clock/reset syscon, same node
	 * etherdwqe.c maps for GMAC0. Separate vmap since it's a
	 * different physical device than UFSHC/UFS_MNG.
	 */
	c->apmureg = (uintptr)vmap(ApmuPhys, ApmuSize);
	if(c->apmureg == 0){
		print("ufs: ERROR vmap failed apmureg base=%#ux\n", ApmuPhys);
		free(c);
		return nil;
	}
	if(debug)
		print("ufs: all vmaps ok, calling ufsinit\n");

	if(ufsinit(c) < 0){
		free(c);
		return nil;
	}

	/*
	 * intrenable() dispatches to aplic.c (MSI mode, via IMSIC) on
	 * this port -- see etherdwqe.c's etherpnp() for the same
	 * pattern and its note about soc.plic == nil. No PLIC despite
	 * the historical name; irq defaults to aplic_int[135] = ufs_intr
	 * (K3 UM ch.7.1) unless the DTB says otherwise.
	 */
	if(irq > 0)
		intrenable(irq, ufsintr, c, BUSUNKNOWN, "ufs");
	if(debug)
		print("ufs: intrenable done, irq=%d\n", irq);

	sdev = malloc(sizeof(SDev));
	sdev->ifc = &sdufsifc;
	sdev->ctlr = c;
	sdev->nunit = 1;	/* UFS LUN 0 (well-known LU / boot LU per config) */
	c->sdev = sdev;

	/*
	 * The SpacemiT K3 UFS has a known issue with LCC; both U-Boot and Linux
	 * work around it by disabling LCC. We should do the same in Plan 9, even
	 * if we inherit an already-initialized link, because this also covers the
	 * case where Plan 9 performed its own `LINKSTARTUP` on a cold boot.
	 */
	ufsdisabledevicetxlcc(c);

	/*
	 * Mandatory UFS device-init handshake per JEDEC spec:
	 * DME_LINKSTARTUP only brings up the UniPro link, not the flash
	 * device's own internal init. Set fDeviceInit via Query Request,
	 * then poll (Read Flag) until the device clears it.
	 *
	 * ufsquery() is separate from ufsrio() because Query UPIUs have
	 * a different transaction-specific field layout
	 * (opcode/idn/index/selector/value) than SCSI Command UPIUs.
	 *
	 * Return value -2 (distinct from plain -1 failure) means
	 * ufsdevinit() had to correct a bRefClkFreq mismatch.
	 */
	{
		int devinitretry;

		for(devinitretry = 0; devinitretry < 2; devinitretry++){
			int dret = ufsdevinit(c);
			if(dret == 0)
				break;
			if(dret != -2){
				print("ufs: WARNING device-init (fDeviceInit) handshake "
					"failed or timed out -- SCSI commands will likely "
					"still fail\n");
				break;
			}
			print("ufs: ufsdevinit requested full reinit (bRefClkFreq "
				"correction) -- redoing ufsinit()+ufsdevinit(), "
				"attempt %d\n", devinitretry+1);
			/*
			 * If we don't set HCE to 0, the driver may think that everything
			 * is already initialized and skip precisely the physical link
			 * reinitialization we're trying to achieve.
			 */
			HCR(c, Rhce) = 0;
			{
				int hcewait;
				for(hcewait = 0; hcewait < 10000 && (HCR(c, Rhce) & 1); hcewait++)
					microdelay(100);
			}
			if(ufsinit(c) < 0){
				print("ufs: WARNING reinit after bRefClkFreq correction "
					"failed\n");
				break;
			}
		}
	}

	print("ufs: attached, %d unit(s)\n", sdev->nunit);
	return sdev;
}

/*
 * ScsiStatusCheck: SCSI status byte value for CHECK CONDITION. Needed
 * inside ufsrio to decide whether to dump/fetch sense data.
 */
enum {
	ScsiStatusCheck	= 0x02,
};

/*
 * ufsrestorephy: re-apply the M-PHY/PA-layer attribute pokes that the
 * K3 UM says must be restored after a LINERESET. Mirrors the same pokes
 * done once in ufsdevinit() after DME_LINKSTARTUP, intentionally NOT
 * re-doing the 0xE8 dummy-frame sequence and NOT re-doing
 * ufsuniproinit()'s pre-LINKSTARTUP-only attributes (avoiding repeating
 * steps that only make sense during the initial link startup).
 */
static void
ufsrestorephy(Ctlr *c)
{
	int j;

	print("ufs: ufsrestorephy: LINERESET detected, restoring M-PHY "
		"attributes per K3 UM\n");

	ufsdmesetsel(c, 0x002C /* TX_LCC_ENABLE */, 0, 0);
	ufsdmesetsel(c, 0x002C, 1, 0);
	ufsdmesetsel(c, 0x0033 /* TX_MIN_ACTIVATETIME */, 0, 0x0);
	ufsdmesetsel(c, 0x0033, 1, 0x0);
	ufsdmeset(c, 0x00C1 /* ANA_HSGEAR_CTRL_ATTR */, 0x25);
	for(j = 0; j < 10000 && !(MNG(c, Mphyputrl) & (1<<31)); j++)
		microdelay(1);
	if(!(MNG(c, Mphyputrl) & (1<<31)))
		print("ufs: ufsrestorephy: WARNING M-PHY PLL lock timed out\n");
}

/*
 * ufsrio: wrap a SCSI CDB into a Command UPIU + PRDT inside the
 * slot's command descriptor, ring the doorbell, poll for completion,
 * and pull the SCSI status/sense out of the response. This is the
 * SDifc.rio entry point (see sdufsifc below) -- called once per SCSI
 * command by sdscsi.c's scsiverify()/scsionline()/scsibio().
 */
static int
ufsrio(SDreq *r)
{
	Ctlr *c = r->unit->dev->ctlr;
	int slot;
	uchar *cd;
	Upiuhdr *cmd;
	Prdt *prd;
	Utrd *ut;
	void *dmatarget;	/* where the PRDT actually points: r->data,
				 * or c->scratchbuf for the small-read
				 * mitigation below */
	int dmalen;		/* real capacity backing dmatarget */
	int usescratch;

	coherence();	/* pair with the write-side fence in ufsintr(), see
			 * that comment -- ensures we see a linereset set by a
			 * different hart's interrupt handler */
	if(c->linereset){
		c->linereset = 0;
		ufsrestorephy(c);
	}

	if(r->clen > CdbMaxlen)
		error("ufs: cdb too long, no EHS support");

	ilock(c);
	for(slot = 0; slot < Maxutr; slot++)
		if((c->busy & (1<<slot)) == 0)
			break;
	if(slot == Maxutr){
		iunlock(c);
		error("ufs: no free slots");
	}
	c->busy |= 1<<slot;
	iunlock(c);

	cd = c->cmddesc[slot];
	memset(cd, 0, Cdbsize);

	cmd = (Upiuhdr*)(cd + CmdUpiuOff);
	cmd->transtype = UpiuCommand;
	/*
	 * Only set R/W flag bits when there's actually a data phase --
	 * for a zero-length command like TEST UNIT READY, claiming a
	 * read (as this unconditionally did before) tells the device
	 * to expect a data phase that will never happen, an internal
	 * contradiction with the UTRD's own DDnone/zero PRDT.
	 */
	cmd->flags = r->dlen == 0? UpiuFlagSimple :
		(r->write? UpiuFlagWrite : UpiuFlagRead);
	cmd->lun = r->lun;	/* SDreq carries the actual SCSI LUN;
				 * unit->subno is the SDunit index within
				 * this SDev, not the LUN */
	/* tasktag = slot number (this driver only ever has one slot
	 * outstanding at a time). */
	cmd->tasktag = slot;
	cmd->iid_cmdset = CmdsetScsi;
	putbe32(cd + CmdUpiuOff + OffExpXferLen, r->dlen);
	memmove(cd + CmdUpiuOff + OffCdb, r->cmd, r->clen);

	prd = (Prdt*)(cd + PrdtOff);
	if(r->dlen){
		/*
		 * MITIGATION for a DMA-write-visibility issue on reads (see
		 * Scratchbufsize / c->scratchbuf's allocation in ufsinit()):
		 * some reads at or below one page complete successfully at
		 * the transport level but the device's write to the
		 * destination buffer never becomes visible to the CPU.
		 * Route any read at or below Scratchbufsize through the
		 * dedicated scratch buffer instead of the caller's r->data,
		 * and copy out just the real r->dlen bytes after completion.
		 * <= (not <) also covers exactly-Scratchbufsize reads that
		 * arrive via a freshly-allocated bounce buffer (e.g. devsd.c's
		 * sdbio() routing a sub-block-aligned access through its own
		 * sdmalloc()'d buffer) rather than the caller's own memory.
		 * Writes are unaffected -- only a device-to-host DMA write
		 * needs to become visible to the CPU here.
		 */
		usescratch = !r->write && r->dlen <= Scratchbufsize &&
			c->scratchbuf != nil;
		dmatarget = usescratch? c->scratchbuf : r->data;
		dmalen = usescratch? Scratchbufsize : r->dlen;

		prd->dbaddr = (u32int)PADDR(dmatarget);
		prd->dbaddru = (u32int)(PADDR(dmatarget)>>32);
		prd->dbc = dmalen - 1;
		if(debug)
			print("ufs: rio PRDT: dmatarget VA=%#p PADDR=%#p dbc=%d "
				"(scratch=%d)\n", dmatarget, (void*)PADDR(dmatarget),
				prd->dbc, usescratch);
		/*
		 * Sentinel: a known value that allows detection that the memory
		 * has not been modified.
		 */
		if(!r->write){
			memset(dmatarget, 0xA5, dmalen);
			cachedwbse(dmatarget, dmalen);
		}
	} else {
		usescratch = 0;
		dmatarget = nil;
		dmalen = 0;
	}

	ut = &c->utrl[slot];
	memset(ut, 0, sizeof *ut);
	ut->dw0 = UtrdCTufs | UtrdIntr |
		(r->dlen == 0? UtrdDDnone : r->write? UtrdDDwrite : UtrdDDread);
	ut->dw2 = Ocsinvalid;		/* preload; hw overwrites on completion */
	ut->ucdba = (u32int)PADDR(cd);		/* must be 128B aligned */
	ut->ucdbau = (u32int)(PADDR(cd)>>32);
	ut->ru = (RespUpiuLen/4) | ((RespUpiuOff/4)<<16);
	/*
	 * DW6 (response UPIU offset/length) and DW7 (PRDT offset/length):
	 * per the real UTP Transfer Request Descriptor layout (matching
	 * Linux's ufshcd.h struct utp_transfer_req_desc), the LOW 16 bits
	 * hold LENGTH and the HIGH 16 bits hold OFFSET, for both fields.
	 */
	ut->prdt = r->dlen?
		(Maxprd | ((PrdtOff/4)<<16)) : 0;

	/*
	 * DMA cache maintenance: the UFSHC DMA-fetches the UTRD array and
	 * command descriptor (Command UPIU/CDB + PRDT) directly from
	 * physical RAM, never through the CPU. On this Zicbom-based,
	 * DMA-incoherent RISC-V system, anything the CPU just wrote via
	 * memset/memmove above can still be sitting in cache and not yet
	 * visible in RAM -- write back only what the DEVICE needs to
	 * read: the command descriptor and the UTRD entry. For a WRITE
	 * request, the caller's data buffer also needs writing back.
	 */
	cachedwbse(cd, Cdbsize);
	cachedwbse(ut, sizeof *ut);
	if(r->dlen && r->write)
		cachedwbse(r->data, r->dlen);

	coherence();
	if(debug){
		print("\n");
		print("ufs: rio submitting slot %d clen=%d cdb=", slot, r->clen);
		{
			int ci;
			for(ci = 0; ci < r->clen; ci++)
				print(" %.2ux", r->cmd[ci]);
		}
		print(" dlen=%d write=%d, ringing doorbell\n", r->dlen, r->write);
	}
	HCR(c, Rutrldbr) = 1<<slot;

	/*
	 * Completion detection per K3 UM 9.7.4.2/9.7.4.3 (Write/Read Data
	 * Operation) -- see ufswaitslot()'s comment. This driver polls
	 * unconditionally rather than sleeping for ufsintr() to wake it,
	 * so it doesn't depend on the DTB always providing a working
	 * interrupt controller.
	 */
	if(ufswaitslot(c, slot) < 0)
		print("ufs: rio slot %d TIMED OUT waiting for completion "
			"(UTRLDBR bit never cleared, no interrupt ever seen "
			"either)\n", slot);
	cachedinvse(ut, sizeof *ut);
	ilock(c);
	c->busy &= ~(1<<slot);
	iunlock(c);
	if(debug)
		print("ufs: rio slot %d done (polled), dw2=%#ux\n", slot, ut->dw2);

	if((ut->dw2 & Ocsmask) != Ocssuccess){
		print("ufs: rio slot %d OCS error dw2=%#ux\n", slot, ut->dw2);
		r->status = SDcheck;
		return SDcheck;
	}
	/*
	 * Invalidate what the DEVICE wrote: the response UPIU inside cd
	 * (status/sense), and, for a read, the DMA target buffer -- both
	 * were DMA-written directly to RAM and need invalidating so the
	 * CPU's next load sees fresh data.
	 *
	 * cd is fine with a plain invalidate: a dedicated, appropriately-
	 * sized buffer we allocated ourselves (mallocalign), not sharing
	 * cache lines with unrelated data. c->scratchbuf (when usescratch)
	 * is the same kind of buffer, for the same reason.
	 *
	 * r->data, when used directly (usescratch==0, i.e. dlen >=
	 * Scratchbufsize), is NOT: it's a caller-provided buffer whose
	 * size/alignment we don't control. cachedinvse() operates on whole
	 * cache lines; a short, unaligned r->data can share a line with
	 * an allocator's adjacent guard/tail metadata, and a pure
	 * invalidate silently discards any dirty data there without
	 * writing it back first -- confirmed on hardware to corrupt a
	 * pool allocator's tail guard and panic the kernel. cachedwbinvse()
	 * (write-back + invalidate) avoids this: dirty neighboring data
	 * gets flushed to RAM first, so nothing is lost even when cache-
	 * line boundaries overlap unrelated memory. Used unconditionally
	 * below (also for the scratch case) since it's safe either way and
	 * keeps this one code path.
	 */
	cachedinvse(cd, Cdbsize);
	if(r->dlen && !r->write){
		/*
		 * Sentinel check: reports whether the mitigation above still
		 * left dmatarget unwritten. A bounded retry-with-delay was
		 * tried here previously and did not help, so this is a single
		 * check with no retry. Only the first r->dlen bytes matter even
		 * when dmalen (the scratch capacity) is larger -- the device
		 * only ever writes the real response size.
		 */
		uchar *p = dmatarget;
		int i, stillsentinel = 1;

		cachedwbinvse(dmatarget, dmalen);
		for(i = 0; i < r->dlen; i++)
			if(p[i] != 0xA5){
				stillsentinel = 0;
				break;
			}
		if(stillsentinel)
			print("ufs: WARNING: dmatarget still reads as the "
				"0xA5 sentinel -- DMA write to the data "
				"buffer never landed (scratch=%d)\n",
				usescratch);
		if(usescratch)
			memmove(r->data, dmatarget, r->dlen);
	}

	/* pull SCSI status + sense data out of the response UPIU if the
	 * caller wants it -- response UPIU's cmd->status byte holds the
	 * SCSI status; sense data (if any) follows as a data segment
	 * per datasegcnt_hi/lo, immediately after the 12-byte header. */
	{
		Upiuhdr *rsp = (Upiuhdr*)(cd + RespUpiuOff);
		uchar *resid = cd + RespUpiuOff + 12;	/* DW3: residual xfer count, big-endian u32 */
		u32int residual = (resid[0]<<24)|(resid[1]<<16)|(resid[2]<<8)|resid[3];
		if(debug)
			print("ufs: rio slot %d response: transtype=%#ux response=%#ux "
				"status=%#ux residual=%ud (requested dlen=%d)\n",
				slot, rsp->transtype, rsp->response, rsp->status,
				residual, r->dlen);
		if(rsp->status == 0 && r->cmd[0] == 0x12 /* INQUIRY */ && r->dlen >= 36){
			uchar *id = r->data;
			print("ufs: INQUIRY: vendor=\"%.8s\" product=\"%.16s\" rev=\"%.4s\"\n",
				(char*)id+8, (char*)id+16, (char*)id+32);
		}
		if(rsp->status == 0 && r->cmd[0] == 0x25 /* READ CAPACITY(10) */ && r->dlen >= 8){
			uchar *rc = r->data;
			u32int lastlba = (rc[0]<<24)|(rc[1]<<16)|(rc[2]<<8)|rc[3];
			u32int blklen = (rc[4]<<24)|(rc[5]<<16)|(rc[6]<<8)|rc[7];
			if(debug)
				print("ufs: READ CAPACITY(10): lastLBA=%ud blockLen=%ud "
					"(total ~%llud bytes)\n", lastlba, blklen,
					((uvlong)(lastlba+1))*blklen);
		}
		if(rsp->status == ScsiStatusCheck){
			/*
			 * Sense data segment layout, per Linux's utp_cmd_rsp
			 * struct (matches JEDEC UFS Response UPIU): 12-byte
			 * basic header, then residual_transfer_count (4 bytes,
			 * offset 12-15, read above), then reserved[4] (FOUR
			 * reserved DWORDs, 16 bytes, offset 16-31 -- not one
			 * DWORD), then sense_data_len (2 bytes, offset 32-33),
			 * then sense_data (offset 34 onward).
			 */
			uchar *sd = cd + RespUpiuOff + 32;
			int senselen = (sd[0]<<8)|sd[1];
			uchar *sb = sd + 2;
			print("ufs: rio slot %d CHECK CONDITION sense: senselen=%d "
				"respcode=%#ux key=%#ux asc=%#ux ascq=%#ux raw:",
				slot, senselen, sb[0], sb[2]&0x0f, sb[12], sb[13]);
			{
				int si;
				for(si = 0; si < senselen && si < 18; si++)
					print(" %.2ux", sb[si]);
			}
			print("\n");
			if(senselen == 0 && r->cmd[0] != 0x03){
				/*
				 * The device returned CHECK CONDITION without attaching sense data
				 * (senselen == 0); any other bytes in the sense buffer are stale data,
				 * not device-generated sense information. Fetch the real sense data
				 * with an explicit SCSI REQUEST SENSE (opcode 0x03). This is safe here
				 * because the slot's busy bit has already been cleared.
				 *
				 * Do not issue REQUEST SENSE recursively: if REQUEST SENSE itself
				 * returns CHECK CONDITION with senselen == 0, stop after this one
				 * recovery attempt. sensebuf is a u32int[] rather than a uchar array
				 * to guarantee the dword alignment required by the HCI PRDT.
				 */
				u32int sensebufw[(18+3)/4];
				uchar *sensebuf = (uchar*)sensebufw;
				uchar reqsense[6];
				SDreq sr;

				memset(&sr, 0, sizeof sr);
				memset(reqsense, 0, sizeof reqsense);
				memset(sensebufw, 0, sizeof sensebufw);
				reqsense[0] = 0x03;		/* REQUEST SENSE */
				reqsense[4] = 18;		/* allocation length */
				sr.unit = r->unit;
				sr.lun = r->lun;
				memmove(sr.cmd, reqsense, sizeof reqsense);
				sr.clen = sizeof reqsense;
				sr.data = sensebuf;
				sr.dlen = 18;
				sr.write = 0;
				ufsrio(&sr);
				print("ufs: REQUEST SENSE follow-up: status=%#ux "
					"respcode=%#ux key=%#ux asc=%#ux ascq=%#ux raw:",
					sr.status, sensebuf[0], sensebuf[2]&0x0f,
					sensebuf[12], sensebuf[13]);
				{
					int si;
					for(si = 0; si < 18; si++)
						print(" %.2ux", sensebuf[si]);
				}
				print("\n");
				/*
				 * Report the REAL sense data from the REQUEST SENSE follow-up to the
				 * generic sd.c layer via r->sense/r->flags. SDreq provides a 32-byte
				 * sense buffer and the SDvalidsense flag for this purpose. The generic
				 * SCSI error/retry logic (scsiverify()/scsibio()) needs the actual sense
				 * data to correctly interpret the sense key, e.g. retrying transient
				 * Unit Attention conditions instead of treating them as hard errors.
				 */
				memmove(r->sense, sensebuf, 18);
				r->flags |= SDvalidsense;
				/*
				 * Do NOT add a recursive retry here: tried once,
				 * caused real memory corruption on hardware when
				 * combined with the REQUEST SENSE sub-call above
				 * also being on the stack. ufsrio()'s own loop-based
				 * retry wrapper (see below) is the safe place for
				 * any future retry logic.
				 */
			} else {
				/* embedded sense was present -- pass it through as-is */
				int n = senselen < (int)sizeof r->sense? senselen : (int)sizeof r->sense;
				memmove(r->sense, sb, n);
				r->flags |= SDvalidsense;
			}
		}
		r->status = rsp->status;
		/*
		 * r->rlen: actual bytes transferred. sdscsi.c's scsibio() --
		 * the generic path for real block reads/writes -- requires
		 * r->status==SDok && r->rlen>0 to treat a transfer as
		 * successful; INQUIRY/READ CAPACITY/TEST UNIT READY go through
		 * ifc->rio() directly instead and never check it.
		 */
		r->rlen = (r->dlen >= residual)? r->dlen - residual : 0;
	}
	return SDok;
}

/*
 * /dev/sdXX/ctl handler: adapted to SDifc's rctl/wctl signatures
 * Write "debug 1" or "debug 0" to /dev/sdXX/ctl to toggle the
 * driver's own trace verbosity at runtime, no reboot required.
 */
enum { CMdebug };
static Cmdtab ufsctlmsg[] = {
	CMdebug,	"debug",	2,
};

static int
ufswctl(SDunit*, Cmdbuf *cb)
{
	Cmdtab *ct;

	ct = lookupcmd(cb, ufsctlmsg, nelem(ufsctlmsg));
	switch(ct->index){
	case CMdebug:
		debug = atoi(cb->f[1]);
		break;
	}
	return 0;
}

static char*
ufsrctl(SDunit *unit, char *p, char *e)
{
	/*
	 * devsd.c's Qctl reader only auto-prints the "geometry <sectors>
	 * <secsize>" line when ifc->rctl==nil, on the theory that a
	 * driver providing its own rctl takes over all geometry
	 * reporting (see its own comment there). Since this driver does
	 * provide one (for the "debug" toggle below), that line stopped
	 * appearing entirely once ufsrctl existed -- and disk/edisk,
	 * disk/fdisk, and opendisk() (libdisk's disk.c) all depend on
	 * reading it to discover the real sector size (4096 here, not
	 * the legacy 512 default) instead of requiring an explicit "-s"
	 * flag on every invocation. Print it ourselves to restore that.
	 */
	if(unit->sectors)
		p = seprint(p, e, "geometry %llud %lud\n",
			unit->sectors, unit->secsize);
	return seprint(p, e, "debug %d\n", debug);
}

/*
 * Layout and field order here MUST match struct SDifc in sd.h
 * exactly (no "legacy" field on this port, unlike some other Plan 9
 * trees). Following the standard devsd pattern used by other
 * SCSI-command-set controllers in 9front: this driver only
 * implements pnp (attach) and rio (execute one SCSI command via a
 * UPIU). verify/online/bio are wired to sdscsi.c's generic
 * scsiverify()/scsionline()/scsibio(), which build the actual CDBs
 * (INQUIRY, READ CAPACITY(10/16), READ/WRITE(10)) and call rio to
 * execute them -- no hand-rolled INQUIRY/REPORT LUNS needed here.
 */
SDifc sdufsifc = {
	"ufs",

	ufspnp,
	nil,		/* enable */
	nil,		/* disable */

	scsiverify,
	scsionline,
	ufsrio,
	ufsrctl,
	ufswctl,

	scsibio,
	nil,		/* probe */
	nil,		/* clear */
	nil,		/* rtopctl */
	nil,		/* wtopctl */
	nil,		/* ataio -- not an ATA device */
};
