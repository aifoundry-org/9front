/*
 * Driver for the SpacemiT K3 Ethernet controller (GMAC0/dwqe): a
 * Synopsys DesignWare GMAC4 core behind the SoC's own clock and reset
 * wrapper (K3 UM 14.3).  Register and bit names below are the core's.
 *
 * The 9front driver structure follows ethersmte.c, the K1 EMAC driver.
 */

#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/netif.h"
#include "../port/etherif.h"
#include "../port/ethermii.h"

#define Rbsz	ROUNDUP(sizeof(Etherpkt)+16, 64)

/*
 * Verbosity. 0 = quiet (errors/warnings only); 1 = bring-up info and the
 * periodic stats line; Set at runtime via 'echo debug 1 >/net/ether0/clone'
 */
static int debug;

#define dprint		if(debug) print

/*
 * Default MMIO layout for the K3's GMAC0, used only as a fallback when
 * no matching DTB entry is found.  Normal operation pulls base/IRQ/
 * APMU base out of /soc/ethernet@... via the Dtbether table.
 */
enum {
	Gmac0Phys	= 0xcac80000UL,
	Gmac0Size	= 0x2000,
	Gmac0Irq	= 131,

	ApmuPhys	= 0xd4282800UL,
	ApmuSize	= 0x400,

	MpmuPhys	= 0xd4050000UL,	/* PLL lock status lives here */
	MpmuSize	= 0x1000,
	MpmuPosr	= 0x10/4,
	Pll2Lock	= 1U << 25,

	PllPhys		= 0xd4090000UL,	/* spacemit,k3-pll (APBS) */
	PllSize		= 0x1000,
	Pll2Swcr2	= 0x11c/4,	/* APBS_PLL2_SWCR2 */
	Pll2Enable	= 1U << 16,	/* type-A PLL enable */
	Pll2D3Gate	= 1U << 2,	/* pll2_d3 output gate */

	ApbcPhys	= 0xd4015000UL,
	ApbcSize	= 0x1000,
	ApbcAib		= 0x3c/4,	/* APBC_AIB_CLK_RST */
	ApbcBusEn	= 1U << 0,
	ApbcFuncEn	= 1U << 1,
	ApbcReset	= 1U << 2,
	ApbcAsfar	= 0x50/4,	/* AIB secure access unlock */
	ApbcAssar	= 0x54/4,
	ApbcAsfarKey	= 0xbaba,
	ApbcAssarKey	= 0xeb10,

	PwrDomainOff	= 0x800,	/* IO power-domain window in the pinctrl */
	PwrV18En	= 1U << 2,

	PinctrlPhys	= 0xd401e000UL,
	PinctrlSize	= 0x1000,
	PadMux		= 7,		/* pad register bits [2:0] */
	PadDriveShift	= 9,		/* K3 drive strength, bits [12:9] */
	PadDriveMask	= 0xf << PadDriveShift,
	PadPullMask	= 7U << 13,	/* pull enable/up/down, bits [15:13] */
	PadDrive9mA	= 4,		/* 1.8V table: 9mA */
	PadDrive25mA	= 10,		/* 1.8V table: 25mA */

	DwqeRxDelay	= 100,
	DwqeTxDelay	= 100,
};

/* GMAC (MAC-level) registers */
enum {
	GmacConfig			= 0x0000/4,
	GmacConfigArpen		= (1U << 31),
	GmacConfigIpc			= (1U << 27),	/* rx checksum offload */
	GmacConfigAcs			= (1U << 20),	/* strip pad/fcs */
	GmacConfigBe			= (1U << 18),	/* back-off enable */
	GmacConfigJd			= (1U << 17),	/* jabber disable */
	GmacConfigJe			= (1U << 16),	/* jumbo enable */
	GmacConfigPs			= (1U << 15),	/* port select: 1=10/100 MII, 0=1000 GMII/RGMII */
	GmacConfigFes			= (1U << 14),	/* speed: with PS, selects 100 vs 10 */
	GmacConfigDm			= (1U << 13),	/* duplex mode: 1=full */
	GmacConfigDcrs		= (1U << 9),
	GmacConfigTe			= (1U << 1),	/* transmitter enable */
	GmacConfigRe			= (1U << 0),	/* receiver enable */

	GmacRxqCtrl0			= 0x00a0/4,
	GmacRxqCtrl0Q0Dcb		= (1U << 1),	/* BIT((queue*2)+1) for queue=0 */

	GmacPacketFilter		= 0x0008/4,
	GmacPfPr		= (1U << 0),	/* promiscuous */
	GmacPfHmc		= (1U << 2),	/* hash multicast */
	GmacPfPm		= (1U << 4),	/* pass all multicast */
	GmacPfHpf		= (1U << 10),	/* hash or perfect filter */
	GmacPfRa		= (1U << 31),	/* receive all */

	GmacIntStatus			= 0x00b0/4,
	GmacIntEn			= 0x00b4/4,

	GmacMdioAddr			= 0x0200/4,
	GmacMdioData			= 0x0204/4,

	GmacHwFeature0		= 0x011c/4,
	GmacHwFeature1		= 0x0120/4,

	GmacAddr0Hi			= 0x0300/4,
	GmacAddr0Lo			= 0x0304/4,
};

/* GmacMdioAddr fields */
enum {
	MdioGbusy			= (1U << 0),
	MdioGocShift		= 2,
	MdioWrite			= (1U << MdioGocShift),		/* GOC=01 */
	MdioRead			= (3U << MdioGocShift),		/* GOC=11 */
	MdioPaShift		= 21,		/* bits [25:21] */
	MdioGrShift		= 16,		/* bits [20:16] */
	MdioCrShift		= 8,		/* bits [11:8], clock range */
	MdioCrVal			= 6,		/* matches ACLK->EMAC0_bus_clk chain (307MHz/div1) */
	MdioGdMask		= 0xffff,
};

/* DMA (global) registers */
enum {
	DmaBusMode			= 0x1000/4,
	DmaBusModeSwr		= (1U << 0),

	DmaSysBusMode		= 0x1004/4,
	DmaSysBusMb			= (1U << 14),
	DmaSysBusEame		= (1U << 11),	/* Enhanced Address Mode: enables all *_HI regs + des1 */
	DmaSysBusFb			= (1U << 0),	/* Fixed burst */
};

/* DMA per-channel registers (channel 0 only -- dwqe uses 1 queue) */
enum {
	DmaChanBase			= 0x1100/4,

	DmaChanControl		= 0x00/4,
	DmaChanTxControl		= 0x04/4,
	DmaControlSt			= (1U << 0),

	DmaChanRxControl		= 0x08/4,
	DmaControlSr			= (1U << 0),
	DmaRbszShift			= 1,		/* bits [14:1], raw byte count << 1 */
	DmaRbszMask			= 0x7ffe,

	DmaChanTxBaseHi	= 0x10/4,		/* bits [63:32] of ring base */
	DmaChanTxBase		= 0x14/4,		/* bits [31:0]  of ring base */
	DmaChanRxBaseHi	= 0x18/4,		/* bits [63:32] of ring base */
	DmaChanRxBase		= 0x1c/4,		/* bits [31:0]  of ring base */
	/*
	 * Tail pointer has no _HI counterpart -- the ring's upper 32 bits
	 * come from *_BASE_ADDR_HI above and apply to the whole ring
	 */
	DmaChanTxEnd		= 0x20/4,		/* tail pointer, 32-bit */
	DmaChanRxEnd		= 0x28/4,		/* tail pointer, 32-bit */
	DmaChanTxLen		= 0x2c/4,		/* N-1 */
	DmaChanRxLen		= 0x30/4,		/* N-1 */

	DmaChanIntrEna		= 0x34/4,
	DmaIntrNie		= (1U << 16),
	DmaIntrAie		= (1U << 15),
	DmaIntrFbe		= (1U << 12),
	DmaIntrRbu		= (1U << 7),
	DmaIntrRie		= (1U << 6),
	DmaIntrTie		= (1U << 0),

	DmaChanStatus			= 0x60/4,		/* W1C */
	DmaStatusNis		= (1U << 15),
	DmaStatusAis		= (1U << 14),
	DmaStatusFbe		= (1U << 12),
	DmaStatusRbu		= (1U << 7),
	DmaStatusRi		= (1U << 6),
	DmaStatusTi		= (1U << 0),
};

/* MTL (queue) registers */
enum {
	MtlChanBase			= 0x0d00/4,
	MtlTxOpMode		= 0x00/4,
	MtlOpTxqEn		= (1U << 3),
	MtlOpTsf			= (1U << 1),	/* store-and-forward */
	MtlOpTqsMask		= 0x01ff0000,	/* bits [24:16] */

	MtlRxOpMode		= 0x30/4,
	MtlOpRsf			= (1U << 5),
	MtlOpRqsMask		= 0x3ff00000,	/* bits [29:20] */

	GmacHwTxFifoSize		= 0x000007c0,	/* HW_FEATURE1 bits [10:6] */
	GmacHwRxFifoSize		= 0x0000001f,	/* HW_FEATURE1 bits [4:0]  */
};

/* APMU registers  */
enum {
	EmacBusEn		= (1U << 0),
	EmacBusRst	= (1U << 1),
	EmacPhySelectShift	= 3,		/* bits [4:3] */
	EmacPhySelectRmii	= 0,		/* 2'b00 */
	EmacPhySelectRgmii	= 1,		/* 2'b01 */
	EmacPhySelectMiiGmii	= 2,		/* 2'b10 */
	EmacAxiMstId		= (1U << 13),
	/*
	 * Official name is EMAC0_1588_CLK_MUX: 0 = 24M, 1 = PLL 125/25/2.5MHz
	 * matching the current speed (1000M/100M/10M) -- a clock-source mux
	 * for the PTP/1588 timestamp counter.
	 */
	Emac1588ClkMux		= (1U << 15),

	/*
	 * Previously left at their power-on-reset defaults; now set
	 * explicitly so behavior doesn't depend on what a given board's
	 * firmware happened to leave here.
	 */
	Emac0ClkRefGate		= (1U << 14),	/* 1 = tx 25M refclk disable, 0 = enable */
	Emac0PhyIntrEn		= (1U << 12),	/* 1 = PHY PMT intr mask enable */
	Emac0LpiIntrEn		= (1U << 9),	/* 1 = LPI intr mask enable */
	/*
	 * RGMII TX clock select/gate.  0 uses the PHY's RX clock, which
	 * does not run until the PHY is out of reset; 1 uses the SoC PLL
	 * 125MHz clock and also gates it on.  The K3 has no PHY reset
	 * handling here, so the MAC needs the SoC clock.
	 */
	Emac0RgmiiTxcSrcSel		= (1U << 8),	/* RGMII only: 1 = SoC PLL, 0 = PHY RX clk */
	EmacRxRefclkPhaseSel	= (1U << 7),	/* 1 = inverted, 0 = normal */
	EmacRmiiTxRefclkPhaseSel	= (1U << 6),	/* RMII only, don't-care in RGMII */

	EmacRgmiiRxcDlinePu		= (1U << 0),
	EmacRgmiiRxcDlineStep	= (0U << 4),
	EmacRgmiiRxcDlineAdjShift	= 8,
	EmacRgmiiTxcDlinePu		= (1U << 16),
	EmacRgmiiTxcDlineStep	= (0U << 20),
	EmacRgmiiTxcDlineAdjShift	= 24,
};

/*
 * Normal descriptors, "Read Format" while owned by the DMA engine,
 * "Write-Back Format" once it hands them back -- both are the same
 * 4 x u32int words, reused as r[4*i + 0..3].
 */
enum {
	/* TDES2 (read format) */
	Tdes2Buf1SizeMask		= 0x3fff,	/* [13:0] */
	Tdes2Ioc	= (1U << 31),

	/* TDES3 (read format) */
	Tdes3PktSizeMask		= 0x7fff,	/* [14:0] */
	Tdes3Ld		= (1U << 28),
	Tdes3Fd		= (1U << 29),
	Tdes3Own			= (1U << 31),

	/* TDES3 (write-back format, subset we check) */
	Tdes3Es		= (1U << 15),

	/* RDES3 (read format) */
	Rdes3Buf1v	= (1U << 24),
	Rdes3Ioc	= (1U << 30),
	Rdes3Own			= (1U << 31),

	/* RDES3 (write-back format) */
	Rdes3PktSizeMask		= 0x7fff,	/* [14:0] */
	Rdes3Es		= (1U << 15),
	Rdes3Ld		= (1U << 28),
	Rdes3Ctxt	= (1U << 30),
};

/* Standard MII registers (RTL8211F): all from ../port/ethermii.h */
enum {
	PhyAddr		= 1,	/* where the RTL8211F answers on this board */
	Linkdelay	= 500,
	Linktries	= 20,	/* 10s per PHY before trying the next address */
};

enum {
	Nrxd	= 512,
	Ntxd	= 512,
	Txirqevery = 64,
	Ndwqe	= 4,		/* GMAC0..3 */

	Dcachelinesz = 64,	/* X60/C908 D-cache line size */
};

/* same non-coherent-DMA cache maintenance as ethersmte.c */
static void
dwqecacheflush(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(Dcachelinesz - 1);
	last = ((uintptr)va + len + Dcachelinesz - 1) & ~(uintptr)(Dcachelinesz - 1);
	for(; a < last; a += Dcachelinesz)
		cboflush((void*)a);
	coherence();
}

static void
dwqecacheinval(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(Dcachelinesz - 1);
	last = ((uintptr)va + len + Dcachelinesz - 1) & ~(uintptr)(Dcachelinesz - 1);
	for(; a < last; a += Dcachelinesz)
		cboinval((void*)a);
	coherence();
}

typedef struct Ctlr Ctlr;
struct Ctlr {
	u32int	*regs;
	u32int	*apmu;
	u32int	apmuctrloff;
	u32int	apmudlineoff;
	u32int	*rxr, *txr;
	Block	**rxs, **txs;
	int	rxprodi, rxconsi, txi;
	u32int	rxdelay, txdelay;
	int	phyaddr;		/* MDIO address of the PHY */
	int	mdiocr;		/* MDIO CSR-clock-range divider */
	int	phyaddrs[32];	/* addresses that answered the MDIO scan */
	int	nphy;
	int	phyidx;
	int	phycur;
	u32int	phydone;	/* PHYs already identified and configured */
	int	rstpin;		/* PHY reset GPIO; -1 if none */
	int	rstactive;		/* reset line asserted by a low level */
	int	txcons;
	int	attach;
	int	dmarunning;
	int	promisc;
	int	allmulti;
	int	irq;
	Lock	txlock;
	Lock	rxlock;
	u32int	irqseen;
	u32int	rxframes;
	u32int	txdone;
	int	statstick;
};

/* a register read may only be used to drain a posted write */
static u32int
rd32(u32int *base, uintptr reg)
{
	coherence();
	return base[reg];
}

/* a register write has to be ordered against what follows it */
static void
wreg(u32int *base, uintptr reg, u32int v)
{
	base[reg] = v;
	coherence();
}

/* MDIO -- GMAC4-style GOC field */

static int
mdiobusy(Ctlr *c)
{
	int i;

	for(i = 0; i < 1000; i++){
		if((rd32(c->regs, GmacMdioAddr) & MdioGbusy) == 0)
			return 0;
		microdelay(10);
	}
	return -1;
}

static void
mdwrite(Ctlr *c, int reg, u16int v)
{
	u32int addr;

	if(mdiobusy(c) < 0)
		return;
	wreg(c->regs, GmacMdioData, v);
	addr = (c->phyaddr & 0x1f) << MdioPaShift
		| (reg & 0x1f) << MdioGrShift
		| c->mdiocr << MdioCrShift
		| MdioWrite
		| MdioGbusy;
	wreg(c->regs, GmacMdioAddr, addr);
	mdiobusy(c);
}

static u16int
mdread(Ctlr *c, int reg)
{
	u32int addr;

	if(mdiobusy(c) < 0)
		return 0xffff;
	addr = (c->phyaddr & 0x1f) << MdioPaShift
		| (reg & 0x1f) << MdioGrShift
		| c->mdiocr << MdioCrShift
		| MdioRead
		| MdioGbusy;
	wreg(c->regs, GmacMdioAddr, addr);
	if(mdiobusy(c) < 0)
		return 0xffff;
	return rd32(c->regs, GmacMdioData) & MdioGdMask;
}

/*
 * Clause-22 scan.  The device tree names one PHY address, but a board
 * may carry more than one PHY on the shared MDIO lines and the one that
 * drives the connected port is not always the one the tree lists.  Keep
 * every address that answers so ethproc can try them in turn.
 */
static int
mdioscan(Ctlr *c, int verbose)
{
	int a;
	u16int s1, s2;

	c->nphy = 0;
	for(a = 0; a < 32; a++){
		c->phyaddr = a;
		s1 = mdread(c, Phyidr1);
		if(s1 == 0xffff || s1 == 0)
			continue;
		s2 = mdread(c, Phyidr2);
		if(s2 == 0xffff)
			continue;
		if(verbose)
			print("dwqe: MDIO %d id %04x%04x\n", a, s1, s2);
		c->phyaddrs[c->nphy++] = a;
		if(c->nphy == nelem(c->phyaddrs))
			break;
	}
	if(c->nphy == 0)
		print("dwqe: no PHY answers on MDIO\n");
	c->phyaddr = c->nphy ? c->phyaddrs[0] : PhyAddr;
	return c->nphy;
}

/*
 * RTL8211F RGMII internal delays.  The board describes the interface
 * as plain "rgmii": the MAC's delay lines provide the timing, so the
 * PHY's own 2ns TX/RX delays must be off.  They are strapping-default
 * controlled and are commonly on; leaving them on doubles the skew and
 * corrupts TX while RX can still work.
 *
 * The page register takes the whole page number in one write.
 */
static void
rtl8211fdly(Ctlr *c)
{
	u16int page, txcr, rxcr;

	page = mdread(c, 0x1f);
	mdwrite(c, 0x1f, 0x0d08);	/* RGMII configuration page */
	txcr = mdread(c, 0x11);
	rxcr = mdread(c, 0x15);
	if(txcr & 0x0100)
		mdwrite(c, 0x11, txcr & ~0x0100);
	if(rxcr & 0x0008)
		mdwrite(c, 0x15, rxcr & ~0x0008);
	mdwrite(c, 0x1f, page);		/* restore page */
	print("dwqe: PHY RTL8211F internal delays off (txcr=%#04ux rxcr=%#04ux)\n",
		txcr, rxcr);
}

/*
 * Real address filtering: unicast perfect-match against GMAC_ADDR0
 *
 * allmulti uses PM (pass all multicast)
 */
static void
dwqesetfilter(Ctlr *c, int promisc, int allmulti)
{
	u32int v;

	v = 0;
	if(promisc)
		v |= GmacPfPr;
	else if(allmulti)
		v |= GmacPfPm;
	wreg(c->regs, GmacPacketFilter, v);
	c->promisc = promisc;
	c->allmulti = allmulti;
	dprint("dwqe: GmacPacketFilter=%#08ux (promisc=%d allmulti=%d)\n",
		v, promisc, allmulti);
}

static void	ethrx(Ether *edev);
static void	ethtx(Ether *edev);

/* link-loss recovery: stop TX DMA, reclaim ring, rewind to base */
static void
txreset(Ctlr *c)
{
	u32int v;
	int i;

	v = rd32(c->regs, DmaChanBase + DmaChanTxControl);
	wreg(c->regs, DmaChanBase + DmaChanTxControl, v & ~DmaControlSt);
	ilock(&c->txlock);
	for(i = 0; i < Ntxd; i++){
		if(c->txs[i] != nil){
			freeb(c->txs[i]);
			c->txs[i] = nil;
		}
		c->txr[4*i] = 0;
		c->txr[4*i+1] = 0;
		c->txr[4*i+2] = 0;
		c->txr[4*i+3] = 0;
	}
	c->txi = 0;
	c->txcons = 0;
	coherence();	/* txr is NC memory now; no cachelines to flush, just order the zeroing */
	iunlock(&c->txlock);
	wreg(c->regs, DmaChanBase + DmaChanTxBase, PADDR(c->txr));
	wreg(c->regs, DmaChanBase + DmaChanTxEnd, PADDR(c->txr));
}

/* PHY / link kproc -- same RTL8211F sequence as ethersmte.c */
static int
physetup(Ctlr *c)
{
	u16int id1, id2;

	c->phyaddr = c->phyaddrs[c->phyidx];
	id1 = mdread(c, Phyidr1);
	id2 = mdread(c, Phyidr2);
	if(id1 == 0xffff || id1 == 0)
		return -1;
	if((c->phydone & (1U << c->phyidx)) == 0){
		c->phydone |= 1U << c->phyidx;
		print("dwqe: PHY %d id %04x%04x (oui=0x%06ux model=0x%02ux rev=0x%01ux)\n",
			c->phyaddr, id1, id2,
			((u32int)id1 << 6) | (id2 >> 10), (id2 >> 4) & 0x3f, id2 & 0xf);
		if(id1 == 0x001c && ((id2 >> 4) & 0x3f) == 0x11)
			rtl8211fdly(c);
	}

	mdwrite(c, Mscr, Mscr1000TFD | Mscr1000THD);
	mdwrite(c, Bmcr, BmcrAne | BmcrRan);
	return 0;
}

/* wait for link, but not forever: let ethproc move on to the next PHY */
static int
waitlink(Ctlr *c)
{
	int i;

	for(i = 0; i < Linktries; i++){
		if(mdread(c, Bmsr) & BmsrLs)
			return 0;
		tsleep(&up->sleep, return0, nil, Linkdelay);
	}
	return -1;
}

static void
ethproc(void *ved)
{
	Ether *edev;
	Ctlr *c;
	u16int bmsr, lpa, gbsr;
	u32int cfg, v;
	int i, speed;
	char *dup;

	edev = ved;
	c    = edev->ctlr;

	mdioscan(c, 0);
	if(c->nphy == 0){
		c->phyaddrs[0] = PhyAddr;
		c->nphy = 1;
	}
	c->phyidx = 0;
	c->phycur = -1;

	for(;;){
		if(c->phycur != c->phyidx){
			c->phycur = c->phyidx;
			if(physetup(c) < 0){
				c->phycur = -1;
				goto nextphy;
			}
		}
		if(waitlink(c) < 0)
			goto nextphy;

		/* let autonegotiation finish, then read BMSR afresh */
		for(i = 0; i < Linktries; i++){
			bmsr = mdread(c, Bmsr);
			if(bmsr & BmsrAnc)
				break;
			tsleep(&up->sleep, return0, nil, Linkdelay);
		}
		if((bmsr & (BmsrAnc|BmsrLs)) != (BmsrAnc|BmsrLs))
			goto nextphy;

		lpa  = mdread(c, Anlpar);
		gbsr = mdread(c, Mssr);

		cfg = rd32(c->regs, GmacConfig)
			& ~(GmacConfigPs | GmacConfigFes | GmacConfigDm);
		dup = "HD";

		if(gbsr & Mssr1000TFD){
			cfg |= GmacConfigDm;		/* PS=0,FES=0 => 1000 */
			speed = 1000; dup = "FD";
		} else if(gbsr & Mssr1000THD){
			speed = 1000;
		} else if(lpa & AnaTXFD){
			cfg |= GmacConfigPs | GmacConfigFes | GmacConfigDm;
			speed = 100; dup = "FD";
		} else if(lpa & AnaTXHD){
			cfg |= GmacConfigPs | GmacConfigFes;
			speed = 100;
		} else if(lpa & Ana10FD){
			cfg |= GmacConfigPs | GmacConfigDm;
			speed = 10; dup = "FD";
		} else {
			cfg |= GmacConfigPs;
			speed = 10;
		}

		wreg(c->regs, GmacConfig, cfg | GmacConfigTe | GmacConfigRe);

		wreg(c->regs, DmaChanBase + DmaChanTxControl,
			rd32(c->regs, DmaChanBase + DmaChanTxControl) | DmaControlSt);
		v = rd32(c->regs, DmaChanBase + DmaChanRxControl);
		v = (v & ~DmaRbszMask) | ((Rbsz << DmaRbszShift) & DmaRbszMask);
		wreg(c->regs, DmaChanBase + DmaChanRxControl, v | DmaControlSr);
		c->dmarunning = 1;

		dprint("dwqe: readback GmacConfig=%#08ux DmaChanRxControl=%#08ux DmaChanTxControl=%#08ux "
			"DmaChanRxBase=%#08ux DmaChanRxEnd=%#08ux DmaChanRxLen=%#08ux "
			"MtlRxOpMode=%#08ux GmacPacketFilter=%#08ux GmacRxqCtrl0=%#08ux\n",
			rd32(c->regs, GmacConfig),
			rd32(c->regs, DmaChanBase + DmaChanRxControl),
			rd32(c->regs, DmaChanBase + DmaChanTxControl),
			rd32(c->regs, DmaChanBase + DmaChanRxBase),
			rd32(c->regs, DmaChanBase + DmaChanRxEnd),
			rd32(c->regs, DmaChanBase + DmaChanRxLen),
			rd32(c->regs, MtlChanBase + MtlRxOpMode),
			rd32(c->regs, GmacPacketFilter),
			rd32(c->regs, GmacRxqCtrl0));

		ethrx(edev);
		ethtx(edev);

		wreg(c->regs, DmaChanBase + DmaChanStatus, ~0U);
		wreg(c->regs, DmaChanBase + DmaChanIntrEna,
			DmaIntrNie | DmaIntrTie
			| DmaIntrRie | DmaIntrRbu
			| DmaIntrAie | DmaIntrFbe);

		print("dwqe: %s link up %dMbps %s on PHY %d "
			"(bmsr=%#04ux lpa=%#04ux mssr=%#04ux)\n",
			edev->name, speed, dup, c->phyaddr, bmsr, lpa, gbsr);
		ethersetspeed(edev, speed);
		ethersetlink(edev, 1);

		while((mdread(c, Bmsr) & BmsrLs) != 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);

		print("dwqe: %s link down\n", edev->name);
		ethersetlink(edev, 0);
		wreg(c->regs, GmacConfig, rd32(c->regs, GmacConfig)
			& ~(GmacConfigTe | GmacConfigRe));
		txreset(c);
		continue;

nextphy:
		c->phyidx = (c->phyidx + 1) % c->nphy;
	}
}

/* RX ring -------------------------------------------------------------- */

static int
replenish(Ctlr *c)
{
	Block *bp;
	u32int *r;
	uintptr pa;
	int i;

	/*
	 * Self-contained critical section: rxprodi, rxs[], and the ring
	 * memory are also touched by ethrx()'s dequeue step.
	 */
	ilock(&c->rxlock);

	/*
	 * Leave one slot permanently empty (classic circular-buffer
	 * capacity N-1): the RX tail-pointer register has no separate
	 * "full" indication, so head==tail must unambiguously mean
	 * "nothing available".
	 */
	while(c->rxs[c->rxprodi] == nil
	    && ((c->rxprodi + 1) & (Nrxd - 1)) != c->rxconsi){
		i  = c->rxprodi;
		bp = iallocb(Rbsz);
		if(bp == nil){
			iprint("dwqe: replenish: out of memory (prod=%d cons=%d)\n",
				c->rxprodi, c->rxconsi);
			iunlock(&c->rxlock);
			return -1;
		}
		c->rxs[i] = bp;
		r  = &c->rxr[4 * i];
		pa = PADDR(bp->rp);
		r[0] = (u32int)pa;			/* RDES0: buffer1 addr lo */
		r[1] = (u32int)(pa >> 32);		/* RDES1: buffer1 addr hi -- K3 DDR is >4G, this is never 0 */
		r[2] = 0;				/* RDES2: unused, single buffer */
		coherence();
		r[3] = Rdes3Own | Rdes3Buf1v
			| Rdes3Ioc;	/* RDES3 */
		coherence();	/* r is NC memory now; order the descriptor write, nothing to flush */
		dwqecacheinval(bp->rp, Rbsz);	/* bp->rp is ordinary cacheable memory -- keep */
		c->rxprodi = (c->rxprodi + 1) & (Nrxd - 1);
	}
	/*
	 * Always advance the tail pointer, even before DMA is running --
	 * writing it early is harmless (it's just a register the engine
	 * reads once SR is set), and it's what lets ethinit()'s initial
	 * fill actually be visible to the DMA engine the moment it starts.
	 */
	wreg(c->regs, DmaChanBase + DmaChanRxEnd,
		PADDR(c->rxr) + 16 * c->rxprodi);
	iunlock(&c->rxlock);
	return 0;
}

static void
ethrx(Ether *edev)
{
	Ctlr *c;
	u32int *r, s;
	Block *bp;
	int len;

	c = edev->ctlr;
	for(;;){
		/*
		 * Critical section: just the dequeue.  Deliberately
		 * released before calling replenish() or etheriq()
		 */
		ilock(&c->rxlock);
		r = &c->rxr[4 * c->rxconsi];	/* NC memory now; no invalidate needed to see the DMA engine's write-back */
		if(r[3] & Rdes3Own){
			iunlock(&c->rxlock);
			break;
		}
		if(r[3] & Rdes3Ctxt){
			/* shouldn't happen with rx_ic disabled context gen, skip */
			c->rxconsi = (c->rxconsi + 1) & (Nrxd - 1);
			iunlock(&c->rxlock);
			continue;
		}
		s   = r[3];
		len = s & Rdes3PktSizeMask;
		bp  = c->rxs[c->rxconsi];
		c->rxs[c->rxconsi] = nil;
		c->rxconsi = (c->rxconsi + 1) & (Nrxd - 1);
		iunlock(&c->rxlock);

		if(bp != nil && len > 0)
			dwqecacheinval(bp->rp, len);

		if(len < 4 || (s & Rdes3Es) || !(s & Rdes3Ld)){
			dprint("dwqe: RX error slot=%d rdes3=0x%08ux len=%d\n",
				c->rxconsi, s, len);
			if(bp != nil)
				freeb(bp);
		} else {
			bp->wp = bp->rp + len - 4;	/* strip FCS */
			etheriq(edev, bp);
			c->rxframes++;
		}
		replenish(c);
	}
}

static char*
ethifstat(void *arg, char *p, char *e)
{
	Ether *edev = arg;
	Ctlr *c = edev->ctlr;
	int i, ntx, nrx;

	if(p >= e)
		return p;
	ntx = 0;
	for(i = 0; i < Ntxd; i++)
		if(c->txs[i] != nil)
			ntx++;
	nrx = 0;
	for(i = 0; i < Nrxd; i++)
		if(c->rxs[i] != nil)
			nrx++;
	p = seprint(p, e, "tx: prod %d cons %d inflight %d done %ud\n",
		c->txi, c->txcons, ntx, c->txdone);
	p = seprint(p, e, "rx: prod %d cons %d bufs %d frames %ud\n",
		c->rxprodi, c->rxconsi, nrx, c->rxframes);
	p = seprint(p, e, "irq bits seen %#ux status %#ux intren %#ux\n",
		c->irqseen,
		rd32(c->regs, DmaChanBase + DmaChanStatus),
		rd32(c->regs, DmaChanBase + DmaChanIntrEna));
	p = seprint(p, e, "gmac config %#ux\n", rd32(c->regs, GmacConfig));
	p = seprint(p, e, "filter %#ux promisc=%d allmulti=%d\n",
		rd32(c->regs, GmacPacketFilter), c->promisc, c->allmulti);
	p = seprint(p, e, "rgmii delay rx %ud tx %ud steps\n", c->rxdelay, c->txdelay);
	return p;
}

/* TX ring ---------------------------------------------------------------- */

static void
ethtx(Ether *edev)
{
	Ctlr *c;
	u32int *r;
	Block *bp;
	uintptr pa;

	c = edev->ctlr;
	ilock(&c->txlock);
	while(c->txs[c->txcons] != nil){
		r = &c->txr[4 * c->txcons];	/* NC memory now; no invalidate needed */
		if(r[3] & Tdes3Own)
			break;
		freeb(c->txs[c->txcons]);
		c->txs[c->txcons] = nil;
		c->txdone++;
		c->txcons = (c->txcons + 1) & (Ntxd - 1);
	}
	for(;;){
		r = &c->txr[4 * c->txi];	/* NC memory now; no invalidate needed */
		if(r[3] & Tdes3Own){
			dprint("dwqe: TX ring full (txi=%d)\n", c->txi);
			break;
		}
		if(c->txs[c->txi] != nil){
			freeb(c->txs[c->txi]);
			c->txs[c->txi] = nil;
		}
		bp = qget(edev->oq);
		if(bp == nil)
			break;
		c->txs[c->txi] = bp;
		pa = PADDR(bp->rp);
		dwqecacheflush(bp->rp, BLEN(bp));

		r[0] = (u32int)pa;			/* TDES0: buffer1 addr lo */
		r[1] = (u32int)(pa >> 32);		/* TDES1: buffer1 addr hi -- K3 DDR is >4G, this is never 0 */
		r[2] = BLEN(bp) & Tdes2Buf1SizeMask;
		if((c->txi & (Txirqevery - 1)) == 0)
			r[2] |= Tdes2Ioc;
		coherence();
		r[3] = Tdes3Fd | Tdes3Ld
			| (BLEN(bp) & Tdes3PktSizeMask) | Tdes3Own;
		coherence();	/* r is NC memory now; order the descriptor write, nothing to flush */

		c->txi = (c->txi + 1) & (Ntxd - 1);
		if(c->dmarunning)
			wreg(c->regs, DmaChanBase + DmaChanTxEnd,
				PADDR(c->txr) + 16 * c->txi);
	}
	iunlock(&c->txlock);
}

/* IRQ / polling ------------------------------------------------------------ */

static void
ethirq(Ureg*, void *arg)
{
	Ether *edev;
	Ctlr *c;
	u32int fl, en;

	edev = arg;
	c    = edev->ctlr;

	en = rd32(c->regs, DmaChanBase + DmaChanIntrEna);
	if(en != 0){
		wreg(c->regs, DmaChanBase + DmaChanIntrEna, 0);
		rd32(c->regs, DmaChanBase + DmaChanIntrEna);	/* drain posted write */
	}

	fl = rd32(c->regs, DmaChanBase + DmaChanStatus);
	if(fl != 0){
		wreg(c->regs, DmaChanBase + DmaChanStatus, fl);	/* W1C */
		if(fl & DmaStatusRbu){
			iprint("dwqe: %s RX buffer unavailable (status=%#08ux prod=%d cons=%d)\n",
				edev->name, fl, c->rxprodi, c->rxconsi);
			replenish(c);
		}
		if(fl & DmaStatusFbe)
			iprint("dwqe: %s fatal bus error (status=%#08ux)\n", edev->name, fl);
	}

	ethrx(edev);
	ethtx(edev);

	if(en != 0){
		coherence();
		wreg(c->regs, DmaChanBase + DmaChanIntrEna, en);
	}
}

static Ether *dwqeether[Ndwqe];
static Ctlr   dwqectlr[Ndwqe];
static int    dwqeclockregistered;

static void
etherserve(Ether *edev)
{
	Ctlr *c;
	u32int fl;

	c = edev->ctlr;
	fl = rd32(c->regs, DmaChanBase + DmaChanStatus);
	c->irqseen |= fl;
	ethirq(nil, edev);

	if(debug && ++c->statstick >= 5000){
		c->statstick = 0;
		print("dwqe: %s stats irq_seen=0x%08ux rxcons=%d rxprod=%d rx_frames=%ud "
			"txi=%d tx_done=%ud\n",
			edev->name, c->irqseen, c->rxconsi, c->rxprodi, c->rxframes,
			c->txi, c->txdone);
		c->irqseen = 0;
	}
}

static void
etherclock(void)
{
	int i;

	for(i = 0; i < Ndwqe; i++)
		if(dwqeether[i] != nil)
			etherserve(dwqeether[i]);
}

static void
ethattach(Ether *edev)
{
	Ctlr *c;

	c = edev->ctlr;
	if(c->attach)
		return;
	c->attach = 1;
	kproc("dwqeproc", ethproc, edev);
}

/* /net/etherN/clone handler ----------------------------------------------- */

enum { CMdebug, CMpromiscuous, CMallmulti };
static Cmdtab dwqectlmsg[] = {
	CMdebug,	"debug",	2,
	CMpromiscuous,	"promisc",	2,
	CMallmulti,	"allmulti",	2,
};

static long
dwqectl(Ether *edev, void *buf, long n)
{
	Ctlr *c;
	Cmdbuf *cb;
	Cmdtab *ct;

	c = edev->ctlr;
	cb = parsecmd(buf, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	ct = lookupcmd(cb, dwqectlmsg, nelem(dwqectlmsg));
	switch(ct->index){
	case CMdebug:
		debug = atoi(cb->f[1]);
		break;
	case CMpromiscuous:
		dwqesetfilter(c, atoi(cb->f[1]) != 0, c->allmulti);
		break;
	case CMallmulti:
		dwqesetfilter(c, c->promisc, atoi(cb->f[1]) != 0);
		break;
	}
	free(cb);
	poperror();
	return n;
}

/*
 * GMAC pad groups.  The pads are normally muxed by the boot loader, but
 * it only sets up the port it booted through; a cold boot to Plan 9 has
 * no Ethernet at all until we do it.  K3 pad N is at pinctrl + 4*N and
 * function 1 is the GMAC function on every port.
 */
static u32int *pinctrl;

/*
 * Set mux, drive strength and bias.  The GMAC pad groups ask for 9mA
 * and bias-disable; the reset line asks for the maximum drive.  RX
 * works without this (the PHY drives those pads) but TX does not.
 */
static void
dwqepadcfg(uint pin, uint func, uint drive)
{
	u32int v;

	if(pinctrl == nil || pin >= 128)
		return;
	v = pinctrl[pin];
	v &= ~(PadMux | PadDriveMask | PadPullMask);
	v |= (func & PadMux) | ((drive & 0xf) << PadDriveShift);
	pinctrl[pin] = v;
}

static void
dwqepads(Ctlr *c, uvlong base)
{
	static u32int *apbc;
	uint lo, hi, pdoff, i, n;
	u32int v;

	switch(base){
	case Gmac0Phys:		lo = 0;  hi = 20; pdoff = 0x04; break;
	case 0xcac82000UL:	lo = 21; hi = 35; pdoff = 0x0c; break;
	case 0xcac8e000UL:	lo = 42; hi = 58; pdoff = 0; break;
	case 0xa0000000UL:	lo = 59; hi = 75; pdoff = 0; break;
	default:
		return;
	}

	if(apbc == nil)
		apbc = vmap(ApbcPhys, ApbcSize);
	if(apbc != nil){
		apbc[ApbcAib] = (apbc[ApbcAib] & ~ApbcReset)
			| ApbcBusEn | ApbcFuncEn;
		coherence();
	}
	if(pinctrl == nil)
		pinctrl = vmap(PinctrlPhys, PinctrlSize);
	if(pinctrl == nil)
		return;

	/*
	 * The GMAC pads share an IO-power domain that the device tree
	 * runs at 1.8V.  A cold boot may leave it at 3.3V, which breaks
	 * the RGMII/MDIO levels (RX can still work).  The domain
	 * registers are protected: unlock via ASFAR/ASSAR for each access.
	 */
	if(apbc != nil && pdoff != 0){
		u32int *pd = &pinctrl[(PwrDomainOff + pdoff)/4];
		u32int before;

		apbc[ApbcAsfar] = ApbcAsfarKey;
		apbc[ApbcAssar] = ApbcAssarKey;
		before = *pd;
		apbc[ApbcAsfar] = ApbcAsfarKey;
		apbc[ApbcAssar] = ApbcAssarKey;
		*pd = PwrV18En;
		coherence();
		print("dwqe: IO domain %#ux: %#ux -> 1.8V\n", pdoff, before);
	}

	n = 0;
	for(i = lo; i <= hi; i++){
		v = pinctrl[i];
		dwqepadcfg(i, 1, PadDrive9mA);
		if(v != pinctrl[i])
			n++;
	}
	/* the PHY reset line is a GPIO, not a GMAC pad */
	if(c->rstpin >= 0)
		dwqepadcfg(c->rstpin, 0, PadDrive25mA);
	if(n)
		print("dwqe: configured %ud GMAC pads for %#llux\n", n, base);
}

/*
 * Release the PHY reset and give it time to start its receive clock.
 * The DMA soft reset below will not complete until that clock runs.
 */
static void
dwqephyreset(Ctlr *c)
{
	int assertlvl, releaselvl;

	if(c->rstpin < 0 || c->rstpin >= 128)
		return;
	gpiok3init();		/* map and clock the GPIO block; idempotent */
	if(c->rstactive){
		assertlvl = 0;
		releaselvl = 1;
	}else{
		assertlvl = 1;
		releaselvl = 0;
	}
	gpioout(c->rstpin, assertlvl);
	microdelay(20000);
	gpioout(c->rstpin, releaselvl);
	microdelay(100000);
	print("dwqe: PHY reset released on gpio%d\n", c->rstpin);
}

/* Core bring-up ------------------------------------------------------------ */

static int
ethinit(Ether *edev)
{
	Ctlr *c;
	u32int v, txclk;
	int i, timeout;

	c = edev->ctlr;

	/*
	 * RGMII TX clock source: the SoC's PLL2 (pll2_d3 /8, gated in the
	 * PLL controller) or the PHY's RX clock.  Firmware may not have
	 * enabled the PLL path.  Use PLL2 only if it is locked; otherwise
	 * fall back to the PHY clock, which the reset below makes sure is
	 * running.
	 */
	{
		u32int *mpmu, *pll;
		u32int posr, swcr2;

		mpmu = vmap(MpmuPhys, MpmuSize);
		pll = vmap(PllPhys, PllSize);
		posr = mpmu ? rd32(mpmu, MpmuPosr) : 0;
		swcr2 = pll ? rd32(pll, Pll2Swcr2) : 0;
		if((posr & Pll2Lock) && pll != nil){
			if((swcr2 & (Pll2Enable|Pll2D3Gate))
			   != (Pll2Enable|Pll2D3Gate))
				wreg(pll, Pll2Swcr2,
					swcr2 | Pll2Enable | Pll2D3Gate);
			txclk = Emac0RgmiiTxcSrcSel;
		}else
			txclk = 0;
		print("dwqe: RGMII TX clock from %s (posr=%#08ux swcr2=%#08ux)\n",
			txclk ? "SoC PLL2" : "PHY RX clock", posr, swcr2);
		if(mpmu)
			vunmap(mpmu, MpmuSize);
		if(pll)
			vunmap(pll, PllSize);
	}

	/*
	 * APMU: bring GMAC0 out of reset with clocks enabled, PHY interface
	 * mode selected, and RGMII delay lines set -- all BEFORE touching
	 * any MAC/DMA register. Skipping it stalls the hart.
	 */
	{
		u32int origv = rd32(c->apmu, c->apmuctrloff);

		/*
		 * Grab whatever real MAC firmware left in GMAC_ADDR0 *now*,
		 * before the sequence below forces an EmacBusRst 0->1
		 * transition -- that's a real reset of the MAC's internal
		 * register file (ADDR0 included). Only safe to read at all
		 * if RST is already 1 in origv -- if it's 0, the register
		 * block isn't accessible yet and reading would hang the
		 * hart the same way skipping the bring-up entirely did.
		 */
		if((origv & EmacBusRst) != 0){
			static uchar zero[Eaddrlen];

			if(memcmp(edev->ea, zero, Eaddrlen) == 0){
				u32int hi, lo;

				wreg(c->apmu, c->apmuctrloff, origv | EmacBusEn);
				microdelay(1000);
				lo = rd32(c->regs, GmacAddr0Lo);
				hi = rd32(c->regs, GmacAddr0Hi);
				edev->ea[0] = lo & 0xff;        edev->ea[1] = (lo>>8) & 0xff;
				edev->ea[2] = (lo>>16) & 0xff;  edev->ea[3] = (lo>>24) & 0xff;
				edev->ea[4] = hi & 0xff;        edev->ea[5] = (hi>>8) & 0xff;
				dprint("dwqe: MAC captured from hardware before reset: "
					"%02X:%02X:%02X:%02X:%02X:%02X\n",
					edev->ea[0], edev->ea[1], edev->ea[2],
					edev->ea[3], edev->ea[4], edev->ea[5]);
			}
		}

		u32int preserve = origv & ~(EmacBusEn | EmacBusRst
			| EmacAxiMstId | Emac1588ClkMux
			| (3U << EmacPhySelectShift)
			| Emac0ClkRefGate | Emac0PhyIntrEn | Emac0LpiIntrEn
			| Emac0RgmiiTxcSrcSel | EmacRxRefclkPhaseSel
			| EmacRmiiTxRefclkPhaseSel);
		u32int physel = EmacPhySelectRgmii << EmacPhySelectShift;
		/*
		 * CLK_REF_GATE=0 (refclk enabled, unused in RGMII anyway),
		 * PHY_INTR_EN=0 / LPI_INTR_EN=0 (no WoL/EEE handler exists),
		 * *_PHASE_SEL=0 (no skew correction needed, RMII one is don't-care).
		 */
		wreg(c->apmu, c->apmuctrloff,
			preserve | physel | txclk);
		microdelay(10000);
		wreg(c->apmu, c->apmuctrloff,
			preserve | physel | txclk | EmacBusEn | Emac1588ClkMux);
		microdelay(10000);
		wreg(c->apmu, c->apmuctrloff,
			preserve | physel | txclk
			| EmacBusEn | Emac1588ClkMux | EmacBusRst | EmacAxiMstId);
		microdelay(10000);
		print("dwqe: APMU CLK_RST[%#ux] %#ux -> %#ux (PHY_SELECT=RGMII, TXC_SRC_SEL=%s)\n",
			c->apmuctrloff, origv, rd32(c->apmu, c->apmuctrloff),
			txclk ? "SoC" : "RXclk");
	}

	/* RGMII delay lines */
	v = EmacRgmiiRxcDlinePu | EmacRgmiiTxcDlinePu
	  | EmacRgmiiRxcDlineStep | EmacRgmiiTxcDlineStep
	  | c->rxdelay << EmacRgmiiRxcDlineAdjShift
	  | c->txdelay << EmacRgmiiTxcDlineAdjShift;
	wreg(c->apmu, c->apmudlineoff, v);
	print("dwqe: APMU DLINE[%#ux]=0x%08ux\n", c->apmudlineoff, v);

	/* CR sanity-check, kept as a debug-only cross-check  */
	{
		u32int aclkctrl = rd32(c->apmu, 0x388/4);
		int mux = aclkctrl & 1;
		int div = ((aclkctrl >> 1) & 3) + 1;
		u32int parent = mux ? 409000000 : 307000000;
		u32int csrclk = parent / div;
		int wantcr;

		if(csrclk > 500000000) wantcr = 7;
		else if(csrclk > 300000000) wantcr = 6;
		else if(csrclk > 250000000) wantcr = 5;
		else if(csrclk > 150000000) wantcr = 4;
		else if(csrclk > 100000000) wantcr = 1;
		else if(csrclk > 60000000) wantcr = 0;
		else if(csrclk > 35000000) wantcr = 3;
		else wantcr = 2;

		dprint("dwqe: APMU_ACLK_CLK_CTRL=%#08ux mux=%d div=%d -> "
			"csrclk~%ud Hz -> CR=%d indicated (using %d)\n",
			aclkctrl, mux, div, csrclk, wantcr, MdioCrVal);
	}

	/* now the MAC block is reachable; stop everything */
	wreg(c->regs, GmacIntEn, 0);
	wreg(c->regs, GmacConfig, 0);
	wreg(c->regs, DmaChanBase + DmaChanTxControl, 0);
	wreg(c->regs, DmaChanBase + DmaChanRxControl, 0);

	/*
	 * If neither a bootarg, the DTB, nor the early hardware capture
	 * above got us a MAC address, edev->ea is still zeroed here.
	 */
	{
		static uchar zero[Eaddrlen];

		if(memcmp(edev->ea, zero, Eaddrlen) == 0) {
			print("dwqe: ERROR no MAC address available for %s -- set "
				"mac-address in the DTB or pass ea= at boot\n", edev->name);
			return -1;
		}
	}

	/*
	 * Cold boot: the boot loader may not have touched this port.  Mux
	 * its pads and release the PHY reset first; the DMA soft reset
	 * needs the PHY's receive clock to complete.
	 */
	dwqepads(c, edev->port);
	dwqephyreset(c);

	/*
	 * Find the PHYs and bring them out of power-down before the DMA
	 * reset: a PHY with no link may stop the RGMII receive clock,
	 * and the reset waits for it.
	 */
	mdioscan(c, 0);
	for(i = 0; i < c->nphy; i++){
		c->phyidx = i;
		physetup(c);
	}

	/* DMA software reset -- polled */
	wreg(c->regs, DmaBusMode, rd32(c->regs, DmaBusMode) | DmaBusModeSwr);
	for(timeout = 0; timeout < 1000; timeout++){
		if((rd32(c->regs, DmaBusMode) & DmaBusModeSwr) == 0)
			break;
		microdelay(1000);
	}
	if(timeout == 1000){
		print("dwqe: ERROR DmaBusMode software reset timed out\n");
		mdioscan(c, 1);
		return -1;
	}
	dprint("dwqe: DMA reset done after %d ms\n", timeout);

	/*
	 * K3 DDR starts at 0x1_00000000 -- everything iallocb()/ucallocalign()
	 * hands us will have PADDR() >= 4G, so 64-bit DMA addressing is required.
	 */
	wreg(c->regs, DmaSysBusMode, DmaSysBusMb | DmaSysBusFb | DmaSysBusEame);

	/* MTL: store-and-forward, enable queue 0, size FIFOs from HW_FEATURE1 */
	{
		u32int feat1 = rd32(c->regs, GmacHwFeature1);
		u32int txfifo = 128 << ((feat1 & GmacHwTxFifoSize) >> 6);
		u32int rxfifo = 128 << (feat1 & GmacHwRxFifoSize);
		u32int tqs = txfifo / 256 - 1;
		u32int rqs = rxfifo / 256 - 1;

		print("dwqe: HW_FEATURE1=%#08ux txfifo=%ud rxfifo=%ud bytes\n",
			feat1, txfifo, rxfifo);

		wreg(c->regs, MtlChanBase + MtlTxOpMode,
			MtlOpTsf | MtlOpTxqEn
			| ((tqs << 16) & MtlOpTqsMask));
		wreg(c->regs, MtlChanBase + MtlRxOpMode,
			MtlOpRsf | ((rqs << 20) & MtlOpRqsMask));
	}

	/* MAC address */
	wreg(c->regs, GmacAddr0Hi,
		(1U << 31) | edev->ea[5]<<8 | edev->ea[4]);
	wreg(c->regs, GmacAddr0Lo,
		edev->ea[3]<<24 | edev->ea[2]<<16 | edev->ea[1]<<8 | edev->ea[0]);
	dprint("dwqe: MAC %02X:%02X:%02X:%02X:%02X:%02X programmed\n",
		edev->ea[0], edev->ea[1], edev->ea[2], edev->ea[3], edev->ea[4], edev->ea[5]);

	/*
	 * Enable RX queue 0 at the MAC's queue-routing level.  This is
	 * upstream of MTL/DMA entirely -- without it the MAC can silently
	 * drop everything before it ever reaches the MTL FIFO.
	 */
	wreg(c->regs, GmacRxqCtrl0, GmacRxqCtrl0Q0Dcb);

	/*
	 * Real filtering now: unicast perfect-match on GMAC_ADDR0 +
	 * broadcast + all multicast. Toggle via the /net/etherN/clone
	 * file ("promisc 1"/"allmulti 0") if you need something
	 * tighter or looser at runtime.
	 */
	dwqesetfilter(c, 0, 1);

	/* GmacConfig base bits; RE/TE turned on by ethproc once link is up */
	wreg(c->regs, GmacConfig,
		GmacConfigJd | GmacConfigBe | GmacConfigDcrs | GmacConfigAcs);

	/* descriptor rings -- both non-cacheable (4 descs/line) */
	c->rxr = ucallocalign(sizeof(u32int) * 4 * Nrxd, 16, 0);
	c->txr = ucallocalign(sizeof(u32int) * 4 * Ntxd, 16, 0);
	if(c->rxr == nil || c->txr == nil){
		print("dwqe: ERROR ring alloc failed\n");
		return -1;
	}
	memset(c->rxr, 0, 4 * Nrxd * sizeof(u32int));
	memset(c->txr, 0, 4 * Ntxd * sizeof(u32int));
	coherence();	/* both rings are NC memory now; order the zeroing, nothing to flush */

	c->rxs = mallocz(sizeof(Block*) * Nrxd, 1);
	c->txs = mallocz(sizeof(Block*) * Ntxd, 1);
	if(c->rxs == nil || c->txs == nil){
		print("dwqe: ERROR mallocz failed\n");
		return -1;
	}

	c->rxprodi = 0;
	c->rxconsi = 0;
	c->txi = 0;
	c->txcons = 0;

	{
		uvlong txpa = PADDR(c->txr), rxpa = PADDR(c->rxr);
		uvlong txend = txpa + 4 * Ntxd * sizeof(u32int) - 1;
		uvlong rxend = rxpa + 4 * Nrxd * sizeof(u32int) - 1;

		/* The tail-pointer registers are 32-bit only */
		if((txpa >> 32) != (txend >> 32) || (rxpa >> 32) != (rxend >> 32)){
			print("dwqe: ERROR ring straddles a 4G boundary "
				"(tx=%#llux..%#llux rx=%#llux..%#llux)\n",
				txpa, txend, rxpa, rxend);
			return -1;
		}

		wreg(c->regs, DmaChanBase + DmaChanTxBaseHi, txpa >> 32);
		wreg(c->regs, DmaChanBase + DmaChanTxBase, (u32int)txpa);
		wreg(c->regs, DmaChanBase + DmaChanRxBaseHi, rxpa >> 32);
		wreg(c->regs, DmaChanBase + DmaChanRxBase, (u32int)rxpa);
		wreg(c->regs, DmaChanBase + DmaChanTxLen, Ntxd - 1);
		wreg(c->regs, DmaChanBase + DmaChanRxLen, Nrxd - 1);
		wreg(c->regs, DmaChanBase + DmaChanTxEnd, (u32int)txpa);
	}

	/*
	 * Fills the ring (leaving the one sentinel gap slot empty and sets
	 * DmaChanRxEnd itself, now that DmaChanRxBase is already programmed.
	 */
	replenish(c);

	dprint("dwqe: rings rxr=%p txr=%p (%d/%d slots)\n", c->rxr, c->txr, Nrxd, Ntxd);

	/* DMA channel stays stopped (ST/SR off); ethproc starts both on link-up */
	wreg(c->regs, DmaChanBase + DmaChanStatus, ~0U);
	wreg(c->regs, DmaChanBase + DmaChanIntrEna, 0);
	wreg(c->regs, GmacIntEn, 0);

	return 0;
}

/* Plug-and-play -------------------------------------------------------- */

static int
etherpnp(Ether *edev)
{
	Ctlr *ct;
	Dtbether *de;
	uvlong base, size, apmubase;
	int irq, i, k, n;

	dprint("dwqe: etherpnp ctlrno=%d\n", edev->ctlrno);

	if(edev->ctlrno < 0 || edev->ctlrno >= Ndwqe){
		dprint("dwqe: ctlrno %d out of range\n", edev->ctlrno);
		return -1;
	}
	ct = &dwqectlr[edev->ctlrno];
	if(ct->regs != nil){
		dprint("dwqe: ether%d already registered\n", edev->ctlrno);
		return -1;
	}

	if(!dtbsocmatch("spacemit,k3")){
		dprint("dwqe: not a K3, skipping\n");
		return -1;
	}

	/* etherN is the Nth enabled gmac node, in device-tree order. */
	de = nil;
	n = dtbnether();
	for(i = k = 0; i < n; i++){
		Dtbether *d = dtbether(i);
		if(d != nil && d->ok
		   && strstr(d->compatible, "spacemit,k3-gmac") != nil){
			if(k++ == edev->ctlrno){
				de = d;
				break;
			}
		}
	}
	if(de != nil){
		base     = de->base;
		size     = de->size ? de->size : Gmac0Size;
		apmubase = de->apmubase ? de->apmubase : ApmuPhys;
		irq      = de->irq;
		ct->apmuctrloff  = de->ctrlregoff  ? de->ctrlregoff/4  : 0x3e4/4;
		ct->apmudlineoff = de->dlineregoff ? de->dlineregoff/4 : 0x3e8/4;
		ct->rstpin       = de->rstpin;
		ct->rstactive    = de->rstactive;
		ct->phyaddr      = PhyAddr;
		ct->mdiocr       = MdioCrVal;
		/*
		 * The fallbacks above are EMAC0's APMU registers.  Driving
		 * another port's MAC with them leaves it gated, and the
		 * first MAC access then hangs the bus -- refuse instead.
		 */
		if((de->ctrlregoff == 0 || de->dlineregoff == 0)
		   && base != Gmac0Phys){
			print("dwqe: ether%d at %#llux: DTB has no APMU ctrl/dline offset\n",
				edev->ctlrno, base);
			return -1;
		}
		print("dwqe: ether%d from DTB %s base=%#llux size=%#llux apmu=%#llux+{%#ux,%#ux} irq=%d\n",
			edev->ctlrno, de->path, base, size, apmubase,
			ct->apmuctrloff, ct->apmudlineoff, irq);
	} else if(edev->ctlrno == 0 && n == 0){
		base     = Gmac0Phys;
		size     = Gmac0Size;
		apmubase = ApmuPhys;
		irq      = Gmac0Irq;
		ct->apmuctrloff  = 0x3e4/4;
		ct->apmudlineoff = 0x3e8/4;
		ct->rstpin       = -1;
		ct->phyaddr      = PhyAddr;
		ct->mdiocr       = MdioCrVal;
		print("dwqe: no DTB, using K3 ether0 defaults\n");
	} else {
		dprint("dwqe: no DTB entry for ether%d\n", edev->ctlrno);
		return -1;
	}

	ct->regs = (u32int*)vmap(base, size);
	ct->apmu = (u32int*)vmap(apmubase, ApmuSize);
	if(ct->regs == nil || ct->apmu == nil){
		print("dwqe: ERROR vmap failed regs=%p apmu=%p\n", ct->regs, ct->apmu);
		ct->regs = nil;
		return -1;
	}
	ct->irq = irq;
	dprint("dwqe: ether%d regs=%p (phys=%#llux) apmu=%p (phys=%#llux)\n",
		edev->ctlrno, ct->regs, base, ct->apmu, apmubase);

	ct->rxdelay = DwqeRxDelay;
	ct->txdelay = DwqeTxDelay;
	if(de != nil){
		if(de->txphase > 0)
			ct->txdelay = (de->txphase * 8000 / 360 * 10 + 78) / 156;
		if(de->rxphase > 0)
			ct->rxdelay = (de->rxphase * 8000 / 360 * 10 + 78) / 156;
	}

	edev->ctlr     = ct;
	edev->port     = base;
	edev->irq      = irq;
	edev->attach   = ethattach;
	edev->transmit = ethtx;
	edev->ifstat   = ethifstat;
	edev->arg      = edev;
	edev->mbps     = 1000;

	{
		static uchar zero[Eaddrlen];
		if(memcmp(edev->ea, zero, Eaddrlen) != 0){
			dprint("dwqe: MAC from bootarg\n");
		} else if(de != nil && de->hasmac && memcmp(de->mac, zero, Eaddrlen) != 0){
			memmove(edev->ea, de->mac, Eaddrlen);
			dprint("dwqe: MAC from DTB: %02X:%02X:%02X:%02X:%02X:%02X\n",
                edev->ea[0], edev->ea[1], edev->ea[2],
                edev->ea[3], edev->ea[4], edev->ea[5]);
		}
	}

	if(ethinit(edev) < 0){
		print("dwqe: ERROR ethinit failed\n");
		edev->ctlr = nil;
		return -1;
	}

	dwqeether[edev->ctlrno] = edev;
	edev->ctl = dwqectl;

	if(irq > 0){
		/*
		 * intrenable() dispatches to aplic.c (MSI mode, via IMSIC)
		 * since this is a K3 -- see intr.c's soc.plic == nil check.
		 * No PLIC involved despite the historical name.
		 */
		intrenable(irq, ethirq, edev, BUSUNKNOWN, edev->name);
		dprint("dwqe: %s irq-driven on irq %d (APLIC/IMSIC)\n", edev->name, irq);
	}
	if(!dwqeclockregistered){
		addclock0link(etherclock, 1);
		dwqeclockregistered = 1;
	}
	return 0;
}

/* Driver registration ----------------------------------------------------- */

void
etherdwqelink(void)
{
	addethercard("dwqe", etherpnp);
}
