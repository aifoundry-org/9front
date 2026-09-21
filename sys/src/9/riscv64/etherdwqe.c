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

	DwqeRxDelay	= 100,
	DwqeTxDelay	= 100,
};

/* GMAC (MAC-level) registers */
enum {
	GmacConfig			= 0x0000,
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

	GmacRxqCtrl0			= 0x00a0,
	GmacRxqCtrl0Q0Dcb		= (1U << 1),	/* BIT((queue*2)+1) for queue=0 */

	GmacPacketFilter		= 0x0008,
	GmacPfPr		= (1U << 0),	/* promiscuous */
	GmacPfHmc		= (1U << 2),	/* hash multicast */
	GmacPfPm		= (1U << 4),	/* pass all multicast */
	GmacPfHpf		= (1U << 10),	/* hash or perfect filter */
	GmacPfRa		= (1U << 31),	/* receive all */

	GmacIntStatus			= 0x00b0,
	GmacIntEn			= 0x00b4,

	GmacMdioAddr			= 0x0200,
	GmacMdioData			= 0x0204,

	GmacHwFeature0		= 0x011c,
	GmacHwFeature1		= 0x0120,

	GmacAddr0Hi			= 0x0300,
	GmacAddr0Lo			= 0x0304,
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
	DmaBusMode			= 0x1000,
	DmaBusModeSwr		= (1U << 0),

	DmaSysBusMode		= 0x1004,
	DmaSysBusMb			= (1U << 14),
	DmaSysBusEame		= (1U << 11),	/* Enhanced Address Mode: enables all *_HI regs + des1 */
	DmaSysBusFb			= (1U << 0),	/* Fixed burst */
};

/* DMA per-channel registers (channel 0 only -- dwqe uses 1 queue) */
enum {
	DmaChanBase			= 0x1100,

	DmaChanControl		= 0x00,
	DmaChanTxControl		= 0x04,
	DmaControlSt			= (1U << 0),

	DmaChanRxControl		= 0x08,
	DmaControlSr			= (1U << 0),
	DmaRbszShift			= 1,		/* bits [14:1], raw byte count << 1 */
	DmaRbszMask			= 0x7ffe,

	DmaChanTxBaseHi	= 0x10,		/* bits [63:32] of ring base */
	DmaChanTxBase		= 0x14,		/* bits [31:0]  of ring base */
	DmaChanRxBaseHi	= 0x18,		/* bits [63:32] of ring base */
	DmaChanRxBase		= 0x1c,		/* bits [31:0]  of ring base */
	/*
	 * Tail pointer has no _HI counterpart -- the ring's upper 32 bits
	 * come from *_BASE_ADDR_HI above and apply to the whole ring
	 */
	DmaChanTxEnd		= 0x20,		/* tail pointer, 32-bit */
	DmaChanRxEnd		= 0x28,		/* tail pointer, 32-bit */
	DmaChanTxLen		= 0x2c,		/* N-1 */
	DmaChanRxLen		= 0x30,		/* N-1 */

	DmaChanIntrEna		= 0x34,
	DmaIntrNie		= (1U << 16),
	DmaIntrAie		= (1U << 15),
	DmaIntrFbe		= (1U << 12),
	DmaIntrRbu		= (1U << 7),
	DmaIntrRie		= (1U << 6),
	DmaIntrTie		= (1U << 0),

	DmaChanStatus			= 0x60,		/* W1C */
	DmaStatusNis		= (1U << 15),
	DmaStatusAis		= (1U << 14),
	DmaStatusFbe		= (1U << 12),
	DmaStatusRbu		= (1U << 7),
	DmaStatusRi		= (1U << 6),
	DmaStatusTi		= (1U << 0),
};

/* MTL (queue) registers */
enum {
	MtlChanBase			= 0x0d00,
	MtlTxOpMode		= 0x00,
	MtlOpTxqEn		= (1U << 3),
	MtlOpTsf			= (1U << 1),	/* store-and-forward */
	MtlOpTqsMask		= 0x01ff0000,	/* bits [24:16] */

	MtlRxOpMode		= 0x30,
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
	Emac0RgmiiTxcSrcSel		= (1U << 8),	/* RGMII only: 1 = TX clock from SoC, 0 = from RX clock */
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
};

enum {
	Nrxd	= 512,
	Ntxd	= 512,
	Txirqevery = 64,
	Ndwqe	= 1,		/* GMAC0 only for now; GMAC1/2 follow the same pattern */

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
	ulong	*regs;
	ulong	*apmu;
	u32int	apmuctrloff;
	u32int	apmudlineoff;
	u32int	*rxr, *txr;
	Block	**rxs, **txs;
	int	rxprodi, rxconsi, txi;
	u32int	rxdelay, txdelay;
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

/* MMIO access -- never touch a raw physical address, only vmap'd bases */

static u32int
csr32r(ulong *base, uintptr off)
{
	coherence();
	return *(volatile u32int*)((uintptr)base + off);
}

static void
csr32w(ulong *base, uintptr off, u32int v)
{
	*(volatile u32int*)((uintptr)base + off) = v;
	coherence();
}

/* MDIO -- GMAC4-style GOC field */

static void
mdwrite(Ctlr *c, int reg, u16int v)
{
	u32int addr;

	while(csr32r(c->regs, GmacMdioAddr) & MdioGbusy)
		tsleep(&up->sleep, return0, nil, 1);
	csr32w(c->regs, GmacMdioData, v);
	addr = (PhyAddr & 0x1f) << MdioPaShift
		| (reg & 0x1f) << MdioGrShift
		| MdioCrVal << MdioCrShift
		| MdioWrite
		| MdioGbusy;
	csr32w(c->regs, GmacMdioAddr, addr);
	while(csr32r(c->regs, GmacMdioAddr) & MdioGbusy)
		tsleep(&up->sleep, return0, nil, 1);
}

static u16int
mdread(Ctlr *c, int reg)
{
	u32int addr;

	while(csr32r(c->regs, GmacMdioAddr) & MdioGbusy)
		tsleep(&up->sleep, return0, nil, 1);
	addr = (PhyAddr & 0x1f) << MdioPaShift
		| (reg & 0x1f) << MdioGrShift
		| MdioCrVal << MdioCrShift
		| MdioRead
		| MdioGbusy;
	csr32w(c->regs, GmacMdioAddr, addr);
	while(csr32r(c->regs, GmacMdioAddr) & MdioGbusy)
		tsleep(&up->sleep, return0, nil, 1);
	return csr32r(c->regs, GmacMdioData) & MdioGdMask;
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
	csr32w(c->regs, GmacPacketFilter, v);
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

	v = csr32r(c->regs, DmaChanBase + DmaChanTxControl);
	csr32w(c->regs, DmaChanBase + DmaChanTxControl, v & ~DmaControlSt);
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
	csr32w(c->regs, DmaChanBase + DmaChanTxBase, PADDR(c->txr));
	csr32w(c->regs, DmaChanBase + DmaChanTxEnd, PADDR(c->txr));
}

/* PHY / link kproc -- same RTL8211F sequence as ethersmte.c */
static void
ethproc(void *ved)
{
	Ether *edev;
	Ctlr *c;
	u16int bmsr, lpa, gbsr, id1, id2;
	u32int cfg, v;
	int speed;
	char *dup;

	edev = ved;
	c    = edev->ctlr;

	id1 = mdread(c, Phyidr1);
	id2 = mdread(c, Phyidr2);
	dprint("dwqe: PHY addr=%d oui=0x%06ux model=0x%02ux rev=0x%01ux\n",
		PhyAddr, ((u32int)id1 << 6) | (id2 >> 10), (id2 >> 4) & 0x3f, id2 & 0xf);

	mdwrite(c, Mscr, Mscr1000TFD | Mscr1000THD);
	mdwrite(c, Bmcr, BmcrAne | BmcrRan);

	for(;;){
		bmsr = mdread(c, Bmsr);
		if((bmsr & BmsrLs) == 0){
			print("dwqe: %s link down (BMSR=0x%04ux)\n", edev->name, bmsr);
			ethersetlink(edev, 0);
			csr32w(c->regs, GmacConfig, csr32r(c->regs, GmacConfig)
				& ~(GmacConfigTe | GmacConfigRe));
			txreset(c);
			while((mdread(c, Bmsr) & BmsrLs) == 0)
				tsleep(&up->sleep, return0, nil, Linkdelay);
		}

		while((mdread(c, Bmsr) & BmsrAnc) == 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);

		lpa  = mdread(c, Anlpar);
		gbsr = mdread(c, Mssr);

		cfg = csr32r(c->regs, GmacConfig)
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

		csr32w(c->regs, GmacConfig, cfg | GmacConfigTe | GmacConfigRe);

		csr32w(c->regs, DmaChanBase + DmaChanTxControl,
			csr32r(c->regs, DmaChanBase + DmaChanTxControl) | DmaControlSt);
		v = csr32r(c->regs, DmaChanBase + DmaChanRxControl);
		v = (v & ~DmaRbszMask) | ((Rbsz << DmaRbszShift) & DmaRbszMask);
		csr32w(c->regs, DmaChanBase + DmaChanRxControl, v | DmaControlSr);
		c->dmarunning = 1;

		dprint("dwqe: readback GmacConfig=%#08ux DmaChanRxControl=%#08ux DmaChanTxControl=%#08ux "
			"DmaChanRxBase=%#08ux DmaChanRxEnd=%#08ux DmaChanRxLen=%#08ux "
			"MtlRxOpMode=%#08ux GmacPacketFilter=%#08ux GmacRxqCtrl0=%#08ux\n",
			csr32r(c->regs, GmacConfig),
			csr32r(c->regs, DmaChanBase + DmaChanRxControl),
			csr32r(c->regs, DmaChanBase + DmaChanTxControl),
			csr32r(c->regs, DmaChanBase + DmaChanRxBase),
			csr32r(c->regs, DmaChanBase + DmaChanRxEnd),
			csr32r(c->regs, DmaChanBase + DmaChanRxLen),
			csr32r(c->regs, MtlChanBase + MtlRxOpMode),
			csr32r(c->regs, GmacPacketFilter),
			csr32r(c->regs, GmacRxqCtrl0));

		ethrx(edev);
		ethtx(edev);

		csr32w(c->regs, DmaChanBase + DmaChanStatus, ~0U);
		csr32w(c->regs, DmaChanBase + DmaChanIntrEna,
			DmaIntrNie | DmaIntrTie
			| DmaIntrRie | DmaIntrRbu
			| DmaIntrAie | DmaIntrFbe);

		print("dwqe: %s link up %dMbps %s\n", edev->name, speed, dup);
		ethersetspeed(edev, speed);
		ethersetlink(edev, 1);

		while((mdread(c, Bmsr) & BmsrLs) != 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);
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
	csr32w(c->regs, DmaChanBase + DmaChanRxEnd,
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
		csr32r(c->regs, DmaChanBase + DmaChanStatus),
		csr32r(c->regs, DmaChanBase + DmaChanIntrEna));
	p = seprint(p, e, "gmac config %#ux\n", csr32r(c->regs, GmacConfig));
	p = seprint(p, e, "filter %#ux promisc=%d allmulti=%d\n",
		csr32r(c->regs, GmacPacketFilter), c->promisc, c->allmulti);
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
			csr32w(c->regs, DmaChanBase + DmaChanTxEnd,
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

	en = csr32r(c->regs, DmaChanBase + DmaChanIntrEna);
	if(en != 0){
		csr32w(c->regs, DmaChanBase + DmaChanIntrEna, 0);
		csr32r(c->regs, DmaChanBase + DmaChanIntrEna);	/* drain posted write */
	}

	fl = csr32r(c->regs, DmaChanBase + DmaChanStatus);
	if(fl != 0){
		csr32w(c->regs, DmaChanBase + DmaChanStatus, fl);	/* W1C */
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
		csr32w(c->regs, DmaChanBase + DmaChanIntrEna, en);
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
	fl = csr32r(c->regs, DmaChanBase + DmaChanStatus);
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

/* Core bring-up ------------------------------------------------------------ */

static int
ethinit(Ether *edev)
{
	Ctlr *c;
	u32int v;
	int timeout;

	c = edev->ctlr;

	/*
	 * APMU: bring GMAC0 out of reset with clocks enabled, PHY interface
	 * mode selected, and RGMII delay lines set -- all BEFORE touching
	 * any MAC/DMA register. Skipping it stalls the hart.
	 */
	{
		u32int origv = csr32r(c->apmu, c->apmuctrloff);

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

				csr32w(c->apmu, c->apmuctrloff, origv | EmacBusEn);
				microdelay(1000);
				lo = csr32r(c->regs, GmacAddr0Lo);
				hi = csr32r(c->regs, GmacAddr0Hi);
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
		 * *_PHASE_SEL=0 (no skew correction needed, RMII one is don't-care)
		 * RGMII_TXC_SRC_SEL=0 (RX-clock-sourced TX clock)
		 */
		csr32w(c->apmu, c->apmuctrloff, preserve | physel);
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff, preserve | physel | EmacBusEn | Emac1588ClkMux);
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff, preserve | physel | EmacBusEn | Emac1588ClkMux
			| EmacBusRst | EmacAxiMstId);
		microdelay(10000);
		print("dwqe: APMU CLK_RST[%#ux] %#ux -> %#ux (PHY_SELECT=RGMII, TXC_SRC_SEL=RXclk)\n",
			c->apmuctrloff, origv, csr32r(c->apmu, c->apmuctrloff));
	}

	/* RGMII delay lines */
	v = EmacRgmiiRxcDlinePu | EmacRgmiiTxcDlinePu
	  | EmacRgmiiRxcDlineStep | EmacRgmiiTxcDlineStep
	  | c->rxdelay << EmacRgmiiRxcDlineAdjShift
	  | c->txdelay << EmacRgmiiTxcDlineAdjShift;
	csr32w(c->apmu, c->apmudlineoff, v);
	print("dwqe: APMU DLINE[%#ux]=0x%08ux\n", c->apmudlineoff, v);

	/* CR sanity-check, kept as a debug-only cross-check  */
	{
		u32int aclkctrl = csr32r(c->apmu, 0x388);
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
	csr32w(c->regs, GmacIntEn, 0);
	csr32w(c->regs, GmacConfig, 0);
	csr32w(c->regs, DmaChanBase + DmaChanTxControl, 0);
	csr32w(c->regs, DmaChanBase + DmaChanRxControl, 0);

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

	/* DMA software reset -- polled */
	csr32w(c->regs, DmaBusMode, csr32r(c->regs, DmaBusMode) | DmaBusModeSwr);
	for(timeout = 0; timeout < 1000; timeout++){
		if((csr32r(c->regs, DmaBusMode) & DmaBusModeSwr) == 0)
			break;
		microdelay(1000);
	}
	if(timeout == 1000){
		print("dwqe: ERROR DmaBusMode software reset timed out\n");
		return -1;
	}
	dprint("dwqe: DMA reset done after %d ms\n", timeout);

	/*
	 * K3 DDR starts at 0x1_00000000 -- everything iallocb()/ucallocalign()
	 * hands us will have PADDR() >= 4G, so 64-bit DMA addressing is required.
	 */
	csr32w(c->regs, DmaSysBusMode, DmaSysBusMb | DmaSysBusFb | DmaSysBusEame);

	/* MTL: store-and-forward, enable queue 0, size FIFOs from HW_FEATURE1 */
	{
		u32int feat1 = csr32r(c->regs, GmacHwFeature1);
		u32int txfifo = 128 << ((feat1 & GmacHwTxFifoSize) >> 6);
		u32int rxfifo = 128 << (feat1 & GmacHwRxFifoSize);
		u32int tqs = txfifo / 256 - 1;
		u32int rqs = rxfifo / 256 - 1;

		print("dwqe: HW_FEATURE1=%#08ux txfifo=%ud rxfifo=%ud bytes\n",
			feat1, txfifo, rxfifo);

		csr32w(c->regs, MtlChanBase + MtlTxOpMode,
			MtlOpTsf | MtlOpTxqEn
			| ((tqs << 16) & MtlOpTqsMask));
		csr32w(c->regs, MtlChanBase + MtlRxOpMode,
			MtlOpRsf | ((rqs << 20) & MtlOpRqsMask));
	}

	/* MAC address */
	csr32w(c->regs, GmacAddr0Hi,
		(1U << 31) | edev->ea[5]<<8 | edev->ea[4]);
	csr32w(c->regs, GmacAddr0Lo,
		edev->ea[3]<<24 | edev->ea[2]<<16 | edev->ea[1]<<8 | edev->ea[0]);
	dprint("dwqe: MAC %02X:%02X:%02X:%02X:%02X:%02X programmed\n",
		edev->ea[0], edev->ea[1], edev->ea[2], edev->ea[3], edev->ea[4], edev->ea[5]);

	/*
	 * Enable RX queue 0 at the MAC's queue-routing level.  This is
	 * upstream of MTL/DMA entirely -- without it the MAC can silently
	 * drop everything before it ever reaches the MTL FIFO.
	 */
	csr32w(c->regs, GmacRxqCtrl0, GmacRxqCtrl0Q0Dcb);

	/*
	 * Real filtering now: unicast perfect-match on GMAC_ADDR0 +
	 * broadcast + all multicast. Toggle via the /net/etherN/clone
	 * file ("promisc 1"/"allmulti 0") if you need something
	 * tighter or looser at runtime.
	 */
	dwqesetfilter(c, 0, 1);

	/* GmacConfig base bits; RE/TE turned on by ethproc once link is up */
	csr32w(c->regs, GmacConfig,
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

		csr32w(c->regs, DmaChanBase + DmaChanTxBaseHi, txpa >> 32);
		csr32w(c->regs, DmaChanBase + DmaChanTxBase, (u32int)txpa);
		csr32w(c->regs, DmaChanBase + DmaChanRxBaseHi, rxpa >> 32);
		csr32w(c->regs, DmaChanBase + DmaChanRxBase, (u32int)rxpa);
		csr32w(c->regs, DmaChanBase + DmaChanTxLen, Ntxd - 1);
		csr32w(c->regs, DmaChanBase + DmaChanRxLen, Nrxd - 1);
		csr32w(c->regs, DmaChanBase + DmaChanTxEnd, (u32int)txpa);
	}

	/*
	 * Fills the ring (leaving the one sentinel gap slot empty and sets
	 * DmaChanRxEnd itself, now that DmaChanRxBase is already programmed.
	 */
	replenish(c);

	dprint("dwqe: rings rxr=%p txr=%p (%d/%d slots)\n", c->rxr, c->txr, Nrxd, Ntxd);

	/* DMA channel stays stopped (ST/SR off); ethproc starts both on link-up */
	csr32w(c->regs, DmaChanBase + DmaChanStatus, ~0U);
	csr32w(c->regs, DmaChanBase + DmaChanIntrEna, 0);
	csr32w(c->regs, GmacIntEn, 0);

	return 0;
}

/* Plug-and-play -------------------------------------------------------- */

static int
etherpnp(Ether *edev)
{
	Ctlr *ct;
	Dtbether *de;
	uvlong base, size, apmubase;
	int irq, i, n;

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

	de = nil;
	n = dtbnether();
	/* Walk through the array, but skip ones that are not ok.
	 * So, e.g., if the first one in the array (0) is not marked
	 * OK, don't count it: in one case we have, the second one (1)
	 * becomes ether0. */
	for(i = 0; i < n; i++){
		Dtbether *d = dtbether(i);
		if(d != nil  && d->ok && strstr(d->compatible, "spacemit,k3-gmac") != nil) {
 			d->aliasidx = edev->ctlrno;
			de = d;
			break;
		}
	}
	if(de != nil){
		base     = de->base;
		size     = de->size ? de->size : Gmac0Size;
		apmubase = de->apmubase ? de->apmubase : ApmuPhys;
		irq      = de->irq;
		ct->apmuctrloff  = de->ctrlregoff  ? de->ctrlregoff  : 0x3e4;
		ct->apmudlineoff = de->dlineregoff ? de->dlineregoff : 0x3e8;
		print("dwqe: ether%d from DTB %s base=%#llux size=%#llux apmu=%#llux+{%#ux,%#ux} irq=%d\n",
			edev->ctlrno, de->path, base, size, apmubase,
			ct->apmuctrloff, ct->apmudlineoff, irq);
	} else if(edev->ctlrno == 0 && n == 0){
		base     = Gmac0Phys;
		size     = Gmac0Size;
		apmubase = ApmuPhys;
		irq      = Gmac0Irq;
		ct->apmuctrloff  = 0x3e4;
		ct->apmudlineoff = 0x3e8;
		print("dwqe: no DTB, using K3 ether0 defaults\n");
	} else {
		dprint("dwqe: no DTB entry for ether%d\n", edev->ctlrno);
		return -1;
	}

	ct->regs = (ulong*)vmap(base, size);
	ct->apmu = (ulong*)vmap(apmubase, ApmuSize);
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
