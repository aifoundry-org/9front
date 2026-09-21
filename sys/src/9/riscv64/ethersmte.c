/*
 * Driver for the SpacemiT K1 Ethernet controller (EMAC/smte)
 * Interrupt-driven via intrenable(); etherclock() remains
 * as a 100Hz backstop in case a wake-up is missed
 *
 * Based on:
 *   OpenBSD if_smte.c   - Patrick Wildt, Mark Kettenis
 *   9front ethercycv.c  - general driver structure
 *
 * Hardware:
 *   0x0000-0x002c  DMA registers
 *   0x0100-0x01e4  MAC/GMAC registers
 *   separate APMU  RGMII delay lines
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
 * periodic stats line; 2 = per-frame RX header dump. Set at runtime via
 *   echo debug 1 >/net/ether0/clone/ctl
 */
static int debug;

#define dprint		if(debug) print

/*
 * Default MMIO layout for the SpacemiT K1's first EMAC, used only as
 * a fallback when no matching DTB entry is found.  Normal operation
 * pulls base / IRQ / APMU base / MAC out of /soc/ethernet@* via
 * the Dtbether table built by bootargs.c.
 */
enum {
	Emac0Phys	= 0xcac80000UL,
	Emac0Size	= 0x420,
	Emac0Irq	= 131,

	ApmuPhys	= 0xd4282800UL,
	ApmuSize	= 0x400,

	/* RGMII delays: 1560 ps = 100 steps x 15.6 ps (from vendor DTS) */
	SmteRxdelaydef	= 100,
	SmteTxdelaydef	= 100,
};

/*
 * K1 pad controller (AIB) and GPIO controller — fixed SoC addresses used
 * for pin muxing (gmacpins) and PHY reset (phyreset).
 */
enum {
	AibPhys		= 0xd401e000UL,
	AibSize		= 0x1000,

	GpioPhys	= 0xd4019000UL,
	GpioSize	= 0x200,
	GpioBank3off	= 0x100,		/* bank 3 register group offset */
	GpioGsdr	= 0x54,			/* set direction = output */
	GpioGpcr	= 0x24,			/* set output low (clear) */
	GpioGpsr	= 0x18,			/* set output high */
};

/* DMA registers (byte offset) */
enum {
	DmaConfig		= 0x0000,
	DmaConfigSwreset	= (1U << 0),
	DmaConfigBurst16	= (0x10 << 1),
	DmaConfigStrictburst	= (1U << 17),
	DmaConfig64bit		= (1U << 18),

	DmaCtrl		= 0x0004,
	DmaCtrlTxstart	= (1U << 0),
	DmaCtrlRxstart	= (1U << 1),

	/*
	 * DmaStatusIrq — W1C.  This controller has no "interrupt summary"
	 * bits (no NIS/AIS-style aggregate); every enabled source acts on
	 * its own and must be W1C'd individually.  Bit 8 is a separate
	 * MAC-level interrupt that we never enable (MacIntrena = 0).
	 */
	DmaStatusIrq		= 0x0008,	/* W1C */
	DmaStatusIrqTxdone	= (1U << 0),
	DmaStatusIrqTxdu	= (1U << 1),
	DmaStatusIrqTxstop	= (1U << 2),
	DmaStatusIrqRxdone	= (1U << 4),
	DmaStatusIrqRxdu	= (1U << 5),
	DmaStatusIrqRxstop	= (1U << 6),
	DmaStatusIrqRxmiss	= (1U << 7),
	DmaStatusIrqMac		= (1U << 8),

	/*
	 * DmaIntrena — per-source enables at the same bit positions
	 * as DmaStatusIrq.  DmaIntrenaRxdu fires the moment the chip
	 * enters Suspended (no chip-owned descriptor at its pointer); the
	 * mask-while-busy pattern in ethirq drains the ring and DmaRxpolldemand
	 * kicks the chip back to Running before we restore DmaIntrena, so
	 * no storm in practice.  DmaIntrenaTxdu stays off — TX submit
	 * already pokes DmaTxpolldemand and IRQ on dequeue is enough.
	 */
	DmaIntrena		= 0x000c,
	DmaIntrenaTxdone	= (1U << 0),
	DmaIntrenaTxdu		= (1U << 1),	/* not safe to enable */
	DmaIntrenaTxstop	= (1U << 2),
	DmaIntrenaRxdone	= (1U << 4),
	DmaIntrenaRxdu		= (1U << 5),
	DmaIntrenaRxstop	= (1U << 6),
	DmaIntrenaRxmiss	= (1U << 7),

	DmaTxautopoll	= 0x0010,
	DmaTxpolldemand	= 0x0014,
	DmaRxpolldemand	= 0x0018,
	DmaTxbase	= 0x001c,
	DmaRxbase	= 0x0020,

	DmaRxmitigation		= 0x002c,
	DmaRxmitigationEna	= (1U << 31),
};

/* MAC registers (byte offset) */
enum {
	MacGlobalctrl		= 0x0100,
	MacGlobalctrlSpeedmask	= (0x3 << 0),
	MacGlobalctrlSpeed10	= (0x0 << 0),
	MacGlobalctrlSpeed100	= (0x1 << 0),
	MacGlobalctrlSpeed1000	= (0x2 << 0),
	MacGlobalctrlDuplex	= (1U << 2),

	MacTxctrl		= 0x0104,
	MacTxctrlEna		= (1U << 0),
	MacTxctrlAutoretry	= (1U << 3),

	MacRxctrl		= 0x0108,
	MacRxctrlEna		= (1U << 0),
	MacRxctrlStoreforward	= (1U << 3),

	MacMaxframe	= 0x010c,
	MacTxjabber	= 0x0110,
	MacRxjabber	= 0x0114,

	MacAddrctrl		= 0x0118,
	MacAddrctrlAddr1ena	= (1U << 0),

	MacAddr1hi	= 0x0120,
	MacAddr1me	= 0x0124,
	MacAddr1lo	= 0x0128,

	MacFcctrl		= 0x0160,	/* flow control */
	MacFcctrlDecodeena	= (1U << 0),	/* decode received pause frames */

	MacMdioctrl		= 0x01a0,
	MacMdioctrlStart	= (1U << 15),
	MacMdioctrlRw		= (1U << 10),
	MacMdioctrlRegshift	= 5,
	MacMdioctrlPhyshift	= 0,

	MacMdiodata	= 0x01a4,

	MacTxfifoalmostfull	= 0x01c0,
	MacTxthresh		= 0x01c4,
	MacRxthresh		= 0x01c8,

	MacStatusirq	= 0x01e0,	/* W1C: underrun, jabber */
	MacIntrena	= 0x01e4,

	/*
	 * IEEE-1588 / PTP timestamping block.  We don't use PTP, but the
	 * status and enable registers need to be quiesced at init: a
	 * prior boot stage can leave Ptp1588Irqena non-zero with
	 * Ptp1588Irqsts bits latched, and those will keep the chip's
	 * IRQ line asserted regardless of what we do to DmaIntrena.
	 */
	Ptp1588Irqsts	= 0x0360,	/* W1C */
	Ptp1588Irqena	= 0x0364,
};

/*
 * APMU registers (byte offset from ApmuPhys = 0xd4282800).
 * Per the K1 device tree we read from /soc/ethernet@*:
 *   EMAC0 ctrl-reg  = 0x3e4   (AXI ID, PHY-intf select, RMII clk select)
 *   EMAC0 dline-reg = 0x3e8   (RGMII delay line)
 */
enum {
	ApmuEmacClkrstctrl	= 0x03e4,
	/*
	 * EMAC0_CLK_RST_CTRL bit fields (from the K1 user manual,
	 * "High-Speed Interface System / EMAC0_CLK_RST_CTRL"):
	 *   bit 0  = BUS_EN — AXI bus clock enable (1 = on)
	 *   bit 1  = BUS_RST — *Reset Release*  (1 = released, 0 = held in reset)
	 *   bit 13 = AXI_MST_ID — use single AXI ID for transfers
	 * EMAC1 is at offset +0x8 with the same layout.
	 */
	ApmuEmacBusena		= (1U << 0),
	ApmuEmacBusrstrelease	= (1U << 1),
	ApmuEmacAximstid	= (1U << 13),
	ApmuEmacPtpena		= (1U << 15),

	ApmuEmacRgmiidline		= 0x03e8,
	ApmuEmacRgmiidlineRxena		= (1U << 0),
	ApmuEmacRgmiidlineRxstep15P6	= (0U << 4),
	ApmuEmacRgmiidlineRxshift	= 8,
	ApmuEmacRgmiidlineTxena		= (1U << 16),
	ApmuEmacRgmiidlineTxstep15P6	= (0U << 20),
	ApmuEmacRgmiidlineTxshift	= 24,
};

/* RX descriptors */
enum {
	RxDesc0Lenmask		= (0x3fff << 0),
	RxDesc0Runt		= (1U << 15),
	RxDesc0Crcerr		= (1U << 20),
	RxDesc0Maxlenerr	= (1U << 21),
	RxDesc0Jabbererr	= (1U << 22),
	RxDesc0Lenerr		= (1U << 23),
	RxDesc0Own		= (1U << 31),
	RxDesc1Sizemask		= (0xfffff << 0),
	RxDesc1Endring		= (1U << 26),
};

/* TX descriptors */
enum {
	TxDesc0Own		= (1U << 31),
	TxDesc1Sizemask		= (0xfff << 0),
	TxDesc1Endring		= (1U << 26),
	TxDesc1First		= (1U << 29),
	TxDesc1Last		= (1U << 30),
	TxDesc1Irqoncomplete	= (1U << 31),
};

/*
 * Standard MII registers/bits come from ethermii.h (Bmcr, Bmsr, Anlpar,
 * Mscr/Mssr — the IEEE 802.3 Master-Slave Control/Status registers used
 * for 1000BASE-T — etc).  Only board-specific constants stay local.
 */
enum {
	Phyaddr		= 1,
	Linkdelay	= 500,
};

enum {
	Rxring	= 512,
	Txring	= 512,
	Txirqevery = 64,	/* request a TX-completion IRQ every Nth frame */
	Maxsmte	= 2,
};


#define SmteCbsz 64	/* X60/C908 D-cache line size */

/*
 * K1 DMA is non-coherent (dma-noncoherent in the vendor DT).
 * CPU->HW: cbo.flush (writeback + invalidate) — push fresh data to DRAM.
 * HW->CPU: cbo.inval (invalidate only) — must NOT writeback or a stale
 * CPU-cached descriptor (e.g. OWN=host from an old replenish that the
 * line later re-prefetched) would clobber the chip's just-written
 * OWN=chip update in DRAM, stranding the frame in a slot ethrx never
 * sees.  Symptom: RX missed frame + ring stuck at prod=cons-1.
 */
static void
smtecacheflush(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(SmteCbsz - 1);
	last = ((uintptr)va + len + SmteCbsz - 1) & ~(uintptr)(SmteCbsz - 1);
	for(; a < last; a += SmteCbsz)
		cboflush((void*)a);
	coherence();
}

static void
smtecacheinval(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(SmteCbsz - 1);
	last = ((uintptr)va + len + SmteCbsz - 1) & ~(uintptr)(SmteCbsz - 1);
	for(; a < last; a += SmteCbsz)
		cboinval((void*)a);
	coherence();
}

typedef struct Ctlr Ctlr;
struct Ctlr {
	ulong	*regs;
	ulong	*apmu;
	u32int	apmuctrloff;	/* APMU offset for this EMAC's clk/reset */
	u32int	apmudlineoff;	/* APMU offset for this EMAC's RGMII dlines */
	u32int	*rxr, *txr;
	Block	**rxs, **txs;
	int	rxprodi, rxconsi, txi;
	u32int	rxdelay, txdelay;
	int	rstpin;		/* PHY reset GPIO; -1 if unknown */
	int	txcons;		/* oldest unreclaimed TX slot */
	int	attach;
	int	dmarunning;	/* DMA started at link-up; gates poll-demand writes */
	Lock	txlock;
	/* debug / bring-up stats */
	u32int	irqseen;
	u32int	rxframes;
	u32int	txdone;
	int	statstick;
};

/*
 * the gmac pad groups (fixed K1 pin assignments, pad function 1) are
 * normally muxed by the boot loader, but it only configures the port
 * it boots through; the other port's MDIO then reads zeros from pads
 * that never leave their reset function.  Mux any pin still not on
 * function 1: 21mA drive (code 2 on these 1.8V pads) and pull-up, the
 * configuration the working port runs with.  Pins 0-85 sit at
 * (pin+1)*4 from the pad controller base.
 */
static void
gmacpins(uvlong base)
{
	static uchar pins0[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,45};
	static uchar pins1[] = {29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,46};
	static ulong *aib;
	uchar *pins;
	u32int v;
	int i, n, fixed;

	if(base == Emac0Phys){
		pins = pins0;
		n = nelem(pins0);
	}else{
		pins = pins1;
		n = nelem(pins1);
	}
	if(aib == nil)
		aib = vmap(AibPhys, AibSize);
	if(aib == nil)
		return;
	fixed = 0;
	for(i = 0; i < n; i++){
		v = aib[pins[i] + 1];
		if((v & 7) == 1)
			continue;
		aib[pins[i] + 1] = (v & ~(7 | 7<<10 | 7<<13)) | 1 | 2<<10 | 3<<14;
		fixed++;
	}
	if(fixed)
		print("smte: muxed %d gmac pads for %#llux\n", fixed, base);
}

/*
 * each PHY hangs off a GPIO reset line (emac,reset-gpio: active low,
 * 10ms low and 100ms settle on this board).  u-boot only releases the
 * PHY of the port it boots through, so do it ourselves for every port.
 * GPIO banks 0-3 sit at +{0,4,8,0x100} from the controller base; within
 * a bank, direction-set is +0x54 and output set/clear +0x18/+0x24 (K1
 * user manual, GPIO registers).
 */
static void
phyreset(Ctlr *c)
{
	static ulong *gpio;
	uint bank, bit;

	if(c->rstpin < 0 || c->rstpin >= 128)
		return;
	if(gpio == nil)
		gpio = vmap(GpioPhys, GpioSize);
	if(gpio == nil)
		return;
	bank = c->rstpin >> 5 == 3? GpioBank3off: (c->rstpin >> 5)*4;
	bit = 1 << (c->rstpin & 31);
	gpio[(bank + GpioGsdr)/4] = bit;	/* direction: output */
	gpio[(bank + GpioGpcr)/4] = bit;	/* drive low: assert reset */
	delay(10);
	gpio[(bank + GpioGpsr)/4] = bit;	/* drive high: release */
	delay(100);
}

/* MMIO access ---------------------------------------------------------------- */

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

/* MDIO ----------------------------------------------------------------------- */

static void
mdwrite(Ctlr *c, int reg, u16int v)
{
	csr32w(c->regs, MacMdiodata, v);
	csr32w(c->regs, MacMdioctrl,
		MacMdioctrlStart
		| (reg & 0x1f) << MacMdioctrlRegshift
		| (Phyaddr & 0x1f) << MacMdioctrlPhyshift);
	while(csr32r(c->regs, MacMdioctrl) & MacMdioctrlStart)
		tsleep(&up->sleep, return0, nil, 1);
}

static u16int
mdread(Ctlr *c, int reg)
{
	csr32w(c->regs, MacMdiodata, 0);
	csr32w(c->regs, MacMdioctrl,
		MacMdioctrlStart | MacMdioctrlRw
		| (reg & 0x1f) << MacMdioctrlRegshift
		| (Phyaddr & 0x1f) << MacMdioctrlPhyshift);
	while(csr32r(c->regs, MacMdioctrl) & MacMdioctrlStart)
		tsleep(&up->sleep, return0, nil, 1);
	return csr32r(c->regs, MacMdiodata) & 0xffff;
}

/* PHY kproc ------------------------------------------------------------------ */

static void	ethrx(Ether *edev);
static void	ethtx(Ether *edev);

/*
 * a link loss aborts the MAC under a live TX engine, after which the
 * chip's ring position no longer matches txi: queued frames then only
 * leave in bursts whenever the two happen to coincide.  Stop TX DMA,
 * reclaim the ring, and point both sides back at the base.  The RX
 * ring survives link bounces (every slot stays chip-owned) and is
 * left alone.
 */
static void
txreset(Ctlr *c)
{
	u32int v;
	int i;

	v = csr32r(c->regs, DmaCtrl);
	csr32w(c->regs, DmaCtrl, v & ~DmaCtrlTxstart);
	ilock(&c->txlock);
	for(i = 0; i < Txring; i++){
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
	smtecacheflush(c->txr, 4 * Txring * sizeof(u32int));
	iunlock(&c->txlock);
	csr32w(c->regs, DmaTxbase, PADDR(c->txr));
}

static void
ethproc(void *ved)
{
	Ether *edev;
	Ctlr *c;
	u16int bmsr, lpa, gbsr, id1, id2;
	u32int ctrl;
	int speed;
	char *dup;

	edev = ved;
	c    = edev->ctlr;

	id1 = mdread(c, Phyidr1);
	id2 = mdread(c, Phyidr2);
	dprint("smte: PHY addr=%d oui=0x%06ux model=0x%02ux rev=0x%01ux\n",
		Phyaddr,
		((u32int)id1 << 6) | (id2 >> 10),
		(id2 >> 4) & 0x3f,
		id2 & 0xf);

	dprint("smte: PHY starting autonegotiation\n");
	/*
	 * Advertise 1000BASE-T in the gigabit control register before
	 * restarting autoneg.  Without this the PHY only offers its power-on
	 * default, which on this board does not include gigabit, so the link
	 * settles at 100BASE-TX and every transfer is capped near 100Mbps.
	 */
	mdwrite(c, Mscr, Mscr1000TFD | Mscr1000THD);
	mdwrite(c, Bmcr, BmcrAne | BmcrRan);

	for(;;){
		bmsr = mdread(c, Bmsr);
		if((bmsr & BmsrLs) == 0){
			print("smte: %s link down (BMSR=0x%04ux)\n", edev->name, bmsr);
			ethersetlink(edev, 0);
			csr32w(c->regs, MacTxctrl, 0);
			csr32w(c->regs, MacRxctrl, 0);
			txreset(c);
			dprint("smte: MAC TX/RX disabled, waiting for link\n");
			while((mdread(c, Bmsr) & BmsrLs) == 0)
				tsleep(&up->sleep, return0, nil, Linkdelay);
			dprint("smte: physical link detected\n");
		}

		dprint("smte: waiting for autoneg (BMSR=0x%04ux)\n", mdread(c, Bmsr));
		while((mdread(c, Bmsr) & BmsrAnc) == 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);

		lpa  = mdread(c, Anlpar);
		gbsr = mdread(c, Mssr);
		dprint("smte: autoneg complete ANLPAR=0x%04ux GBSR=0x%04ux\n", lpa, gbsr);

		ctrl = csr32r(c->regs, MacGlobalctrl)
			& ~(MacGlobalctrlSpeedmask | MacGlobalctrlDuplex);
		dup  = "HD";

		if(gbsr & Mssr1000TFD){
			ctrl |= MacGlobalctrlSpeed1000 | MacGlobalctrlDuplex;
			speed = 1000; dup = "FD";
		} else if(gbsr & Mssr1000THD){
			ctrl |= MacGlobalctrlSpeed1000;
			speed = 1000;
		} else if(lpa & AnaTXFD){
			ctrl |= MacGlobalctrlSpeed100 | MacGlobalctrlDuplex;
			speed = 100; dup = "FD";
		} else if(lpa & AnaTXHD){
			ctrl |= MacGlobalctrlSpeed100;
			speed = 100;
		} else if(lpa & Ana10FD){
			ctrl |= MacGlobalctrlSpeed10 | MacGlobalctrlDuplex;
			speed = 10; dup = "FD";
		} else {
			ctrl |= MacGlobalctrlSpeed10;
			speed = 10;
		}

		csr32w(c->regs, MacGlobalctrl, ctrl);
		csr32w(c->regs, MacTxctrl,
			MacTxctrlEna | MacTxctrlAutoretry);
		csr32w(c->regs, MacRxctrl,
			MacRxctrlEna | MacRxctrlStoreforward);

		/*
		 * Link is up.  Start TX/RX DMA now, after MAC TX/RX are armed
		 * so only frames the MAC has just accepted flow into our
		 * descriptors — no kexec-leftover FIFO residue.  Then drain
		 * once for safety, W1C status, and finally arm DmaIntrena
		 * for IRQ-driven completion.
		 */
		csr32w(c->regs, DmaCtrl,
			DmaCtrlTxstart | DmaCtrlRxstart);
		c->dmarunning = 1;	/* poll-demand writes are now safe */
		ethrx(edev);
		ethtx(edev);
		csr32w(c->regs, DmaStatusIrq, ~0U);
		/*
		 * DmaIntrenaRxdone deliberately omitted: enabling it after a
		 * Linux kexec breaks DHCP/RX (the chip seems to refuse further RX
		 * until bit 4 is acked, and W1C of bit 4 doesn't reliably clear on
		 * this silicon); RX completion is served by the 1ms etherclock poll.
		 *
		 * DmaIntrenaTxdone stays enabled, but ethtx only sets the per-
		 * descriptor IRQ-on-completion bit every Txirqevery frames.  A
		 * per-frame TX IRQ (the default) outpaces ethirq's drain once
		 * secondaries generate TX concurrently, pinning plic.c's bounded
		 * claim/complete loop until the source is masked as a storm (seen
		 * under tcpbw -c at *ncpu>1).  Coalescing keeps reclaim prompt
		 * enough to avoid TX stalls while cutting the IRQ rate ~Txirqevery-
		 * fold; the 1ms poll still mops up the tail when TX goes idle.
		 */
		csr32w(c->regs, DmaIntrena,
			DmaIntrenaTxdone
			| DmaIntrenaTxstop
			| DmaIntrenaRxdu
			| DmaIntrenaRxstop
			| DmaIntrenaRxmiss);
		/* readback drains the posted write */
		ctrl = csr32r(c->regs, DmaIntrena);
		dprint("smte: %s irq-driven DmaIntrena=%#ux DmaStatusIrq=%#ux\n",
			edev->name, ctrl, csr32r(c->regs, DmaStatusIrq));

		print("smte: %s link up %dMbps %s\n", edev->name, speed, dup);
		dprint("smte:   MacGlobalctrl=0x%08ux\n", ctrl);
		dprint("smte:   MacTxctrl=0x%08ux MacRxctrl=0x%08ux\n",
			csr32r(c->regs, MacTxctrl),
			csr32r(c->regs, MacRxctrl));

		ethersetspeed(edev, speed);
		ethersetlink(edev, 1);

		while((mdread(c, Bmsr) & BmsrLs) != 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);
	}
}

/* RX ring -------------------------------------------------------------------- */

static int
replenish(Ctlr *c)
{
	Block *bp;
	u32int *r;
	uintptr pa;
	int i;

	/* Refill every slot ethrx has freed (rxs[i] == nil).  Bookkeeping
	 * is rxs[]-based rather than the classic prod/cons gap because the
	 * gap creates a structural lockstep once the chip catches up to it:
	 * the chip suspends on the OWN=host gap, the loop refuses to refill
	 * it, and the poll-demand write just bounces off.  With all Rxring slots
	 * usable, replenish can always close the loop. */
	while(c->rxs[c->rxprodi] == nil){
		i  = c->rxprodi;
		bp = iallocb(Rbsz);
		if(bp == nil){
			/* iprint: replenish is reachable from ethirq (interrupt ctx) */
			iprint("smte: replenish: out of memory (prod=%d cons=%d)\n",
				c->rxprodi, c->rxconsi);
			return -1;
		}
		c->rxs[i] = bp;
		r  = &c->rxr[4 * i];
		pa = PADDR(bp->rp);		/* physical address for DMA */
		/* don't pre-clear r[0]: leaving it OWN=0 in the dirty cache
		 * line races chip-side reads — see ethrx skip handling */
		r[1] = Rbsz & RxDesc1Sizemask;
		if(i == Rxring - 1)
			r[1] |= RxDesc1Endring;
		r[2] = (u32int)pa;
		r[3] = 0;
		coherence();
		r[0] = RxDesc0Own;
		/* push descriptor, drop any stale buffer cache so HW writes win */
		smtecacheflush(r, 16);
		smtecacheinval(bp->rp, Rbsz);
		c->rxprodi = (c->rxprodi + 1) & (Rxring - 1);
	}
	/*
	 * DmaRxpolldemand is only meaningful while DMA RX is in Running or
	 * Suspended state.  Writing it while DMA is Stopped (the initial
	 * fill from ethinit before link-up) latches DmaStatusIrqRxdone
	 * (bit 4) — and that bit then refuses to W1C,
	 * driving the chip's IRQ line high and storming the PLIC once we
	 * enable the source.
	 */
	if(c->dmarunning)
		csr32w(c->regs, DmaRxpolldemand, 0xFF);
	return 0;
}

/* ethrx --------------------------------------------------------------------- */

static void
ethrx(Ether *edev)
{
	Ctlr *c;
	u32int *r;
	Block *bp;
	u32int s;
	int len;

	c = edev->ctlr;
	for(;;){
		r = &c->rxr[4 * c->rxconsi];
		smtecacheinval(r, 16);		/* discard stale cache before reading HW status */
		if(r[0] & RxDesc0Own){
			/*
			 * The K1 RX engine occasionally lands frames in later
			 * descriptors while leaving an earlier slot's OWN bit
			 * set.  Scan forward; if any nearby slot is OWN-clear,
			 * the chip skipped us — drop the stuck buffers so they
			 * can be refilled on the next ring pass.
			 */
			int k, found;
			u32int *rn;
			found = -1;
			for(k = 1; k < 64; k++){
				rn = &c->rxr[4 * ((c->rxconsi + k) & (Rxring - 1))];
				smtecacheinval(rn, 16);
				if((rn[0] & RxDesc0Own) == 0){
					found = k;
					break;
				}
			}
			if(found < 0)
				break;
			dprint("smte: RX skip %d stuck slots starting=%d\n",
				found, c->rxconsi);
			for(k = 0; k < found; k++){
				bp = c->rxs[c->rxconsi];
				c->rxs[c->rxconsi] = nil;
				if(bp != nil)
					freeb(bp);
				c->rxconsi = (c->rxconsi + 1) & (Rxring - 1);
			}
			replenish(c);
			continue;
		}
		s   = r[0];
		len = s & RxDesc0Lenmask;
		bp  = c->rxs[c->rxconsi];
		c->rxs[c->rxconsi] = nil;
		if(bp != nil && len > 0)
			smtecacheinval(bp->rp, len);

		if(len < 4 || s & (RxDesc0Runt   | RxDesc0Crcerr |
		                    RxDesc0Maxlenerr | RxDesc0Jabbererr |
		                    RxDesc0Lenerr)){
			dprint("smte: RX error slot=%d desc0=0x%08ux len=%d\n",
				c->rxconsi, s, len);
			freeb(bp);
		} else {
			bp->wp = bp->rp + len - 4;
			if(debug >= 2){
				uchar *p = bp->rp;
				int type = p[12]<<8 | p[13];
				int isbcast = (p[0]&p[1]&p[2]&p[3]&p[4]&p[5]) == 0xff;
				int isarp = (type == 0x0806);
				int isdhcp = 0;
				if(type == 0x0800 && (len-4) >= 42){
					int sport = p[34]<<8 | p[35];
					int dport = p[36]<<8 | p[37];
					if(p[23] == 17 && (sport == 67 || dport == 67 ||
					                    sport == 68 || dport == 68))
						isdhcp = 1;
				}
				if(c->rxframes < 4 || isbcast || isarp || isdhcp){
					print("smte: rx[%ud] len=%d "
						"dst=%02ux:%02ux:%02ux:%02ux:%02ux:%02ux "
						"src=%02ux:%02ux:%02ux:%02ux:%02ux:%02ux "
						"type=%04ux%s%s%s\n",
						c->rxframes, len-4,
						p[0],p[1],p[2],p[3],p[4],p[5],
						p[6],p[7],p[8],p[9],p[10],p[11],
						type,
						isbcast? " [BCAST]" : "",
						isarp? " [ARP]" : "",
						isdhcp? " [DHCP]" : "");
				}
			}
			etheriq(edev, bp);
			c->rxframes++;
		}
		c->rxconsi = (c->rxconsi + 1) & (Rxring - 1);
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
	for(i = 0; i < Txring; i++)
		if(c->txs[i] != nil)
			ntx++;
	nrx = 0;
	for(i = 0; i < Rxring; i++)
		if(c->rxs[i] != nil)
			nrx++;
	p = seprint(p, e, "tx: prod %d cons %d inflight %d done %ud\n",
		c->txi, c->txcons, ntx, c->txdone);
	p = seprint(p, e, "rx: prod %d cons %d bufs %d frames %ud\n",
		c->rxprodi, c->rxconsi, nrx, c->rxframes);
	p = seprint(p, e, "irq bits seen %#ux dma ctl %#ux status %#ux intren %#ux\n",
		c->irqseen, csr32r(c->regs, DmaCtrl),
		csr32r(c->regs, DmaStatusIrq),
		csr32r(c->regs, DmaIntrena));
	p = seprint(p, e, "mac global %#ux tx %#ux rx %#ux\n",
		csr32r(c->regs, MacGlobalctrl),
		csr32r(c->regs, MacTxctrl),
		csr32r(c->regs, MacRxctrl));
	p = seprint(p, e, "rgmii delay rx %ud tx %ud steps, phy reset pin %d\n",
		c->rxdelay, c->txdelay, c->rstpin);
	return p;
}

/* ethtx --------------------------------------------------------------------- */

static void
ethtx(Ether *edev)
{
	Ctlr *c;
	u32int *r;
	Block *bp;
	uintptr pa;

	c = edev->ctlr;
	ilock(&c->txlock);
	/*
	 * reclaim transmitted frames eagerly, oldest first, so buffers
	 * don't sit in the ring until the write position laps them.
	 */
	while(c->txs[c->txcons] != nil){
		r = &c->txr[4 * c->txcons];
		smtecacheinval(r, 16);
		if(r[0] & TxDesc0Own)
			break;
		freeb(c->txs[c->txcons]);
		c->txs[c->txcons] = nil;
		c->txdone++;
		c->txcons = (c->txcons + 1) & (Txring - 1);
	}
	for(;;){
		r = &c->txr[4 * c->txi];
		smtecacheinval(r, 16);		/* discard stale cache before reading HW status */
		if(r[0] & TxDesc0Own){
			dprint("smte: TX ring full (txi=%d)\n", c->txi);
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
		pa  = PADDR(bp->rp);		/* physical address for DMA */
		/* push frame payload to DRAM before HW reads it */
		smtecacheflush(bp->rp, BLEN(bp));
		r[1] = (BLEN(bp) & TxDesc1Sizemask)
			| TxDesc1First | TxDesc1Last;
		/* coalesce: only every Nth frame raises a completion IRQ */
		if((c->txi & (Txirqevery - 1)) == 0)
			r[1] |= TxDesc1Irqoncomplete;
		if(c->txi == Txring - 1)
			r[1] |= TxDesc1Endring;
		r[2] = (u32int)pa;
		r[3] = 0;
		coherence();
		r[0] = TxDesc0Own;
		smtecacheflush(r, 16);		/* push descriptor to DRAM for HW */
		if(c->dmarunning)
			csr32w(c->regs, DmaTxpolldemand, 0xFF);
		c->txi = (c->txi + 1) & (Txring - 1);
	}
	iunlock(&c->txlock);
}

/* IRQ / polling -------------------------------------------------------------- */

/*
 * Mask-then-clear interrupt handling.  The chip's IRQ output is the
 * level of (DmaStatusIrq & DmaIntrena) — any enabled source
 * that re-asserts between our W1C and our return drives the PLIC's
 * level-triggered gateway to re-forward on every completion message
 * (PLIC v1.0 spec, "Interrupt Gateways"), and plic.c's claim/complete
 * loop will keep dispatching us forever.  We zero DmaIntrena
 * entirely for the duration of one handler entry, drain both rings,
 * and restore.  Doing the full mask (not just the two completion
 * bits) is paranoia that costs a single MMIO write per IRQ and means
 * a stale DMA-stopped or missed-frame status bit also can't pin the line high.
 */
static void
ethirq(Ureg *, void *arg)
{
	Ether *edev;
	Ctlr *c;
	u32int fl, en, macsts, ptpsts;

	edev = arg;
	c    = edev->ctlr;

	en = csr32r(c->regs, DmaIntrena);
	if(en != 0){
		csr32w(c->regs, DmaIntrena, 0);
		/*
		 * Read back to force the posted write to commit before we
		 * sample status: otherwise the chip's IRQ line can still be
		 * asserted from a stale enabled-source view when plic.c
		 * re-claims, and the 64-trip cap mistakes us for a storm.
		 */
		en = csr32r(c->regs, DmaIntrena);
		USED(en);
	}

	fl = csr32r(c->regs, DmaStatusIrq);
	macsts = csr32r(c->regs, MacStatusirq);
	ptpsts = csr32r(c->regs, Ptp1588Irqsts);
	/* Belt-and-suspenders: W1C the other two too, in case one of them
	 * is feeding the chip's IRQ line. */
	if(macsts != 0)
		csr32w(c->regs, MacStatusirq, macsts);
	if(ptpsts != 0)
		csr32w(c->regs, Ptp1588Irqsts, ptpsts);
	if(fl != 0){
		/* W1C every bit we observed.  No NIS/AIS summary on this part. */
		csr32w(c->regs, DmaStatusIrq, fl);

		/*
		 * Diagnostics use iprint() because ethirq runs at interrupt
		 * level once intrenable() wires us up; print() can block on
		 * the output queue, which is not legal here.
		 */
		if(fl & DmaStatusIrqRxdu)
			/* chip just entered Suspended; silent kick (fires often) */
			csr32w(c->regs, DmaRxpolldemand, 0xFF);
		if(fl & DmaStatusIrqRxmiss){
			iprint("smte: %s RX missed frame (status=%#08ux "
				"prod=%d cons=%d)\n",
				edev->name, fl, c->rxprodi, c->rxconsi);
			csr32w(c->regs, DmaRxpolldemand, 0xFF);
		}
		if(fl & DmaStatusIrqRxstop){
			iprint("smte: %s RX DMA stopped, restarting\n", edev->name);
			csr32w(c->regs, DmaRxpolldemand, 0xFF);
		}
		if(fl & DmaStatusIrqTxstop){
			iprint("smte: %s TX DMA stopped, restarting\n", edev->name);
			csr32w(c->regs, DmaTxpolldemand, 0xFF);
		}
	}

	/*
	 * Drain unconditionally — ethrx/ethtx are self-bounded by the
	 * per-descriptor OWN flag, so calling them when nothing is
	 * ready is a cheap one-descriptor read.  Also covers the polling
	 * backstop path (etherclock -> etherserve -> ethirq).
	 */
	ethrx(edev);
	ethtx(edev);

	if(en != 0){
		coherence();
		csr32w(c->regs, DmaIntrena, en);
	}
}

static Ether *smteether[Maxsmte];
static Ctlr smtectlr[Maxsmte];
static int smteclockregistered;

static void
etherserve(Ether *edev)
{
	Ctlr *c;
	u32int fl;
	int lasttxi;

	c = edev->ctlr;

	/* accumulate raw DMA status bits before ethirq() W1Cs them */
	fl = csr32r(c->regs, DmaStatusIrq);
	c->irqseen |= fl;

	ethirq(nil, edev);

	/* e.g. echo debug 1 > /net/ether0/0/ctl to get periodic stats */
	if(debug && ++c->statstick >= 5000){
		c->statstick = 0;
		lasttxi = (c->txi - 1) & (Txring - 1);
		print("smte: %s stats irq_seen=0x%08ux rxcons=%d rxprod=%d "
			"rx_frames=%ud txi=%d tx_done=%ud "
			"rxd0=0x%08ux txd0[%d]=0x%08ux "
			"RXCTL=0x%08ux TXCTL=0x%08ux DMACTL=0x%08ux\n",
			edev->name, c->irqseen, c->rxconsi, c->rxprodi, c->rxframes,
			c->txi, c->txdone,
			c->rxr[4*c->rxconsi], lasttxi, c->txr[4*lasttxi],
			csr32r(c->regs, MacRxctrl),
			csr32r(c->regs, MacTxctrl),
			csr32r(c->regs, DmaCtrl));
		c->irqseen = 0;
	}
}

static void
etherclock(void)
{
	int i;

	for(i = 0; i < Maxsmte; i++)
		if(smteether[i] != nil)
			etherserve(smteether[i]);
}

/* Attach --------------------------------------------------------------------- */

static void
ethattach(Ether *edev)
{
	Ctlr *c;

	c = edev->ctlr;
	if(c->attach)
		return;
	c->attach = 1;
	dprint("smte: attach ok, starting PHY kproc\n");
	kproc("smteproc", ethproc, edev);
}

/* ethinit -------------------------------------------------------------------- */

static int
ethinit(Ether *edev)
{
	Ctlr *c;
	u32int v;

	c = edev->ctlr;

	/*
	 * APMU: bring the EMAC out of reset with clocks enabled before
	 * we touch any MAC register.  Under kexec-from-Linux the MAC is
	 * already ungated when we take over, so the earlier ordering
	 * (MAC accesses first, then APMU) worked incidentally.  Under a
	 * cold u-boot boot u-boot never touches EMAC0 and the bus clock
	 * is still gated + reset asserted; any csr32 to c->regs then
	 * stalls the hart.  Do APMU first so the MAC block responds.
	 *
	 * Gate the AXI clock AND assert bus-reset (ApmuEmacBusrstrelease=0 =
	 * held in reset), then after a generous delay re-enable the
	 * clock and release reset, preserving unrelated clock-config
	 * bits firmware may have set (1588/PTP, RGMII TXC source, etc).
	 * Bit 15 gates the PTP clock; enable it too so the 1588 register
	 * block is reachable if we ever query it.
	 */
	{
		u32int origv = csr32r(c->apmu, c->apmuctrloff);
		u32int preserve = origv & ~(ApmuEmacBusena
			| ApmuEmacBusrstrelease | ApmuEmacAximstid
			| ApmuEmacPtpena);
		csr32w(c->apmu, c->apmuctrloff, preserve);	/* clock off + reset asserted */
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff,
			preserve | ApmuEmacBusena | ApmuEmacPtpena);	/* clocks on, still in reset */
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff,
			preserve | ApmuEmacBusena | ApmuEmacPtpena
			| ApmuEmacBusrstrelease | ApmuEmacAximstid);
		microdelay(10000);
		print("smte: APMU ApmuEmacClkrstctrl[%#ux] %#ux -> %#ux\n",
			c->apmuctrloff, origv, csr32r(c->apmu, c->apmuctrloff));
	}

	/* Now the MAC block is reachable; stop everything. */
	csr32w(c->regs, MacIntrena,   0);
	csr32w(c->regs, DmaIntrena,   0);
	csr32w(c->regs, MacTxctrl, 0);
	csr32w(c->regs, MacRxctrl,  0);
	csr32w(c->regs, DmaCtrl,          0);
	print("smte: MAC/DMA stopped\n");

	phyreset(c);

	/* APMU: RGMII delay lines */
	v = ApmuEmacRgmiidlineRxena   | ApmuEmacRgmiidlineTxena
	  | ApmuEmacRgmiidlineRxstep15P6 | ApmuEmacRgmiidlineTxstep15P6
	  | c->rxdelay << ApmuEmacRgmiidlineRxshift
	  | c->txdelay << ApmuEmacRgmiidlineTxshift;
	csr32w(c->apmu, c->apmudlineoff, v);
	print("smte: APMU DLINE[%#ux]=0x%08ux (rx=%d tx=%d steps x 15.6ps)\n",
		c->apmudlineoff, v, c->rxdelay, c->txdelay);

	/* MAC address */
	csr32w(c->regs, MacAddr1hi, edev->ea[1]<<8 | edev->ea[0]);
	csr32w(c->regs, MacAddr1me, edev->ea[3]<<8 | edev->ea[2]);
	csr32w(c->regs, MacAddr1lo, edev->ea[5]<<8 | edev->ea[4]);
	csr32w(c->regs, MacAddrctrl, MacAddrctrlAddr1ena);
	dprint("smte: MAC %02X:%02X:%02X:%02X:%02X:%02X programmed\n",
		edev->ea[0], edev->ea[1], edev->ea[2],
		edev->ea[3], edev->ea[4], edev->ea[5]);

	/* FIFO thresholds and frame size limits */
	csr32w(c->regs, MacTxfifoalmostfull,        0x1f8);
	csr32w(c->regs, MacTxthresh,  192);
	csr32w(c->regs, MacRxthresh,   12);
	csr32w(c->regs, MacMaxframe,               1536);
	csr32w(c->regs, MacTxjabber,             1514+18);
	csr32w(c->regs, MacRxjabber,              1514+18);
	dprint("smte: FIFO thresholds and frame sizes programmed\n");

	/* reset DMA engine */
	csr32w(c->regs, DmaConfig, DmaConfigSwreset);
	microdelay(10000);
	csr32w(c->regs, DmaConfig, 0);
	microdelay(10000);
	/*
	 * DmaConfig64bit: enables 64-bit buffer addresses in descriptors
	 * (d2=bits[31:0] + d3=bits[63:32] per buffer). Required because
	 * iallocb() returns blocks above 4 GB on this system.
	 * The ring BASE address is still 32-bit (DmaRxbase),
	 * so descriptor rings must stay below 4 GB.
	 */
	v = DmaConfig64bit | DmaConfigStrictburst | DmaConfigBurst16;
	csr32w(c->regs, DmaConfig, v);
	dprint("smte: DMA reset, DmaConfig=%#ux\n", csr32r(c->regs, DmaConfig));

	/* Enable flow-control decode; harmless even without pause frames. */
	csr32w(c->regs, MacFcctrl, MacFcctrlDecodeena);

	/*
	 * Post-reset DMA tunables.  These registers are wiped by
	 * DmaConfigSwreset above, so they MUST be programmed
	 * after the reset completes.
	 *
	 * RX interrupt coalescing: disabled.  We poll from etherclock()
	 * at HZ rate; with coalescing on, DmaStatusIrqRxdone is held off
	 * until either 64 frames pile up or the timer fires.
	 *
	 * TX auto-poll: when the engine finds OWN=0 (no work pending)
	 * it idles until either this counter expires or someone writes
	 * DmaTxpolldemand.  The reset value on the K1 EMAC produces
	 * a ~40 ms idle->recheck period, which matched the ICMP echo
	 * latency pattern we observed.  Set to the minimum so the
	 * engine effectively spins on the descriptor.
	 */
	/*
	 * Do NOT clear DmaRxmitigation here.  The reset default
	 * (0x00FFFF01: frame_count=1, timeout=0xFFFF, mitigation disabled)
	 * is what we want.  Writing 0 sets frame_count to 0 which the spec
	 * says is illegal ("Legal values: 1 - 255") — and the K1 silicon
	 * responds by *permanently* latching DmaStatusIrq bit 4
	 * (DmaStatusIrqRxdone, bit 4).  Once latched it will not W1C even
	 * with DMA running, which stalls IRQ-driven RX entirely.
	 *
	 * DmaTxautopoll default is 0 already, so leave
	 * it alone too.
	 */
	dprint("smte: leaving MITIGATION=%#ux AUTOPOLL=%#ux at reset defaults\n",
		csr32r(c->regs, DmaRxmitigation),
		csr32r(c->regs, DmaTxautopoll));

	/* K1 EMAC uses 32-bit DMA addresses */
	c->rxr = xspanalloc(sizeof(u32int) * 4 * Rxring, 16, 0);
	/*
	 * TX descriptors are 16 bytes, so four share a 64-byte cache line.
	 * ethtx's smtecacheflush() of a freshly-queued descriptor writes the
	 * whole line back, and that carries our stale copies of the other
	 * three descriptors -- clobbering OWN=host completions the chip just
	 * wrote to them.  The completions are then lost (the ring jams full,
	 * inflight stuck near Txring) and the chip sees OWN=cpu, reports
	 * DmaStatusIrqTxdu and suspends, so bulk TX trickles.  Put the ring
	 * in non-cacheable memory (Svpbmt NC) so there is no line to clobber;
	 * the cbo calls on it become no-ops.
	 */
	c->txr = ucallocalign(sizeof(u32int) * 4 * Txring, 16, 0);
	memset(c->rxr, 0, 4 * Rxring * sizeof(u32int));
	memset(c->txr, 0, 4 * Txring * sizeof(u32int));
	smtecacheflush(c->rxr, 4 * Rxring * sizeof(u32int));
	smtecacheflush(c->txr, 4 * Txring * sizeof(u32int));

	c->rxs = mallocz(sizeof(Block*) * Rxring, 1);
	c->txs = mallocz(sizeof(Block*) * Txring, 1);
	if(c->rxs == nil || c->txs == nil){
		print("smte: ERROR mallocz failed\n");
		return -1;
	}
	dprint("smte: rings rxr=%p txr=%p (%d/%d slots, %d bytes/buf)\n",
		c->rxr, c->txr, Rxring, Txring, Rbsz);

	/* fill all Rxring slots (rxs[] starts all nil from mallocz) */
	c->rxprodi = 0;
	c->rxconsi = 0;
	replenish(c);

	c->txi = 0;

	uintptr rxpa, txpa;
	rxpa = PADDR(c->rxr);
	txpa = PADDR(c->txr);
	csr32w(c->regs, DmaRxbase, (u32int)rxpa);
	csr32w(c->regs, DmaTxbase, (u32int)txpa);
	dprint("smte: DMA base RX=0x%08ux TX=0x%08ux (readback RX=0x%08ux TX=0x%08ux)\n",
		(u32int)rxpa, (u32int)txpa,
		csr32r(c->regs, DmaRxbase),
		csr32r(c->regs, DmaTxbase));

	/*
	 * Clear stale interrupt state from whoever ran before us (typically
	 * a Linux kexec source kernel that had MAC RX enabled with PTP and
	 * traffic in flight).  The chip's IRQ output is the OR of
	 * DmaStatusIrq, MacStatusirq, and Ptp1588Irqsts — clear all.
	 *
	 * DmaIntrena stays at 0 across this window: even with MAC RX
	 * disabled, the previous stage's frames are still buffered in the
	 * RX FIFO, and starting DMA RX (below) immediately writes them into
	 * our descriptors and asserts DmaStatusIrqRxdone much faster than we
	 * can drain.  We let the 100Hz etherclock polling path consume that
	 * residue; ethproc flips DmaIntrena on once the link is up.
	 */
	csr32w(c->regs, Ptp1588Irqena, 0);
	csr32w(c->regs, Ptp1588Irqsts, ~0U);
	csr32w(c->regs, DmaStatusIrq, ~0U);
	csr32w(c->regs, MacStatusirq, ~0U);
	csr32w(c->regs, MacIntrena, 0);
	csr32w(c->regs, DmaIntrena, 0);
	dprint("smte: post-init DmaStatusIrq=%#ux Ptp1588Irqsts=%#ux (polled until link up)\n",
		csr32r(c->regs, DmaStatusIrq),
		csr32r(c->regs, Ptp1588Irqsts));

	/*
	 * DMA stays stopped until ethproc sees the link come up and turns
	 * both MAC TX/RX and the corresponding DMA channels on together.
	 * Starting RX DMA here, with MAC RX off but kexec residue in the
	 * RX FIFO, made DMA pull those frames into our descriptors and
	 * latch DmaStatusIrqRxdone faster than we could W1C it.
	 */
	csr32w(c->regs, DmaCtrl, 0);
	dprint("smte: DMA held stopped; ethproc starts on link up\n");

	return 0;
}

/* /net/etherN/clone/ctl handler ---------------------------------------------- */

enum {
	CMdebug,
};

static Cmdtab smtectlmsg[] = {
	CMdebug,	"debug",	2,
};

static long
smtectl(Ether *edev, void *buf, long n)
{
	Cmdbuf *cb;
	Cmdtab *ct;

	USED(edev);
	cb = parsecmd(buf, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	ct = lookupcmd(cb, smtectlmsg, nelem(smtectlmsg));
	switch(ct->index){
	case CMdebug:
		debug = atoi(cb->f[1]);
		break;
	}
	free(cb);
	poperror();
	return n;
}

/* Plug-and-play -------------------------------------------------------------- */

static int
etherpnp(Ether *edev)
{
	Ctlr *ct;
	Dtbether *de;
	uvlong base, size, apmubase;
	int irq, i, n;

	dprint("smte: etherpnp ctlrno=%d\n", edev->ctlrno);

	if(edev->ctlrno < 0 || edev->ctlrno >= Maxsmte){
		dprint("smte: ctlrno %d out of range\n", edev->ctlrno);
		return -1;
	}
	ct = &smtectlr[edev->ctlrno];
	if(ct->regs != nil){
		dprint("smte: ether%d already registered\n", edev->ctlrno);
		return -1;
	}

	/*
	 * This driver only knows the K1's EMAC, pad mux, GPIO and APMU addresses.
	 * Trust the DTB root compatible rather than sniffing MMIO, to avoid faulting
	 * or hanging the bus.
	 */
	if(!dtbsocmatch("spacemit,k1")){
		dprint("smte: not a K1, skipping\n");
		return -1;
	}

	/*
	 * Locate this controller in the DTB by alias index.  ether0 falls back
	 * to K1 EMAC0 defaults if the DTB is missing; the secondary doesn't.
	 */
	de = nil;
	n = dtbnether();
	for(i = 0; i < n; i++){
		Dtbether *d = dtbether(i);
		if(d != nil && d->ok && d->aliasidx == edev->ctlrno
		    && strstr(d->compatible, "emac") != nil){
			de = d;
			break;
		}
	}
	if(de != nil){
		base     = de->base;
		size     = de->size ? de->size : Emac0Size;
		apmubase = de->apmubase ? de->apmubase : ApmuPhys;
		irq      = de->irq;
		ct->apmuctrloff  = de->ctrlregoff  ? de->ctrlregoff  : ApmuEmacClkrstctrl;
		ct->apmudlineoff = de->dlineregoff ? de->dlineregoff : ApmuEmacRgmiidline;
		print("smte: ether%d from DTB %s base=%#llux size=%#llux apmu=%#llux+{%#ux,%#ux} irq=%d\n",
			edev->ctlrno, de->path, base, size, apmubase,
			ct->apmuctrloff, ct->apmudlineoff, irq);
	} else if(edev->ctlrno == 0 && n == 0){
		base     = Emac0Phys;
		size     = Emac0Size;
		apmubase = ApmuPhys;
		irq      = Emac0Irq;
		ct->apmuctrloff  = ApmuEmacClkrstctrl;
		ct->apmudlineoff = ApmuEmacRgmiidline;
		print("smte: no DTB, using K1 ether0 defaults\n");
	} else {
		dprint("smte: no DTB entry for ether%d\n", edev->ctlrno);
		return -1;
	}

	/* plic.c's intrinit has already masked every PLIC source. */
	ct->regs = (ulong*)vmap(base, size);
	ct->apmu = (ulong*)vmap(apmubase, ApmuSize);
	if(ct->regs == nil || ct->apmu == nil){
		print("smte: ERROR vmap failed regs=%p apmu=%p\n", ct->regs, ct->apmu);
		ct->regs = nil;
		return -1;
	}
	gmacpins(base);
	dprint("smte: ether%d regs=%p (phys=%#llux) apmu=%p (phys=%#llux)\n",
		edev->ctlrno, ct->regs, base, ct->apmu, apmubase);

	/*
	 * RGMII delays are per-port board tuning.  The DTB gives them as
	 * degrees of the 125MHz clock (tx-phase/rx-phase); the delay line
	 * counts 15.6ps steps, so steps = deg * 8000ps/360 / 15.6ps.
	 * Without DTB values, keep the old working defaults (~70 deg).
	 */
	ct->rxdelay = SmteRxdelaydef;
	ct->txdelay = SmteTxdelaydef;
	ct->rstpin = -1;
	if(de != nil){
		if(de->txphase > 0)
			ct->txdelay = (de->txphase * 8000 / 360 * 10 + 78) / 156;
		if(de->rxphase > 0)
			ct->rxdelay = (de->rxphase * 8000 / 360 * 10 + 78) / 156;
		ct->rstpin = de->rstpin;
	}

	edev->ctlr     = ct;
	edev->port     = base;
	edev->irq      = irq;
	edev->attach   = ethattach;
	edev->transmit = ethtx;
	edev->ifstat   = ethifstat;
	edev->arg      = edev;
	edev->mbps     = 1000;

	/*
	 * MAC address resolution, in priority order:
	 *   1. bootarg ea= (already in edev->ea via devether.c)
	 *   2. DT local-mac-address / mac-address
	 *   3. hardware register
	 */
	{
		static uchar zero[Eaddrlen];
		u32int hi, me, lo;
		if(memcmp(edev->ea, zero, Eaddrlen) != 0){
			dprint("smte: MAC from bootarg: %02X:%02X:%02X:%02X:%02X:%02X\n",
				edev->ea[0], edev->ea[1], edev->ea[2],
				edev->ea[3], edev->ea[4], edev->ea[5]);
		} else if(de != nil && de->hasmac
		    && memcmp(de->mac, zero, Eaddrlen) != 0){
			memmove(edev->ea, de->mac, Eaddrlen);
			dprint("smte: MAC from DTB: %02X:%02X:%02X:%02X:%02X:%02X\n",
				edev->ea[0], edev->ea[1], edev->ea[2],
				edev->ea[3], edev->ea[4], edev->ea[5]);
		} else {
			hi = csr32r(ct->regs, MacAddr1hi);
			me = csr32r(ct->regs, MacAddr1me);
			lo = csr32r(ct->regs, MacAddr1lo);
			edev->ea[0] = hi & 0xff;  edev->ea[1] = (hi>>8) & 0xff;
			edev->ea[2] = me & 0xff;  edev->ea[3] = (me>>8) & 0xff;
			edev->ea[4] = lo & 0xff;  edev->ea[5] = (lo>>8) & 0xff;
			dprint("smte: MAC read from hardware: %02X:%02X:%02X:%02X:%02X:%02X\n",
				edev->ea[0], edev->ea[1], edev->ea[2],
				edev->ea[3], edev->ea[4], edev->ea[5]);
		}
	}

	if(ethinit(edev) < 0){
		print("smte: ERROR ethinit failed\n");
		edev->ctlr = nil;
		return -1;
	}

	smteether[edev->ctlrno] = edev;
	edev->ctl = smtectl;

	if(irq > 0){
		/*
		 * IRQ-driven.  plic.c routes our source to the boot hart's
		 * S-context, and ethirq drains the rings + W1Cs status on
		 * every entry.  The K1 EMAC has no summary bits, but every
		 * source we enable is one we can fully ack from inside
		 * ethirq, so no storm.  We still register etherclock as a
		 * backstop so transient masking bugs or missed wake-ups
		 * don't strand frames in the rings indefinitely.
		 */
		intrenable(irq, ethirq, edev, BUSUNKNOWN, edev->name);
		dprint("smte: %s irq-driven on irq %d\n", edev->name, irq);
	}
	if(!smteclockregistered){
		/* 1ms rather than 10ms polling interval to improve worst-case latency */
		addclock0link(etherclock, 1);
		smteclockregistered = 1;
		dprint("smte: %s clock-poll backstop tick=1ms\n", edev->name);
	}
	return 0;
}

/* Driver registration -------------------------------------------------------- */

void
ethersmtelink(void)
{
	addethercard("smte", etherpnp);
}
