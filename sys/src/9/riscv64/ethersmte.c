/*
 * Driver for the SpacemiT K1 Ethernet controller (EMAC/smte)
 * for 9front / Plan 9 from Bell Labs
 *
 * Based on:
 *   OpenBSD if_smte.c   - Patrick Wildt, Mark Kettenis
 *   9front ethercycv.c  - general driver structure
 *
 * Hardware:
 *   0x0000-0x002c  DMA registers
 *   0x0100-0x01e4  MAC/GMAC registers
 *   separate APMU  RGMII delay lines
 *
 * riscv64/K1 notes:
 *   - cleandse/invaldse not yet available on riscv64: omitted
 *   - completion is interrupt-driven via intrenable(); etherclock()
 *     remains registered as a 100Hz backstop in case a wake-up is missed
 *   - SoC identified via SDHC_VID_PID (vendor 0xa1312)
 */

#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/netif.h"
#include "../port/etherif.h"

#define Rbsz	ROUNDUP(sizeof(Etherpkt)+16, 64)

/*
 * Verbosity. 0 = quiet (errors/warnings only); 1 = bring-up info and the
 * periodic stats line; 2 = per-frame RX header dump. Set at runtime via
 *   echo debug 1 >/net/ether0/clone/ctl
 */
static int debug;

#define dprint		if(debug) print
#define dprint2		if(debug >= 2) print

/*
 * Default MMIO layout for the SpacemiT K1's first EMAC, used only as
 * a fallback when no matching DTB entry is found.  Normal operation
 * pulls base / IRQ / APMU base / MAC out of /soc/ethernet@... via
 * the Dtbether table built by bootargs.c.
 */
enum {
	EMAC0_PHYS	= 0xcac80000UL,
	EMAC0_SIZE	= 0x420,
	EMAC0_IRQ	= 131,

	APMU_PHYS	= 0xd4282800UL,
	APMU_SIZE	= 0x400,

	/* RGMII delays: 1560 ps = 100 steps x 15.6 ps (from vendor DTS) */
	SMTE_DEFAULT_RXDELAY	= 100,
	SMTE_DEFAULT_TXDELAY	= 100,

	/* SDH used for SoC fingerprinting */
	SDH1_PHYS	= 0xD4280000,
	SDH1_SIZE	= 0x1000,
	SDHC_VID_PID	= 0x100,
	K1_VENDOR_ID	= 0xa1312,
};

/* DMA registers (byte offset) */
enum {
	DMA_CONFIG				= 0x0000,
	DMA_CONFIG_SOFTWARE_RESET		= (1U << 0),
	DMA_CONFIG_BURST_LENGTH_16		= (0x10 << 1),
	DMA_CONFIG_STRICT_BURST			= (1U << 17),
	DMA_CONFIG_DMA_64BIT_MODE		= (1U << 18),

	DMA_CTRL				= 0x0004,
	DMA_CTRL_START_STOP_TX_DMA		= (1U << 0),
	DMA_CTRL_START_STOP_RX_DMA		= (1U << 1),

	/*
	 * DMA_STATUS_IRQ — W1C.  This controller has no "interrupt summary"
	 * bits (no NIS/AIS-style aggregate); every enabled source acts on
	 * its own and must be W1C'd individually.  Bit 8 is a separate
	 * MAC-level interrupt that we never enable (MAC_INTR_ENABLE = 0).
	 */
	DMA_STATUS_IRQ				= 0x0008,	/* W1C */
	DMA_STATUS_IRQ_TX_TRANSFER_DONE		= (1U << 0),
	DMA_STATUS_IRQ_TX_DES_UNAVAILABLE	= (1U << 1),
	DMA_STATUS_IRQ_TX_DMA_STOPPED		= (1U << 2),
	DMA_STATUS_IRQ_RX_TRANSFER_DONE		= (1U << 4),
	DMA_STATUS_IRQ_RX_DES_UNAVAILABLE	= (1U << 5),
	DMA_STATUS_IRQ_RX_DMA_STOPPED		= (1U << 6),
	DMA_STATUS_IRQ_RX_MISSED_FRAME		= (1U << 7),
	DMA_STATUS_IRQ_MAC			= (1U << 8),

	/*
	 * DMA_INTR_ENABLE — per-source enables at the same bit positions
	 * as DMA_STATUS_IRQ.  RX_DES_UNAVAILABLE fires the moment the chip
	 * enters Suspended (no chip-owned descriptor at its pointer); the
	 * mask-while-busy pattern in ethirq drains the ring and POLL_DEMAND
	 * kicks the chip back to Running before we restore INTR_ENABLE, so
	 * no storm in practice.  TX_DES_UNAVAILABLE stays off — TX submit
	 * already POLL_DEMANDs and IRQ on dequeue is enough.
	 */
	DMA_INTR_ENABLE				= 0x000c,
	DMA_INTR_ENABLE_TX_TRANSFER_DONE	= (1U << 0),
	DMA_INTR_ENABLE_TX_DES_UNAVAILABLE	= (1U << 1),	/* not safe to enable */
	DMA_INTR_ENABLE_TX_DMA_STOPPED		= (1U << 2),
	DMA_INTR_ENABLE_RX_TRANSFER_DONE	= (1U << 4),
	DMA_INTR_ENABLE_RX_DES_UNAVAILABLE	= (1U << 5),
	DMA_INTR_ENABLE_RX_DMA_STOPPED		= (1U << 6),
	DMA_INTR_ENABLE_RX_MISSED_FRAME		= (1U << 7),

	DMA_TRANSMIT_AUTO_POLL_COUNTER		= 0x0010,
	DMA_TRANSMIT_POLL_DEMAND		= 0x0014,
	DMA_RECEIVE_POLL_DEMAND			= 0x0018,
	DMA_TRANSMIT_BASE_ADDRESS		= 0x001c,
	DMA_RECEIVE_BASE_ADDRESS		= 0x0020,

	DMA_RECEIVE_IRQ_MITIGATION		= 0x002c,
	DMA_RECEIVE_IRQ_MITIGATION_ENABLE	= (1U << 31),
};

/* MAC registers (byte offset) */
enum {
	MAC_GLOBAL_CTRL				= 0x0100,
	MAC_GLOBAL_CTRL_SPEED_MASK		= (0x3 << 0),
	MAC_GLOBAL_CTRL_SPEED_10		= (0x0 << 0),
	MAC_GLOBAL_CTRL_SPEED_100		= (0x1 << 0),
	MAC_GLOBAL_CTRL_SPEED_1000		= (0x2 << 0),
	MAC_GLOBAL_CTRL_DUPLEX_MODE		= (1U << 2),

	MAC_TRANSMIT_CTRL			= 0x0104,
	MAC_TRANSMIT_CTRL_TX_ENABLE		= (1U << 0),
	MAC_TRANSMIT_CTRL_TX_AUTO_RETRY		= (1U << 3),

	MAC_RECEIVE_CTRL			= 0x0108,
	MAC_RECEIVE_CTRL_RX_ENABLE		= (1U << 0),
	MAC_RECEIVE_CTRL_STORE_FORWARD		= (1U << 3),

	MAC_MAXIMUM_FRAME_SIZE			= 0x010c,
	MAC_TRANSMIT_JABBER_SIZE		= 0x0110,
	MAC_RECEIVE_JABBER_SIZE			= 0x0114,

	MAC_ADDR_CTRL				= 0x0118,
	MAC_ADDR_CTRL_MAC_ADDR1_ENABLE		= (1U << 0),

	MAC_ADDR1_HI				= 0x0120,
	MAC_ADDR1_ME				= 0x0124,
	MAC_ADDR1_LO				= 0x0128,

	MAC_MDIO_CTRL				= 0x01a0,
	MAC_MDIO_CTRL_START_MDIO_TRANS		= (1U << 15),
	MAC_MDIO_CTRL_MDIO_READ_WRITE		= (1U << 10),
	MAC_MDIO_CTRL_REGISTER_ADDRESS_SHIFT	= 5,
	MAC_MDIO_CTRL_PHY_ADDRESS_SHIFT		= 0,

	MAC_MDIO_DATA				= 0x01a4,

	MAC_TRANSMIT_FIFO_ALMOST_FULL		= 0x01c0,
	MAC_TRANSMIT_PACKET_START_THRESHOLD	= 0x01c4,
	MAC_RECEIVE_PACKET_START_THRESHOLD	= 0x01c8,

	MAC_STATUS_IRQ				= 0x01e0,	/* W1C: underrun, jabber */
	MAC_INTR_ENABLE				= 0x01e4,

	/*
	 * IEEE-1588 / PTP timestamping block.  We don't use PTP, but the
	 * status and enable registers need to be quiesced at init: a
	 * prior boot stage can leave PTP_1588_IRQ_EN non-zero with
	 * PTP_1588_IRQ_STS bits latched, and those will keep the chip's
	 * IRQ line asserted regardless of what we do to DMA_INTR_ENABLE.
	 */
	PTP_1588_IRQ_STS			= 0x0360,	/* W1C */
	PTP_1588_IRQ_EN				= 0x0364,
};

/*
 * APMU registers (byte offset from APMU_PHYS = 0xd4282800).
 * Per the K1 device tree we read from /soc/ethernet@…:
 *   EMAC0 ctrl-reg  = 0x3e4   (AXI ID, PHY-intf select, RMII clk select)
 *   EMAC0 dline-reg = 0x3e8   (RGMII delay line)
 * Jose's original 0x0000/0x0004 were poking at the wrong module's registers
 * (CCIC1/ISP clocks); EMAC operation came from firmware pre-init.
 */
enum {
	APMU_EMAC_CLK_RST_CTRL			= 0x03e4,
	/*
	 * EMAC0_CLK_RST_CTRL bit fields (from the K1 user manual,
	 * "High-Speed Interface System / EMAC0_CLK_RST_CTRL"):
	 *   bit 0  = BUS_EN — AXI bus clock enable (1 = on)
	 *   bit 1  = BUS_RST — *Reset Release*  (1 = released, 0 = held in reset)
	 *   bit 13 = AXI_MST_ID — use single AXI ID for transfers
	 * EMAC1 is at offset +0x8 with the same layout.
	 */
	APMU_EMAC_BUS_EN			= (1U << 0),
	APMU_EMAC_BUS_RST_RELEASE		= (1U << 1),
	APMU_EMAC_AXI_MST_ID			= (1U << 13),
	APMU_EMAC_PTP_EN			= (1U << 15),

	APMU_EMAC_RGMII_DLINE			= 0x03e8,
	APMU_EMAC_RGMII_DLINE_RX_EN		= (1U << 0),
	APMU_EMAC_RGMII_DLINE_RX_STEP_15P6	= (0U << 4),
	APMU_EMAC_RGMII_DLINE_RX_DELAY_SHIFT	= 8,
	APMU_EMAC_RGMII_DLINE_TX_EN		= (1U << 16),
	APMU_EMAC_RGMII_DLINE_TX_STEP_15P6	= (0U << 20),
	APMU_EMAC_RGMII_DLINE_TX_DELAY_SHIFT	= 24,
};

/* RX descriptors */
enum {
	RX_DESC0_FRAME_PACKET_LENGTH_MASK	= (0x3fff << 0),
	RX_DESC0_FRAME_RUNT			= (1U << 15),
	RX_DESC0_FRAME_CRC_ERR			= (1U << 20),
	RX_DESC0_FRAME_MAX_LEN_ERR		= (1U << 21),
	RX_DESC0_FRAME_JABBER_ERR		= (1U << 22),
	RX_DESC0_FRAME_LENGTH_ERR		= (1U << 23),
	RX_DESC0_OWN				= (1U << 31),
	RX_DESC1_SIZE1_MASK			= (0xfffff << 0),
	RX_DESC1_END_RING			= (1U << 26),
};

/* TX descriptors */
enum {
	TX_DESC0_OWN				= (1U << 31),
	TX_DESC1_SIZE1_MASK			= (0xfff << 0),
	TX_DESC1_END_RING			= (1U << 26),
	TX_DESC1_FIRST_SEGMENT			= (1U << 29),
	TX_DESC1_LAST_SEGMENT			= (1U << 30),
	TX_DESC1_INTERRUPT_ON_COMPLETION	= (1U << 31),
};

/* Standard MII registers */
enum {
	MII_BMCR	= 0,
	MII_BMSR	= 1,
	MII_PHYSID1	= 2,
	MII_PHYSID2	= 3,
	MII_ANLPAR	= 5,
	MII_GBCR	= 9,	/* 1000BASE-T control (advertisement) */
	MII_GBSR	= 10,

	BMCR_ANENABLE	= (1U << 12),
	BMCR_ANRESTART	= (1U << 9),

	GBCR_ADV1000FULL	= (1U << 9),
	GBCR_ADV1000HALF	= (1U << 8),

	BMSR_ANEGCOMPLETE	= (1U << 5),
	BMSR_LSTATUS		= (1U << 2),

	LPA_100FULL	= (1U << 8),
	LPA_100HALF	= (1U << 7),
	LPA_10FULL	= (1U << 6),
	LPA_10HALF	= (1U << 5),

	GBSR_LP1000FULL	= (1U << 11),
	GBSR_LP1000HALF	= (1U << 10),

	PHY_ADDR	= 1,
	Linkdelay	= 500,
};

enum {
	RXRING	= 512,
	TXRING	= 512,
	MAXSMTE	= 2,
};


#define SMTE_CBSZ 64	/* X60/C908 D-cache line size */

/*
 * K1 DMA is non-coherent (dma-noncoherent in the vendor DT).
 * CPU→HW: cbo.flush (writeback + invalidate) — push fresh data to DRAM.
 * HW→CPU: cbo.inval (invalidate only) — must NOT writeback or a stale
 * CPU-cached descriptor (e.g. OWN=host from an old replenish that the
 * line later re-prefetched) would clobber the chip's just-written
 * OWN=chip update in DRAM, stranding the frame in a slot ethrx never
 * sees.  Symptom: RX missed frame + ring stuck at prod=cons-1.
 */
static void
smtecacheflush(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(SMTE_CBSZ - 1);
	last = ((uintptr)va + len + SMTE_CBSZ - 1) & ~(uintptr)(SMTE_CBSZ - 1);
	for(; a < last; a += SMTE_CBSZ)
		cboflush((void*)a);
	coherence();
}

static void
smtecacheinval(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(SMTE_CBSZ - 1);
	last = ((uintptr)va + len + SMTE_CBSZ - 1) & ~(uintptr)(SMTE_CBSZ - 1);
	for(; a < last; a += SMTE_CBSZ)
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
	int	dmarunning;	/* DMA started at link-up; gates POLL_DEMAND writes */
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

	if(base == EMAC0_PHYS){
		pins = pins0;
		n = nelem(pins0);
	}else{
		pins = pins1;
		n = nelem(pins1);
	}
	if(aib == nil)
		aib = vmap(0xd401e000, 0x1000);
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
		gpio = vmap(0xd4019000, 0x200);
	if(gpio == nil)
		return;
	bank = c->rstpin >> 5 == 3? 0x100: (c->rstpin >> 5)*4;
	bit = 1 << (c->rstpin & 31);
	gpio[(bank + 0x54)/4] = bit;	/* direction: output */
	gpio[(bank + 0x24)/4] = bit;	/* drive low: assert reset */
	delay(10);
	gpio[(bank + 0x18)/4] = bit;	/* drive high: release */
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
	csr32w(c->regs, MAC_MDIO_DATA, v);
	csr32w(c->regs, MAC_MDIO_CTRL,
		MAC_MDIO_CTRL_START_MDIO_TRANS
		| (reg & 0x1f) << MAC_MDIO_CTRL_REGISTER_ADDRESS_SHIFT
		| (PHY_ADDR & 0x1f) << MAC_MDIO_CTRL_PHY_ADDRESS_SHIFT);
	while(csr32r(c->regs, MAC_MDIO_CTRL) & MAC_MDIO_CTRL_START_MDIO_TRANS)
		tsleep(&up->sleep, return0, nil, 1);
}

static u16int
mdread(Ctlr *c, int reg)
{
	csr32w(c->regs, MAC_MDIO_DATA, 0);
	csr32w(c->regs, MAC_MDIO_CTRL,
		MAC_MDIO_CTRL_START_MDIO_TRANS | MAC_MDIO_CTRL_MDIO_READ_WRITE
		| (reg & 0x1f) << MAC_MDIO_CTRL_REGISTER_ADDRESS_SHIFT
		| (PHY_ADDR & 0x1f) << MAC_MDIO_CTRL_PHY_ADDRESS_SHIFT);
	while(csr32r(c->regs, MAC_MDIO_CTRL) & MAC_MDIO_CTRL_START_MDIO_TRANS)
		tsleep(&up->sleep, return0, nil, 1);
	return csr32r(c->regs, MAC_MDIO_DATA) & 0xffff;
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

	v = csr32r(c->regs, DMA_CTRL);
	csr32w(c->regs, DMA_CTRL, v & ~DMA_CTRL_START_STOP_TX_DMA);
	ilock(&c->txlock);
	for(i = 0; i < TXRING; i++){
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
	smtecacheflush(c->txr, 4 * TXRING * sizeof(u32int));
	iunlock(&c->txlock);
	csr32w(c->regs, DMA_TRANSMIT_BASE_ADDRESS, PADDR(c->txr));
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

	id1 = mdread(c, MII_PHYSID1);
	id2 = mdread(c, MII_PHYSID2);
	dprint("smte: PHY addr=%d oui=0x%06ux model=0x%02ux rev=0x%01ux\n",
		PHY_ADDR,
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
	mdwrite(c, MII_GBCR, GBCR_ADV1000FULL | GBCR_ADV1000HALF);
	mdwrite(c, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	for(;;){
		bmsr = mdread(c, MII_BMSR);
		if((bmsr & BMSR_LSTATUS) == 0){
			print("smte: %s link down (BMSR=0x%04ux)\n", edev->name, bmsr);
			ethersetlink(edev, 0);
			csr32w(c->regs, MAC_TRANSMIT_CTRL, 0);
			csr32w(c->regs, MAC_RECEIVE_CTRL, 0);
			txreset(c);
			dprint("smte: MAC TX/RX disabled, waiting for link\n");
			while((mdread(c, MII_BMSR) & BMSR_LSTATUS) == 0)
				tsleep(&up->sleep, return0, nil, Linkdelay);
			dprint("smte: physical link detected\n");
		}

		dprint("smte: waiting for autoneg (BMSR=0x%04ux)\n", mdread(c, MII_BMSR));
		while((mdread(c, MII_BMSR) & BMSR_ANEGCOMPLETE) == 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);

		lpa  = mdread(c, MII_ANLPAR);
		gbsr = mdread(c, MII_GBSR);
		dprint("smte: autoneg complete ANLPAR=0x%04ux GBSR=0x%04ux\n", lpa, gbsr);

		ctrl = csr32r(c->regs, MAC_GLOBAL_CTRL)
			& ~(MAC_GLOBAL_CTRL_SPEED_MASK | MAC_GLOBAL_CTRL_DUPLEX_MODE);
		dup  = "HD";

		if(gbsr & GBSR_LP1000FULL){
			ctrl |= MAC_GLOBAL_CTRL_SPEED_1000 | MAC_GLOBAL_CTRL_DUPLEX_MODE;
			speed = 1000; dup = "FD";
		} else if(gbsr & GBSR_LP1000HALF){
			ctrl |= MAC_GLOBAL_CTRL_SPEED_1000;
			speed = 1000;
		} else if(lpa & LPA_100FULL){
			ctrl |= MAC_GLOBAL_CTRL_SPEED_100 | MAC_GLOBAL_CTRL_DUPLEX_MODE;
			speed = 100; dup = "FD";
		} else if(lpa & LPA_100HALF){
			ctrl |= MAC_GLOBAL_CTRL_SPEED_100;
			speed = 100;
		} else if(lpa & LPA_10FULL){
			ctrl |= MAC_GLOBAL_CTRL_SPEED_10 | MAC_GLOBAL_CTRL_DUPLEX_MODE;
			speed = 10; dup = "FD";
		} else {
			ctrl |= MAC_GLOBAL_CTRL_SPEED_10;
			speed = 10;
		}

		csr32w(c->regs, MAC_GLOBAL_CTRL, ctrl);
		csr32w(c->regs, MAC_TRANSMIT_CTRL,
			MAC_TRANSMIT_CTRL_TX_ENABLE | MAC_TRANSMIT_CTRL_TX_AUTO_RETRY);
		csr32w(c->regs, MAC_RECEIVE_CTRL,
			MAC_RECEIVE_CTRL_RX_ENABLE | MAC_RECEIVE_CTRL_STORE_FORWARD);

		/*
		 * Link is up.  Start TX/RX DMA now, after MAC TX/RX are armed
		 * so only frames the MAC has just accepted flow into our
		 * descriptors — no kexec-leftover FIFO residue.  Then drain
		 * once for safety, W1C status, and finally arm DMA_INTR_ENABLE
		 * for IRQ-driven completion.  This order matches Linux's
		 * emac_configure_{tx,rx}.
		 */
		csr32w(c->regs, DMA_CTRL,
			DMA_CTRL_START_STOP_TX_DMA | DMA_CTRL_START_STOP_RX_DMA);
		c->dmarunning = 1;	/* POLL_DEMAND writes are now safe */
		ethrx(edev);
		ethtx(edev);
		csr32w(c->regs, DMA_STATUS_IRQ, ~0U);
		/*
		 * RX_TRANSFER_DONE deliberately omitted: enabling it after a
		 * Linux kexec breaks DHCP/RX, independently confirmed even
		 * with timebase correctly harvested from DTB.  Hypothesis:
		 * chip refuses to process further RX frames until host acks
		 * bit 4 via W1C, and W1C of bit 4 doesn't reliably clear it
		 * on this silicon.  RX completion served by 1ms polling
		 * backstop; TX completion is IRQ-driven.
		 */
		csr32w(c->regs, DMA_INTR_ENABLE,
			DMA_INTR_ENABLE_TX_TRANSFER_DONE
			| DMA_INTR_ENABLE_TX_DMA_STOPPED
			| DMA_INTR_ENABLE_RX_DES_UNAVAILABLE
			| DMA_INTR_ENABLE_RX_DMA_STOPPED
			| DMA_INTR_ENABLE_RX_MISSED_FRAME);
		/* readback drains the posted write */
		ctrl = csr32r(c->regs, DMA_INTR_ENABLE);
		dprint("smte: %s irq-driven DMA_INTR_ENABLE=%#ux DMA_STATUS=%#ux\n",
			edev->name, ctrl, csr32r(c->regs, DMA_STATUS_IRQ));

		print("smte: %s link up %dMbps %s\n", edev->name, speed, dup);
		dprint("smte:   MAC_GLOBAL_CTRL=0x%08ux\n", ctrl);
		dprint("smte:   MAC_TX_CTRL=0x%08ux MAC_RX_CTRL=0x%08ux\n",
			csr32r(c->regs, MAC_TRANSMIT_CTRL),
			csr32r(c->regs, MAC_RECEIVE_CTRL));

		ethersetspeed(edev, speed);
		ethersetlink(edev, 1);

		while((mdread(c, MII_BMSR) & BMSR_LSTATUS) != 0)
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
	 * it, and POLL_DEMAND just bounces off.  With all RXRING slots
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
		r[1] = Rbsz & RX_DESC1_SIZE1_MASK;
		if(i == RXRING - 1)
			r[1] |= RX_DESC1_END_RING;
		r[2] = (u32int)pa;
		r[3] = 0;
		coherence();
		r[0] = RX_DESC0_OWN;
		/* push descriptor, drop any stale buffer cache so HW writes win */
		smtecacheflush(r, 16);
		smtecacheinval(bp->rp, Rbsz);
		c->rxprodi = (c->rxprodi + 1) & (RXRING - 1);
	}
	/*
	 * POLL_DEMAND is only meaningful while DMA RX is in Running or
	 * Suspended state.  Writing it while DMA is Stopped (the initial
	 * fill from ethinit before link-up) latches DMA_STATUS_IRQ bit 4
	 * (RECEIVE_TRANSFER_DONE_IRQ) — and that bit then refuses to W1C,
	 * driving the chip's IRQ line high and storming the PLIC once we
	 * enable the source.
	 */
	if(c->dmarunning)
		csr32w(c->regs, DMA_RECEIVE_POLL_DEMAND, 0xFF);
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
		if(r[0] & RX_DESC0_OWN){
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
				rn = &c->rxr[4 * ((c->rxconsi + k) & (RXRING - 1))];
				smtecacheinval(rn, 16);
				if((rn[0] & RX_DESC0_OWN) == 0){
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
				c->rxconsi = (c->rxconsi + 1) & (RXRING - 1);
			}
			replenish(c);
			continue;
		}
		s   = r[0];
		len = s & RX_DESC0_FRAME_PACKET_LENGTH_MASK;
		bp  = c->rxs[c->rxconsi];
		c->rxs[c->rxconsi] = nil;
		if(bp != nil && len > 0)
			smtecacheinval(bp->rp, len);

		if(len < 4 || s & (RX_DESC0_FRAME_RUNT   | RX_DESC0_FRAME_CRC_ERR |
		                    RX_DESC0_FRAME_MAX_LEN_ERR | RX_DESC0_FRAME_JABBER_ERR |
		                    RX_DESC0_FRAME_LENGTH_ERR)){
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
		c->rxconsi = (c->rxconsi + 1) & (RXRING - 1);
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
	for(i = 0; i < TXRING; i++)
		if(c->txs[i] != nil)
			ntx++;
	nrx = 0;
	for(i = 0; i < RXRING; i++)
		if(c->rxs[i] != nil)
			nrx++;
	p = seprint(p, e, "tx: prod %d cons %d inflight %d done %ud\n",
		c->txi, c->txcons, ntx, c->txdone);
	p = seprint(p, e, "rx: prod %d cons %d bufs %d frames %ud\n",
		c->rxprodi, c->rxconsi, nrx, c->rxframes);
	p = seprint(p, e, "irq bits seen %#ux dma ctl %#ux status %#ux intren %#ux\n",
		c->irqseen, csr32r(c->regs, DMA_CTRL),
		csr32r(c->regs, DMA_STATUS_IRQ),
		csr32r(c->regs, DMA_INTR_ENABLE));
	p = seprint(p, e, "mac global %#ux tx %#ux rx %#ux\n",
		csr32r(c->regs, MAC_GLOBAL_CTRL),
		csr32r(c->regs, MAC_TRANSMIT_CTRL),
		csr32r(c->regs, MAC_RECEIVE_CTRL));
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
		if(r[0] & TX_DESC0_OWN)
			break;
		freeb(c->txs[c->txcons]);
		c->txs[c->txcons] = nil;
		c->txdone++;
		c->txcons = (c->txcons + 1) & (TXRING - 1);
	}
	for(;;){
		r = &c->txr[4 * c->txi];
		smtecacheinval(r, 16);		/* discard stale cache before reading HW status */
		if(r[0] & TX_DESC0_OWN){
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
		r[1] = (BLEN(bp) & TX_DESC1_SIZE1_MASK)
			| TX_DESC1_FIRST_SEGMENT | TX_DESC1_LAST_SEGMENT
			| TX_DESC1_INTERRUPT_ON_COMPLETION;
		if(c->txi == TXRING - 1)
			r[1] |= TX_DESC1_END_RING;
		r[2] = (u32int)pa;
		r[3] = 0;
		coherence();
		r[0] = TX_DESC0_OWN;
		smtecacheflush(r, 16);		/* push descriptor to DRAM for HW */
		if(c->dmarunning)
			csr32w(c->regs, DMA_TRANSMIT_POLL_DEMAND, 0xFF);
		c->txi = (c->txi + 1) & (TXRING - 1);
	}
	iunlock(&c->txlock);
}

/* IRQ / polling -------------------------------------------------------------- */

/*
 * Mask-then-clear interrupt handling.  The chip's IRQ output is the
 * level of (DMA_STATUS_IRQ & DMA_INTR_ENABLE) — any enabled source
 * that re-asserts between our W1C and our return drives the PLIC's
 * level-triggered gateway to re-forward on every completion message
 * (PLIC v1.0 spec, "Interrupt Gateways"), and plic.c's claim/complete
 * loop will keep dispatching us forever.  We zero DMA_INTR_ENABLE
 * entirely for the duration of one handler entry, drain both rings,
 * and restore.  Doing the full mask (not just the two completion
 * bits) is paranoia that costs a single MMIO write per IRQ and means
 * a stale DMA_STOPPED or MISSED_FRAME also can't pin the line high.
 */
static void
ethirq(Ureg *, void *arg)
{
	Ether *edev;
	Ctlr *c;
	u32int fl, en, macsts, ptpsts;

	edev = arg;
	c    = edev->ctlr;

	en = csr32r(c->regs, DMA_INTR_ENABLE);
	if(en != 0){
		csr32w(c->regs, DMA_INTR_ENABLE, 0);
		/*
		 * Read back to force the posted write to commit before we
		 * sample status: otherwise the chip's IRQ line can still be
		 * asserted from a stale enabled-source view when plic.c
		 * re-claims, and the 64-trip cap mistakes us for a storm.
		 */
		(void)csr32r(c->regs, DMA_INTR_ENABLE);
	}

	fl = csr32r(c->regs, DMA_STATUS_IRQ);
	macsts = csr32r(c->regs, MAC_STATUS_IRQ);
	ptpsts = csr32r(c->regs, PTP_1588_IRQ_STS);
	/* Belt-and-suspenders: W1C the other two too, in case one of them
	 * is feeding the chip's IRQ line. */
	if(macsts != 0)
		csr32w(c->regs, MAC_STATUS_IRQ, macsts);
	if(ptpsts != 0)
		csr32w(c->regs, PTP_1588_IRQ_STS, ptpsts);
	if(fl != 0){
		/* W1C every bit we observed.  No NIS/AIS summary on this part. */
		csr32w(c->regs, DMA_STATUS_IRQ, fl);

		/*
		 * Diagnostics use iprint() because ethirq runs at interrupt
		 * level once intrenable() wires us up; print() can block on
		 * the output queue, which is not legal here.
		 */
		if(fl & DMA_STATUS_IRQ_RX_DES_UNAVAILABLE)
			/* chip just entered Suspended; silent kick (fires often) */
			csr32w(c->regs, DMA_RECEIVE_POLL_DEMAND, 0xFF);
		if(fl & DMA_STATUS_IRQ_RX_MISSED_FRAME){
			iprint("smte: %s RX missed frame (status=%#08ux "
				"prod=%d cons=%d)\n",
				edev->name, fl, c->rxprodi, c->rxconsi);
			csr32w(c->regs, DMA_RECEIVE_POLL_DEMAND, 0xFF);
		}
		if(fl & DMA_STATUS_IRQ_RX_DMA_STOPPED){
			iprint("smte: %s RX DMA stopped, restarting\n", edev->name);
			csr32w(c->regs, DMA_RECEIVE_POLL_DEMAND, 0xFF);
		}
		if(fl & DMA_STATUS_IRQ_TX_DMA_STOPPED){
			iprint("smte: %s TX DMA stopped, restarting\n", edev->name);
			csr32w(c->regs, DMA_TRANSMIT_POLL_DEMAND, 0xFF);
		}
	}

	/*
	 * Drain unconditionally — ethrx/ethtx are self-bounded by the
	 * per-descriptor OWN flag, so calling them when nothing is
	 * ready is a cheap one-descriptor read.  Also covers the polling
	 * backstop path (etherclock → etherserve → ethirq).
	 */
	ethrx(edev);
	ethtx(edev);

	if(en != 0){
		coherence();
		csr32w(c->regs, DMA_INTR_ENABLE, en);
	}
}

static Ether *smteether[MAXSMTE];
static Ctlr smtectlr[MAXSMTE];
static int smteclockregistered;

static void
etherserve(Ether *edev)
{
	Ctlr *c;
	u32int fl;
	int lasttxi;

	c = edev->ctlr;

	/* accumulate raw DMA status bits before ethirq() W1Cs them */
	fl = csr32r(c->regs, DMA_STATUS_IRQ);
	c->irqseen |= fl;

	ethirq(nil, edev);

	/* e.g. echo debug 1 > /net/ether0/0/ctl to get periodic stats */
	if(debug && ++c->statstick >= 5000){
		c->statstick = 0;
		lasttxi = (c->txi - 1) & (TXRING - 1);
		print("smte: %s stats irq_seen=0x%08ux rxcons=%d rxprod=%d "
			"rx_frames=%ud txi=%d tx_done=%ud "
			"rxd0=0x%08ux txd0[%d]=0x%08ux "
			"RXCTL=0x%08ux TXCTL=0x%08ux DMACTL=0x%08ux\n",
			edev->name, c->irqseen, c->rxconsi, c->rxprodi, c->rxframes,
			c->txi, c->txdone,
			c->rxr[4*c->rxconsi], lasttxi, c->txr[4*lasttxi],
			csr32r(c->regs, MAC_RECEIVE_CTRL),
			csr32r(c->regs, MAC_TRANSMIT_CTRL),
			csr32r(c->regs, DMA_CTRL));
		c->irqseen = 0;
	}
}

static void
etherclock(void)
{
	int i;

	for(i = 0; i < MAXSMTE; i++)
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
	 * Gate the AXI clock AND assert bus-reset (BUS_RST_RELEASE=0 =
	 * held in reset), then after a generous delay re-enable the
	 * clock and release reset, preserving unrelated clock-config
	 * bits firmware may have set (1588/PTP, RGMII TXC source, etc).
	 * Also enables PTP clock gate (BIT(15)) to match Linux probe,
	 * so 1588 register block is reachable if we ever query it.
	 */
	{
		u32int origv = csr32r(c->apmu, c->apmuctrloff);
		u32int preserve = origv & ~(APMU_EMAC_BUS_EN
			| APMU_EMAC_BUS_RST_RELEASE | APMU_EMAC_AXI_MST_ID
			| APMU_EMAC_PTP_EN);
		csr32w(c->apmu, c->apmuctrloff, preserve);	/* clock off + reset asserted */
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff,
			preserve | APMU_EMAC_BUS_EN | APMU_EMAC_PTP_EN);	/* clocks on, still in reset */
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff,
			preserve | APMU_EMAC_BUS_EN | APMU_EMAC_PTP_EN
			| APMU_EMAC_BUS_RST_RELEASE | APMU_EMAC_AXI_MST_ID);
		microdelay(10000);
		print("smte: APMU CLK_RST[%#ux] %#ux -> %#ux\n",
			c->apmuctrloff, origv, csr32r(c->apmu, c->apmuctrloff));
	}

	/* Now the MAC block is reachable; stop everything. */
	csr32w(c->regs, MAC_INTR_ENABLE,   0);
	csr32w(c->regs, DMA_INTR_ENABLE,   0);
	csr32w(c->regs, MAC_TRANSMIT_CTRL, 0);
	csr32w(c->regs, MAC_RECEIVE_CTRL,  0);
	csr32w(c->regs, DMA_CTRL,          0);
	print("smte: MAC/DMA stopped\n");

	phyreset(c);

	/* APMU: RGMII delay lines */
	v = APMU_EMAC_RGMII_DLINE_RX_EN   | APMU_EMAC_RGMII_DLINE_TX_EN
	  | APMU_EMAC_RGMII_DLINE_RX_STEP_15P6 | APMU_EMAC_RGMII_DLINE_TX_STEP_15P6
	  | c->rxdelay << APMU_EMAC_RGMII_DLINE_RX_DELAY_SHIFT
	  | c->txdelay << APMU_EMAC_RGMII_DLINE_TX_DELAY_SHIFT;
	csr32w(c->apmu, c->apmudlineoff, v);
	print("smte: APMU DLINE[%#ux]=0x%08ux (rx=%d tx=%d steps x 15.6ps)\n",
		c->apmudlineoff, v, c->rxdelay, c->txdelay);

	/* MAC address */
	csr32w(c->regs, MAC_ADDR1_HI, edev->ea[1]<<8 | edev->ea[0]);
	csr32w(c->regs, MAC_ADDR1_ME, edev->ea[3]<<8 | edev->ea[2]);
	csr32w(c->regs, MAC_ADDR1_LO, edev->ea[5]<<8 | edev->ea[4]);
	csr32w(c->regs, MAC_ADDR_CTRL, MAC_ADDR_CTRL_MAC_ADDR1_ENABLE);
	dprint("smte: MAC %02X:%02X:%02X:%02X:%02X:%02X programmed\n",
		edev->ea[0], edev->ea[1], edev->ea[2],
		edev->ea[3], edev->ea[4], edev->ea[5]);

	/* FIFO thresholds and frame size limits */
	csr32w(c->regs, MAC_TRANSMIT_FIFO_ALMOST_FULL,        0x1f8);
	csr32w(c->regs, MAC_TRANSMIT_PACKET_START_THRESHOLD,  192);
	csr32w(c->regs, MAC_RECEIVE_PACKET_START_THRESHOLD,   12);
	csr32w(c->regs, MAC_MAXIMUM_FRAME_SIZE,               1536);
	csr32w(c->regs, MAC_TRANSMIT_JABBER_SIZE,             1514+18);
	csr32w(c->regs, MAC_RECEIVE_JABBER_SIZE,              1514+18);
	dprint("smte: FIFO thresholds and frame sizes programmed\n");

	/* reset DMA engine */
	csr32w(c->regs, DMA_CONFIG, DMA_CONFIG_SOFTWARE_RESET);
	microdelay(10000);
	csr32w(c->regs, DMA_CONFIG, 0);
	microdelay(10000);
	/*
	 * DMA_64BIT: enables 64-bit buffer addresses in descriptors
	 * (d2=bits[31:0] + d3=bits[63:32] per buffer). Required because
	 * iallocb() returns blocks above 4 GB on this system.
	 * The ring BASE address is still 32-bit (DMA_RECEIVE_BASE_ADDRESS),
	 * so descriptor rings must stay below 4 GB.
	 */
	v = DMA_CONFIG_DMA_64BIT_MODE | DMA_CONFIG_STRICT_BURST | DMA_CONFIG_BURST_LENGTH_16;
	csr32w(c->regs, DMA_CONFIG, v);
	dprint("smte: DMA reset, DMA_CONFIG=%#ux\n", csr32r(c->regs, DMA_CONFIG));

	/* Enable flow-control decode; harmless even without pause frames. */
	csr32w(c->regs, 0x0160 /* MAC_FC_CONTROL */, 1 /* DECODE_ENABLE */);

	/*
	 * Post-reset DMA tunables.  These registers are wiped by
	 * DMA_CONFIG_SOFTWARE_RESET above, so they MUST be programmed
	 * after the reset completes.
	 *
	 * RX interrupt coalescing: disabled.  We poll from etherclock()
	 * at HZ rate; with coalescing on, RX_TRANSFER_DONE is held off
	 * until either 64 frames pile up or the timer fires.
	 *
	 * TX auto-poll: when the engine finds OWN=0 (no work pending)
	 * it idles until either this counter expires or someone writes
	 * TRANSMIT_POLL_DEMAND.  The reset value on the K1 EMAC produces
	 * a ~40 ms idle->recheck period, which matched the ICMP echo
	 * latency pattern we observed.  Set to the minimum so the
	 * engine effectively spins on the descriptor.
	 */
	/*
	 * Do NOT clear DMA_RECEIVE_IRQ_MITIGATION here.  The reset default
	 * (0x00FFFF01: frame_count=1, timeout=0xFFFF, mitigation disabled)
	 * is what we want.  Writing 0 sets frame_count to 0 which the spec
	 * says is illegal ("Legal values: 1 - 255") — and the K1 silicon
	 * responds by *permanently* latching DMA_STATUS_IRQ bit 4
	 * (RECEIVE_TRANSFER_DONE_IRQ).  Once latched it will not W1C even
	 * with DMA running, which stalls IRQ-driven RX entirely.
	 *
	 * DMA_TRANSMIT_AUTO_POLL_COUNTER default is 0 already, so leave
	 * it alone too.
	 */
	dprint("smte: leaving MITIGATION=%#ux AUTOPOLL=%#ux at reset defaults\n",
		csr32r(c->regs, DMA_RECEIVE_IRQ_MITIGATION),
		csr32r(c->regs, DMA_TRANSMIT_AUTO_POLL_COUNTER));

	/* K1 EMAC uses 32-bit DMA addresses */
	c->rxr = xspanalloc(sizeof(u32int) * 4 * RXRING, 16, 0);
	c->txr = xspanalloc(sizeof(u32int) * 4 * TXRING, 16, 0);
	memset(c->rxr, 0, 4 * RXRING * sizeof(u32int));
	memset(c->txr, 0, 4 * TXRING * sizeof(u32int));
	smtecacheflush(c->rxr, 4 * RXRING * sizeof(u32int));
	smtecacheflush(c->txr, 4 * TXRING * sizeof(u32int));

	c->rxs = mallocz(sizeof(Block*) * RXRING, 1);
	c->txs = mallocz(sizeof(Block*) * TXRING, 1);
	if(c->rxs == nil || c->txs == nil){
		print("smte: ERROR mallocz failed\n");
		return -1;
	}
	dprint("smte: rings rxr=%p txr=%p (%d/%d slots, %d bytes/buf)\n",
		c->rxr, c->txr, RXRING, TXRING, Rbsz);

	/* fill all RXRING slots (rxs[] starts all nil from mallocz) */
	c->rxprodi = 0;
	c->rxconsi = 0;
	replenish(c);

	c->txi = 0;

	uintptr rxpa, txpa;
	rxpa = PADDR(c->rxr);
	txpa = PADDR(c->txr);
	csr32w(c->regs, DMA_RECEIVE_BASE_ADDRESS, (u32int)rxpa);
	csr32w(c->regs, DMA_TRANSMIT_BASE_ADDRESS, (u32int)txpa);
	dprint("smte: DMA base RX=0x%08ux TX=0x%08ux (readback RX=0x%08ux TX=0x%08ux)\n",
		(u32int)rxpa, (u32int)txpa,
		csr32r(c->regs, DMA_RECEIVE_BASE_ADDRESS),
		csr32r(c->regs, DMA_TRANSMIT_BASE_ADDRESS));

	/*
	 * Clear stale interrupt state from whoever ran before us (typically
	 * a Linux kexec source kernel that had MAC RX enabled with PTP and
	 * traffic in flight).  The chip's IRQ output is the OR of
	 * DMA_STATUS_IRQ, MAC_STATUS_IRQ, and PTP_1588_IRQ_STS — clear all.
	 *
	 * DMA_INTR_ENABLE stays at 0 across this window: even with MAC RX
	 * disabled, the previous stage's frames are still buffered in the
	 * RX FIFO, and starting DMA RX (below) immediately writes them into
	 * our descriptors and asserts RX_TRANSFER_DONE much faster than we
	 * can drain.  We let the 100Hz etherclock polling path consume that
	 * residue; ethproc flips DMA_INTR_ENABLE on once the link is up.
	 */
	csr32w(c->regs, PTP_1588_IRQ_EN, 0);
	csr32w(c->regs, PTP_1588_IRQ_STS, ~0U);
	csr32w(c->regs, DMA_STATUS_IRQ, ~0U);
	csr32w(c->regs, MAC_STATUS_IRQ, ~0U);
	csr32w(c->regs, MAC_INTR_ENABLE, 0);
	csr32w(c->regs, DMA_INTR_ENABLE, 0);
	dprint("smte: post-init DMA_STATUS_IRQ=%#ux PTP_STS=%#ux (polled until link up)\n",
		csr32r(c->regs, DMA_STATUS_IRQ),
		csr32r(c->regs, PTP_1588_IRQ_STS));

	/*
	 * DMA stays stopped until ethproc sees the link come up and turns
	 * both MAC TX/RX and the corresponding DMA channels on together.
	 * Starting RX DMA here, with MAC RX off but kexec residue in the
	 * RX FIFO, made DMA pull those frames into our descriptors and
	 * latch RX_TRANSFER_DONE faster than we could W1C it.
	 */
	csr32w(c->regs, DMA_CTRL, 0);
	dprint("smte: DMA held stopped; ethproc starts on link up\n");

	return 0;
}

/* SoC fingerprint ------------------------------------------------------------ */

static int
checksoc(void)
{
	ulong *sdh;
	u32int val, vendor;

	dprint("smte: probing SoC (SDH1 @ 0x%08ux)\n", SDH1_PHYS);
	sdh = (ulong*)vmap(SDH1_PHYS, SDH1_SIZE);
	if(sdh == nil){
		print("smte: ERROR vmap SDH failed\n");
		return 0;
	}
	val    = csr32r(sdh, SDHC_VID_PID);
	vendor = val & 0xFFFFF;
	vunmap(sdh, SDH1_SIZE);

	dprint("smte: SDHC_VID_PID=0x%08ux vendor=0x%05ux\n", val, vendor);
	if(vendor == K1_VENDOR_ID)
		return 1;
	print("smte: unknown SoC vendor 0x%05ux (expected K1 0x%05ux)\n",
		vendor, K1_VENDOR_ID);
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

	if(edev->ctlrno < 0 || edev->ctlrno >= MAXSMTE){
		dprint("smte: ctlrno %d out of range\n", edev->ctlrno);
		return -1;
	}
	ct = &smtectlr[edev->ctlrno];
	if(ct->regs != nil){
		dprint("smte: ether%d already registered\n", edev->ctlrno);
		return -1;
	}

	if(!checksoc())
		return -1;

	/*
	 * Locate this controller in the DTB by alias index.  No fallback
	 * to hardcoded values for the secondary card — if DTB has nothing
	 * for it, we don't know where it lives.  ether0 retains a fallback
	 * to the K1 EMAC0 defaults so kernels booted without a DTB still
	 * come up with one interface.
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
		size     = de->size ? de->size : EMAC0_SIZE;
		apmubase = de->apmubase ? de->apmubase : APMU_PHYS;
		irq      = de->irq;
		ct->apmuctrloff  = de->ctrlregoff  ? de->ctrlregoff  : APMU_EMAC_CLK_RST_CTRL;
		ct->apmudlineoff = de->dlineregoff ? de->dlineregoff : APMU_EMAC_RGMII_DLINE;
		print("smte: ether%d from DTB %s base=%#llux size=%#llux apmu=%#llux+{%#ux,%#ux} irq=%d\n",
			edev->ctlrno, de->path, base, size, apmubase,
			ct->apmuctrloff, ct->apmudlineoff, irq);
	} else if(edev->ctlrno == 0 && n == 0){
		base     = EMAC0_PHYS;
		size     = EMAC0_SIZE;
		apmubase = APMU_PHYS;
		irq      = EMAC0_IRQ;
		ct->apmuctrloff  = APMU_EMAC_CLK_RST_CTRL;
		ct->apmudlineoff = APMU_EMAC_RGMII_DLINE;
		print("smte: no DTB, using K1 ether0 defaults\n");
	} else {
		dprint("smte: no DTB entry for ether%d\n", edev->ctlrno);
		return -1;
	}

	/*
	 * plic.c's intrinit (driven by soc.plic from socidentify/DTB) has
	 * already masked every PLIC source, so we don't need to defang the
	 * prior stage's routing here.  intrenable() below opts us back in
	 * for this card's IRQ once the DMA descriptors are live.
	 */

	ct->regs = (ulong*)vmap(base, size);
	ct->apmu = (ulong*)vmap(apmubase, APMU_SIZE);
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
	 * Without DTB values keep the old defaults (~70°), which are known
	 * to work on the BPI-F3's first port.
	 */
	ct->rxdelay = SMTE_DEFAULT_RXDELAY;
	ct->txdelay = SMTE_DEFAULT_TXDELAY;
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
	 * On K1 the hardware reset value is uninitialized junk
	 * (e.g. fe:fe:fe:..) so a bootarg-supplied address must not
	 * be clobbered.
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
			hi = csr32r(ct->regs, MAC_ADDR1_HI);
			me = csr32r(ct->regs, MAC_ADDR1_ME);
			lo = csr32r(ct->regs, MAC_ADDR1_LO);
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
		/*
		 * 1ms polling interval (not 1000/HZ = 10ms) because RX
		 * completion on the K1 is currently polled rather than
		 * interrupt-driven (RX_TRANSFER_DONE in DMA_INTR_ENABLE
		 * breaks DHCP — see ethproc above).  The clock subsystem
		 * supports sub-tick intervals via stimecmp; the only cost
		 * is ~1000 timer interrupts per second instead of 100,
		 * which is well under 1% of CPU on a 1.6GHz K1.
		 * Bounded ping latency: ~1ms instead of ~10ms.
		 */
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
