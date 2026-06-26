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
 *   - intrenable not yet available: addclock0link used for polling
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

/* Physical addresses */
enum {
	EMAC0_PHYS	= 0xcac80000UL,
	EMAC0_SIZE	= 0x420,
	EMAC0_IRQ	= 141,

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

	DMA_STATUS_IRQ				= 0x0008,	/* W1C */
	DMA_STATUS_IRQ_TX_TRANSFER_DONE		= (1U << 0),
	DMA_STATUS_IRQ_RX_TRANSFER_DONE	= (1U << 4),
	DMA_STATUS_IRQ_RX_DES_UNAVAILABLE	= (1U << 5),
	DMA_STATUS_IRQ_RX_DMA_STOPPED		= (1U << 6),
	DMA_STATUS_IRQ_RX_MISSED_FRAME		= (1U << 7),

	DMA_INTR_ENABLE				= 0x000c,
	DMA_INTR_ENABLE_TX_TRANSFER_DONE	= (1U << 0),
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

	MAC_INTR_ENABLE				= 0x01e4,
};

/* APMU registers (byte offset) */
enum {
	APMU_EMAC_CLK_RST_CTRL			= 0x0000,
	APMU_EMAC_AXI_MST_ID			= (1U << 13),

	APMU_EMAC_RGMII_DLINE			= 0x0004,
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
	MII_GBSR	= 10,

	BMCR_ANENABLE	= (1U << 12),
	BMCR_ANRESTART	= (1U << 9),

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
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	ulong	*regs;
	ulong	*apmu;
	u32int	*rxr, *txr;
	Block	**rxs, **txs;
	int	rxprodi, rxconsi, txi;
	u32int	rxdelay, txdelay;
	int	attach;
	Lock	txlock;
};

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
	print("smte: PHY addr=%d oui=0x%06ux model=0x%02ux rev=0x%01ux\n",
		PHY_ADDR,
		((u32int)id1 << 6) | (id2 >> 10),
		(id2 >> 4) & 0x3f,
		id2 & 0xf);

	print("smte: PHY starting autonegotiation\n");
	mdwrite(c, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	for(;;){
		bmsr = mdread(c, MII_BMSR);
		if((bmsr & BMSR_LSTATUS) == 0){
			print("smte: link down (BMSR=0x%04ux)\n", bmsr);
			ethersetlink(edev, 0);
			csr32w(c->regs, MAC_TRANSMIT_CTRL, 0);
			csr32w(c->regs, MAC_RECEIVE_CTRL, 0);
			print("smte: MAC TX/RX disabled, waiting for link\n");
			while((mdread(c, MII_BMSR) & BMSR_LSTATUS) == 0)
				tsleep(&up->sleep, return0, nil, Linkdelay);
			print("smte: physical link detected\n");
		}

		print("smte: waiting for autoneg (BMSR=0x%04ux)\n", mdread(c, MII_BMSR));
		while((mdread(c, MII_BMSR) & BMSR_ANEGCOMPLETE) == 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);

		lpa  = mdread(c, MII_ANLPAR);
		gbsr = mdread(c, MII_GBSR);
		print("smte: autoneg complete ANLPAR=0x%04ux GBSR=0x%04ux\n", lpa, gbsr);

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

		print("smte: link up %dMbps %s\n", speed, dup);
		print("smte:   MAC_GLOBAL_CTRL=0x%08ux\n", ctrl);
		print("smte:   MAC_TX_CTRL=0x%08ux MAC_RX_CTRL=0x%08ux\n",
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

	/* always leave one free slot between prod and cons to distinguish
	 * a full ring from an empty one */
	while(((c->rxprodi + 1) & (RXRING - 1)) != c->rxconsi){
		i  = c->rxprodi;
		bp = iallocb(Rbsz);
		if(bp == nil){
			print("smte: replenish: out of memory (prod=%d cons=%d)\n",
				c->rxprodi, c->rxconsi);
			return -1;
		}
		c->rxs[i] = bp;
		r  = &c->rxr[4 * i];
		pa = PADDR(bp->rp);		/* physical address for DMA */
		if(i < 4)
        	print("RX buf[%d] pa=%#llux\n", i, pa);
		r[0] = 0;
		r[1] = Rbsz & RX_DESC1_SIZE1_MASK;
		if(i == RXRING - 1)
			r[1] |= RX_DESC1_END_RING;
		r[2] = (u32int)pa;
		r[3] = 0;
		coherence();
		r[0] = RX_DESC0_OWN;
		coherence();
		c->rxprodi = (c->rxprodi + 1) & (RXRING - 1);
	}
	csr32w(c->regs, DMA_RECEIVE_POLL_DEMAND, 1);
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

	static int once;

    if(once++ < 10)
        print("ethrx called\n");

	c = edev->ctlr;
	while((r = &c->rxr[4 * c->rxconsi]), (r[0] & RX_DESC0_OWN) == 0){
		s   = r[0];
		len = s & RX_DESC0_FRAME_PACKET_LENGTH_MASK;
		bp  = c->rxs[c->rxconsi];
		c->rxs[c->rxconsi] = nil;

		if(len < 4 || s & (RX_DESC0_FRAME_RUNT   | RX_DESC0_FRAME_CRC_ERR |
		                    RX_DESC0_FRAME_MAX_LEN_ERR | RX_DESC0_FRAME_JABBER_ERR |
		                    RX_DESC0_FRAME_LENGTH_ERR)){
			print("smte: RX error slot=%d desc0=0x%08ux len=%d\n",
				c->rxconsi, s, len);
			freeb(bp);
		} else {
			bp->wp = bp->rp + len - 4;
			etheriq(edev, bp);
		}
		c->rxconsi = (c->rxconsi + 1) & (RXRING - 1);
		replenish(c);
	}
}

/* ethtx --------------------------------------------------------------------- */

static void
ethtx(Ether *edev)
{
	Ctlr *c;
	u32int *r;
	Block *bp;
	uintptr pa;

	static int once;

    if(once++ < 10)
        print("ethtx called\n");

	c = edev->ctlr;
	ilock(&c->txlock);
	for(;;){
		r = &c->txr[4 * c->txi];
		if(r[0] & TX_DESC0_OWN){
			print("smte: TX ring full (txi=%d)\n", c->txi);
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
		r[1] = (BLEN(bp) & TX_DESC1_SIZE1_MASK)
			| TX_DESC1_FIRST_SEGMENT | TX_DESC1_LAST_SEGMENT
			| TX_DESC1_INTERRUPT_ON_COMPLETION;
		if(c->txi == TXRING - 1)
			r[1] |= TX_DESC1_END_RING;
		r[2] = (u32int)pa;
		r[3] = 0;
		coherence();
		r[0] = TX_DESC0_OWN;
		coherence();
		csr32w(c->regs, DMA_TRANSMIT_POLL_DEMAND, 1);
		c->txi = (c->txi + 1) & (TXRING - 1);
	}
	iunlock(&c->txlock);
}

/* IRQ / polling -------------------------------------------------------------- */

static void
ethirq(Ureg *, void *arg)
{
	Ether *edev;
	Ctlr *c;
	u32int fl;

	edev = arg;
	c    = edev->ctlr;
	fl   = csr32r(c->regs, DMA_STATUS_IRQ);
	if(fl == 0)
		return;

	csr32w(c->regs, DMA_STATUS_IRQ, fl);	/* W1C: clear handled bits */

	if(fl & DMA_STATUS_IRQ_RX_MISSED_FRAME)
		print("smte: WARNING DMA RX missed frames "
			"(status=0x%08ux rxring prod=%d cons=%d)\n",
			fl, c->rxprodi, c->rxconsi);
	if(fl & DMA_STATUS_IRQ_RX_DMA_STOPPED){
		print("smte: WARNING DMA RX stopped, restarting\n");
		csr32w(c->regs, DMA_RECEIVE_POLL_DEMAND, 1);
	}
	if(fl & (DMA_STATUS_IRQ_RX_TRANSFER_DONE |
	         DMA_STATUS_IRQ_RX_DES_UNAVAILABLE |
	         DMA_STATUS_IRQ_RX_MISSED_FRAME))
		ethrx(edev);
	if(fl & DMA_STATUS_IRQ_TX_TRANSFER_DONE)
		ethtx(edev);
}

static Ether *thisether;

static void
etherclock(void)
{
	ethirq(nil, thisether);
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
	print("smte: attach ok, starting PHY kproc\n");
	kproc("smteproc", ethproc, edev);
}

/* ethinit -------------------------------------------------------------------- */

static int
ethinit(Ether *edev)
{
	Ctlr *c;
	u32int v;

	c = edev->ctlr;

	/* stop everything */
	csr32w(c->regs, MAC_INTR_ENABLE,   0);
	csr32w(c->regs, DMA_INTR_ENABLE,   0);
	csr32w(c->regs, MAC_TRANSMIT_CTRL, 0);
	csr32w(c->regs, MAC_RECEIVE_CTRL,  0);
	csr32w(c->regs, DMA_CTRL,          0);
	print("smte: MAC/DMA stopped\n");

	/* APMU: enable AXI master ID */
	v = csr32r(c->apmu, APMU_EMAC_CLK_RST_CTRL) | APMU_EMAC_AXI_MST_ID;
	csr32w(c->apmu, APMU_EMAC_CLK_RST_CTRL, v);
	print("smte: APMU CLK_RST=0x%08ux\n",
		csr32r(c->apmu, APMU_EMAC_CLK_RST_CTRL));

	/* APMU: RGMII delay lines */
	v = APMU_EMAC_RGMII_DLINE_RX_EN   | APMU_EMAC_RGMII_DLINE_TX_EN
	  | APMU_EMAC_RGMII_DLINE_RX_STEP_15P6 | APMU_EMAC_RGMII_DLINE_TX_STEP_15P6
	  | c->rxdelay << APMU_EMAC_RGMII_DLINE_RX_DELAY_SHIFT
	  | c->txdelay << APMU_EMAC_RGMII_DLINE_TX_DELAY_SHIFT;
	csr32w(c->apmu, APMU_EMAC_RGMII_DLINE, v);
	print("smte: APMU DLINE=0x%08ux (rx=%d tx=%d steps x 15.6ps)\n",
		v, c->rxdelay, c->txdelay);

	/* MAC address */
	csr32w(c->regs, MAC_ADDR1_HI, edev->ea[1]<<8 | edev->ea[0]);
	csr32w(c->regs, MAC_ADDR1_ME, edev->ea[3]<<8 | edev->ea[2]);
	csr32w(c->regs, MAC_ADDR1_LO, edev->ea[5]<<8 | edev->ea[4]);
	csr32w(c->regs, MAC_ADDR_CTRL, MAC_ADDR_CTRL_MAC_ADDR1_ENABLE);
	print("smte: MAC %02X:%02X:%02X:%02X:%02X:%02X programmed\n",
		edev->ea[0], edev->ea[1], edev->ea[2],
		edev->ea[3], edev->ea[4], edev->ea[5]);

	/* FIFO thresholds and frame size limits */
	csr32w(c->regs, MAC_TRANSMIT_FIFO_ALMOST_FULL,        0x1f8);
	csr32w(c->regs, MAC_TRANSMIT_PACKET_START_THRESHOLD,  1518);
	csr32w(c->regs, MAC_RECEIVE_PACKET_START_THRESHOLD,   12);
	csr32w(c->regs, MAC_MAXIMUM_FRAME_SIZE,               1514);
	csr32w(c->regs, MAC_TRANSMIT_JABBER_SIZE,             1514+18);
	csr32w(c->regs, MAC_RECEIVE_JABBER_SIZE,              1514+18);
	print("smte: FIFO thresholds and frame sizes programmed\n");

	/* RX interrupt coalescing: up to 64 frames or ~600 us */
	v = (64 << 0) | ((600*312) << 8) | DMA_RECEIVE_IRQ_MITIGATION_ENABLE;
	csr32w(c->regs, DMA_RECEIVE_IRQ_MITIGATION, v);

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
	print("smte: DMA reset, DMA_CONFIG=0x%08ux (64-bit mode enabled for buffers)\n",
		csr32r(c->regs, DMA_CONFIG));

	/* IMP - k1 uses 32-bit addresses for DMA access */
	c->rxr = xspanalloc(sizeof(u32int) * 4 * RXRING, 16, 0);
	c->txr = xspanalloc(sizeof(u32int) * 4 * TXRING, 16, 0);
	memset(c->rxr, 0, 4 * RXRING * sizeof(u32int));
	memset(c->txr, 0, 4 * TXRING * sizeof(u32int));
	
	print("rxring bytes=%d\n",
      4 * RXRING * sizeof(u32int));

	print("txring bytes=%d\n",
      4 * TXRING * sizeof(u32int));

	c->rxs = mallocz(sizeof(Block*) * RXRING, 1);
	c->txs = mallocz(sizeof(Block*) * TXRING, 1);
	if(c->rxs == nil || c->txs == nil){
		print("smte: ERROR mallocz failed\n");
		return -1;
	}
	print("smte: anillos rxr=%p txr=%p (%d/%d slots, %d bytes/buf)\n",
		c->rxr, c->txr, RXRING, TXRING, Rbsz);

	/* fill the entire RX ring (condition: prod+1 != cons)
	 * with rxcons=0 and RXRING=512, 511 slots are filled (1 kept free) */
	c->rxprodi = 0;
	c->rxconsi = 0;
	replenish(c);
	//wbflush();	/* ensure descriptors are in DRAM before starting DMA */
	print("smte: RX ring full (prod=%d cons=%d free=%d)\n",
		c->rxprodi, c->rxconsi,
		(c->rxprodi - c->rxconsi + RXRING) & (RXRING-1));

	c->txi = 0;

	uintptr rxpa, txpa;
	rxpa = PADDR(c->rxr);
	txpa = PADDR(c->txr);
	print("RX phys=%#llux TX phys=%#llux\n", rxpa, txpa);
	print("DMA RX low=%#ux\n", (u32int)rxpa);
	print("DMA TX low=%#ux\n", (u32int)txpa);
	print("smte: DMA base RX phys=0x%08lux TX phys=0x%08lux\n",
		(ulong)rxpa, (ulong)txpa);
	csr32w(c->regs, DMA_RECEIVE_BASE_ADDRESS, (u32int)rxpa);
	csr32w(c->regs, DMA_TRANSMIT_BASE_ADDRESS, (u32int)txpa);

	/* verify the hardware accepted the addresses */
	print("smte: DMA_RX_BASE read=0x%08ux DMA_TX_BASE read=0x%08ux\n",
		csr32r(c->regs, DMA_RECEIVE_BASE_ADDRESS),
		csr32r(c->regs, DMA_TRANSMIT_BASE_ADDRESS));

	/* clear and enable DMA interrupts */
	csr32w(c->regs, DMA_STATUS_IRQ, ~0U);
	csr32w(c->regs, MAC_INTR_ENABLE, 0);
	v = DMA_INTR_ENABLE_TX_TRANSFER_DONE  |
	    DMA_INTR_ENABLE_RX_TRANSFER_DONE  |
	    DMA_INTR_ENABLE_RX_DES_UNAVAILABLE|
	    DMA_INTR_ENABLE_RX_MISSED_FRAME;
	csr32w(c->regs, DMA_INTR_ENABLE, v);
	print("smte: DMA_INTR_ENABLE=0x%08ux\n", v);

	/* start DMA (MAC TX/RX are enabled in ethproc after autoneg) */
	v = DMA_CTRL_START_STOP_TX_DMA | DMA_CTRL_START_STOP_RX_DMA;
	csr32w(c->regs, DMA_CTRL, v);
	print("smte: DMA started DMA_CTRL=0x%08ux\n",
		csr32r(c->regs, DMA_CTRL));

	/* dump first descriptor to confirm hardware can see it */
	print("smte: desc[0] d0=0x%08ux d1=0x%08ux d2=0x%08ux d3=0x%08ux\n",
		c->rxr[0], c->rxr[1], c->rxr[2], c->rxr[3]);
	/* d0 must be 0x80000000 (RX_DESC0_OWN=1) if writes reached DRAM */

	return 0;
}

/* SoC fingerprint ------------------------------------------------------------ */

static int
checksoc(void)
{
	ulong *sdh;
	u32int val, vendor;

	print("smte: probing SoC (SDH1 @ 0x%08ux)\n", SDH1_PHYS);
	sdh = (ulong*)vmap(SDH1_PHYS, SDH1_SIZE);
	if(sdh == nil){
		print("smte: ERROR vmap SDH failed\n");
		return 0;
	}
	val    = csr32r(sdh, SDHC_VID_PID);
	vendor = val & 0xFFFFF;
	vunmap(sdh, SDH1_SIZE);

	print("smte: SDHC_VID_PID=0x%08ux vendor=0x%05ux\n", val, vendor);
	if(vendor == K1_VENDOR_ID){
		print("smte: SoC SpacemiT K1 confirmed\n");
		return 1;
	}
	print("smte: unknown vendor 0x%05ux (expected 0x%05ux)\n",
		vendor, K1_VENDOR_ID);
	return 0;
}

/* Plug-and-play -------------------------------------------------------------- */

static int
etherpnp(Ether *edev)
{
	static Ctlr ct;

	print("smte: etherpnp\n");

	if(ct.regs != nil){
		print("smte: already registered\n");
		return -1;
	}

	if(!checksoc())
		return -1;

	ct.regs = (ulong*)vmap(EMAC0_PHYS, EMAC0_SIZE);
	ct.apmu = (ulong*)vmap(APMU_PHYS,  APMU_SIZE);
	if(ct.regs == nil || ct.apmu == nil){
		print("smte: ERROR vmap failed regs=%p apmu=%p\n", ct.regs, ct.apmu);
		return -1;
	}
	print("smte: regs=%p (phys=0x%08ux) apmu=%p (phys=0x%08ux)\n",
		ct.regs, EMAC0_PHYS, ct.apmu, APMU_PHYS);

	ct.rxdelay = SMTE_DEFAULT_RXDELAY;
	ct.txdelay = SMTE_DEFAULT_TXDELAY;

	edev->ctlr     = &ct;
	edev->port     = EMAC0_PHYS;
	edev->irq      = -1;		/* intrenable not yet available */
	edev->attach   = ethattach;
	edev->transmit = ethtx;
	edev->arg      = edev;
	edev->mbps     = 1000;

	/* read MAC address from hardware */
	{
		u32int hi, me, lo;
		hi = csr32r(ct.regs, MAC_ADDR1_HI);
		me = csr32r(ct.regs, MAC_ADDR1_ME);
		lo = csr32r(ct.regs, MAC_ADDR1_LO);
		edev->ea[0] = hi & 0xff;  edev->ea[1] = (hi>>8) & 0xff;
		edev->ea[2] = me & 0xff;  edev->ea[3] = (me>>8) & 0xff;
		edev->ea[4] = lo & 0xff;  edev->ea[5] = (lo>>8) & 0xff;
		print("smte: MAC read from hardware: %02X:%02X:%02X:%02X:%02X:%02X\n",
			edev->ea[0], edev->ea[1], edev->ea[2],
			edev->ea[3], edev->ea[4], edev->ea[5]);
	}

	if(ethinit(edev) < 0){
		print("smte: ERROR ethinit failed\n");
		edev->ctlr = nil;
		return -1;
	}

	/*
	 * When intrenable becomes available, replace addclock0link with:
	 *   intrenable(EMAC0_IRQ, ethirq, edev, LEVEL, edev->name);
	 */
	thisether = edev;
	addclock0link(etherclock, 1000/HZ);
	print("smte: polling via addclock0link (HZ=%d tick=%dms)\n",
		HZ, 1000/HZ);

	print("smte: driver ready\n");
	return 0;
}

/* Driver registration -------------------------------------------------------- */

void
ethersmtelink(void)
{
	print("smte: registering driver\n");
	addethercard("smte", etherpnp);
}
