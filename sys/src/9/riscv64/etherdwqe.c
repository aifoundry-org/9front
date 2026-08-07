/*
 * Driver for the SpacemiT K3 Ethernet controller (GMAC0/dwqe)
 *
 * Based on:
 *   9front ethersmte.c (K1 EMAC driver) - overall 9front driver architecture
 *   Linux stmmac driver - dwmac4/5 register + descriptor layout reference
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
	GMAC0_PHYS	= 0xcac80000UL,
	GMAC0_SIZE	= 0x2000,
	GMAC0_IRQ	= 131,

	APMU_PHYS	= 0xd4282800UL,
	APMU_SIZE	= 0x400,

	DWQE_DEFAULT_RXDELAY	= 100,
	DWQE_DEFAULT_TXDELAY	= 100,
};

/* GMAC (MAC-level) registers -- verified against dwmac4.h */
enum {
	GMAC_CONFIG			= 0x0000,
	GMAC_CONFIG_ARPEN		= (1U << 31),
	GMAC_CONFIG_IPC			= (1U << 27),	/* rx checksum offload */
	GMAC_CONFIG_ACS			= (1U << 20),	/* strip pad/fcs */
	GMAC_CONFIG_BE			= (1U << 18),	/* back-off enable */
	GMAC_CONFIG_JD			= (1U << 17),	/* jabber disable */
	GMAC_CONFIG_JE			= (1U << 16),	/* jumbo enable */
	GMAC_CONFIG_PS			= (1U << 15),	/* port select: 1=10/100 MII, 0=1000 GMII/RGMII */
	GMAC_CONFIG_FES			= (1U << 14),	/* speed: with PS, selects 100 vs 10 */
	GMAC_CONFIG_DM			= (1U << 13),	/* duplex mode: 1=full */
	GMAC_CONFIG_DCRS		= (1U << 9),
	GMAC_CONFIG_TE			= (1U << 1),	/* transmitter enable */
	GMAC_CONFIG_RE			= (1U << 0),	/* receiver enable */

	GMAC_RXQ_CTRL0			= 0x00a0,
	GMAC_RXQCTRL_Q0_DCB_EN		= (1U << 1),	/* BIT((queue*2)+1) for queue=0 */

	GMAC_PACKET_FILTER		= 0x0008,
	GMAC_PACKET_FILTER_PR		= (1U << 0),	/* promiscuous */
	GMAC_PACKET_FILTER_HMC		= (1U << 2),	/* hash multicast */
	GMAC_PACKET_FILTER_PM		= (1U << 4),	/* pass all multicast */
	GMAC_PACKET_FILTER_HPF		= (1U << 10),	/* hash or perfect filter */
	GMAC_PACKET_FILTER_RA		= (1U << 31),	/* receive all */

	GMAC_INT_STATUS			= 0x00b0,
	GMAC_INT_EN			= 0x00b4,

	GMAC_MDIO_ADDR			= 0x0200,
	GMAC_MDIO_DATA			= 0x0204,

	GMAC_HW_FEATURE0		= 0x011c,
	GMAC_HW_FEATURE1		= 0x0120,

	GMAC_ADDR0_HIGH			= 0x0300,
	GMAC_ADDR0_LOW			= 0x0304,
};

/* GMAC_MDIO_ADDR fields -- verified against stmmac_mdio.c + dwmac4_core.c */
enum {
	MII_ADDR_GBUSY			= (1U << 0),
	MII_GMAC4_GOC_SHIFT		= 2,
	MII_GMAC4_WRITE			= (1U << MII_GMAC4_GOC_SHIFT),		/* GOC=01 */
	MII_GMAC4_READ			= (3U << MII_GMAC4_GOC_SHIFT),		/* GOC=11 */
	MII_ADDR_PA_SHIFT		= 21,		/* bits [25:21] */
	MII_ADDR_GR_SHIFT		= 16,		/* bits [20:16] */
	MII_ADDR_CR_SHIFT		= 8,		/* bits [11:8], clock range */
	MII_ADDR_CR_VAL			= 6,		/* matches ACLK->EMAC0_bus_clk chain (307MHz/div1) */
	MII_DATA_GD_MASK		= 0xffff,
};

/* DMA (global) registers -- verified against dwmac4_dma.h */
enum {
	DMA_BUS_MODE			= 0x1000,
	DMA_BUS_MODE_SFT_RESET		= (1U << 0),

	DMA_SYS_BUS_MODE		= 0x1004,
	DMA_SYS_BUS_MB			= (1U << 14),
	DMA_SYS_BUS_EAME		= (1U << 11),	/* Enhanced Address Mode: enables all *_HI regs + des1 */
	DMA_SYS_BUS_FB			= (1U << 0),	/* Fixed Burst */
};

/* DMA per-channel registers (channel 0 only -- dwqe uses 1 queue) */
enum {
	DMA_CHAN_BASE			= 0x1100,

	DMA_CHAN_CONTROL		= 0x00,
	DMA_CHAN_TX_CONTROL		= 0x04,
	DMA_CONTROL_ST			= (1U << 0),

	DMA_CHAN_RX_CONTROL		= 0x08,
	DMA_CONTROL_SR			= (1U << 0),
	DMA_RBSZ_SHIFT			= 1,		/* bits [14:1], raw byte count << 1 */
	DMA_RBSZ_MASK			= 0x7ffe,

	DMA_CHAN_TX_BASE_ADDR_HI	= 0x10,		/* bits [63:32] of ring base */
	DMA_CHAN_TX_BASE_ADDR		= 0x14,		/* bits [31:0]  of ring base */
	DMA_CHAN_RX_BASE_ADDR_HI	= 0x18,		/* bits [63:32] of ring base */
	DMA_CHAN_RX_BASE_ADDR		= 0x1c,		/* bits [31:0]  of ring base */
	/*
	 * Tail pointer has no _HI counterpart -- the ring's upper 32 bits
	 * come from *_BASE_ADDR_HI above and apply to the whole ring
	 */
	DMA_CHAN_TX_END_ADDR		= 0x20,		/* tail pointer, 32-bit */
	DMA_CHAN_RX_END_ADDR		= 0x28,		/* tail pointer, 32-bit */
	DMA_CHAN_TX_RING_LEN		= 0x2c,		/* lenght N-1 */
	DMA_CHAN_RX_RING_LEN		= 0x30,		/* lenght N-1 */

	DMA_CHAN_INTR_ENA		= 0x34,
	DMA_CHAN_INTR_ENA_NIE		= (1U << 16),
	DMA_CHAN_INTR_ENA_AIE		= (1U << 15),
	DMA_CHAN_INTR_ENA_FBE		= (1U << 12),
	DMA_CHAN_INTR_ENA_RBU		= (1U << 7),
	DMA_CHAN_INTR_ENA_RIE		= (1U << 6),
	DMA_CHAN_INTR_ENA_TIE		= (1U << 0),

	DMA_CHAN_STATUS			= 0x60,
	DMA_CHAN_STATUS_NIS		= (1U << 15),
	DMA_CHAN_STATUS_AIS		= (1U << 14),
	DMA_CHAN_STATUS_FBE		= (1U << 12),
	DMA_CHAN_STATUS_RBU		= (1U << 7),
	DMA_CHAN_STATUS_RI		= (1U << 6),
	DMA_CHAN_STATUS_TI		= (1U << 0),
};

/* MTL (queue) registers -- verified against dwmac4.h */
enum {
	MTL_CHAN_BASE			= 0x0d00,
	MTL_CHAN_TX_OP_MODE		= 0x00,
	MTL_OP_MODE_TXQEN		= (1U << 3),
	MTL_OP_MODE_TSF			= (1U << 1),	/* store-and-forward */
	MTL_OP_MODE_TQS_MASK		= 0x01ff0000,	/* bits [24:16] */

	MTL_CHAN_RX_OP_MODE		= 0x30,
	MTL_OP_MODE_RSF			= (1U << 5),
	MTL_OP_MODE_RQS_MASK		= 0x3ff00000,	/* bits [29:20] */

	GMAC_HW_TXFIFOSIZE		= 0x000007c0,	/* HW_FEATURE1 bits [10:6] */
	GMAC_HW_RXFIFOSIZE		= 0x0000001f,	/* HW_FEATURE1 bits [4:0]  */
};

/* APMU registers  */
enum {
	EMAC_BUS_EN		= (1U << 0),
	EMAC_BUS_RST	= (1U << 1),
	EMAC_PHY_SELECT_SHIFT	= 3,		/* bits [4:3] */
	EMAC_PHY_SELECT_RMII	= 0,		/* 2'b00 */
	EMAC_PHY_SELECT_RGMII	= 1,		/* 2'b01 */
	EMAC_PHY_SELECT_MII_GMII	= 2,		/* 2'b10 */
	EMAC_AXI_MST_ID		= (1U << 13),
	/*
	 * Official name is EMAC0_1588_CLK_MUX: 0 = 24M, 1 = PLL 125/25/2.5MHz
	 * matching the current speed (1000M/100M/10M) -- a clock-source mux
	 * for the PTP/1588 timestamp counter
	 */
	EMAC_1588_CLK_MUX		= (1U << 15),

	/*
	 * Previously left at their power-on-reset defaults; now set
	 * explicitly so behavior doesn't depend on what a given board's
	 * firmware happened to leave here
	 */
	EMAC0_CLK_REF_GATE		= (1U << 14),	/* 1 = tx 25M refclk disable, 0 = enable */
	EMAC0_PHY_INTR_EN		= (1U << 12),	/* 1 = PHY PMT intr mask enable */
	EMAC0_LPI_INTR_EN		= (1U << 9),	/* 1 = LPI intr mask enable */
	EMAC0_RGMII_TXC_SRC_SEL		= (1U << 8),	/* RGMII only: 1 = TX clock from SoC, 0 = from RX clock */
	EMAC_RX_REFCLK_PHASE_SEL	= (1U << 7),	/* 1 = inverted, 0 = normal */
	EMAC_RMII_TX_REFCLK_PHASE_SEL	= (1U << 6),	/* RMII only, don't-care in RGMII */

	EMAC_RGMII_RXC_DLINE_PU		= (1U << 0),
	EMAC_RGMII_RXC_DLINE_STEP_15P6	= (0U << 4),
	EMAC_RGMII_RXC_DLINE_ADJ_SHIFT	= 8,
	EMAC_RGMII_TXC_DLINE_PU		= (1U << 16),
	EMAC_RGMII_TXC_DLINE_STEP_15P6	= (0U << 20),
	EMAC_RGMII_TXC_DLINE_ADJ_SHIFT	= 24,
};

/*
 * Normal descriptors, "Read Format" while owned by the DMA engine,
 * "Write-Back Format" once it hands them back -- both are the same
 * 4 x u32int words, reused as r[4*i + 0..3].  Bit values verified
 * against dwmac4_descs.h.
 */
enum {
	/* TDES2 (read format) */
	TDES2_BUFFER1_SIZE_MASK		= 0x3fff,	/* [13:0] */
	TDES2_INTERRUPT_ON_COMPLETION	= (1U << 31),

	/* TDES3 (read format) */
	TDES3_PACKET_SIZE_MASK		= 0x7fff,	/* [14:0] */
	TDES3_LAST_DESCRIPTOR		= (1U << 28),
	TDES3_FIRST_DESCRIPTOR		= (1U << 29),
	TDES3_OWN			= (1U << 31),

	/* TDES3 (write-back format) */
	TDES3_ERROR_SUMMARY		= (1U << 15),

	/* RDES3 (read format) */
	RDES3_BUFFER1_VALID_ADDR	= (1U << 24),
	RDES3_INT_ON_COMPLETION_EN	= (1U << 30),
	RDES3_OWN			= (1U << 31),

	/* RDES3 (write-back format) */
	RDES3_PACKET_SIZE_MASK		= 0x7fff,	/* [14:0] */
	RDES3_ERROR_SUMMARY		= (1U << 15),
	RDES3_LAST_DESCRIPTOR		= (1U << 28),
	RDES3_CONTEXT_DESCRIPTOR	= (1U << 30),
};

/* Standard MII registers (same PHY, RTL8211F, as ethersmte.c) */
enum {
	MII_BMCR	= 0,
	MII_BMSR	= 1,
	MII_PHYSID1	= 2,
	MII_PHYSID2	= 3,
	MII_ANLPAR	= 5,
	MII_GBCR	= 9,
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
	Txirqevery = 64,
	MAXDWQE	= 1,		/* GMAC0 only for now; GMAC1/2 follow the same pattern */
};

#define DWQE_CBSZ 64	/* X60/C908 D-cache line size */

/* same non-coherent-DMA cache maintenance as ethersmte.c */
static void
dwqecacheflush(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(DWQE_CBSZ - 1);
	last = ((uintptr)va + len + DWQE_CBSZ - 1) & ~(uintptr)(DWQE_CBSZ - 1);
	for(; a < last; a += DWQE_CBSZ)
		cboflush((void*)a);
	coherence();
}

static void
dwqecacheinval(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(DWQE_CBSZ - 1);
	last = ((uintptr)va + len + DWQE_CBSZ - 1) & ~(uintptr)(DWQE_CBSZ - 1);
	for(; a < last; a += DWQE_CBSZ)
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

/* MDIO -- GMAC4-style GOC field, verified against stmmac_mdio.c */

static void
mdwrite(Ctlr *c, int reg, u16int v)
{
	u32int addr;

	while(csr32r(c->regs, GMAC_MDIO_ADDR) & MII_ADDR_GBUSY)
		tsleep(&up->sleep, return0, nil, 1);
	csr32w(c->regs, GMAC_MDIO_DATA, v);
	addr = (PHY_ADDR & 0x1f) << MII_ADDR_PA_SHIFT
		| (reg & 0x1f) << MII_ADDR_GR_SHIFT
		| MII_ADDR_CR_VAL << MII_ADDR_CR_SHIFT
		| MII_GMAC4_WRITE
		| MII_ADDR_GBUSY;
	csr32w(c->regs, GMAC_MDIO_ADDR, addr);
	while(csr32r(c->regs, GMAC_MDIO_ADDR) & MII_ADDR_GBUSY)
		tsleep(&up->sleep, return0, nil, 1);
}

static u16int
mdread(Ctlr *c, int reg)
{
	u32int addr;

	while(csr32r(c->regs, GMAC_MDIO_ADDR) & MII_ADDR_GBUSY)
		tsleep(&up->sleep, return0, nil, 1);
	addr = (PHY_ADDR & 0x1f) << MII_ADDR_PA_SHIFT
		| (reg & 0x1f) << MII_ADDR_GR_SHIFT
		| MII_ADDR_CR_VAL << MII_ADDR_CR_SHIFT
		| MII_GMAC4_READ
		| MII_ADDR_GBUSY;
	csr32w(c->regs, GMAC_MDIO_ADDR, addr);
	while(csr32r(c->regs, GMAC_MDIO_ADDR) & MII_ADDR_GBUSY)
		tsleep(&up->sleep, return0, nil, 1);
	return csr32r(c->regs, GMAC_MDIO_DATA) & MII_DATA_GD_MASK;
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
		v |= GMAC_PACKET_FILTER_PR;
	else if(allmulti)
		v |= GMAC_PACKET_FILTER_PM;
	csr32w(c->regs, GMAC_PACKET_FILTER, v);
	c->promisc = promisc;
	c->allmulti = allmulti;
	dprint("dwqe: GMAC_PACKET_FILTER=%#08ux (promisc=%d allmulti=%d)\n",
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

	v = csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_CONTROL);
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_CONTROL, v & ~DMA_CONTROL_ST);
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
	coherence();	/* txr is NC memory now; no cachelines to flush, just order the zeroing */
	iunlock(&c->txlock);
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_BASE_ADDR, PADDR(c->txr));
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_END_ADDR, PADDR(c->txr));
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

	id1 = mdread(c, MII_PHYSID1);
	id2 = mdread(c, MII_PHYSID2);
	dprint("dwqe: PHY addr=%d oui=0x%06ux model=0x%02ux rev=0x%01ux\n",
		PHY_ADDR, ((u32int)id1 << 6) | (id2 >> 10), (id2 >> 4) & 0x3f, id2 & 0xf);

	mdwrite(c, MII_GBCR, GBCR_ADV1000FULL | GBCR_ADV1000HALF);
	mdwrite(c, MII_BMCR, BMCR_ANENABLE | BMCR_ANRESTART);

	for(;;){
		bmsr = mdread(c, MII_BMSR);
		if((bmsr & BMSR_LSTATUS) == 0){
			print("dwqe: %s link down (BMSR=0x%04ux)\n", edev->name, bmsr);
			ethersetlink(edev, 0);
			csr32w(c->regs, GMAC_CONFIG, csr32r(c->regs, GMAC_CONFIG)
				& ~(GMAC_CONFIG_TE | GMAC_CONFIG_RE));
			txreset(c);
			while((mdread(c, MII_BMSR) & BMSR_LSTATUS) == 0)
				tsleep(&up->sleep, return0, nil, Linkdelay);
		}

		while((mdread(c, MII_BMSR) & BMSR_ANEGCOMPLETE) == 0)
			tsleep(&up->sleep, return0, nil, Linkdelay);

		lpa  = mdread(c, MII_ANLPAR);
		gbsr = mdread(c, MII_GBSR);

		cfg = csr32r(c->regs, GMAC_CONFIG)
			& ~(GMAC_CONFIG_PS | GMAC_CONFIG_FES | GMAC_CONFIG_DM);
		dup = "HD";

		if(gbsr & GBSR_LP1000FULL){
			cfg |= GMAC_CONFIG_DM;		/* PS=0,FES=0 => 1000 */
			speed = 1000; dup = "FD";
		} else if(gbsr & GBSR_LP1000HALF){
			speed = 1000;
		} else if(lpa & LPA_100FULL){
			cfg |= GMAC_CONFIG_PS | GMAC_CONFIG_FES | GMAC_CONFIG_DM;
			speed = 100; dup = "FD";
		} else if(lpa & LPA_100HALF){
			cfg |= GMAC_CONFIG_PS | GMAC_CONFIG_FES;
			speed = 100;
		} else if(lpa & LPA_10FULL){
			cfg |= GMAC_CONFIG_PS | GMAC_CONFIG_DM;
			speed = 10; dup = "FD";
		} else {
			cfg |= GMAC_CONFIG_PS;
			speed = 10;
		}

		csr32w(c->regs, GMAC_CONFIG, cfg | GMAC_CONFIG_TE | GMAC_CONFIG_RE);

		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_CONTROL,
			csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_CONTROL) | DMA_CONTROL_ST);
		v = csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_CONTROL);
		v = (v & ~DMA_RBSZ_MASK) | ((Rbsz << DMA_RBSZ_SHIFT) & DMA_RBSZ_MASK);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_CONTROL, v | DMA_CONTROL_SR);
		c->dmarunning = 1;

		dprint("dwqe: readback GMAC_CONFIG=%#08ux RX_CONTROL=%#08ux TX_CONTROL=%#08ux "
			"RX_BASE=%#08ux RX_END=%#08ux RX_RING_LEN=%#08ux "
			"MTL_RX_OP=%#08ux PACKET_FILTER=%#08ux RXQ_CTRL0=%#08ux\n",
			csr32r(c->regs, GMAC_CONFIG),
			csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_CONTROL),
			csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_CONTROL),
			csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_BASE_ADDR),
			csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_END_ADDR),
			csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_RING_LEN),
			csr32r(c->regs, MTL_CHAN_BASE + MTL_CHAN_RX_OP_MODE),
			csr32r(c->regs, GMAC_PACKET_FILTER),
			csr32r(c->regs, GMAC_RXQ_CTRL0));

		ethrx(edev);
		ethtx(edev);

		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_STATUS, ~0U);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA,
			DMA_CHAN_INTR_ENA_NIE | DMA_CHAN_INTR_ENA_TIE
			| DMA_CHAN_INTR_ENA_RIE | DMA_CHAN_INTR_ENA_RBU
			| DMA_CHAN_INTR_ENA_AIE | DMA_CHAN_INTR_ENA_FBE);

		print("dwqe: %s link up %dMbps %s\n", edev->name, speed, dup);
		ethersetspeed(edev, speed);
		ethersetlink(edev, 1);

		while((mdread(c, MII_BMSR) & BMSR_LSTATUS) != 0)
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
	 * memory are also touched by ethrx()'s dequeue step
	 */
	ilock(&c->rxlock);

	/*
	 * Leave one slot permanently empty (classic circular-buffer
	 * capacity N-1): the RX tail-pointer register has no separate
	 * "full" indication, so head==tail must unambiguously mean
	 * "nothing available".
	 */
	while(c->rxs[c->rxprodi] == nil
	    && ((c->rxprodi + 1) & (RXRING - 1)) != c->rxconsi){
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
		r[3] = RDES3_OWN | RDES3_BUFFER1_VALID_ADDR
			| RDES3_INT_ON_COMPLETION_EN;	/* RDES3 */
		coherence();	/* r is NC memory now; order the descriptor write, nothing to flush */
		dwqecacheinval(bp->rp, Rbsz);	/* bp->rp is ordinary cacheable memory -- keep */
		c->rxprodi = (c->rxprodi + 1) & (RXRING - 1);
	}
	/*
	 * Always advance the tail pointer, even before DMA is running --
	 * writing it early is harmless (it's just a register the engine
	 * reads once SR is set), and it's what lets ethinit()'s initial
	 * fill actually be visible to the DMA engine the moment it starts.
	 */
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_END_ADDR,
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
		if(r[3] & RDES3_OWN){
			iunlock(&c->rxlock);
			break;
		}
		if(r[3] & RDES3_CONTEXT_DESCRIPTOR){
			/* shouldn't happen with rx_ic disabled context gen, skip */
			c->rxconsi = (c->rxconsi + 1) & (RXRING - 1);
			iunlock(&c->rxlock);
			continue;
		}
		s   = r[3];
		len = s & RDES3_PACKET_SIZE_MASK;
		bp  = c->rxs[c->rxconsi];
		c->rxs[c->rxconsi] = nil;
		c->rxconsi = (c->rxconsi + 1) & (RXRING - 1);
		iunlock(&c->rxlock);

		if(bp != nil && len > 0)
			dwqecacheinval(bp->rp, len);

		if(len < 4 || (s & RDES3_ERROR_SUMMARY) || !(s & RDES3_LAST_DESCRIPTOR)){
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
	p = seprint(p, e, "irq bits seen %#ux status %#ux intren %#ux\n",
		c->irqseen,
		csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_STATUS),
		csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA));
	p = seprint(p, e, "gmac config %#ux\n", csr32r(c->regs, GMAC_CONFIG));
	p = seprint(p, e, "filter %#ux promisc=%d allmulti=%d\n",
		csr32r(c->regs, GMAC_PACKET_FILTER), c->promisc, c->allmulti);
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
		if(r[3] & TDES3_OWN)
			break;
		freeb(c->txs[c->txcons]);
		c->txs[c->txcons] = nil;
		c->txdone++;
		c->txcons = (c->txcons + 1) & (TXRING - 1);
	}
	for(;;){
		r = &c->txr[4 * c->txi];	/* NC memory now; no invalidate needed */
		if(r[3] & TDES3_OWN){
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
		r[2] = BLEN(bp) & TDES2_BUFFER1_SIZE_MASK;
		if((c->txi & (Txirqevery - 1)) == 0)
			r[2] |= TDES2_INTERRUPT_ON_COMPLETION;
		coherence();
		r[3] = TDES3_FIRST_DESCRIPTOR | TDES3_LAST_DESCRIPTOR
			| (BLEN(bp) & TDES3_PACKET_SIZE_MASK) | TDES3_OWN;
		coherence();	/* r is NC memory now; order the descriptor write, nothing to flush */

		c->txi = (c->txi + 1) & (TXRING - 1);
		if(c->dmarunning)
			csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_END_ADDR,
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

	en = csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA);
	if(en != 0){
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA, 0);
		csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA);	/* drain posted write */
	}

	fl = csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_STATUS);
	if(fl != 0){
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_STATUS, fl);	/* W1C */
		if(fl & DMA_CHAN_STATUS_RBU){
			iprint("dwqe: %s RX buffer unavailable (status=%#08ux prod=%d cons=%d)\n",
				edev->name, fl, c->rxprodi, c->rxconsi);
			replenish(c);
		}
		if(fl & DMA_CHAN_STATUS_FBE)
			iprint("dwqe: %s fatal bus error (status=%#08ux)\n", edev->name, fl);
	}

	ethrx(edev);
	ethtx(edev);

	if(en != 0){
		coherence();
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA, en);
	}
}

static Ether *dwqeether[MAXDWQE];
static Ctlr   dwqectlr[MAXDWQE];
static int    dwqeclockregistered;

static void
etherserve(Ether *edev)
{
	Ctlr *c;
	u32int fl;

	c = edev->ctlr;
	fl = csr32r(c->regs, DMA_CHAN_BASE + DMA_CHAN_STATUS);
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

	for(i = 0; i < MAXDWQE; i++)
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

/* /net/etherN/clone/ handler ---------------------------------------------- */

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
	 * any MAC/DMA register. Skipping it stalls the hart
	 */
	{
		u32int origv = csr32r(c->apmu, c->apmuctrloff);

		/*
		 * Grab whatever real MAC firmware left in GMAC_ADDR0 *now*,
		 * before the sequence below forces an EMAC_BUS_RST 0->1
		 * transition -- that's a real reset of the MAC's internal
		 * register file (ADDR0 included). Only safe to read at all
		 * if RST is already 1 in origv  -- if it's 0, the register
		 * block isn't accessible yet and reading would hang the
		 * hart the same way skipping the bring-up entirely did.
		 */
		if((origv & EMAC_BUS_RST) != 0){
			static uchar zero[Eaddrlen];

			if(memcmp(edev->ea, zero, Eaddrlen) == 0){
				u32int hi, lo;

				csr32w(c->apmu, c->apmuctrloff, origv | EMAC_BUS_EN);
				microdelay(1000);
				lo = csr32r(c->regs, GMAC_ADDR0_LOW);
				hi = csr32r(c->regs, GMAC_ADDR0_HIGH);
				edev->ea[0] = lo & 0xff;        edev->ea[1] = (lo>>8) & 0xff;
				edev->ea[2] = (lo>>16) & 0xff;  edev->ea[3] = (lo>>24) & 0xff;
				edev->ea[4] = hi & 0xff;        edev->ea[5] = (hi>>8) & 0xff;
				dprint("dwqe: MAC read from hardware: %02X:%02X:%02X:%02X:%02X:%02X\n",
					edev->ea[0], edev->ea[1], edev->ea[2],
					edev->ea[3], edev->ea[4], edev->ea[5]);
			}
		}

		u32int preserve = origv & ~(EMAC_BUS_EN | EMAC_BUS_RST
			| EMAC_AXI_MST_ID | EMAC_1588_CLK_MUX
			| (3U << EMAC_PHY_SELECT_SHIFT)
			| EMAC0_CLK_REF_GATE | EMAC0_PHY_INTR_EN | EMAC0_LPI_INTR_EN
			| EMAC0_RGMII_TXC_SRC_SEL | EMAC_RX_REFCLK_PHASE_SEL
			| EMAC_RMII_TX_REFCLK_PHASE_SEL);
		u32int physel = EMAC_PHY_SELECT_RGMII << EMAC_PHY_SELECT_SHIFT;
		/*
		 * CLK_REF_GATE=0 (refclk enabled, unused in RGMII anyway),
		 * PHY_INTR_EN=0 / LPI_INTR_EN=0 (no WoL/EEE handler exists),
		 * *_PHASE_SEL=0 (no skew correction needed, RMII one is don't-care)
		 * RGMII_TXC_SRC_SEL=0 (RX-clock-sourced TX clock)
		 */
		csr32w(c->apmu, c->apmuctrloff, preserve | physel);
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff, preserve | physel | EMAC_BUS_EN | EMAC_1588_CLK_MUX);
		microdelay(10000);
		csr32w(c->apmu, c->apmuctrloff, preserve | physel | EMAC_BUS_EN | EMAC_1588_CLK_MUX
			| EMAC_BUS_RST | EMAC_AXI_MST_ID);
		microdelay(10000);
		print("dwqe: APMU CLK_RST[%#ux] %#ux -> %#ux (PHY_SELECT=RGMII, TXC_SRC_SEL=RXclk)\n",
			c->apmuctrloff, origv, csr32r(c->apmu, c->apmuctrloff));
	}

	/* RGMII delay lines */
	v = EMAC_RGMII_RXC_DLINE_PU | EMAC_RGMII_TXC_DLINE_PU
	  | EMAC_RGMII_RXC_DLINE_STEP_15P6 | EMAC_RGMII_TXC_DLINE_STEP_15P6
	  | c->rxdelay << EMAC_RGMII_RXC_DLINE_ADJ_SHIFT
	  | c->txdelay << EMAC_RGMII_TXC_DLINE_ADJ_SHIFT;
	csr32w(c->apmu, c->apmudlineoff, v);
	print("dwqe: APMU DLINE[%#ux]=0x%08ux\n", c->apmudlineoff, v);

	/* CR sanity-check, kept as a debug-only cross-check */
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
			aclkctrl, mux, div, csrclk, wantcr, MII_ADDR_CR_VAL);
	}

	/* now the MAC block is reachable; stop everything */
	csr32w(c->regs, GMAC_INT_EN, 0);
	csr32w(c->regs, GMAC_CONFIG, 0);
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_CONTROL, 0);
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_CONTROL, 0);

	/*
	 * If neither a bootarg, the DTB, nor the early hardware capture
	 * above got us a MAC address, edev->ea is still zeroed here
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
	csr32w(c->regs, DMA_BUS_MODE, csr32r(c->regs, DMA_BUS_MODE) | DMA_BUS_MODE_SFT_RESET);
	for(timeout = 0; timeout < 1000; timeout++){
		if((csr32r(c->regs, DMA_BUS_MODE) & DMA_BUS_MODE_SFT_RESET) == 0)
			break;
		microdelay(1000);
	}
	if(timeout == 1000){
		print("dwqe: ERROR DMA_BUS_MODE software reset timed out\n");
		return -1;
	}
	dprint("dwqe: DMA reset done after %d ms\n", timeout);

	/*
	 * K3 DDR starts at 0x1_00000000 -- everything iallocb()/ucallocalign()
	 * hands us will have PADDR() >= 4G, so 64-bit DMA addressing is required
	 */
	csr32w(c->regs, DMA_SYS_BUS_MODE, DMA_SYS_BUS_MB | DMA_SYS_BUS_FB | DMA_SYS_BUS_EAME);

	/* MTL: store-and-forward, enable queue 0, size FIFOs from HW_FEATURE1 */
	{
		u32int feat1 = csr32r(c->regs, GMAC_HW_FEATURE1);
		u32int txfifo = 128 << ((feat1 & GMAC_HW_TXFIFOSIZE) >> 6);
		u32int rxfifo = 128 << (feat1 & GMAC_HW_RXFIFOSIZE);
		u32int tqs = txfifo / 256 - 1;
		u32int rqs = rxfifo / 256 - 1;

		print("dwqe: HW_FEATURE1=%#08ux txfifo=%ud rxfifo=%ud bytes\n",
			feat1, txfifo, rxfifo);

		csr32w(c->regs, MTL_CHAN_BASE + MTL_CHAN_TX_OP_MODE,
			MTL_OP_MODE_TSF | MTL_OP_MODE_TXQEN
			| ((tqs << 16) & MTL_OP_MODE_TQS_MASK));
		csr32w(c->regs, MTL_CHAN_BASE + MTL_CHAN_RX_OP_MODE,
			MTL_OP_MODE_RSF | ((rqs << 20) & MTL_OP_MODE_RQS_MASK));
	}

	/* MAC address */
	csr32w(c->regs, GMAC_ADDR0_HIGH,
		(1U << 31) | edev->ea[5]<<8 | edev->ea[4]);
	csr32w(c->regs, GMAC_ADDR0_LOW,
		edev->ea[3]<<24 | edev->ea[2]<<16 | edev->ea[1]<<8 | edev->ea[0]);
	dprint("dwqe: MAC %02X:%02X:%02X:%02X:%02X:%02X programmed\n",
		edev->ea[0], edev->ea[1], edev->ea[2], edev->ea[3], edev->ea[4], edev->ea[5]);

	/*
	 * Enable RX queue 0 at the MAC's queue-routing level.  This is
	 * upstream of MTL/DMA entirely -- without it the MAC can silently
	 * drop everything before it ever reaches the MTL FIFO
	 */
	csr32w(c->regs, GMAC_RXQ_CTRL0, GMAC_RXQCTRL_Q0_DCB_EN);

	/*
	 * Real filtering now: unicast perfect-match on GMAC_ADDR0 +
	 * broadcast + all multicast. Toggle via the /net/etherN/clone
	 * file ("promisc 1"/"allmulti 0") if you need something
	 * tighter or looser at runtime.
	 */
	dwqesetfilter(c, 0, 1);

	/* GMAC_CONFIG base bits; RE/TE turned on by ethproc once link is up */
	csr32w(c->regs, GMAC_CONFIG,
		GMAC_CONFIG_JD | GMAC_CONFIG_BE | GMAC_CONFIG_DCRS | GMAC_CONFIG_ACS);

	/* descriptor rings -- both non-cacheable (4 descs/line) */
	c->rxr = ucallocalign(sizeof(u32int) * 4 * RXRING, 16, 0);
	c->txr = ucallocalign(sizeof(u32int) * 4 * TXRING, 16, 0);
	if(c->rxr == nil || c->txr == nil){
		print("dwqe: ERROR ring alloc failed\n");
		return -1;
	}
	memset(c->rxr, 0, 4 * RXRING * sizeof(u32int));
	memset(c->txr, 0, 4 * TXRING * sizeof(u32int));
	coherence();	/* both rings are NC memory now; order the zeroing, nothing to flush */

	c->rxs = mallocz(sizeof(Block*) * RXRING, 1);
	c->txs = mallocz(sizeof(Block*) * TXRING, 1);
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
		uvlong txend = txpa + 4 * TXRING * sizeof(u32int) - 1;
		uvlong rxend = rxpa + 4 * RXRING * sizeof(u32int) - 1;

		/* The tail-pointer registers are 32-bit only */
		if((txpa >> 32) != (txend >> 32) || (rxpa >> 32) != (rxend >> 32)){
			print("dwqe: ERROR ring straddles a 4G boundary "
				"(tx=%#llux..%#llux rx=%#llux..%#llux)\n",
				txpa, txend, rxpa, rxend);
			return -1;
		}

		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_BASE_ADDR_HI, txpa >> 32);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_BASE_ADDR, (u32int)txpa);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_BASE_ADDR_HI, rxpa >> 32);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_BASE_ADDR, (u32int)rxpa);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_RING_LEN, TXRING - 1);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_RX_RING_LEN, RXRING - 1);
		csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_TX_END_ADDR, (u32int)txpa);
	}

	/*
	 * Fills the ring (leaving the one sentinel gap slot empty and sets
	 * DMA_CHAN_RX_END_ADDR itself, now that RX_BASE_ADDR is already programmed.
	 */
	replenish(c);

	dprint("dwqe: rings rxr=%p txr=%p (%d/%d slots)\n", c->rxr, c->txr, RXRING, TXRING);

	/* DMA channel stays stopped (ST/SR off); ethproc starts both on link-up */
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_STATUS, ~0U);
	csr32w(c->regs, DMA_CHAN_BASE + DMA_CHAN_INTR_ENA, 0);
	csr32w(c->regs, GMAC_INT_EN, 0);

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

	if(edev->ctlrno < 0 || edev->ctlrno >= MAXDWQE){
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
	for(i = 0; i < n; i++){
		Dtbether *d = dtbether(i);
		if(d != nil && d->ok && d->aliasidx == edev->ctlrno
		    && strstr(d->compatible, "spacemit,k3-gmac") != nil){
			de = d;
			break;
		}
	}
	if(de != nil){
		base     = de->base;
		size     = de->size ? de->size : GMAC0_SIZE;
		apmubase = de->apmubase ? de->apmubase : APMU_PHYS;
		irq      = de->irq;
		ct->apmuctrloff  = de->ctrlregoff  ? de->ctrlregoff  : 0x3e4;
		ct->apmudlineoff = de->dlineregoff ? de->dlineregoff : 0x3e8;
		print("dwqe: ether%d from DTB %s base=%#llux size=%#llux apmu=%#llux+{%#ux,%#ux} irq=%d\n",
			edev->ctlrno, de->path, base, size, apmubase,
			ct->apmuctrloff, ct->apmudlineoff, irq);
	} else if(edev->ctlrno == 0 && n == 0){
		base     = GMAC0_PHYS;
		size     = GMAC0_SIZE;
		apmubase = APMU_PHYS;
		irq      = GMAC0_IRQ;
		ct->apmuctrloff  = 0x3e4;
		ct->apmudlineoff = 0x3e8;
		print("dwqe: no DTB, using K3 ether0 defaults\n");
	} else {
		dprint("dwqe: no DTB entry for ether%d\n", edev->ctlrno);
		return -1;
	}

	ct->regs = (ulong*)vmap(base, size);
	ct->apmu = (ulong*)vmap(apmubase, APMU_SIZE);
	if(ct->regs == nil || ct->apmu == nil){
		print("dwqe: ERROR vmap failed regs=%p apmu=%p\n", ct->regs, ct->apmu);
		ct->regs = nil;
		return -1;
	}
	ct->irq = irq;
	dprint("dwqe: ether%d regs=%p (phys=%#llux) apmu=%p (phys=%#llux)\n",
		edev->ctlrno, ct->regs, base, ct->apmu, apmubase);

	ct->rxdelay = DWQE_DEFAULT_RXDELAY;
	ct->txdelay = DWQE_DEFAULT_TXDELAY;
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
