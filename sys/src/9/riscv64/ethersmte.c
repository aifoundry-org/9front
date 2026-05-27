#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/netif.h"
#include "../port/etherif.h"

/* Physical address and IRQ */
enum {
	EMAC0_PHYS		= 0xcac80000UL,
	EMAC0_SIZE		= 0x420,
	EMAC0_IRQ		= 141, 
	EMAC0_APMU_OFF	= 0x0000,

	APMU_PHYS		= 0xd4282800UL,
	APMU_SIZE		= 0x400,

	/* Default RGMII delays: 1560 ps = 100 × 15.6 ps (vendor DTS default) */
	SMTE_DEFAULT_RXDELAY_PS	= 1560,
	SMTE_DEFAULT_TXDELAY_PS	= 1560,
};

/* DMA/MAC registers */
enum {
	DMA_CONFIG						= 0x0000,
	DMA_CONFIG_DMA_64BIT_MODE		= (1U << 18),
	DMA_CONFIG_STRICT_BURST			= (1U << 17),
	DMA_CONFIG_BURST_LENGTH_MASK	= (0x7f << 1),
	DMA_CONFIG_BURST_LENGTH_16		= (0x10 << 1),
	DMA_CONFIG_SOFTWARE_RESET		= (1U << 0),

	DMA_CTRL						= 0x0004,
	DMA_CTRL_START_STOP_RX_DMA		= (1U << 1),
	DMA_CTRL_START_STOP_TX_DMA		= (1U << 0),

	DMA_STATUS_IRQ						= 0x0008,
	DMA_STATUS_IRQ_RX_MISSED_FRAME		= (1U << 7),
	DMA_STATUS_IRQ_RX_DMA_STOPPED		= (1U << 6),
	DMA_STATUS_IRQ_RX_DES_UNAVAILABLE	= (1U << 5),
	DMA_STATUS_IRQ_RX_TRANSFER_DONE		= (1U << 4),
	DMA_STATUS_IRQ_TX_TRANSFER_DONE		= (1U << 0),

	DMA_INTR_ENABLE						= 0x000c,
	DMA_INTR_ENABLE_RX_MISSED_FRAME		= (1U << 7),
	DMA_INTR_ENABLE_RX_DMA_STOPPED		= (1U << 6),
	DMA_INTR_ENABLE_RX_DES_UNAVAILABLE	= (1U << 5),
	DMA_INTR_ENABLE_RX_TRANSFER_DONE	= (1U << 4),
	DMA_INTR_ENABLE_TX_TRANSFER_DONE	= (1U << 0),

	DMA_TRANSMIT_AUTO_POLL_COUNTER		= 0x0010,
	DMA_TRANSMIT_POLL_DEMAND			= 0x0014,
	DMA_RECEIVE_POLL_DEMAND				= 0x0018,
	DMA_TRANSMIT_BASE_ADDRESS			= 0x001c,
	DMA_RECEIVE_BASE_ADDRESS			= 0x0020,

	DMA_RECEIVE_IRQ_MITIGATION							= 0x002c,
	DMA_RECEIVE_IRQ_MITIGATION_FRAME_COUNTER_SHIFT 		= 0,
	DMA_RECEIVE_IRQ_MITIGATION_TIMEOUT_COUNTER_SHIFT 	= 8,
	DMA_RECEIVE_IRQ_MITIGATION_MITIGATION_ENABLE 		= (1U << 31),

	MAC_GLOBAL_CTRL					= 0x0100,
	MAC_GLOBAL_CTRL_DUPLEX_MODE		= (1U << 2),
	MAC_GLOBAL_CTRL_SPEED_MASK		= (0x3 << 0),
	MAC_GLOBAL_CTRL_SPEED_10		= (0x0 << 0),
	MAC_GLOBAL_CTRL_SPEED_100		= (0x1 << 0),
	MAC_GLOBAL_CTRL_SPEED_1000		= (0x2 << 0),

	MAC_TRANSMIT_CTRL				= 0x0104,
	MAC_TRANSMIT_CTRL_IFG_LEN_MASK	= (0x7 << 4),
	MAC_TRANSMIT_CTRL_TX_AUTO_RETRY	= (1U << 3),
	MAC_TRANSMIT_CTRL_TX_ENABLE		= (1U << 0),

	MAC_RECEIVE_CTRL				= 0x0108,
	MAC_RECEIVE_CTRL_STORE_FORWARD	= (1U << 3),
	MAC_RECEIVE_CTRL_RX_ENABLE		= (1U << 0),

	MAC_MAXIMUM_FRAME_SIZE			= 0x010c,
	MAC_TRANSMIT_JABBER_SIZE		= 0x0110,
	MAC_RECEIVE_JABBER_SIZE			= 0x0114,

	MAC_ADDR_CTRL					= 0x0118,
	MAC_ADDR_CTRL_PROMISCUOUS_MODE	= (1U << 8),
	MAC_ADDR_CTRL_MAC_ADDR1_ENABLE	= (1U << 0),

	MAC_ADDR1_HI				= 0x0120,
	MAC_ADDR1_ME				= 0x0124,
	MAC_ADDR1_LO				= 0x0128,

	MAC_MULTICAST_HASH_TABLE1		= 0x0150,
	MAC_MULTICAST_HASH_TABLE2		= 0x0154,
	MAC_MULTICAST_HASH_TABLE3		= 0x0158,
	MAC_MULTICAST_HASH_TABLE4		= 0x015c,

	MAC_MDIO_CTRL							= 0x01a0,
	MAC_MDIO_CTRL_START_MDIO_TRANS			= (1U << 15),
	MAC_MDIO_CTRL_MDIO_READ_WRITE			= (1U << 10),
	MAC_MDIO_CTRL_REGISTER_ADDRESS_SHIFT	= 5,
	MAC_MDIO_CTRL_PHY_ADDRESS_SHIFT			= 0,

	MAC_MDIO_DATA						= 0x01a4,

	MAC_TRANSMIT_FIFO_ALMOST_FULL		= 0x01c0,
	MAC_TRANSMIT_PACKET_START_THRESHOLD	= 0x01c4,
	MAC_RECEIVE_PACKET_START_THRESHOLD	= 0x01c8,

	MAC_INTR_ENABLE						= 0x01e4,
};

/* APMU registers */
enum {
	APMU_EMAC_CLK_RST_CTRL			= 0x0000,
	APMU_EMAC_AXI_MST_ID			= (1U << 13),
	APMU_EMAC_PHY_INTR_EN			= (1U << 12),
	APMU_EMAC_RGMII_TXC_SRC_SEL		= (1U << 8),

	APMU_EMAC_RGMII_DLINE			= 0x0004,

	APMU_EMAC_RGMII_DLINE_TX_DELAY_MASK		= (0xff << 24),
	APMU_EMAC_RGMII_DLINE_TX_DELAY_SHIFT	= 8,

	APMU_EMAC_RGMII_DLINE_TX_STEP_MASK	= (0x3 << 20),
	APMU_EMAC_RGMII_DLINE_TX_STEP_15P6	= (0x0 << 20),

	APMU_EMAC_RGMII_DLINE_TX_EN		= (1U << 16),

	APMU_EMAC_RGMII_DLINE_RX_DELAY_MASK		= (0xff << 8),
	APMU_EMAC_RGMII_DLINE_RX_DELAY_SHIFT	= 8,

	APMU_EMAC_RGMII_DLINE_RX_STEP_MASK	= (0x3 << 4),
	APMU_EMAC_RGMII_DLINE_RX_STEP_15P6	= (0x0 << 4),

	APMU_EMAC_RGMII_DLINE_RX_EN		= (1U << 0),
};

/* Descriptors */
typedef struct SmteDesc SmteDesc;

struct SmteDesc {
	u32int	sd_desc0;
	u32int	sd_desc1;
	u32int	sd_addr1;
	u32int	sd_addr2;
};

/* Rx bits */
enum {
	RX_DESC0_FRAME_PACKET_LENGTH_MASK	= (0x3fff << 0),
	RX_DESC0_FRAME_PACKET_LENGTH_SHIFT	= 0,
	RX_DESC0_FRAME_RUNT					= (1U << 15),
	RX_DESC0_FRAME_CRC_ERR				= (1U << 20),
	RX_DESC0_FRAME_MAX_LEN_ERR			= (1U << 21),
	RX_DESC0_FRAME_JABBER_ERR			= (1U << 22),
	RX_DESC0_FRAME_LENGTH_ERR			= (1U << 23),
	RX_DESC0_OWN						= (1U << 31),

	RX_DESC1_SIZE1_MASK			= (0xfffff << 0),
	RX_DESC1_SIZE1_SHIFT		= 0,
	RX_DESC1_SIZE2_MASK			= (0xfffff << 12),
	RX_DESC1_SIZE2_SHIFT		= 12,
	RX_DESC1_END_RING			= (1U << 26),
};

/* Tx bits */
enum {
	TX_DESC0_OWN				= (1U << 31),

	TX_DESC1_SIZE1_MASK			= (0xfff << 0),
	TX_DESC1_SIZE1_SHIFT		= 0,

	TX_DESC1_SIZE2_MASK			= (0xfff << 12),
	TX_DESC1_SIZE2_SHIFT		= 12,

	TX_DESC1_END_RING					= (1U << 26),
	TX_DESC1_FIRST_SEGMENT				= (1U << 29),
	TX_DESC1_LAST_SEGMENT				= (1U << 30),
	TX_DESC1_INTERRUPT_ON_COMPLETION	= (1U << 31),
};

/* SDH data for check Vendor_ID Spacemit Soc */
enum {
	SDH1_PHYS		= 0xD4280000,
	SDH1_SIZE		= 0x1000,
	SDHC_VID_PID	= 0x100,
};

typedef struct Ctlr Ctlr;

struct Ctlr {
	void	*regs;		/* MAC/DMA MMIO */
	void	*apmu;		/* APMU MMIO */
	u32int	apmuoff;
	int	irq;

	u32int	rxdelay;
	u32int	txdelay;
};

/*------------------------------------------------
static void
mdwrite(Ctlr *c, int r, u16int v)
{
// RGMII
// MDIO write control PHY RTL8211
}

static u16int
mdread(Ctlr *c, int r)
{
// RGMII
//MDIO read control PHY RTL8211
}

static void
ethproc(void *ved)
{

}

static int
replenish(Ctlr *c)
{

	return 0;
}

static void
ethrx(Ether *edev)
{

}

static void
ethtx(Ether *edev)
{

}

static void
ethirq(Ureg *, void *arg)
{

}

static int
ethinit(Ether *edev)
{

	return 0;
}

static void
ethattach(Ether *edev)
{
	Ctlr *c;

	c = edev->ctlr;
	if(c->attach)
		return;
	c->attach = 1;
	kproc("ethproc", ethproc, edev);
}

static void
ethprom(void *arg, int on)
{

}

static void
sethash(uchar *ea, ulong *hash)
{
	ulong crc;
	int i;
	uchar n;
	
	crc = ethercrc(ea, 6);
	n = 0;
	for(i = 0; i < 8; i++){
		n = n << 1 | crc & 1;
		crc >>= 1;
	}
	n ^= 0xff;
	hash[n>>5] |= (1<<(n & 31));
}

static void
ethmcast(void *arg, uchar *ea, int on)
{

}

static char*
ethifstat(void *arg, char *p, char *e)
{

}
------------------------------------------------*/



static u32int
csr32r(uintptr base, uintptr off)
{
	coherence();
	return *(volatile u32int*)(base + off);
}

static void
smtereadhwaddr(uintptr base, uchar ea[Eaddrlen])
{
	u32int hi, me, lo;

	hi = csr32r(base, MAC_ADDR1_HI);
	me = csr32r(base, MAC_ADDR1_ME);
	lo = csr32r(base, MAC_ADDR1_LO);

	ea[0] = hi & 0xff;
	ea[1] = (hi >> 8) & 0xff;
	ea[2] = me & 0xff;
	ea[3] = (me >> 8) & 0xff;
	ea[4] = lo & 0xff;
	ea[5] = (lo >> 8) & 0xff;

	print("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", ea[0], ea[1], ea[2], ea[3], ea[4], ea[5]);
}

int
check_soc_fingerprint(void)
{
    uintptr sdh_mem;
    u32int val, vendor_id;

    sdh_mem = (uintptr)vmap(SDH1_PHYS, SDH1_SIZE);

    val = csr32r(sdh_mem, SDHC_VID_PID);
    
    // VENDOR_ID bits [19:0]
    vendor_id = val & 0xFFFFF;

    print("k1: SDHC_VID_PID register: 0x%ux\n", val);
    print("k1: Detected VENDOR_ID: 0x%ux\n", vendor_id);

    if(vendor_id == 0xa1312) {
        print("k1: Hardware fingerprint confirmed: SpacemiT SoC\n");
        return 1;
    }
    print("k1: Warning: SOC Vendor ID mismatch (Expected 0xa1312)\n");
    return 0;
}

static int
etherpnp(Ether *edev)
{
	static Ctlr ct;

	if (check_soc_fingerprint()) {
		ct.regs 	= vmap(EMAC0_PHYS, EMAC0_SIZE);
		ct.apmu 	= vmap(APMU_PHYS, APMU_SIZE);
		ct.apmuoff 	= EMAC0_APMU_OFF;
		ct.irq     	= EMAC0_IRQ;
		ct.rxdelay 	= SMTE_DEFAULT_RXDELAY_PS;
		ct.txdelay 	= SMTE_DEFAULT_TXDELAY_PS;
	
		edev->ctlr 	= &ct;
		edev->port 	= EMAC0_PHYS;
		edev->irq 	= EMAC0_IRQ;

		smtereadhwaddr((uintptr)ct.regs, edev->ea);
	
		return -1;	/* Unfinish */
	}
	return -1;
}

void
ethersmtelink(void)
{
	addethercard("eth", etherpnp);
}
