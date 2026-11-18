/*
 * Cadence GEM on the Microchip PolarFire SoC, SGMII to an external
 * PHY.  The U54s cannot clean or invalidate the cache by address, so
 * the descriptor rings and packet buffers live in the non-cached DDR
 * window (NCDMABASE) and frames are copied through them.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/error.h"
#include "../port/netif.h"
#include "../port/etherif.h"

enum {
	Nrx		= 128,
	Ntx		= 64,
	Bufsz		= 1600,		/* a multiple of 64, the rx buffer unit */
	Maxctlr		= 2,
	Linkdelay	= 500,
};

enum {				/* registers, as u32int indices */
	Ncr		= 0x000/4,
	Ncfgr		= 0x004/4,
	Nsr		= 0x008/4,
	Dmacfg		= 0x010/4,
	Tsr		= 0x014/4,
	Rbqp		= 0x018/4,
	Tbqp		= 0x01c/4,
	Rsr		= 0x020/4,
	Isr		= 0x024/4,
	Ier		= 0x028/4,
	Idr		= 0x02c/4,
	Man		= 0x034/4,
	Hrb		= 0x080/4,
	Hrt		= 0x084/4,
	Sa1b		= 0x088/4,
	Sa1t		= 0x08c/4,
	Pcsctl		= 0x200/4,
	Dcfg6		= 0x294/4,
	Tbqp1		= 0x440/4,	/* priority queues 1-15 */
	Rbqp1		= 0x480/4,
	Tbqph		= 0x4c8/4,
	Rbqph		= 0x4d4/4,
};

enum {
	/* Ncr */
	Rxen		= 1<<2,
	Txen		= 1<<3,
	Mpe		= 1<<4,		/* management port (MDIO) */
	Clrstat		= 1<<5,
	Tstart		= 1<<9,

	/* Ncfgr */
	Spd		= 1<<0,		/* 100 Mb/s */
	Fd		= 1<<1,
	Caf		= 1<<4,		/* copy all frames */
	Mti		= 1<<6,		/* multicast hash */
	Big		= 1<<8,		/* 1536-byte frames */
	Gbe		= 1<<10,
	Pcssel		= 1<<11,
	Mdc64		= 4<<18,	/* MDC = pclk/64 */
	Dbw		= 3<<21,	/* data bus width, set by the hardware */
	Sgmii		= 1<<27,

	/* Nsr */
	Mdioidle	= 1<<2,

	/* Dmacfg */
	Incr16		= 0x10,
	Rxpbsmax	= 3<<8,
	Txpbsmax	= 1<<10,
	Rxbsshift	= 16,

	/* Isr, Ier, Idr */
	Rcomp		= 1<<1,
	Rxubr		= 1<<2,
	Tur		= 1<<4,
	Tcomp		= 1<<7,
	Rovr		= 1<<10,
	Hresp		= 1<<11,

	/* Pcsctl */
	Pcsrestart	= 1<<9,
	Pcsan		= 1<<12,

	/* Man */
	Manread		= 1<<30 | 2<<28 | 2<<16,
	Manwrite	= 1<<30 | 1<<28 | 2<<16,
};

enum {				/* descriptors */
	Rxused		= 1<<0,		/* word 0: frame written by the MAC */
	Rxwrap		= 1<<1,
	Rxlen		= 0x1fff,	/* word 1 */
	Rxeof		= 1<<15,
	Rxsof		= 1<<14,

	Txlast		= 1<<15,	/* word 1 */
	Txwrap		= 1<<30,
	Txused		= 1<<31,	/* owned by software */
};

enum {				/* PHY registers */
	Bmcr		= 0,
	Bmsr		= 1,
	Phyid1		= 2,
	Phyid2		= 3,
	Anar		= 4,
	Anlpar		= 5,
	Gbcr		= 9,
	Gbsr		= 10,

	Bmcran		= 1<<12,
	Bmcrrestart	= 1<<9,
	Bmsrlink	= 1<<2,
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	u32int	*r;
	int	phy;
	int	attached;

	u32int	*rxd;		/* rx ring: 2 words each */
	uchar	*rxb;
	int	rxi;

	Lock	txlock;
	u32int	*txd;
	uchar	*txb;
	int	txi;

	u32int	*dummy;		/* parks the unused priority queues */
};

static Ctlr ctlrs[Maxctlr];
static uintptr ncnext = NCDMABASE;

/* never freed: the rings live as long as the kernel */
static void*
ncalloc(usize n)
{
	uintptr p;

	p = ROUND(ncnext, 64);
	if(p + n > NCDMABASE + NCDMASIZE)
		panic("ethermpfs: out of non-cached memory");
	ncnext = p + n;
	memset((void*)p, 0, n);
	return (void*)p;
}

static int
mdiowait(Ctlr *c)
{
	int i;

	for(i = 0; i < 1000; i++){
		if(c->r[Nsr] & Mdioidle)
			return 0;
		microdelay(10);
	}
	return -1;
}

static int
mdioread(Ctlr *c, int phy, int reg)
{
	if(mdiowait(c) < 0)
		return -1;
	c->r[Man] = Manread | phy<<23 | reg<<18;
	if(mdiowait(c) < 0)
		return -1;
	return c->r[Man] & 0xffff;
}

static void
mdiowrite(Ctlr *c, int phy, int reg, int v)
{
	if(mdiowait(c) < 0)
		return;
	c->r[Man] = Manwrite | phy<<23 | reg<<18 | v & 0xffff;
	mdiowait(c);
}

/* the PHY on this MAC's MDIO bus: try the DTB's address 0 first */
static int
phyfind(Ctlr *c)
{
	int a, id;

	for(a = 0; a < 32; a++){
		id = mdioread(c, a, Phyid1);
		if(id < 0)
			return -1;
		if(id != 0 && id != 0xffff)
			return a;
	}
	return -1;
}

static void
linkup(Ether *edev)
{
	Ctlr *c;
	int gb, an, mbps, fd;
	u32int cfg;

	c = edev->ctlr;
	gb = mdioread(c, c->phy, Gbcr) & mdioread(c, c->phy, Gbsr)>>2;
	an = mdioread(c, c->phy, Anar) & mdioread(c, c->phy, Anlpar);
	if(gb & 0x300){
		mbps = 1000;
		fd = (gb & 0x200) != 0;
	}else if(an & 0x180){
		mbps = 100;
		fd = (an & 0x100) != 0;
	}else{
		mbps = 10;
		fd = (an & 0x40) != 0;
	}
	cfg = c->r[Ncfgr] & ~(Spd|Fd|Gbe);
	if(mbps == 1000)
		cfg |= Gbe;
	else if(mbps == 100)
		cfg |= Spd;
	if(fd)
		cfg |= Fd;
	c->r[Ncfgr] = cfg;
	print("#l%d: mpfs: link up %d Mb/s %s duplex\n", edev->ctlrno, mbps, fd? "full": "half");
	ethersetspeed(edev, mbps);
	ethersetlink(edev, 1);
}

static void
linkproc(void *a)
{
	Ether *edev;
	Ctlr *c;
	int bmsr, link;

	edev = a;
	c = edev->ctlr;
	mdiowrite(c, c->phy, Bmcr, Bmcran|Bmcrrestart);
	link = 0;
	for(;;){
		/* link status latches low: the second read is current */
		mdioread(c, c->phy, Bmsr);
		bmsr = mdioread(c, c->phy, Bmsr);
		if(bmsr >= 0 && (bmsr & Bmsrlink) != 0){
			if(!link)
				linkup(edev);
			link = 1;
		}else if(link){
			print("#l%d: mpfs: link down\n", edev->ctlrno);
			ethersetlink(edev, 0);
			link = 0;
		}
		tsleep(&up->sleep, return0, nil, Linkdelay);
	}
}

static void
rxintr(Ether *edev)
{
	Ctlr *c;
	u32int *d;
	Block *b;
	int n;

	c = edev->ctlr;
	for(;;){
		d = &c->rxd[2*c->rxi];
		if((d[0] & Rxused) == 0)
			break;
		n = d[1] & Rxlen;
		if((d[1] & (Rxsof|Rxeof)) == (Rxsof|Rxeof) && n > 0 && n <= Bufsz
		&& (b = iallocb(n)) != nil){
			memmove(b->wp, c->rxb + c->rxi*Bufsz, n);
			b->wp += n;
			etheriq(edev, b);
		}
		coherence();
		d[0] &= ~Rxused;
		c->rxi = (c->rxi + 1) % Nrx;
	}
}

static void
transmit(Ether *edev)
{
	Ctlr *c;
	u32int *d;
	Block *b;
	int n;

	c = edev->ctlr;
	ilock(&c->txlock);
	for(;;){
		d = &c->txd[2*c->txi];
		if((d[1] & Txused) == 0)
			break;		/* ring full; Tcomp restarts us */
		b = qget(edev->oq);
		if(b == nil)
			break;
		n = BLEN(b);
		if(n > Bufsz)
			n = Bufsz;
		memmove(c->txb + c->txi*Bufsz, b->rp, n);
		freeb(b);
		coherence();
		d[1] = n | Txlast | (c->txi == Ntx-1? Txwrap: 0);
		coherence();
		c->r[Ncr] |= Tstart;
		c->txi = (c->txi + 1) % Ntx;
	}
	iunlock(&c->txlock);
}

static void
interrupt(Ureg*, void *a)
{
	Ether *edev;
	Ctlr *c;
	u32int s;

	edev = a;
	c = edev->ctlr;
	s = c->r[Isr];
	c->r[Isr] = s;
	if(s & (Rcomp|Rxubr|Rovr)){
		c->r[Rsr] = c->r[Rsr];
		rxintr(edev);
	}
	if(s & (Tcomp|Tur)){
		c->r[Tsr] = c->r[Tsr];
		transmit(edev);
	}
	if(s & Hresp)
		iprint("#l%d: mpfs: dma bus error\n", edev->ctlrno);
}

static void
promiscuous(void *a, int on)
{
	Ctlr *c;

	c = ((Ether*)a)->ctlr;
	if(on)
		c->r[Ncfgr] |= Caf;
	else
		c->r[Ncfgr] &= ~Caf;
}

static void
multicast(void *a, uchar *ea, int on)
{
	Ether *edev;
	Ctlr *c;
	uvlong v;
	int x;

	edev = a;
	c = edev->ctlr;
	if(!on)
		return;
	v = (uvlong)ea[0] | (uvlong)ea[1]<<8 | (uvlong)ea[2]<<16
		| (uvlong)ea[3]<<24 | (uvlong)ea[4]<<32 | (uvlong)ea[5]<<40;
	x = (v ^ v>>6 ^ v>>12 ^ v>>18 ^ v>>24 ^ v>>30 ^ v>>36 ^ v>>42) & 63;
	if(x < 32)
		c->r[Hrb] |= 1<<x;
	else
		c->r[Hrt] |= 1<<(x-32);
	c->r[Ncfgr] |= Mti;
}

static void
attach(Ether *edev)
{
	Ctlr *c;

	c = edev->ctlr;
	if(c->attached)
		return;
	c->attached = 1;
	c->r[Ncr] = Mpe | Txen | Rxen;
	kproc("etherlink", linkproc, edev);
}

static void
reset(Ether *edev)
{
	Ctlr *c;
	int i, nq;

	c = edev->ctlr;
	c->r[Ncr] = Mpe;
	c->r[Idr] = ~0;
	c->r[Isr] = ~0;
	c->r[Tsr] = ~0;
	c->r[Rsr] = ~0;
	c->r[Ncr] = Mpe | Clrstat;
	c->r[Ncfgr] = c->r[Ncfgr] & Dbw | Sgmii | Pcssel | Mdc64 | Big | Fd | Gbe;
	c->r[Pcsctl] = Pcsan | Pcsrestart;
	/* 32-bit descriptors: everything is below 4GB */
	c->r[Dmacfg] = (Bufsz/64)<<Rxbsshift | Txpbsmax | Rxpbsmax | Incr16;

	c->rxd = ncalloc(8*Nrx);
	c->rxb = ncalloc(Nrx*Bufsz);
	for(i = 0; i < Nrx; i++){
		c->rxd[2*i] = PADDR(c->rxb + i*Bufsz);
		c->rxd[2*i+1] = 0;
	}
	c->rxd[2*(Nrx-1)] |= Rxwrap;
	c->txd = ncalloc(8*Ntx);
	c->txb = ncalloc(Ntx*Bufsz);
	for(i = 0; i < Ntx; i++){
		c->txd[2*i] = PADDR(c->txb + i*Bufsz);
		c->txd[2*i+1] = Txused;
	}
	c->txd[2*(Ntx-1)+1] |= Txwrap;

	/* a used, wrapping descriptor stops the other queues */
	c->dummy = ncalloc(16);
	c->dummy[0] = Rxused | Rxwrap;
	c->dummy[1] = Txused | Txwrap;
	nq = c->r[Dcfg6] & 0xfffe;
	for(i = 1; i < 16; i++)
		if(nq & 1<<i){
			c->r[Tbqp1+i-1] = PADDR(c->dummy);
			c->r[Rbqp1+i-1] = PADDR(c->dummy);
		}
	c->r[Tbqph] = 0;
	c->r[Rbqph] = 0;
	c->r[Rbqp] = PADDR(c->rxd);
	c->r[Tbqp] = PADDR(c->txd);
	c->rxi = 0;
	c->txi = 0;

	c->r[Hrb] = 0;
	c->r[Hrt] = 0;
	c->r[Sa1b] = edev->ea[0] | edev->ea[1]<<8 | edev->ea[2]<<16 | edev->ea[3]<<24;
	c->r[Sa1t] = edev->ea[4] | edev->ea[5]<<8;
	c->r[Ier] = Rcomp | Rxubr | Rovr | Tcomp | Tur | Hresp;
}

static int
pnp(Ether *edev)
{
	static int probed[Maxctlr];
	Dtbether *d;
	Ctlr *c;
	int i, k, n;

	n = dtbnether();
	k = 0;
	for(i = 0; i < n && k < Maxctlr; i++){
		d = dtbether(i);
		if(d == nil || !d->ok || strstr(d->compatible, "microchip,mpfs-macb") == nil)
			continue;
		c = &ctlrs[k];
		if(probed[k++])
			continue;
		probed[k-1] = 1;
		c->r = vmap(d->base, d->size? d->size: 0x2000);
		if(c->r == nil)
			continue;
		/* skip MACs with no PHY on their MDIO bus */
		c->r[Ncr] = Mpe;
		c->r[Ncfgr] = c->r[Ncfgr] & Dbw | Mdc64;
		c->phy = phyfind(c);
		if(c->phy < 0){
			c->r[Ncr] = 0;
			print("ethermpfs: %#llux: no phy\n", d->base);
			continue;
		}
		edev->ctlr = c;
		edev->port = d->base;
		edev->irq = d->irq;
		if(d->hasmac)
			memmove(edev->ea, d->mac, Eaddrlen);
		print("ethermpfs: %#llux irq %d phy %d id %#ux:%#ux\n", d->base, d->irq,
			c->phy, mdioread(c, c->phy, Phyid1), mdioread(c, c->phy, Phyid2));
		reset(edev);
		edev->mbps = 1000;
		edev->attach = attach;
		edev->transmit = transmit;
		edev->promiscuous = promiscuous;
		edev->multicast = multicast;
		edev->arg = edev;
		intrenable(edev->irq, interrupt, edev, BUSUNKNOWN, edev->name);
		return 0;
	}
	return -1;
}

void
ethermpfslink(void)
{
	addethercard("mpfs", pnp);
}
