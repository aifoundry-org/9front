/*
 * SpacemiT K1 SD host (eMMC at 0xd4281000)
 *
 * derived from zynq/emmc.c
 */

#include "u.h"
#include "../port/lib.h"
#include "../port/error.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/sd.h"

enum {
	/* Standard SDHCI registers */
	Sysaddr			= 0x00>>2,
	Blksizecnt		= 0x04>>2,
	Arg1			= 0x08>>2,
	Cmdtm			= 0x0c>>2,
	Resp0			= 0x10>>2,
	Resp1			= 0x14>>2,
	Resp2			= 0x18>>2,
	Resp3			= 0x1c>>2,
	Data			= 0x20>>2,
	Status			= 0x24>>2,
	Control0		= 0x28>>2,
	Control1		= 0x2c>>2,
	Interrupt		= 0x30>>2,
	Irptmask		= 0x34>>2,
	Irpten			= 0x38>>2,
	Capabilites		= 0x40>>2,
	Forceirpt		= 0x50>>2,
	Boottimeout		= 0x60>>2,
	Dbgsel			= 0x64>>2,
	Spiintspt		= 0xf0>>2,
	Slotisrver		= 0xfc>>2,

	/* SpacemiT-specific extension registers */
	Sdhclegctl		= 0x10c>>2,
	 Genpadclkon	= 1<<6,
	Sdhcmmcctl		= 0x114>>2,
	 Mmccardmode	= 1<<12,
	Sdhctxcfg		= 0x11c>>2,
	 Txintclksel	= 1<<30,
	Sdhcphyctl		= 0x160>>2,
	 Phyfuncen		= 1<<0,
	 Phyplllock		= 1<<1,
	Sdhcphypadcfg	= 0x178>>2,
	 Phydrivesel	= 4<<0,
	 Rxbiasctl		= 1<<5,

	/* Control0 */
	Dwidth4			= 1<<1,
	Dwidth1			= 0<<1,

	/* Control1 */
	Srstdata		= 1<<26,	/* reset data circuit */
	Srstcmd			= 1<<25,	/* reset command circuit */
	Srsthc			= 1<<24,	/* reset complete host controller */
	Datatoshift		= 16,		/* data timeout unit exponent */
	Datatomask		= 0xF0000,
		DTO		= 14,		/* data timeout exponent (guesswork) */
	Clkfreq8shift		= 8,		/* SD clock base divider LSBs */
	Clkfreq8mask		= 0xFF00,
	Clkfreqms2shift		= 6,		/* SD clock base divider MSBs */
	Clkfreqms2mask		= 0xC0,
	Clkgendiv		= 0<<5,		/* SD clock divided */
	Clkgenprog		= 1<<5,		/* SD clock programmable */
	Clken			= 1<<2,		/* SD clock enable */
	Clkstable		= 1<<1,
	Clkintlen		= 1<<0,		/* enable internal EMMC clocks */

	/* Cmdtm */
	Indexshift		= 24,
	Suspend			= 1<<22,
	Resume			= 2<<22,
	Abort			= 3<<22,
	Isdata			= 1<<21,
	Ixchken			= 1<<20,
	Crcchken		= 1<<19,
	Respmask		= 3<<16,
	Respnone		= 0<<16,
	Resp136			= 1<<16,
	Resp48			= 2<<16,
	Resp48busy		= 3<<16,
	Multiblock		= 1<<5,
	Host2card		= 0<<4,
	Card2host		= 1<<4,
	Autocmd12		= 1<<2,
	Autocmd23		= 2<<2,
	Blkcnten		= 1<<1,
	Dmaen			= 1<<0,

	/* Interrupt */
	Acmderr		= 1<<24,
	Denderr		= 1<<22,
	Dcrcerr		= 1<<21,
	Dtoerr		= 1<<20,
	Cbaderr		= 1<<19,
	Cenderr		= 1<<18,
	Ccrcerr		= 1<<17,
	Ctoerr		= 1<<16,
	Err		= 1<<15,
	Cardintr	= 1<<8,
	Cardinsert	= 1<<6,
	Readrdy		= 1<<5,
	Writerdy	= 1<<4,
	Dmaintr		= 1<<3,
	Datadone	= 1<<1,
	Cmddone		= 1<<0,

	/* Status */
	Present		= 1<<18,
	Bufread		= 1<<11,
	Bufwrite	= 1<<10,
	Readtrans	= 1<<9,
	Writetrans	= 1<<8,
	Datactive	= 1<<2,
	Datinhibit	= 1<<1,
	Cmdinhibit	= 1<<0,

	/* K1 SoC */
	SDH_PHYS	= 0xd4281000UL,	/* eMMC */
	SDH_SIZE	= 0x200,
	SDH_IRQ		= 101,
	SDH_EXTCLK	= 102400000,	/* pll1_d6_409p6 / 4 (u-boot default) */

	APMU_PHYS	= 0xd4282800UL,
	APMU_SIZE	= 0x400,
	APMU_SDH0	= 0x054,	/* shared: SDH_AXI gate (bit 3) + reset (bit 0) */
	APMU_SDH2	= 0x0e0,	/* eMMC: SDH2 gate (bit 11) + reset (bit 1) */
	Sdh_axi_gate	= 1<<3,
	Sdh_axi_rst	= 1<<0,
	Sdh2_gate	= 1<<11,
	Sdh2_rst	= 1<<1,

	Cbsz		= 64,	/* X60/C908 D-cache line size */
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	Rendez	r;
	u32int	*regs;
	int	datadone;
	ulong	extclk;
	int	irq;
};

static Ctlr sdh;

static void
cacheflush(void *va, int len)
{
	uintptr a, last;

	a    = (uintptr)va & ~(uintptr)(Cbsz - 1);
	last = ((uintptr)va + len + Cbsz - 1) & ~(uintptr)(Cbsz - 1);
	for(; a < last; a += Cbsz)
		cboflush((void*)a);
	coherence();
}

static uint
clkdiv(uint d)
{
	uint v;

	assert(d < 1<<10);
	v = (d << Clkfreq8shift) & Clkfreq8mask;
	v |= ((d >> 8) << Clkfreqms2shift) & Clkfreqms2mask;
	return v;
}

static void
interrupt(Ureg*, void*)
{
	u32int *r;
	int i;

	r = sdh.regs;
	i = r[Interrupt];
	r[Interrupt] = i & (Datadone|Err);
	sdh.datadone = i;
	wakeup(&sdh.r);
}

static int
datadone(void*)
{
	return sdh.datadone;
}

static int
sdhinit(SDio*)
{
	u32int *r, *apmu;
	int i;

	apmu = vmap(APMU_PHYS, APMU_SIZE);
	apmu[APMU_SDH0>>2] |= Sdh_axi_gate | Sdh_axi_rst;
	apmu[APMU_SDH2>>2] |= Sdh2_gate | Sdh2_rst;
	coherence();
	vunmap(apmu, APMU_SIZE);

	sdh.extclk = SDH_EXTCLK;
	sdh.irq = SDH_IRQ;
	r = vmap(SDH_PHYS, SDH_SIZE);
	sdh.regs = r;
	r[Control1] = Srsthc;
	for(i = 0; i < 100; i++){
		delay(10);
		if((r[Control1] & Srsthc) == 0)
			break;
	}
	if(r[Control1] & Srsthc){
		print("sdh: reset timeout!\n");
		return -1;
	}

	/* K1 PHY bring-up after host reset */
	r[Sdhcphyctl] |= Phyfuncen | Phyplllock;
	r[Sdhcphypadcfg] = (r[Sdhcphypadcfg] & ~7) | Phydrivesel | Rxbiasctl;
	r[Sdhcmmcctl] |= Mmccardmode;
	r[Sdhclegctl] |= Genpadclkon;
	r[Sdhctxcfg] |= Txintclksel;
	return 0;
}

static int
sdhinquiry(SDio*, char *inquiry, int inqlen)
{
	uint ver;

	ver = sdh.regs[Slotisrver] >> 16;
	return snprint(inquiry, inqlen,
		"SpacemiT SD/eMMC Host Controller %2.2x Version %2.2x",
		ver&0xFF, ver>>8);
}

static void
sdhclk(uint freq)
{
	u32int *r;
	int i;

	r = sdh.regs;
	r[Control1] = clkdiv(sdh.extclk / freq - 1) |
			DTO << Datatoshift | Clkgendiv | Clken | Clkintlen;
	for(i = 0; i < 1000; i++){
		delay(1);
		if(r[Control1] & Clkstable)
			return;
	}
	print("SD clock won't initialise!\n");
}

static void
sdhenable(SDio *io)
{
	sdhclk(400000);
	sdh.regs[Irptmask] = ~(Dtoerr|Cardintr|Dmaintr);
	intrenable(sdh.irq, interrupt, nil, BUSUNKNOWN, io->name);
}

static int
sdhcmd(SDio*, SDiocmd *cmd, u32int arg, u32int *resp)
{
	ulong now;
	u32int *r;
	u32int c;
	int i;

	c = (u32int)cmd->index << Indexshift;
	switch(cmd->resp){
	case 0:
		c |= Respnone;
		break;
	case 1:
		if(cmd->busy){
			c |= Resp48busy | Ixchken | Crcchken;
			break;
		}
	default:
		c |= Resp48 | Ixchken | Crcchken;
		break;
	case 2:
		c |= Resp136 | Crcchken;
		break;
	case 3:
		c |= Resp48;
		break;
	}
	if(cmd->data){
		if(cmd->data & 1)
			c |= Isdata | Card2host | Dmaen;
		else
			c |= Isdata | Host2card | Dmaen;
		if(cmd->data > 2)
			c |= Multiblock | Blkcnten;
	}

	r = sdh.regs;
	if(r[Status] & Cmdinhibit){
		print("sdhcmd: need to reset Cmdinhibit intr %ux stat %ux\n",
			r[Interrupt], r[Status]);
		r[Control1] |= Srstcmd;
		while(r[Control1] & Srstcmd)
			;
		while(r[Status] & Cmdinhibit)
			;
	}
	if((c & Isdata || (c & Respmask) == Resp48busy) &&
	    r[Status] & Datinhibit){
		print("sdhcmd: need to reset Datinhibit intr %ux stat %ux\n",
			r[Interrupt], r[Status]);
		r[Control1] |= Srstdata;
		while(r[Control1] & Srstdata)
			;
		while(r[Status] & Datinhibit)
			;
	}
	r[Arg1] = arg;
	if((i = r[Interrupt]) != 0){
		if(i != Cardinsert)
			print("sdh: before command, intr was %ux\n", i);
		r[Interrupt] = i;
	}
	r[Cmdtm] = c;
	now = m->ticks;
	while(((i=r[Interrupt])&(Cmddone|Err)) == 0)
		if((long)(m->ticks-now) > HZ)
			break;
	if((i&(Cmddone|Err)) != Cmddone){
		if((i&~Err) != Ctoerr)
			print("sdh: cmd %ux error intr %ux stat %ux\n", c, i, r[Status]);
		r[Interrupt] = i;
		if(r[Status]&Cmdinhibit){
			r[Control1] |= Srstcmd;
			while(r[Control1]&Srstcmd)
				;
		}
		error(Eio);
	}
	r[Interrupt] = i & ~(Datadone|Readrdy|Writerdy);
	switch(c & Respmask){
	case Resp136:
		resp[0] = r[Resp0]<<8;
		resp[1] = r[Resp0]>>24 | r[Resp1]<<8;
		resp[2] = r[Resp1]>>24 | r[Resp2]<<8;
		resp[3] = r[Resp2]>>24 | r[Resp3]<<8;
		break;
	case Resp48:
	case Resp48busy:
		resp[0] = r[Resp0];
		break;
	case Respnone:
		resp[0] = 0;
		break;
	}
	if((c & Respmask) == Resp48busy){
		r[Irpten] = Datadone|Err;
		tsleep(&sdh.r, datadone, 0, 3000);
		i = sdh.datadone;
		sdh.datadone = 0;
		r[Irpten] = 0;
		if((i & Datadone) == 0)
			print("sdhio: no Datadone after %s\n", cmd->name);
		if(i & Err)
			print("sdhio: %s error interrupt %ux\n",
				cmd->name, r[Interrupt]);
		r[Interrupt] = i;
	}
	return 0;
}

static void
sdhiosetup(SDio*, int, void *buf, int bsize, int bcount)
{
	u32int *r;
	uintptr pa;
	int len;

	len = bsize*bcount;
	if(len > (0x1000<<7))
		error(Etoobig);

	pa = PADDR(buf);
	cacheflush(buf, len);

	r = sdh.regs;
	r[Sysaddr] = pa;
	r[Blksizecnt] = 7<<12 | bcount<<16 | bsize;
	r[Irpten] = Datadone|Err;
}

static void
sdhio(SDio*, int write, uchar *buf, int len)
{
	u32int *r;
	int i;

	tsleep(&sdh.r, datadone, 0, 3000);
	i = sdh.datadone;
	sdh.datadone = 0;

	r = sdh.regs;
	r[Irpten] = 0;
	if((i & Datadone) == 0){
		print("sdhio: %d timeout intr %ux stat %ux\n",
			write, i, r[Status]);
		r[Interrupt] = i;
		error(Eio);
	}
	if(i & Err){
		print("sdhio: %d error intr %ux stat %ux\n",
			write, r[Interrupt], r[Status]);
		r[Interrupt] = i;
		error(Eio);
	}
	if(i)
		r[Interrupt] = i;

	if(!write)
		cacheflush(buf, len);
}

static void
sdhbus(SDio*, int width, int speed)
{
	u32int *r;

	r = sdh.regs;
	switch(width){
	case 1:
		r[Control0] &= ~Dwidth4;
		break;
	case 4:
		r[Control0] |= Dwidth4;
		break;
	}
	if(speed)
		sdhclk(speed);
}

void
sdspacemitlink(void)
{
	static SDio io = {
		"sdspacemit",
		sdhinit,
		sdhenable,
		sdhinquiry,
		sdhcmd,
		sdhiosetup,
		sdhio,
		sdhbus,
	};
	addmmcio(&io);
}
