/*
 * Cadence SD4HC eMMC host on the Microchip PolarFire SoC: a standard
 * SDHCI register set at offset 0x200, driven by ../port/sdmmc.c.
 * The U54s cannot clean or invalidate the cache by address, so SDMA
 * goes through a bounce buffer in the non-cached DDR window.
 * Read-only for now: writes fail.
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
	Mmcbase		= 0x20008000,
	Mmcirq		= 88,
	Srsoff		= 0x200,	/* SDHCI registers */
	Bouncesz	= 512*KiB,	/* the SDMA boundary, so never crossed */
	Maxclk		= 25000000,	/* legacy timing only, for now */

	/* SDHCI registers, as u32int indices from Srsoff */
	Sysaddr		= 0x00>>2,
	Blksizecnt	= 0x04>>2,
	Arg1		= 0x08>>2,
	Cmdtm		= 0x0c>>2,
	Resp0		= 0x10>>2,
	Resp1		= 0x14>>2,
	Resp2		= 0x18>>2,
	Resp3		= 0x1c>>2,
	Status		= 0x24>>2,
	Control0	= 0x28>>2,
	Control1	= 0x2c>>2,
	Interrupt	= 0x30>>2,
	Irptmask	= 0x34>>2,
	Irpten		= 0x38>>2,
	Capabilities	= 0x40>>2,
	Slotisrver	= 0xfc>>2,

	/* Control0 */
	Powermask	= 0xf<<8,	/* bus voltage and power */
	V3v3		= 7<<9,
	Buspower	= 1<<8,
	Dwidth8		= 1<<5,
	Dmasdma		= 0<<3,
	Hispeed		= 1<<2,
	Dwidth4		= 1<<1,

	/* Control1 */
	Srstdata	= 1<<26,
	Srstcmd		= 1<<25,
	Srsthc		= 1<<24,
	Datatoshift	= 16,
	Dto		= 14,
	Clkfreq8shift	= 8,
	Clkfreqms2shift	= 6,
	Clken		= 1<<2,
	Clkstable	= 1<<1,
	Clkintlen	= 1<<0,

	/* Cmdtm */
	Indexshift	= 24,
	Isdata		= 1<<21,
	Ixchken		= 1<<20,
	Crcchken	= 1<<19,
	Respmask	= 3<<16,
	Respnone	= 0<<16,
	Resp136		= 1<<16,
	Resp48		= 2<<16,
	Resp48busy	= 3<<16,
	Multiblock	= 1<<5,
	Card2host	= 1<<4,
	Blkcnten	= 1<<1,
	Dmaen		= 1<<0,

	/* Interrupt */
	Ctoerr		= 1<<16,
	Err		= 1<<15,
	Cardintr	= 1<<8,
	Cardinsert	= 1<<6,
	Readrdy		= 1<<5,
	Writerdy	= 1<<4,
	Dmaintr		= 1<<3,
	Datadone	= 1<<1,
	Cmddone		= 1<<0,
	Dtoerr		= 1<<20,

	/* Status */
	Datinhibit	= 1<<1,
	Cmdinhibit	= 1<<0,
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	u32int	*regs;
	int	irq;
	ulong	baseclk;
	uchar	*bounce;
	Rendez	r;
	int	datadone;
};

static Ctlr ctlr;

static void
interrupt(Ureg*, void*)
{
	u32int *r;
	int i;

	r = ctlr.regs;
	i = r[Interrupt];
	r[Interrupt] = i & (Datadone|Err);
	ctlr.datadone = i;
	wakeup(&ctlr.r);
}

static int
datadone(void*)
{
	return ctlr.datadone;
}

static void
resetwait(u32int *r, u32int bit)
{
	int i;

	r[Control1] |= bit;
	for(i = 0; i < 1000 && (r[Control1] & bit) != 0; i++)
		microdelay(10);
}

static int
mpfsinit(SDio*)
{
	u32int *r, pwr, cap;
	int i;

	ctlr.irq = Mmcirq;
	r = vmap(Mmcbase, 0x1000);
	if(r == nil)
		return -1;
	r += Srsoff/4;
	ctlr.regs = r;

	/* the HSS powered the eMMC: keep its bus voltage across the reset */
	pwr = r[Control0] & Powermask;
	if(pwr == 0)
		pwr = V3v3 | Buspower;
	r[Control1] = Srsthc;
	for(i = 0; i < 100; i++){
		delay(10);
		if((r[Control1] & Srsthc) == 0)
			break;
	}
	if(i == 100){
		print("sdmpfs: reset timeout\n");
		return -1;
	}
	r[Control0] = pwr | Dmasdma;
	cap = r[Capabilities];
	ctlr.baseclk = (cap>>8 & 0xff) * 1000000;
	if(ctlr.baseclk == 0)
		ctlr.baseclk = 200000000;
	ctlr.bounce = ncalloc(Bouncesz, Bouncesz);
	print("sdmpfs: %#ux irq %d base clock %lud Hz\n", Mmcbase, ctlr.irq, ctlr.baseclk);
	return 0;
}

static int
mpfsinquiry(SDio*, char *inquiry, int inqlen)
{
	uint ver;

	ver = ctlr.regs[Slotisrver] >> 16;
	return snprint(inquiry, inqlen,
		"Cadence SD4HC eMMC Host Controller %2.2x Version %2.2x",
		ver&0xff, ver>>8);
}

/* SDHCI v3 10-bit divided clock: f = base / 2n, or base for n = 0 */
static void
mpfsclk(uint freq)
{
	u32int *r;
	uint n;
	int i;

	if(freq > Maxclk)
		freq = Maxclk;
	n = (ctlr.baseclk + 2*freq - 1) / (2*freq);
	if(n > 0x3ff)
		n = 0x3ff;
	r = ctlr.regs;
	r[Control1] &= ~Clken;
	r[Control1] = (n & 0xff)<<Clkfreq8shift | (n>>8 & 3)<<Clkfreqms2shift
		| Dto<<Datatoshift | Clkintlen;
	for(i = 0; i < 1000; i++){
		if(r[Control1] & Clkstable)
			break;
		delay(1);
	}
	if(i == 1000)
		print("sdmpfs: clock won't stabilise\n");
	r[Control1] |= Clken;
	delay(1);
}

static void
mpfsenable(SDio *io)
{
	mpfsclk(400000);
	ctlr.regs[Irptmask] = ~(Dtoerr|Cardintr|Dmaintr);
	intrenable(ctlr.irq, interrupt, nil, BUSUNKNOWN, io->name);
}

static int
mpfscmd(SDio*, SDiocmd *cmd, u32int arg, u32int *resp)
{
	ulong now;
	u32int *r, c;
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
		c |= Isdata | Dmaen;
		if(cmd->data & 1)
			c |= Card2host;
		if(cmd->data > 2)
			c |= Multiblock | Blkcnten;
	}

	r = ctlr.regs;
	if(r[Status] & Cmdinhibit)
		resetwait(r, Srstcmd);
	if((c & Isdata || (c & Respmask) == Resp48busy) && r[Status] & Datinhibit)
		resetwait(r, Srstdata);
	r[Arg1] = arg;
	if((i = r[Interrupt]) != 0)
		r[Interrupt] = i;
	r[Cmdtm] = c;
	now = m->ticks;
	while(((i = r[Interrupt]) & (Cmddone|Err)) == 0)
		if((long)(m->ticks - now) > HZ)
			break;
	if((i & (Cmddone|Err)) != Cmddone){
		if((i & ~Err) != Ctoerr)
			print("sdmpfs: %s error intr %ux stat %ux\n", cmd->name, i, r[Status]);
		r[Interrupt] = i;
		if(r[Status] & Cmdinhibit)
			resetwait(r, Srstcmd);
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
		tsleep(&ctlr.r, datadone, 0, 3000);
		i = ctlr.datadone;
		ctlr.datadone = 0;
		r[Irpten] = 0;
		if((i & Datadone) == 0)
			print("sdmpfs: no Datadone after %s\n", cmd->name);
		if(i & Err)
			print("sdmpfs: %s error interrupt %ux\n", cmd->name, r[Interrupt]);
		r[Interrupt] = i;
	}
	return 0;
}

static void
mpfsiosetup(SDio*, int write, void*, int bsize, int bcount)
{
	u32int *r;

	if(write)
		error("eMMC is read-only");
	if(bsize*bcount > Bouncesz)
		error(Etoobig);
	r = ctlr.regs;
	r[Sysaddr] = PADDR(ctlr.bounce);
	r[Blksizecnt] = 7<<12 | bcount<<16 | bsize;
	r[Irpten] = Datadone|Err;
}

static void
mpfsio(SDio*, int write, uchar *buf, int len)
{
	u32int *r;
	int i;

	tsleep(&ctlr.r, datadone, 0, 3000);
	i = ctlr.datadone;
	ctlr.datadone = 0;
	r = ctlr.regs;
	r[Irpten] = 0;
	if((i & Datadone) == 0){
		print("sdmpfs: %d timeout intr %ux stat %ux\n", write, i, r[Status]);
		r[Interrupt] = i;
		error(Eio);
	}
	if(i & Err){
		print("sdmpfs: %d error intr %ux stat %ux\n", write, r[Interrupt], r[Status]);
		r[Interrupt] = i;
		error(Eio);
	}
	if(i)
		r[Interrupt] = i;
	if(!write)
		memmove(buf, ctlr.bounce, len);
}

static void
mpfsbus(SDio*, int width, int speed)
{
	u32int *r;

	r = ctlr.regs;
	switch(width){
	case 1:
		r[Control0] &= ~(Dwidth4|Dwidth8);
		break;
	case 4:
		r[Control0] = r[Control0] & ~Dwidth8 | Dwidth4;
		break;
	case 8:
		r[Control0] = r[Control0] & ~Dwidth4 | Dwidth8;
		break;
	}
	if(speed)
		mpfsclk(speed);
}

void
sdmpfslink(void)
{
	static SDio io = {
		"sdmpfs",
		mpfsinit,
		mpfsenable,
		mpfsinquiry,
		mpfscmd,
		mpfsiosetup,
		mpfsio,
		mpfsbus,
	};

	addmmcio(&io);
}
