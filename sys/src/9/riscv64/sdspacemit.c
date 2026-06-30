/*
 * SpacemiT K1 SD host (eMMC at 0xd4281000, microSD at 0xd4280000)
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
	V3_3			= 7<<9,
	Buspower		= 1<<8,
	Dwidth8			= 1<<5,	/* SDHCI v3 extended 8-bit data width */
	Hispeed			= 1<<2,
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
	Wrcrctoken	= 1<<31,	/* Spacemit: write CRC status token bad */
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
	Cardremove	= 1<<7,
	Cardinsert	= 1<<6,
	Readrdy		= 1<<5,
	Writerdy	= 1<<4,
	Dmaintr		= 1<<3,
	Datadone	= 1<<1,
	Cmddone		= 1<<0,

	/* Status */
	Present		= 1<<18,
	Cardinserted	= 1<<16,	/* SDHCI Present State: card inserted (level) */
	Bufread		= 1<<11,
	Bufwrite	= 1<<10,
	Readtrans	= 1<<9,
	Writetrans	= 1<<8,
	Datactive	= 1<<2,
	Datinhibit	= 1<<1,
	Cmdinhibit	= 1<<0,

	/* K1 SoC */
	SDH_SIZE	= 0x200,
	SD_PHYS		= 0xd4280000UL,
	SD_IRQ		= 99,
	EMMC_PHYS	= 0xd4281000UL,
	EMMC_IRQ	= 101,
	SDH_EXTCLK	= 102400000,	/* pll1_d6_409p6 / 4 (u-boot default) */

	APMU_PHYS	= 0xd4282800UL,
	APMU_SIZE	= 0x400,

	/*
	 * BPI-F3 microSD card-detect is GPIO 80 (cd-inverted in DT), not
	 * the SDHCI Cardinserted level bit — that bit reads as 1 stuck on
	 * this board and doesn't track physical insertion. K1 GPIO bank 2
	 * (GPIOs 64..95) lives at controller base + 0x8 with GPLR at +0;
	 * GPIO 80 is bit 16 within the bank.
	 */
	GPIO_PHYS	= 0xd4019000UL,
	GPIO_SIZE	= 0x800,
	GPIO_BPIF3_SDCD_OFF = 0x8,	/* bank 2 GPLR offset within controller */
	GPIO_BPIF3_SDCD_BIT = 16,	/* GPIO 80 mod 32 */
	APMU_SDH0	= 0x054,	/* shared SDH_AXI + per-SDH0 bits */
	APMU_SDH2	= 0x0e0,	/* per-SDH2 bits */
	Sdh_axi_gate	= 1<<3,
	Sdh_axi_rst	= 1<<0,
	Sdh_gate	= 1<<11,
	Sdh_rst		= 1<<1,

	Cbsz		= 64,	/* X60/C908 D-cache line size */
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	/* per-controller config */
	uintptr	regbase;
	u32int	apmuoff;
	int	irq;
	ulong	extclk;

	/* runtime state */
	char	*name;		/* io->name, copied at sdhinit for IRQ-side prints */
	Rendez	r;
	u32int	*regs;
	int	datadone;
	u32int	errstat;	/* Status snapshot at IRQ when Err latched */
	ulong	clock;		/* most recent programmed SDCLK in Hz */
	int	hispeed;	/* Hispeed bit set when clock was programmed */
	int	cardpresent;	/* 1 if Status & Cardinserted at last edge or init */
	ulong	cardevents;	/* total Cardinsert|Cardremove transitions seen */
	ulong	nerr;		/* total error interrupts seen */
	ulong	ntmo;		/* total Datadone timeouts seen */
};

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
interrupt(Ureg*, void *arg)
{
	Ctlr *c;
	u32int *r;
	int i;

	c = arg;
	r = c->regs;
	i = r[Interrupt];
	if(i & Err)
		c->errstat = r[Status];	/* snapshot before ack races state */
	if(i & (Cardinsert|Cardremove)){
		int now = (r[Status] & Cardinserted) != 0;
		if(now != c->cardpresent){
			c->cardevents++;
			c->cardpresent = now;
			print("%s: card %s (event %lud, stat %ux)\n",
				c->name? c->name: "sd",
				now? "inserted": "removed",
				c->cardevents, r[Status]);
		}
	}
	r[Interrupt] = i & (Datadone|Err|Cardinsert|Cardremove);
	c->datadone = i & (Datadone|Err);	/* don't wake I/O on a card event */
	if(c->datadone)
		wakeup(&c->r);
}

static void
fmtintr(char *buf, int buflen, u32int i)
{
	static const struct { u32int bit; char *name; } b[] = {
		{Wrcrctoken, "Wrcrctoken"},
		{Acmderr, "Acmderr"},
		{Denderr, "Denderr"},
		{Dcrcerr, "Dcrcerr"},
		{Dtoerr, "Dtoerr"},
		{Cbaderr, "Cbaderr"},
		{Cenderr, "Cenderr"},
		{Ccrcerr, "Ccrcerr"},
		{Ctoerr, "Ctoerr"},
		{Err, "Err"},
		{Cardintr, "Cardintr"},
		{Cardremove, "Cardremove"},
		{Cardinsert, "Cardinsert"},
		{Readrdy, "Readrdy"},
		{Writerdy, "Writerdy"},
		{Dmaintr, "Dmaintr"},
		{Datadone, "Datadone"},
		{Cmddone, "Cmddone"},
	};
	char *p, *e;
	int k, first;

	p = buf;
	e = buf + buflen;
	first = 1;
	for(k = 0; k < nelem(b); k++){
		if(i & b[k].bit){
			p = seprint(p, e, "%s%s", first? "":"|", b[k].name);
			first = 0;
		}
	}
	if(first)
		seprint(p, e, "none");
}

static int
datadone(void *arg)
{
	Ctlr *c = arg;
	return c->datadone;
}

static int
sdhinit(SDio *io)
{
	Ctlr *c;
	u32int *r, *apmu;
	int i;

	c = io->aux;
	c->name = io->name;

	apmu = vmap(APMU_PHYS, APMU_SIZE);
	apmu[APMU_SDH0>>2] |= Sdh_axi_gate | Sdh_axi_rst;
	apmu[c->apmuoff>>2] |= Sdh_gate | Sdh_rst;
	coherence();
	vunmap(apmu, APMU_SIZE);

	r = vmap(c->regbase, SDH_SIZE);
	c->regs = r;
	r[Control1] = Srsthc;
	for(i = 0; i < 100; i++){
		delay(10);
		if((r[Control1] & Srsthc) == 0)
			break;
	}
	if(r[Control1] & Srsthc){
		print("%s: reset timeout!\n", io->name);
		return -1;
	}

	/* K1 PHY bring-up after host reset */
	r[Sdhcphyctl] |= Phyfuncen | Phyplllock;
	r[Sdhcphypadcfg] = (r[Sdhcphypadcfg] & ~7) | Phydrivesel | Rxbiasctl;
	r[Sdhcmmcctl] |= Mmccardmode;
	r[Sdhclegctl] |= Genpadclkon;
	r[Sdhctxcfg] |= Txintclksel;

	/* Srsthc cleared Power Control; restore 3.3V bus power. */
	r[Control0] = V3_3 | Buspower | Dwidth1;

	/*
	 * eMMC is soldered and non-removable; for the removable slots
	 * presence comes from a board-specific GPIO read in sdhenable.
	 */
	c->cardpresent = 1;

	return 0;
}

static int
sdhinquiry(SDio *io, char *inquiry, int inqlen)
{
	Ctlr *c = io->aux;
	uint ver;

	ver = c->regs[Slotisrver] >> 16;
	return snprint(inquiry, inqlen,
		"SpacemiT %s Host Controller %2.2x Version %2.2x",
		io->name, ver&0xFF, ver>>8);
}

static void
sdhclk(Ctlr *c, uint freq)
{
	u32int *r;
	uint d;
	int i;

	/*
	 * The simple divisor formula (extclk/freq - 1) picks d=1 for
	 * freq=50MHz on extclk=102.4MHz, which runs the wire at 51.2MHz —
	 * inside HS card spec but at this controller's PHY limit, producing
	 * Wrcrctoken|Dcrcerr on every write block. For HS speeds choose the
	 * smallest d such that actual <= freq, giving 25.6MHz at request 50M
	 * and clean writes. Keep the old formula for sub-MHz init speeds —
	 * the spec-true 400KHz makes repeated cardinit attempts unstable on
	 * this hardware; 200KHz (from the old formula) is reliable.
	 */
	if(freq >= 26000000){
		d = (c->extclk + 2*freq - 1) / (2*freq);
		if(d > 0x3FF)
			d = 0x3FF;
	}else
		d = c->extclk / freq - 1;

	r = c->regs;
	r[Control1] &= ~Clken;
	r[Control1] = clkdiv(d) |
			DTO << Datatoshift | Clkgendiv | Clkintlen;
	for(i = 0; i < 1000; i++){
		delay(1);
		if(r[Control1] & Clkstable)
			break;
	}
	if((r[Control1] & Clkstable) == 0){
		print("SD clock won't initialise!\n");
		return;
	}
	r[Control1] |= Clken;
	c->clock = d == 0? c->extclk : c->extclk / (2*d);
}

/*
 * The K1 controller updates the Cardinserted level bit in Present State
 * when the card-detect line changes but doesn't seem to raise the
 * matching Cardinsert/Cardremove edge IRQs — we leave the IRQ-side
 * logic in place in case it ever does, and poll the level bit from a
 * kproc as the actually-reliable detector for removable slots.
 */
static u32int *gpioregs;	/* shared GPIO controller mapping for cd polling */

static int
sdcardpresentbpif3(void)
{
	u32int gplr;

	if(gpioregs == nil)
		return 1;
	gplr = gpioregs[GPIO_BPIF3_SDCD_OFF>>2];
	/*
	 * DT lists the cd-gpio as ACTIVE_HIGH then applies cd-inverted, which
	 * net-net means active-low: card present when GPIO line is LOW.
	 * Confirmed by reading gplr2 = 0x1c067c0e (bit 16 = 0) with the
	 * Linux microSD seated.
	 */
	return (gplr & (1<<GPIO_BPIF3_SDCD_BIT)) == 0;
}

static void
cardpollproc(void *arg)
{
	Ctlr *c = arg;
	int now;

	for(;;){
		tsleep(&up->sleep, return0, nil, 500);
		now = sdcardpresentbpif3();
		if(now != c->cardpresent){
			c->cardevents++;
			c->cardpresent = now;
			print("%s: card %s (event %lud)\n",
				c->name, now? "inserted": "removed",
				c->cardevents);
		}
	}
}

static void
sdhenable(SDio *io)
{
	Ctlr *c = io->aux;

	sdhclk(c, 400000);
	c->regs[Irptmask] = ~(Dtoerr|Cardintr|Dmaintr);
	c->regs[Irpten] = Cardinsert|Cardremove;	/* baseline signal mask */
	intrenable(c->irq, interrupt, c, BUSUNKNOWN, io->name);

	/*
	 * eMMC is soldered; the microSD's "Cardinserted" status bit lies on
	 * this board so we read GPIO 80 directly. Map the GPIO controller
	 * lazily on first removable-slot enable.
	 */
	if(strcmp(io->name, "emmc") == 0)
		return;
	if(gpioregs == nil)
		gpioregs = vmap(GPIO_PHYS, GPIO_SIZE);
	c->cardpresent = sdcardpresentbpif3();
	kproc("cardpoll", cardpollproc, c);
}

static int
sdhcmd(SDio *io, SDiocmd *cmd, u32int arg, u32int *resp)
{
	Ctlr *ct = io->aux;
	ulong now;
	u32int *r;
	u32int c;
	int i;

	/*
	 * Refuse commands when no card is present so that sdmmc.c's existing
	 * media-change-via-Eio path in mmconline (waserror -> unit->sectors=0)
	 * resets cached card state. The next I/O after a swap then re-runs
	 * cardinit on the new card.
	 */
	if(ct->cardpresent == 0)
		error(Eio);

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

	r = ct->regs;
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
			print("%s: before command, intr was %ux\n", io->name, i);
		r[Interrupt] = i;
	}
	r[Cmdtm] = c;
	now = m->ticks;
	while(((i=r[Interrupt])&(Cmddone|Err)) == 0)
		if((long)(m->ticks-now) > HZ)
			break;
	if((i&(Cmddone|Err)) != Cmddone){
		if((i&~Err) != Ctoerr)
			print("%s: cmd %ux error intr %ux stat %ux\n",
				io->name, c, i, r[Status]);
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
		r[Irpten] = Datadone|Err|Cardinsert|Cardremove;
		tsleep(&ct->r, datadone, ct, 3000);
		i = ct->datadone;
		ct->datadone = 0;
		r[Irpten] = Cardinsert|Cardremove;
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
sdhiosetup(SDio *io, int, void *buf, int bsize, int bcount)
{
	Ctlr *c = io->aux;
	u32int *r;
	uintptr pa;
	int len;

	len = bsize*bcount;
	if(len > (0x1000<<7))
		error(Etoobig);

	pa = PADDR(buf);
	cacheflush(buf, len);

	r = c->regs;
	r[Sysaddr] = pa;
	r[Blksizecnt] = 7<<12 | bcount<<16 | bsize;
	r[Irpten] = Datadone|Err|Cardinsert|Cardremove;
}

static void
sdhio(SDio *io, int write, uchar *buf, int len)
{
	Ctlr *c = io->aux;
	u32int *r;
	int i;

	tsleep(&c->r, datadone, c, 3000);
	i = c->datadone;
	c->datadone = 0;

	r = c->regs;
	r[Irpten] = Cardinsert|Cardremove;
	if((i & Datadone) == 0){
		c->ntmo++;
		print("sdhio: %d timeout intr %ux stat %ux clk %lud hs %d ntmo %lud\n",
			write, i, r[Status], c->clock, c->hispeed, c->ntmo);
		r[Interrupt] = i;
		error(Eio);
	}
	if(i & Err){
		char buf[128];
		c->nerr++;
		fmtintr(buf, sizeof buf, i);
		print("sdhio: %d err [%s] intr %ux prestat %ux poststat %ux clk %lud hs %d nerr %lud\n",
			write, buf, i, c->errstat, r[Status],
			c->clock, c->hispeed, c->nerr);
		r[Interrupt] = i;
		error(Eio);
	}
	if(i)
		r[Interrupt] = i;

	if(!write)
		cacheflush(buf, len);
}

static void
sdhbus(SDio *io, int width, int speed)
{
	Ctlr *c = io->aux;
	u32int *r;

	r = c->regs;
	switch(width){
	case 1:
		r[Control0] &= ~(Dwidth4|Dwidth8);
		break;
	case 4:
		r[Control0] = (r[Control0] & ~Dwidth8) | Dwidth4;
		break;
	case 8:
		r[Control0] |= Dwidth8;
		break;
	}
	if(speed){
		if(speed > 26000000){
			r[Control0] |= Hispeed;
			c->hispeed = 1;
		}else{
			r[Control0] &= ~Hispeed;
			c->hispeed = 0;
		}
		sdhclk(c, speed);
	}
}

void
sdspacemitlink(void)
{
	static Ctlr emmc = {
		.regbase = EMMC_PHYS,
		.apmuoff = APMU_SDH2,
		.irq	 = EMMC_IRQ,
		.extclk	 = SDH_EXTCLK,
	};
	static Ctlr sd = {
		.regbase = SD_PHYS,
		.apmuoff = APMU_SDH0,
		.irq	 = SD_IRQ,
		.extclk	 = SDH_EXTCLK,
	};
	static SDio io_emmc = {
		.name	 = "emmc",
		.init	 = sdhinit,
		.enable	 = sdhenable,
		.inquiry = sdhinquiry,
		.cmd	 = sdhcmd,
		.iosetup = sdhiosetup,
		.io	 = sdhio,
		.bus	 = sdhbus,
		.aux	 = &emmc,
	};
	static SDio io_sd = {
		.name	 = "sd",
		.init	 = sdhinit,
		.enable	 = sdhenable,
		.inquiry = sdhinquiry,
		.cmd	 = sdhcmd,
		.iosetup = sdhiosetup,
		.io	 = sdhio,
		.bus	 = sdhbus,
		.aux	 = &sd,
	};
	addmmcio(&io_emmc);
	addmmcio(&io_sd);
}
