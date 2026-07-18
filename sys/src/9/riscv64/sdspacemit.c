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
	/*
	 * the upper half of Irpten is the error-signal enable; the Err
	 * summary bit itself is fixed to zero in the enable registers, so
	 * an error only raises the IRQ if its individual bit is set here.
	 */
	Anyerr		= Wrcrctoken|Acmderr|Denderr|Dcrcerr|Dtoerr|
			  Cbaderr|Cenderr|Ccrcerr|Ctoerr,
	Cardintr	= 1<<8,
	Cardremove	= 1<<7,
	Cardinsert	= 1<<6,
	Readrdy		= 1<<5,
	Writerdy	= 1<<4,
	Dmaintr		= 1<<3,
	Datadone	= 1<<1,
	Cmddone		= 1<<0,

	/* Status */
	Dat0line	= 1<<20,	/* DAT[0] level; low while the card is busy */
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
	SDH_EXTCLK	= 102400000,	/* fallback when APMU decode fails */

	APMU_PHYS	= 0xd4282800UL,
	APMU_SIZE	= 0x400,

	/*
	 * pad configuration: pins 104-110 sit at (pin+6)*4 from the pad
	 * controller base; drive strength is a 3-bit field at bits 12:10,
	 * code 1 = 19mA on a 3.3V pad.  The BPI-F3 device tree runs all
	 * six mmc1 pads (d3 d2 d1 d0 cmd clk = pins 104-109) at 19mA.
	 */
	AIB_PHYS	= 0xd401e000UL,
	AIB_SIZE	= 0x1000,
	PAD_MMC1	= (104+6)*4,	/* first mmc1 pad register */
	NMMC1PADS	= 6,
	Paddrivemask	= 7<<10,
	Paddrive19ma	= 1<<10,

	/*
	 * each pad group also has a power-domain register selecting 1.8V
	 * or 3.3V pad mode; the mmc group's is at 0x81c from the pad
	 * controller base, bit 2 set meaning 1.8V.  These registers are
	 * key-protected: each access must be preceded by the 0xbaba /
	 * 0xeb10 magic writes to the APBC access-security registers.
	 */
	IOPD_MMC	= 0x81c,
	 Iopdv18en	= 1<<2,
	APBC_PHYS	= 0xd4015000UL,
	APBC_SIZE	= 0x100,
	APBC_ASFAR	= 0x50,
	APBC_ASSAR	= 0x54,

	/*
	 * the slot's pad rail vqmmc comes from the power-management
	 * chip's aldo1: enable bit 0 at register 0x5b, 7-bit voltage
	 * selector at 0x5c, 500mV + (sel-11)*25mV.
	 */
	PMIC_ALDO1EN	= 0x5b,
	PMIC_ALDO1V	= 0x5c,
	Aldosel3v3	= 123,
	Aldoselmask	= 0x7f,

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
	ushort	rca;		/* SD relative address, stashed for the probes */
	int	datadone;
	u32int	errstat;	/* Status snapshot at IRQ when Err latched */
	ulong	clock;		/* most recent programmed SDCLK in Hz */
	int	hispeed;	/* Hispeed bit set when clock was programmed */
	int	cardpresent;	/* 1 if Status & Cardinserted at last edge or init */
	ulong	cardevents;	/* total Cardinsert|Cardremove transitions seen */
	ulong	nerr;		/* total error interrupts seen */
	ulong	ntmo;		/* total Datadone timeouts seen */
	ulong	ncto;		/* total command timeouts, reported silently */
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
	r[Interrupt] = i & ~Cmddone;	/* sdhcmd polls for and acks Cmddone */
	if(i & Dmaintr){
		/*
		 * SDMA pauses at every boundary from Blksizecnt bits 12-14
		 * (512K here) and won't move again until Sysaddr, which now
		 * reads back as the next address, is rewritten.
		 */
		r[Sysaddr] = r[Sysaddr];
	}
	if(i & (Datadone|Err)){		/* don't wake I/O on a card event */
		c->datadone |= i;
		wakeup(&c->r);
	}
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

/*
 * reset whichever circuits a failed transfer left busy, so the
 * following STOP_TRANSMISSION and retry start from a clean controller.
 */
static void
sdhabort(Ctlr *c, int write)
{
	u32int *r = c->regs;
	int i;

	if(r[Status] & (Cmdinhibit|Datinhibit)){
		r[Control1] |= Srstcmd|Srstdata;
		while(r[Control1] & (Srstcmd|Srstdata))
			;
	}

	/*
	 * a card that rejected a write block holds DAT0 low while it
	 * finishes internally and can ignore commands until it is done;
	 * give busy a bounded chance to release before the recovery STOP.
	 */
	if(write)
		for(i = 0; i < 500 && (r[Status] & Dat0line) == 0; i++)
			tsleep(&up->sleep, return0, nil, 1);
}

/*
 * SDCLK is a PLL tap selected and divided by fields in the same APMU
 * register as the gate/reset bits: source select in bits 7:5,
 * divisor-1 in bits 10:8 (PMU_SDHx_CLK_RES_CTRL in the K1 user
 * manual).  Decode what u-boot programmed rather than trusting a
 * compiled-in rate.
 */
static ulong
sdhbaseclk(Ctlr *c, u32int v)
{
	static ulong parent[8] = {
		409600000,	/* pll1_d6 */
		614400000,	/* pll1_d4 */
		375000000,	/* pll2_d8 */
		0,		/* slot-specific, below */
		223418182,	/* pll1_d11 */
		189046154,	/* pll1_d13 */
		106852174,	/* pll1_d23 */
		0,
	};
	ulong f;

	f = parent[(v>>5) & 7];
	if(((v>>5) & 7) == 3)	/* pll1_d3 on the eMMC slot, pll2_d5 on sd */
		f = c->apmuoff == APMU_SDH2? 819200000: 600000000;
	return f / (((v>>8) & 7) + 1);
}

static int
sdhinit(SDio *io)
{
	Ctlr *c;
	u32int *r, *apmu;
	u32int v;
	ulong f;
	int i;

	c = io->aux;
	c->name = io->name;

	apmu = vmap(APMU_PHYS, APMU_SIZE);
	apmu[APMU_SDH0>>2] |= Sdh_axi_gate | Sdh_axi_rst;
	apmu[c->apmuoff>>2] |= Sdh_gate | Sdh_rst;
	coherence();
	v = apmu[c->apmuoff>>2];
	vunmap(apmu, APMU_SIZE);

	f = sdhbaseclk(c, v);
	if(f != 0)
		c->extclk = f;
	print("%s: base clock %lud Hz (clkctl %ux)\n", io->name, c->extclk, v);

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
	if(strcmp(io->name, "emmc") == 0)
		r[Sdhcmmcctl] |= Mmccardmode;
	r[Sdhclegctl] |= Genpadclkon;
	r[Sdhctxcfg] |= Txintclksel;

	/*
	 * u-boot's UHS negotiation with a bootable card leaves the pad
	 * rail vqmmc at 1.8V, and a freshly inserted card signals at
	 * 3.3V: reads still work, but host-driven edges sit right at the
	 * card's input threshold, so writes fail and commands are
	 * sporadically ignored.  Put the rail, the pad group's power
	 * domain (touching only the 1.8V-mode bit; the others are live)
	 * and the pad drive back to what the board expects.
	 */
	if(strcmp(io->name, "emmc") != 0){
		u32int *aib, *apbc;
		u32int w;
		int sel, en;

		sel = k1pmicread(PMIC_ALDO1V);
		if(sel >= 0 && (sel & Aldoselmask) != Aldosel3v3){
			print("%s: vqmmc was %d mV\n", io->name,
				500 + ((sel & Aldoselmask) - 11)*25);
			k1pmicwrite(PMIC_ALDO1V,
				(sel & ~Aldoselmask) | Aldosel3v3);
			delay(5);
		}
		en = k1pmicread(PMIC_ALDO1EN);
		if(en >= 0 && (en & 1) == 0)
			k1pmicwrite(PMIC_ALDO1EN, en | 1);

		aib = vmap(AIB_PHYS, AIB_SIZE);
		if(aib != nil){
			for(i = 0; i < NMMC1PADS; i++)
				aib[(PAD_MMC1>>2) + i] =
					(aib[(PAD_MMC1>>2) + i] & ~Paddrivemask) |
					Paddrive19ma;
			apbc = vmap(APBC_PHYS, APBC_SIZE);
			if(apbc != nil){
				apbc[APBC_ASFAR>>2] = 0xbaba;
				apbc[APBC_ASSAR>>2] = 0xeb10;
				w = aib[IOPD_MMC>>2];
				if(w & Iopdv18en){
					print("%s: pad domain was %ux\n",
						io->name, w);
					apbc[APBC_ASFAR>>2] = 0xbaba;
					apbc[APBC_ASSAR>>2] = 0xeb10;
					aib[IOPD_MMC>>2] = w & ~Iopdv18en;
				}
				vunmap(apbc, APBC_SIZE);
			}
			coherence();
			vunmap(aib, AIB_SIZE);
		}
	}

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
	 * For 26MHz and up choose the smallest divisor that keeps the
	 * wire at or below the request.  Identification speeds keep the
	 * older formula, which lands near half the request: cards must
	 * accept 100-400KHz, so ~200KHz is always legal and reliable.
	 */
	if(freq >= 26000000)
		d = (c->extclk + 2*freq - 1) / (2*freq);
	else
		d = c->extclk / freq - 1;
	if(d > 0x3FF)
		d = 0x3FF;

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
	c->regs[Irptmask] = ~(Dtoerr|Cardintr);
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

static int sdhcmd(SDio*, SDiocmd*, u32int, u32int*);

static int
sdhtrystatus(SDio *io, u32int *st)
{
	static SDiocmd status = { 13, 1, 0, 0, "SEND_STATUS" };
	Ctlr *c = io->aux;
	u32int r[4];

	if(waserror())
		return -1;
	sdhcmd(io, &status, (u32int)c->rca<<16, r);
	poperror();
	if(st != nil)
		*st = r[0];
	return 0;
}

/*
 * a card that ignores a transfer command usually isn't deaf but in a
 * state where that command is illegal, and it answers CMD13 from any
 * state: the R1 reply carries the current state in bits 12:9 (4 tran,
 * 5 data, 6 rcv, 7 prg, 3 stby, 8 dis) and, in bit 22, an
 * illegal-command flag for the command it ignored.
 */
static void
sdhprobe(SDio *io, char *failed)
{
	u32int st;

	if(sdhtrystatus(io, &st) == 0)
		print("%s: %s unanswered but CMD13 ok: status %ux state %ud%s\n",
			io->name, failed, st, (st>>9) & 0xF,
			st & (1<<22)? " (illegal cmd)": "");
	else
		print("%s: %s unanswered, CMD13 silent too\n",
			io->name, failed);
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
		if((i&~Err) == Ctoerr){
			ct->ncto++;
			switch(cmd->index){
			case 1:		/* SEND_OP_COND */
			case 8:		/* SD_SEND_IF_COND */
			case 41:	/* SD_SEND_OP_COND */
			case 55:	/* APP_CMD */
				break;	/* card-type probing; timeouts expected */
			default:
				print("%s: %s timeout stat %ux ncto %lud\n",
					io->name, cmd->name, r[Status], ct->ncto);
			}
		}else
			print("%s: cmd %ux error intr %ux stat %ux\n",
				io->name, c, i, r[Status]);
		r[Interrupt] = i;
		if(r[Status]&Cmdinhibit){
			r[Control1] |= Srstcmd;
			while(r[Control1]&Srstcmd)
				;
		}
		if((i&~Err) == Ctoerr)
		switch(cmd->index){
		case 12:
		case 18:
		case 25:
			sdhprobe(io, cmd->name);
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
	if(cmd->index == 3 && cmd->resp == 6)	/* SEND_RELATIVE_ADDR */
		ct->rca = resp[0] >> 16;
	if((c & Respmask) == Resp48busy){
		r[Irpten] = Anyerr|Datadone|Err|Cardinsert|Cardremove;
		tsleep(&ct->r, datadone, ct, 3000);
		i = ct->datadone;
		ct->datadone = 0;
		r[Irpten] = Cardinsert|Cardremove;
		if((i & Datadone) == 0)
			print("%s: no Datadone after %s\n", io->name, cmd->name);
		if(i & Err)
			print("%s: %s error interrupt %ux\n",
				io->name, cmd->name, i);
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

	/*
	 * transfers bigger than the 512K SDMA boundary are fine now that
	 * the interrupt handler resumes the pause at each crossing; the
	 * remaining limit is the 16-bit block count field in Blksizecnt.
	 */
	len = bsize*bcount;
	if(bcount > 0xFFFF)
		error(Etoobig);

	pa = PADDR(buf);
	cacheflush(buf, len);

	r = c->regs;
	r[Sysaddr] = pa;
	r[Blksizecnt] = 7<<12 | bcount<<16 | bsize;
	r[Irpten] = Dmaintr|Datadone|Err|Cardinsert|Cardremove;
}

static void
sdhio(SDio *io, int write, uchar *buf, int len)
{
	Ctlr *c = io->aux;
	u32int *r;
	int i;

	r = c->regs;
	r[Irpten] = Anyerr|Dmaintr|Datadone|Err|Cardinsert|Cardremove;
	tsleep(&c->r, datadone, c, 3000);
	i = c->datadone;
	c->datadone = 0;

	r[Irpten] = Cardinsert|Cardremove;
	if(i & Err){
		char buf[128];
		c->nerr++;
		fmtintr(buf, sizeof buf, i);
		print("%s: %d err [%s] intr %ux prestat %ux poststat %ux clk %lud hs %d nerr %lud ncto %lud\n",
			io->name, write, buf, i, c->errstat, r[Status],
			c->clock, c->hispeed, c->nerr, c->ncto);
		sdhabort(c, write);
		error(Eio);
	}
	if((i & Datadone) == 0){
		c->ntmo++;
		print("%s: %d timeout intr %ux/%ux stat %ux sysaddr %ux clk %lud hs %d ntmo %lud ncto %lud\n",
			io->name, write, i, r[Interrupt], r[Status], r[Sysaddr],
			c->clock, c->hispeed, c->ntmo, c->ncto);
		r[Interrupt] = r[Interrupt] & ~Cmddone;
		sdhabort(c, write);
		error(Eio);
	}

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
	/*
	 * All the addresses here (SDHCI, APMU, AIB, PMIC) are the K1's;
	 * sdhinit writes them unconditionally, so don't register on any
	 * other SoC — the probe itself faults or hangs the bus there.
	 */
	if(!dtbsocmatch("spacemit,k1"))
		return;
	addmmcio(&io_emmc);
	addmmcio(&io_sd);
}
