/*
 * 8250/16550 UART for riscv64.  Register layer adapted from
 * sys/src/9/mt7688/uarti8250.c; multi-port discovery via pnp()
 * returning a chained Uart list follows sys/src/9/pc/uartpci.c.
 * DTB-driven enumeration uses the table built by bootargs.c
 * (DTB walker derived from sys/src/9/bcm/bootargs.c).
 *
 * Every node under /soc whose name matches serial@/uart@/r_uart
 * and whose status = "okay" becomes an eia<N> port, numbered in
 * /aliases serial<N> order.  The port that /chosen/stdout-path
 * resolves to is marked as the kernel console.
 *
 * SoC-specific quirks are gated on the DT "compatible" string.
 * For "spacemit,*" parts (SpacemiT-family SoCs with the PXA-derived
 * UART block — BPI-F3's K1 today, K3 once silicon arrives) we
 * reprogram FCR/IER with the bit patterns the PXA-derived RX
 * datapath requires: FIFO+8-byte trigger and the unit-enable bit
 * plus the receive-line/data/timeout interrupt enables.  See the
 * Marvell PXA TRM, "UART chapter", which the SpacemiT silicon
 * inherits register-for-register.
 *
 * The console UART runs interrupt-driven via intrenable() when its
 * DTB irq is known; other UARTs and IRQ-less ports stay on the
 * 100-tick clock-callback polling path.  (Secondary UARTs are gated
 * off the IRQ path until we confirm their power/clock state is sane
 * — a mis-clocked port returning 0x00 from IIR would otherwise storm
 * the PLIC.)  plic.c is now driven by soc.plic — set preferentially
 * from the DTB by bootargsinit() and otherwise from a vendor-keyed
 * default in socidentify() — so the chip-level PLIC is properly
 * masked before the first source is enabled and the per-driver
 * socplicmask workaround is no longer required.
 */

#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"

enum {					/* registers (word index when stride == 4) */
	Rbr		= 0,		/* Receiver Buffer (RO) */
	Thr		= 0,		/* Transmitter Holding (WO) */
	Ier		= 1,		/* Interrupt Enable */
	Iir		= 2,		/* Interrupt Identification (RO) */
	Fcr		= 2,		/* FIFO Control (WO) */
	Lcr		= 3,		/* Line Control */
	Mcr		= 4,		/* Modem Control */
	Lsr		= 5,		/* Line Status */
	Msr		= 6,		/* Modem Status */
	Scr		= 7,		/* Scratch Pad */
	Usr		= 31,		/* dwapb UART Status */
	Dll		= 0,		/* Divisor Latch LSB */
	Dlm		= 1,		/* Divisor Latch MSB */
};

enum {					/* Ier */
	Erda		= 0x01,
	Ethre		= 0x02,
	Erls		= 0x04,
	Ems		= 0x08,
	/* PXA-family extensions (per the Marvell PXA TRM, UART chapter) */
	Ertoi		= 0x10,		/* receive timeout */
	Iunite		= 0x40,		/* UART unit enable */
	/* RX-startup IER mask required by the PXA-derived datapath:
	 * data + line-status + timeout, with the unit enabled. */
	Pxaier		= Erda|Erls|Ertoi|Iunite,
};

enum {					/* Iir */
	Ip		= 0x01,
	IirMASK		= 0x0e,
	Ims		= 0x00,		/* modem status */
	Ithre		= 0x02,		/* THR empty */
	Irda		= 0x04,		/* received data available */
	Irls		= 0x06,		/* receive line status */
	Ictoi		= 0x0c,		/* character timeout indication */
};

enum {					/* Fcr */
	FIFOena		= 0x01,
	FIFOrclr	= 0x02,
	FIFOtclr	= 0x04,
	FIFO1		= 0x00,
	FIFO4		= 0x40,
	FIFO8		= 0x80,
	FIFO14		= 0xC0,
	/* PXA 8-byte trigger + FIFO enable */
	Pxafcr		= FIFOena|FIFO4,
};

enum {					/* Lcr */
	Wls5		= 0x00,
	Wls6		= 0x01,
	Wls7		= 0x02,
	Wls8		= 0x03,
	WlsMASK		= 0x03,
	Stb		= 0x04,
	Pen		= 0x08,
	Eps		= 0x10,
	Stp		= 0x20,
	Brk		= 0x40,
	Dlab		= 0x80,
};

enum {					/* Mcr */
	Dtr		= 0x01,
	Rts		= 0x02,
	Out1		= 0x04,
	Out2		= 0x08,
	Dm		= 0x10,
};

enum {					/* Lsr */
	Dr		= 0x01,
	Oe		= 0x02,
	Pe		= 0x04,
	Fe		= 0x08,
	Bi		= 0x10,
	Thre		= 0x20,
	Temt		= 0x40,
	FIFOerr		= 0x80,
};

enum {					/* Msr */
	Dcts		= 0x01,
	Ddsr		= 0x02,
	Teri		= 0x04,
	Ddcd		= 0x08,
	Cts		= 0x10,
	Dsr		= 0x20,
	Ri		= 0x40,
	Dcd		= 0x80,
};

typedef struct Ctlr {
	uchar	*io;
	int	regshift;	/* DT reg-shift: 0 for ns16550a, 2 for PXA/DW APB */
	int	irq;
	int	tbdf;
	int	iena;
	int	poll;
	int	ispxa;		/* PXA-family UART quirks (FCR/IER) needed */

	uchar	sticky[Scr+1];

	Lock;
	int	hasfifo;
	int	checkfifo;
	int	fena;
} Ctlr;

extern PhysUart i8250physuart;

/*
 * Boot-time placeholder used by uartconsinit() before the DTB has been
 * parsed: putc thunks to sbiputc, getc returns -1.  Once i8250pnp()
 * runs (from devuart's reset path, after bootargsinit) we either reuse
 * this slot for the DT-derived console UART or leave it behind as a
 * passive sbi-only fallback.
 */
static Ctlr i8250ctlr = {
	.tbdf	= -1,
	.irq	= -1,
	.poll	= 1,
};

static Uart i8250uart = {
	.regs	= &i8250ctlr,
	.name	= "eia0",
	.freq	= 0,
	.phys	= &i8250physuart,
	.console = 1,
	.next	= nil,
};

static Ctlr *pollctlr;		/* drained from clock callback */

/*
 * Two access widths are in play:
 *   regshift=0: ns16550a-style, byte-addressed, 8-bit MMIO.
 *   regshift=2: PXA/DW-APB-style, 32-bit-strided, 32-bit MMIO
 *               (DT says reg-io-width=<4>; byte reads return junk on
 *               real silicon).  Access width tracks regshift.
 */
#define UREG(c, r)	((volatile void*)((c)->io + ((r) << (c)->regshift)))
#define csr8r(c, r)	((c)->regshift >= 2 \
			 ? (uchar)*(volatile u32int*)UREG(c, r) \
			 : *(volatile uchar*)UREG(c, r))
#define csr8w(c, r, v)	((c)->regshift >= 2 \
			 ? (*(volatile u32int*)UREG(c, r) = (c)->sticky[r] | (v)) \
			 : (*(volatile uchar*)UREG(c, r) = (c)->sticky[r] | (v)))
#define csr8o(c, r, v)	((c)->regshift >= 2 \
			 ? (*(volatile u32int*)UREG(c, r) = (v)) \
			 : (*(volatile uchar*)UREG(c, r) = (v)))

static char*
i8250status(Uart* uart, char *p, char *e)
{
	Ctlr *ctlr;
	uchar ier, lcr, mcr, msr;

	ctlr = uart->regs;
	mcr = ctlr->sticky[Mcr];
	msr = ctlr->io ? csr8r(ctlr, Msr) : 0;
	ier = ctlr->sticky[Ier];
	lcr = ctlr->sticky[Lcr];
	return seprint(p, e,
		"b%d c%d d%d e%d l%d m%d p%c r%d s%d i%d\n"
		"dev(%d) type(%d) framing(%d) overruns(%d) "
		"berr(%d) serr(%d)%s%s%s%s\n",

		uart->baud,
		uart->hup_dcd,
		(msr & Dsr) != 0,
		uart->hup_dsr,
		(lcr & WlsMASK) + 5,
		(ier & Ems) != 0,
		(lcr & Pen) ? ((lcr & Eps) ? 'e': 'o'): 'n',
		(mcr & Rts) != 0,
		(lcr & Stb) ? 2: 1,
		ctlr->fena,

		uart->dev,
		uart->type,
		uart->ferr,
		uart->oerr,
		uart->berr,
		uart->serr,
		(msr & Cts) ? " cts": "",
		(msr & Dsr) ? " dsr": "",
		(msr & Dcd) ? " dcd": "",
		(msr & Ri) ? " ring": ""
	);
}

static void
i8250fifo(Uart* uart, int level)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	if(ctlr->hasfifo == 0 || ctlr->io == nil)
		return;

	ilock(ctlr);
	while(!(csr8r(ctlr, Lsr) & Temt))
		;

	ctlr->fena = level;
	switch(level){
	case 0:
		break;
	case 1:
		level = FIFO1|FIFOena;
		break;
	case 4:
		level = FIFO4|FIFOena;
		break;
	case 8:
		level = FIFO8|FIFOena;
		break;
	default:
		level = FIFO14|FIFOena;
		break;
	}
	csr8w(ctlr, Fcr, level);
	csr8w(ctlr, Fcr, level);
	iunlock(ctlr);
}

static void
i8250dtr(Uart* uart, int on)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	if(on)
		ctlr->sticky[Mcr] |= Dtr;
	else
		ctlr->sticky[Mcr] &= ~Dtr;
	if(ctlr->io != nil)
		csr8w(ctlr, Mcr, 0);
}

static void
i8250rts(Uart* uart, int on)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	if(on)
		ctlr->sticky[Mcr] |= Rts;
	else
		ctlr->sticky[Mcr] &= ~Rts;
	if(ctlr->io != nil)
		csr8w(ctlr, Mcr, 0);
}

static void
i8250modemctl(Uart* uart, int on)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	ilock(&uart->tlock);
	if(on){
		ctlr->sticky[Ier] |= Ems;
		if(ctlr->io != nil)
			csr8w(ctlr, Ier, 0);
		uart->modem = 1;
		uart->cts = ctlr->io ? (csr8r(ctlr, Msr) & Cts) : 0;
	}
	else{
		ctlr->sticky[Ier] &= ~Ems;
		if(ctlr->io != nil)
			csr8w(ctlr, Ier, 0);
		uart->modem = 0;
		uart->cts = 1;
	}
	iunlock(&uart->tlock);

	(*uart->phys->fifo)(uart, on);
}

static int
i8250parity(Uart* uart, int parity)
{
	int lcr;
	Ctlr *ctlr;

	ctlr = uart->regs;
	lcr = ctlr->sticky[Lcr] & ~(Eps|Pen);

	switch(parity){
	case 'e':
		lcr |= Eps|Pen;
		break;
	case 'o':
		lcr |= Pen;
		break;
	case 'n':
		break;
	default:
		return -1;
	}
	ctlr->sticky[Lcr] = lcr;
	if(ctlr->io != nil)
		csr8w(ctlr, Lcr, 0);

	uart->parity = parity;

	return 0;
}

static int
i8250stop(Uart* uart, int stop)
{
	int lcr;
	Ctlr *ctlr;

	ctlr = uart->regs;
	lcr = ctlr->sticky[Lcr] & ~Stb;

	switch(stop){
	case 1:
		break;
	case 2:
		lcr |= Stb;
		break;
	default:
		return -1;
	}
	ctlr->sticky[Lcr] = lcr;
	if(ctlr->io != nil)
		csr8w(ctlr, Lcr, 0);

	uart->stop = stop;

	return 0;
}

static int
i8250bits(Uart* uart, int bits)
{
	int lcr;
	Ctlr *ctlr;

	ctlr = uart->regs;
	lcr = ctlr->sticky[Lcr] & ~WlsMASK;

	switch(bits){
	case 5:
		lcr |= Wls5;
		break;
	case 6:
		lcr |= Wls6;
		break;
	case 7:
		lcr |= Wls7;
		break;
	case 8:
		lcr |= Wls8;
		break;
	default:
		return -1;
	}
	ctlr->sticky[Lcr] = lcr;
	if(ctlr->io != nil)
		csr8w(ctlr, Lcr, 0);

	uart->bits = bits;

	return 0;
}

static int
i8250baud(Uart* uart, int baud)
{
	/*
	 * The previous stage (OpenSBI / U-Boot) has already programmed a
	 * sane baud rate.  Reprogramming would require knowing the input
	 * clock, which varies across boards; trust what we inherited.
	 */
	uart->baud = baud;
	return 0;
}

static void
i8250break(Uart* uart, int ms)
{
	Ctlr *ctlr;

	if (up == nil)
		panic("i8250break: nil up");
	if(ms <= 0)
		ms = 200;

	ctlr = uart->regs;
	if(ctlr->io == nil)
		return;
	csr8w(ctlr, Lcr, Brk);
	tsleep(&up->sleep, return0, 0, ms);
	csr8w(ctlr, Lcr, 0);
}

static void
i8250kick(Uart* uart)
{
	Ctlr *ctlr = uart->regs;
	int i;

	if(ctlr->io == nil){
		/* Fall back to SBI legacy putchar for the boot placeholder
		 * (before DTB-derived MMIO has been mapped). */
		while(uart->op < uart->oe || uartstageoutput(uart) > 0)
			sbiputc((uchar)*uart->op++);
		return;
	}
	while(uart->op < uart->oe || uartstageoutput(uart) > 0){
		for(i = 0; !(csr8r(ctlr, Lsr) & Thre) && i < 1000; i++)
			;
		csr8o(ctlr, Thr, (uchar)*uart->op++);
	}
}

static void
i8250poll(void)
{
	Ctlr *ctlr;
	Uart *uart;
	int lsr, r, n;

	ctlr = pollctlr;
	if(ctlr == nil || ctlr->io == nil || consuart == nil)
		return;
	uart = consuart;
	lsr = csr8r(ctlr, Lsr);
	n = 0;
	while((lsr & Dr) && n < 64){
		r = csr8r(ctlr, Rbr);
		if(!(lsr & (Bi|Fe|Pe))){
			if(r == '\r')
				r = '\n';
			uartrecv(uart, (u8int)r);
		}
		lsr = csr8r(ctlr, Lsr);
		n++;
	}
}

static void
i8250disable(Uart* uart)
{
	Ctlr *ctlr;

	(*uart->phys->dtr)(uart, 0);
	(*uart->phys->rts)(uart, 0);
	(*uart->phys->fifo)(uart, 0);

	ctlr = uart->regs;
	ctlr->sticky[Ier] = 0;
	if(ctlr->io != nil)
		csr8w(ctlr, Ier, 0);
}

/*
 * PLIC-driven RX/TX.  Modelled on mt7688/uarti8250.c#i8250interrupt:
 * loop reading IIR until "no interrupt pending", dispatching by the
 * IIR ID field.  On the PXA-derived block a receive-timeout indication
 * (Ictoi) is the natural "drain me" event for sub-FIFO chunks; the
 * standard 16550 only ever raises Irda for that case.
 */
static void
i8250interrupt(Ureg*, void* arg)
{
	Uart *uart;
	Ctlr *ctlr;
	int iir, lsr, r, trips;

	uart = arg;
	ctlr = uart->regs;
	if(ctlr->io == nil)
		return;

	/*
	 * Cap the iteration count so a stuck IIR can't pin this CPU.  An
	 * unpowered or mis-clocked PXA UART can return 0x00 from every
	 * read (no Ip bit, ID=000=Ims), which would otherwise loop forever
	 * with reads of Msr that never settle.  64 passes is enough to
	 * drain any sensible burst.
	 */
	trips = 64;
	for(iir = csr8r(ctlr, Iir); !(iir & Ip) && trips-- > 0;
	    iir = csr8r(ctlr, Iir)){
		switch(iir & IirMASK){
		case Ims:		/* modem status: just clear */
			(void)csr8r(ctlr, Msr);
			break;
		case Ithre:		/* THR empty: kick TX */
			uartkick(uart);
			break;
		case Irda:		/* RX data available */
		case Irls:		/* RX line status (errors latched in LSR) */
		case Ictoi:		/* RX timeout (FIFO has stuck bytes) */
			/*
			 * Always read LSR first to clear any latched error
			 * bits (Oe/Pe/Fe/Bi/FIFOerr); otherwise an Irls without
			 * Dr=1 would re-assert IIR forever.
			 */
			lsr = csr8r(ctlr, Lsr);
			if(lsr & (FIFOerr|Oe))
				uart->oerr++;
			if(lsr & Pe)
				uart->perr++;
			if(lsr & Fe)
				uart->ferr++;
			while(lsr & Dr){
				r = csr8r(ctlr, Rbr);
				if(!(lsr & (Bi|Fe|Pe))){
					if(r == '\r')
						r = '\n';
					uartrecv(uart, (u8int)r);
				}
				lsr = csr8r(ctlr, Lsr);
				if(lsr & (FIFOerr|Oe))
					uart->oerr++;
				if(lsr & Pe)
					uart->perr++;
				if(lsr & Fe)
					uart->ferr++;
			}
			break;
		default:
			/* not all IIR codes are documented for every variant; just clear */
			(void)csr8r(ctlr, Lsr);
			(void)csr8r(ctlr, Rbr);
			break;
		}
	}
}

static void
i8250enable(Uart* uart, int ie)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	if(ctlr->io == nil)
		return;

	/*
	 * Force the UART back to a known PIO-mode state after the prior
	 * stage hands off (it may have left DMA or FIFOs armed).  The
	 * enable-then-clear-then-reprogram sequence is the standard
	 * 16550 reset: arm the FIFO so the clear bits are honored, drop
	 * both RX and TX queues, then either leave it disabled (generic
	 * 16550) or set the PXA 8-byte trigger we need for medium baud.
	 */
	csr8o(ctlr, Fcr, FIFOena);
	csr8o(ctlr, Fcr, FIFOena|FIFOrclr|FIFOtclr);
	csr8o(ctlr, Fcr, ctlr->ispxa ? Pxafcr : 0);

	/* Drain any stale state. */
	(void)csr8r(ctlr, Lsr);
	(void)csr8r(ctlr, Rbr);
	(void)csr8r(ctlr, Iir);
	(void)csr8r(ctlr, Msr);

	/* 8N1 line config. */
	ctlr->sticky[Lcr] = Wls8;
	csr8o(ctlr, Lcr, Wls8);

	/* MCR: DTR + RTS + Out2.  Out2 is required by PXA-family parts
	 * to route the chip-level IRQ out even in polled operation. */
	ctlr->sticky[Mcr] = Dtr|Rts|Out2;
	csr8o(ctlr, Mcr, Dtr|Rts|Out2);

	/*
	 * IER: PXA-family needs the unit-enable bit plus data/line/timeout
	 * enables before RX bytes land in the FIFO.  A generic 16550 starts
	 * with interrupts off and gets Erda once we've successfully wired
	 * an intrenable below; if we end up in the polling fallback we
	 * leave it at 0 so the line never asserts spuriously.
	 */
	ctlr->sticky[Ier] = ctlr->ispxa ? Pxaier : 0;
	csr8o(ctlr, Ier, ctlr->sticky[Ier]);

	if(ie && ctlr->iena == 0){
		/*
		 * Only the console UART gets the IRQ-driven path for now.
		 * Secondary UARTs enumerated from the DTB may be in odd power
		 * states (eia2 on the K1 is "okay" in the DT but the chip
		 * itself may not be fully clocked at the moment) and a misread
		 * IIR there would storm the PLIC.  When we trust the per-UART
		 * clock/power state, this gate can drop.
		 */
		if(ctlr->irq > 0 && uart->console){
			intrenable(ctlr->irq, i8250interrupt, uart, BUSUNKNOWN, uart->name);
			ctlr->poll = 0;
			ctlr->iena = 1;
			/* generic 16550: enable RX-data-available now that an ISR exists */
			if(!ctlr->ispxa){
				ctlr->sticky[Ier] = Erda;
				csr8o(ctlr, Ier, ctlr->sticky[Ier]);
			}
			print("uarti8250: irq-driven on %s irq %d\n",
				uart->name, ctlr->irq);
		} else if(uart->console){
			/*
			 * No IRQ from the DTB — fall back to the 100-tick clock
			 * callback polling path so the console is still usable.
			 */
			ctlr->poll = 1;
			pollctlr = ctlr;
			addclock0link(i8250poll, 100);
			ctlr->iena = 1;
			print("uarti8250: input polling enabled on %s\n", uart->name);
		}
	}
}

static int issupportedcompat(char *s)
{
	return strcmp(s, "spacemit,pxa-uart") == 0
	    || strncmp(s, "spacemit,rcpu-pxa-uart", 22) == 0
	    || strcmp(s, "ns16550a") == 0
	    || strcmp(s, "ns16550") == 0
	    || strcmp(s, "snps,dw-apb-uart") == 0;
}

static Uart*
allocuart(Dtbuart *du, int aliasidx)
{
	Ctlr *ctlr;
	Uart *u;
	void *va;
	char *name;

	va = vmap(du->base, BY2PG);
	if(va == nil){
		print("uarti8250: vmap(%#llux) failed for %s\n", du->base, du->path);
		return nil;
	}

	if(du->isconsole){
		/* Reuse the static placeholder so consuart stays valid
		 * through the transition from sbi-only early prints to MMIO. */
		ctlr = &i8250ctlr;
		u = &i8250uart;
	} else {
		ctlr = malloc(sizeof(Ctlr));
		u = malloc(sizeof(Uart));
		if(ctlr == nil || u == nil){
			free(ctlr);
			free(u);
			return nil;
		}
		memset(ctlr, 0, sizeof(*ctlr));
		memset(u, 0, sizeof(*u));
		ctlr->tbdf = -1;
		u->regs = ctlr;
		u->phys = &i8250physuart;
	}
	ctlr->io = (uchar*)va;
	ctlr->regshift = du->regshift;
	ctlr->irq = du->irq;
	ctlr->poll = 1;
	ctlr->ispxa = strncmp(du->compatible, "spacemit,", 9) == 0;

	name = malloc(16);
	if(name != nil){
		snprint(name, 16, "eia%d", aliasidx);
		u->name = name;
	}
	u->console = du->isconsole;
	u->next = nil;
	print("uarti8250: %s at %#llux irq %d compat %q%s\n",
		u->name, du->base, du->irq, du->compatible,
		du->isconsole ? " [console]" : "");
	return u;
}

static Uart*
i8250pnp(void)
{
	static int done;
	Uart *head, *tail, *u;
	Dtbuart *du;
	int i, j, n;

	if(done)
		return i8250uart.next != nil || i8250ctlr.io != nil ? &i8250uart : nil;
	done = 1;

	n = dtbnuart();
	head = tail = nil;

	/* Iterate aliases in order so eia<N> matches DT serial<N>. */
	for(i = 0; i < 32; i++){
		du = nil;
		for(j = 0; j < n; j++){
			Dtbuart *d = dtbuart(j);
			if(d != nil && d->aliasidx == i){
				du = d;
				break;
			}
		}
		if(du == nil || !du->ok || du->base == 0)
			continue;
		if(du->compatible[0] && !issupportedcompat(du->compatible))
			continue;
		u = allocuart(du, i);
		if(u == nil)
			continue;
		if(head == nil)
			head = u;
		else
			tail->next = u;
		tail = u;
	}

	/* Also catch any okay uart-like nodes that lack a serial<N> alias. */
	for(j = 0; j < n; j++){
		du = dtbuart(j);
		if(du == nil || du->aliasidx >= 0)
			continue;
		if(!du->ok || du->base == 0)
			continue;
		if(du->compatible[0] && !issupportedcompat(du->compatible))
			continue;
		u = allocuart(du, 32 + j);
		if(u == nil)
			continue;
		if(head == nil)
			head = u;
		else
			tail->next = u;
		tail = u;
	}

	if(head == nil){
		/* No DTB UARTs — leave the static placeholder so consuart
		 * keeps sbi-thunking putc. */
		return &i8250uart;
	}
	return head;
}

static int
i8250getc(Uart* uart)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	if(ctlr->io == nil)
		return -1;
	if(!(csr8r(ctlr, Lsr) & Dr))
		return -1;
	return csr8r(ctlr, Rbr);
}

static void
i8250putc(Uart *uart, int c)
{
	Ctlr *ctlr = uart->regs;
	int i;

	if(ctlr->io == nil){
		sbiputc((uchar)c);
		return;
	}
	for(i = 0; !(csr8r(ctlr, Lsr) & Thre) && i < 1000; i++)
		;
	csr8o(ctlr, Thr, (uchar)c);
}

void
uartconsinit(void)
{
	/*
	 * Wire consuart now so early prints have somewhere to go.  The
	 * actual MMIO mapping comes later from i8250pnp() once bootargsinit
	 * has populated the DTB UART table; until then putc falls back to
	 * sbiputc and getc returns -1.  Don't call uartctl here: it would
	 * call csr8w which touches ctlr->io.  We trust whatever baud/format
	 * U-Boot or OpenSBI left configured.
	 */
	consuart = &i8250uart;
	consuart->console = 1;
}

static void
donothing(Uart*, int)
{
}

PhysUart i8250physuart = {
	.name		= "i8250",
	.pnp		= i8250pnp,
	.enable		= i8250enable,
	.disable	= i8250disable,
	.kick		= i8250kick,
	.dobreak	= i8250break,
	.baud		= i8250baud,
	.bits		= i8250bits,
	.stop		= i8250stop,
	.parity		= i8250parity,
	.modemctl	= i8250modemctl,
	.rts		= i8250rts,
	.dtr		= i8250dtr,
	.status		= i8250status,
	.fifo		= i8250fifo,
	.getc		= i8250getc,
	.putc		= i8250putc,
	.power		= donothing,
};
