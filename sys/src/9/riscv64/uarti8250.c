/*
 * 8250/16550-class UART for the riscv64 kernel.
 *
 * Based on sys/src/9/mt7688/uarti8250.c (which itself derives from the
 * generic 8250 driver), adapted for boards where the UART MMIO base is
 * not known at build time.  The base address (and IRQ, when interrupts
 * are wired) come from the boot-time config via getconf:
 *
 *   uartbase=0xd4017000          # required; physical address of the UART
 *   uartstride=4                 # optional; bytes per register (default 4)
 *
 * The launcher (lib/linux/kexec/kexec) appends these to /chosen/bootargs
 * with its -b flag, e.g.:
 *
 *   ./kexec -b "uartbase=0xd4017000" 9bpif3.elf
 *
 * Input is polled from the system clock callback (addclock0link), not
 * driven by a PLIC interrupt — the riscv64 PLIC base is currently
 * hardcoded for QEMU virt and a board-portable PLIC is a separate
 * change.  The 100-tick poll interval matches uartsbi.c.
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

enum {					/* Usr (dwapb) */
	Busy		= 0x01,
};

enum {					/* Ier */
	Erda		= 0x01,
	Ethre		= 0x02,
	Erls		= 0x04,
	Ems		= 0x08,
};

enum {					/* Iir */
	Ims		= 0x00,
	Ip		= 0x01,
	Ithre		= 0x02,
	Irda		= 0x04,
	Irls		= 0x06,
	Ictoi		= 0x0C,
	IirMASK		= 0x3F,
	Ifena		= 0xC0,
};

enum {					/* Fcr */
	FIFOena		= 0x01,
	FIFOrclr	= 0x02,
	FIFOtclr	= 0x04,
	FIFO1		= 0x00,
	FIFO4		= 0x40,
	FIFO8		= 0x80,
	FIFO14		= 0xC0,
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
	u32int	*io;
	int	irq;
	int	tbdf;
	int	iena;
	int	poll;

	uchar	sticky[Scr+1];

	Lock;
	int	hasfifo;
	int	checkfifo;
	int	fena;
} Ctlr;

extern PhysUart i8250physuart;

/*
 * Static so uartconsinit() can wire up consuart before bootargsinit()
 * has populated conf[].  Until i8250pnp() runs (which happens later,
 * from devuart's normal init path) and stores a vmap()ped pointer in
 * ctlr->io, putc falls back to sbiputc and getc returns -1.  That way
 * early kernel prints flow via the SBI legacy putchar that works on
 * every board, and only the input path needs MMIO to come up.
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

#define csr8r(c, r)	((c)->io[r])
#define csr8w(c, r, v)	((c)->io[r] = (c)->sticky[r] | (v))
#define csr8o(c, r, v)	((c)->io[r] = (v))

static char*
i8250status(Uart* uart, char *p, char *e)
{
	Ctlr *ctlr;
	uchar ier, lcr, mcr, msr;

	ctlr = uart->regs;
	mcr = ctlr->sticky[Mcr];
	msr = csr8r(ctlr, Msr);
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
	if(ctlr->hasfifo == 0)
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
		csr8w(ctlr, Ier, 0);
		uart->modem = 1;
		uart->cts = csr8r(ctlr, Msr) & Cts;
	}
	else{
		ctlr->sticky[Ier] &= ~Ems;
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
	csr8w(ctlr, Lcr, Brk);
	tsleep(&up->sleep, return0, 0, ms);
	csr8w(ctlr, Lcr, 0);
}

static void
i8250kick(Uart* uart)
{
	/* See i8250putc: same reasoning, drain the staged output via SBI. */
	while(uart->op < uart->oe || uartstageoutput(uart) > 0)
		sbiputc((uchar)*uart->op++);
}

/*
 * Clock-callback polling.  Drains anything in the RX FIFO into uartrecv.
 * Runs from the system clock at 100-tick intervals (≈100 ms at HZ=100 / 10 ms at HZ=1000).
 */
static void
i8250poll(void)
{
	Ctlr *ctlr;
	int lsr, r, n;

	ctlr = &i8250ctlr;
	if(ctlr->io == nil)
		return;
	lsr = csr8r(ctlr, Lsr);
	n = 0;
	while((lsr & Dr) && n < 64){
		r = csr8r(ctlr, Rbr);
		if(!(lsr & (Bi|Fe|Pe))){
			if(r == '\r')
				r = '\n';
			uartrecv(&i8250uart, (u8int)r);
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
	csr8w(ctlr, Ier, 0);
}

static void
i8250enable(Uart* uart, int ie)
{
	Ctlr *ctlr;

	ctlr = uart->regs;
	if(ctlr->io == nil){
		print("uarti8250: enable called but uartbase not set\n");
		return;
	}

	/*
	 * Reset and re-enable both FIFOs.  The previous stage (U-Boot
	 * or Linux's serial8250_shutdown) may have left the RX FIFO in
	 * a stale state.  Single MMIO store, no spinning.
	 */
	csr8o(ctlr, Fcr, FIFOena|FIFOrclr|FIFOtclr);

	/*
	 * Don't touch LCR/MCR/IER beyond what bits/parity/stop already did.
	 * On the K1 (and likely other dwapb-derivatives) the RX FIFO never
	 * fills with characters typed from the serial cable even though
	 * direct MMIO writes to THR do reach the cable.  Probably a pinmux
	 * or RX-enable bit somewhere outside the standard 16550 register
	 * set.  TODO: figure that out, or move the kernel to M-mode so we
	 * own the UART traps directly.
	 */
	if(ie && ctlr->iena == 0){
		ctlr->poll = 1;
		addclock0link(i8250poll, 100);
		ctlr->iena = 1;
		print("uarti8250: input polling enabled (RX currently dead, "
			"see TODO in driver)\n");
	}
}

static Uart*
i8250pnp(void)
{
	char *p;
	uvlong pa;
	void *va;

	if(i8250ctlr.io != nil)
		return &i8250uart;	/* already mapped */

	p = getconf("uartbase");
	if(p == nil){
		/* No uartbase yet — return the uart anyway so consuart wiring
		 * works; putc will fall through to sbiputc. */
		return &i8250uart;
	}
	pa = strtoull(p, nil, 0);
	if(pa == 0){
		print("uarti8250: bad uartbase %q\n", p);
		return &i8250uart;
	}
	va = vmap(pa, BY2PG);
	if(va == nil){
		print("uarti8250: vmap(%#llux) failed\n", pa);
		return &i8250uart;
	}
	i8250ctlr.io = (u32int*)va;
	print("uarti8250: eia0 at %#llux -> %#p\n", pa, va);
	return &i8250uart;
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
i8250putc(Uart*, int c)
{
	/*
	 * Output via SBI legacy putchar even after MMIO is mapped.
	 * On this board legacy putchar works reliably; bypassing SBI for
	 * TX gains us nothing and just gives us a way to hang on a
	 * dodgy Thre status.  The point of this driver is input.
	 */
	sbiputc((uchar)c);
}

void
uartconsinit(void)
{
	/*
	 * Wire consuart now so early prints have somewhere to go.  The
	 * actual MMIO mapping comes later from i8250pnp() once bootargsinit
	 * has populated conf[] with uartbase=... ; until then putc falls
	 * back to sbiputc and getc returns -1.  Don't call uartctl here:
	 * it would call csr8w which touches ctlr->io.  We trust whatever
	 * baud/format U-Boot or OpenSBI left configured.
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
