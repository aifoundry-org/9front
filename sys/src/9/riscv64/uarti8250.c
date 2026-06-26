/*
 * 8250/16550-class UART for the riscv64 kernel.  Based on
 * sys/src/9/mt7688/uarti8250.c, adapted for boards where the UART
 * MMIO base is supplied via getconf rather than hardcoded.
 *
 * Boot config knobs (set via launcher's -b flag, plumbed through
 * DTB /chosen/bootargs and read at init via getconf):
 *
 *   uartbase=0xd4017000   required; physical address of the UART.
 *   k1ier=0x55            optional; override IER (default UUE only).
 *                         K1 needs 0x55 = RLSI|RDI|RTOIE|UUE for RX.
 *   k1fcr=0x41            optional; override FCR (default 0 = 16450).
 *                         K1 needs 0x41 = FIFO+PXAR8 trigger.
 *   k1plicmask=42         optional; mask given IRQ at K1 PLIC (base
 *                         0xe0000000) by writing priority 0.  Needed
 *                         to prevent IRQ storm when k1ier arms RX
 *                         interrupts that Plan 9's plic.c (which has
 *                         QEMU-virt PLICBASE hardcoded) can't mask.
 *   k1apbc=<mask>         optional; OR mask into APBC+0 at 0xd4015000.
 *                         Theoretically deasserts UART reset; in
 *                         practice the K1 inherits bit 2 = 0 from the
 *                         prior owner's runtime state and writing
 *                         bit 2 = 1 hangs the kernel.  Leave unset.
 *   k1debug=1             optional; dump UART + apbc registers at init.
 *
 * Proven invocation for BPI-F3 (Spacemit K1, 2 GB):
 *
 *   ./kexec -b "*maxmem=0x80000000 uartbase=0xd4017000 \
 *                k1plicmask=42 k1ier=0x55 k1fcr=0x41 console=0" \
 *           9bpif3.elf
 *
 *   *maxmem caps meminit at the upper RAM bank end (kernel's hardcoded
 *           VDRAM+2GiB span would walk off RAM).
 *   k1plicmask masks UART IRQ 42 at the K1 PLIC so chip-level IRQs
 *           (armed by k1ier=0x55) don't storm us.
 *   k1ier+k1fcr match the PXA-style set_termios values the K1 needs —
 *           together they enable the RX datapath that was dead with
 *           bare IER=UUE.
 *   console=0 tells bootrc's kbdfs to bind /dev/eia0 to /dev/cons so
 *           the bootargs prompt's `dd` reads see typing.
 *
 * Input is polled from a 100-ms clock callback (addclock0link), not
 * driven by interrupts — the riscv64 PLIC base is hardcoded for
 * QEMU virt and a board-portable PLIC is a separate change.
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
	/* PXA-style extensions (K1 inherits from Marvell PXA — see
	 * the Marvell PXA TRM, UART chapter) */
	Rtoie		= 0x10,		/* Receive Timeout Interrupt Enable */
	Nrze		= 0x20,		/* NRZ Encoding Enable */
	UUE		= 0x40,		/* UART Unit Enable */
	DMAE		= 0x80,		/* DMA Enable */
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
/*
 * BPI-F3 / K1 "sys-led" kernel-alive heartbeat.  GPIO pin 96 = bank 3
 * bit 0 of the controller at 0xd4019000; bank 3's register block starts
 * at offset 0x100 within the controller (banks 0/1/2 interleave at
 * 4-byte stride before that).  GPSR (atomic set) = +0x18, GPCR (atomic
 * clear) = +0x24.  The prior owner left the pin in output mode driving high.
 */
static u32int *bpif3gpio;
static int bpif3ledphase;

static void
bpif3heartbeat(void)
{
	if(bpif3gpio == nil)
		return;
	if(++bpif3ledphase & 1)
		bpif3gpio[(0x100 + 0x18)/4] = 0x1;	/* GPSR3: on */
	else
		bpif3gpio[(0x100 + 0x24)/4] = 0x1;	/* GPCR3: off */
}

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
	char *p;
	ulong apbcset;

	ctlr = uart->regs;
	if(ctlr->io == nil){
		print("uarti8250: enable called but uartbase not set\n");
		return;
	}

	apbcset = 0;
	if((p = getconf("k1apbc")) != nil)
		apbcset = strtoul(p, nil, 0);

	/*
	 * K1 PLIC is at 0xe0000000, not the QEMU virt 0x0c000000 that
	 * our plic.c hardcodes.  The prior owner left it programmed to
	 * route the UART IRQ to the CPU.  If we now enable chip-level interrupts
	 * in IER (k1ier=0x55), they'd fire forever because Plan 9's
	 * plic.c can't see or mask the real PLIC.
	 *
	 * Mask the UART IRQ at the K1 PLIC by writing priority 0 to its
	 * source register: base + irq*4.  UART IRQ = 42 (DTS interrupts
	 * = <0x2a>).  Opt-in via -b "k1plicmask=42".
	 */
	if((p = getconf("k1plicmask")) != nil){
		int irq = strtoul(p, nil, 0);
		u32int *plic;

		plic = (u32int*)vmap(0xe0000000ULL, BY2PG);
		if(plic != nil && irq > 0 && irq < 1024){
			plic[irq] = 0;	/* priority 0 = masked */
			coherence();
		}
	}

	/*
	 * PXA-style startup so the UART comes back to a known PIO-mode
	 * state after kexec.
	 *
	 * K1 apbc experimentation:  At kexec time we observe APBC+0 (0xd4015000)
	 * = 0x13 = bits 0+1+4 set, bit 2 clear.  The prior owner ran the UART
	 * successfully with bit 2 = 0, which suggests bit 2 = 0 is actually the
	 * OPERATIONAL state — opposite of what the BSP's "deassert_val=BIT(2)"
	 * reset-controller naming would imply.  Writing 0x7 (setting bit 2)
	 * hung the kernel, consistent with that putting the UART INTO reset.
	 *
	 * Tunable via boot arg: -b "k1apbc=<hex>" gives an OR mask applied to
	 * the existing APBC+0 value.  Useful values to try:
	 *    k1apbc not set — skip the write entirely
	 *    k1apbc=0       — write existing value back (sanity check: writes work?)
	 *    k1apbc=0x4     — set bit 2 (the BSP "deassert" semantic; previously
	 *                     hung the kernel — but we now do this AFTER setting
	 *                     IER=UUE so DMA can't fire as the UART unhalts)
	 *    k1apbc=0x3     — force clocks on
	 */

	/* Enable FIFO, clear both RX/TX, then by default DISABLE FIFO so
	 * the UART runs in 16450 mode (no DMA hand-off).  The K1 PXA-style
	 * set_termios re-enables it with PXAR8 trigger (FCR=0x41) for medium
	 * baud rates after startup; override here via -b "k1fcr=N" if 16450
	 * mode turns out not to work for RX on the K1. */
	csr8o(ctlr, Fcr, FIFOena);
	csr8o(ctlr, Fcr, FIFOena|FIFOrclr|FIFOtclr);
	if((p = getconf("k1fcr")) != nil)
		csr8o(ctlr, Fcr, strtoul(p, nil, 0));
	else
		csr8o(ctlr, Fcr, 0);

	/* Drain any stale interrupt state. */
	(void)csr8r(ctlr, Lsr);
	(void)csr8r(ctlr, Rbr);
	(void)csr8r(ctlr, Iir);
	(void)csr8r(ctlr, Msr);

	/* 8N1 line config. */
	ctlr->sticky[Lcr] = Wls8;
	csr8o(ctlr, Lcr, Wls8);

	/* MCR: DTR + RTS + Out2 (Out2 routes the UART IRQ to the system —
	 * the PXA convention sets it unconditionally even in polled use). */
	ctlr->sticky[Mcr] = Dtr|Rts|Out2;
	csr8o(ctlr, Mcr, Dtr|Rts|Out2);

	/* IER: default UUE only (no DMA, no IRQs).  The K1 PXA-style
	 * startup sets RLSI|RDI|RTOIE|UUE (= 0x55) — try that with
	 * -b "k1ier=0x55" if just-UUE doesn't get RX working.  Note PLIC
	 * doesn't route the UART IRQ to us by default (we never called
	 * intrenable), so device-level interrupts SHOULD just sit pending
	 * harmlessly in the chip without causing a storm. */
	if((p = getconf("k1ier")) != nil)
		csr8o(ctlr, Ier, strtoul(p, nil, 0));
	else
		csr8o(ctlr, Ier, UUE);

	/* Sys-led GPIO for kernel-alive heartbeat.  Mapped before the
	 * apbc write so we can see beacons even if the deassert hangs. */
	bpif3gpio = (u32int*)vmap(0xd4019000ULL, BY2PG);

	/*
	 * Conditional apbc write — only if -b "k1apbc=..." was passed.
	 * Default boot path skips this and inherits whatever state the
	 * prior owner left (which lets the kernel finish booting; just
	 * with RX dead).
	 *
	 * IER=UUE was set above before this, so the UART can't fire DMA
	 * requests if/when it transitions out of reset here.
	 *
	 * LED beacons surround the write so we can detect a hang at the
	 * exact transition.  Pattern: 1 blink before, 2 blinks after.
	 */
	if(getconf("k1apbc") != nil && bpif3gpio != nil){
		u32int *apbc;
		u32int pre, post;

		/* 1 blink: about to write apbc. */
		bpif3gpio[(0x100 + 0x24)/4] = 0x1;
		delay(150);
		bpif3gpio[(0x100 + 0x18)/4] = 0x1;
		delay(150);

		apbc = (u32int*)vmap(0xd4015000ULL, BY2PG);
		pre = post = 0;
		if(apbc != nil){
			pre = apbc[0];
			apbc[0] = pre | apbcset;
			coherence();
			delay(10);	/* let UART settle if reset just toggled */
			post = apbc[0];
		}

		/* 2 blinks: write completed.  If we never see them, the write
		 * (or the resulting UART state) hung the kernel. */
		bpif3gpio[(0x100 + 0x24)/4] = 0x1;
		delay(150);
		bpif3gpio[(0x100 + 0x18)/4] = 0x1;
		delay(150);
		bpif3gpio[(0x100 + 0x24)/4] = 0x1;
		delay(150);
		bpif3gpio[(0x100 + 0x18)/4] = 0x1;
		delay(150);

		print("uarti8250: apbc[0] pre=%#ux mask=%#lux post=%#ux\n",
			pre, apbcset, post);
	}

	/* Final drain — UART is now alive, clear any state that may have
	 * accumulated during the reset transition. */
	(void)csr8r(ctlr, Lsr);
	(void)csr8r(ctlr, Rbr);
	(void)csr8r(ctlr, Iir);
	(void)csr8r(ctlr, Msr);

	/*
	 * Diagnostic dump on -b "k1debug=1".  Reads the UART's standard
	 * 16550 registers plus the baud divisor (requires DLAB toggle) and
	 * a few PXA-specific registers, plus the APBC clock register.
	 * Useful for the next RX-debug session — particularly DLL/DLM,
	 * since if the prior owner's shutdown lost the baud divisor we'd
	 * see start bits ignored at the wrong rate.
	 */
	if(getconf("k1debug") != nil){
		u32int *apbc;
		uchar dll, dlm, savelcr;

		print("uarti8250: dump LCR=%#ux MCR=%#ux LSR=%#ux MSR=%#ux IER=%#ux IIR=%#ux SCR=%#ux\n",
			csr8r(ctlr, Lcr) & 0xff, csr8r(ctlr, Mcr) & 0xff,
			csr8r(ctlr, Lsr) & 0xff, csr8r(ctlr, Msr) & 0xff,
			csr8r(ctlr, Ier) & 0xff, csr8r(ctlr, Iir) & 0xff,
			csr8r(ctlr, Scr) & 0xff);
		/* Toggle DLAB to read baud divisor. */
		savelcr = csr8r(ctlr, Lcr) & 0xff;
		csr8o(ctlr, Lcr, savelcr | Dlab);
		dll = csr8r(ctlr, Dll) & 0xff;
		dlm = csr8r(ctlr, Dlm) & 0xff;
		csr8o(ctlr, Lcr, savelcr);
		print("uarti8250: baud divisor DLL=%#ux DLM=%#ux (combined %d)\n",
			dll, dlm, dlm<<8 | dll);
		apbc = (u32int*)vmap(0xd4015000ULL, BY2PG);
		if(apbc != nil)
			print("uarti8250: apbc[0]=%#ux\n", apbc[0]);
	}

	/* Hand off to the continuous heartbeat callback. */
	if(bpif3gpio != nil)
		addclock0link(bpif3heartbeat, 500);

	if(ie && ctlr->iena == 0){
		ctlr->poll = 1;
		addclock0link(i8250poll, 100);
		ctlr->iena = 1;
		print("uarti8250: input polling enabled\n");
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
