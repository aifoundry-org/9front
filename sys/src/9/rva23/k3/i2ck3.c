/*
 * i2ck3.c -- I2C master driver for the SpacemiT K3
 * ("spacemit,k1-i2c" block), polled master mode.
 *
 * The K1 has the same controller at the same addresses; the only difference is
 * the K1 also has an i2c7 (0xD401D000, irq 18, apbc 0x68), unused on the BPI-F3.
 *
 * Buses, SoC properties (k3-pico-itx.dtb and com260.dtb, clock 400000 in both):
 *
 *	label   base        irq   pico-itx                     com260-ifx
 *	------  ----------  ----  ---------------------------  ---------------------------
 *	i2c0    0xD4010800   36   disabled                     FUSB301 CC @0x25
 *	i2c1    0xD4011000   37   FUSB301 CC @0x25             EEPROM @0x50
 *	i2c2    0xD4012000   38   EEPROM GT24C02B @0x50        (no devices)
 *	i2c3    0xF0614000   39   ES8326B audio @0x19          panel @0x45, touch ft5426 @0x38
 *	i2c4    0xD4012800   40   disabled                     disabled
 *	i2c5    0xD4013800   41   disabled                     CSI cameras imx219@10, ov5647@36, ov5640@3c, imx415@37
 *	i2c6    0xD4018800   70   disabled                     ctf2301 @0x4c
 *	i2c8    0xD401D800   19   PMIC P1 @0x41 (node disabled) disabled
 *
 * ri2c0/ri2c1 (0xC0886000/100, irqs 272/273) are deliberately not inited:
 * their clock/reset syscon ("spacemit,k3-syscon-rcpu-i2cctrl" @0xC0886F00) is
 * in the RCPU (real-time core) address range, and the only documented device
 * (VR AU4562 @0x75 on ri2c1, per the schematic) is power-sequencing hardware
 * the RCPU should own. The AP becoming a second, uncoordinated master on that
 * bus risks corrupting the regulator; check whether an AP<->RCPU mailbox path
 * is intended before touching these.
 *
 * The two "google,cros-ec-i2c-tunnel" buses (remote-bus 1 and 4) are proxied
 * through the EC over eSPI, not memory-mapped controllers; they need a client
 * on the cros_ec transport, not this driver.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "../port/i2c.h"

/*
 * spacemit,k1-i2c registers: icr, isr, isar, idbr, ilcr, iwcr, irst_cyc, ibmr,
 * four bytes apart.
 *
 * This is a byte-at-a-time controller, not a Synopsys DesignWare APB layout.
 * The ILCR/IWCR timing values are left at their power-on-reset defaults for
 * standard and fast mode.
 */
enum {
	Icr			= 0x00/4,		/* control register */
	Isr			= 0x04/4,		/* status register */
	Isar		= 0x08,		/* slave address (slave mode, unused) */
	Idbr		= 0x0C/4,		/* data buffer */
	Ilcr		= 0x10/4,		/* load count (clock timing) */
	Iwcr		= 0x14/4,		/* wait count */
	Irstcyc		= 0x18/4,		/* bus reset cycle counter */
	Ibmr		= 0x1C/4,		/* bus monitor (raw SCL/SDA) */
	Iwfifo		= 0x20,		/* write fifo, unused: byte-at-a-time */
};

/* ICR bits. The *IE bits only gate the IRQ line; the driver is polled. */
enum {
	Crstart		= 1<<0,		/* start condition */
	Crstop		= 1<<1,		/* stop condition */
	Cracknak	= 1<<2,		/* 1 = send/expect NAK, 0 = ACK */
	Crtb		= 1<<3,		/* transfer byte */
	Crmodefast	= 1<<8,		/* 1 = fast mode (400kHz), 0 = standard */
	Crmodemask	= 3<<8,		/* mode is a 2-bit field (room for hs) */
	Crur		= 1<<10,	/* unit reset */
	Crma		= 1<<12,	/* master abort */
	Crscle		= 1<<13,	/* master SCL output enable */
	Criue		= 1<<14,	/* I2C unit enable */
	Craldie		= 1<<18,	/* arbitration-loss interrupt enable */
	Critie		= 1<<19,	/* tx-empty interrupt enable */
	Crirfie		= 1<<20,	/* rx-full interrupt enable */
	Crgcd		= 1<<21,	/* general call disable; set at init so we
				 * never ack the general-call address */
	Crbeie		= 1<<22,	/* bus-error interrupt enable */
	Crsadie		= 1<<23,	/* slave-address interrupt enable */
	Crssdie		= 1<<24,	/* slave-stop interrupt enable */
};

/*
 * ISR bits. Srebb, Srmsd and Srtxdone appear in only one description
 * of this block; this driver does not rely on them.
 */
enum {
	Srrwm		= 1<<13,	/* read/write mode of last transfer */
	Sracknak	= 1<<14,	/* ACK/NAK of last byte: 1 = NAK */
	Srub		= 1<<15,	/* unit busy */
	Sribb		= 1<<16,	/* bus busy */
	Srebb		= 1<<17,	/* early bus busy (single-sourced) */
	Srald		= 1<<18,	/* arbitration loss */
	Srite		= 1<<19,	/* tx buffer empty, ready for next byte */
	Srirf		= 1<<20,	/* rx buffer full, byte received */
	Srgcad		= 1<<21,	/* general call address detected */
	Srbed		= 1<<22,	/* bus error (e.g. no ACK/NAK) */
	Srsad		= 1<<23,	/* slave address detected */
	Srssd		= 1<<24,	/* slave stop detected */
	Srmsd		= 1<<26,	/* master stop detected (single-sourced) */
	Srtxdone	= 1<<27,	/* transaction done (single-sourced) */
};

/*
 * AP i2c clock/reset gates in the "apbc"/"apbc2" syscon blocks.
 *
 * Each AP i2c controller sits behind its own gate; until enabled the
 * registers read back all zero, so the gate must be opened before
 * touching the controller.
 *
 * The offsets below are the K1's; ApbcTsen (0x6C) is the same offset
 * k3thermal.c uses for the TSEN sensor, which cross-validates the K1
 * layout for K3's "spacemit,k3-syscon-apbc".
 *
 * Bit layout per the vendor TRM: bit0 = APB/bus clock, bit1 =
 * functional clock, bit2 = reset (1 = held in reset).
 *
 * i2c0/1/2/4/5/6/8 are in the apbc block; i2c3 is in apbc2.
 * TWSI8 (i2c8) is special: its gate always reads back zero, so it
 * must be written with a literal value, not read-modify-written, to
 * avoid clobbering the mux-select bits.
 */
enum {
	ApbcPhys	= 0xD4015000ull,
	Apbc2Phys	= 0xF0610000ull,

	ApbcTwsi8	= 0x20/4,		/* i2c8 -- special, write-only */
	ApbcTwsi0	= 0x2C/4,		/* i2c0 */
	ApbcTwsi1	= 0x30/4,		/* i2c1 */
	ApbcTwsi2	= 0x38/4,		/* i2c2 */
	ApbcTwsi4	= 0x40/4,		/* i2c4 */
	ApbcTwsi5	= 0x4C/4,		/* i2c5 */
	ApbcTwsi6	= 0x60/4,		/* i2c6 */
	ApbcTsen	= 0x6C/4,		/* not i2c; cross-check reference */

	Apbc2Twsi3	= 0x08/4,		/* i2c3, in apbc2 */

	ApbcBusEn	= 1<<0,		/* APB/bus clock enable */
	ApbcFuncEn	= 1<<1,		/* functional clock enable */
	ApbcReset	= 1<<2,		/* 1 = held in reset, 0 = released */
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	I2Cbus	i2c;		/* bus struct handed to addi2cbus() */
	char	*name;
	uintptr	pa;			/* physical MMIO base */
	u32int	*base;		/* mapped virtual base */
	ulong	irq;		/* informational; driver is polled */
	ulong	speed;		/* 100000 or 400000 */
	int	rdomain;		/* 1 if bus is in the RCPU domain */
	int	glitchnofix;	/* 1 if dt has "spacemit,sda-glitch-nofix" */
	uintptr	clkphys;	/* apbc/apbc2 base for this bus's gate, 0 if none */
	ulong	clkoff;		/* gate register offset in that block */
	int	clkwriteonly;	/* 1 for i2c8: never read-modify-write the gate */
};

typedef struct Cfg Cfg;
struct Cfg {
	char	*name;
	uintptr	pa;
	ulong	irq;
	ulong	speed;
	int	rdomain;
	int	glitchnofix;
	uintptr	clkphys;
	ulong	clkoff;
	int	clkwriteonly;
};

/*
 * Registers all controller instances regardless of a board's dt status;
 * probebus() finds nothing on buses with no wired devices.
 * glitchnofix is only set for i2c8 (k3-pico-itx.dtb).
 */
static Cfg cfg[] = {
	{ "i2c0",	0xD4010800,	36,	400000,	0,	0,	ApbcPhys,	ApbcTwsi0,	0, },	/* pico-itx: unused; com260-ifx: FUSB301 CC @0x25 */
	{ "i2c1",	0xD4011000,	37,	400000,	0,	0,	ApbcPhys,	ApbcTwsi1,	0, },	/* pico-itx: FUSB301 CC @0x25; com260-ifx: EEPROM @0x50 */
	{ "i2c2",	0xD4012000,	38,	400000,	0,	0,	ApbcPhys,	ApbcTwsi2,	0, },	/* pico-itx: EEPROM GT24C02B @0x50; com260-ifx: no devices */
	{ "i2c3",	0xF0614000,	39,	400000,	0,	0,	Apbc2Phys,	Apbc2Twsi3,	0, },	/* pico-itx: ES8326B audio @0x19; com260-ifx: RPi-style panel+touch @0x45/0x38 */
	{ "i2c4",	0xD4012800,	40,	400000,	0,	0,	ApbcPhys,	ApbcTwsi4,	0, },	/* unused on both boards */
	{ "i2c5",	0xD4013800,	41,	400000,	0,	0,	ApbcPhys,	ApbcTwsi5,	0, },	/* pico-itx: unused; com260-ifx: CSI cameras (imx219/ov5647/ov5640/imx415) */
	{ "i2c6",	0xD4018800,	70,	400000,	0,	0,	ApbcPhys,	ApbcTwsi6,	0, },	/* pico-itx: unused; com260-ifx: ctf2301 @0x4c */
	{ "i2c8",	0xD401D800,	19,	400000,	0,	1,	ApbcPhys,	ApbcTwsi8,	1, },	/* pico-itx: PWR_I2C, PMIC P1 @0x41; com260-ifx: disabled. clkwriteonly: TWSI8 gate reads back zero always */
	{ "ri2c0",	0xC0886000,	272,	400000,	1,	0,	0,		0,		0, },	/* NOT IMPL, pending confirmation of domain -- see header */
	{ "ri2c1",	0xC0886100,	273,	400000,	1,	0,	0,		0,		0, },	/* NOT IMPL, pending confirmation of domain -- schematic: VR AU4562 @0x75 */
};

static Ctlr ctlr[nelem(cfg)];

static void
wreg(u32int *base, uintptr reg, u32int v)
{
	base[reg] = v;
	coherence();
}

/*
 * Wait for (bit|Srbed); return the status word in *statusp.
 * Only clear the bits we waited for, not the full snapshot:
 * a concurrent bit that settled in the same read (notably Sracknak)
 * must survive for the caller to check.
 */
static int
k3i2cwait(Ctlr *c, ulong bit, ulong *statusp)
{
	int i;
	ulong s;

	for(i = 0; i < 200000; i++){
		s = c->base[Isr];
		if(s & (bit|Srbed)){
			*statusp = s;
			wreg(c->base, Isr, s & (bit|Srbed));	/* write-1-to-clear, only what we waited for */
			return 0;
		}
	}
	return -1;
}

/*
 * Poll until (bit) is clear in ISR or Srbed fires or timeout.
 * Used to settle Sracknak after Srite. The 50000 bound is deliberately wide;
 * dial it down to the smallest value that still probes correctly, since
 * bus probing calls this for every address.
 */
static int
k3i2cwaitclear(Ctlr *c, ulong bit)
{
	int i;
	ulong s;

	for(i = 0; i < 50000; i++){
		s = c->base[Isr];
		if(s & Srbed){
			wreg(c->base, Isr, s);
			return -1;
		}
		if((s & bit) == 0)
			return 0;
	}
	return -1;
}

/* wait for the bus to be idle before starting a transaction */
static int
k3i2cidle(Ctlr *c)
{
	int i;

	for(i = 0; i < 200000; i++)
		if((c->base[Isr] & Sribb) == 0)
			return 0;
	return -1;
}

/* twiddle START/STOP/ACKNAK/TB on top of the persistent ICR config bits */
static void
k3i2cpulse(Ctlr *c, int start, int stop, int nak)
{
	ulong icr;

	icr = c->base[Icr];
	icr &= ~(Crstart|Crstop|Cracknak);
	icr |= Crtb;
	if(start)
		icr |= Crstart;
	if(stop)
		icr |= Crstop;
	if(nak)
		icr |= Cracknak;
	wreg(c->base, Icr, icr);
}

static int
k3i2cputc(Ctlr *c, int byte, int start, int stop)
{
	ulong s;

	wreg(c->base, Idbr, byte & 0xFF);
	k3i2cpulse(c, start, stop, 0);
	if(k3i2cwait(c, Srite, &s) < 0)
		return -1;
	if(s & Srbed)
		return -1;
	/* ACK/NAK settles after Srite; wait for Sracknak clear (ACK) */
	return k3i2cwaitclear(c, Sracknak);
}

static int
k3i2cgetc(Ctlr *c, int start, int stop, int nak)
{
	ulong s;

	k3i2cpulse(c, start, stop, nak);
	if(k3i2cwait(c, Srirf, &s) < 0)
		return -1;
	if(s & Srbed)
		return -1;
	return c->base[Idbr] & 0xFF;
}

/*
 * bus->io. pkt[0] carries the 7-bit device address and the final transfer
 * direction (putaddr()); following bytes are subaddress bytes, always written.
 * If ilen>0, repeat start back to the address (the address is just the first
 * byte on the wire; there's no target-address register) and read ilen bytes.
 *
 * 10-bit addressing is not implemented.
 */
static int
k3i2cio(I2Cdev *dev, uchar *pkt, int olen, int ilen)
{
	Ctlr *c = dev->bus->ctlr;
	int addrlen, i, nwrite, quick;
	uchar addrbyte;

	if(dev->a10)
		return -1;

	addrlen = 1;
	if(olen < addrlen)
		return -1;

	if(k3i2cidle(c) < 0)
		return -1;

	nwrite = olen - addrlen;
	quick = nwrite == 0 && ilen == 0;

	addrbyte = pkt[0];
	if(nwrite > 0)
		addrbyte &= ~1;		/* subaddress bytes must be written */

	if(quick){
		/*
		 * This controller cannot combine START and STOP in one TB pulse.
		 * Split into START+write(addr|R), then  a closing STOP+read+NAK
		 * (a read, so probing never puts real data on the bus).
		 */
		if(k3i2cputc(c, addrbyte | 1, 1, 0) < 0)
			return -1;
		if(k3i2cgetc(c, 0, 1, 1) < 0)
			return -1;
		return olen;
	}

	if(k3i2cputc(c, addrbyte, 1, 0) < 0)
		return -1;

	for(i = 1; i < olen; i++)
		if(k3i2cputc(c, pkt[i], 0, i == olen-1 && ilen == 0) < 0)
			return -1;

	if(ilen > 0){
		if(nwrite > 0){
			/* repeated start; pkt[0] already has the read bit set */
			if(k3i2cputc(c, pkt[0], 1, 0) < 0)
				return -1;
		}
		for(i = 0; i < ilen; i++){
			int b = k3i2cgetc(c, 0, i == ilen-1, i == ilen-1);
			if(b < 0)
				return -1;
			pkt[olen+i] = b;
		}
	}

	return olen + ilen;
}

/*
 * Enable this bus's clock gate and release its reset in the
 * apbc/apbc2 syscon block. apbc/apbc2 are mapped once and shared
 * by every bus whose gate lives in the same block.
 *
 * Buses with clkphys == 0 (ri2c0/ri2c1) always fail: their syscon
 * is in the RCPU domain; see the header note.
 */
static uintptr apbcbase, apbc2base;

static int
k3i2cclkenable(Ctlr *c)
{
	uintptr *basep;
	u32int *base;
	ulong v;

	if(c->clkphys == 0)
		return -1;	/* no known gate for this bus yet (ri2c0/ri2c1) */

	basep = (c->clkphys == ApbcPhys)? &apbcbase: &apbc2base;
	if(*basep == 0){
		*basep = (uintptr)vmap(c->clkphys, 4096);
		if(*basep == 0)
			return -1;
	}
	base = (u32int*)*basep;

	if(c->clkwriteonly){
		/*
		 * TWSI8: gate always reads back zero, so write a known'good literal:
		 * bus+func enabled, reset released, mux forced to 0.
		 */
		base[c->clkoff] = ApbcBusEn | ApbcFuncEn;
		coherence();
		return 0;
	}

	v = base[c->clkoff];
	base[c->clkoff] = (v & ~ApbcReset) | ApbcBusEn | ApbcFuncEn;
	coherence();
	return 0;
}

/*
 * Failures here must go through error(), not a plain return -1:
 * probebus()'s "user && waserror() || (*bus->init)(bus)" takes the
 * init side on a plain nonzero return, so nexterror() runs with no
 * exception in flight -- a locked Q then panics the kernel
 * ("qunlock called with qlock not held"). Only bus->init has this
 * requirement; bus->io may return -1.
 */
static int
k3i2cinit(I2Cbus *bus)
{
	Ctlr *c = bus->ctlr;
	ulong icr;

	if(c->rdomain || k3i2cclkenable(c) < 0)
		error(Eio);	/* ri2c0/ri2c1: not implemented, see header */

	if(c->base == nil){
		c->base = vmap(c->pa, 4096);
		if(c->base == nil)
			error(Eio);
	}

	/* soft reset, then bring up in polled master mode */
	wreg(c->base, Icr, Crur);
	delay(1);
	wreg(c->base, Icr, 0);

	/* clear status latched before our reset, by snapshot rather than mask */
	wreg(c->base, Isr, c->base[Isr]);

	icr = Crscle | Crgcd;
	if(c->speed >= 400000)
		icr |= Crmodefast;
	wreg(c->base, Icr, icr);

	/* IUE goes high last, after the config bits settle */
	wreg(c->base, Icr, icr | Criue);
	delay(1);

	if(c->glitchnofix)
		wreg(c->base, Irstcyc, c->base[Irstcyc] | (1<<7));

	bus->probed = 0;
	return 0;
}

/* register one I2Cbus per table entry */
static void
i2clinkbuses(void)
{
	Ctlr *c;
	int i;

	for(i = 0; i < nelem(cfg); i++){
		c = &ctlr[i];
		memset(c, 0, sizeof *c);
		c->name = cfg[i].name;
		c->pa = cfg[i].pa;
		c->irq = cfg[i].irq;
		c->speed = cfg[i].speed;
		c->rdomain = cfg[i].rdomain;
		c->glitchnofix = cfg[i].glitchnofix;
		c->clkphys = cfg[i].clkphys;
		c->clkoff = cfg[i].clkoff;
		c->clkwriteonly = cfg[i].clkwriteonly;

		c->i2c.name = c->name;
		c->i2c.speed = c->speed;
		c->i2c.ctlr = c;
		c->i2c.init = k3i2cinit;
		c->i2c.io = k3i2cio;

		addi2cbus(&c->i2c);
	}
}

/* called at boot via config "link:" mechanism */
void
i2ck3link(void)
{
	/* the bus table is the K3's; the K1 has the same block elsewhere */
	if(dtbfind("spacemit,k3", 0xD4010800) == nil)
		return;
	i2clinkbuses();
}
