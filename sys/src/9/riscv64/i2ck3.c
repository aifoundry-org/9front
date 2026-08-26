/*
 * i2ck3.c -- I2C master controller driver for the SpacemiT K3 SoC
 * ("spacemit,k1-i2c" IP), for 9front.
 *
 * STATUS: working, validated on real k3-pico-itx hardware. i2c1's
 * FUSB301 @0x25, i2c2's EEPROM @0x50, and i2c3's ES8326B codec @0x19
 * all show up correctly under #J, and every bus/address with no
 * hardware on it stays correctly empty (no false positives). Getting
 * here needed two hardware-specific fixes beyond the register-level
 * cross-checks below:
 *
 *   1. Clock gating (see k3i2cclkenable()): every AP i2c controller
 *      sits behind its own gate in the "apbc"/"apbc2" syscon blocks
 *      and reads back all-zero registers until that gate is enabled
 *      -- confirmed empirically (a write to ICR read back as 0x0)
 *      before this was added.
 *
 *   2. This IP cannot combine START and STOP in the same TB pulse
 *      (see the "quick" case in k3i2cio()): a zero-data probe has to
 *      be split into two separate pulses, confirmed both empirically
 *      and against SpacemiT's own u-boot driver's __i2c_probe_chip().
 *
 * NOT implemented: ri2c0/ri2c1, pending confirmation of their domain.
 * These live in the RCPU (real-time companion core) address range and
 * their DT-listed device (VR AU4562 @0x75, per the schematic) looks
 * like RCPU-side power-sequencing hardware, not something the AP
 * should be sharing a bus with as a second, uncoordinated I2C master.
 * See the longer note near k3i2cclkenable() below.
 *
 * REVISION NOTE: an earlier version of this file assumed a Synopsys
 * DesignWare APB I2C register layout.  That was WRONG.  Both boards'
 * device trees say:
 *
 *	compatible = "spacemit,k1-i2c";
 *
 * which is a different, byte-at-a-time controller (closer in spirit
 * to the old Marvell PXA i2c block than to DesignWare's FIFO-based
 * IP).  The register map and bit positions below were cross-checked
 * against two independent, real sources: the upstream Linux driver
 * under review (drivers/i2c/busses/i2c-k1.c, Troy Mitchell/SpacemiT,
 * mainlined 2024-2025) and SpacemiT's own shipped u-boot driver
 * (spacemit_i2c.c/.h, GPL-2.0+, (C) 2023 Spacemit). Both agree
 * exactly on every register offset and bit position used here; none
 * of that GPL code is reproduced below -- only the factual register
 * layout and transfer protocol (not copyrightable expression) were
 * used to write this from scratch in 9front's own driver style.
 * Two corrections came out of that cross-check that the Linux-only
 * pass had missed: ICR_GCD (general-call disable, confirmed bit 21)
 * needs to be set at init, and every transaction should confirm the
 * bus is idle (ISR IBB clear) before pulsing the first byte.
 *
 * This file has been checked against device trees from two different
 * carrier boards built around the same K3 SoC:
 *
 *	k3-pico-itx.dtb   model "SpacemiT K3 Pico ITX"
 *	                  compatible "spacemit,k3-pico-itx"
 *	com260.dtb        model "SpacemiT K3 Com260 IFX"
 *	                  compatible "spacemit,k3-com260-ifx"
 *
 * Every controller's physical base address, register block size
 * (0x38), interrupt number and clock-frequency (400000, fast mode)
 * are byte-for-byte identical between the two -- expected, since
 * those live in the SoC, not the carrier board.  What differs
 * per-board is only which buses are wired out (dt status) and which
 * peripheral hangs off each one.  Bus table, combining both:
 *
 *	label   phys base    irq   pico-itx status/device        com260-ifx status/device
 *	------  -----------  ----  --------------------------    ----------------------------
 *	i2c0    0xD4010800    36   disabled                      okay: FUSB301 (CC) @0x25
 *	i2c1    0xD4011000    37   okay: FUSB301 (CC) @0x25       okay: EEPROM @0x50
 *	i2c2    0xD4012000    38   okay: EEPROM GT24C02B @0x50    okay, no devices declared
 *	i2c3    0xF0614000    39   okay: ES8326B audio @0x19      okay: raspits-panel@45,
 *	                                                           raspits-touch-ft5426@38
 *	i2c4    0xD4012800    40   disabled                       disabled
 *	i2c5    0xD4013800    41   disabled                       okay: CSI cameras
 *	                                                           (imx219@10, ov5647@36,
 *	                                                           ov5640@3c, imx415@37)
 *	i2c6    0xD4018800    70   disabled                       okay: ctf2301@4c
 *	i2c8    0xD401D800    19   okay: PWR_I2C, PMIC P1 @0x41    disabled
 *	                           (p1@41 node itself disabled
 *	                           in this board's dts)
 *	ri2c0   0xC0886000   272   NOT IMPL (see below)           NOT IMPL (see below)
 *	ri2c1   0xC0886100   273   NOT IMPL (schematic: R_I2C0/1  NOT IMPL (see below)
 *	                           RCPU domain, VR AU4562 @0x75
 *	                           on ri2c1)
 *
 * Note the same physical bus can carry a different peripheral on
 * each board (e.g. the CC-logic FUSB301 sits on i2c1 on Pico-ITX but
 * on i2c0 on Com260-IFX) -- this driver only registers the buses by
 * their generic i2cN/ri2cN name; which device is expected on which
 * bus is a board/config matter for whatever calls i2cbus(), not for
 * this file.
 *
 * All of the above use clock-frequency = 400000 (fast mode) in both
 * device trees.
 *
 * NOT handled by this driver:
 *
 *   - ri2c0/ri2c1: NOT IMPLEMENTED, pending confirmation of their
 *     domain. reg/clocks/resets all point into the RCPU (real-time
 *     companion core) address range (0xC0886xxx, syscon
 *     "spacemit,k3-syscon-rcpu-i2cctrl" @ 0xC0886F00), and the
 *     schematic's only documented device on this bus is a VR
 *     (AU4562 @0x75 on ri2c1) -- power-sequencing hardware, the kind
 *     of thing an RTOS running on the RCPU would own and manage on
 *     its own timing. If that's right, the AP (this driver) has no
 *     business becoming a second, uncoordinated I2C master on that
 *     bus -- two masters without arbitration can corrupt each other's
 *     transactions, and one of them would be a voltage regulator.
 *     Before writing anything here, that ownership question needs an
 *     actual answer (ideally: is there an ACPU<->RCPU IPC/mailbox
 *     path meant to be used instead of touching these registers
 *     directly?), not just a register offset. c->rdomain deliberately
 *     always fails k3i2cinit() until that's resolved -- see the note
 *     at k3i2cclkenable().
 *
 *   - The two "google,cros-ec-i2c-tunnel" buses (remote-bus 1 and 4)
 *     seen under the cros-ec node are NOT memory-mapped controllers
 *     at all -- they're virtual buses proxied through the EC over
 *     eSPI (the same EC/cros_ec path behind the fan's hwmon7 node).
 *     A real implementation needs a client riding the cros_ec eSPI
 *     transport, not this MMIO driver.
 *
 *   - clocks/resets/power-domains: this driver assumes the buses
 *     marked "okay" already have their clock enabled and reset
 *     deasserted by firmware/bootloader (typical for status=okay
 *     nodes at kernel boot). If that's not the case on your build,
 *     the SoC clock/reset controller (dtb phandle target of the
 *     "clocks"/"resets" properties) needs to be driven first.
 *
 * Board matching: i2ck3link() is gated on dtbsocmatch("spacemit,k3"),
 * the same check (and the same string) k3thermal.c's k3thermalinit()
 * uses -- no point mapping and poking these registers if we're not
 * actually running on this SoC. Confirmed working on real hardware.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "../port/i2c.h"

/*
 * spacemit,k1-i2c registers. Offsets confirmed against both the
 * upstream Linux driver and SpacemiT's u-boot driver (struct
 * spacemit_i2c: icr/isr/isar/idbr/ilcr/iwcr/irst_cyc/ibmr, in that
 * order, 4 bytes apart).
 */
enum {
	Icr		= 0x00,		/* control register */
	Isr		= 0x04,		/* status register */
	Isar		= 0x08,		/* slave address register (slave mode; unused) */
	Idbr		= 0x0C,		/* data buffer register */
	Ilcr		= 0x10,		/* load count register (clock timing) */
	Iwcr		= 0x14,		/* wait count register */
	Irstcyc		= 0x18,		/* bus reset cycle counter */
	Ibmr		= 0x1C,		/* bus monitor register (raw SCL/SDA state) */
	Iwfifo		= 0x20,		/* write fifo -- Linux-only, not in u-boot's
					 * struct; unused: we run byte-at-a-time. */
};

/* ICR bits -- all positions below confirmed directly against the real
 * spacemit_i2c.h header (ICR_START..ICR_SADIE match bit-for-bit). */
enum {
	Crstart		= 1<<0,		/* start condition */
	Crstop		= 1<<1,		/* stop condition */
	Cracknak	= 1<<2,		/* 1 = send/expect NAK, 0 = ACK */
	Crtb		= 1<<3,		/* transfer byte: kick one byte across the wire */
	Crmodefast	= 1<<8,		/* 1 = fast mode (400kHz), 0 = standard (100kHz) */
	Crmodemask	= 3<<8,		/* mode field is 2 bits wide (room for hs mode) */
	Crur		= 1<<10,	/* unit reset */
	Crma		= 1<<12,	/* master abort */
	Crscle		= 1<<13,	/* master SCL clock output enable */
	Criue		= 1<<14,	/* I2C unit enable */
	Craldie		= 1<<18,	/* arbitration-loss interrupt enable */
	Critie		= 1<<19,	/* tx-empty interrupt enable */
	Crirfie		= 1<<20,	/* rx-full interrupt enable */
	Crgcd		= 1<<21,	/* general call disable -- always set: we
					 * don't want to ack the general-call address */
	Crbeie		= 1<<22,	/* bus-error interrupt enable */
	Crsadie		= 1<<23,	/* slave-address-detected interrupt enable */
	Crssdie		= 1<<24,	/* slave-stop-detected interrupt enable */
	/*
	 * Craldie/Critie/Crirfie/Crbeie/Crsadie/Crssdie mirror the ISR
	 * bit positions below and gate the IRQ line; since this driver
	 * is fully polled (no interrupt controller wired up) they're
	 * defined for completeness but never set -- status bits latch
	 * in ISR regardless of whether the matching *IE bit is enabled.
	 */
};

/* ISR bits -- ISR_RWM..ISR_SSD confirmed directly against the real
 * spacemit_i2c.h header, bit-for-bit; Srebb/Srmsd/Srtxdone are only
 * in the Linux driver under review, not in u-boot's header (not
 * contradicted either -- u-boot's driver just doesn't use them). */
enum {
	Srrwm		= 1<<13,	/* read/write mode (of the last addressed transfer) */
	Sracknak	= 1<<14,	/* ACK/NAK status of the last byte: 1 = NAK */
	Srub		= 1<<15,	/* unit busy */
	Sribb		= 1<<16,	/* i2c bus busy */
	Srebb		= 1<<17,	/* early bus busy (Linux-only; not in u-boot's
					 * header, but not contradicted either) */
	Srald		= 1<<18,	/* arbitration loss detected */
	Srite		= 1<<19,	/* tx buffer empty -- byte accepted, ready for next */
	Srirf		= 1<<20,	/* rx buffer full -- byte received */
	Srgcad		= 1<<21,	/* general call address detected */
	Srbed		= 1<<22,	/* bus error (e.g. no ACK/NAK seen) */
	Srsad		= 1<<23,	/* slave address detected */
	Srssd		= 1<<24,	/* slave stop detected */
	Srmsd		= 1<<26,	/* master stop detected (Linux-only, see Srebb note) */
	Srtxdone	= 1<<27,	/* transaction done (Linux-only, see Srebb note) */
};

/*
 * Peripheral clock/reset gates ("APBC"/"APBC2" syscon blocks).
 *
 * This is the piece that was missing and caused every AP i2c bus to
 * read back all-zero registers (see the big comment further down,
 * at k3i2cclkenable()): each i2c controller instance sits behind its
 * own clock+reset gate in one of two SoC syscon blocks, and nothing
 * enables those gates before this driver touches the controller's
 * own registers.
 *
 * Offsets and bit layout confirmed from real, public Linux clock
 * driver source (drivers/clk/spacemit/ccu-k1.c) and the SpacemiT I2C
 * dt-bindings review thread describing APBC_TWSI0_CLK_RST -- both for
 * the K1 SoC, not K3, but K3's apbc block already cross-validates:
 * ApbcTsen below (0x6C) is the exact same offset k3thermal.c already
 * uses successfully for the real TSEN sensor on this board, so the
 * K1 apbc layout transfers directly to K3's "spacemit,k3-syscon-apbc"
 * (same vendor, same block, likely same or a superset register map).
 * The bit layout (bit0=bus/APB clock, bit1=func clock, bit2=reset) is
 * independently confirmed twice: once by the CCU_GATE_DEFINE() macro
 * arguments in that same driver source, and once by SpacemiT's own
 * TRM text quoted in the i2c dt-bindings review ("bit[1] - FNCLK...
 * bit[0] - APBCLK... 2 RST ... resets both the APB and functional
 * domain").
 *
 * i2c0/1/2/4/5/6/8 live in the main "apbc" block (same physical base
 * as k3thermal.c's ApbcPhys, 0xD4015000). i2c3 lives in a *different*
 * block, "apbc2" (0xF0610000, "spacemit,k3-syscon-apbc2" in the dtb,
 * phandle target of i2c3's "clocks" property) -- confirmed directly
 * from k3-pico-itx.dtb, not inferred.
 *
 * i2c8 (TWSI8) is a documented special case: its gate register has a
 * hardware quirk where reads always return zero (see the Linux commit
 * "clk: spacemit: k1: Add TWSI8 bus and function clocks"). Doing a
 * normal read-modify-write on it would silently write back a bogus
 * all-zero base value (clobbering the mux-select bits the real
 * register also contains, per that same commit message) instead of
 * preserving them. So TWSI8 is written blindly with a literal value
 * (bus+func enable, reset deasserted, mux forced to 0 -- mirroring
 * exactly what the real Linux driver hardcodes for it) rather than
 * read-modify-written like every other gate here.
 */
enum {
	ApbcPhys	= 0xD4015000ull,
	Apbc2Phys	= 0xF0610000ull,

	ApbcTwsi8	= 0x20,		/* i2c8 -- SPECIAL, see above: write-only */
	ApbcTwsi0	= 0x2C,		/* i2c0 */
	ApbcTwsi1	= 0x30,		/* i2c1 */
	ApbcTwsi2	= 0x38,		/* i2c2 */
	ApbcTwsi4	= 0x40,		/* i2c4 */
	ApbcTwsi5	= 0x4C,		/* i2c5 */
	ApbcTwsi6	= 0x60,		/* i2c6 */
	ApbcTsen	= 0x6C,		/* not used here -- cross-check reference, see comment above */

	Apbc2Twsi3	= 0x08,		/* i2c3, in the apbc2 block, not apbc */

	ApbcBusEn	= 1<<0,		/* APB/bus clock enable */
	ApbcFuncEn	= 1<<1,		/* functional clock enable */
	ApbcReset	= 1<<2,		/* 1 = held in reset, 0 = released */
};

typedef struct Ctlr Ctlr;
struct Ctlr {
	I2Cbus	i2c;		/* generic bus struct handed to addi2cbus() */
	char	*name;
	uintptr	pa;		/* physical MMIO base */
	uintptr	base;		/* mapped virtual base, set on first init */
	ulong	irq;		/* informational only; driver runs polled */
	ulong	speed;		/* 100000 or 400000 */
	int	rdomain;	/* 1 if this bus lives in the RCPU/"R" domain */
	int	glitchnofix;	/* 1 if dt has "spacemit,sda-glitch-nofix" */
	uintptr	clkphys;	/* apbc/apbc2 physical base for this bus's gate, 0 if none known */
	ulong	clkoff;		/* gate register offset within that block */
	int	clkwriteonly;	/* 1 for i2c8/TWSI8: never read-modify-write the gate */
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
 * Bus table.  Physical addresses/irqs/speed are SoC properties and
 * are identical on k3-pico-itx.dtb and com260.dtb (Com260 IFX) -- see
 * header comment for the combined per-board device mapping.  This
 * table registers every controller instance regardless of a given
 * board's dt status; probebus() harmlessly finds nothing on a bus
 * that has no devices wired to it on a particular board.
 *
 * glitchnofix mirrors the dt property "spacemit,sda-glitch-nofix",
 * which is only present on i2c8 in k3-pico-itx.dtb (com260.dtb doesn't
 * have it anywhere, and its i2c8 is disabled anyway).
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

static ulong
csr32r(Ctlr *c, ulong r)
{
	return *(ulong*)(c->base + r);
}

static void
csr32w(Ctlr *c, ulong r, ulong v)
{
	*(ulong*)(c->base + r) = v;
	coherence();
}

/*
 * Wait for one of (bit | Srbed); returns the status word via
 * *statusp.  Only clears (bit|Srbed) on the way out -- NOT the
 * whole snapshot -- so any other bit that happened to be set in
 * the same read (notably Sracknak, which can settle in the same
 * poll iteration as Srite) survives for a caller to check
 * separately. Clearing the full snapshot here was a real bug in an
 * earlier version of this file: it could silently wipe a NAK before
 * k3i2cputc()'s dedicated ACK/NAK check ever saw it.
 */
static int
k3i2cwait(Ctlr *c, ulong bit, ulong *statusp)
{
	int i;
	ulong s;

	for(i = 0; i < 200000; i++){
		s = csr32r(c, Isr);
		if(s & (bit|Srbed)){
			*statusp = s;
			csr32w(c, Isr, s & (bit|Srbed));	/* write-1-to-clear, only what we waited for */
			return 0;
		}
	}
	return -1;
}

/*
 * Poll until (bit) is clear in ISR, or Srbed fires, or we time out.
 * Used only to settle ISR_ACKNAK right after ITE.
 *
 * TEMPORARILY WIDENED from 1000 to 50000 iterations: with the clock
 * gates now enabled, every AP bus reads a healthy ICR/IBMR but the
 * address probe still finds zero devices even on buses with known
 * hardware (i2c1/FUSB301, i2c2/EEPROM on pico-itx). One live
 * hypothesis is that this bound, sized for "ACKNAK lags ITE by about
 * one cycle", was too tight for this IP/interconnect and was cutting
 * off real ACKs before they settled, not just correctly timing out
 * on genuine NAKs. If devices start showing up with this widened,
 * dial it back down to whatever's the smallest value that still
 * works, to keep bus probing (which does this for every one of ~1136
 * addresses per bus) reasonably fast. If it *doesn't* help, the
 * timeout wasn't the problem and this should be reverted.
 */
static int
k3i2cwaitclear(Ctlr *c, ulong bit)
{
	int i;
	ulong s;

	for(i = 0; i < 50000; i++){
		s = csr32r(c, Isr);
		if(s & Srbed){
			csr32w(c, Isr, s);
			return -1;
		}
		if((s & bit) == 0)
			return 0;
	}
	return -1;
}

/*
 * Wait for the bus to go idle (ISR IBB clear) before starting a
 * transaction.  Confirmed against the real u-boot driver: it checks
 * this before every single byte it pulses, we only need it once,
 * before the address byte, since we then own the bus for the whole
 * transaction until our own STOP.
 */
static int
k3i2cidle(Ctlr *c)
{
	int i;

	for(i = 0; i < 200000; i++)
		if((csr32r(c, Isr) & Sribb) == 0)
			return 0;
	return -1;
}

/* twiddle START/STOP/ACKNAK/TB on top of the persistent ICR config bits */
static void
k3i2cpulse(Ctlr *c, int start, int stop, int nak)
{
	ulong icr;

	icr = csr32r(c, Icr);
	icr &= ~(Crstart|Crstop|Cracknak);
	icr |= Crtb;
	if(start)
		icr |= Crstart;
	if(stop)
		icr |= Crstop;
	if(nak)
		icr |= Cracknak;
	csr32w(c, Icr, icr);
}

static int
k3i2cputc(Ctlr *c, int byte, int start, int stop)
{
	ulong s;

	csr32w(c, Idbr, byte & 0xFF);
	k3i2cpulse(c, start, stop, 0);
	if(k3i2cwait(c, Srite, &s) < 0)
		return -1;
	if(s & Srbed)
		return -1;
	/*
	 * ACK/NAK is checked as a second, separate poll after ITE,
	 * matching the real driver: it waits for ISR_ACKNAK to clear
	 * (0 = ACK) rather than trusting the bit's value in the same
	 * snapshot as ITE, since sampling can lag by a cycle.
	 */
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
	return csr32r(c, Idbr) & 0xFF;
}

/*
 * bus->io.  pkt[] is built by devi2c.c's putaddr(): pkt[0] encodes
 * the 7 bit device address and the *final* transfer direction; any
 * bytes after it are subaddress/register bytes that must always be
 * written first.  If ilen>0 we then need a repeated start back to
 * the address byte (this IP has no separate target-address register
 * like DesignWare -- the address is just the first byte on the wire,
 * same as manual bit-banging) followed by ilen bytes read.
 *
 * 10 bit addressing is not implemented: none of this board's devices
 * use it.
 */
static int
k3i2cio(I2Cdev *dev, uchar *pkt, int olen, int ilen)
{
	/*
	 * dev->bus->ctlr is the intended back-pointer -- confirmed
	 * against the real port/i2c.h: I2Cbus has "void *ctlr;" for
	 * exactly this. An earlier version of this file assumed a
	 * field named "aux" instead (it doesn't exist -- that build
	 * failed with "not a member of struct/union: aux") and then,
	 * before seeing the real header, worked around it with a
	 * first-member pointer cast. That cast wasn't wrong, just an
	 * unnecessary hack now that the real, intended field is known.
	 */
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
		addrbyte &= ~1;		/* subaddress bytes must be written, regardless of final direction */

	if(quick){
		/*
		 * This IP cannot combine START and STOP in the same TB
		 * pulse. CONFIRMED two ways, and validated against real
		 * hardware: (1) empirically -- combining them produced
		 * zero bus activity (ISR stayed 0x0 through the entire
		 * wait, not even a NAK) on every address tried; splitting
		 * into two pulses as done below made FUSB301 @0x25 on
		 * i2c1, the EEPROM @0x50 on i2c2, and the ES8326B codec
		 * @0x19 on i2c3 all show up correctly, with every unused
		 * bus/address still correctly empty -- no false positives
		 * introduced. (2) SpacemiT's own u-boot driver's
		 * __i2c_probe_chip() -- the routine that does exactly
		 * this "is anything at this address" test -- always
		 * splits into two separate pulses: START + write(addr|R),
		 * then a dummy STOP + read + NAK. Its condition enum
		 * (NORMAL/START/STOP) doesn't even have a combined
		 * START+STOP value. The closing pulse is a READ, not a
		 * repeat WRITE, specifically so probing never risks
		 * putting meaningful data on the bus -- mirrored here
		 * exactly, regardless of the direction bit devi2c.c's
		 * putaddr() put in pkt[0], since a zero-data "quick"
		 * request only exists to test presence, so direction
		 * doesn't matter to the caller either way.
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
			/* repeated start, re-send address -- pkt[0] already
			 * has the read bit set whenever ilen>0 (devi2c.c's
			 * putaddr() guarantees this) */
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
 * k3i2cclkenable -- enable this bus's clock gate and release its
 * reset in the appropriate apbc/apbc2 syscon block, before touching
 * the i2c controller's own registers.
 *
 * THIS WAS THE ROOT CAUSE of every AP bus reading back all-zero
 * registers (confirmed empirically: after writing a nonzero value to
 * ICR, reading it straight back gave 0x0, on every bus tested,
 * uniformly -- the classic signature of an unclocked/gated
 * peripheral, not a protocol bug). See the big comment above the
 * apbc/apbc2 enum for where these offsets came from.
 *
 * apbc and apbc2 are mapped once and shared across every bus that
 * needs them (multiple i2c controllers' gates live in the same
 * physical block), rather than re-mapping the same range per bus.
 *
 * ri2c0/ri2c1 (c->clkphys == 0, c->rdomain == 1) always return -1
 * here, on purpose: their clock/reset syscon ("spacemit,k3-syscon-
 * rcpu-i2cctrl" @ 0xC0886F00) belongs to the RCPU domain, and it's
 * not yet confirmed whether the AP is even meant to touch these two
 * buses directly. See the "NOT handled by this driver" note near the
 * top of this file for why that's an open ownership question, not
 * just a missing register offset.
 */
static uintptr apbcbase, apbc2base;

static int
k3i2cclkenable(Ctlr *c)
{
	uintptr *basep;
	uintptr base;
	ulong v;

	if(c->clkphys == 0)
		return -1;	/* no known gate for this bus yet (ri2c0/ri2c1) */

	basep = (c->clkphys == ApbcPhys)? &apbcbase: &apbc2base;
	if(*basep == 0){
		*basep = (uintptr)vmap(c->clkphys, 4096);
		if(*basep == 0)
			return -1;
	}
	base = *basep;

	if(c->clkwriteonly){
		/*
		 * TWSI8 (i2c8): the gate register always reads back
		 * zero, so a read-modify-write would silently write
		 * zero for the mux-select bits the real register also
		 * has (per the Linux commit documenting this quirk).
		 * Write a known-good literal instead: bus+func enabled,
		 * reset deasserted, mux forced to 0 -- mirrors exactly
		 * what the real Linux driver hardcodes for this one.
		 */
		*(ulong*)(base + c->clkoff) = ApbcBusEn | ApbcFuncEn;
		coherence();
		return 0;
	}

	v = *(ulong*)(base + c->clkoff);
	*(ulong*)(base + c->clkoff) = (v & ~ApbcReset) | ApbcBusEn | ApbcFuncEn;
	coherence();
	return 0;
}

/*
 * bus->init.
 *
 * IMPORTANT: failures here MUST be signaled with error(), not a
 * plain "return -1". Confirmed against devi2c.c's probebus():
 *
 *	if(user && waserror()
 *	|| (*bus->init)(bus)){
 *		leavebus(bus);
 *		if(user) nexterror();
 *		return;
 *	}
 *
 * When called interactively (e.g. via ls, "user" is true), a plain
 * nonzero return from bus->init() takes the "(*bus->init)(bus)"
 * side of that ||, which never actually unwound through waserror()
 * -- so nexterror() is called with no real exception in flight. On
 * this kernel that manifests as "qunlock called with qlock not
 * held" and the bus never appears (confirmed empirically: this
 * function used to "return -1" for the rdomain stub below, and
 * ls /dev/ri2c0 crashed with exactly that message every time; the
 * AP buses, whose init() never hit a "return -1" path, didn't). The
 * probe loop further down in probebus() is fine with a plain -1
 * return from bus->io() -- only bus->init() has this requirement.
 */
static int
k3i2cinit(I2Cbus *bus)
{
	/* see k3i2cio()'s comment: bus->ctlr is the real back-pointer */
	Ctlr *c = bus->ctlr;
	ulong icr;

	if(c->rdomain || k3i2cclkenable(c) < 0){
		/*
		 * ri2c0/ri2c1: NOT IMPLEMENTED, pending confirmation of
		 * their domain (see k3i2cclkenable() and the header).
		 * Any AP bus whose clkphys/clkoff isn't set (there
		 * shouldn't be any left) falls into the same bucket.
		 */
		error(Eio);
	}

	if(c->base == 0){
		/*
		 * vmap() maps a device physical range for MMIO access.
		 * If this port instead uses an identity-mapped kernel
		 * (KADDR()), swap the line below accordingly.
		 */
		c->base = (uintptr)vmap(c->pa, 4096);
		if(c->base == 0)
			error(Eio);
	}

	/* soft reset the unit, then bring it up in polled master mode */
	csr32w(c, Icr, Crur);
	delay(1);
	csr32w(c, Icr, 0);

	/* clear any status bits latched before our reset (e.g. left over
	 * from u-boot's own use of this bus) -- confirmed idiom, see
	 * I2C_ISR_INIT in the real driver's header; we clear by snapshot
	 * instead of hardcoding that mask. */
	csr32w(c, Isr, csr32r(c, Isr));

	icr = Crscle | Crgcd;
	if(c->speed >= 400000)
		icr |= Crmodefast;		/* mode field (Crmodemask) starts at 0 above */
	/*
	 * ILCR/IWCR (clock load/wait counts) are deliberately left at
	 * their power-on-reset values: confirmed against the real
	 * driver, which never touches them either and relies on the
	 * POR defaults for standard/fast mode timing.
	 */
	csr32w(c, Icr, icr);

	/*
	 * Enable the unit as a separate, final write, matching the
	 * confirmed reference sequence (config bits land and settle
	 * before IUE goes high) rather than bundling it into the write
	 * above.
	 */
	csr32w(c, Icr, icr | Criue);
	delay(1);

	if(c->glitchnofix)
		csr32w(c, Irstcyc, csr32r(c, Irstcyc) | (1<<7));

	bus->probed = 0;
	return 0;
}

/*
 * i2clinkbuses -- does the actual work: registers one I2Cbus per
 * table entry.  Split out from i2ck3link() so the board-match gate
 * below reads the same way k3thermal.c's k3thermalinit()/thermalinit()
 * split does.
 */
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
		c->i2c.speed = c->speed;	/* I2Cbus has its own int speed; keep both in sync */
		c->i2c.ctlr = c;		/* back-pointer, used by k3i2cio()/k3i2cinit() */
		c->i2c.init = k3i2cinit;
		c->i2c.io = k3i2cio;

		addi2cbus(&c->i2c);
	}
}

/*
 * i2ck3link -- call this once at boot, before any driver tries to
 * look up a bus by name via i2cbus().
 *
 * HOOK MECHANISM (migrated): this is no longer called explicitly from
 * devarch.c's archinit(). It's picked up by the config's generic
 * "link:" mechanism instead -- the rv64 config file (and any other
 * $CONFLIST entry that wants this driver) has a bare
 *
 *	i2ck3
 *
 * line under its "link" section, alongside sdspacemit, ethersmte,
 * etherdwqe, etc.
 *
 * CONFIRMED (the hard way, via an actual build failure): the link:
 * entry name has to match the SOURCE FILE name (i2ck3.c), not just
 * the function name. An earlier version of this driver used the
 * entry "k3i2c" on the theory that mkbootrules only cared about
 * generating a call to "<entry>link()" (which this function's name
 * already matched with zero renaming). That was wrong: building with
 * that entry failed with
 *
 *	mk: no recipe to make 'k3i2c.j' in directory /sys/src/9/riscv64
 *
 * (".j" being $O for riscv64) -- mkbootrules itself generates a
 * dependency on "<entry>.$O", compiled from "<entry>.c" via mk's
 * ordinary implicit rule, entirely separately from anything in this
 * file's own mkfile OBJ line. Since the real source file is i2ck3.c,
 * the link: entry has to be "i2ck3", which in turn means the function
 * it calls has to be named i2ck3link() (hence the rename from the
 * earlier k3i2clink) -- both the file and the function must match the
 * entry name, not just one of them.
 *
 * None of the *other* link: entries (ethersmtelink, sdspacemitlink,
 * usbxhcibpif3link, ...) have their own extern anywhere in fns.h
 * either, so the generated dispatch code (living in the build's
 * $CONF.c, not in this file) is expected to emit its own extern for
 * each entry -- no fns.h entry needed here either.
 *
 * Gated on dtbsocmatch("spacemit,k3"); confirmed working on real
 * hardware.
 */
void
i2ck3link(void)
{
	if(!dtbsocmatch("spacemit,k3"))
		return;
	i2clinkbuses();
}
