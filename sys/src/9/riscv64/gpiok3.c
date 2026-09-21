/*
 * gpiok3.c -- GPIO controller driver for the SpacemiT K3 SoC
 * ("spacemit,k3-gpio" IP), for 9front.
 *
 * STATUS: written from documented/cross-checked sources, PARTIALLY
 * validated against real hardware (unlike i2ck3.c, which carries its
 * own full "tested on real k3-pico-itx hardware" note). On real
 * k3-pico-itx hardware, via devgpiopicoitx.c:
 *
 *   - gpioin() confirmed, one pin per bank (see the FIELD-CONFIRMED
 *     note on BankStride below for what that did/didn't establish).
 *   - gpioout() confirmed at the REGISTER level only: writing 0 then
 *     1 then 0 to WLAN_EN_3V3 (pin 35, bank 1) and reading each back
 *     via gpioin() round-tripped correctly every time. That's Pdr/
 *     Psr/Pcr/Plr doing something self-consistent -- it is NOT yet
 *     confirmed that the physical pad actually changes voltage: on
 *     this IP, as on many others, Plr on a pin already configured as
 *     output may just reflect the output flip-flop you wrote,
 *     independent of whatever the pad itself is doing (e.g. if
 *     pinctrl never actually muxed this ball to the GPIO alternate
 *     function). PENDING: multimeter/scope on the WLAN_EN_3V3 net
 *     (or the module's WL_EN pin past level-shifter Q25, schematic
 *     p.47) while toggling, or any other independent physical
 *     confirmation (backlight-enable driving a real eDP panel,
 *     PCIe4 enumeration tracking the pin, etc.).
 *
 * gpiointrenable()'s interrupt path is still completely UNVERIFIED.
 * Treat every register write below as register-level-only-confirmed
 * at best until an independent physical check closes the loop above,
 * and treat interrupts as unconfirmed until someone reports
 * gpiointrenable() firing a real edge interrupt.
 *
 * SOURCES, and what each one contributed:
 *
 *   1. SpacemiT K3 User Manual, chapter 16.8 "GPIO" (pages 589-596,
 *      "16. System Peripherals"), the user-supplied PDF excerpt.
 *      This is the primary source for the per-bank register offsets,
 *      bit layout, and reset behaviour used below (Plr/Pdr/Psr/Pcr/
 *      Rer/Fer/Edr and the two interrupt masks). It documents 14
 *      registers per bank at offsets 0x00 through 0x38, all
 *      explicitly labelled with an offset EXCEPT the bit-wise-clear-
 *      rising-edge register (which the running numbering implies
 *      sits at 0x28, between GPIO_SRER at 0x24 and GPIO_SFER at
 *      0x2C) -- see the Crer entry below, marked UNCONFIRMED, and
 *      not used by this driver for exactly that reason.
 *
 *   2. k3-pico-itx.dtb and com260.dtb (both user-supplied device-tree
 *      dumps). CONFIRMED byte-for-byte identical between the two
 *      boards (same node, same "reg", same "interrupts", same
 *      phandles) at:
 *
 *		gpio@d4019000 {
 *			compatible = "spacemit,k3-gpio";
 *			reg = <0x0 0xd4019000 0x0 0x100>;
 *			clocks = <&apbc 0x14 &apbc 0x15>;
 *			clock-names = "core", "bus";
 *			interrupts = <0x3a 0x4>;
 *			interrupt-parent = <&aplic>;
 *			...
 *		};
 *
 *      This gives the physical base (0xD4019000), the total MMIO
 *      window (0x100 -- confirming 4 banks of 0x40 bytes each, see
 *      the BankStride note below), and the shared interrupt number
 *      (0x3a = 58, on the riscv,aplic interrupt-controller@e0804000,
 *      #interrupt-cells=2, second cell 0x4 = level-triggered line --
 *      matched to how every other driver in this tree calls
 *      intrenable() with a bare irq number and lets intr.c/aplic.c
 *      pick the controller, so no separate trigger-type call is made
 *      here either).
 *
 *      Neither dtb's "gpio-ranges" property encodes register byte
 *      offsets (it's pin-number space: <&pinctrl 0 0 32>, <&pinctrl
 *      32 32 32>, ... -- just the 4x32 pin partition), so it wasn't
 *      usable for the register map, only for confirming 4 banks of
 *      32 pins = 128 pins total, matching the manual's "up to 128
 *      GPIOs" text.
 *
 *   2b. K3 User Manual, chapter 6.1 "Main CPU Domain Address Mapping"
 *      (page 57-59). Independently confirms the GPIO base seen in
 *      the dtb ("GPIO  0xD401_9000  0x0000_0800") and, more usefully,
 *      confirms "GPIO Edge" as its OWN, separate memory-map entry at
 *      0xD401_9800/0x800 -- i.e. not a sub-range of the GPIO block,
 *      matching the dtb's separate "syscon-gpio-edge" node and the
 *      manual's own 16.8 text about the Pin Control Unit's
 *      independent wakeup-edge logic (three independent sources now
 *      agreeing it's out of scope here, see the NOT-implemented list
 *      below).
 *
 *      This table also SHARPENS, rather than resolves, the
 *      GPIO3_BASE question raised below: chapter 6.1 gives GPIO a
 *      full 0x800 of decoded address space (room enough for a bank
 *      at +0x100), but the dtb's own "reg" size for this exact
 *      hardware is 0x100, i.e. the mapped/valid window ends AT
 *      0xD4019100 (exclusive) -- so if a fourth bank genuinely lived
 *      at 0xD4019100 as chapter 16.8.3.1's address table literally
 *      states, the dtb's reg size would have to be at least 0x140,
 *      and it isn't. That makes the stride-derived 0x90C0 (used
 *      below) the only offset actually reachable within the window
 *      the vendor's own device tree says to map -- not a coin flip
 *      between two equally-plausible numbers.
 *
 *   3. Another implementation of this block for the same SoC family,
 *      seen only from the outside.  It corroborates two things found
 *      in isolation from the manual: (a) that K3 really does use a
 *      DIFFERENT register layout/bank-offset than the earlier K1 IP
 *      (the K1 offsets -- GPLR 0x00, GPDR 0x0c, GPSR 0x18, ... GAPMASK
 *      0x9c, all packed into one 0xa8-byte block covering all 4 banks
 *      together -- do NOT match this file, on purpose: they're the
 *      wrong SoC), and (b) that pin direction belongs on PDR as a
 *      plain read-modify-write and not on the SDR/CDR pulse
 *      registers, which is a mistake that block's first driver made
 *      and had to correct.  This driver does plain RMW on Pdr/Rer/Fer
 *      only -- see the "not used by this driver" notes on
 *      Sdr/Cdr/Srer/Crer/Sfer/Cfer below.  No K3-specific offset
 *      table was obtainable from there, so every offset below comes
 *      from source (1); this is corroboration that a K3-specific
 *      layout is expected to exist, not the origin of the numbers.
 *
 *   4. K3 User Manual, chapter 17.4.2.3 "APBCLOCK" (page 624),
 *      "CLOCK/RESET CONTROL REGISTER FOR GPIO". Gives the real
 *      clock-gate register for this block (APBC_GPIO_CLK_RST, apbc
 *      offset 0x8) -- see the now-resolved former "NOT implemented"
 *      item below for the details and the cross-check against
 *      k3thermal.c/i2ck3.c's existing apbc offsets.
 *
 *   5. K3 User Manual, chapter 7.1 "Main CPU Interrupts" (page 67):
 *      "aplic_int[58]  m1_gpio_int_ap". Independently confirms the
 *      dtb-derived GpioIrq=0x3a(58) is in fact this GPIO block's own
 *      (AP, non-secure) interrupt line, not a coincidence of numbering
 *      -- two unrelated sources now agree. The same table also
 *      confirms, in passing, two exclusions already made above on
 *      other grounds: aplic_int[59] "m1_gpio_int_ap_sec" is
 *      SEC_GPIO's own line (the secure-domain block this driver
 *      doesn't touch) and aplic_int[60] "m1_gpio_edge_det_wakeup" is
 *      the Pin Control Unit wakeup-edge line (the "GPIO Edge" block
 *      this driver also doesn't touch) -- neither is GpioIrq, and
 *      neither was ever wired into this file.
 *
 * NOT implemented, on purpose, pending confirmation (same policy as
 * i2ck3.c's ri2c0/ri2c1 stub -- ownership/register-offset questions
 * left open rather than guessed at):
 *
 *   - Clock gating. RESOLVED: confirmed against the K3 User Manual,
 *     chapter 17.4.2.3 "APBCLOCK" (page 624), "CLOCK/RESET CONTROL
 *     REGISTER FOR GPIO", APBC_GPIO_CLK_RST at apbc offset 0x8: bit 0
 *     APBCLK (bus clock enable), bit 1 FNCLK (functional clock
 *     enable), bit 2 RST (0 = release reset, 1 = reset), bits 6:4
 *     FNCLKSEL (only defined value is 0x0 = 24 MHz). Same table also
 *     lists APBC_TSEN_CLK_RST at offset 0x6C and APBC_TWSIn_CLK_RST
 *     with TWSI0 at offset 0x2C -- both offsets, and both the
 *     bus/func/reset bit positions, EXACTLY matching what
 *     k3thermal.c's ApbcTsen and i2ck3.c's ApbcTwsi0 already use in
 *     this tree, independently confirming this is the same physical
 *     apbc block (0xD4015000) and the same bit convention, not a new
 *     guess. gpiok3init() now gates this exactly the way
 *     thermalinit() gates ApbcTsen.
 *
 *   - SEC_GPIO0-3 (0xF061_9000 series, per the manual's base-address
 *     table) and AUD_GPIO (0xC088_9400) -- these are separate
 *     physical blocks in the secure/audio domain, not sub-ranges of
 *     the 0xD4019000 block this file maps. Whether the AP is even
 *     meant to be a second owner of the secure-domain instance is
 *     exactly the kind of open ownership question i2ck3.c declined
 *     to guess at for ri2c0/ri2c1; same call here.
 *
 *   - Pin-mux / pull-up / pull-down / drive-strength. On K3 these
 *     live in the SEPARATE "spacemit,k3-pinctrl" block at 0xd401e000
 *     (confirmed in both dtbs, right after the gpio node), not in
 *     the GPIO block itself -- unlike the BCM gpio.c example, which
 *     has gpiopull()/gpiopullup()/gpiopulldown() because the BCM
 *     GPIO block owns pull config directly (PUD/PUPPDN* registers).
 *     There is no k3 equivalent of those three calls here: a
 *     bitbang consumer modelled on i2cgpio.c (which calls
 *     gpiopullup()) would need a pinctrl driver written first. This
 *     file only drives pins that pinctrl (or firmware) has already
 *     muxed to their GPIO alternate function.
 *
 *   - The Pin Control Unit's own separate wakeup-edge-detect logic,
 *     at "syscon-gpio-edge"/0xd4019800 (referenced by both the gpio
 *     node's "spacemit,gpio-edge" and the pinctrl node's
 *     "syscon-gpio-edge" phandle, by name in the manual's GPIO
 *     RISING-EDGE section: "independent of, and in addition to, the
 *     Pin Control Unit edge-detect logic ... mainly used as a wakeup
 *     event", AND as its own separate "GPIO Edge" row in chapter
 *     6.1's address map -- three independent sources, all agreeing
 *     it's a distinct block). That's a suspend/wake path belonging
 *     to whatever eventually does suspend-to-RAM on this port, not
 *     to a plain GPIO driver -- deliberately left alone.
 *
 *   - CPMASK_REG (offset 0x38, confirmed in the manual, same
 *     bit-per-pin layout as APMASK_REG at 0x34). The manual documents
 *     two independent interrupt masks per bank without naming which
 *     consumer owns which; going by the K3 domain split seen
 *     throughout this tree (i2ck3.c's rdomain/RCPU distinction,
 *     k1thermal.c's separate RCPU block) "AP" vs "CP" almost
 *     certainly means "this core" vs "the RCPU companion core", so
 *     this driver only ever sets bits in APMASK_REG and never touches
 *     CPMASK_REG -- masking a pin's interrupt to the RCPU would be
 *     RCPU's call to make, not this driver's.
 *
 * Pin numbering: pin = bank*32 + offset, bank 0..3, offset 0..31 --
 * i.e. exactly fns.h's existing GPIO_PIN(n, m) macro. This isn't a
 * new convention invented for this file: GPIO_PIN(), and the
 * gpioout()/gpioin()/gpiointrenable()/gpiointrdisable() prototypes
 * this file implements, were ALREADY declared in fns.h before this
 * file existed (matching the imx8 gpio.c example's API, not the BCM
 * example's gpiosel()/gpiopull()/gpioselevent()/gpiomeminit() API --
 * so that choice of API shape was made upstream of this file, not by
 * it). The GpioLow/GpioHigh/GpioRising/GpioFalling/GpioEdge mode
 * constants gpiointrenable() takes were likewise already defined in
 * dat.h. This driver only supports the two edge modes this IP
 * actually has, GpioRising/GpioFalling/GpioEdge(=both); GpioLow/
 * GpioHigh (level-triggered) have no equivalent in the K3 GPIO block
 * -- gpiointrenable() silently does nothing for those, same as
 * gpioout()/gpioin() silently doing nothing before gpiok3init() has
 * run or on a non-K3 board (mirrors gpioin() returning -1 / gpioout()
 * returning early on ctlr==nil in the imx8 example).
 */
#include "u.h"
#include "../port/lib.h"
#include "../port/error.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"

enum {
	Nbank		= 4,		/* confirmed: gpio-ranges 4x32, manual "up to 128 GPIOs" */
	Nport		= 32,		/* pins per bank */
	Npin		= Nbank*Nport,	/* 128 */

	GpioPhys	= 0xD4019000ull,	/* confirmed: both dtbs, gpio@d4019000 */
	GpioSize	= 0x100,		/* confirmed: both dtbs' reg size; == Nbank*BankStride */
	BankStride	= 0x40,			/* derived: GpioSize/Nbank; the manual's 14
						 * registers (0x00-0x38) fit inside one 0x40
						 * window with 4 bytes to spare. NOTE: the
						 * manual's own base-address table lists
						 * GPIO3_BASE as 0xD401_9100, which is NOT
						 * 0xD4019000+3*0x40 (0xD40190C0) -- that
						 * table entry looks like a documentation
						 * typo (0xD401_9100 breaks the otherwise
						 * clean 0x40 stride the other three
						 * entries, and the dtb's own "reg" size
						 * of 0x100 only makes sense as 4*0x40).
						 * This driver trusts the dtb-confirmed
						 * 0x100 total window and the clean stride
						 * over that one table cell.
						 *
						 * FIELD-CONFIRMED (real k3-pico-itx
						 * hardware, via devgpiopicoitx.c, one
						 * pin per bank): gpioin() on
						 * SOC_eDP0_HPD_3V3 (bank 0, pin 23),
						 * EC_CPU_ALLPG (bank 1, pin 42),
						 * CC_INT_N (bank 2, pin 85), and
						 * SOC_eDP0_PWR-EN_3V3 (bank 3, pin
						 * 101 -> gpiobase+3*0x40) all returned
						 * plausible, stable values -- no bus
						 * fault, no trap, nothing that would
						 * indicate bank 3 sitting outside the
						 * mapped 0x100 window. That's exactly
						 * what this driver predicts and the
						 * "0xD401_9100 is a typo" call above
						 * predicts; it's the opposite of what
						 * a fourth bank genuinely living at
						 * +0x100 (i.e. outside this file's
						 * vmap(GpioPhys, GpioSize) window)
						 * would produce. Doesn't yet confirm
						 * gpioout()/interrupts on bank 3, just
						 * that the offset math and the mapped
						 * window size are right. */

	GpioIrq		= 0x3a,			/* confirmed twice: both dtbs'
						 * interrupts=<0x3a 0x4>, AND K3 UM
						 * 7.1 "aplic_int[58]  m1_gpio_int_ap" */

	/* per-bank register offsets -- confirmed against K3 TRM 16.8.3
	 * except where noted. */
	Plr		= 0x00,	/* pin level			RO	*/
	Pdr		= 0x04,	/* pin direction		RW	0=in 1=out */
	Psr		= 0x08,	/* output set			WO	*/
	Pcr		= 0x0C,	/* output clear			WO	*/
	Rer		= 0x10,	/* rising-edge detect enable	RW	*/
	Fer		= 0x14,	/* falling-edge detect enable	RW	*/
	Edr		= 0x18,	/* edge-detect status		RWC (write 1 to clear) */
	/*
	 * Sdr/Cdr/Srer/Crer/Sfer/Cfer (0x1C, 0x20, 0x24, 0x28, 0x2C,
	 * 0x30): bit-wise set/clear pulse registers for Pdr/Rer/Fer.
	 * Confirmed present in the manual (except Crer at 0x28, which
	 * is UNCONFIRMED -- see the big comment above) but NOT USED by
	 * this driver: Pdr/Rer/Fer are documented plain R/W registers,
	 * so a direct read-modify-write is both simpler and avoids the
	 * direction-control mistake noted as source 3 above.
	 */
	Apmask		= 0x34,	/* AP-domain edge-irq mask, 1=unmasked	RW */
	Cpmask		= 0x38,	/* CP/RCPU-domain edge-irq mask -- not touched, see header */

	/*
	 * apbc clock/reset gate for this block -- confirmed against K3
	 * User Manual 17.4.2.3 "APBCLOCK", APBC_GPIO_CLK_RST. Same
	 * physical block (0xD4015000) and identical bus/func/reset bit
	 * convention as k3thermal.c's ApbcTsen (offset 0x6C) and
	 * i2ck3.c's ApbcTwsi0 (offset 0x2C) -- both independently
	 * present in the very same manual table, confirming this isn't
	 * a one-off guess. Gated the same way thermalinit() gates Tsen.
	 */
	ApbcPhys	= 0xD4015000ull,
	ApbcSize	= 0x1000,
	ApbcGpio	= 0x8,		/* APBC_GPIO_CLK_RST */
	ApbcBusEn	= 1<<0,		/* APBCLK -- APB bus clock enable */
	ApbcFuncEn	= 1<<1,		/* FNCLK -- functional clock (24 MHz) enable */
	ApbcReset	= 1<<2,		/* RST -- 0 = release reset, 1 = reset */
};

typedef struct Ivec Ivec;
struct Ivec
{
	void	(*f)(uint pin, void *a);
	void	*a;
};

static uintptr gpiobase;	/* mapped VA of the whole 0x100 window, 0 if unavailable */
static int gpioready;
static Lock gpiolk;
static Ivec vec[Npin];

static uintptr
bankbase(uint pin)
{
	return gpiobase + (pin/Nport)*BankStride;
}

static ulong
rd(uint pin, int reg)
{
	return *(ulong*)(bankbase(pin) + reg);
}

static void
wr(uint pin, int reg, ulong v)
{
	*(ulong*)(bankbase(pin) + reg) = v;
	coherence();
}

/*
 * Shared ISR: one irq (GpioIrq) covers all 4 banks -- both dtbs give
 * the whole gpio@d4019000 node a single "interrupts" cell, not one
 * per bank. Walk every bank's Edr (write-1-to-clear, per the manual),
 * dispatch any pin with a registered handler, and clear the status
 * bits we just read.
 */
static void
gpiointerrupt(Ureg*, void*)
{
	uint bank, pin;
	ulong edr;

	ilock(&gpiolk);
	for(bank = 0; bank < Nbank; bank++){
		edr = *(ulong*)(gpiobase + bank*BankStride + Edr);
		if(edr == 0)
			continue;
		*(ulong*)(gpiobase + bank*BankStride + Edr) = edr;	/* W1C */
		coherence();
		for(pin = bank*Nport; pin < (bank+1)*Nport; pin++)
			if((edr & (1<<(pin%Nport))) && vec[pin].f != nil)
				(*vec[pin].f)(pin, vec[pin].a);
	}
	iunlock(&gpiolk);
}

/*
 * gpioout -- fns.h/imx8-example API: drive pin high or low, flipping
 * it to an output first if it wasn't already one (mirrors the imx8
 * gpio.c example's enable()+gpioout() pattern: direction follows use,
 * callers don't need a separate "set direction" call).
 *
 * FIELD-CONFIRMED (real k3-pico-itx hardware, via devgpiopicoitx.c):
 * gpioout(WLAN_EN_3V3, 0), then 1, then 0 -- Pdr/Psr/Pcr writes here
 * and the subsequent gpioin() reads round-tripped correctly every
 * time. REGISTER-LEVEL ONLY: this confirms Pdr/Psr/Pcr/Plr are
 * self-consistent, not that the physical pad's voltage actually
 * changed (Plr on a pin already switched to output may just echo the
 * output flip-flop). Independent physical confirmation (multimeter/
 * scope on the net, or a visible downstream effect) is still
 * pending -- see the STATUS note at the top of this file.
 */
void
gpioout(uint pin, int set)
{
	ulong bit, pdr;

	if(!gpioready || pin >= Npin)
		return;
	bit = 1<<(pin%Nport);
	pdr = rd(pin, Pdr);
	if((pdr & bit) == 0)
		wr(pin, Pdr, pdr | bit);	/* 0->1: configure as output */
	wr(pin, set? Psr: Pcr, bit);
}

/*
 * gpioin -- read a pin, flipping it to an input first if it was
 * configured as an output (same direction-follows-use pattern).
 */
int
gpioin(uint pin)
{
	ulong bit, pdr;

	if(!gpioready || pin >= Npin)
		return -1;
	bit = 1<<(pin%Nport);
	pdr = rd(pin, Pdr);
	if(pdr & bit)
		wr(pin, Pdr, pdr & ~bit);	/* 1->0: configure as input */
	return (rd(pin, Plr) & bit) != 0;
}

/*
 * gpiointrenable -- mode is one of dat.h's GpioRising/GpioFalling/
 * GpioEdge (either edge); GpioLow/GpioHigh (level) are not
 * implementable on this IP (no level-detect registers exist in the
 * manual's register set) and are silently ignored -- callers wanting
 * level-triggered behaviour on K3 need a different approach (e.g.
 * poll gpioin(), there's no hardware level IRQ here to give them).
 */
void
gpiointrenable(uint pin, int mode, void (*f)(uint pin, void *a), void *a)
{
	ulong bit, rer, fer;

	if(!gpioready || pin >= Npin)
		return;
	bit = 1<<(pin%Nport);

	ilock(&gpiolk);

	/* interrupts only make sense on inputs */
	wr(pin, Pdr, rd(pin, Pdr) & ~bit);

	rer = rd(pin, Rer);
	fer = rd(pin, Fer);
	switch(mode){
	case GpioRising:
		rer |= bit;
		fer &= ~bit;
		break;
	case GpioFalling:
		fer |= bit;
		rer &= ~bit;
		break;
	case GpioEdge:		/* either edge */
		rer |= bit;
		fer |= bit;
		break;
	default:		/* GpioLow/GpioHigh: no level-IRQ on this IP */
		iunlock(&gpiolk);
		return;
	}
	wr(pin, Rer, rer);
	wr(pin, Fer, fer);

	wr(pin, Edr, bit);				/* clear any stale status, W1C */
	wr(pin, Apmask, rd(pin, Apmask) | bit);	/* unmask to the AP */

	vec[pin].f = f;
	vec[pin].a = a;
	iunlock(&gpiolk);
}

void
gpiointrdisable(uint pin)
{
	ulong bit;

	if(!gpioready || pin >= Npin)
		return;
	bit = 1<<(pin%Nport);

	ilock(&gpiolk);
	wr(pin, Apmask, rd(pin, Apmask) & ~bit);
	wr(pin, Rer, rd(pin, Rer) & ~bit);
	wr(pin, Fer, rd(pin, Fer) & ~bit);
	vec[pin].f = nil;
	vec[pin].a = nil;
	iunlock(&gpiolk);
}

/*
 * gpiok3init -- call once at boot (see devarch.c's archinit(), same
 * place k1thermalinit()/k3thermalinit()/k3ecfaninit() are called
 * from). Gated on dtbsocmatch("spacemit,k3"), same string k3thermal.c
 * and i2ck3.c gate on -- no point mapping and poking these registers
 * if we're not actually running on this SoC.
 */
void
gpiok3init(void)
{
	uchar *apbc;
	ulong v;

	if(!dtbsocmatch("spacemit,k3"))
		return;

	gpiobase = (uintptr)vmap(GpioPhys, GpioSize);
	if(gpiobase == 0)
		return;

	/*
	 * Ungate the block's APB bus clock and 24 MHz functional clock,
	 * and release it from reset -- confirmed register/bits, see the
	 * ApbcGpio enum comment above. Mirrors k3thermal.c's
	 * thermalinit() exactly: map apbc just long enough to do the
	 * read-modify-write, then unmap it again (apbc is a shared
	 * block; nothing here keeps it mapped between drivers).
	 */
	apbc = vmap(ApbcPhys, ApbcSize);
	if(apbc != nil){
		v = *(ulong*)(apbc + ApbcGpio);
		*(ulong*)(apbc + ApbcGpio) = (v & ~ApbcReset) | ApbcBusEn | ApbcFuncEn;
		coherence();
		vunmap(apbc, ApbcSize);
	}

	gpioready = 1;

	intrenable(GpioIrq, gpiointerrupt, nil, BUSUNKNOWN, "gpio");
}
