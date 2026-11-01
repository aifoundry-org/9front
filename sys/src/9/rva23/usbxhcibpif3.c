/*
 * Driver for SpacemiT K1 USB controller on Banana Pi BPI-F3
 *
 * Based on:
 *   9front usbxhciimx.c  - general driver structure on imx8 arch (arm NXP processor)
 *   9front usbxhcilx2k.c - general driver structure on lx2k arch (arm NXP processor)
 */

#include    "u.h"
#include    "../port/lib.h"
#include    "mem.h"
#include    "dat.h"
#include    "fns.h"
#include    "io.h"
#include    "../port/error.h"
#include    "../port/usb.h"
#include    "../port/usbxhci.h"
#include    "k1.h"

enum {
	/* SoC K1 base addresses (per K1 memory map / DTB) */
	XhciPhys    = 0xC0A00000,    /* usb@c0a00000: DWC3 xHCI controller */
	XhciSize    = 0x100000,
	Usb2phyPhys = 0xC0A30000,    /* phy@c0a30000: standalone USB2 PHY block */
	Usb2phySize = 0x200,

	/*
	 * GPIO controller (gpio@d4019000). Two active-high enables:
	 *   GPIO 97  -> USB30_VBUS  (port VBUS 5V)
	 *   GPIO 123 -> USB30_HUB   (Vdd of the on-board VL817 USB3 hub that
	 *                            fans the DWC3 out to the 4 type-A ports)
	 *   GPIO 124 -> hub RESET    (active-low; hub stays in reset, and so
	 *                            never presents a connect, until driven high)
	 * All are in bank 3 (GPIOs 96..127).  Per the K1 GPIO map the bank-3
	 * register group is at controller+0x100, with Gsdr (set-as-output),
	 * Gpsr (set-output-high) and Gpcr (set-output-low) all write-1-per-bit
	 * and atomic, so we disturb no other GPIO in the bank.
	 */
	GpioPhys   = 0xD4019000,
	GpioSize   = 0x800,
	Gpiobank3  = 0x100/4,
	Gpsr       = 0x18/4,    /* set output high */
	Gpcr       = 0x24/4,    /* set output low */
	Gsdr       = 0x54/4,    /* set direction = output */
	Vbusgpio   = 97,        /* USB30_VBUS enable, bank 3 bit 1 */
	Hubgpio    = 123,       /* USB30_HUB enable,  bank 3 bit 27 */
	Hubrstgpio = 124,       /* VL817 reset (active low), bank 3 bit 28 */

	/* PMU registers for USB */
	Usbclkres      = 0x5C/4,     /* PMU_USB_CLK_RES_CTRL */
	Usbphyctrl0    = 0x110/4,    /* combo-phy mode select (phy_sel @ 0xd4282910) */
	Combophyrst    = 0x3CC/4,    /* APMU_PCIE_CLK_RES_CTRL_0 */
	Combophyrstbit = 1<<8,       /* RESET_COMBO_PHY: clear this bit to release */

	/*
	 * PMU_USB_CLK_RES_CTRL bit map.  All bits are active-high: set = clock
	 * enabled / reset released.
	 *   bit0  RESET_USB_AXI        bit1  usb_axi_clk
	 *   bit4  RESET_USBP1_AXI      bit5  usb_p1_aclk   (per-port AXI rst+clk)
	 *   bit8  usb30_clk            bit9  RESET_USB30_AHB
	 *   bit10 RESET_USB30_VCC      bit11 RESET_USB30_PHY
	 * The per-port AXI clock (bit5) and reset (bit4) are required: without
	 * them the USB2 port block has no clock and stays in reset, so it sees an
	 * analog connect (PORTSC.CCS) but can never do the digital port reset
	 * (PORTSC.PR has no effect and PED never sets).
	 */
	Usb30CoreMask = 0x0F00,    /* bits 8-11: usb30 clk + AHB/VCC/PHY resets */
	UsbAxiMask    = 0x0003,    /* bits 0-1: AXI clock + AXI reset release */
	UsbP1Mask     = 0x0030,    /* bits 4-5: per-port AXI reset + clock */
	UsbPmuEnable  = Usb30CoreMask | UsbAxiMask | UsbP1Mask,
	ComboPhyUsb3  = 1<<3,      /* Assign the Combo PHY to USB3 or PCIe (0) */

	IrqUsb3 = 125,    /* sys_int_ap interrupt of the USB3 controller */
};

/*
 * Drive the BPI-F3 USB power-enable GPIOs high: port VBUS and, crucially,
 * the on-board USB hub's Vdd.  Without hub power nothing downstream of the
 * DWC3 ever connects, so no device is enumerated on either root port.
 */
static void
usbgpiopower(void)
{
	u32int *gpio, pwr, rst;

	gpio = vmap(GpioPhys, GpioSize);
	if(gpio == nil){
		print("usbxhcirv64: vmap gpio failed, base=%ux\n", GpioPhys);
		return;
	}
	pwr = 1<<(Vbusgpio%32) | 1<<(Hubgpio%32);
	rst = 1<<(Hubrstgpio%32);

	/* configure VBUS, hub power and hub reset as outputs */
	gpio[Gpiobank3 + Gsdr] = pwr | rst;

	/* power on VBUS + hub while holding the hub in reset (reset is low) */
	gpio[Gpiobank3 + Gpcr] = rst;       /* assert reset (active low) */
	gpio[Gpiobank3 + Gpsr] = pwr;       /* VBUS + hub power high */
	delay(10);                          /* let the rails settle in reset */

	/* release the hub from reset and give it time to come up */
	gpio[Gpiobank3 + Gpsr] = rst;       /* deassert reset (drive high) */
	delay(50);

	vunmap(gpio, GpioSize);
}

/*
 * Bring up the USB3 combo PHY (spacemit,k1x-combphy @ 0xC0B10000) enough to
 * provide the DWC3's SuperSpeed PIPE clock.  This is not optional even for
 * USB2: the DWC3 HCRST resets every port block, and the SS block waits
 * forever for this clock, so without the combo PHY HCRST never completes and
 * the whole controller is wedged.
 *
 * Prerequisites done in reset(): USB3 combo-phy mode selected (ComboPhyUsb3)
 * and RESET_COMBO_PHY released (Combophyrst bit).  Here we load the USB3 PHY
 * register set and wait for its PLL to lock.  Register values are the K1
 * combo-phy USB3 programming constants.
 */
static void
usb3phyinit(void)
{
	u32int *phy;
	int i;
	enum {
		CombophyPhys = 0xC0B10000,
		CombophySize = 0x800,
		Reg1         = 0x68/4,    /* = 0 */
		Reg2         = 0x48/4,    /* = 0x603a2276 */
		Reg3         = 0x08/4,    /* = 0x97c; doubles as the PLL status reg */
		Reg4         = 0x18/4,    /* = 0 */
		PllRdy       = 1<<0,      /* PLL locked, read from Reg3 */
		Lfps         = 0x58/4,    /* LFPS threshold, bits [10:8] */
	};

	phy = vmap(CombophyPhys, CombophySize);
	if(phy == nil){
		print("usbxhcirv64: vmap combophy failed\n");
		return;
	}
	phy[Reg1] = 0x0;
	phy[Reg2] = 0x603a2276;
	phy[Reg3] = 0x97c;
	phy[Reg4] = 0x0;
	coherence();
	for(i = 0; i < 1000; i++){
		if(phy[Reg3] & PllRdy)
			break;
		microdelay(10);
	}
	phy[Lfps] = (phy[Lfps] & ~(0x7<<8)) | (0x3<<8);     /* lfps threshold = 3 */
	if((phy[Reg3] & PllRdy) == 0)
		print("usbxhcirv64: usb3 combophy PLL not ready (reg8=%ux)\n", phy[Reg3]);
	vunmap(phy, CombophySize);
}

/*
 * Bring up the K1's standalone USB2 PHY (register block at Usb2phyPhys,
 * its own MMIO region -- NOT a register inside the DWC3 wrapper).  The
 * DWC3's UTMI HS port stays electrically dead until this PHY's PLL is
 * configured for the 24 MHz reference clock, reported ready, and the
 * PLL/CDR/MAC resets are de-asserted; without it PORTSC.CCS never
 * asserts and no downstream device (or the on-board hub) is ever seen.
 *
 * Register/bit map per the K1 USB2 PHY block:
 *   RST_MODE_CTRL (0x04): bit0 PLL_RDY (ro), bit1 CLK_CDR_EN,
 *       bit2 CLK_PLL_EN, bit3 CLK_MAC_EN, bit5 MAC_RSTN, bit6 CDR_RSTN,
 *       bit7 PLL_RSTN, bit13 HS line-state tx-mode, bit14 FS line-state
 *       tx-mode (both default 1).
 *   TX_HOST_CTRL (0x10): bit2 auto-clear HS host-disconnect on reconnect.
 *   HSTXP_HW_CTRL (0x34): bit2 HSTXP_RSTN, bit3 CLK_HSTXP_EN, bit4
 *       HSTXP_MODE.
 *   PLL_DIV_CFG (0x98): fdiv_reg[12:0], freq_sel[14:13] (1 => 24 MHz ref),
 *       bit15 DIV_LOCAL_EN (0 => use fdiv_reg override).
 */
static void
usb2phyinit(u32int *phy)
{
	enum {
		RstModeCtrl = 0x04/4,
			PllRdy      = 1<<0,
			ClkCdrEn    = 1<<1,
			ClkPllEn    = 1<<2,
			ClkMacEn    = 1<<3,
			MacRstn     = 1<<5,
			CdrRstn     = 1<<6,
			PllRstn     = 1<<7,
			HsLineTxMode= 1<<13,
			FsLineTxMode= 1<<14,
		TxHostCtrl  = 0x10/4,
			HstDiscAutoClr = 1<<2,
		HstxpHwCtrl = 0x34/4,
			HstxpRstn   = 1<<2,
			ClkHstxpEn  = 1<<3,
			HstxpMode   = 1<<4,
		PllDivCfg   = 0x98/4,
			/*
			 * fdiv_reg[12:0]=0x1ec4, freq_sel 24 MHz (1) in [14:13], and
			 * DIV_LOCAL_EN (bit15) SET so the PLL uses its internal
			 * divider for the selected 24 MHz reference.  With bit15
			 * clear the raw fdiv_reg override is used instead and the
			 * VCO never locks (PLL_RDY stays 0, rstmode reads 0x60e8).
			 */
			PllDiv24m   = 0x1ec4 | 0x01<<13 | 1<<15,
	};
	int i;

	/* let any pending controller reset settle before touching the PHY */
	microdelay(200);

	/* program the PLL divider for the 24 MHz reference clock */
	phy[PllDivCfg] = PllDiv24m;

	/* wait for the PLL to lock (datasheet budget ~50 ms) */
	for(i = 0; i < 5000; i++){
		if(phy[RstModeCtrl] & PllRdy)
			break;
		microdelay(10);
	}
	if((phy[RstModeCtrl] & PllRdy) == 0)
		print("usbxhcirv64: USB2 PHY PLL not ready (rstmode=%ux)\n",
			phy[RstModeCtrl]);

	/* enable PLL/CDR/MAC clocks and release their resets */
	phy[RstModeCtrl] = FsLineTxMode | HsLineTxMode |
		ClkPllEn | ClkCdrEn | ClkMacEn |
		PllRstn | CdrRstn | MacRstn;

	/* release the HS-TX clock and enable it */
	phy[HstxpHwCtrl] = HstxpRstn | ClkHstxpEn | HstxpMode;

	/* auto-clear HS host-disconnect so a re-plugged device re-enumerates */
	phy[TxHostCtrl] |= HstDiscAutoClr;
}

static void
coreinit(u32int *reg)
{
	enum {
		Gctl         = 0xC110/4,
		/* Additional global registers */
		Guctl1       = 0xC11C/4,    /* Global User Control Register 1 */
		Gusb2phycfg  = 0xC200/4,    /* USB2 (UTMI/ULPI) PHY config, port 0 */
		Gusb3pipectl = 0xC2C0/4,    /* USB3 PIPE control, port 0 */
		Gfladj       = 0xC630/4,    /* Global Frame Length Adjustment */

		/* Guctl1 host quirk bits for the K1 DWC_usb3 core */
		TxIpgapLinecheckDis = 1<<28,    /* disable TX inter-packet-gap line check */
		ParkmodeDisableSs   = 1<<17,    /* disable SuperSpeed host park mode */
		PwrdnscaleShift     = 19,

		PwrdnscaleMask = 0x3FFF << PwrdnscaleShift,
		/* Literal for 125MHz */
		Pwrdnscale125m = 0x2c,

		PrtcapdirShift = 12,
		PrtcapdirMask  = 3 << PrtcapdirShift,
		Disscramble    = 1<<3,
		Dsblclkgtng    = 1<<0,

		Gfladj30MhzMask     = 0x3F,
		Gfladj30MhzSdbndSel = 1<<7,

		/* Gusb2phycfg fields (this K1 wires a UTMI+ 8-bit HS PHY) */
		U2Freeclkexists = 1<<30,    /* free clock does not exist here */
		U2Enblslpm      = 1<<8,     /* let controller suspend the PHY */
		U2Susphy        = 1<<6,     /* suspend USB2 PHY when idle */
		U2UlpiUtmi      = 1<<4,     /* 0 = UTMI+, 1 = ULPI */
		U2Phyif         = 1<<3,     /* 0 = 8-bit, 1 = 16-bit UTMI */
		U2UsbtrdtimMask = 0xF<<10,
		U2Usbtrdtim8bit = 9<<10,    /* turnaround time for 8-bit UTMI */

		/* Gusb3pipectl */
		U3Physoftrst = 1u<<31,    /* hold USB3 PIPE PHY iface in reset */
		U3Susphy     = 1<<17,     /* suspend USB3 PHY when idle */
	};
	u32int v;

	/* 1. Global Control configuration (Gctl) */
	v = reg[Gctl];
	v &= ~(PwrdnscaleMask | Disscramble | Dsblclkgtng | PrtcapdirMask);
	v |= Pwrdnscale125m<<PwrdnscaleShift | 1<<PrtcapdirShift;
	reg[Gctl] = v;

	/*
	 * 2. Host-mode global user-control quirks for the K1's DWC_usb3 core.
	 * Leave GUCTL Host-IN-Auto-Retry (bit 14) clear; the K1 host
	 * configuration does not use it (when set, a NAKed bulk IN retries
	 * internally without bound).  Instead set two Guctl1 quirks - the bits
	 * exist on this >=2.50a core - matching the K1's documented host setup:
	 * bit 28 TxIpgapLinecheckDis disables the transmit inter-packet-gap
	 * line-state check, and bit 17 ParkmodeDisableSs disables SuperSpeed
	 * host park mode.
	 */
	reg[Guctl1] |= TxIpgapLinecheckDis | ParkmodeDisableSs;

	/* 3. Frame Length Adjustment (Gfladj) */
	/* A typical adjustment value of 0x20 is used for synchronization,
	   matching the fladj_30mhz_reg field seen in the K1 manual */
	reg[Gfladj] = (reg[Gfladj] & ~Gfladj30MhzMask) | 0x20 | Gfladj30MhzSdbndSel;

	/*
	 * 4. USB2 PHY interface (Gusb2phycfg).  The reset default can leave
	 * the port in ULPI mode and/or with PHY auto-suspend enabled, which
	 * keeps the UTMI HS receiver powered down so no downstream connect is
	 * ever seen (PORTSC stuck at CCS=0, PLS=RxDetect).  Select UTMI+ 8-bit
	 * and disable the suspend/sleep/free-clock behaviours the K1 doesn't
	 * support, matching the controller's required configuration.
	 */
	v = reg[Gusb2phycfg];
	v &= ~(U2UlpiUtmi | U2Phyif | U2UsbtrdtimMask |
	       U2Susphy | U2Enblslpm | U2Freeclkexists);
	v |= U2Usbtrdtim8bit;
	reg[Gusb2phycfg] = v;

	/*
	 * 5. Keep the USB3 PHY active (not suspended, not held in soft reset).
	 * The combo PHY is brought up in usb3phyinit() so it provides the
	 * SuperSpeed PIPE clock; that clock is what lets the controller's HCRST
	 * complete.  If the SS port block is left without a clock, HCRST never
	 * finishes (USBCMD.HCRST stays set) and the whole controller wedges --
	 * even PORTSC.PR on the USB2 port becomes inert.
	 */
	reg[Gusb3pipectl] &= ~(U3Susphy | U3Physoftrst);
}

static int
reset(Hci *hp)
{
	static Xhci *ctlr;
	u32int *reg;
	u32int *phy;
	void *mmio;

	if(ctlr != nil)
		return -1;

	/*
	 * Unified kernel: this driver owns the K1's DWC3 USB3 controller.
	 * Gate on the DTB root compatible so it stays quiet on any other
	 * SoC (e.g. the K3, whose usbxhcirv64k3 handles USB instead).  The
	 * K1's PMU register block and controller MMIO are not present, or
	 * mean something else, on other SoCs -- poking them would wedge the
	 * bus, so bail before the first vmap/write.
	 */
	if(!dtbsocmatch("spacemit,k1"))
		return -1;

	/* 0. Power the port VBUS and the on-board USB hub via GPIO */
	usbgpiopower();

	/* 1. PMU: enable clocks and release the controller's resets */
	reg = vmap(ApmuPhys, ApmuSize);
	if(reg != nil){
		reg[Usbclkres] |= UsbPmuEnable;       /* usb30 clocks + resets released */
		reg[Usbphyctrl0] |= ComboPhyUsb3;     /* select USB3 combo-phy mode */
		reg[Combophyrst] &= ~Combophyrstbit;    /* release RESET_COMBO_PHY */
		coherence();
		vunmap(reg, ApmuSize);
	} else {
		print("usbxhcirv64: vmap pmu failed, base=%ux len=%ux\n", ApmuPhys, ApmuSize);
		return -1;
	}

	/*
	 * 1b. Bring up the USB3 combo PHY so the DWC3 SuperSpeed port block has a
	 * PIPE clock; otherwise the controller's HCRST never completes and even
	 * USB2 is dead.
	 */
	usb3phyinit();

	/* 2. Set up the Xhci structure and the DWC3 core */
	mmio = vmap(XhciPhys, XhciSize);
	if(mmio == nil) {
		print("usbxhcirv64: vmap xhci failed\n");
		return -1;
	}
	ctlr = xhcialloc(mmio, XhciPhys, XhciSize);
	if(ctlr == nil){
		print("usbxhcirv64: xhcialloc failed\n");
		vunmap(mmio, XhciSize);
		return -1;
	}

	/* 3. Configure the DWC3 core */
	coreinit(ctlr->mmio);

	/* 4. Bring up the standalone USB2 PHY (its own MMIO block) */
	phy = vmap(Usb2phyPhys, Usb2phySize);
	if(phy != nil){
		usb2phyinit(phy);
		vunmap(phy, Usb2phySize);
	} else
		print("usbxhcirv64: vmap usb2phy failed, base=%ux len=%ux\n",
			Usb2phyPhys, Usb2phySize);

	/* 5. Link with the USB system */
	hp->irq = IrqUsb3;
	hp->tbdf = BUSUNKNOWN;
	xhcilinkage(hp, ctlr);

	return 0;
}

void
usbxhcibpif3link(void)
{
	addhcitype("xhci", reset);
}
