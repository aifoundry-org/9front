/*
 * Driver for the SpacemiT K1 USB controller 
 * for 9front / Plan 9 from Bell Labs
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

enum {
    /* SoC K1 base addresses (per K1 memory map / DTB) */
    Xhcibase    = 0xC0A00000,    /* usb@c0a00000: DWC3 xHCI controller */
    Xhcilen     = 0x100000,
    Usb2phybase = 0xC0A30000,    /* phy@c0a30000: standalone USB2 PHY block */
    Usb2phylen  = 0x200,
    Pmubase     = 0xD4282800,    /* Power Management Unit (AP PMU) */
    Pmulen      = 0x1000,

    /*
     * GPIO controller (gpio@d4019000).  On the BPI-F3 two active-high
     * enables must be driven to make USB usable, and nothing else on the
     * board drives them under Plan 9:
     *   GPIO 97  -> USB30_VBUS  (port VBUS 5V)
     *   GPIO 123 -> USB30_HUB   (Vdd of the on-board VL817 USB3 hub that
     *                            fans the DWC3 out to the 4 type-A ports)
     *   GPIO 124 -> hub RESET    (active-low; hub stays in reset, and so
     *                            never presents a connect, until driven high)
     * All live in bank 3 (GPIOs 96..127).  Per the K1 GPIO map the bank-3
     * register group is at controller+0x100, with GSDR (set-as-output),
     * GPSR (set-output-high) and GPCR (set-output-low) all write-1-per-bit
     * and atomic, so we disturb no other GPIO in the bank.
     */
    Gpiobase    = 0xD4019000,
    Gpiolen     = 0x800,
    Gpiobank3   = 0x100/4,
    GPSR        = 0x18/4,        /* set output high */
    GPCR        = 0x24/4,        /* set output low */
    GSDR        = 0x54/4,        /* set direction = output */
    Vbusgpio    = 97,            /* USB30_VBUS enable, bank 3 bit 1 */
    Hubgpio     = 123,           /* USB30_HUB enable,  bank 3 bit 27 */
    Hubrstgpio  = 124,           /* VL817 reset (active low), bank 3 bit 28 */

    /* PMU registers for USB */
    Usbclkres   = 0x5C/4,        /* PMU_USB_CLK_RES_CTRL */
    Usbphyctrl0 = 0x110/4,       /* combo-phy mode select (phy_sel @ 0xd4282910) */
    Combophyrst = 0x3CC/4,       /* APMU_PCIE_CLK_RES_CTRL_0 */
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
    Usb3_0_core_mask = 0x0F00,   /* bits 8-11: usb30 clk + AHB/VCC/PHY resets */
    Usb_axi_mask     = 0x0003,   /* bits 0-1: AXI clock + AXI reset release */
    Usb_p1_mask      = 0x0030,   /* bits 4-5: per-port AXI reset + clock */
    Usb_pmu_enable   = Usb3_0_core_mask | Usb_axi_mask | Usb_p1_mask,
    Combo_phy_usb3   = 1<<3,     /* Assign the Combo PHY to USB3 or PCIe (0) */

    IrqUsb3     = 125,           /* sys_int_ap interrupt of the USB3 controller */
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

    gpio = vmap(Gpiobase, Gpiolen);
    if(gpio == nil){
        print("usbxhcirv64: vmap gpio failed, base=%ux\n", Gpiobase);
        return;
    }
    pwr = 1<<(Vbusgpio%32) | 1<<(Hubgpio%32);
    rst = 1<<(Hubrstgpio%32);

    /* configure VBUS, hub power and hub reset as outputs */
    gpio[Gpiobank3 + GSDR] = pwr | rst;

    /* power on VBUS + hub while holding the hub in reset (reset is low) */
    gpio[Gpiobank3 + GPCR] = rst;       /* assert reset (active low) */
    gpio[Gpiobank3 + GPSR] = pwr;       /* VBUS + hub power high */
    delay(10);                          /* let the rails settle in reset */

    /* release the hub from reset and give it time to come up */
    gpio[Gpiobank3 + GPSR] = rst;       /* deassert reset (drive high) */
    delay(50);

    vunmap(gpio, Gpiolen);
}

/*
 * Bring up the USB3 combo PHY (spacemit,k1x-combphy @ 0xC0B10000) enough to
 * provide the DWC3's SuperSpeed PIPE clock.  This is not optional even for
 * USB2: the DWC3 HCRST resets every port block, and the SS block waits
 * forever for this clock, so without the combo PHY HCRST never completes and
 * the whole controller is wedged.
 *
 * Prerequisites done in reset(): USB3 combo-phy mode selected (Combo_phy_usb3)
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
        Combophybase = 0xC0B10000,
        Combophylen  = 0x800,
        Reg1   = 0x68/4,        /* = 0 */
        Reg2   = 0x48/4,        /* = 0x603a2276 */
        Reg3   = 0x08/4,        /* = 0x97c; doubles as the PLL status reg */
        Reg4   = 0x18/4,        /* = 0 */
        Pllrdy = 1<<0,          /* PLL locked, read from Reg3 */
        Lfps   = 0x58/4,        /* LFPS threshold, bits [10:8] */
    };

    phy = vmap(Combophybase, Combophylen);
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
        if(phy[Reg3] & Pllrdy)
            break;
        microdelay(10);
    }
    phy[Lfps] = (phy[Lfps] & ~(0x7<<8)) | (0x3<<8);     /* lfps threshold = 3 */
    if((phy[Reg3] & Pllrdy) == 0)
        print("usbxhcirv64: usb3 combophy PLL not ready (reg8=%ux)\n", phy[Reg3]);
    vunmap(phy, Combophylen);
}

/*
 * Bring up the K1's standalone USB2 PHY (register block at Usb2phybase,
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
        GCTL             = 0xC110/4,
        /* Additional global registers */
        GUCTL            = 0xC12C/4,    /* Global User Control Register */
        GUSB2PHYCFG      = 0xC200/4,    /* USB2 (UTMI/ULPI) PHY config, port 0 */
        GUSB3PIPECTL     = 0xC2C0/4,    /* USB3 PIPE control, port 0 */
        GFLADJ           = 0xC630/4,    /* Global Frame Length Adjustment */
        PWRDNSCALE_SHIFT = 19,

        PWRDNSCALE_MASK  = 0x3FFF << PWRDNSCALE_SHIFT,
        /* Literal for 125MHz */
        Pwrdnscale_125m  = 0x2c,

        PRTCAPDIR_SHIFT  = 12,
        PRTCAPDIR_MASK   = 3 << PRTCAPDIR_SHIFT,
        DISSCRAMBLE      = 1<<3,
        DSBLCLKGTNG      = 1<<0,

        /* Specific bits */
        USBHSTINAUTORETRY = 1<<14,      /* Host IN Auto-Retry Enable */

        GFLADJ_30MHZ_MASK       = 0x3F,
        GFLADJ_30MHZ_SDBND_SEL  = 1<<7,

        /* GUSB2PHYCFG fields (this K1 wires a UTMI+ 8-bit HS PHY) */
        U2_FREECLKEXISTS = 1<<30,       /* free clock does not exist here */
        U2_ENBLSLPM      = 1<<8,        /* let controller suspend the PHY */
        U2_SUSPHY        = 1<<6,        /* suspend USB2 PHY when idle */
        U2_ULPI_UTMI     = 1<<4,        /* 0 = UTMI+, 1 = ULPI */
        U2_PHYIF         = 1<<3,        /* 0 = 8-bit, 1 = 16-bit UTMI */
        U2_USBTRDTIM_MASK= 0xF<<10,
        U2_USBTRDTIM_8BIT= 9<<10,       /* turnaround time for 8-bit UTMI */

        /* GUSB3PIPECTL */
        U3_PHYSOFTRST    = 1u<<31,      /* hold USB3 PIPE PHY iface in reset */
        U3_SUSPHY        = 1<<17,       /* suspend USB3 PHY when idle */
    };
    u32int v;

    /* 1. Global Control configuration (GCTL) */
    v = reg[GCTL];
    v &= ~(PWRDNSCALE_MASK | DISSCRAMBLE | DSBLCLKGTNG | PRTCAPDIR_MASK);
    v |= Pwrdnscale_125m<<PWRDNSCALE_SHIFT | 1<<PRTCAPDIR_SHIFT;
    reg[GCTL] = v;

    /* 2. Enable Automatic Retry of IN Transactions (GUCTL) */
    /* This bit lets the host automatically retry when there are CRC
       errors instead of waiting for software intervention. */
    reg[GUCTL] |= USBHSTINAUTORETRY;

    /* 3. Frame Length Adjustment (GFLADJ) */
    /* A typical adjustment value of 0x20 is used for synchronization,
       matching the fladj_30mhz_reg field seen in the K1 manual */
    reg[GFLADJ] = (reg[GFLADJ] & ~GFLADJ_30MHZ_MASK) | 0x20 | GFLADJ_30MHZ_SDBND_SEL;

    /*
     * 4. USB2 PHY interface (GUSB2PHYCFG).  The reset default can leave
     * the port in ULPI mode and/or with PHY auto-suspend enabled, which
     * keeps the UTMI HS receiver powered down so no downstream connect is
     * ever seen (PORTSC stuck at CCS=0, PLS=RxDetect).  Select UTMI+ 8-bit
     * and disable the suspend/sleep/free-clock behaviours the K1 doesn't
     * support, matching the controller's required configuration.
     */
    v = reg[GUSB2PHYCFG];
    v &= ~(U2_ULPI_UTMI | U2_PHYIF | U2_USBTRDTIM_MASK |
           U2_SUSPHY | U2_ENBLSLPM | U2_FREECLKEXISTS);
    v |= U2_USBTRDTIM_8BIT;
    reg[GUSB2PHYCFG] = v;

    /*
     * 5. Keep the USB3 PHY active (not suspended, not held in soft reset).
     * The combo PHY is brought up in usb3phyinit() so it provides the
     * SuperSpeed PIPE clock; that clock is what lets the controller's HCRST
     * complete.  If the SS port block is left without a clock, HCRST never
     * finishes (USBCMD.HCRST stays set) and the whole controller wedges --
     * even PORTSC.PR on the USB2 port becomes inert.
     */
    reg[GUSB3PIPECTL] &= ~(U3_SUSPHY | U3_PHYSOFTRST);
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
    reg = vmap(Pmubase, Pmulen);
    if(reg != nil){
        reg[Usbclkres] |= Usb_pmu_enable;       /* usb30 clocks + resets released */
        reg[Usbphyctrl0] |= Combo_phy_usb3;     /* select USB3 combo-phy mode */
        reg[Combophyrst] &= ~Combophyrstbit;    /* release RESET_COMBO_PHY */
        coherence();
        vunmap(reg, Pmulen);
    } else {
        print("usbxhcirv64: vmap pmu failed, base=%ux len=%ux\n", Pmubase, Pmulen);
        return -1;
    }

    /*
     * 1b. Bring up the USB3 combo PHY so the DWC3 SuperSpeed port block has a
     * PIPE clock; otherwise the controller's HCRST never completes and even
     * USB2 is dead.
     */
    usb3phyinit();

    /* 2. Set up the Xhci structure and the DWC3 core */
    mmio = vmap(Xhcibase, Xhcilen);
    if(mmio == nil) {
        print("usbxhcirv64: vmap xhci failed\n");
        return -1;
    }
    ctlr = xhcialloc(mmio, Xhcibase, Xhcilen);
    if(ctlr == nil){
        print("usbxhcirv64: xhcialloc failed\n");
        vunmap(mmio, Xhcilen);
        return -1;
    }

    /* 3. Configure the DWC3 core */
    coreinit(ctlr->mmio);

    /* 4. Bring up the standalone USB2 PHY (its own MMIO block) */
    phy = vmap(Usb2phybase, Usb2phylen);
    if(phy != nil){
        usb2phyinit(phy);
        vunmap(phy, Usb2phylen);
    } else
        print("usbxhcirv64: vmap usb2phy failed, base=%ux len=%ux\n",
            Usb2phybase, Usb2phylen);

    /* 5. Link with the USB system */
    hp->irq = IrqUsb3;
    hp->tbdf = BUSUNKNOWN;
    xhcilinkage(hp, ctlr);

    return 0;
}

void
usbxhcirv64link(void)
{
    addhcitype("xhci", reset);
}
