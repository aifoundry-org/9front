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
    /* SoC K1 base addresses */
    Xhcibase    = 0xC0A00000,    /* xHCI controller */
    Xhcilen     = 0x100000,
    Pmubase     = 0xD4282800,    /* Power Management Unit (AP PMU) */
    Pmulen      = 0x1000,

    /* PMU registers for USB */
    Usbclkres   = 0x5C/4,        /* PMU_USB_CLK_RES_CTRL */
    Usbphyctrl0 = 0x110/4,       /* PMUA_USB_PHY_CTRL0 */

    /* PMU bit masks */
    Usb3_0_core_mask = 0x0F00,   /* PHY, VCC, AHB resets and Bus clock */
    Usb_axi_mask     = 0x0003,   /* AXI clock and AXI de-reset */
    Usb_pmu_enable   = Usb3_0_core_mask | Usb_axi_mask,
    Combo_phy_usb3   = 1<<3,     /* Assign the Combo PHY to USB3 or PCIe (0) */

    IrqUsb3     = 125,           /* sys_int_ap interrupt of the USB3 controller */
};

static void
usb2phyinit(u32int *mmio)
{
    enum {
        USB_CTL     = 0x10104/4,        /* Xhcibase + 0x10000 (wrapper) + 0x104 */
        Usb2phyregreset = 1<<14,        /* soft reset for USB2 PHY registers */
    };

    /*
     * K1 ch. 15, USB_CTL bit 14: "Soft reset for USB2 PHY registers.
     * This reset is for backup purposes only." No PLL-ready/status bit
     * is documented for this port's USB2 PHY, so there is nothing to
     * poll on: pulse the reset and move on.
     */
    mmio[USB_CTL] |= Usb2phyregreset;
    microdelay(10);
    mmio[USB_CTL] &= ~Usb2phyregreset;

    /*
     * Leave reg_sel (bit 6) at its reset value of 0 so the USB3.0 DRD
     * controller drives op_mode/xcvr_select/term_select on the PHY
     * interface itself, instead of overriding them by hand.
     */
}

static void
coreinit(u32int *reg)
{
    enum {
        GCTL             = 0xC110/4,
        /* Additional global registers */
        GUCTL            = 0xC12C/4,    /* Global User Control Register */
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
    };
    u32int v;

    /* 1. Global Control configuration (GCTL) */
    v = reg[GCTL];
    v &= ~(PWRDNSCALE_MASK | DISSCRAMBLE | DSBLCLKGTNG | PRTCAPDIR_MASK);
    v |= Pwrdnscale_125m<<PWRDNSCALE_SHIFT | 1<<PRTCAPDIR_SHIFT;
    reg[GCTL] = v;
    print("usbxhcirv64: gctl=%ux\n", reg[GCTL]); 

    /* 2. Enable Automatic Retry of IN Transactions (GUCTL) */
    /* This bit lets the host automatically retry when there are CRC
       errors instead of waiting for software intervention. */
    reg[GUCTL] |= USBHSTINAUTORETRY;

    /* 3. Frame Length Adjustment (GFLADJ) */
    /* A typical adjustment value of 0x20 is used for synchronization,
       matching the fladj_30mhz_reg field seen in the K1 manual */
    reg[GFLADJ] = (reg[GFLADJ] & ~GFLADJ_30MHZ_MASK) | 0x20 | GFLADJ_30MHZ_SDBND_SEL;
}

static int
reset(Hci *hp)
{
    static Xhci *ctlr;
    u32int *reg;
    void *mmio;

    print("usbxhcirv64: reset called\n");
    if(ctlr != nil)
        return -1;

    /* 1. PMU: enable clocks and release the controller's resets */
    reg = vmap(Pmubase, Pmulen);
    if(reg != nil){
        print("usbxhcirv64: pmu usbclkres=%ux\n", reg[Usbclkres]);
        reg[Usbclkres] |= Usb_pmu_enable;
        reg[Usbphyctrl0] |= Combo_phy_usb3;
        vunmap(reg, Pmulen);
    } else {
        print("usbxhcirv64: vmap pmu failed, base=%ux len=%ux\n", Pmubase, Pmulen);
        return -1;
    }

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
    print("usb mmio=%ux\n", ctlr->mmio[1]);

    /* 3. Configure the DWC3 core and reset the USB2 PHY */
    coreinit(ctlr->mmio);
    print("usb mmio=%ux\n", ctlr->mmio[1]);
    usb2phyinit(ctlr->mmio);
 
    /* 4. Link with the USB system */
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
