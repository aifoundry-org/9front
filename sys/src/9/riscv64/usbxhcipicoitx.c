/*
 * Driver for the SpacemiT K3 USB host controller (xhci)
 *
 * Based on:
 *   9front usbxhciimx.c  - general driver structure on imx8 arch
 *   9front usbxhcilx2k.c - general driver structure on lx2k arch
 *   usbxhcibpif3.c         - original SpacemiT K1 port (this file's ancestor)
 *
 * What the K3 device tree gives for this controller:
 *   usb2_host (dwc3):  reg 0xC0A00000, len 0x10000
 *   usb2_phy:          reg 0xC0A20000, len 0x200   (separate MMIO block,
 *                       unlike K1 where the USB2 PHY reset lived inside
 *                       the DWC3 wrapper at offset 0x10104)
 *   interrupt:         105, routed via saplic (K3's interrupt controller,
 *                       NOT plic as on K1) -- confirm the IRQ numbering
 *                       your saplic/plic glue in 9front actually uses.
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
    /* SoC K3 base addresses -- from the K3 device tree */
    Xhcibase    = 0xC0A00000,    /* usb2_host DWC3 controller */
    Xhcilen     = 0x10000,
    Usb2phybase = 0xC0A20000,    /* usb2_phy, separate block on K3 */
    Usb2phylen  = 0x200,

    /* AP PMU base */
    Apmubase    = 0xD4282800,
    Apmulen     = 0x1000,

    /*
     * PMU_USB_CLK_RES_CTRL:
     *   bits [3:0]   USB2 host
     *   bits [7:4]   USB3 port A (DRD)
     *   bits [11:8]  USB3 port B (host)
     *   bits [15:12] USB3 port C (host)
     *   bits [19:16] USB3 port D (host)
     *   bits [31:20] reserved
     *
     * Within the USB2 group specifically:
     *   bit 3  USB2_PORT_PHY_RESETN  (1 = de-assert reset)
     *   bit 2  USB2_PORT_VCC_RESETN  (1 = de-assert reset)
     *   I'm not sure about the following
     *   bit 1  either AHB_RSTN or BUS_CLK_EN
     *   bit 0  the other one of the above
     *
     * The bit0/bit1 swap doesn't matter for bring-up: reset value of
     * all four bits is 0, and writing 0xF to bits[3:0] sets clock
     * enable and de-asserts every reset for the port simultaneously
     */
    Usbclkres    = 0x5C/4,
    Usb2mask     = 0xF,        /* bits [3:0]: USB2 host clk+3 resets */
    Usb3amask    = 0xF<<4,     /* bits [7:4]: USB3 port A -- unused here */
    Usb3bmask    = 0xF<<8,     /* bits [11:8]: USB3 port B -- unused here */
    Usb3cmask    = 0xF<<12,    /* bits [15:12]: USB3 port C -- unused here */
    Usb3dmask    = 0xF<<16,    /* bits [19:16]: USB3 port D -- unused here */

    /*
     * PMU_USB_SD_ROT_WAKE_CLR: wake/VBUS driver control for the USB2 host
     * port specifically. Left declared but unused below.
     */
    Usbsdrotwakeclr = 0x7C/4,

    IrqUsb2     = 105,            /* usb2_host interrupt, per the DT */
};

/* usb2phyinit - reset/bring-up for the standalone usb2_phy block */
static void
usb2phyinit(u32int *phymmio)
{
    enum {
        Phyctrl0    = 0x0/4,
        Physoftrst  = 1<<0,
    };

    phymmio[Phyctrl0] |= Physoftrst;
    microdelay(10);
    phymmio[Phyctrl0] &= ~Physoftrst;
}

static void
coreinit(u32int *reg)
{
    /*
     * DWC3 core registers (GCTL/GUCTL/GFLADJ) are part of the
     * Synopsys IP itself, not the SoC wrapper, so these offsets and
     * bitfields are carried over unchanged from the K1 driver and
     * should still be correct on K3 -- same IP block.
     */
    enum {
        Gctl             = 0xC110/4,
        Guctl            = 0xC12C/4,
        Gfladj           = 0xC630/4,
        Pwrdnscaleshift  = 19,

        Pwrdnscalemask   = 0x3FFF << Pwrdnscaleshift,
        Pwrdnscale125m   = 0x2c,

        Prtcapdirshift   = 12,
        Prtcapdirmask    = 3 << Prtcapdirshift,
        Disscramble      = 1<<3,
        Dsblclkgtng      = 1<<0,

        Usbhstinautoretry = 1<<14,

        Gfladj30mhzmask      = 0x3F,
        Gfladj30mhzsdbndsel  = 1<<7,
    };
    u32int v;

    /* 1. Global Control configuration (GCTL) */
    v = reg[Gctl];
    v &= ~(Pwrdnscalemask | Disscramble | Dsblclkgtng | Prtcapdirmask);
    v |= Pwrdnscale125m<<Pwrdnscaleshift | 1<<Prtcapdirshift;
    reg[Gctl] = v;
    print("usbxhcipicoitx: gctl=%ux\n", reg[Gctl]);

    /* 2. Enable Automatic Retry of IN Transactions (GUCTL) */
    reg[Guctl] |= Usbhstinautoretry;

    /* 3. Frame Length Adjustment (GFLADJ) */
    reg[Gfladj] = (reg[Gfladj] & ~Gfladj30mhzmask) | 0x20 | Gfladj30mhzsdbndsel;
}

static int
reset(Hci *hp)
{
    static Xhci *ctlr;
    u32int *apmureg;
    u32int *phymmio;
    void *mmio;

    if(ctlr != nil)
        return -1;

    /*
     * Unified kernel: this driver owns the K3's usb2_host DWC3
     * controller.  Gate on the DTB root compatible so it stays quiet
     * on any other SoC
     */
    if(!dtbsocmatch("spacemit,k3-pico-itx"))
        return -1;
    print("usbxhcipicoitx: reset called\n");

    /* 1. APMU: enable clocks and release the controller's resets */
    apmureg = vmap(Apmubase, Apmulen);
    if(apmureg != nil){
        print("usbxhcipicoitx: apmu usbclkres=%ux\n", apmureg[Usbclkres]);
        apmureg[Usbclkres] |= Usb2mask;
        vunmap(apmureg, Apmulen);
    } else {
        print("usbxhcipicoitx: vmap apmu failed, base=%ux len=%ux\n", Apmubase, Apmulen);
        return -1;
    }

    /* 2. Set up the Xhci structure and the DWC3 core */
    mmio = vmap(Xhcibase, Xhcilen);
    if(mmio == nil) {
        print("usbxhcipicoitx: vmap xhci failed\n");
        return -1;
    }
    ctlr = xhcialloc(mmio, Xhcibase, Xhcilen);
    if(ctlr == nil){
        print("usbxhcipicoitx: xhcialloc failed\n");
        vunmap(mmio, Xhcilen);
        return -1;
    }
    print("usb mmio=%ux\n", ctlr->mmio[1]);

    /* 3. Configure the DWC3 core */
    coreinit(ctlr->mmio);
    print("usb mmio=%ux\n", ctlr->mmio[1]);

    /* 4. Reset the standalone USB2 PHY block (separate MMIO on K3) */
    phymmio = vmap(Usb2phybase, Usb2phylen);
    if(phymmio != nil){
        usb2phyinit(phymmio);
        vunmap(phymmio, Usb2phylen);
    } else {
        print("usbxhcipicoitx: vmap usb2phy failed, base=%ux len=%ux\n",
            Usb2phybase, Usb2phylen);
        /* not necessarily fatal -- PHY may already be in a usable
         * state depending on what the FSBL/U-Boot left it in, but
         * you probably want this to fail loudly until proven otherwise */
        xhcilinkage(hp, ctlr);
        return -1;
    }

    /* 5. Link with the USB system */
    hp->irq = IrqUsb2;      /* NB: 105 on K3, not 125 as on K1 */
    hp->tbdf = BUSUNKNOWN;
    xhcilinkage(hp, ctlr);

    return 0;
}

void
usbxhcipicoitxlink(void)
{
    addhcitype("xhci", reset);
}
