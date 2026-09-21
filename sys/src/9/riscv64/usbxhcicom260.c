/*
 * Driver for the SpacemiT K3 USB host controller (xhci)
 * -- K3-CoM260 (BPI-SM10) variant --
 *
 * Brings up usb3@81400000 ("usb3_portb" in the device tree), the DWC3
 * instance whose downstream USB-A ports are the onboard VL817 hub.
 * Confirmed by the board's compiled DTB, where the VL817's two USB
 * IDs (usb2109,2817 / usb2109,817) are declared as direct children of
 * this node.
 *
 * usb2@c0a00000 ("usb2_host") is a separate, USB2-only DWC3 instance
 * wired to the M.2 Key-E slot (Bluetooth); this file does not touch it.
 *
 * Addresses, IRQs and apmu offsets come from the board's DTB.  The
 * combo-PHY mux and the USB3 and USB2 PHY bring-up sequences are not
 * documented in the manual; the same USB2 PHY answers to both the K1
 * and K3 compatible strings, so what usbxhcibpif3.c established on the
 * K1 applies here.
 *
 * Based on:
 *   9front usbxhciimx.c   - general driver structure on imx8 arch
 *   9front usbxhcilx2k.c  - general driver structure on lx2k arch
 *   usbxhcibpif3.c         - original SpacemiT K1 port
 *   usbxhcipicoitx.c       - K3 Pico-ITX port
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
    /* usb3_portb: the DWC3 instance behind the onboard VL817 hub */
    Xhcibase    = 0x81400000,
    Xhcilen     = 0x00010000,

    /* usb3_portb_u2phy: phy@81500000, "spacemit,k3-usb2-phy" */
    Usb2phybase = 0x81500000,
    Usb2phylen  = 0x200,

    /* usb3_portb_u3phy: phy@81f00000, "spacemit,k3-usb3-phy" */
    Usb3phybase = 0x81f00000,
    Usb3phylen  = 0x200,

    /* AP PMU (APMU) block */
    Apmubase    = 0xD4282800,
    Apmulen     = 0x400,

    /* usb3_portb's dedicated DWC3 clk/reset register within APMU.
     * Each DWC3 instance has its own 32-bit register: 0x7C (usb2),
     * 0x3C4/0x3C8/0x3CC/0x3D0 (USB3 porta/b/c/d). */
    Usb3bclkres  = 0x3c8/4,
    Usb3bmask    = 0xF,        /* bits [3:0]: clk-enable + 3 resets */

    /* PMUA_PCIE_SUBSYS_MGMT, within APMU: PCIe/USB3 combo lane mux */
    Pciesubsysmgmt   = 0x1d8/4,
    X8disablebit     = 1<<4,     /* 0 = PCIe-A x8, 1 = combo/matrix mode */
    Usb3bcombobit    = 1<<2,     /* combo-usb-bit=2: 0=PCIe, 1=USB for port B */

    /* "apb-spare" syscon = pll@0xD4090000. Used only for the USB3
     * PHY's PU_CAL/RCAL_HSIO calibration handshake. */
    Apbsparebase = 0xD4090000,
    Apbsparelen  = 0x200,

    IrqUsb3b     = 125,          /* dwc3 irq for usb3_portb */
    IrqUsb3bWake = 157,          /* wakeup irq -- unused below */
};

/*
 * USB3 PHY registers, relative to Usb3phybase (0x81f00000).
 */
enum {
    Phyversion      = 0x0/4,

    Phyresetcfg     = 0x04/4,
    Ensampledataafterlock = 1<<6,

    Phyclkcfg       = 0x08/4,
    Pllready        = 1<<0,
    Cfgrxclken      = 1<<3,
    Cfgtxclken      = 1<<4,
    Cfgpclken       = 1<<5,
    Cfgpipepclken   = 1<<6,
    Cfgrefclkfreqshift = 7,
    Refclk24m       = 0x2,
    Cfgswinitdone   = 1<<11,
    Cfgpussout      = 1<<23,

    Pciephyoverride = 0x18/4,
    Ovrdmpuu3       = 1<<17,
    Cfgmpuu3        = 1<<16,

    Phymodecfg      = 0x0C/4,
    Cfglfpstperiodshift = 8,
    Cfglfpstperiodmask  = 3<<8,
    Lfpstperiodusb  = 0x3,

    Phypuckreg      = 0x54/4,
    Purefclk100     = 1<<25,

    Phypllreg1      = 0x58/4,
    Frefselshift    = 13,
    Fref24m         = 0x1,
    Sscdepselshift  = 24,
    Ssc5000ppm      = 0xa,
    Sscmodeshift    = 28,
    Sscdownspread1  = 0x3,

    Phypllreg2      = 0x5c/4,
    Selref100       = 1<<21,

    Phyrxrega       = 0x60/4,
    Rxreg0mask      = 0xFF,
    Rxreg0rload     = 1<<4,
    Rxreg1mask      = 0xFF<<8,
    Rxreg1rccalidefault  = 7<<12,
    Rxreg1rtermdefault   = 8<<8,
    Rxreg2mask      = 0xFF<<16,
    Rxreg2pseldefault    = 4<<21,
    Rxreg2forcecsel = 1<<20,
    Rxreg2cseldefault    = 8<<16,
    Rxreg3mask      = 0xFFu<<24,
    Rxreg3rdeg1default   = 3u<<30,
    Rxreg3adjbiasdefault = 1u<<28,
    Rxreg3selcboostcode  = 1<<27,
    Rxreg3iloaddefault   = 7<<24,

    Phyrxregb       = 0x64/4,
    Rxreg4mask      = 0xFF,
    Rxreg4manualcfg = 1<<7,
    Rxreg4rtermsel  = 1<<5,
    Rxreg4envos     = 1<<4,
    Rxreg4rdeg2default   = 2<<1,
    Rxreg5mask      = 0xFF<<8,
    Rxreg5rcellbiasdefault = 8<<12,
    Rxreg5rcellvcmdefault  = 8<<8,
    Rxreg6mask      = 0xFF<<16,
    Rxreg6adaptgaindefault = 2<<20,
    Rxreg6h1regdefault     = 8<<16,

    Phyrxeqtime     = 0xb4/4,
    Rxeqtimeovrdampsoc  = 1<<24,
    Rxeqtimecfgampsocshift = 22,
    Rxeqtimecfgampsocmask  = 3<<22,
    Ampsoc900m      = 0x3,      /* default when amp-lvl absent from DT */

    Phyadptcfg0     = 0x140/4,
    Afeadptrstovrden  = 1<<1,
    Afeadptrstovrdval = 1<<4,
};

/*
 * apb-spare (pll@0xD4090000) registers used for USB3 PHY calibration.
 */
enum {
    Apbsparepucal   = 0x178/4,
    Pucal           = 1<<17,

    Apbsparercalhsio = 0x17c/4,
    Pucaldone       = 1<<8,
    Rcalovrdstableen  = 1u<<31,
    Rcalovrdstableval = 1<<30,
    Rcalovrdntrimen   = 1<<29,
    Rcalovrdptrimen   = 1<<28,
    Rcalovrdntrimshift = 24,
    Rcalovrdptrimshift = 20,
    Ntrimdefault    = 0x6,
    Ptrimdefault    = 0xa,

    Polldelayus     = 500,
    Pucaltimeoutus  = 2000000,
    Plltimeoutus    = 500000,
};

/*
 * USB2 PHY registers, relative to Usb2phybase (0x81500000).
 */
enum {
    Phyrstmodectrl  = 0x04/4,
    Physub2pllrdy   = 1<<0,
    Physub2clkcdren = 1<<1,
    Physub2clkpllen = 1<<2,
    Physub2clkmacen = 1<<3,
    Physub2macrstn  = 1<<5,
    Physub2cdrrstn  = 1<<6,
    Physub2pllrstn  = 1<<7,
    Physub2hslinetxmode = 1<<13,
    Physub2fslinetxmode = 1<<14,

    Phytxhostctrl   = 0x10/4,
    Physub2hstdiscautoclr = 1<<2,

    Phyhstxphwctrl  = 0x34/4,
    Physub2hstxprstn  = 1<<2,
    Physub2clkhstxpen = 1<<3,
    Physub2hstxpmode  = 1<<4,

    /* PLL divider field: bits[12:0], select 24MHz reference */
    Phyplldivcfg    = 0x98/4,
    Physub2fdivregmask  = 0x1FFF,
    Physub2fdivregval   = 0x1ec4,
    Physub2fdivfract01shift = 13,
    Physub2selfreq24mhz = 0x01,
    Physub2divlocalen   = 1<<15,

    Usb2physetuptimeus  = 200,
    Usb2physetuptimemsp = 50,
};

/* pollset - busy-poll reg[idx] until (reg[idx]&mask)==mask, or timeout */
static int
pollset(u32int *reg, int idx, u32int mask, int timeoutus)
{
    int waited;

    for(waited = 0; waited < timeoutus; waited += Polldelayus){
        if((reg[idx] & mask) == mask)
            return 0;
        microdelay(Polldelayus);
    }
    return -1;
}

/* usb3bcomboselectusb - route usb3_portb's shared SerDes lane to USB3 */
static void
usb3bcomboselectusb(u32int *apmureg)
{
    u32int mask, val;

    mask = Usb3bcombobit | X8disablebit;
    val  = Usb3bcombobit | X8disablebit;

    apmureg[Pciesubsysmgmt] = (apmureg[Pciesubsysmgmt] & ~mask) | val;
    print("usbxhcicom260: pcie_subsys_mgmt=%ux (port B -> USB)\n",
        apmureg[Pciesubsysmgmt]);
}

/* usb3phyinit - bring up the USB3/combo PHY at Usb3phybase */
static int
usb3phyinit(u32int *phyreg, u32int *apbspare)
{
    u32int reg, version;

    phyreg[Pciephyoverride] &= ~(Ovrdmpuu3 | Cfgmpuu3);

    reg = phyreg[Phyclkcfg];
    if(reg & Cfgswinitdone){
        print("usbxhcicom260: usb3-phy already initialized, skip\n");
        return 0;
    }

    version = phyreg[Phyversion];

    /* HSIO resistor calibration */
    apbspare[Apbsparepucal] |= Pucal;
    if(pollset(apbspare, Apbsparercalhsio, Pucaldone, Pucaltimeoutus) < 0){
        print("usbxhcicom260: usb3-phy PU/RCAL timeout, forcing trim override\n");
        apbspare[Apbsparercalhsio] =
            (apbspare[Apbsparercalhsio] &
                ~(Rcalovrdntrimen|Rcalovrdptrimen|Rcalovrdstableval|
                  (0xFu<<Rcalovrdntrimshift)|(0xFu<<Rcalovrdptrimshift))) |
            Rcalovrdntrimen | Rcalovrdptrimen | Rcalovrdstableval |
            (Ntrimdefault<<Rcalovrdntrimshift) |
            (Ptrimdefault<<Rcalovrdptrimshift);
        apbspare[Apbsparercalhsio] |= Rcalovrdstableen;
    }

    /* Don't wait for CDR lock before sampling data */
    phyreg[Phyresetcfg] &= ~Ensampledataafterlock;

    /* Power down the unused 100MHz refclk buffer */
    phyreg[Phypuckreg] &= ~Purefclk100;

    /* PLL SSC config: down-spread, 5000ppm, 24MHz reference */
    phyreg[Phypllreg1] =
        (Sscdownspread1<<Sscmodeshift) |
        (Ssc5000ppm<<Sscdepselshift) |
        (Fref24m<<Frefselshift);

    /* Un-select the 100MHz PLL reference path */
    phyreg[Phypllreg2] &= ~Selref100;

    /* USB (not PCIe) LFPS period */
    phyreg[Phymodecfg] = (phyreg[Phymodecfg] & ~Cfglfpstperiodmask) |
        (Lfpstperiodusb<<Cfglfpstperiodshift);

    /* Force AFE adaptation reset */
    phyreg[Phyadptcfg0] |= Afeadptrstovrden | Afeadptrstovrdval;

    /* Driver amplitude: 900mV default */
    phyreg[Phyrxeqtime] =
        (phyreg[Phyrxeqtime] & ~(Rxeqtimeovrdampsoc|Rxeqtimecfgampsocmask)) |
        Rxeqtimeovrdampsoc | (Ampsoc900m<<Rxeqtimecfgampsocshift);

    /* RX analog parameter defaults */
    phyreg[Phyrxrega] = (phyreg[Phyrxrega] & ~Rxreg0mask) | Rxreg0rload;
    phyreg[Phyrxrega] = (phyreg[Phyrxrega] & ~Rxreg1mask) |
        Rxreg1rccalidefault | Rxreg1rtermdefault;
    phyreg[Phyrxrega] = (phyreg[Phyrxrega] & ~Rxreg2mask) |
        Rxreg2pseldefault | Rxreg2forcecsel | Rxreg2cseldefault;
    phyreg[Phyrxrega] = (phyreg[Phyrxrega] & ~Rxreg3mask) |
        Rxreg3rdeg1default | Rxreg3adjbiasdefault |
        Rxreg3selcboostcode | Rxreg3iloaddefault;
    phyreg[Phyrxregb] = (phyreg[Phyrxregb] & ~Rxreg4mask) |
        Rxreg4manualcfg | Rxreg4rtermsel | Rxreg4envos | Rxreg4rdeg2default;
    phyreg[Phyrxregb] = (phyreg[Phyrxregb] & ~Rxreg5mask) |
        Rxreg5rcellbiasdefault | Rxreg5rcellvcmdefault;
    phyreg[Phyrxregb] = (phyreg[Phyrxregb] & ~Rxreg6mask) |
        Rxreg6adaptgaindefault | Rxreg6h1regdefault;

    /* Commit config; PLL won't start locking until this is written */
    phyreg[Phyclkcfg] =
        Cfgswinitdone | Cfgpussout |
        (Refclk24m<<Cfgrefclkfreqshift) |
        Cfgrxclken | Cfgpclken | Cfgpipepclken | Cfgtxclken;

    if(pollset(phyreg, Phyclkcfg, Pllready, Plltimeoutus) < 0){
        print("usbxhcicom260: usb3-phy PLL lock timeout!\n");
        return -1;
    }

    print("usbxhcicom260: usb3-phy version=%ux initialized as USB3 mode\n",
        version);
    return 0;
}

/* usb2phyinit - bring up the standalone usb2-phy block */
static int
usb2phyinit(u32int *phyreg)
{
    u32int val;

    microdelay(Usb2physetuptimeus);

    val = (Physub2fdivregval & Physub2fdivregmask) |
        (Physub2selfreq24mhz<<Physub2fdivfract01shift) |
        Physub2divlocalen;
    phyreg[Phyplldivcfg] = val;

    if(pollset(phyreg, Phyrstmodectrl, Physub2pllrdy,
        Usb2physetuptimemsp*1000) < 0){
        print("usbxhcicom260: usb2-phy PLL_RDY timeout\n");
        return -1;
    }

    val = Physub2fslinetxmode | Physub2hslinetxmode |
        Physub2clkpllen | Physub2clkcdren | Physub2clkmacen |
        Physub2pllrstn | Physub2cdrrstn | Physub2macrstn;
    phyreg[Phyrstmodectrl] = val;

    val = Physub2hstxprstn | Physub2clkhstxpen | Physub2hstxpmode;
    phyreg[Phyhstxphwctrl] = val;

    phyreg[Phytxhostctrl] |= Physub2hstdiscautoclr;

    print("usbxhcicom260: usb2-phy initialized\n");
    return 0;
}

static void
coreinit(u32int *reg)
{
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

    v = reg[Gctl];
    v &= ~(Pwrdnscalemask | Disscramble | Dsblclkgtng | Prtcapdirmask);
    v |= Pwrdnscale125m<<Pwrdnscaleshift | 1<<Prtcapdirshift;
    reg[Gctl] = v;

    reg[Guctl] |= Usbhstinautoretry;

    reg[Gfladj] = (reg[Gfladj] & ~Gfladj30mhzmask) | 0x20 | Gfladj30mhzsdbndsel;
}

static int
reset(Hci *hp)
{
    static Xhci *ctlr;
    u32int *apmureg;
    u32int *apbsparereg;
    u32int *phymmio;
    void *mmio;

    if(ctlr != nil)
        return -1;

    if(!dtbsocmatch("spacemit,k3-com260-ifx"))
        return -1;
    print("usbxhcicom260: reset called\n");

    /* 1. Select combo mode = USB for port B, then release this
     * port's own DWC3 clock/reset */
    apmureg = vmap(Apmubase, Apmulen);
    if(apmureg == nil){
        print("usbxhcicom260: vmap apmu failed\n");
        return -1;
    }
    usb3bcomboselectusb(apmureg);
    apmureg[Usb3bclkres] |= Usb3bmask;
    vunmap(apmureg, Apmulen);

    /* 2. Set up the Xhci structure and the DWC3 core */
    mmio = vmap(Xhcibase, Xhcilen);
    if(mmio == nil) {
        print("usbxhcicom260: vmap xhci failed\n");
        return -1;
    }
    ctlr = xhcialloc(mmio, Xhcibase, Xhcilen);
    if(ctlr == nil){
        print("usbxhcicom260: xhcialloc failed\n");
        vunmap(mmio, Xhcilen);
        return -1;
    }

    /* 3. Configure the DWC3 core */
    coreinit(ctlr->mmio);

    /* 4a. Bring up the USB2 PHY */
    phymmio = vmap(Usb2phybase, Usb2phylen);
    if(phymmio == nil){
        print("usbxhcicom260: vmap usb2-phy failed\n");
        xhcilinkage(hp, ctlr);
        return -1;
    }
    if(usb2phyinit(phymmio) < 0)
        print("usbxhcicom260: usb2-phy init failed, continuing anyway\n");
    vunmap(phymmio, Usb2phylen);

    /* 4b. Bring up the USB3/combo PHY */
    apbsparereg = vmap(Apbsparebase, Apbsparelen);
    if(apbsparereg == nil){
        print("usbxhcicom260: vmap apb-spare failed\n");
        xhcilinkage(hp, ctlr);
        return -1;
    }
    phymmio = vmap(Usb3phybase, Usb3phylen);
    if(phymmio == nil){
        print("usbxhcicom260: vmap usb3-phy failed\n");
        vunmap(apbsparereg, Apbsparelen);
        xhcilinkage(hp, ctlr);
        return -1;
    }
    if(usb3phyinit(phymmio, apbsparereg) < 0)
        print("usbxhcicom260: usb3-phy init failed, continuing anyway\n");
    vunmap(phymmio, Usb3phylen);
    vunmap(apbsparereg, Apbsparelen);

    /* 5. Link with the USB system */
    hp->irq = IrqUsb3b;
    hp->tbdf = BUSUNKNOWN;
    xhcilinkage(hp, ctlr);

    return 0;
}

void
usbxhcicom260link(void)
{
    addhcitype("xhci", reset);
}
