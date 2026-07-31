/*
 * Driver for the SpacemiT K3 USB2 host controller (usb2_host node)
 * for 9front / Plan 9 from Bell Labs
 *
 * Based on:
 *   9front usbxhciimx.c  - general driver structure on imx8 arch
 *   9front usbxhcilx2k.c - general driver structure on lx2k arch
 *   usbxhcirv64.c         - original SpacemiT K1 port (this file's ancestor)
 *
 * SKELETON / NOT PRODUCTION READY.
 *
 * VERIFIED against the upstream Linux patch "riscv: dts: spacemit: k3:
 * Add USB2.0 support" (applied to k3.dtsi, March 2026):
 *   usb2_host (dwc3):  reg 0xC0A00000, len 0x10000
 *   usb2_phy:          reg 0xC0A20000, len 0x200   (separate MMIO block,
 *                       unlike K1 where the USB2 PHY reset lived inside
 *                       the DWC3 wrapper at offset 0x10104)
 *   interrupt:         105, routed via saplic (K3's interrupt controller,
 *                       NOT plic as on K1) -- confirm the IRQ numbering
 *                       your saplic/plic glue in 9front actually uses.
 *
 * USER-SUPPLIED, NOT INDEPENDENTLY VERIFIED (no public source found for
 * these -- not in the upstream k3.dtsi fragment or any Linux patch I
 * could locate, so treat as a working hypothesis rather than confirmed
 * fact until cross-checked against the actual K3 TRM):
 *   AP PMU base:             0xD4282800 (matches K1's Pmubase, plausible)
 *   PMU_USB_CLK_RES_CTRL:    offset 0x5C -- ONE register shared by all
 *                            5 ports (USB2 host + USB3 A/B/C/D), each
 *                            with its own BUS_CLK_EN/PHY_RESETN/
 *                            VCC_RESETN/AHB_RSTN bits.
 *   PMU_USB_SD_ROT_WAKE_CLR: offset 0x7C -- wake/VBUS driver control
 *                            specifically for the USB2 host port.
 *
 * STILL MISSING / BLOCKING: the exact bit positions inside 0x5C that
 * belong to the USB2 port specifically, as opposed to ports A/B/C/D.
 * Since one register covers all five ports, guessing this wrong risks
 * touching USB3 port reset/clock/PHY-mux state (shared with PCIe combo
 * PHYs) as a side effect of bringing up USB2 alone. Until that bit map
 * is confirmed, Usb2busmask below is a PLACEHOLDER -- do not widen it
 * without the real per-port bit layout.
 *
 * Also, on the earlier worry about Secure/Machine-mode-only access to
 * 0xD4282800: the actual Linux clock driver for this SoC family
 * (drivers/clk/spacemit/ccu-k1.c, and ccu-k3.c which the upstream
 * "clk: spacemit: k3: add the clock tree" series describes as sharing
 * the same common CCU code) is an ordinary syscon/regmap-mmio driver --
 * plain reads/writes from kernel context, no SBI ecalls anywhere in
 * it. That's real merged kernel source, not a manual summary, and it's
 * reasonably strong evidence that direct vmap()+write from S-mode (the
 * same approach K1's driver uses) is in fact how this SoC expects the
 * AP PMU to be touched -- not something requiring Machine Mode or an
 * SBI vendor extension. Still worth a cautious first test on hardware
 * you don't mind hanging, but this is no longer treated as a likely
 * blocker.
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
    /* SoC K3 base addresses -- VERIFIED against upstream k3.dtsi */
    Xhcibase    = 0xC0A00000,    /* usb2_host DWC3 controller */
    Xhcilen     = 0x10000,
    Usb2phybase = 0xC0A20000,    /* usb2_phy, separate block on K3 */
    Usb2phylen  = 0x200,

    /*
     * AP PMU base -- USER-SUPPLIED, matches K1's Pmubase, plausible
     * but not independently confirmed by any source I could locate.
     */
    Apmubase    = 0xD4282800,
    Apmulen     = 0x1000,

    /*
     * PMU_USB_CLK_RES_CTRL: USER-SUPPLIED, but now with a specific and
     * internally-consistent per-port bit layout (still not cross-
     * checked by me against a public/primary source, so keep treating
     * it as "reported by the person I'm working with" rather than
     * independently confirmed):
     *
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
     *   bit 1  either AHB_RSTN or BUS_CLK_EN, depending which chapter
     *          of the manual you read -- the two chapters disagree
     *   bit 0  the other one of the above
     *
     * The bit0/bit1 swap doesn't matter for bring-up: reset value of
     * all four bits is 0, and writing 0xF to bits[3:0] sets clock
     * enable and de-asserts every reset for the port simultaneously,
     * regardless of which of bit0/bit1 is which. Do NOT rely on this
     * mask if you ever need to toggle AHB reset independently of the
     * bus clock (e.g. for a controlled reset sequence rather than a
     * one-shot bring-up) -- resolve the bit0/bit1 ambiguity first.
     */
    Usbclkres    = 0x5C/4,
    Usb2mask     = 0xF,        /* bits [3:0]: USB2 host clk+3 resets */
    Usb3amask    = 0xF<<4,     /* bits [7:4]: USB3 port A -- unused here */
    Usb3bmask    = 0xF<<8,     /* bits [11:8]: USB3 port B -- unused here */
    Usb3cmask    = 0xF<<12,    /* bits [15:12]: USB3 port C -- unused here */
    Usb3dmask    = 0xF<<16,    /* bits [19:16]: USB3 port D -- unused here */

    /*
     * PMU_USB_SD_ROT_WAKE_CLR: USER-SUPPLIED offset, wake/VBUS driver
     * control for the USB2 host port specifically. Bit layout within
     * it is STILL not given -- left declared but unused below.
     */
    Usbsdrotwakeclr = 0x7C/4,

    IrqUsb2     = 105,            /* usb2_host interrupt, per upstream DT */
};

/*
 * usb2phyinit - reset/bring-up for the standalone usb2_phy block.
 *
 * On K1 this touched a bit inside the DWC3 wrapper's own register
 * space (USB_CTL @ Xhcibase+0x10104). On K3 the PHY is its own MMIO
 * region entirely (Usb2phybase), so this needs its own vmap and its
 * own reset sequence -- shape is a guess pending real K3 PHY register
 * documentation. Treat every offset/bit here as UNVERIFIED.
 */
static void
usb2phyinit(u32int *phymmio)
{
    enum {
        /* UNVERIFIED: placeholder soft-reset control register/bit */
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
        GCTL             = 0xC110/4,
        GUCTL            = 0xC12C/4,
        GFLADJ           = 0xC630/4,
        PWRDNSCALE_SHIFT = 19,

        PWRDNSCALE_MASK  = 0x3FFF << PWRDNSCALE_SHIFT,
        Pwrdnscale_125m  = 0x2c,

        PRTCAPDIR_SHIFT  = 12,
        PRTCAPDIR_MASK   = 3 << PRTCAPDIR_SHIFT,
        DISSCRAMBLE      = 1<<3,
        DSBLCLKGTNG      = 1<<0,

        USBHSTINAUTORETRY = 1<<14,

        GFLADJ_30MHZ_MASK       = 0x3F,
        GFLADJ_30MHZ_SDBND_SEL  = 1<<7,
    };
    u32int v;

    /* 1. Global Control configuration (GCTL) */
    v = reg[GCTL];
    v &= ~(PWRDNSCALE_MASK | DISSCRAMBLE | DSBLCLKGTNG | PRTCAPDIR_MASK);
    v |= Pwrdnscale_125m<<PWRDNSCALE_SHIFT | 1<<PRTCAPDIR_SHIFT;
    reg[GCTL] = v;
    print("usbxhcirv64k3: gctl=%ux\n", reg[GCTL]);

    /* 2. Enable Automatic Retry of IN Transactions (GUCTL) */
    reg[GUCTL] |= USBHSTINAUTORETRY;

    /* 3. Frame Length Adjustment (GFLADJ) */
    reg[GFLADJ] = (reg[GFLADJ] & ~GFLADJ_30MHZ_MASK) | 0x20 | GFLADJ_30MHZ_SDBND_SEL;
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
     * on any other SoC (e.g. the K1, whose usbxhcirv64 handles USB
     * instead).  Without this gate the K3 addresses/PMU bits are poked
     * on the K1 -- the controller reads back as mmio=0 and nothing is
     * detected -- so bail before the first vmap/write.
     */
    if(!dtbsocmatch("spacemit,k3"))
        return -1;
    print("usbxhcirv64k3: reset called\n");

    /*
     * 1. APMU: enable clocks and release the controller's resets.
     *
     * Bit layout for the USB2 group (bits [3:0] of 0x5C) is reported
     * as internally consistent (see header) even though the bit0/bit1
     * roles are ambiguous between manual chapters -- writing all 4
     * bits together sidesteps that ambiguity for a one-shot bring-up.
     * Direct MMIO access to this register from S-mode is expected to
     * work based on how Linux's ccu-k1/ccu-k3 drivers touch the same
     * register family (see header) -- but this is still unverified
     * against actual K3 hardware, so treat the first test cautiously.
     */
    apmureg = vmap(Apmubase, Apmulen);
    if(apmureg != nil){
        print("usbxhcirv64k3: apmu usbclkres=%ux\n", apmureg[Usbclkres]);
        apmureg[Usbclkres] |= Usb2mask;
        vunmap(apmureg, Apmulen);
    } else {
        print("usbxhcirv64k3: vmap apmu failed, base=%ux len=%ux\n", Apmubase, Apmulen);
        return -1;
    }

    /* 2. Set up the Xhci structure and the DWC3 core */
    mmio = vmap(Xhcibase, Xhcilen);
    if(mmio == nil) {
        print("usbxhcirv64k3: vmap xhci failed\n");
        return -1;
    }
    ctlr = xhcialloc(mmio, Xhcibase, Xhcilen);
    if(ctlr == nil){
        print("usbxhcirv64k3: xhcialloc failed\n");
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
        print("usbxhcirv64k3: vmap usb2phy failed, base=%ux len=%ux\n",
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
usbxhcirv64k3link(void)
{
    addhcitype("xhci", reset);
}
