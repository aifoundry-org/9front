/*
 * SpacemiT K3 Pico-ITX board
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k3fns.h"

/*
 * PIN TABLE SOURCE AND SCOPE -- PICO-ITX
 * -----------------------------------------
 * Every {pin,name} entry in picoitxpins[] below comes from the
 * schematic's "GPIO ASSIGNMENT" sheet (page 10) and the
 * SOC-GPIO-1..5 sheets (pages 12-16), which agree with each other on
 * every entry used here. The "GPIO[x]_NN" label's NN suffix *is* the
 * flat 0..127 pin number GPIO_PIN() expects
 * (bank=NN/32); the "[x]" prefix is just the vendor ballout table's
 * own pad-group label, unrelated to the register bank.
 *
 * Included: signals wired as a plain control/status/wake/interrupt
 * line (module enable/reset, CC/HPD/WAKE interrupts, PCIe PERST#/
 * CLKREQ#/WAKE#, eDP power/backlight enable, EC handshake lines).
 *
 * Excluded, on purpose:
 *   - Any net with an "R-" prefix (R-GMAC3_*, R-CAN0_*, R-I2C1_*,
 *     ...): those are R.GPIO[n] pins, a SEPARATE RCPU-owned GPIO
 *     controller this driver never touches (see the header on
 *     APMASK_REG vs CPMASK_REG).
 *   - Dedicated-function pins with no plain-GPIO use on this board:
 *     RGMII (GMAC0/2/3), LPDDR5, eSPI0 data/clk/cs lines, MIPI/eDP/
 *     UFS/QSPI lanes, and JTAG/boot-strap pins (Strap0-5, PRI_T*),
 *     which are sampled once at reset, not runtime GPIO.
 *   - CPU_EC_SWDIO/CPU_EC_SWCLK (pins 82/83) are the one exception:
 *     they carry the EC's SWD debug port, not a general-purpose
 *     signal, but the schematic does route them through plain
 *     GPIO[4]_82/83 pads, so they're listed for anyone who wants to
 *     bit-bang SWD before EC firmware exists.
 */

/*
 * pin                       name                     schematic sheet(s)
 */
static Gpiopin picoitxpins[] = {
{18,	"MODULE_TURN_ON_1V8"},		/* 10,12,37: WWAN module power-on, out */
{19,	"WWAN_RESET#_1V8"},		/* 10,12,37: WWAN module reset, out, active-low */
{23,	"SOC_eDP0_HPD_3V3"},		/* 10,19,46: eDP0 hot-plug detect, in */
{24,	"DP1_HPD_3V3"},			/* 10,19: DP1 hot-plug detect, in */
{25,	"PCIE2_PERSTN_3V3_CN"},		/* 10,37: PCIe2 (M.2 B-key/LTE) PERST#, out */
{26,	"BT_WAKE_HOST_3V3"},		/* 10,47: Bluetooth wake host, in */
{27,	"PCIE2_CLKREQN_33_CN"},		/* 10,37: PCIe2 CLKREQ#, in */
{30,	"BT_EN_3V3"},			/* 10,47: Bluetooth enable, out */
{35,	"WLAN_EN_3V3"},			/* 10,47: WLAN enable, out */
{41,	"HOST_WAKE_BT_3V3"},		/* 10,47: host wake BT, out */
{42,	"EC_CPU_ALLPG"},		/* 10,43: EC all-power-good, in */
{43,	"EC_CPU_INT_N"},		/* 10,43: EC interrupt, active low, in */
{76,	"PCIE4_PERSTN"},		/* 10,47: PCIe4 (WiFi) PERST#, out */
{77,	"PCIE4_WAKEN"},			/* 10,47: PCIe4 WAKE#, in */
{78,	"PCIE4_CLKREQN"},		/* 10,47: PCIe4 CLKREQ#, in */
{79,	"PCIE0_PERSTN"},		/* 10,38: PCIe0 (M.2 M-key SSD) PERST#, out */
{80,	"SOC_EC_RST"},			/* 10,43: reset to EC, out */
{81,	"PCIE0_CLKREQN"},		/* 10,38: PCIe0 CLKREQ#, in */
{82,	"CPU_EC_SWDIO"},		/* 10,43: EC SWD data -- see header note */
{83,	"CPU_EC_SWCLK"},		/* 10,43: EC SWD clock -- see header note */
{84,	"CPU_W_DISABLE#1"},		/* 10,37: M.2 W_DISABLE# (radio kill), out */
{85,	"CC_INT_N"},			/* 10,34: USB-C CC ctlr (FUSB301) interrupt, in */
{86,	"USB30_DRD_DIR"},		/* 10,34: USB3 DRD direction select, out */
{88,	"PCIE1_PERSTN"},		/* 10,37: PCIe1 (M.2 A/B x2) PERST#, out */
{89,	"MUX_CPU_M2B2"},		/* 10,36: PCIe1 2:4 mux select (B-key path), out */
{90,	"PCIE1_CLKREQN"},		/* 10,37: PCIe1 CLKREQ#, in */
{101,	"SOC_eDP0_PWR-EN_3V3"},		/* 10,46: eDP panel power enable, out */
{106,	"SOC_eDP0_BL-PWM_3V3"},		/* 10,46: eDP backlight PWM/enable, out */
{118,	"SOC_eDP0_BL-EN_3V3"},		/* 10,46: eDP backlight enable, out */
};

static void
picoitxinit(void)
{
	k3init();
	gpionames(picoitxpins, nelem(picoitxpins));
	k3ecfaninit();
	dpuk3();
}

Socmod sockpicoitx = {
	"k3-pico-itx",
	"spacemit,k3-pico-itx",
	picoitxinit,
};
