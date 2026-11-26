/*
 * SpacemiT K3 CoM260 on its IFX carrier board
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k3fns.h"

/*
 * PIN TABLE SOURCE AND SCOPE -- K3-CoM260 -- MUCH THINNER, AND WHY
 * --------------------------------------------------------------
 * Unlike the pico-itx schematic, the CoM260 PDF's own "GPIO
 * ASSIGNMENT" sheet (page 8) carries no extractable pin-table text --
 * title only, no data (presumably an image/graphic table that didn't
 * OCR). Without that sheet, there is no page that pairs a net name
 * directly with its GPIO[x]_NN the way pico-itx's page 10 did.
 *
 * What IS available is the SODIMM edge connector's own pinout (pages
 * 9-10, connectors J2A/J2B), which lists every
 * "GPIO[x]_NN { alt-func-1 / alt-func-2 / ... } <connector-pin>"
 * entry -- the K3 ballout's own alternate-function table. Each entry
 * in com260pins[] below was accepted ONLY if:
 *
 *   1. its net name appears as a literal alternate-function string
 *      inside exactly ONE GPIO[x]_NN entry across all of J2A/J2B --
 *      if the same name shows up as an alt-func of two or more
 *      different pins (this happens a lot: I2C0_SCL, PCIeA_PERSTn,
 *      PCIeA_WAKEn, PCIeD_CLKREQn, PCIeD_PERSTn, UART6_TXD/RXD, ...
 *      all appear more than once), there's no way from this text
 *      alone to know which physical pin the board actually muxed to
 *      that function, so it's left out rather than guessed at;
 *
 *   2. it isn't a generic multiplexed bus (I2C0/I2C1/I2C3/I2C4/I2C5,
 *      UART1/UART6, SPI0/R-SPI0, MMC2, I2S0, CAN) -- those are
 *      carrier-passthrough buses on the 40-pin GPIO Expansion Header
 *      and SODIMM connector, properly the domain of the uart/i2c/
 *      mmc drivers, not plain GPIO, same exclusion rule applied to
 *      pico-itx's eSPI/RGMII/etc above;
 *
 *   3. it isn't one of the BMCU_* / PMIC_RST_OUTn/SHUTDOWN_REQ/
 *      POWER_EN/SLEEP-WAKE/MODULE_ID family -- those go to the
 *      on-module power-sequencing MCU (EFM8SB10F2G, U1402, see the
 *      "MCU DEBUG" sheet), not to a K3 GPIO[x]_NN pin at all, so
 *      gpiok3.c has nothing to drive there regardless.
 *
 * That leaves a short list: two PCIe PERST# lines and three PCIe
 * CLKREQ# lines, each confirmed unique. This table is intentionally
 * conservative -- it's very likely the real "GPIO ASSIGNMENT" sheet
 * has more, but guessing at ambiguous matches would be worse than
 * listing fewer, correct ones.
 */

/*
 * pin                   name                    schematic sheet(s), notes
 *
 * Every one of these appears as an alternate-function name inside
 * exactly one GPIO[x]_NN entry on connectors J2A/J2B (pages 9-10) --
 * see above for why so few entries met that bar.
 */
static Gpiopin com260pins[] = {
{44,	"PCIeA_CLKREQn"},	/* 9,22: GPIO[3]_44, M.2 Key-M[A] NVMe CLKREQ#, in */
{58,	"PCIeC_PERSTn"},	/* 9,23: GPIO[3]_58, M.2 Key-M[B] NVMe PERST#, out */
{60,	"PCIeC_CLKREQn"},	/* 9,21,23: GPIO[3]_60, M.2 Key-M[B]/Key-E CLKREQ#, in */
{76,	"PCIeE_PERSTn"},	/* 10,21: GPIO[4]_76, M.2 Key-E (WiFi/BT) PERST#, out */
{78,	"PCIeE_CLKREQn"},	/* 10,21: GPIO[4]_78, M.2 Key-E (WiFi/BT) CLKREQ#, in */
};

static void
com260init(void)
{
	k3init();
	gpionames(com260pins, nelem(com260pins));
	i2ck3faninit();
	dpuk3com260();
}

Socmod sockcom260 = {
	"k3-com260-ifx",
	"spacemit,k3-com260-ifx",
	com260init,
};
