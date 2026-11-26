/*
 * Firefly AIBOX-K3: a K3 CoM260 on Firefly's carrier, which keeps the
 * CoM260 compatible and is told apart by the root model
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k3fns.h"

enum {
	Pwroffpin	= 88,		/* driving it low cuts power */
};

/*
 * front-panel power and boot buttons, the three gpio-leds, and the
 * poweroff request to the PMIC.  Buttons are active low; the LEDs
 * are active high.
 */
static Gpiopin aiboxpins[] = {
{56,	"PWR_LED"},		/* front power LED, out */
{57,	"USER_LED"},		/* front work LED, out */
{69,	"BOOTKEY"},		/* boot button, in, active low */
{84,	"PWRKEY"},		/* power button, in, active low */
{Pwroffpin,	"PWROFF"},	/* drive low to cut power */
{103,	"RED_LED"},		/* front red LED, out */
};

static void
aiboxinit(void)
{
	k3init();
	gpionames(aiboxpins, nelem(aiboxpins));
	i2ck3faninit();
}

/* the PMIC takes a few seconds to cut power */
static void
aiboxpoweroff(void)
{
	gpioout(Pwroffpin, 0);
	delay(3000);
}

Socmod sockaibox = {
	"aibox-k3",
	"spacemit,k3-com260",
	aiboxinit,
	nil,
	aiboxpoweroff,
	"AIBOX",
};
