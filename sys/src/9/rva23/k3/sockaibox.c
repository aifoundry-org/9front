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

static void
aiboxinit(void)
{
	k3init();
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
