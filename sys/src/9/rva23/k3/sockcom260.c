/*
 * SpacemiT K3 CoM260 on its IFX carrier board
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k3fns.h"

static void
com260init(void)
{
	k3init();
	i2ck3faninit();
	dpuk3com260();
}

Socmod sockcom260 = {
	"k3-com260-ifx",
	"spacemit,k3-com260-ifx",
	com260init,
};
