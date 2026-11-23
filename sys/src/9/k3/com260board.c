/*
 * SpacemiT K3 CoM260 on the IFX carrier: board devices, run
 * from archinit.  dpuk3com260 brings up DP1 on the carrier's
 * DisplayPort connector, in the framebuffer that screenpicoitx.c's
 * dpuk3fbaddr reserves, and screeninit hands it to devdraw.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
boardinit(void)
{
	gpiok3init();
	k3thermalinit();
	dpuk3com260();
	screeninit();
}
