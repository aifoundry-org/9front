/*
 * SpacemiT K3, common to its boards
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k3fns.h"

void
k3init(void)
{
	gpiok3init();
	k3thermalinit();
}

Socmod sock3 = {
	"k3",
	"spacemit,k3",
	k3init,
};
