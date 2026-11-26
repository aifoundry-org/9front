/*
 * SpacemiT K1: BPI-F3 and kin
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k1fns.h"

static void
k1init(void)
{
	k1thermalinit();
}

Socmod sock1 = {
	"k1",
	"spacemit,k1",
	k1init,
	k1reset,
	k1poweroff,
};
