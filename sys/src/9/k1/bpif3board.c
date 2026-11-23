/*
 * Banana Pi BPI-F3 board devices, run from archinit.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
boardinit(void)
{
	k1thermalinit();
}
