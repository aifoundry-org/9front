#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

/*
 * Board device setup, run from archinit, and the device tree
 * properties of the board's devices, from bootargsinit's walk of
 * the tree.  The generic kernel has none; a SoC directory supplies
 * its own.
 */
void
boardinit(void)
{
}

void
boarddtbprop(char*, char*, uchar*, int)
{
}
