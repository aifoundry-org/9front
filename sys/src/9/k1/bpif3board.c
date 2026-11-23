/*
 * Banana Pi BPI-F3 board devices, run from archinit, and their
 * device tree properties.
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

void
boarddtbprop(char *path, char *key, uchar *val, int len)
{
	smtedtbprop(path, key, val, len);
}
