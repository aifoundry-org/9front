/*
 * SpacemiT K3 Pico-ITX board devices, run from archinit, and
 * their device tree properties.  dpuk3 brings up DP1 and
 * reserves the framebuffer that screeninit hands to devdraw.
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
	k3ecfaninit();
	dpuk3();
	screeninit();
}

void
boarddtbprop(char *path, char *key, uchar *val, int len)
{
	dwqedtbprop(path, key, val, len);
	ufsdtbprop(path, key, val, len);
}
