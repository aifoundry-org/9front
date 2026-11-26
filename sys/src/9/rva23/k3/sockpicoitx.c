/*
 * SpacemiT K3 Pico-ITX board
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "k3fns.h"

static void
picoitxinit(void)
{
	k3init();
	k3ecfaninit();
	dpuk3();
}

Socmod sockpicoitx = {
	"k3-pico-itx",
	"spacemit,k3-pico-itx",
	picoitxinit,
};
