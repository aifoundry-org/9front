/*
 * The Maxion payload has no PCI host bridge.  In particular, the QEMU
 * ECAM address is a real Maxion mailbox, so it must never be probed.
 * Satisfy the shared interrupt dispatch API without linking a PCI scanner.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

void
boardinit(void)
{
	/* No SpacemiT GPIO, thermal, display or USB bring-up. */
}

void
pciintrenable(int, void (*)(Ureg*, void*), void*)
{
	panic("maxion: PCI interrupts unavailable");
}

void
pciintrdisable(int, void (*)(Ureg*, void*), void*)
{
}
