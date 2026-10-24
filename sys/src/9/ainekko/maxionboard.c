/*
 * Esperanto ET-SoC-1 Maxion board hooks for the generic riscv64 kernel.
 *
 * OpenSBI starts hart 0 of the Maxion cluster in an owned, 1 GiB
 * aligned RAM window at 0x8600000000 with the kernel and a device tree
 * there.  The harts have Sv48 but neither Sstc, Zicbom nor Svpbmt, and
 * DMA is not coherent; this port is a uniprocessor with no PCI, storage
 * or DMA devices.  In particular, the QEMU ECAM address is a real
 * Maxion mailbox, so no PCI scanner is ever linked.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

/*
 * Called once DT and bootargs are parsed, before meminit.  Fail
 * closed: only an ET-SoC-1 tree (or *maxion=1, for QEMU tests) and an
 * explicit, aligned, at most 1 GiB loader window will do.  All kernel
 * data stays in that window; it is not a host reservation.
 */
void
boardearly(void)
{
	extern void _start(void);
	uintptr l;
	char *p;

	if(!dtbsocmatch("esperanto,et-soc-1") && getconf("*maxion") == nil)
		panic("maxion: not an ET-SoC-1 device tree");
	print("maxion: physical entry %#p, embedded RAM-root kernel\n", (uintptr)_start);

	l = VDRAM + 2ULL * GiB;
	if(p = getconf("*maxmem"))
		l = strtoull(p, 0, 0);
	if((VDRAM & (GiB-1ULL)) != 0 || l <= VDRAM ||
	   l - VDRAM > GiB || (l & (GiB-1ULL)) != 0)
		panic("maxion: require aligned 1 GiB RAM window");
	kbase = VDRAM;
	klimit = l;

	soc.sbitimer = 1;
	soc.nocbo = 1;
	soc.uartfifo = 1;
	soc.uartpollms = 10;
	soc.maxmach = 1;
	soc.cpuname = "ET-SoC-1 Maxion, SBI timer, uniprocessor";
}

void
boardinit(void)
{
	/* No SpacemiT GPIO, thermal, display or USB bring-up. */
}

/* every DT property, from bootargsinit's walk; none is needed yet */
void
boarddtprop(char*, char*, uchar*, int)
{
}
