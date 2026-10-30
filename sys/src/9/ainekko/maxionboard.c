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
#include "maxion.h"

static Dtbmxp mxpdt = { .abi = -1 };
static Dtbrtc rtcdt;

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

static u32int
beget4(uchar *p)
{
	return (u32int)p[0]<<24 | (u32int)p[1]<<16 | (u32int)p[2]<<8 | (u32int)p[3];
}

static int
ismxpnode(char *path)
{
	char *base;

	base = strrchr(path, '/');
	if(base == nil)
		return 0;
	return strncmp(base+1, "packet@", 7) == 0;
}

/*
 * The usable packet mailbox description, or nil.  Requires exactly one
 * enabled esperanto,maxion-packet-v1 node with a one-page reg and the
 * complete ABI v1 /chosen bootstrap; the endpoint core checks the rest.
 */
Dtbmxp*
dtbmxp(void)
{
	Dtbmxp *d;

	d = &mxpdt;
	if(d->nnodes != 1 || !d->compat || !d->hasreg || !d->ok
	|| !d->inited || d->abi != 1 || !d->hasfwid
	|| (d->fwid[0] | d->fwid[1] | d->fwid[2] | d->fwid[3]) == 0
	|| d->size != BY2PG || d->base == 0 || (d->base & (BY2PG-1)) != 0
	|| d->base >= (1ULL<<39))
		return nil;
	return d;
}

/*
 * The host time sample, or nil.  Requires exactly one well-formed
 * sample after 2001; whether the counter has since been reset is
 * devrtc's check, once the clock runs.
 */
Dtbrtc*
dtbrtc(void)
{
	Dtbrtc *r;

	r = &rtcdt;
	if(r->nsample != 1 || !r->ok || r->ns < 1000000000LL*1000000000LL)
		return nil;
	return r;
}

/* every DT property, from bootargsinit's walk; collect the mailbox's and the clock's */
void
boarddtprop(char *path, char *key, uchar *p, int len)
{
	Dtbmxp *d;
	int i, n;

	d = &mxpdt;
	if(strcmp(path, "/chosen") == 0){
		if(strcmp(key, "esperanto,mxpacket-page-initialized") == 0)
			d->inited = len == 0;
		else if(strcmp(key, "esperanto,mxpacket-abi") == 0)
			d->abi = len == 4? beget4(p): -1;
		else if(strcmp(key, "esperanto,mxpacket-fw-id") == 0 && len == 16){
			for(i = 0; i < 4; i++)
				d->fwid[i] = beget4(p + 4*i);
			d->hasfwid = 1;
		}else if(strcmp(key, "rng-seed") == 0 && len > 0){
			n = len;
			if(n > sizeof(d->seed))
				n = sizeof(d->seed);
			memmove(d->seed, p, n);
			d->nseed = n;
		}else if(strcmp(key, "ainekko,rtc-sample") == 0){
			rtcdt.nsample++;
			rtcdt.ok = len == 16;
			if(rtcdt.ok){
				rtcdt.mtime = (uvlong)beget4(p)<<32 | beget4(p+4);
				rtcdt.ns = (uvlong)beget4(p+8)<<32 | beget4(p+12);
			}
		}else if(strcmp(key, "ainekko,rtc-error-ns") == 0 && len == 4){
			rtcdt.err = beget4(p);
			rtcdt.haserr = 1;
		}
		return;
	}
	if(!ismxpnode(path))
		return;
	if(strcmp(d->path, path) != 0){
		/* a new packet node: a second one disables the device */
		d->nnodes++;
		n = strlen(path);
		if(n >= sizeof(d->path))
			n = sizeof(d->path) - 1;
		memmove(d->path, path, n);
		d->path[n] = 0;
		d->ok = 1;	/* DT default when status is absent */
	}
	if(strcmp(key, "compatible") == 0){
		d->compat = len == 27 && memcmp(p, "esperanto,maxion-packet-v1", 27) == 0;
	}else if(strcmp(key, "reg") == 0){
		/* root #address-cells = #size-cells = 2 on the Maxion tree */
		d->hasreg = len == 16;
		if(d->hasreg){
			d->base = (uvlong)beget4(p)<<32 | beget4(p+4);
			d->size = (uvlong)beget4(p+8)<<32 | beget4(p+12);
		}
	}else if(strcmp(key, "status") == 0){
		d->hasstatus = 1;
		d->ok = len == 5 && memcmp(p, "okay", 5) == 0;
	}
}
