/*
 * apply the relocation table emitted by jl -u (see a.out(6)).
 *
 * code is entirely pc-relative, but initialized data holds absolute
 * pointers (devtab, systab, format tables, string tables, ...) baked
 * for the link address.  when the image runs elsewhere, each recorded
 * pointer needs the displacement added.
 *
 * the table follows the data segment in the image, so a loader that
 * loads the whole file (booti, qemu, kexec) leaves it at edata, where
 * it is overlaid by bss: this must run before bss is zeroed or used,
 * and before anything consults a pointer table (so no print here).
 * an ELF loader drops the table, but it also loads the kernel at its
 * linked address, so there is nothing to fix up.
 * linkstart is initialized data holding the linked address of _start
 * and is itself in the table; once fixed up, delta reads as 0, so a
 * second call is harmless.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

extern void _start(void);
extern char edata[];

static void *linkstart = (void*)_start;

/*
 * runtime memory layout (see mem.h): the kernel runs in place, so
 * these are set here from the running address of _start, before any
 * other C.  Initialized nonzero so they live in data, not bss — bss
 * is zeroed after they are set.
 */
uintptr ktzero = 1;
uintptr vdram = 1;

static ulong
beget(uchar *p, int n)
{
	ulong v;

	v = 0;
	while(n-- > 0)
		v = v<<8 | *p++;
	return v;
}

void
relocfixup(void)
{
	uchar *p, *ep;
	uintptr delta, wa;
	ulong size, n;
	int m, c;

	ktzero = (uintptr)_start;
	vdram = ktzero - 0x200000;

	delta = (uintptr)_start - (uintptr)linkstart;
	if(delta == 0)
		return;

	p = (uchar*)edata;
	size = beget(p, 4);
	ep = p + 4 + size;
	p += 4;
	if(beget(p, 4) != 0)		/* imports: none expected */
		return;
	p += 4;
	n = beget(p, 4);
	p += 4;

	/* validate before touching anything */
	if(n > size/2)
		return;
	while(n-- > 0){
		if(p >= ep || (*p & 0xF) != 0)
			return;
		p += 1 + (1 << (*p>>6));
	}
	if(p != ep)
		return;

	p = (uchar*)edata + 12;
	n = beget((uchar*)edata + 8, 4);
	wa = (uintptr)_start;
	while(n-- > 0){
		m = *p++;
		c = 1 << (m>>6);
		wa += (uintptr)beget(p, c) << 2;
		p += c;
		*(uvlong*)wa += delta;
	}
}
