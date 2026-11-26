#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"

#define ECAMSIZE 0x01000000	/* map at most 16 buses */

typedef struct Intvec Intvec;

struct Intvec {
	Pcidev *p;
	void (*f)(Ureg*, void*);
	void *a;
};

static uchar *ecam;
static ulong ecamsize;
static uvlong mmiobase;
static Intvec vec[32];

static void*
cfgaddr(int tbdf, int rno)
{
	ulong off;

	off = BUSBNO(tbdf)<<20 | BUSDNO(tbdf)<<15 | BUSFNO(tbdf)<<12;
	if(ecam == nil || off >= ecamsize)
		return nil;
	return ecam + off + rno;
}

int
pcicfgrw32(int tbdf, int rno, int data, int read)
{
	u32int *p;

	if((p = cfgaddr(tbdf, rno & ~3)) != nil){
		if(read)
			data = *p;
		else
			*p = data;
	} else {
		data = -1;
	}
	return data;
}

int
pcicfgrw16(int tbdf, int rno, int data, int read)
{
	u16int *p;

	if((p = cfgaddr(tbdf, rno & ~1)) != nil){
		if(read)
			data = *p;
		else
			*p = data;
	} else {
		data = -1;
	}
	return data;
}

int
pcicfgrw8(int tbdf, int rno, int data, int read)
{
	u8int *p;

	if((p = cfgaddr(tbdf, rno)) != nil){
		if(read)
			data = *p;
		else
			*p = data;
	} else {
		data = -1;
	}
	return data;
}

static void
pciinterrupt(Ureg *ureg, void *)
{
	int i;
	Intvec *v;

	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->f)
			v->f(ureg, v->a);
	}
}

static void
pciintrinit(void)
{
	intrenable(IRQpci1, pciinterrupt, nil, BUSUNKNOWN, "pci");
	intrenable(IRQpci2, pciinterrupt, nil, BUSUNKNOWN, "pci");
	intrenable(IRQpci3, pciinterrupt, nil, BUSUNKNOWN, "pci");
	intrenable(IRQpci4, pciinterrupt, nil, BUSUNKNOWN, "pci");
}

void
pciintrenable(int tbdf, void (*f)(Ureg*, void*), void *a)
{
	Pcidev *p;
	int i;
	Intvec *v;

	if((p = pcimatchtbdf(tbdf)) == nil){
		print("pciintrenable: %T: unknown device\n", tbdf);
		return;
	}

	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->f == nil){
			v->p = p;
			v->f = f;
			v->a = a;
			return;
		}
	}
}

void
pciintrdisable(int tbdf, void (*f)(Ureg*, void*), void *a)
{
	Pcidev *p;
	int i;
	Intvec *v;

	if((p = pcimatchtbdf(tbdf)) == nil){
		print("pciintrdisable: %T: unknown device\n", tbdf);
		return;
	}

	for(i = 0; i < nelem(vec); i++){
		v = &vec[i];
		if(v->p == p && v->f == f && v->a == a)
			v->f = nil;
	}
}

static void
pcicfginit(void)
{
	char *p;
	Pcidev *pciroot;
	ulong ioa;
	uvlong base;

	fmtinstall('T', tbdffmt);

	pcimaxdno = 32;
	if(p = getconf("*pcimaxdno"))
		pcimaxdno = strtoul(p, 0, 0);

	pciscan(0, &pciroot, nil);
	if(pciroot == nil)
		return;

	pciintrinit();

	ioa = 0;
	base = mmiobase;
	pcibusmap(pciroot, &base, &ioa, 1);

	if(getconf("*pcihinv"))
		pcihinv(pciroot);
}

static uvlong
be8(uchar *p)
{
	return (uvlong)(p[0]<<24 | p[1]<<16 | p[2]<<8 | p[3])<<32
		| (u32int)(p[4]<<24 | p[5]<<16 | p[6]<<8 | p[7]);
}

/*
 * the 32-bit memory window from ranges: <space pci.hi pci.lo
 * cpu.hi cpu.lo size.hi size.lo>; bus and cpu addresses must agree
 */
static uvlong
mem32window(Dtbnode *n)
{
	uchar *p;

	for(p = n->ranges; p+28 <= n->ranges+n->rangeslen; p += 28)
		if((p[0] & 3) == 2 && be8(p+4) == be8(p+12) && be8(p+20) != 0)
			return be8(p+12);
	return 0;
}

/*
 * QEMU virt's host bridge; the INTx lines are virt's too
 */
void
pciqemulink(void)
{
	Dtbnode *n;

	n = dtbfind("pci-host-ecam-generic", 0);
	if(n == nil || !n->ok || n->base == 0)
		return;
	mmiobase = mem32window(n);
	if(mmiobase == 0){
		print("pci: %#llux: no 32-bit memory window\n", n->base);
		return;
	}
	ecamsize = n->size;
	if(ecamsize == 0 || ecamsize > ECAMSIZE)
		ecamsize = ECAMSIZE;
	ecam = vmap(n->base, ecamsize);
	if(ecam == nil)
		return;
	pcicfginit();
}
