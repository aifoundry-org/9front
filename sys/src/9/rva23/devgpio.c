/*
 * #G/gpio: named GPIO pins over the gpio.c API, in the style of
 * bcm/devgpio.c.  The SoC driver registers the controller with
 * addgpio and the board's pin names with gpionames.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "io.h"

enum {
	PinBits		= 7,
	PinMask		= (1<<PinBits)-1,
	Ngpiopin	= 1<<PinBits,
	Nbankwords	= Ngpiopin/32,	/* eventvalue size in u32ints */
};

#define PIN_OFFSET	(PARENT_OFFSET + PARENT_BITS)
#define PIN_NUMBER(q)	(((q).path >> PIN_OFFSET) & PinMask)

#define PARENT_OFFSET	(FILE_OFFSET + FILE_BITS)
#define PARENT_BITS	3
#define PARENT_MASK	((1 << PARENT_BITS) - 1)
#define PARENT_TYPE(q)	(((q).path >> PARENT_OFFSET) & PARENT_MASK)

#define FILE_OFFSET	0
#define FILE_BITS	3
#define FILE_MASK	((1 << FILE_BITS) - 1)
#define FILE_TYPE(q)	(((q).path >> FILE_OFFSET) & FILE_MASK)

/* pin is valid only when file==Qdata, otherwise 0 */
#define PATH(pin, parent, file) \
	(((uvlong)(pin) & PinMask) << PIN_OFFSET) \
	| (((uvlong)(parent) & PARENT_MASK) << PARENT_OFFSET) \
	| (((uvlong)(file) & FILE_MASK) << FILE_OFFSET)

#define SET_BIT(f, pin, value) \
	((f)[(pin)/32] = ((f)[(pin)/32] & ~(1UL<<((pin)%32))) | ((ulong)(value)<<((pin)%32)))
#define GET_BIT(f, pin) \
	(((f)[(pin)/32] >> ((pin)%32)) & 1)

enum {
	/* parent types */
	Qtopdir = 0,
	Qgpiodir,
	/* file types */
	Qdir,
	Qdata,
	Qctl,
	Qevent,
};

/* commands: CMzero/CMone are written straight to a pin's Qdata file,
 * CMevent goes to Qctl -- exactly like bcm's shared gpiocmd[]. */
enum {
	CMzero,
	CMone,
	CMevent,
};

static Cmdtab gpiocmd[] = {
	CMzero,	"0",		1,
	CMone,	"1",		1,
	CMevent,"event",	4,	/* event <edge> <enable|disable> <pinname> */
};

/* dat.h's GpioRising/GpioFalling/GpioEdge, in the order evtypename[] names them */
static char *evtypename[] = {
	"edge-rising", "edge-falling", "edge-both",
};
static int evmode[] = {
	GpioRising, GpioFalling, GpioEdge,
};

Dirtab topdir = { "#G", {PATH(0, Qtopdir, Qdir), 0, QTDIR}, 0, 0555 };
Dirtab gpiodir = { "gpio", {PATH(0, Qgpiodir, Qdir), 0, QTDIR}, 0, 0555 };

static Dirtab typedir[] = {
	"ctl",		{ PATH(0, Qgpiodir, Qctl), 0, QTFILE }, 0, 0666,
	"event",	{ PATH(0, Qgpiodir, Qevent), 0, QTFILE }, 0, 0444,
};

static char *gpiotable[Ngpiopin];	/* filled in by gpionames() */

static Rendez rend;
static u32int eventvalue[Nbankwords];
static long eventinuse;
static Lock eventlock;

static void
mkdeventry(Chan *c, Qid qid, Dirtab *tab, Dir *db)
{
	mkqid(&qid, tab->qid.path, tab->qid.vers, tab->qid.type);
	devdir(c, qid, tab->name, tab->length, eve, tab->perm, db);
}

static int
gpiogen(Chan *c, char*, Dirtab*, int, int s, Dir *db)
{
	Qid qid;
	int parent, l;

	qid.vers = 0;
	parent = PARENT_TYPE(c->qid);

	if(s == DEVDOTDOT){
		switch(parent){
		case Qtopdir:
		case Qgpiodir:
			mkdeventry(c, qid, &topdir, db);
			break;
		default:
			return -1;
		}
		return 1;
	}

	if(parent == Qtopdir){
		switch(s){
		case 0:
			mkdeventry(c, qid, &gpiodir, db);
			break;
		default:
			return -1;
		}
		return 1;
	}

	if(parent == Qgpiodir){
		l = nelem(typedir);
		if(s < l){
			mkdeventry(c, qid, &typedir[s], db);
		} else if(s < l+Ngpiopin){
			s -= l;
			if(gpiotable[s] == nil)
				return 0;
			mkqid(&qid, PATH(s, Qgpiodir, Qdata), 0, QTFILE);
			devdir(c, qid, gpiotable[s], 0, eve, 0666, db);
		} else
			return -1;
		return 1;
	}

	return 1;
}

/* gpiointrenable()'s per-pin callback: mark the bit and wake any reader. */
static void
gpioevent(uint pin, void*)
{
	SET_BIT(eventvalue, pin, 1);
	wakeup(&rend);
}

/* name the board's pins; replaces any earlier names */
void
gpionames(Gpiopin *p, int n)
{
	int i;

	memset(gpiotable, 0, sizeof gpiotable);
	for(i = 0; i < n; i++)
		if(p[i].pin >= 0 && p[i].pin < Ngpiopin)
			gpiotable[p[i].pin] = p[i].name;
}

static void
gpioshutdown(void)
{
}

static Chan*
gpioattach(char *spec)
{
	return devattach('G', spec);
}

static Walkqid*
gpiowalk(Chan *c, Chan *nc, char **name, int nname)
{
	return devwalk(c, nc, name, nname, 0, 0, gpiogen);
}

static int
gpiostat(Chan *c, uchar *db, int n)
{
	return devstat(c, db, n, 0, 0, gpiogen);
}

static Chan*
gpioopen(Chan *c, int omode)
{
	c = devopen(c, omode, 0, 0, gpiogen);

	switch(FILE_TYPE(c->qid)){
	case Qdata:
		c->iounit = 1;
		break;
	case Qevent:
		lock(&eventlock);
		if(eventinuse != 0){
			c->flag &= ~COPEN;
			unlock(&eventlock);
			error(Einuse);
		}
		eventinuse = 1;
		unlock(&eventlock);
		memset(eventvalue, 0, sizeof eventvalue);
		c->iounit = sizeof eventvalue;
		break;
	}

	return c;
}

static void
gpioclose(Chan *c)
{
	if(FILE_TYPE(c->qid) == Qevent && (c->flag & COPEN))
		eventinuse = 0;
}

static int
isset(void*)
{
	int i;

	for(i = 0; i < Nbankwords; i++)
		if(eventvalue[i])
			return 1;
	return 0;
}

static long
gpioread(Chan *c, void *va, long n, vlong off)
{
	int type;
	uint pin;
	char *a;
	int v;

	a = va;

	if(c->qid.type & QTDIR)
		return devdirread(c, va, n, 0, 0, gpiogen);

	type = FILE_TYPE(c->qid);

	switch(type){
	case Qdata:
		/* one byte, then EOF */
		if(off > 0){
			n = 0;
			break;
		}
		pin = PIN_NUMBER(c->qid);
		v = gpiopeek(pin);
		if(v < 0)
			error(Eio);	/* no controller */
		a[0] = v? '1': '0';
		n = 1;
		break;
	case Qctl:
		n = 0;
		break;
	case Qevent:
		if(off >= sizeof eventvalue){
			off %= sizeof eventvalue;
			memset(eventvalue, 0, sizeof eventvalue);
		}
		sleep(&rend, isset, 0);
		if(off+n > sizeof eventvalue)
			n = sizeof eventvalue - off;
		memmove(a, (uchar*)eventvalue+off, n);
		break;
	}

	return n;
}

static int
getpin(char *pinname)
{
	int i;

	for(i = 0; i < Ngpiopin; i++)
		if(gpiotable[i] != nil && strcmp(gpiotable[i], pinname) == 0)
			return i;
	return -1;
}

static long
gpiowrite(Chan *c, void *va, long n, vlong)
{
	int type, i, mode;
	uint pin;
	char *arg;
	Cmdbuf *cb;
	Cmdtab *ct;

	if(c->qid.type & QTDIR)
		error(Eisdir);

	type = FILE_TYPE(c->qid);

	cb = parsecmd(va, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	ct = lookupcmd(cb, gpiocmd, nelem(gpiocmd));
	if(ct == nil)
		error(Ebadctl);

	switch(type){
	case Qdata:
		pin = PIN_NUMBER(c->qid);
		switch(ct->index){
		case CMzero:
			gpioout(pin, 0);
			break;
		case CMone:
			gpioout(pin, 1);
			break;
		default:
			error(Ebadctl);
		}
		break;
	case Qctl:
		switch(ct->index){
		case CMevent:
			mode = getpin(cb->f[3]);
			if(mode == -1)
				error(Ebadctl);
			pin = mode;
			arg = cb->f[1];
			mode = -1;
			for(i = 0; i < nelem(evtypename); i++)
				if(strcmp(evtypename[i], arg) == 0){
					mode = evmode[i];
					break;
				}
			if(mode == -1)
				error(Ebadctl);
			if(cb->f[2][0] == 'e')
				gpiointrenable(pin, mode, gpioevent, nil);
			else if(cb->f[2][0] == 'd')
				gpiointrdisable(pin);
			else
				error(Ebadctl);
			break;
		default:
			error(Ebadctl);
		}
		break;
	default:
		error(Ebadctl);
	}

	free(cb);
	poperror();
	return n;
}

Dev gpiodevtab = {
	'G',
	"gpio",

	devreset,
	devinit,
	gpioshutdown,
	gpioattach,
	gpiowalk,
	gpiostat,
	gpioopen,
	devcreate,
	gpioclose,
	gpioread,
	devbread,
	gpiowrite,
	devbwrite,
	devremove,
	devwstat,
};
