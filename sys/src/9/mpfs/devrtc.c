/*
 *  PolarFire SoC MSS RTC, in binary mode: a 1 Hz seconds counter
 */

#include	"u.h"
#include	"../port/lib.h"
#include	"mem.h"
#include	"dat.h"
#include	"fns.h"
#include	"../port/error.h"

enum {
	Rtcbase		= 0x20124000,
	Prescale	= 1000000-1,	/* hardware adds 1; rtcref is 1 MHz */

	Control		= 0x00/4,
	Mode		= 0x04/4,
	Prescaler	= 0x08/4,
	Datelo		= 0x20/4,
	Datehi		= 0x24/4,

	/* Control */
	Counting	= 1<<0,	/* read */
	Start		= 1<<0,	/* write */
	Stop		= 1<<1,
	Upload		= 1<<5,

	/* Mode */
	Calendar	= 1<<0,

	Himask		= (1<<30)-1,
};

enum{
	Qdir = 0,
	Qrtc,
};

Dirtab rtcdir[]={
	".",	{Qdir, 0, QTDIR},	0,	0555,
	"rtc",	{Qrtc, 0},		0,	0664,
};

static u32int *regs;
static Lock rtclock;

static void
rtcinit(void)
{
	regs = vmap(Rtcbase, 0x1000);
	if(regs == nil)
		return;
	if((regs[Control] & Counting) == 0){
		regs[Mode] &= ~Calendar;
		regs[Prescaler] = Prescale;
	}
}

static Chan*
rtcattach(char* spec)
{
	if(regs == nil)
		error(Enonexist);
	return devattach('r', spec);
}

static Walkqid*
rtcwalk(Chan* c, Chan *nc, char** name, int nname)
{
	return devwalk(c, nc, name, nname, rtcdir, nelem(rtcdir), devgen);
}

static int
rtcstat(Chan* c, uchar* dp, int n)
{
	return devstat(c, dp, n, rtcdir, nelem(rtcdir), devgen);
}

static Chan*
rtcopen(Chan* c, int omode)
{
	omode = openmode(omode);
	switch((ulong)c->qid.path){
	case Qrtc:
		if(strcmp(up->user, eve)!=0 && omode!=OREAD)
			error(Eperm);
		break;
	}
	return devopen(c, omode, rtcdir, nelem(rtcdir), devgen);
}

static void
rtcclose(Chan*)
{
}

long
rtctime(void)
{
	u32int hi, lo;

	if((regs[Control] & Counting) == 0)
		return 0;
	/* retry if the low word carried into the high one */
	do {
		hi = regs[Datehi];
		lo = regs[Datelo];
	} while(regs[Datehi] != hi);
	return (uvlong)(hi & Himask)<<32 | lo;
}

static void
setrtc(uvlong secs)
{
	int i;

	ilock(&rtclock);
	regs[Datelo] = secs;
	regs[Datehi] = secs>>32 & Himask;
	regs[Control] = (regs[Control] & ~Stop) | Upload;
	for(i = 0; i < 1000 && (regs[Control] & Upload) != 0; i++)
		microdelay(1);
	regs[Control] = (regs[Control] & ~Stop) | Start;
	iunlock(&rtclock);
	if(i == 1000)
		error("rtc upload timed out");
}

static long
rtcread(Chan* c, void* buf, long n, vlong off)
{
	if(c->qid.type & QTDIR)
		return devdirread(c, buf, n, rtcdir, nelem(rtcdir), devgen);

	switch((ulong)c->qid.path){
	case Qrtc:
		return readnum((ulong)off, buf, n, rtctime(), 12);
	}
	error(Ebadarg);
}

static long
rtcwrite(Chan* c, void* buf, long n, vlong off)
{
	char *cp, sbuf[32];

	switch((ulong)c->qid.path){
	case Qrtc:
		if(off != 0 || n >= sizeof(sbuf)-1)
			error(Ebadarg);
		memmove(sbuf, buf, n);
		sbuf[n] = '\0';
		for(cp = sbuf; *cp != '\0'; cp++)
			if(*cp >= '0' && *cp <= '9')
				break;
		setrtc(strtoull(cp, 0, 0));
		return n;
	}
	error(Egreg);
}

Dev rtcdevtab = {
	'r',
	"rtc",

	devreset,
	rtcinit,
	devshutdown,
	rtcattach,
	rtcwalk,
	rtcstat,
	rtcopen,
	devcreate,
	rtcclose,
	rtcread,
	devbread,
	rtcwrite,
	devbwrite,
	devremove,
	devwstat,
};
