/*
 * ET-SoC-1 Maxion real-time clock: the CLINT mtime counter plus an
 * epoch.  mtime is a free-running 64-bit count of the uncore clock
 * divided by 100 (about 9.33 MHz, the DT timebase), read here through
 * rdtime; only an uncore (cold) reset restarts it, so it survives
 * guest reboots, but there is no battery, calendar or non-volatile
 * RAM, and S-mode cannot write the counter.
 *
 * The epoch comes from the host: /chosen ainekko,rtc-sample pairs one
 * mtime value with the Unix time at which the host read it, and the
 * clock is extrapolated from there at the timebase.  At boot the time
 * of day is set from it.  Without a sample the clock counts from the
 * epoch since the last cold reset.  Writing rtc moves only the
 * in-kernel epoch, until the next boot.
 *
 * rtc reads and writes whole seconds, as on other machines.  Reading
 * rtcctl gives the state in full; writing "tod" to it sets the time
 * of day from the rtc to the nanosecond, undoing the truncation of
 * boot/bootrc's cat '#r/rtc' >/dev/time.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "maxion.h"

enum {
	Qdir = 0,
	Qrtc,
	Qctl,

	Sec = 1000000000LL,
	Maxsecs = 9000000000LL,	/* keeps secs*Sec in a vlong */
};

static Dirtab rtcdir[] = {
	".",		{Qdir, 0, QTDIR},	0,	0555,
	"rtc",		{Qrtc, 0},	0,	0664,
	"rtcctl",	{Qctl, 0},	0,	0664,
};

static struct {
	Lock;
	uvlong	mtime;		/* counter value at ns */
	vlong	ns;		/* Unix ns at mtime */
	uvlong	err;		/* uncertainty of ns, from the host */
	char	*src;		/* where ns came from */
	uvlong	bootmtime;	/* counter when the time of day was set */
	vlong	bootns;		/* and what it was set to; 0 if not */
} rtc;

/* counter ticks to ns without overflow for any 64-bit count */
static uvlong
ticks2ns(uvlong t, uvlong hz)
{
	return t/hz*Sec + t%hz*Sec/hz;
}

static vlong
rtcns(uvlong *mtp)
{
	uvlong now, hz;
	vlong ns;

	ilock(&rtc);
	now = fastticks(&hz);
	ns = rtc.ns;
	if(now > rtc.mtime)
		ns += ticks2ns(now - rtc.mtime, hz);
	iunlock(&rtc);
	if(mtp != nil)
		*mtp = now;
	return ns;
}

static void
rtcinit(void)
{
	Dtbrtc *d;
	uvlong now, hz, ago;
	vlong t;

	rtc.src = "cold reset";
	d = dtbrtc();
	if(d == nil){
		print("rtc: no host time sample; counting from the epoch since the cold reset\n");
		return;
	}
	now = fastticks(&hz);
	if(now < d->mtime){
		print("rtc: mtime %#llux is behind the host sample %#llux; counter reset since, sample ignored\n",
			now, d->mtime);
		return;
	}
	ago = ticks2ns(now - d->mtime, hz);
	t = d->ns + ago;
	ilock(&rtc);
	rtc.mtime = d->mtime;
	rtc.ns = d->ns;
	rtc.err = d->haserr? d->err: 0;
	rtc.src = "host";
	rtc.bootmtime = now;
	rtc.bootns = t;
	iunlock(&rtc);
	todset(t, 0, 0);
	print("rtc: host sample %lld.%09lld ±%lludµs at mtime %#llux, %llud.%03llud s ago; time of day set\n",
		d->ns/Sec, d->ns%Sec, rtc.err/1000, d->mtime, ago/Sec, ago%Sec/1000000);
}

static Chan*
rtcattach(char *spec)
{
	return devattach('r', spec);
}

static Walkqid*
rtcwalk(Chan *c, Chan *nc, char **name, int nname)
{
	return devwalk(c, nc, name, nname, rtcdir, nelem(rtcdir), devgen);
}

static int
rtcstat(Chan *c, uchar *dp, int n)
{
	return devstat(c, dp, n, rtcdir, nelem(rtcdir), devgen);
}

static Chan*
rtcopen(Chan *c, int omode)
{
	omode = openmode(omode);
	if((ulong)c->qid.path != Qdir && omode != OREAD && strcmp(up->user, eve) != 0)
		error(Eperm);
	return devopen(c, omode, rtcdir, nelem(rtcdir), devgen);
}

static void
rtcclose(Chan*)
{
}

static long
rtcread(Chan *c, void *a, long n, vlong off)
{
	char *buf, *p, *e, *src;
	uvlong now, hz, mt, err, bmt;
	vlong ns, epoch, bns;

	if(c->qid.type & QTDIR)
		return devdirread(c, a, n, rtcdir, nelem(rtcdir), devgen);

	switch((ulong)c->qid.path){
	case Qrtc:
		return readnum((ulong)off, a, n, (ulong)(rtcns(nil)/Sec), 12);
	case Qctl:
		ilock(&rtc);
		now = fastticks(&hz);
		mt = rtc.mtime;
		epoch = rtc.ns;
		err = rtc.err;
		src = rtc.src;
		bmt = rtc.bootmtime;
		bns = rtc.bootns;
		iunlock(&rtc);
		ns = epoch;
		if(now > mt)
			ns += ticks2ns(now - mt, hz);
		buf = smalloc(READSTR);
		if(waserror()){
			free(buf);
			nexterror();
		}
		e = buf + READSTR;
		p = seprint(buf, e, "source %s\n", src);
		p = seprint(p, e, "epoch %lld at mtime %llud\n", epoch, mt);
		p = seprint(p, e, "error %llud\n", err);
		p = seprint(p, e, "timebase %llud\n", hz);
		p = seprint(p, e, "mtime %llud\n", now);
		p = seprint(p, e, "rtc %lld\n", ns);
		seprint(p, e, "boot %lld at mtime %llud\n", bns, bmt);
		n = readstr((ulong)off, a, n, buf);
		poperror();
		free(buf);
		return n;
	}
	error(Egreg);
}

static long
rtcwrite(Chan *c, void *a, long n, vlong off)
{
	char buf[32], *p;
	uvlong now, hz;
	vlong secs, cur, tod;

	if(off != 0 || n >= sizeof(buf))
		error(Ebadarg);
	memmove(buf, a, n);
	buf[n] = 0;
	if((ulong)c->qid.path == Qctl){
		if(strcmp(buf, "tod") != 0 && strcmp(buf, "tod\n") != 0)
			error(Ebadctl);
		todset(rtcns(nil), 0, 0);
		return n;
	}
	if((ulong)c->qid.path != Qrtc)
		error(Eperm);
	for(p = buf; *p != 0; p++)
		if(*p >= '0' && *p <= '9')
			break;
	if(*p == 0)
		error(Ebadarg);
	secs = strtoll(p, nil, 10);
	if(secs < 0 || secs > Maxsecs)
		error(Ebadarg);

	/*
	 * keep the sub-second phase when the second is unchanged; else
	 * take the time of day's if it is in that second (aux/timesync
	 * writes rtc just after setting the time of day).
	 */
	todget(&tod, nil, nil, nil);
	ilock(&rtc);
	now = fastticks(&hz);
	cur = rtc.ns;
	if(now > rtc.mtime)
		cur += ticks2ns(now - rtc.mtime, hz);
	if(cur/Sec != secs){
		rtc.ns = tod/Sec == secs? tod: secs*Sec;
		rtc.mtime = now;
		rtc.err = 0;
		rtc.src = "write";
	}
	iunlock(&rtc);
	return n;
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
