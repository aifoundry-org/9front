/*
 * The ET-SoC-1 host's device tree bindings: the packet mailbox page
 * and its bootstrap in the packet@ node and /chosen, and the time
 * sample in /chosen.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "maxion.h"

static Dtbmxp mxp;
static Dtbrtc rtc;

enum {
	DtHeader	= 0xd00dfeed,
	DtBeginNode	= 1,
	DtEndNode	= 2,
	DtProp		= 3,
	DtNop		= 4,
	DtEnd		= 9,

	Maxchosen	= 16,
};

typedef struct Prop Prop;
struct Prop {
	char	*name;
	uchar	*val;
	int	len;
};

static Prop chosenprops[Maxchosen];
static int nchosen;
static int npacket;

static u32int
beget4(uchar *p)
{
	return (u32int)p[0]<<24 | (u32int)p[1]<<16 | (u32int)p[2]<<8 | (u32int)p[3];
}

/*
 * Collect the /chosen properties and count the packet nodes, straight
 * from the firmware's tree, which stays reachable until kmapram
 * switches to Sv48.
 */
static void
dtscan(void)
{
	extern uintptr dtbphys;
	uchar *d, *p, *e, *v;
	char *strs, *name;
	int depth, chosen, len;

	d = (uchar*)dtbphys;
	if(dtbphys == 0 || dtbphys == ~(uintptr)0 || beget4(d) != DtHeader)
		return;
	p = d + beget4(d+8);
	e = d + beget4(d+4);
	strs = (char*)d + beget4(d+12);
	depth = 0;
	chosen = 0;
	while(p+4 <= e){
		switch(beget4(p)){
		case DtBeginNode:
			name = (char*)p+4;
			depth++;
			if(depth == 2)
				chosen = strcmp(name, "chosen") == 0;
			p += 4 + ROUND(strlen(name)+1, 4);
			break;
		case DtEndNode:
			if(depth == 2)
				chosen = 0;
			depth--;
			p += 4;
			break;
		case DtProp:
			if(p+12 > e)
				return;
			len = beget4(p+4);
			name = strs + beget4(p+8);
			v = p+12;
			if(v+len > e)
				return;
			if(chosen && depth == 2 && nchosen < Maxchosen){
				chosenprops[nchosen].name = name;
				chosenprops[nchosen].val = v;
				chosenprops[nchosen].len = len;
				nchosen++;
			}
			if(strcmp(name, "compatible") == 0
			&& dtblisthas((char*)v, len, "esperanto,maxion-packet-v1"))
				npacket++;
			p = v + ROUND(len, 4);
			break;
		case DtNop:
			p += 4;
			break;
		default:
			return;
		}
	}
}

/* the /chosen property key if it is len bytes long, or any length if len < 0 */
static uchar*
chosen(char *key, int len, int *lenp)
{
	Prop *c;

	for(c = chosenprops; c < chosenprops+nchosen; c++)
		if(strcmp(c->name, key) == 0){
			if(len >= 0 && c->len != len)
				return nil;
			if(lenp != nil)
				*lenp = c->len;
			return c->val;
		}
	return nil;
}

/*
 * The packet mailbox: exactly one enabled esperanto,maxion-packet-v1
 * node with a one-page reg and the complete ABI v1 /chosen bootstrap;
 * the endpoint core checks the rest.
 */
static void
mxpdt(void)
{
	Dtbnode *n;
	uchar *p;
	int i, len;

	if((p = chosen("rng-seed", -1, &len)) != nil){
		mxp.nseed = len < sizeof mxp.seed? len: sizeof mxp.seed;
		memmove(mxp.seed, p, mxp.nseed);
		memset(p, 0, len);
	}
	n = dtbfind("esperanto,maxion-packet-v1", 0);
	if(n == nil || !n->ok || npacket != 1
	|| n->size != BY2PG || n->base == 0 || (n->base & (BY2PG-1)) != 0)
		return;
	if(chosen("esperanto,mxpacket-page-initialized", 0, nil) == nil)
		return;
	if((p = chosen("esperanto,mxpacket-abi", 4, nil)) == nil || beget4(p) != 1)
		return;
	if((p = chosen("esperanto,mxpacket-fw-id", 16, nil)) == nil)
		return;
	for(i = 0; i < 4; i++)
		mxp.fwid[i] = beget4(p + 4*i);
	if((mxp.fwid[0] | mxp.fwid[1] | mxp.fwid[2] | mxp.fwid[3]) == 0)
		return;
	mxp.base = n->base;
	mxp.ok = 1;
}

/*
 * The host time sample, if well formed and after 2001; whether the
 * counter has since been reset is devmxrtc's check, once the clock runs.
 */
static void
rtcdt(void)
{
	uchar *p;

	if((p = chosen("ainekko,rtc-sample", 16, nil)) == nil)
		return;
	rtc.mtime = (uvlong)beget4(p)<<32 | beget4(p+4);
	rtc.ns = (uvlong)beget4(p+8)<<32 | beget4(p+12);
	rtc.ok = rtc.ns >= 1000000000LL*1000000000LL;
	if((p = chosen("ainekko,rtc-error-ns", 4, nil)) != nil){
		rtc.err = beget4(p);
		rtc.haserr = 1;
	}
}

Dtbmxp*
dtbmxp(void)
{
	return mxp.base != 0? &mxp: nil;
}

Dtbrtc*
dtbrtc(void)
{
	return rtc.ok? &rtc: nil;
}

/* read the host's bindings; kmapram calls it before leaving Bare mode */
void
maxiondt(void)
{
	dtscan();
	mxpdt();
	rtcdt();
}
