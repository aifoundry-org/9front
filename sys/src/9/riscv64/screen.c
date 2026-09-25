/*
 * screen.c -- generic memimage/devdraw framebuffer console, adapted
 * for the K3 Pico-ITX from riscv64-video-branch's own screen.c (BPI-F3
 * / K1, HDMI, firmware-inherited framebuffer).
 *
 * THAT reference version never programs the display hardware at all
 * -- it takes over whatever framebuffer u-boot left scanning out on
 * HDMI, which the BPI-F3's own u-boot does but this board's does not
 * (its own comment says so explicitly: "With no reservation there is
 * no screen and devdraw fails attach, which is what happens on qemu
 * and the K3"). This board's video path is entirely different: DP1
 * (USB-C DisplayPort Alt-Mode), programmed from scratch in
 * screenpicoitx.c -- real link training, MSA, the works -- confirmed
 * showing a clean, stable image on real hardware (see
 * k3-9front-display-investigation.md). By the time archinit() gets
 * here, dpuk3dp() has already finished and the DPU/DP1 pipeline is
 * live; this file's only job is to hand that already-working
 * framebuffer to the generic memimage/devdraw layer, which is what
 * everything below this comment still does unchanged from the
 * reference version -- newscreen(), attachscreen(), flushmemscreen(),
 * the console (myscreenputs/screenwin), and the cursor.
 *
 * Also shared with the K3 CoM260 IFX carrier board (screencom260.c) --
 * this file is board-agnostic, it just hands off whichever board's
 * already-live DP1 framebuffer to the generic memimage/devdraw layer.
 *
 * Removed from the reference version, since none of it applies here:
 * dpuprobe() (guessing what firmware left running -- we already know,
 * we programmed it), the whole EDID/DDC/choosemode/screensize mode-
 * negotiation path (hdmi.c/i2cddc.c dependencies we haven't ported;
 * this codebase uses one fixed 1920x1080@60 mode throughout, and
 * v2dflush()/v2dhwdraw() (v2d.c's 2D acceleration engine, also not
 * ported -- flushmemscreen() falls back to its own plain CPU copy
 * loop, same as the reference version does when there's no engine).
 *
 * untested on real hardware: this whole file (screenpicoitx.c's own
 * hardware bring-up is confirmed; this integration layer is new).
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

#define	Image	IMAGE
#include <draw.h>
#include <memdraw.h>
#include <cursor.h>
#include "screen.h"

enum {
	Tabstop		= 4,
	Scroll		= 8,
};

Memimage *gscreen;

static uchar *fbraw;		/* K3 Pico-ITX's own DP1 framebuffer, made
				 * non-cacheable in place by
				 * dpuk3fbaddr() (screenpicoitx.c) via
				 * mmuuncache() -- not a second mapping */
static int fbstride;		/* its bytes per line */

static Memimage *conscol;
static Memimage *back;
static Memsubfont *memdefont;

static Lock screenlock;

static Point	curpos;
static int	h, w;
static Rectangle window;

static void myscreenputs(char *s, int n);
static void screenputc(char *buf);
static void screenwin(void);

enum
{
	CMaccelerated,
	CMlinear,
};

static Cmdtab mousectlmsg[] =
{
	CMaccelerated,		"accelerated",		0,
	CMlinear,		"linear",		1,
};

void
mousectl(Cmdbuf *cb)
{
	Cmdtab *ct;

	ct = lookupcmd(cb, mousectlmsg, nelem(mousectlmsg));
	switch(ct->index){
	case CMaccelerated:
		mouseaccelerate(cb->nf == 1? 1: atoi(cb->f[1]));
		break;
	case CMlinear:
		mouseaccelerate(0);
		break;
	}
}

void
cursoron(void)
{
	swcursorhide(0);
	swcursordraw(mousexy());
}

void
cursoroff(void)
{
	swcursorhide(0);
}

void
setcursor(Cursor* curs)
{
	swcursorload(curs);
}

/*
 * Blanking would mean turning the display off, which needs the hardware we
 * have no documentation for.  Leave the picture up.
 */
void
blankscreen(int)
{
}

int
hwdraw(Memdrawparam *par)
{
	Memimage *dst, *src, *mask;
	uchar *scrd;

	if((dst = par->dst) == nil || dst->data == nil)
		return 0;
	if((src = par->src) && src->data == nil)
		src = nil;
	if((mask = par->mask) && mask->data == nil)
		mask = nil;

	scrd = gscreen->data->bdata;
	if(dst->data->bdata == scrd)
		swcursoravoid(par->r);
	if(src && src->data->bdata == scrd)
		swcursoravoid(par->sr);
	if(mask && mask->data->bdata == scrd)
		swcursoravoid(par->mr);

	/* no 2D engine ported (v2d.c, reference-only) -- tell devdraw to
	 * fall back to its own plain software path */
	return 0;
}

/*
 * Tilt turns the picture on a portrait monitor: the screen everyone draws
 * on is the rotated one, and flushmemscreen turns it back on the way to
 * the framebuffer, as the pc does.  Untilted, the flush is a straight
 * memmove and none of this is touched.
 */
static int tilt;
char *tiltstr[4] = {
	"none",
	"left",
	"inverted",
	"right",
};

static Point
tiltpt(int tilt, Point dim, Point p)
{
	switch(tilt&3){
	case 1:	return Pt(dim.y-p.y-1, p.x);
	case 2:	return Pt(dim.x-p.x-1, dim.y-p.y-1);
	case 3:	return Pt(p.y, dim.x-p.x-1);
	}
	return p;
}

static Rectangle
tiltrect(int tilt, Point dim, Rectangle r)
{
	switch(tilt&3){
	case 1:	return Rect(dim.y-r.max.y, r.min.x, dim.y-r.min.y, r.max.x);
	case 2:	return Rect(dim.x-r.max.x, dim.y-r.max.y, dim.x-r.min.x, dim.y-r.min.y);
	case 3:	return Rect(r.min.y, dim.x-r.max.x, r.max.y, dim.x-r.min.x);
	}
	return r;
}

static Point
tiltsize(int tilt, Point dim)
{
	return (tilt & 1) != 0 ? Pt(dim.y, dim.x) : dim;
}

int
screentilt(void)
{
	return tilt;
}

static void screenwin(void);

/*
 * Switch to a mode; called with drawlock held, or before there is one.
 * The framebuffer stays where the firmware put it, so the mode must fit
 * in what is left of the reservation.
 */
static char*
newscreen(int w, int h, int t)
{
	Memimage *new;

	new = allocmemimage(Rpt(ZP, tiltsize(t, Pt(w, h))), XRGB32);
	if(new == nil)
		return "no memory for screen";
	if(gscreen != nil)
		freememimage(gscreen);
	gscreen = new;
	tilt = t;
	fbstride = w * 4;
	if(screenputs == myscreenputs)
		screenwin();
	mouseresize();
	return nil;
}

/* none, left, inverted or right; called with drawlock held */
char*
screensettilt(char *name)
{
	int t;

	for(t = 0; t < nelem(tiltstr); t++)
		if(cistrcmp(name, tiltstr[t]) == 0)
			break;
	if(t == nelem(tiltstr))
		return "bad tilt";
	if(gscreen == nil)
		return "no screen";
	return newscreen(fbstride/4, tilt & 1? Dx(gscreen->r): Dy(gscreen->r), t);
}

void
screeninit(void)
{
	uvlong fbaddr;
	int width, height, depth, stride;
	char *p;

	/*
	 * K3 Pico-ITX: the framebuffer is our own, programmed from
	 * scratch by screenpicoitx.c's dpuk3dp() (real DP1 link training,
	 * MSA, the works -- confirmed showing a clean, stable image on
	 * real hardware) -- not inherited from firmware, unlike the
	 * reference version this file is adapted from. dpuk3fbaddr()
	 * reserves (idempotent: a second call just returns the same
	 * physical address) and marks that range non-cacheable in place
	 * via mmuuncache(), same mechanism the reference screeninit()
	 * used on its own firmware-inherited framebuffer -- see its own
	 * comment in screenpicoitx.c. devarch.c calls this AFTER
	 * dpuk3dp() has already finished, so the DP1 pipeline is live by
	 * the time we get here, and the mode is the fixed 1920x1080@60
	 * this whole codebase uses throughout (no EDID/mode negotiation
	 * ported -- see this file's own header).
	 */
	fbaddr = dpuk3fbaddr();
	if(fbaddr == 0){
		print("screeninit: no DP1 framebuffer\n");
		return;
	}

	width = 1920;
	height = 1080;
	depth = 32;
	stride = width*depth/8;	/* 7680, matches dpuk3mode()'s own stride */

	memimageinit();

	fbraw = dpuk3fbkaddr();
	if(fbraw == nil){
		print("screeninit: dpuk3fbkaddr() returned nil\n");
		return;
	}
	fbstride = stride;

	if(gscreen == nil && (p = newscreen(width, height, tilt)) != nil){
		print("screeninit: %s\n", p);
		return;
	}

	conf.monitor = 1;
	print("screen: %dx%dx%d stride %d at %#llux\n",
		width, height, depth, fbstride, fbaddr);

	memdefont = getmemdefont();
	screenwin();
	myscreenputs(kmesg.buf, kmesg.n);
	screenputs = myscreenputs;
	swcursorinit();
}

void
flushmemscreen(Rectangle r)
{
	int n, spitch;
	uchar *d, *s;

	if(gscreen == nil || fbraw == nil)
		return;
	if(!rectclip(&r, gscreen->r))
		return;

	if(tilt){
		Point size;
		ulong *sp, *dp;
		int incs, x;

		size = gscreen->r.max;			/* as drawn on */
		r = tiltrect(-tilt, size, r);		/* as scanned out */
		size = tiltsize(-tilt, size);
		sp = (ulong*)byteaddr(gscreen, tiltpt(tilt, size, r.min));
		incs = (ulong*)byteaddr(gscreen, tiltpt(tilt, size, Pt(r.min.x+1, r.min.y))) - sp;
		for(;;){
			dp = (ulong*)(fbraw + r.min.y*fbstride);
			for(x = r.min.x; x < r.max.x; x++, sp += incs)
				dp[x] = *sp;
			if(++r.min.y >= r.max.y)
				break;
			sp = (ulong*)byteaddr(gscreen, tiltpt(tilt, size, r.min));
		}
		return;
	}

	/*
	 * No 2D engine ported (v2d.c, reference-only) -- always the plain
	 * CPU copy loop below. fbraw is non-cacheable (mmuuncache(), see
	 * dpuk3fbaddr() in screenpicoitx.c), so each memmove here lands
	 * in real DRAM immediately, visible to the DPU's own DMA without
	 * any separate cache-flush step.
	 */
	d = fbraw + (r.min.y - gscreen->r.min.y)*fbstride
	    + (r.min.x - gscreen->r.min.x)*gscreen->depth/8;

	spitch = gscreen->width*sizeof(ulong);
	n = bytesperline(r, gscreen->depth);
	s = byteaddr(gscreen, r.min);
	while(r.min.y++ < r.max.y){
		memmove(d, s, n);
		d += fbstride;
		s += spitch;
	}
}

Memdata*
attachscreen(Rectangle *r, ulong *chan, int *d, int *width, int *softscreen)
{
	if(gscreen == nil)
		return nil;

	*r = gscreen->r;
	*d = gscreen->depth;
	*chan = gscreen->chan;
	*width = gscreen->width;
	*softscreen = 1;

	gscreen->data->ref++;
	return gscreen->data;
}

void
getcolor(ulong p, ulong *pr, ulong *pg, ulong *pb)
{
	USED(p, pr, pg, pb);
}

int
setcolor(ulong p, ulong r, ulong g, ulong b)
{
	USED(p, r, g, b);
	return 0;
}

static void
myscreenputs(char *s, int n)
{
	int i;
	Rune r;
	char buf[4];

	if(!islo()) {
		/* don't deadlock trying to print in interrupt */
		if(!canlock(&screenlock))
			return;
	}
	else {
		while(!canlock(&screenlock))
			;
	}

	while(n > 0){
		i = chartorune(&r, s);
		if(i == 0){
			s++;
			--n;
			continue;
		}
		memmove(buf, s, i);
		buf[i] = 0;
		n -= i;
		s += i;
		screenputc(buf);
	}
	unlock(&screenlock);
}

static void
screenwin(void)
{
	char *greet;
	Memimage *orange;
	Point p, q;
	Rectangle r;

	back = memblack;
	conscol = memwhite;

	orange = allocmemimage(Rect(0, 0, 1, 1), RGB16);
	orange->flags |= Frepl;
	orange->clipr = gscreen->r;
	orange->data->bdata[0] = 0x40;		/* magic: colour? */
	orange->data->bdata[1] = 0xfd;		/* magic: colour? */

	w = memdefont->info[' '].width;
	h = memdefont->height;

	r = gscreen->r;
	memimagedraw(gscreen, r, memwhite, ZP, memopaque, ZP, S);
	window = insetrect(r, 4);
	memimagedraw(gscreen, window, memblack, ZP, memopaque, ZP, S);

	memimagedraw(gscreen, Rect(window.min.x, window.min.y,
		window.max.x, window.min.y + h + 5 + 6), orange, ZP, nil, ZP, S);

	freememimage(orange);
	window = insetrect(window, 5);

	greet = " Plan 9 Console ";
	p = addpt(window.min, Pt(10, 0));
	q = memsubfontwidth(memdefont, greet);
	USED(q);
	memimagestring(gscreen, p, conscol, ZP, memdefont, greet);
	flushmemscreen(r);
	window.min.y += h + 6;
	curpos = window.min;
	window.max.y = window.min.y + ((window.max.y - window.min.y) / h) * h;
}

static void
scroll(void)
{
	int o;
	Point p;
	Rectangle r;

	o = Scroll*h;
	r = Rpt(window.min, Pt(window.max.x, window.max.y-o));
	p = Pt(window.min.x, window.min.y+o);
	memimagedraw(gscreen, r, gscreen, p, nil, p, S);
	flushmemscreen(r);
	r = Rpt(Pt(window.min.x, window.max.y-o), window.max);
	memimagedraw(gscreen, r, back, ZP, nil, ZP, S);
	flushmemscreen(r);

	curpos.y -= o;
}

static void
screenputc(char *buf)
{
	int w;
	uint pos;
	Point p;
	Rectangle r;
	static int *xp;
	static int xbuf[256];

	if (xp < xbuf || xp >= &xbuf[nelem(xbuf)])
		xp = xbuf;

	switch (buf[0]) {
	case '\n':
		if (curpos.y + h >= window.max.y)
			scroll();
		curpos.y += h;
		screenputc("\r");
		break;
	case '\r':
		xp = xbuf;
		curpos.x = window.min.x;
		break;
	case '\t':
		p = memsubfontwidth(memdefont, " ");
		w = p.x;
		if (curpos.x >= window.max.x - Tabstop * w)
			screenputc("\n");

		pos = (curpos.x - window.min.x) / w;
		pos = Tabstop - pos % Tabstop;
		*xp++ = curpos.x;
		r = Rect(curpos.x, curpos.y, curpos.x + pos * w, curpos.y + h);
		memimagedraw(gscreen, r, back, back->r.min, nil, back->r.min, S);
		flushmemscreen(r);
		curpos.x += pos * w;
		break;
	case '\b':
		if (xp <= xbuf)
			break;
		xp--;
		r = Rect(*xp, curpos.y, curpos.x, curpos.y + h);
		memimagedraw(gscreen, r, back, back->r.min, nil, back->r.min, S);
		flushmemscreen(r);
		curpos.x = *xp;
		break;
	case '\0':
		break;
	default:
		p = memsubfontwidth(memdefont, buf);
		w = p.x;

		if (curpos.x >= window.max.x - w)
			screenputc("\n");

		*xp++ = curpos.x;
		r = Rect(curpos.x, curpos.y, curpos.x + w, curpos.y + h);
		memimagedraw(gscreen, r, back, back->r.min, nil, back->r.min, S);
		memimagestring(gscreen, curpos, conscol, ZP, memdefont, buf);
		flushmemscreen(r);
		curpos.x += w;
		break;
	}
}
