/*
 * virtio 1.0 9P transport (#9): a host directory exported by
 * qemu's -device virtio-9p-pci with -fsdev local, mounted over a
 * virtqueue with no network in the path.
 *
 *	mount -c '#9/0' /n/host
 *
 * The device has one request queue, and each descriptor chain is
 * one RPC: a device-readable buffer with the T-message and a
 * device-writable one for the R-message.  devmnt drives the
 * channel as it would a tcp connection: write sends a whole
 * T-message, read returns the replies as a byte stream, so write
 * posts a chain and read serves one complete reply at a time.
 *
 * qemu speaks only 9P2000.u, so the messages are translated on
 * the way.  Tversion, Tattach, Tauth, Tcreate and Twstat get the
 * .u version, n_uname, extension and stat; Rversion, Rerror, Rstat
 * and directory reads lose them again.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "io.h"
#include "../port/pci.h"
#include "../port/error.h"
#include "../port/virtio10.h"

typedef struct Vqueue Vqueue;
typedef struct Slot Slot;
typedef struct Fid Fid;
typedef struct Ctlr Ctlr;

enum {
	Typ9p	= 9,
	Legacy9p	= 0x1009,	/* transitional device id */

	/* status flags */
	Acknowledge	= 1,
	Driver		= 2,
	DriverOk	= 4,
	FeaturesOk	= 8,

	/* descriptor flags */
	Next	= 1,
	Write	= 2,

	VringSize	= 4,

	Fmounttag	= 1,		/* feat[0]: the config has a mount tag */

	Msize		= IOHDRSZ+32768,	/* devmnt's MAXRPC */
	Slack		= 64,		/* for the T-messages' growth */
	Maxslot		= 16,		/* RPCs in flight */
	Fidhash		= 64,

	/* message offsets */
	Otype	= BIT32SZ,
	Otag	= Otype+BIT8SZ,
	Obody	= Otag+BIT16SZ,		/* fid, msize, qid, nstat or count */
	/* stat entry offsets, from its size[2] */
	Omode	= BIT16SZ+BIT16SZ+BIT32SZ+QIDSZ,
	Olen	= Omode+3*BIT32SZ,
	Oname	= Olen+BIT64SZ,
};

struct Vqueue
{
	Lock;

	Vio	notify;
	int	idx;
	int	size;
	int	free;
	int	nfree;

	Vdesc	*desc;

	Vring	*avail;
	u16int	*availent;
	u16int	*availevent;

	Vring	*used;
	Vused	*usedent;
	u16int	*usedevent;
	u16int	lastused;

	void	*rock[];
};

struct Slot
{
	uchar	*out;		/* T-message, translated */
	uchar	*in;		/* R-message */
	ulong	inlen;
	ulong	rp;		/* read offset in in */
	int	op;		/* the T-message's type */
	u32int	fid;		/* and fid, for the reply */
	Slot	*next;
};

/* a fid open on a directory */
struct Fid
{
	u32int	fid;
	vlong	offset;		/* qemu's offset: .u entries are longer */
	Fid	*next;
};

struct Ctlr
{
	Pcidev	*pci;
	u32int	feat[2];
	char	tag[64];

	Vio	cfg;
	Vio	isr;
	Vio	notify;
	u32int	notifyoffmult;

	Vqueue	*vq;

	Slot	*slot;
	int	nslot;
	Slot	*free;
	Slot	*done;		/* replies, oldest first */
	Slot	*donetl;
	Slot	*cur;		/* the reply read is serving */
	Rendez	rr;		/* for a reply */
	QLock	wlk;		/* one waiter on wr at a time */
	Rendez	wr;		/* for a free slot */

	QLock	fidlk;
	Fid	*fids[Fidhash];

	int	started;
	int	inuse;

	Ctlr	*next;
};

static Ctlr *ctlrs;

static void clearfids(Ctlr*);

static Vqueue*
mkvqueue(int size)
{
	Vqueue *q;
	uchar *p;
	int i;

	q = malloc(sizeof(*q) + sizeof(void*)*size);
	p = mallocalign(
		PGROUND(sizeof(Vdesc)*size +
			VringSize +
			sizeof(u16int)*size +
			sizeof(u16int)) +
		PGROUND(VringSize +
			sizeof(Vused)*size +
			sizeof(u16int)),
		BY2PG, 0, 0);
	if(p == nil || q == nil){
		print("virtio9p: no memory for Vqueue\n");
		free(p);
		free(q);
		return nil;
	}

	q->desc = (void*)p;
	p += sizeof(Vdesc)*size;
	q->avail = (void*)p;
	p += VringSize;
	q->availent = (void*)p;
	p += sizeof(u16int)*size;
	q->availevent = (void*)p;
	p += sizeof(u16int);

	p = (uchar*)PGROUND((uintptr)p);
	q->used = (void*)p;
	p += VringSize;
	q->usedent = (void*)p;
	p += sizeof(Vused)*size;
	q->usedevent = (void*)p;

	q->free = -1;
	q->nfree = q->size = size;
	for(i=0; i<size; i++){
		q->desc[i].next = q->free;
		q->free = i;
	}

	return q;
}

static int
matchvirtiocfgcap(Pcidev *p, int cap, int off, int typ)
{
	int bar;

	if(cap != 9 || pcicfgr8(p, off+3) != typ)
		return 1;

	/* skip invalid bars */
	bar = pcicfgr8(p, off+4);
	if(bar < 0 || bar >= nelem(p->mem)
	|| p->mem[bar].size == 0)
		return 1;

	return 0;
}

static int
virtiocap(Pcidev *p, int typ)
{
	return pcienumcaps(p, matchvirtiocfgcap, typ);
}

static void
readtag(Ctlr *c)
{
	Vio dev;
	int i, n;

	c->tag[0] = 0;
	if((c->feat[0] & Fmounttag) == 0)
		return;
	if(virtiomapregs(c->pci, virtiocap(c->pci, 4), 0, &dev) == nil)
		return;
	n = vin16(&dev, 0);
	if(n >= sizeof(c->tag))
		n = sizeof(c->tag)-1;
	for(i = 0; i < n; i++)
		c->tag[i] = vin8(&dev, BIT16SZ+i);
	c->tag[n] = 0;
}

static Ctlr*
v9pnp(Pcidev *p)
{
	Ctlr *c;
	Vio cfg;
	Vqueue *q;
	Slot *s;
	int cap, n, i;

	if((cap = virtiocap(p, 1)) < 0)
		return nil;
	if(virtiomapregs(p, cap, Vconf_sz, &cfg) == nil)
		return nil;
	if((c = mallocz(sizeof(*c), 1)) == nil){
		print("virtio9p: no memory for Ctlr\n");
		return nil;
	}
	c->pci = p;
	c->cfg = cfg;
	pcienable(p);

	if(virtiomapregs(p, virtiocap(p, 3), 0, &c->isr) == nil)
		goto Bad;
	cap = virtiocap(p, 2);
	if(virtiomapregs(p, cap, 0, &c->notify) == nil)
		goto Bad;
	c->notifyoffmult = pcicfgr32(p, cap+16);

	/* reset */
	coherence();
	vout8(&cfg, Vconf_status, 0);
	while(vin8(&cfg, Vconf_status) != 0)
		delay(1);
	vout8(&cfg, Vconf_status, Acknowledge|Driver);

	/* version 1 and the mount tag */
	vout32(&cfg, Vconf_devfeatsel, 1);
	c->feat[1] = vin32(&cfg, Vconf_devfeat);
	vout32(&cfg, Vconf_devfeatsel, 0);
	c->feat[0] = vin32(&cfg, Vconf_devfeat);
	vout32(&cfg, Vconf_drvfeatsel, 1);
	vout32(&cfg, Vconf_drvfeat, c->feat[1] & Fversion1);
	vout32(&cfg, Vconf_drvfeatsel, 0);
	vout32(&cfg, Vconf_drvfeat, c->feat[0] & Fmounttag);
	vout8(&cfg, Vconf_status, vin8(&cfg, Vconf_status) | FeaturesOk);
	if((vin8(&cfg, Vconf_status) & FeaturesOk) == 0){
		print("virtio9p: features rejected\n");
		goto Bad;
	}

	vout16(&cfg, Vconf_queuesel, 0);
	n = vin16(&cfg, Vconf_queuesize);
	if(n < 2 || (n & (n-1)) != 0 || (q = mkvqueue(n)) == nil)
		goto Bad;
	q->notify = c->notify;
	if(q->notify.type == Vio_port)
		q->notify.port += c->notifyoffmult * vin16(&cfg, Vconf_queuenotifyoff);
	else
		q->notify.mem += c->notifyoffmult * vin16(&cfg, Vconf_queuenotifyoff);
	q->idx = 0;
	c->vq = q;
	coherence();
	vout64(&cfg, Vconf_queuedesc, PADDR(q->desc));
	vout64(&cfg, Vconf_queueavail, PADDR(q->avail));
	vout64(&cfg, Vconf_queueused, PADDR(q->used));

	readtag(c);

	/* two descriptors a slot */
	c->nslot = n/2;
	if(c->nslot > Maxslot)
		c->nslot = Maxslot;
	if((c->slot = mallocz(sizeof(Slot)*c->nslot, 1)) == nil)
		goto Nomem;
	for(i = 0; i < c->nslot; i++){
		s = &c->slot[i];
		s->out = malloc(Msize+Slack);
		s->in = malloc(Msize+Slack);
		if(s->out == nil || s->in == nil)
			goto Nomem;
		s->next = c->free;
		c->free = s;
	}
	return c;

Nomem:
	print("virtio9p: no memory for slots\n");
Bad:
	pcidisable(p);
	free(c);
	return nil;
}

static void
v9reset(void)
{
	Ctlr *c, **tail;
	Pcidev *p;
	int n;

	n = 0;
	tail = &ctlrs;
	for(p = nil; p = pcimatch(p, 0x1AF4, 0);){
		/* transitional too: no legacy driver to leave them to */
		if(p->did != 0x1040+Typ9p && p->did != Legacy9p)
			continue;
		if((c = v9pnp(p)) == nil)
			continue;
		*tail = c;
		tail = &c->next;
		print("#9/%d: virtio9p tag %s\n", n++, c->tag);
	}
}

/* recycle the completed chains, queue their replies */
static void
vqharvest(Ctlr *c)
{
	Vqueue *q;
	Slot *s;
	int id, free, m, i;

	q = c->vq;
	m = q->size-1;
	ilock(q);
	while((q->lastused ^ q->used->idx) & 0xFFFF){
		i = q->lastused++ & m;
		id = q->usedent[i].id;
		s = q->rock[id];
		q->rock[id] = nil;
		if(s != nil){
			s->inlen = q->usedent[i].len;
			s->next = nil;
			if(c->donetl != nil)
				c->donetl->next = s;
			else
				c->done = s;
			c->donetl = s;
		}
		do {
			free = id;
			id = q->desc[free].next;
			q->desc[free].next = q->free;
			q->free = free;
			q->nfree++;
		} while(q->desc[free].flags & Next);
	}
	iunlock(q);
	wakeup(&c->rr);
}

static void
v9interrupt(Ureg*, void *arg)
{
	Ctlr *c;

	c = arg;
	if(vin8(&c->isr, 0) & 1)
		vqharvest(c);
}

static void
v9start(Ctlr *c)
{
	char name[32];

	if(c->started)
		return;
	pcisetbme(c->pci);
	snprint(name, sizeof(name), "#9/%s", c->tag);
	intrenable(c->pci->intl, v9interrupt, c, c->pci->tbdf, name);
	coherence();
	vout16(&c->cfg, Vconf_queuesel, 0);
	vout16(&c->cfg, Vconf_queueenable, 1);
	vout8(&c->cfg, Vconf_status, vin8(&c->cfg, Vconf_status) | DriverOk);
	c->started = 1;
}

static int
havereply(void *arg)
{
	return ((Ctlr*)arg)->done != nil;
}

static int
haveslot(void *arg)
{
	return ((Ctlr*)arg)->free != nil;
}

static Slot*
getslot(Ctlr *c)
{
	Vqueue *q;
	Slot *s;

	q = c->vq;
	qlock(&c->wlk);
	ilock(q);
	while(c->free == nil){
		iunlock(q);
		while(waserror())
			;
		tsleep(&c->wr, haveslot, c, 1000);
		poperror();
		ilock(q);
	}
	s = c->free;
	c->free = s->next;
	iunlock(q);
	qunlock(&c->wlk);
	s->next = nil;
	return s;
}

static void
putslot(Ctlr *c, Slot *s)
{
	Vqueue *q;

	q = c->vq;
	ilock(q);
	s->next = c->free;
	c->free = s;
	iunlock(q);
	wakeup(&c->wr);
}

static Slot*
getreply(Ctlr *c)
{
	Vqueue *q;
	Slot *s;

	q = c->vq;
	ilock(q);
	while(c->done == nil){
		iunlock(q);
		while(waserror())
			;
		tsleep(&c->rr, havereply, c, 1000);
		poperror();
		if(c->done == nil)
			vqharvest(c);	/* a lost interrupt */
		ilock(q);
	}
	s = c->done;
	c->done = s->next;
	if(c->done == nil)
		c->donetl = nil;
	iunlock(q);
	s->next = nil;
	return s;
}

/* nslot <= size/2: a free slot has its two descriptors */
static void
submit(Ctlr *c, Slot *s, long n)
{
	Vqueue *q;
	Vdesc *d;
	int head, free;

	q = c->vq;
	ilock(q);
	head = free = q->free;
	d = &q->desc[free];
	free = d->next;
	d->addr = PADDR(s->out);
	d->len = n;
	d->flags = Next;
	d = &q->desc[free];
	free = d->next;
	d->addr = PADDR(s->in);
	d->len = Msize+Slack;
	d->flags = Write;
	q->free = free;
	q->nfree -= 2;
	q->rock[head] = s;
	q->availent[q->avail->idx & (q->size-1)] = head;
	coherence();
	q->avail->idx++;
	iunlock(q);
	if((q->used->flags & 1) == 0)
		vout16(&q->notify, 0, q->idx);
}

/* drop the replies nobody will read */
static void
v9flush(Ctlr *c)
{
	Vqueue *q;
	Slot *s, *nx;

	q = c->vq;
	ilock(q);
	if(c->cur != nil){
		c->cur->next = c->free;
		c->free = c->cur;
		c->cur = nil;
	}
	for(s = c->done; s != nil; s = nx){
		nx = s->next;
		s->next = c->free;
		c->free = s;
	}
	c->done = c->donetl = nil;
	iunlock(q);
	clearfids(c);
}

/*
 * Directory reads.  The client's offsets count 9P2000 bytes and
 * qemu's 9P2000.u bytes, so each fid open on a directory keeps
 * qemu's offset, for the Treads.
 */
static Fid**
lookfid(Ctlr *c, u32int fid)
{
	Fid **l;

	for(l = &c->fids[fid % Fidhash]; *l != nil; l = &(*l)->next)
		if((*l)->fid == fid)
			break;
	return l;
}

static void
setdirfid(Ctlr *c, u32int fid, int isdir)
{
	Fid **l, *f;

	qlock(&c->fidlk);
	l = lookfid(c, fid);
	if((f = *l) != nil){
		if(isdir)
			f->offset = 0;
		else {
			*l = f->next;
			free(f);
		}
	} else if(isdir && (f = malloc(sizeof(Fid))) != nil){
		f->fid = fid;
		f->offset = 0;
		f->next = nil;
		*l = f;
	}
	qunlock(&c->fidlk);
}

/* qemu's offset for a Tread at offset on fid, if a directory */
static int
diroffset(Ctlr *c, u32int fid, vlong offset, vlong *qoffset)
{
	Fid *f;

	qlock(&c->fidlk);
	f = *lookfid(c, fid);
	if(f != nil){
		if(offset == 0)
			f->offset = 0;
		*qoffset = f->offset;
	}
	qunlock(&c->fidlk);
	return f != nil;
}

/* advance fid's offset by n of qemu's bytes, if a directory */
static int
diradvance(Ctlr *c, u32int fid, long n)
{
	Fid *f;

	qlock(&c->fidlk);
	f = *lookfid(c, fid);
	if(f != nil)
		f->offset += n;
	qunlock(&c->fidlk);
	return f != nil;
}

static void
clearfids(Ctlr *c)
{
	Fid *f, *nx;
	int i;

	qlock(&c->fidlk);
	for(i = 0; i < Fidhash; i++){
		for(f = c->fids[i]; f != nil; f = nx){
			nx = f->next;
			free(f);
		}
		c->fids[i] = nil;
	}
	qunlock(&c->fidlk);
}

/*
 * A 9P2000.u stat entry is a 9P2000 one with extension[s]
 * n_uid[4] n_gid[4] n_muid[4] after muid[s].  statlen returns the
 * length of the 9P2000 part of the entry at p, after its size[2].
 */
static int
statlen(uchar *p, uchar *e)
{
	uchar *q;
	int i;

	q = p + Oname - BIT16SZ;
	for(i = 0; i < 4; i++){		/* name uid gid muid */
		if(q+BIT16SZ > e)
			return -1;
		q += BIT16SZ + GBIT16(q);
	}
	if(q > e)
		return -1;
	return q - p;
}

/* .u stat entry at in to 9P2000 at out; returns its length */
static int
ustat2stat(uchar *in, uchar *e, uchar *out)
{
	int n;

	if((n = statlen(in+BIT16SZ, e)) < 0)
		return -1;
	/* directories have length 0, not the host's */
	if(in[Omode+BIT32SZ-1] & (DMDIR>>24))
		memset(in+Olen, 0, BIT64SZ);
	PBIT16(out, n);
	memmove(out+BIT16SZ, in+BIT16SZ, n);
	return BIT16SZ+n;
}

/* 9P2000 stat entry at in to .u at out; returns its length */
static int
stat2ustat(uchar *in, uchar *e, uchar *out)
{
	uchar *q;
	int n;

	if((n = statlen(in+BIT16SZ, e)) < 0)
		return -1;
	PBIT16(out, n + BIT16SZ+3*BIT32SZ);
	memmove(out+BIT16SZ, in+BIT16SZ, n);
	q = out+BIT16SZ+n;
	PBIT16(q, 0);			/* extension */
	q += BIT16SZ;
	PBIT32(q, ~0);			/* n_uid */
	q += BIT32SZ;
	PBIT32(q, ~0);			/* n_gid */
	q += BIT32SZ;
	PBIT32(q, ~0);			/* n_muid */
	return BIT16SZ+n + BIT16SZ+3*BIT32SZ;
}

/* T/Rversion m as o, with version v and at most Msize; returns its size */
static long
fixversion(uchar *m, uchar *o, char *v)
{
	long n, msize;

	n = strlen(v);
	if(o != m)
		memmove(o, m, Obody+BIT32SZ);
	msize = GBIT32(o+Obody);
	if(msize > Msize)
		PBIT32(o+Obody, Msize);
	PBIT16(o+Obody+BIT32SZ, n);
	memmove(o+Obody+BIT32SZ+BIT16SZ, v, n);
	n += Obody+BIT32SZ+BIT16SZ;
	PBIT32(o, n);
	return n;
}

/* T-message m, n long, as .u in o; returns its size */
static long
tshim(uchar *m, long n, uchar *o)
{
	int ns;

	switch(m[Otype]){
	case Tversion:
		return fixversion(m, o, "9P2000.u");
	case Tauth:
	case Tattach:
		memmove(o, m, n);
		PBIT32(o+n, ~0);		/* n_uname */
		n += BIT32SZ;
		PBIT32(o, n);
		return n;
	case Tcreate:
		memmove(o, m, n);
		PBIT16(o+n, 0);			/* extension */
		n += BIT16SZ;
		PBIT32(o, n);
		return n;
	case Twstat:
		ns = Obody+BIT32SZ;		/* nstat[2] */
		if(ns+BIT16SZ > n)
			break;
		if((n = stat2ustat(m+ns+BIT16SZ, m+n, o+ns+BIT16SZ)) < 0)
			error(Ebadstat);
		memmove(o, m, ns);
		PBIT16(o+ns, n);
		n += ns+BIT16SZ;
		PBIT32(o, n);
		return n;
	}
	memmove(o, m, n);
	return n;
}

static void
rstat(Slot *s)
{
	uchar *m;
	int n, ns;

	m = s->in;
	ns = Obody;				/* nstat[2] */
	if((n = ustat2stat(m+ns+BIT16SZ, m+s->inlen, m+ns+BIT16SZ)) < 0)
		return;
	PBIT16(m+ns, n);
	s->inlen = ns+BIT16SZ+n;
	PBIT32(m, s->inlen);
}

/* Plan 9 directories have no . or .., and du, mk and the like would recurse */
static int
isdot(uchar *p, uchar *e)
{
	uchar *s;
	int n;

	s = p+Oname;
	if(s+BIT16SZ > e)
		return 0;
	n = GBIT16(s);
	s += BIT16SZ;
	if(s+n > e)
		return 0;
	return (n == 1 && s[0] == '.') || (n == 2 && s[0] == '.' && s[1] == '.');
}

static void
rreaddir(Slot *s)
{
	uchar *m, *r, *w, *e;
	int n, o;

	m = s->in;
	o = Obody+BIT32SZ;			/* data[count] */
	r = w = m+o;
	e = r + GBIT32(m+Obody);
	if(e > m+s->inlen)
		e = m+s->inlen;
	while(r+BIT16SZ <= e){
		n = BIT16SZ + GBIT16(r);
		if(r+n > e)
			break;
		if(!isdot(r, e)){
			if((o = ustat2stat(r, e, w)) < 0)
				break;
			w += o;
		}
		r += n;
	}
	n = w - (m+Obody+BIT32SZ);
	PBIT32(m+Obody, n);
	s->inlen = Obody+BIT32SZ+n;
	PBIT32(m, s->inlen);
}

/* Linux errnos, from qemu, as the errors Plan 9 programs know */
static char*
uerror(int err)
{
	switch(err){
	case 1:		/* EPERM */
	case 13:	/* EACCES */
		return Eperm;
	case 2:		/* ENOENT */
		return Enonexist;
	case 5:		/* EIO */
		return Eio;
	case 12:	/* ENOMEM */
		return Enomem;
	case 16:	/* EBUSY */
		return Einuse;
	case 17:	/* EEXIST */
		return Eexist;
	case 20:	/* ENOTDIR */
		return Enotdir;
	case 21:	/* EISDIR */
		return Eisdir;
	case 22:	/* EINVAL */
		return Ebadarg;
	case 28:	/* ENOSPC */
		return "file system full";
	case 30:	/* EROFS */
		return "file system read only";
	case 39:	/* ENOTEMPTY */
		return "directory not empty";
	}
	return nil;
}

/* reply s from .u to 9P2000, in place */
static void
rshim(Ctlr *c, Slot *s)
{
	uchar *m;
	char *e;
	int n;

	m = s->in;
	if(s->inlen < Obody || s->inlen > Msize)
		return;
	switch(m[Otype]){
	case Rversion:
		s->inlen = fixversion(m, m, "9P2000");
		break;
	case Rerror:
		/* ename[s] errno[4] */
		n = GBIT16(m+Obody);
		if(Obody+BIT16SZ+n+BIT32SZ > s->inlen)
			break;
		if((e = uerror(GBIT32(m+Obody+BIT16SZ+n))) != nil){
			n = strlen(e);
			memmove(m+Obody+BIT16SZ, e, n);
			PBIT16(m+Obody, n);
		}
		s->inlen = Obody+BIT16SZ+n;
		PBIT32(m, s->inlen);
		break;
	case Ropen:
	case Rcreate:
		setdirfid(c, s->fid, m[Obody] & QTDIR);
		break;
	case Rclunk:
	case Rremove:
		setdirfid(c, s->fid, 0);
		break;
	case Rstat:
		rstat(s);
		break;
	case Rread:
		if(diradvance(c, s->fid, GBIT32(m+Obody)))
			rreaddir(s);
		break;
	}
}

static Ctlr*
ctlr(int i)
{
	Ctlr *c;

	for(c = ctlrs; c != nil && i > 0; c = c->next)
		i--;
	return c;
}

static int
v9gen(Chan *c, char*, Dirtab*, int, int s, Dir *dp)
{
	Qid q;
	char buf[16];

	if(s == DEVDOTDOT){
		mkqid(&q, 0, 0, QTDIR);
		devdir(c, q, "#9", 0, eve, DMDIR|0555, dp);
		return 1;
	}
	if(ctlr(s) == nil)
		return -1;
	snprint(buf, sizeof(buf), "%d", s);
	mkqid(&q, s+1, 0, QTFILE);
	devdir(c, q, buf, 0, eve, 0660, dp);
	return 1;
}

static Chan*
v9attach(char *spec)
{
	return devattach('9', spec);
}

static Walkqid*
v9walk(Chan *c, Chan *nc, char **name, int nname)
{
	return devwalk(c, nc, name, nname, nil, 0, v9gen);
}

static int
v9stat(Chan *c, uchar *db, int n)
{
	return devstat(c, db, n, nil, 0, v9gen);
}

static Chan*
v9open(Chan *c, int omode)
{
	Ctlr *cl;
	Vqueue *q;

	if(c->qid.type & QTDIR)
		return devopen(c, omode, nil, 0, v9gen);
	if((cl = ctlr(c->qid.path-1)) == nil)
		error(Enonexist);
	devpermcheck(eve, 0660, omode);

	/* one conversation at a time */
	q = cl->vq;
	ilock(q);
	if(cl->inuse){
		iunlock(q);
		error(Einuse);
	}
	cl->inuse = 1;
	iunlock(q);
	v9start(cl);
	v9flush(cl);

	c->aux = cl;
	c->mode = openmode(omode);
	c->flag |= COPEN;
	c->offset = 0;
	return c;
}

static void
v9close(Chan *c)
{
	Ctlr *cl;
	Vqueue *q;

	if((c->flag & COPEN) == 0 || (c->qid.type & QTDIR))
		return;
	if((cl = c->aux) == nil)
		return;
	v9flush(cl);
	q = cl->vq;
	ilock(q);
	cl->inuse = 0;
	iunlock(q);
}

static long
v9read(Chan *c, void *a, long n, vlong)
{
	Ctlr *cl;
	Slot *s;

	if(c->qid.type & QTDIR)
		return devdirread(c, a, n, nil, 0, v9gen);
	if(n <= 0)
		return 0;
	cl = c->aux;
	if((s = cl->cur) == nil){
		s = getreply(cl);
		rshim(cl, s);
		s->rp = 0;
		cl->cur = s;
	}
	if(n > s->inlen - s->rp)
		n = s->inlen - s->rp;
	memmove(a, s->in + s->rp, n);
	s->rp += n;
	if(s->rp >= s->inlen){
		cl->cur = nil;
		putslot(cl, s);
	}
	return n;
}

static long
v9write(Chan *c, void *a, long n, vlong)
{
	Ctlr *cl;
	Slot *s;
	uchar *m;
	vlong offset;
	long k;

	if(c->qid.type & QTDIR)
		error(Eperm);
	m = a;
	if(n < Obody || n > Msize || GBIT32(m) != n)
		error("invalid 9P message");

	cl = c->aux;
	s = getslot(cl);
	if(waserror()){
		putslot(cl, s);
		nexterror();
	}
	s->op = m[Otype];
	s->fid = 0;
	switch(s->op){
	case Topen:
	case Tcreate:
	case Tread:
	case Tclunk:
	case Tremove:
		if(n < Obody+BIT32SZ)
			error("invalid 9P message");
		s->fid = GBIT32(m+Obody);
		break;
	}
	k = tshim(m, n, s->out);
	if(s->op == Tread && k >= Obody+BIT32SZ+BIT64SZ
	&& diroffset(cl, s->fid, GBIT64(m+Obody+BIT32SZ), &offset)){
		PBIT64(s->out+Obody+BIT32SZ, offset);
	}
	poperror();
	submit(cl, s, k);
	return n;
}

Dev virtio9pdevtab = {
	'9',
	"virtio9p",

	v9reset,
	devinit,
	devshutdown,
	v9attach,
	v9walk,
	v9stat,
	v9open,
	devcreate,
	v9close,
	v9read,
	devbread,
	v9write,
	devbwrite,
	devremove,
	devwstat,
};
