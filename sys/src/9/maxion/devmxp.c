/*
 * Maxion PC<->MX packet mailbox ("MXP"): a raw IP point-to-point
 * link to the PCIe host, the 9front guest side of the et-soc1
 * host driver's etmx0 netdev.  The Linux guest frontend in et-platform
 * (device-bootloaders/src/Maxion/linux/packet-driver) is the model:
 * the same endpoint core (mxpcore.c), the same bootstrap rules and the
 * same IPv4/IPv6 framing checks; Plan 9 binds it to IP with
 *
 *	echo bind netdev /dev/mxp >/net/ipifc/clone
 *
 * Files:
 *	mxp	one raw IPv4/IPv6 packet per read or write
 *	mxpctl	status; write "quiesce" (terminal) or "poll active idle" (ms)
 *
 * The mailbox page has no interrupt: a kproc polls it.  The page is
 * touched only when the DT carries the complete one-boot bootstrap, and
 * a session is never reattached: quiesce and faults are terminal until
 * the next cold boot, exactly as on Linux.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "maxion.h"
#include "mxp.h"

enum {
	Qdir,
	Qdata,
	Qctl,

	Qlimit		= 64*1024,	/* bytes queued each way (~32 packets) */
	Budget		= 8,		/* mailbox operations per poll pass */
	Activems	= 1,		/* poll period while traffic is recent */
	Idlems		= 10,		/* poll period otherwise */
	Activetime	= 2000,		/* ms of recent traffic to stay active */
	Txexpire	= 5000,		/* ms a packet may wait for credit */
	Attachtries	= 1000,
};

enum {
	Pabsent,	/* no usable bootstrap: page never touched */
	Pwait,		/* waiting for entropy */
	Pattach,
	Prun,
	Pstopped,	/* quiesced, faulted or rejected: terminal */
};

static char *mxpphases[] = {
[Pabsent]	"absent",
[Pwait]		"entropy",
[Pattach]	"attach",
[Prun]		"run",
[Pstopped]	"stopped",
};

static char *mxpstates[] = {
[MXP_STATE_DOWN]	"down",
[MXP_STATE_OFFER]	"offer",
[MXP_STATE_ACCEPT]	"accept",
[MXP_STATE_UP]		"up",
[MXP_STATE_QUIESCING]	"quiescing",
[MXP_STATE_FAULT]	"fault",
};

static char *mxpfaults[] = {
[MXP_FAULT_NONE]	"none",
[MXP_FAULT_METADATA]	"metadata",
[MXP_FAULT_STATE]	"state",
[MXP_FAULT_IDENTITY]	"identity",
[MXP_FAULT_OFFSET]	"offset",
[MXP_FAULT_HEADER]	"header",
[MXP_FAULT_CRC]		"crc",
[MXP_FAULT_SEQUENCE]	"sequence",
[MXP_FAULT_PEER]	"peer",
[MXP_FAULT_INITIAL]	"initial",
[MXP_FAULT_FIRMWARE]	"firmware",
};

typedef struct Ctlr Ctlr;
struct Ctlr
{
	QLock;			/* serializes the endpoint core */
	Mxp	mxp;
	Mxpops	ops;
	uintptr	page;		/* identity-mapped mailbox */
	Dtbmxp	*dt;
	int	phase;
	char	why[ERRMAX];	/* why the link stopped */

	Queue	*rq;		/* received packets for readers */
	Queue	*wq;		/* packets waiting to be sent */
	Block	*pend;		/* head packet waiting for credit */
	ulong	pendms;
	Rendez	r;
	int	kick;
	int	quiesce;
	Rendez	stopr;
	int	dataopen;
	int	activems;
	int	idlems;
	ulong	lastms;		/* last packet moved */
	u32int	sink;		/* drain readback */

	uvlong	polls;
	uvlong	rxbad;		/* framing rejected on receive */
	uvlong	rxdrop;		/* reader queue full */
	uvlong	txbad;		/* framing rejected on send */
	uvlong	txdrop;		/* send queue full or link not up */
	uvlong	txexpired;
};

static Ctlr ctlr;

static Dirtab mxpdir[] = {
	".",		{Qdir, 0, QTDIR},	0,	DMDIR|0555,
	"mxp",		{Qdata},		0,	0660,
	"mxpctl",	{Qctl},			0,	0660,
};

/* the page is little-endian SRAM, uncached; the core bounds offsets */
static u32int
mxpread32(void *a, u32int off)
{
	Ctlr *c;

	c = a;
	if(off >= BY2PG || (off & 3) != 0)
		return ~0;
	return *(u32int*)(c->page + off);
}

static void
mxpwrite32(void *a, u32int off, u32int v)
{
	Ctlr *c;

	c = a;
	if(off >= BY2PG || (off & 3) != 0)
		return;
	*(u32int*)(c->page + off) = v;
}

static void
mxporder(void*)
{
	coherence();
}

/* complete earlier writes at the SRAM: barrier, same-page readback, barrier */
static void
mxpdrain(void *a)
{
	Ctlr *c;

	c = a;
	coherence();
	c->sink = *(u32int*)(c->page + MXP_M_MAGIC);
	coherence();
}

/*
 * The Linux frontend's framing rule: one whole IPv4 or IPv6 packet
 * whose header length agrees exactly with the record length.
 */
static int
ipframe(uchar *p, int n)
{
	int hl;

	if(n < 1)
		return 0;
	switch(p[0]>>4){
	case 4:
		if(n < 20)
			return 0;
		hl = (p[0] & 15)*4;
		return hl >= 20 && hl <= n && (p[2]<<8 | p[3]) == n;
	case 6:
		return n >= 40 && (p[4]<<8 | p[5]) + 40 == n;
	}
	return 0;
}

static void
mxpstop(Ctlr *c, char *why)
{
	if(c->phase != Pstopped){
		c->phase = Pstopped;
		strecpy(c->why, c->why+sizeof(c->why), why);
		print("mxp: link stopped: %s\n", why);
	}
	if(c->pend != nil){
		freeb(c->pend);
		c->pend = nil;
	}
	qhangup(c->wq, c->why);
	qhangup(c->rq, c->why);
	wakeup(&c->stopr);
}

/* why the core refused, as a stop reason */
static void
mxpfailed(Ctlr *c, char *op, int r)
{
	Mxpstatus s;
	char buf[ERRMAX];

	mxpgetstatus(&c->mxp, &s);
	if(s.localstate == MXP_STATE_FAULT && s.faultreason < nelem(mxpfaults))
		snprint(buf, sizeof(buf), "%s: fault %s", op, mxpfaults[s.faultreason]);
	else if(s.localstate == MXP_STATE_QUIESCING)
		snprint(buf, sizeof(buf), "%s: quiesced (peer %s)", op,
			s.peerstate < nelem(mxpstates)? mxpstates[s.peerstate]: "?");
	else
		snprint(buf, sizeof(buf), "%s: result %d", op, r);
	mxpstop(c, buf);
}

static int
mxpkicked(void *a)
{
	Ctlr *c;

	c = a;
	return c->kick || c->quiesce;
}

/* the poller has packets to send */
static void
mxpwake(void *a)
{
	Ctlr *c;

	c = a;
	c->kick = 1;
	wakeup(&c->r);
}

/* one bounded pass with the core held; returns packets moved */
static int
mxppass(Ctlr *c)
{
	Mxpstatus s;
	Block *b;
	u32int n;
	int i, r, moved, progress;

	moved = 0;
	r = mxpservice(&c->mxp);
	if(r == MXP_AGAIN)
		return 0;
	if(r == MXP_NOT_UP){
		mxpgetstatus(&c->mxp, &s);
		if(s.localstate == MXP_STATE_QUIESCING){
			mxpfailed(c, "service", r);
			return 0;
		}
		/* still negotiating: nothing is queued for a link not up */
		if(c->pend != nil){
			freeb(c->pend);
			c->pend = nil;
			c->txdrop++;
		}
		while((b = qget(c->wq)) != nil){
			freeb(b);
			c->txdrop++;
		}
		return 0;
	}
	if(r != MXP_OK){
		mxpfailed(c, "service", r);
		return 0;
	}
	for(i = 0; i < Budget; i++){
		progress = 0;
		mxpgetstatus(&c->mxp, &s);
		/* a full reader queue leaves records in the ring: the host waits for credit */
		if(s.rxused != 0 && !qfull(c->rq)){
			b = iallocb(MXP_MAX_PAYLOAD);
			if(b == nil)
				break;
			r = mxprecv(&c->mxp, b->wp, MXP_MAX_PAYLOAD, &n);
			if(r != MXP_OK){
				freeb(b);
				if(r != MXP_AGAIN){
					mxpfailed(c, "recv", r);
					return moved;
				}
			}else{
				b->wp += n;
				if(!ipframe(b->rp, n)){
					c->rxbad++;
					freeb(b);
				}else if(qpass(c->rq, b) < 0)
					c->rxdrop++;
				progress = 1;
			}
		}
		if(c->pend == nil && (c->pend = qget(c->wq)) != nil)
			c->pendms = TK2MS(MACHP(0)->ticks);
		if(c->pend != nil){
			n = BLEN(c->pend);
			mxpgetstatus(&c->mxp, &s);
			if(s.txfree >= mxprecordbytes(n)){
				r = mxpsend(&c->mxp, c->pend->rp, n);
				if(r == MXP_OK){
					freeb(c->pend);
					c->pend = nil;
					progress = 1;
				}else if(r != MXP_AGAIN){
					mxpfailed(c, "send", r);
					return moved;
				}
			}else if(TK2MS(MACHP(0)->ticks) - c->pendms > Txexpire){
				freeb(c->pend);
				c->pend = nil;
				c->txexpired++;
			}
		}
		if(!progress)
			break;
		moved++;
	}
	return moved;
}

/* a fresh nonzero boot identity: kernel entropy mixed with the host seed */
static void
mxpbootid(Ctlr *c, Mxpid *id)
{
	uchar b[16];
	int i;

	do{
		randomread(b, sizeof(b));
		for(i = 0; i < c->dt->nseed; i++)
			b[i%16] ^= c->dt->seed[i];
		for(i = 0; i < 4; i++)
			id->w[i] = b[4*i] | b[4*i+1]<<8 | b[4*i+2]<<16 | (u32int)b[4*i+3]<<24;
	}while((id->w[0] | id->w[1] | id->w[2] | id->w[3]) == 0);
	memset(b, 0, sizeof(b));
	memset(c->dt->seed, 0, sizeof(c->dt->seed));
}

static void
mxpproc(void *a)
{
	Ctlr *c;
	Mxpid id;
	Mxpstatus s;
	int i, r, ms;

	c = a;
	if(waserror()){
		print("mxp: %s\n", up->errstr);
		pexit(up->errstr, 1);
	}
	mxpbootid(c, &id);

	qlock(c);
	c->phase = Pattach;
	for(i = 0; ; i++){
		r = mxpattach(&c->mxp, &c->ops, c, MXP_GUEST, &id);
		if(r != MXP_AGAIN || i >= Attachtries || c->quiesce)
			break;
		qunlock(c);
		tsleep(&up->sleep, return0, 0, 1);
		qlock(c);
	}
	memset(&id, 0, sizeof(id));
	if(r != MXP_OK){
		mxpfailed(c, "attach", r);
		qunlock(c);
		pexit("", 1);
	}
	/* the page must be the generation the host bootstrap described */
	mxpgetstatus(&c->mxp, &s);
	if(memcmp(s.fwid.w, c->dt->fwid, sizeof(s.fwid.w)) != 0){
		mxpstop(c, "attach: firmware id differs from bootstrap");
		qunlock(c);
		pexit("", 1);
	}
	c->phase = Prun;
	print("mxp: attached guest endpoint, page %#p\n", c->page);
	qunlock(c);

	for(;;){
		qlock(c);
		c->kick = 0;
		if(c->quiesce){
			r = mxpshutdown(&c->mxp);
			mxpfailed(c, "quiesce", r);
		}else if(mxppass(c) > 0)
			c->lastms = TK2MS(MACHP(0)->ticks);
		c->polls++;
		if(c->phase == Pstopped){
			qunlock(c);
			pexit("", 1);
		}
		ms = c->idlems;
		if(c->pend != nil || qlen(c->wq) > 0
		|| TK2MS(MACHP(0)->ticks) - c->lastms < Activetime)
			ms = c->activems;
		qunlock(c);
		tsleep(&c->r, mxpkicked, c, ms);
	}
}

static void
mxpinit(void)
{
	Ctlr *c;

	c = &ctlr;
	c->phase = Pabsent;
	c->dt = dtbmxp();
	if(c->dt == nil || !c->dt->ok)
		return;
	c->page = c->dt->base;
	c->ops.read32 = mxpread32;
	c->ops.write32 = mxpwrite32;
	c->ops.order = mxporder;
	c->ops.drain = mxpdrain;
	c->activems = Activems;
	c->idlems = Idlems;
	c->rq = qopen(Qlimit, Qmsg, 0, 0);
	c->wq = qopen(Qlimit, Qmsg|Qkick, mxpwake, c);
	if(c->rq == nil || c->wq == nil)
		panic("mxpinit: no queues");
	c->phase = Pwait;
	kproc("mxp", mxpproc, c);
}

static int
mxpgen(Chan *c, char *name, Dirtab *tab, int ntab, int i, Dir *dp)
{
	/* an absent link has an empty directory */
	if(ctlr.phase == Pabsent && i != DEVDOTDOT)
		ntab = 1;
	return devgen(c, name, tab, ntab, i, dp);
}

static Chan*
mxpdevattach(char *spec)
{
	return devattach(L'E', spec);
}

static Walkqid*
mxpwalk(Chan *c, Chan *nc, char **name, int nname)
{
	return devwalk(c, nc, name, nname, mxpdir, nelem(mxpdir), mxpgen);
}

static int
mxpstat(Chan *c, uchar *dp, int n)
{
	return devstat(c, dp, n, mxpdir, nelem(mxpdir), mxpgen);
}

static Chan*
mxpopen(Chan *c, int omode)
{
	Ctlr *ctl;

	ctl = &ctlr;
	c = devopen(c, omode, mxpdir, nelem(mxpdir), mxpgen);
	if((ulong)c->qid.path == Qdata){
		/* one IP binding or test client at a time */
		qlock(ctl);
		if(ctl->dataopen){
			qunlock(ctl);
			c->flag &= ~COPEN;
			error(Einuse);
		}
		ctl->dataopen = 1;
		qunlock(ctl);
	}
	return c;
}

static void
mxpclose(Chan *c)
{
	if((c->flag & COPEN) == 0 || (ulong)c->qid.path != Qdata)
		return;
	qlock(&ctlr);
	ctlr.dataopen = 0;
	qunlock(&ctlr);
}

static char*
idstr(char *p, char *e, char *name, Mxpid *id)
{
	return seprint(p, e, "%s %.8ux%.8ux%.8ux%.8ux\n", name,
		id->w[0], id->w[1], id->w[2], id->w[3]);
}

static long
mxpctlread(void *a, long n, vlong off)
{
	Ctlr *c;
	Mxpstatus s;
	Mxpdiag d;
	char *buf, *p, *e;

	c = &ctlr;
	buf = smalloc(READSTR);
	if(waserror()){
		free(buf);
		nexterror();
	}
	p = buf;
	e = buf + READSTR;
	qlock(c);
	p = seprint(p, e, "phase %s\n", mxpphases[c->phase]);
	if(c->phase == Pstopped)
		p = seprint(p, e, "stopped %s\n", c->why);
	if(c->phase >= Pattach){
		mxpgetstatus(&c->mxp, &s);
		mxpgetdiag(&c->mxp, &d);
		p = seprint(p, e, "page %#p\n", c->page);
		p = seprint(p, e, "local %s\npeer %s\n",
			s.localstate < nelem(mxpstates)? mxpstates[s.localstate]: "?",
			s.peerstate < nelem(mxpstates)? mxpstates[s.peerstate]: "?");
		p = seprint(p, e, "fault %s\n",
			s.faultreason < nelem(mxpfaults)? mxpfaults[s.faultreason]: "?");
		p = seprint(p, e, "ready %#ux\ntxfree %ud\nrxused %ud\ntxseq %ud\nrxseq %ud\n",
			s.ready, s.txfree, s.rxused, s.txseq, s.rxseq);
		p = idstr(p, e, "fwid", &s.fwid);
		p = idstr(p, e, "hostnonce", &s.hostnonce);
		p = idstr(p, e, "guestid", &s.guestid);
		p = seprint(p, e, "txpackets %llud\nrxpackets %llud\ntxbytes %llud\nrxbytes %llud\n",
			d.txpackets, d.rxpackets, d.txbytes, d.rxbytes);
		p = seprint(p, e, "txfull %llud\nrxempty %llud\nfaults %llud\n",
			d.txfull, d.rxempty, d.faults);
	}
	p = seprint(p, e, "polls %llud\nrxbad %llud\nrxdrop %llud\ntxbad %llud\ntxdrop %llud\ntxexpired %llud\n",
		c->polls, c->rxbad, c->rxdrop, c->txbad, c->txdrop, c->txexpired);
	seprint(p, e, "poll %d %d\n", c->activems, c->idlems);
	qunlock(c);
	n = readstr(off, a, n, buf);
	poperror();
	free(buf);
	return n;
}

static long
mxpread(Chan *c, void *a, long n, vlong off)
{
	switch((ulong)c->qid.path){
	case Qdir:
		return devdirread(c, a, n, mxpdir, nelem(mxpdir), mxpgen);
	case Qctl:
		return mxpctlread(a, n, off);
	case Qdata:
		return qread(ctlr.rq, a, n);
	}
	error(Egreg);
	return 0;
}

static Block*
mxpbread(Chan *c, long n, ulong off)
{
	if((ulong)c->qid.path == Qdata)
		return qbread(ctlr.rq, n);
	return devbread(c, n, off);
}

/*
 * queue one outgoing packet.  Like etheroq, a full queue holds the
 * writer until the poller drains it; IP never sees an error.
 */
static void
mxpqueue(Block *b)
{
	Ctlr *c;

	c = &ctlr;
	if(b->next != nil)
		b = concatblock(b);
	if(BLEN(b) > MXP_MAX_PAYLOAD || !ipframe(b->rp, BLEN(b))){
		c->txbad++;
		freeb(b);
		return;
	}
	if(c->phase != Prun){
		c->txdrop++;
		freeb(b);
		return;
	}
	if(waserror()){
		/* queue hung up (block freed) or interrupted after queueing */
		c->txdrop++;
		return;
	}
	qbwrite(c->wq, b);
	poperror();
}

static long
mxpbwrite(Chan *c, Block *b, ulong off)
{
	long n;

	if((ulong)c->qid.path != Qdata)
		return devbwrite(c, b, off);
	n = BLEN(b);
	mxpqueue(b);
	return n;
}

static long
mxpwrite(Chan *c, void *a, long n, vlong)
{
	Ctlr *ctl;
	Cmdbuf *cb;
	Block *b;

	ctl = &ctlr;
	switch((ulong)c->qid.path){
	case Qdata:
		if(n < 1 || n > MXP_MAX_PAYLOAD)
			error(Etoobig);
		b = allocb(n);
		if(waserror()){
			freeb(b);
			nexterror();
		}
		memmove(b->wp, a, n);
		poperror();
		b->wp += n;
		mxpqueue(b);
		return n;
	case Qctl:
		cb = parsecmd(a, n);
		if(waserror()){
			free(cb);
			nexterror();
		}
		if(cb->nf == 1 && strcmp(cb->f[0], "quiesce") == 0){
			if(ctl->phase == Pabsent)
				error("no packet mailbox");
			ctl->quiesce = 1;
			wakeup(&ctl->r);
			while(ctl->phase != Pstopped)
				tsleep(&ctl->stopr, return0, 0, 10);
		}else if(cb->nf == 3 && strcmp(cb->f[0], "poll") == 0){
			ctl->activems = atoi(cb->f[1]);
			ctl->idlems = atoi(cb->f[2]);
			if(ctl->activems < 1)
				ctl->activems = 1;
			if(ctl->idlems < ctl->activems)
				ctl->idlems = ctl->activems;
			if(ctl->idlems > 1000)
				ctl->idlems = 1000;
		}else
			error(Ebadctl);
		poperror();
		free(cb);
		return n;
	}
	error(Egreg);
	return 0;
}

Dev mxpdevtab = {
	L'E',
	"mxp",

	devreset,
	mxpinit,
	devshutdown,
	mxpdevattach,
	mxpwalk,
	mxpstat,
	mxpopen,
	devcreate,
	mxpclose,
	mxpread,
	mxpbread,
	mxpwrite,
	mxpbwrite,
	devremove,
	devwstat,
};
