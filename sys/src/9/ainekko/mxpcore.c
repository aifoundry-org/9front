/*
 * Maxion packet mailbox v1 endpoint core.  A statement-for-statement
 * Plan 9 C port of et-common-libs transports/mxpacket/core.h
 * (GPL-2.0-only OR BSD-2-Clause): the same checks, ordering callbacks
 * and word accesses in the same order, so its behaviour and MMIO trace
 * match the Linux endpoints.  No allocation, sleeping, locks or
 * physical addresses; the caller serializes every call on one context.
 */
#include "u.h"
#include "mxp.h"

static u32int
rd(Mxp *p, u32int off)
{
	return p->ops.read32(p->arg, off);
}

static void
wr(Mxp *p, u32int off, u32int w)
{
	p->ops.write32(p->arg, off, w);
}

static void
order(Mxp *p)
{
	p->ops.order(p->arg);
}

static void
drain(Mxp *p)
{
	p->ops.drain(p->arg);
}

static int
idequal(Mxpid *a, Mxpid *b)
{
	return a->w[0] == b->w[0] && a->w[1] == b->w[1] &&
		a->w[2] == b->w[2] && a->w[3] == b->w[3];
}

static int
idnonzero(Mxpid *a)
{
	return (a->w[0] | a->w[1] | a->w[2] | a->w[3]) != 0;
}

static Mxpid
readid(Mxp *p, u32int off)
{
	Mxpid id;
	int i;

	for(i = 0; i < 4; i++)
		id.w[i] = rd(p, off + i*4);
	return id;
}

static void
writeid(Mxp *p, u32int off, Mxpid *id)
{
	int i;

	for(i = 0; i < 4; i++)
		wr(p, off + i*4, id->w[i]);
}

static u32int
ownstateoff(Mxp *p)
{
	return p->role == MXP_HOST? MXP_H_STATE: MXP_G_STATE;
}

static u32int
distance(u32int head, u32int tail)
{
	return (head + MXP_RING_BYTES - tail) % MXP_RING_BYTES;
}

static int
offsetvalid(u32int off)
{
	return off < MXP_RING_BYTES && (off & 3) == 0;
}

u32int
mxprecordbytes(u32int length)
{
	return MXP_HEADER_BYTES + ((length + 3) & ~3u);
}

static void
putle32(uchar *d, u32int w)
{
	d[0] = w;
	d[1] = w>>8;
	d[2] = w>>16;
	d[3] = w>>24;
}

static u32int
getle32(uchar *s)
{
	return (u32int)s[0] | (u32int)s[1]<<8 | (u32int)s[2]<<16 | (u32int)s[3]<<24;
}

/* reflected CRC32C (Castagnoli, 0x82f63b78), byte at a time */
static u32int crctab[256] = {
	0x00000000, 0xf26b8303, 0xe13b70f7, 0x1350f3f4, 0xc79a971f, 0x35f1141c, 0x26a1e7e8, 0xd4ca64eb,
	0x8ad958cf, 0x78b2dbcc, 0x6be22838, 0x9989ab3b, 0x4d43cfd0, 0xbf284cd3, 0xac78bf27, 0x5e133c24,
	0x105ec76f, 0xe235446c, 0xf165b798, 0x030e349b, 0xd7c45070, 0x25afd373, 0x36ff2087, 0xc494a384,
	0x9a879fa0, 0x68ec1ca3, 0x7bbcef57, 0x89d76c54, 0x5d1d08bf, 0xaf768bbc, 0xbc267848, 0x4e4dfb4b,
	0x20bd8ede, 0xd2d60ddd, 0xc186fe29, 0x33ed7d2a, 0xe72719c1, 0x154c9ac2, 0x061c6936, 0xf477ea35,
	0xaa64d611, 0x580f5512, 0x4b5fa6e6, 0xb93425e5, 0x6dfe410e, 0x9f95c20d, 0x8cc531f9, 0x7eaeb2fa,
	0x30e349b1, 0xc288cab2, 0xd1d83946, 0x23b3ba45, 0xf779deae, 0x05125dad, 0x1642ae59, 0xe4292d5a,
	0xba3a117e, 0x4851927d, 0x5b016189, 0xa96ae28a, 0x7da08661, 0x8fcb0562, 0x9c9bf696, 0x6ef07595,
	0x417b1dbc, 0xb3109ebf, 0xa0406d4b, 0x522bee48, 0x86e18aa3, 0x748a09a0, 0x67dafa54, 0x95b17957,
	0xcba24573, 0x39c9c670, 0x2a993584, 0xd8f2b687, 0x0c38d26c, 0xfe53516f, 0xed03a29b, 0x1f682198,
	0x5125dad3, 0xa34e59d0, 0xb01eaa24, 0x42752927, 0x96bf4dcc, 0x64d4cecf, 0x77843d3b, 0x85efbe38,
	0xdbfc821c, 0x2997011f, 0x3ac7f2eb, 0xc8ac71e8, 0x1c661503, 0xee0d9600, 0xfd5d65f4, 0x0f36e6f7,
	0x61c69362, 0x93ad1061, 0x80fde395, 0x72966096, 0xa65c047d, 0x5437877e, 0x4767748a, 0xb50cf789,
	0xeb1fcbad, 0x197448ae, 0x0a24bb5a, 0xf84f3859, 0x2c855cb2, 0xdeeedfb1, 0xcdbe2c45, 0x3fd5af46,
	0x7198540d, 0x83f3d70e, 0x90a324fa, 0x62c8a7f9, 0xb602c312, 0x44694011, 0x5739b3e5, 0xa55230e6,
	0xfb410cc2, 0x092a8fc1, 0x1a7a7c35, 0xe811ff36, 0x3cdb9bdd, 0xceb018de, 0xdde0eb2a, 0x2f8b6829,
	0x82f63b78, 0x709db87b, 0x63cd4b8f, 0x91a6c88c, 0x456cac67, 0xb7072f64, 0xa457dc90, 0x563c5f93,
	0x082f63b7, 0xfa44e0b4, 0xe9141340, 0x1b7f9043, 0xcfb5f4a8, 0x3dde77ab, 0x2e8e845f, 0xdce5075c,
	0x92a8fc17, 0x60c37f14, 0x73938ce0, 0x81f80fe3, 0x55326b08, 0xa759e80b, 0xb4091bff, 0x466298fc,
	0x1871a4d8, 0xea1a27db, 0xf94ad42f, 0x0b21572c, 0xdfeb33c7, 0x2d80b0c4, 0x3ed04330, 0xccbbc033,
	0xa24bb5a6, 0x502036a5, 0x4370c551, 0xb11b4652, 0x65d122b9, 0x97baa1ba, 0x84ea524e, 0x7681d14d,
	0x2892ed69, 0xdaf96e6a, 0xc9a99d9e, 0x3bc21e9d, 0xef087a76, 0x1d63f975, 0x0e330a81, 0xfc588982,
	0xb21572c9, 0x407ef1ca, 0x532e023e, 0xa145813d, 0x758fe5d6, 0x87e466d5, 0x94b49521, 0x66df1622,
	0x38cc2a06, 0xcaa7a905, 0xd9f75af1, 0x2b9cd9f2, 0xff56bd19, 0x0d3d3e1a, 0x1e6dcdee, 0xec064eed,
	0xc38d26c4, 0x31e6a5c7, 0x22b65633, 0xd0ddd530, 0x0417b1db, 0xf67c32d8, 0xe52cc12c, 0x1747422f,
	0x49547e0b, 0xbb3ffd08, 0xa86f0efc, 0x5a048dff, 0x8ecee914, 0x7ca56a17, 0x6ff599e3, 0x9d9e1ae0,
	0xd3d3e1ab, 0x21b862a8, 0x32e8915c, 0xc083125f, 0x144976b4, 0xe622f5b7, 0xf5720643, 0x07198540,
	0x590ab964, 0xab613a67, 0xb831c993, 0x4a5a4a90, 0x9e902e7b, 0x6cfbad78, 0x7fab5e8c, 0x8dc0dd8f,
	0xe330a81a, 0x115b2b19, 0x020bd8ed, 0xf0605bee, 0x24aa3f05, 0xd6c1bc06, 0xc5914ff2, 0x37faccf1,
	0x69e9f0d5, 0x9b8273d6, 0x88d28022, 0x7ab90321, 0xae7367ca, 0x5c18e4c9, 0x4f48173d, 0xbd23943e,
	0xf36e6f75, 0x0105ec76, 0x12551f82, 0xe03e9c81, 0x34f4f86a, 0xc69f7b69, 0xd5cf889d, 0x27a40b9e,
	0x79b737ba, 0x8bdcb4b9, 0x988c474d, 0x6ae7c44e, 0xbe2da0a5, 0x4c4623a6, 0x5f16d052, 0xad7d5351,
};

static u32int
crcbyte(u32int crc, uint b)
{
	return (crc >> 8) ^ crctab[(crc ^ b) & 255];
}

u32int
mxpcrc32c(void *v, u32int n)
{
	uchar *s;
	u32int i, crc;

	s = v;
	crc = MXP_CRC32C_INITIAL;
	for(i = 0; i < n; i++)
		crc = crcbyte(crc, s[i]);
	return crc ^ MXP_CRC32C_XOROUT;
}

/* the CRC word itself counts as four zero bytes */
static u32int
recordcrc(uchar *s, u32int n)
{
	u32int i, crc;

	crc = MXP_CRC32C_INITIAL;
	for(i = 0; i < n; i++)
		crc = crcbyte(crc, i >= MXP_R_CRC && i < MXP_R_CRC+4? 0: s[i]);
	return crc ^ MXP_CRC32C_XOROUT;
}

static u32int metaexpect[16] = {
	MXP_PAGE_MAGIC, MXP_ABI_VERSION, MXP_PAGE_BYTES, MXP_META_BYTES,
	MXP_RING_BYTES, MXP_HEADER_BYTES, MXP_MAX_PAYLOAD, MXP_V1_FEATURES,
	0, 0, 0, 0, MXP_H2G_BASE, MXP_G2H_BASE, MXP_GAP_BYTES, 0,
};

/*
 * Immutable header, firmware identity, PAGE_FLAGS, FW_FAULT and the
 * reserved words, twice across ordering.  Nonzero is a fault reason;
 * no caller may then write into what may be a new page generation.
 */
static u32int
metadatafault(Mxp *p)
{
	u32int i, v;

	if(rd(p, MXP_M_FW_FAULT))
		return MXP_FAULT_FIRMWARE;
	if(rd(p, MXP_M_PAGE_FLAGS) != MXP_PAGE_INITIALIZED)
		return MXP_FAULT_METADATA;
	order(p);
	for(i = 0; i < 16; i++){
		v = i >= 8 && i < 12? p->fwid.w[i-8]: metaexpect[i];
		if(rd(p, i*4) != v)
			return MXP_FAULT_METADATA;
	}
	for(i = 0x04c; i < 0x080; i += 4)
		if(rd(p, i))
			return MXP_FAULT_METADATA;
	for(i = 0x0b0; i < 0x0c0; i += 4)
		if(rd(p, i))
			return MXP_FAULT_METADATA;
	for(i = 0x0f0; i < MXP_META_BYTES; i += 4)
		if(rd(p, i))
			return MXP_FAULT_METADATA;
	order(p);
	if(rd(p, MXP_M_FW_FAULT))
		return MXP_FAULT_FIRMWARE;
	if(rd(p, MXP_M_PAGE_FLAGS) != MXP_PAGE_INITIALIZED)
		return MXP_FAULT_METADATA;
	return 0;
}

static void
publishstate(Mxp *p, u32int state)
{
	drain(p);
	wr(p, ownstateoff(p), state);
	drain(p);
	p->localstate = state;
}

/* latch a terminal local fault; publish it only into a proven page */
static int
fail(Mxp *p, u32int reason, int publish)
{
	if(p->localstate != MXP_STATE_FAULT){
		if(publish && p->attached && !metadatafault(p)){
			wr(p, p->role == MXP_HOST? MXP_H_FAULT: MXP_G_FAULT, reason);
			publishstate(p, MXP_STATE_FAULT);
		}
		p->localstate = MXP_STATE_FAULT;
		p->diag.faultreason = reason;
		p->diag.faults++;
	}
	p->txfree = p->rxused = 0;
	p->ready = MXP_READY_ERROR | MXP_READY_HUP;
	return MXP_FAULTED;
}

/* ring helpers take validated cursors and word-rounded bounded lengths */
static u32int
ringword(Mxp *p, u32int base, u32int start, u32int off)
{
	return rd(p, base + (start + off) % MXP_RING_BYTES);
}

static void
ringread(Mxp *p, u32int base, u32int start, uchar *d, u32int n)
{
	u32int i;

	for(i = 0; i < n; i += 4)
		putle32(d + i, ringword(p, base, start, i));
}

static void
ringwrite(Mxp *p, u32int base, u32int start, uchar *s, u32int n)
{
	u32int i;

	for(i = 0; i < n; i += 4)
		wr(p, base + (start + i) % MXP_RING_BYTES, getle32(s + i));
}

/* header only: readiness never asserts payload integrity */
static int
peek(Mxp *p, u32int *length, u32int *bytes)
{
	u32int h[8], i, format, base;

	base = p->role == MXP_HOST? MXP_G2H_BASE: MXP_H2G_BASE;
	if(p->rxused == 0)
		return MXP_AGAIN;
	if(p->rxused < MXP_HEADER_BYTES)
		return fail(p, MXP_FAULT_HEADER, 1);
	for(i = 0; i < 8; i++)
		h[i] = ringword(p, base, p->rxtail, i*4);
	format = h[1];
	*length = format & MXP_FORMAT_LENGTH_MASK;
	if(h[0] != MXP_RECORD_MAGIC || *length < MXP_MIN_PAYLOAD ||
	   *length > MXP_MAX_PAYLOAD ||
	   (format >> MXP_FORMAT_TYPE_SHIFT & 255) != MXP_TYPE_PACKET ||
	   (format >> MXP_FORMAT_VERSION_SHIFT) != MXP_RECORD_VERSION)
		return fail(p, MXP_FAULT_HEADER, 1);
	*bytes = mxprecordbytes(*length);
	if(*bytes > p->rxused)
		return fail(p, MXP_FAULT_HEADER, 1);
	for(i = 0; i < 4; i++)
		if(h[i+4] != p->hostnonce.w[i])
			return fail(p, MXP_FAULT_IDENTITY, 1);
	if(h[2] != p->rxseq)
		return fail(p, MXP_FAULT_SEQUENCE, 1);
	return MXP_OK;
}

/*
 * Read-only preflight of a fresh page, then (host only) OFFER.
 * Success means the context was accepted, not that the session is up.
 * The caller guarantees an exclusive first attachment: reattaching is
 * never a recovery path.
 */
int
mxpattach(Mxp *p, Mxpops *ops, void *arg, int role, Mxpid *localid)
{
	Mxpid id, idcopy, zero;
	Mxpops opscopy;
	u32int hs, gs, map, reason, i;
	uchar *b;

	if(p == nil || ops == nil || ops->read32 == nil || ops->write32 == nil ||
	   ops->order == nil || ops->drain == nil || localid == nil ||
	   !idnonzero(localid) || (role != MXP_HOST && role != MXP_GUEST))
		return MXP_INVALID;
	opscopy = *ops;
	idcopy = *localid;
	for(i = 0; i < 4; i++)
		zero.w[i] = 0;
	b = (uchar*)p;
	for(i = 0; i < sizeof(*p); i++)
		b[i] = 0;
	p->ops = opscopy;
	p->arg = arg;
	p->role = role;
	p->localid = idcopy;
	p->fwid = readid(p, MXP_M_FW_ID);
	reason = metadatafault(p);
	if(reason || !idnonzero(&p->fwid))
		return fail(p, reason? reason: MXP_FAULT_METADATA, 0);
	hs = rd(p, MXP_H_STATE);
	gs = rd(p, MXP_G_STATE);
	order(p);
	map = rd(p, MXP_M_HOST_MAP);
	if(rd(p, MXP_H_TX_HEAD) || rd(p, MXP_H_RX_TAIL) ||
	   rd(p, MXP_G_TX_HEAD) || rd(p, MXP_G_RX_TAIL) ||
	   rd(p, MXP_H_FAULT) || rd(p, MXP_G_FAULT))
		return fail(p, MXP_FAULT_INITIAL, 0);
	id = readid(p, MXP_G_BOOT_ID);
	if(!idequal(&id, &zero))
		return fail(p, MXP_FAULT_INITIAL, 0);
	id = readid(p, MXP_G_HOST_ACK);
	if(!idequal(&id, &zero))
		return fail(p, MXP_FAULT_INITIAL, 0);
	id = readid(p, MXP_H_GUEST_ACK);
	if(!idequal(&id, &zero))
		return fail(p, MXP_FAULT_INITIAL, 0);
	id = readid(p, MXP_H_NONCE);
	order(p);
	/* a racing publication: retry before interpreting staged fields */
	if(hs != rd(p, MXP_H_STATE) || gs != rd(p, MXP_G_STATE))
		return MXP_AGAIN;
	if(gs != MXP_STATE_DOWN ||
	   (role == MXP_HOST && (hs != MXP_STATE_DOWN || map || idnonzero(&id))) ||
	   (role == MXP_GUEST && (map > 1 ||
	    (hs != MXP_STATE_DOWN && hs != MXP_STATE_OFFER) ||
	    (hs == MXP_STATE_OFFER && (!map || !idnonzero(&id))))))
		return fail(p, MXP_FAULT_INITIAL, 0);
	reason = metadatafault(p);
	if(reason)
		return fail(p, reason, 0);
	p->attached = 1;
	p->peerstate = role == MXP_HOST? gs: hs;
	if(role == MXP_HOST){
		p->hostnonce = idcopy;
		wr(p, MXP_M_HOST_MAP, MXP_HOST_MAPPING_READY);
		drain(p);
		writeid(p, MXP_H_NONCE, &p->hostnonce);
		publishstate(p, MXP_STATE_OFFER);
	}else if(hs == MXP_STATE_OFFER)
		p->hostnonce = id;
	return MXP_OK;
}

/* one-way local stop: drain and publish QUIESCING; never clears SRAM */
int
mxpshutdown(Mxp *p)
{
	u32int reason;

	if(p == nil)
		return MXP_INVALID;
	if(p->localstate == MXP_STATE_FAULT)
		return MXP_FAULTED;
	if(!p->attached)
		return MXP_INVALID;
	if(p->localstate == MXP_STATE_QUIESCING)
		return MXP_OK;
	reason = metadatafault(p);
	if(reason)
		return fail(p, reason, 0);
	order(p);
	publishstate(p, MXP_STATE_QUIESCING);
	p->ready = MXP_READY_HUP;
	p->txfree = p->rxused = 0;
	return MXP_OK;
}

/*
 * One bounded handshake/validation step: OK when both ends are UP,
 * NOT_UP while negotiating or after quiesce, AGAIN for an inconsistent
 * concurrent snapshot, FAULTED once a fault has latched.
 */
int
mxpservice(Mxp *p)
{
	Mxpid id, ack, expect, zero;
	u32int reason, ps, pf, map, ph, pt, oldused, used, length, bytes, i;
	u32int peerstateoff, peeridoff, peerackoff, ownidoff, ownackoff;
	int stopped;

	if(p == nil)
		return MXP_INVALID;
	if(p->localstate == MXP_STATE_FAULT)
		return MXP_FAULTED;
	if(!p->attached)
		return MXP_INVALID;
	if(p->localstate == MXP_STATE_QUIESCING)
		return MXP_NOT_UP;
	for(i = 0; i < 4; i++)
		zero.w[i] = 0;
	reason = metadatafault(p);
	if(reason)
		return fail(p, reason, 0);
	if(rd(p, ownstateoff(p)) != p->localstate ||
	   rd(p, p->role == MXP_HOST? MXP_H_FAULT: MXP_G_FAULT))
		return fail(p, MXP_FAULT_STATE, 1);
	map = rd(p, MXP_M_HOST_MAP);
	if(map > 1 || (p->role == MXP_HOST && map != 1))
		return fail(p, MXP_FAULT_STATE, 1);
	ownidoff = p->role == MXP_HOST? MXP_H_NONCE: MXP_G_BOOT_ID;
	ownackoff = p->role == MXP_HOST? MXP_H_GUEST_ACK: MXP_G_HOST_ACK;
	expect = p->localstate == MXP_STATE_DOWN? zero: p->localid;
	id = readid(p, ownidoff);
	if(!idequal(&id, &expect))
		return fail(p, MXP_FAULT_IDENTITY, 1);
	if(p->role == MXP_HOST)
		expect = p->localstate == MXP_STATE_UP? p->guestid: zero;
	else
		expect = p->localstate == MXP_STATE_DOWN? zero: p->hostnonce;
	ack = readid(p, ownackoff);
	if(!idequal(&ack, &expect))
		return fail(p, MXP_FAULT_IDENTITY, 1);
	if(rd(p, p->role == MXP_HOST? MXP_H_TX_HEAD: MXP_G_TX_HEAD) != p->txhead ||
	   rd(p, p->role == MXP_HOST? MXP_H_RX_TAIL: MXP_G_RX_TAIL) != p->rxtail)
		return fail(p, MXP_FAULT_OFFSET, 1);
	peerstateoff = p->role == MXP_HOST? MXP_G_STATE: MXP_H_STATE;
	peeridoff = p->role == MXP_HOST? MXP_G_BOOT_ID: MXP_H_NONCE;
	peerackoff = p->role == MXP_HOST? MXP_G_HOST_ACK: MXP_H_GUEST_ACK;
	ps = rd(p, peerstateoff);
	order(p);
	id = readid(p, peeridoff);
	ack = readid(p, peerackoff);
	pf = rd(p, p->role == MXP_HOST? MXP_G_FAULT: MXP_H_FAULT);
	order(p);
	if(ps != rd(p, peerstateoff)){
		p->ready = 0;
		return MXP_AGAIN;
	}
	ph = rd(p, p->role == MXP_HOST? MXP_G_TX_HEAD: MXP_H_TX_HEAD);
	order(p);
	pt = rd(p, p->role == MXP_HOST? MXP_G_RX_TAIL: MXP_H_RX_TAIL);
	/*
	 * The peer can go ACCEPT->UP and enqueue between the identity
	 * snapshot and the cursor reads: widen the snapshot first.
	 */
	order(p);
	if(ps != rd(p, peerstateoff)){
		p->ready = 0;
		return MXP_AGAIN;
	}
	if(ps > MXP_STATE_FAULT || ps < p->peerstate ||
	   (p->role == MXP_HOST && ps == MXP_STATE_OFFER) ||
	   (p->role == MXP_GUEST && ps == MXP_STATE_ACCEPT))
		return fail(p, MXP_FAULT_STATE, 1);
	p->peerstate = ps;
	if(ps == MXP_STATE_FAULT || pf)
		return fail(p, MXP_FAULT_PEER, 1);
	if(ps == MXP_STATE_QUIESCING){
		stopped = mxpshutdown(p);
		return stopped == MXP_FAULTED? stopped: MXP_NOT_UP;
	}
	/* HOST_MAP may have been sampled before the host's OFFER */
	if(p->role == MXP_GUEST)
		map = rd(p, MXP_M_HOST_MAP);
	if(p->role == MXP_HOST){
		if((p->localstate == MXP_STATE_UP && ps == MXP_STATE_DOWN) ||
		   (p->localstate == MXP_STATE_OFFER && ps == MXP_STATE_UP))
			return fail(p, MXP_FAULT_STATE, 1);
		if(ps == MXP_STATE_ACCEPT || ps == MXP_STATE_UP){
			if(!idnonzero(&id) || !idequal(&ack, &p->hostnonce) ||
			   (idnonzero(&p->guestid) && !idequal(&id, &p->guestid)))
				return fail(p, MXP_FAULT_IDENTITY, 1);
			p->guestid = id;
		}
	}else{
		if((p->localstate != MXP_STATE_DOWN && ps == MXP_STATE_DOWN) ||
		   (p->localstate == MXP_STATE_DOWN && ps == MXP_STATE_UP))
			return fail(p, MXP_FAULT_STATE, 1);
		if(ps == MXP_STATE_OFFER || ps == MXP_STATE_UP){
			if(map != 1 || !idnonzero(&id) ||
			   (idnonzero(&p->hostnonce) && !idequal(&id, &p->hostnonce)) ||
			   (ps == MXP_STATE_UP && !idequal(&ack, &p->localid)))
				return fail(p, MXP_FAULT_IDENTITY, 1);
			p->hostnonce = id;
		}
	}
	if(!offsetvalid(ph) || !offsetvalid(pt) ||
	   (ps != MXP_STATE_UP && (ph || pt)))
		return fail(p, MXP_FAULT_OFFSET, 1);
	/* head/credit cannot move further than the peer was entitled to */
	oldused = distance(p->peerhead, p->rxtail);
	if(oldused > MXP_MAX_USED ||
	   distance(ph, p->peerhead) > MXP_MAX_USED - oldused ||
	   distance(pt, p->peertail) > distance(p->txhead, p->peertail))
		return fail(p, MXP_FAULT_OFFSET, 1);
	used = distance(p->txhead, pt);
	p->rxused = distance(ph, p->rxtail);
	if(used > MXP_MAX_USED || p->rxused > MXP_MAX_USED)
		return fail(p, MXP_FAULT_OFFSET, 1);
	p->peerhead = ph;
	p->peertail = pt;
	p->txfree = MXP_MAX_USED - used;
	reason = metadatafault(p);
	if(reason)
		return fail(p, reason, 0);
	if(p->role == MXP_HOST && p->localstate == MXP_STATE_OFFER && ps == MXP_STATE_ACCEPT){
		writeid(p, MXP_H_GUEST_ACK, &p->guestid);
		publishstate(p, MXP_STATE_UP);
	}else if(p->role == MXP_GUEST && p->localstate == MXP_STATE_DOWN && ps == MXP_STATE_OFFER){
		p->guestid = p->localid;
		writeid(p, MXP_G_BOOT_ID, &p->guestid);
		writeid(p, MXP_G_HOST_ACK, &p->hostnonce);
		publishstate(p, MXP_STATE_ACCEPT);
	}else if(p->role == MXP_GUEST && p->localstate == MXP_STATE_ACCEPT && ps == MXP_STATE_UP)
		publishstate(p, MXP_STATE_UP);
	p->ready = 0;
	if(p->localstate != MXP_STATE_UP || ps != MXP_STATE_UP)
		return MXP_NOT_UP;
	if(p->rxused){
		if(peek(p, &length, &bytes) != MXP_OK)
			return MXP_FAULTED;
		p->ready |= MXP_READY_READ;
	}
	if(p->txfree >= mxprecordbytes(MXP_MIN_PAYLOAD))
		p->ready |= MXP_READY_WRITE;
	return MXP_OK;
}

/* enqueue exactly one packet, or AGAIN with no writes when full */
int
mxpsend(Mxp *p, void *payload, u32int length)
{
	uchar *s;
	u32int i, bytes, base, head;
	int r;

	s = payload;
	if(p == nil || payload == nil || length < MXP_MIN_PAYLOAD || length > MXP_MAX_PAYLOAD)
		return MXP_INVALID;
	r = mxpservice(p);
	if(r != MXP_OK)
		return r;
	bytes = mxprecordbytes(length);
	if(p->txfree < bytes){
		p->diag.txfull++;
		return MXP_AGAIN;
	}
	for(i = 0; i < bytes; i++)
		p->scratch[i] = 0;
	putle32(p->scratch + MXP_R_MAGIC, MXP_RECORD_MAGIC);
	putle32(p->scratch + MXP_R_FORMAT, length |
		MXP_TYPE_PACKET << MXP_FORMAT_TYPE_SHIFT |
		MXP_RECORD_VERSION << MXP_FORMAT_VERSION_SHIFT);
	putle32(p->scratch + MXP_R_SEQUENCE, p->txseq);
	for(i = 0; i < 4; i++)
		putle32(p->scratch + MXP_R_SESSION + i*4, p->hostnonce.w[i]);
	for(i = 0; i < length; i++)
		p->scratch[MXP_HEADER_BYTES + i] = s[i];
	putle32(p->scratch + MXP_R_CRC, recordcrc(p->scratch, bytes));
	base = p->role == MXP_HOST? MXP_H2G_BASE: MXP_G2H_BASE;
	ringwrite(p, base, p->txhead, p->scratch, bytes);
	drain(p);
	/* a failure here leaves only unpublished data in free space */
	r = mxpservice(p);
	if(r != MXP_OK)
		return r;
	head = (p->txhead + bytes) % MXP_RING_BYTES;
	wr(p, p->role == MXP_HOST? MXP_H_TX_HEAD: MXP_G_TX_HEAD, head);
	drain(p);
	p->txhead = head;
	p->txfree -= bytes;
	p->txseq++;
	p->diag.txpackets++;
	p->diag.txbytes += length;
	if(p->txfree < mxprecordbytes(MXP_MIN_PAYLOAD))
		p->ready &= ~MXP_READY_WRITE;
	return MXP_OK;
}

/*
 * Copy and verify one record before returning its credit.  Integrity
 * and state faults discard the scratch copy, never the credit.
 */
int
mxprecv(Mxp *p, void *payload, u32int cap, u32int *length)
{
	uchar *d;
	u32int len, bytes, i, base, tail;
	int r;

	d = payload;
	if(p == nil || length == nil || (payload == nil && cap != 0))
		return MXP_INVALID;
	*length = 0;
	r = mxpservice(p);
	if(r != MXP_OK)
		return r;
	r = peek(p, &len, &bytes);
	if(r == MXP_AGAIN){
		p->diag.rxempty++;
		return r;
	}
	if(r != MXP_OK)
		return r;
	if(cap < len){
		*length = len;
		p->diag.rxshort++;
		return MXP_SHORT;
	}
	base = p->role == MXP_HOST? MXP_G2H_BASE: MXP_H2G_BASE;
	ringread(p, base, p->rxtail, p->scratch, bytes);
	if(getle32(p->scratch + MXP_R_MAGIC) != MXP_RECORD_MAGIC ||
	   getle32(p->scratch + MXP_R_FORMAT) != (len |
	    MXP_TYPE_PACKET << MXP_FORMAT_TYPE_SHIFT |
	    MXP_RECORD_VERSION << MXP_FORMAT_VERSION_SHIFT))
		return fail(p, MXP_FAULT_HEADER, 1);
	if(getle32(p->scratch + MXP_R_SEQUENCE) != p->rxseq)
		return fail(p, MXP_FAULT_SEQUENCE, 1);
	for(i = 0; i < 4; i++)
		if(getle32(p->scratch + MXP_R_SESSION + i*4) != p->hostnonce.w[i])
			return fail(p, MXP_FAULT_IDENTITY, 1);
	if(getle32(p->scratch + MXP_R_CRC) != recordcrc(p->scratch, bytes))
		return fail(p, MXP_FAULT_CRC, 1);
	for(i = MXP_HEADER_BYTES + len; i < bytes; i++)
		if(p->scratch[i] != 0)
			return fail(p, MXP_FAULT_HEADER, 1);
	r = mxpservice(p);
	if(r != MXP_OK)
		return r;
	for(i = 0; i < len; i++)
		d[i] = p->scratch[MXP_HEADER_BYTES + i];
	order(p);
	tail = (p->rxtail + bytes) % MXP_RING_BYTES;
	wr(p, p->role == MXP_HOST? MXP_H_RX_TAIL: MXP_G_RX_TAIL, tail);
	drain(p);
	p->rxtail = tail;
	p->rxused -= bytes;
	p->rxseq++;
	p->diag.rxpackets++;
	p->diag.rxbytes += len;
	*length = len;
	if(p->rxused == 0)
		p->ready &= ~MXP_READY_READ;
	return MXP_OK;
}

u32int
mxpreadiness(Mxp *p, u32int next)
{
	int r;

	r = mxpservice(p);
	if(p == nil)
		return MXP_READY_ERROR | MXP_READY_HUP;
	if(r != MXP_OK)
		return p->ready;
	p->ready &= ~MXP_READY_WRITE;
	if(next >= MXP_MIN_PAYLOAD && next <= MXP_MAX_PAYLOAD &&
	   p->txfree >= mxprecordbytes(next))
		p->ready |= MXP_READY_WRITE;
	return p->ready;
}

/* local snapshots only: no mailbox access */
void
mxpgetstatus(Mxp *p, Mxpstatus *s)
{
	if(p == nil || s == nil)
		return;
	s->localstate = p->localstate;
	s->peerstate = p->peerstate;
	s->faultreason = p->diag.faultreason;
	s->ready = p->ready;
	s->txfree = p->txfree;
	s->rxused = p->rxused;
	s->txseq = p->txseq;
	s->rxseq = p->rxseq;
	s->fwid = p->fwid;
	s->hostnonce = p->hostnonce;
	s->guestid = p->guestid;
}

void
mxpgetdiag(Mxp *p, Mxpdiag *d)
{
	if(p != nil && d != nil)
		*d = p->diag;
}
