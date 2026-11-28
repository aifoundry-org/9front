/*
 * Maxion PC<->MX packet mailbox v1 ("MXP").
 *
 * Plan 9 C port of the shared endpoint core in et-platform
 * et-common-libs/include/transports/mxpacket/{abi,api,core}.h
 * (GPL-2.0-only OR BSD-2-Clause); protocol.md there is the contract.
 * Constants keep their ABI names.  The core knows only byte offsets
 * inside the 4096-byte page; all accesses are aligned 32-bit words
 * through Mxpops, never a structure overlay or a peer atomic.
 */

#define MXP_ABI_VERSION		1u
#define MXP_PAGE_MAGIC		0x3150584du	/* "MXP1" */
#define MXP_RECORD_MAGIC	0x3152584du	/* "MXR1" */
#define MXP_PAGE_BYTES		4096u
#define MXP_META_BYTES		512u
#define MXP_RING_BYTES		1792u
#define MXP_HEADER_BYTES	32u
#define MXP_MIN_PAYLOAD		1u
#define MXP_MAX_PAYLOAD		1500u
#define MXP_GAP_BYTES		4u
#define MXP_MAX_USED		1788u
#define MXP_MAX_RECORD		1532u
#define MXP_H2G_BASE		0x200u
#define MXP_G2H_BASE		0x900u
#define MXP_V1_FEATURES		1u	/* CRC32C */

/* firmware-owned metadata */
#define MXP_M_MAGIC		0x000u
#define MXP_M_FW_ID		0x020u	/* four words */
#define MXP_M_PAGE_FLAGS	0x040u
#define MXP_M_HOST_MAP		0x044u
#define MXP_M_FW_FAULT		0x048u
#define MXP_PAGE_INITIALIZED	1u
#define MXP_HOST_MAPPING_READY	1u

/* endpoint-owned words */
#define MXP_H_STATE		0x080u
#define MXP_H_NONCE		0x084u	/* four words */
#define MXP_H_GUEST_ACK		0x094u	/* four words */
#define MXP_H_TX_HEAD		0x0a4u
#define MXP_H_RX_TAIL		0x0a8u
#define MXP_H_FAULT		0x0acu
#define MXP_G_STATE		0x0c0u
#define MXP_G_BOOT_ID		0x0c4u	/* four words */
#define MXP_G_HOST_ACK		0x0d4u	/* four words */
#define MXP_G_TX_HEAD		0x0e4u
#define MXP_G_RX_TAIL		0x0e8u
#define MXP_G_FAULT		0x0ecu

#define MXP_STATE_DOWN		0u
#define MXP_STATE_OFFER		1u	/* host only */
#define MXP_STATE_ACCEPT	2u	/* guest only */
#define MXP_STATE_UP		3u
#define MXP_STATE_QUIESCING	4u	/* terminal */
#define MXP_STATE_FAULT		5u	/* terminal */

#define MXP_FAULT_NONE		0u
#define MXP_FAULT_METADATA	1u
#define MXP_FAULT_STATE		2u
#define MXP_FAULT_IDENTITY	3u
#define MXP_FAULT_OFFSET	4u
#define MXP_FAULT_HEADER	5u
#define MXP_FAULT_CRC		6u
#define MXP_FAULT_SEQUENCE	7u
#define MXP_FAULT_PEER		8u
#define MXP_FAULT_INITIAL	9u
#define MXP_FAULT_FIRMWARE	10u

/* record header, relative to the ring cursor */
#define MXP_R_MAGIC		0u
#define MXP_R_FORMAT		4u	/* length | type<<16 | version<<24 */
#define MXP_R_SEQUENCE		8u
#define MXP_R_CRC		12u
#define MXP_R_SESSION		16u	/* host nonce, four words */
#define MXP_RECORD_VERSION	1u
#define MXP_TYPE_PACKET		1u
#define MXP_FORMAT_LENGTH_MASK	0xffffu
#define MXP_FORMAT_TYPE_SHIFT	16
#define MXP_FORMAT_VERSION_SHIFT	24
#define MXP_CRC32C_INITIAL	0xffffffffu
#define MXP_CRC32C_XOROUT	0xffffffffu

/* roles and results; values match api.h */
enum {
	MXP_HOST = 0,
	MXP_GUEST = 1,

	MXP_OK = 0,
	MXP_AGAIN,
	MXP_SHORT,
	MXP_NOT_UP,
	MXP_INVALID,
	MXP_FAULTED,

	MXP_READY_READ = 1,
	MXP_READY_WRITE = 2,
	MXP_READY_ERROR = 4,
	MXP_READY_HUP = 8,
};

typedef struct Mxpid Mxpid;
typedef struct Mxpops Mxpops;
typedef struct Mxpdiag Mxpdiag;
typedef struct Mxpstatus Mxpstatus;
typedef struct Mxp Mxp;

struct Mxpid
{
	u32int	w[4];
};

/*
 * read32/write32 move host-order values of little-endian wire words.
 * order is a full I/O and memory barrier.  drain completes every
 * earlier mailbox write at the peer-visible SRAM (barrier, safe
 * same-page readback, barrier).  None can fail.
 */
struct Mxpops
{
	u32int	(*read32)(void*, u32int);
	void	(*write32)(void*, u32int, u32int);
	void	(*order)(void*);
	void	(*drain)(void*);
};

struct Mxpdiag
{
	uvlong	txpackets;
	uvlong	rxpackets;
	uvlong	txbytes;
	uvlong	rxbytes;
	uvlong	txfull;
	uvlong	rxempty;
	uvlong	rxshort;
	uvlong	faults;
	u32int	faultreason;
};

struct Mxpstatus
{
	u32int	localstate;
	u32int	peerstate;
	u32int	faultreason;
	u32int	ready;
	u32int	txfree;
	u32int	rxused;
	u32int	txseq;
	u32int	rxseq;
	Mxpid	fwid;
	Mxpid	hostnonce;
	Mxpid	guestid;
};

/* one externally serialized context per endpoint; treat as opaque */
struct Mxp
{
	Mxpops	ops;
	void	*arg;
	int	role;
	u32int	attached;
	u32int	localstate;
	u32int	peerstate;
	u32int	ready;
	Mxpid	fwid;
	Mxpid	hostnonce;
	Mxpid	guestid;
	Mxpid	localid;
	u32int	txhead;
	u32int	rxtail;
	u32int	peerhead;
	u32int	peertail;
	u32int	txfree;
	u32int	rxused;
	u32int	txseq;
	u32int	rxseq;
	Mxpdiag	diag;
	uchar	scratch[MXP_MAX_RECORD];
};

int	mxpattach(Mxp*, Mxpops*, void*, int, Mxpid*);
int	mxpservice(Mxp*);
int	mxpsend(Mxp*, void*, u32int);
int	mxprecv(Mxp*, void*, u32int, u32int*);
u32int	mxpreadiness(Mxp*, u32int);
void	mxpgetstatus(Mxp*, Mxpstatus*);
void	mxpgetdiag(Mxp*, Mxpdiag*);
int	mxpshutdown(Mxp*);
u32int	mxpcrc32c(void*, u32int);
u32int	mxprecordbytes(u32int);
