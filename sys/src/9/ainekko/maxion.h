/*
 * Maxion PC<->MX packet mailbox page and its one-boot bootstrap, from
 * the packet@ node and /chosen (et-platform PacketDriver.md), collected
 * by boarddtprop.  Nothing may touch the page unless all of it was
 * supplied and is consistent.
 */
typedef struct Dtbmxp Dtbmxp;

struct Dtbmxp {
	char	path[64];	/* packet node, e.g. "/packet@30001000" */
	uvlong	base;		/* page physical base */
	uvlong	size;
	int	nnodes;		/* packet nodes seen; exactly one is usable */
	int	compat;		/* compatible = "esperanto,maxion-packet-v1" */
	int	hasreg;
	int	hasstatus;
	int	ok;		/* status = "okay" */
	int	inited;		/* esperanto,mxpacket-page-initialized */
	int	abi;		/* esperanto,mxpacket-abi; -1 if absent */
	int	hasfwid;
	u32int	fwid[4];	/* esperanto,mxpacket-fw-id */
	int	nseed;
	uchar	seed[32];	/* rng-seed (host entropy) */
};

Dtbmxp	*dtbmxp(void);

/*
 * One host sample of the CLINT mtime counter against Unix time, from
 * /chosen ainekko,rtc-sample = <mtime-hi mtime-lo ns-hi ns-lo> and the
 * optional ainekko,rtc-error-ns = <ns> (half the host's bracketing
 * round trip).  It stays valid until an uncore (cold) reset restarts
 * mtime; devrtc.c refuses it if the counter is behind it.
 */
typedef struct Dtbrtc Dtbrtc;

struct Dtbrtc {
	int	nsample;	/* ainekko,rtc-sample properties seen */
	int	ok;		/* the last was 16 bytes */
	uvlong	mtime;		/* counter value at ns */
	vlong	ns;		/* Unix ns when the counter read mtime */
	int	haserr;
	uvlong	err;		/* ainekko,rtc-error-ns */
};

Dtbrtc	*dtbrtc(void);
