/*
 * Maxion PC<->MX packet mailbox page and its one-boot bootstrap, from
 * the packet@ node and /chosen (et-platform PacketDriver.md), checked
 * by mxdt.c.  Nothing may touch the page unless all of it was
 * supplied and is consistent.
 */
typedef struct Dtbmxp Dtbmxp;

struct Dtbmxp {
	uvlong	base;		/* page physical base */
	int	ok;		/* the page may be used */
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
 * mtime; devmxrtc.c refuses it if the counter is behind it.
 */
typedef struct Dtbrtc Dtbrtc;

struct Dtbrtc {
	int	ok;
	uvlong	mtime;		/* counter value at ns */
	vlong	ns;		/* Unix ns when the counter read mtime */
	int	haserr;
	uvlong	err;		/* ainekko,rtc-error-ns */
};

Dtbrtc	*dtbrtc(void);

void	maxiondt(void);
