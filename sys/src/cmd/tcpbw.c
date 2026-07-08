/*
 * tcpbw - minimal TCP throughput meter, native Plan 9 (no APE).
 *
 * Streams zeroes over one TCP connection and reports the rate, to
 * measure the raw stack+driver path with no 9P round trips in the way.
 *
 *	tcpbw -c host [-p port] [-t sec] [-b kbytes]	send to host
 *	tcpbw -s        [-p port]        [-b kbytes]	receive, print rate
 *
 * Interoperates with iperf2: a plain stream is what iperf's flags==0
 * path expects, so `iperf -s` measures our -c, and `iperf -c` feeds our
 * -s.  Also talks to another tcpbw, so it needs nothing on the peer.
 *
 *	linux$ iperf -s			bpi# tcpbw -c linux -t 10   (upload)
 *	bpi#   tcpbw -s			linux$ iperf -c bpi -t 10   (download)
 */
#include <u.h>
#include <libc.h>

enum { Maxbuf = 1024*1024 };
static char buf[Maxbuf];

static void
usage(void)
{
	fprint(2, "usage: tcpbw -c host [-p port] [-t sec] [-b kbytes]\n");
	fprint(2, "       tcpbw -s [-p port] [-b kbytes]\n");
	exits("usage");
}

/* rate of nb bytes over dns nanoseconds, in MB/s (1e6) */
static double
rate(vlong nb, vlong dns)
{
	if(dns <= 0)
		dns = 1;
	return (double)nb / ((double)dns/1e9) / 1e6;
}

static void
report(char *tag, vlong nb, vlong dns)
{
	double s = (double)dns/1e9;

	fprint(2, "%s: %lld bytes in %.2f s = %.1f Mbit/s (%.1f MB/s)\n",
		tag, nb, s, rate(nb, dns)*8.0, rate(nb, dns));
}

void
main(int argc, char **argv)
{
	char *host, *port;
	int server, bufsz, fd, n;
	double secs;
	vlong tot, t0, tend, now, lastt, lastb;

	host = nil;
	port = "5001";
	server = 0;
	bufsz = 64*1024;
	secs = 10.0;

	ARGBEGIN{
	case 's': server = 1; break;
	case 'c': server = 0; host = EARGF(usage()); break;
	case 'p': port = EARGF(usage()); break;
	case 't': secs = atof(EARGF(usage())); break;
	case 'b': bufsz = atoi(EARGF(usage()))*1024; break;
	default:  usage();
	}ARGEND;
	if(bufsz <= 0 || bufsz > Maxbuf)
		bufsz = Maxbuf;

	if(server){
		char adir[40], ldir[40], addr[64];
		int acfd, lcfd, dfd;

		snprint(addr, sizeof addr, "tcp!*!%s", port);
		if((acfd = announce(addr, adir)) < 0)
			sysfatal("announce %s: %r", addr);
		USED(acfd);	/* held open to keep the announcement up */
		fprint(2, "tcpbw: listening on %s\n", addr);
		for(;;){
			if((lcfd = listen(adir, ldir)) < 0)
				sysfatal("listen: %r");
			if((dfd = accept(lcfd, ldir)) < 0){
				close(lcfd);
				continue;
			}
			tot = 0;
			t0 = lastt = lastb = 0;
			while((n = read(dfd, buf, bufsz)) > 0){
				now = nsec();
				if(t0 == 0)
					t0 = lastt = now;
				tot += n;
				if(now - lastt >= 1000000000LL){
					fprint(2, "  %6.1f MB/s\n", rate(tot-lastb, now-lastt));
					lastt = now;
					lastb = tot;
				}
			}
			now = nsec();
			report("recv", tot, t0? now-t0: 0);
			close(dfd);
			close(lcfd);
		}
	}else{
		if(host == nil)
			usage();
		if((fd = dial(netmkaddr(host, "tcp", port), 0, 0, 0)) < 0)
			sysfatal("dial %s!%s: %r", host, port);
		tot = 0;
		t0 = lastt = nsec();
		lastb = 0;
		tend = t0 + (vlong)(secs*1e9);
		while((now = nsec()) < tend){
			if((n = write(fd, buf, bufsz)) <= 0)
				break;
			tot += n;
			if(now - lastt >= 1000000000LL){
				fprint(2, "  %6.1f MB/s\n", rate(tot-lastb, now-lastt));
				lastt = now;
				lastb = tot;
			}
		}
		close(fd);
		report("send", tot, nsec()-t0);
	}
	exits(nil);
}
