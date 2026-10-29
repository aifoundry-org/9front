#include <u.h>
#include <libc.h>

void
main(void)
{
	uchar *p;
	ulong i, n;
	int pid, fd;
	char buf[64], *text;
	Waitmsg *w;
	double x;
	vlong t;

	n = 16*1024*1024;
	p = malloc(n);
	if(p == nil)
		sysfatal("malloc: %r");
	for(i = 0; i < n; i++)
		p[i] = (i*17 + 0xa5) & 255;
	for(i = 0; i < n; i++)
		if(p[i] != ((i*17 + 0xa5) & 255))
			sysfatal("memory mismatch at %lud", i);
	pid = fork();
	if(pid < 0)
		sysfatal("fork: %r");
	if(pid == 0){
		p[0] = 0;
		exits(nil);
	}
	w = wait();
	if(w == nil || w->pid != pid || w->msg[0] != 0 || p[0] != 0xa5)
		sysfatal("fork/wait or address-space isolation");
	free(w);
	free(p);
	x = strtod("1.25", nil);
	if(x*4.0 != 5.0)
		sysfatal("floating point");
	text = "9front Maxion volatile RAM filesystem\n";
	fd = create("/tmp/maxion-test", ORDWR, 0600);
	if(fd < 0 || write(fd, text, strlen(text)) != strlen(text) ||
	   seek(fd, 0, 0) != 0 || readn(fd, buf, strlen(text)) != strlen(text) ||
	   memcmp(buf, text, strlen(text)) != 0)
		sysfatal("RAM filesystem: %r");
	close(fd);
	t = nsec();
	sleep(100);
	t = nsec() - t;
	if(t < 80000000LL || t > 2000000000LL)
		sysfatal("timer interval %lld ns", t);
	print("MAXION-SMOKE-PASS: 16MiB memory, fork isolation, FPU, RAM files, timer %lld ns\n", t);
	exits(nil);
}
