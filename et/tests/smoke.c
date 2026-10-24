#include <u.h>
#include <libc.h>

void
main(void)
{
	char buf[64];
	char *p;
	int fd, n;
	double x;

	if(sizeof(void*) != 8 || sizeof(vlong) != 8)
		sysfatal("not a 64-bit target");
	p = malloc(8192);
	if(p == nil)
		sysfatal("malloc: %r");
	memset(p, 0x5a, 8192);
	memmove(p+1, p, 8191);
	if(p[8191] != 0x5a)
		sysfatal("memmove");
	free(p);
	x = strtod("1.25", nil);
	if(x*4.0 != 5.0)
		sysfatal("floating point");
	fd = create("persist.txt", ORDWR, 0664);
	if(fd < 0)
		sysfatal("create: %r");
	p = "9front riscv64 disk persistence\n";
	n = strlen(p);
	if(write(fd, p, n) != n || seek(fd, 0, 0) != 0)
		sysfatal("write/seek: %r");
	if(readn(fd, buf, n) != n || memcmp(buf, p, n) != 0)
		sysfatal("readback: %r");
	close(fd);
	print("PASS: native riscv64, allocation, memmove, floating point, disk I/O\n");
	exits(nil);
}
