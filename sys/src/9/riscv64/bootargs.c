/*
 * boot-time configuration from the DTB passed in by SBI.
 *
 * The previous-stage firmware (OpenSBI under qemu, U-Boot/OpenSBI on hardware)
 * passes us hartid in a0 and a pointer to a flattened device tree in a1.  
 * l.s stashes that pointer in `dtbphys`.  We scan the DTB's /chosen/bootargs
 * property for a plan9.ini-style "key=value key=value ..." string and stuff
 * each pair into conf[].  bootrc and friends then see them as ordinary kernel
 * env via getconf()/setconfenv().
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

#define	MAXCONF		64

/*
 * Written by _start in l.s with the DTB physical address passed in a1.
 * Initialized non-zero so the linker places it in .data, not .bss :
 * otherwise main()'s `memset(edata, 0, end-edata)` would wipe the value
 * after _start has already written it.  -1 as uintptr is also a sentinel
 * bootargsinit() uses to detect "_start never wrote anything here".
 */
uintptr dtbphys = -1;

static char *confname[MAXCONF];
static char *confval[MAXCONF];
static int nconf;
static char bootargsbuf[BOOTARGSLEN];

static int
findconf(char *k)
{
	int i;

	for(i = 0; i < nconf; i++)
		if(cistrcmp(confname[i], k) == 0)
			return i;
	return -1;
}

static void
addconf(char *k, char *v)
{
	int i;

	i = findconf(k);
	if(i < 0){
		if(nconf >= MAXCONF)
			return;
		i = nconf++;
		confname[i] = k;
	}
	confval[i] = v;
}

/*
 * parse a plan9.ini-style buffer into conf[].  cmdline == 1 means
 * "tokenize on whitespace" (the DTB /chosen/bootargs flavor);
 * cmdline == 0 means "one key=value per line" (an embedded plan9.ini).
 */
static void
plan9iniinit(char *s, int cmdline)
{
	char *toks[MAXCONF];
	int i, c, n;
	char *v;

	if((c = *s) < ' ' || c >= 0x80)
		return;
	if(cmdline)
		n = tokenize(s, toks, MAXCONF);
	else
		n = getfields(s, toks, MAXCONF, 1, "\n");
	for(i = 0; i < n; i++){
		if(toks[i][0] == '#')
			continue;
		v = strchr(toks[i], '=');
		if(v == nil)
			continue;
		*v++ = '\0';
		addconf(toks[i], v);
	}
}

typedef struct Devtree Devtree;
struct Devtree
{
	uchar	*base;
	uchar	*end;
	char	*stab;
	char	path[1024];
};

enum {
	DtHeader	= 0xd00dfeed,
	DtBeginNode	= 1,
	DtEndNode	= 2,
	DtProp		= 3,
	DtEnd		= 9,
};

static u32int
beget4(uchar *p)
{
	return (u32int)p[0]<<24 | (u32int)p[1]<<16 | (u32int)p[2]<<8 | (u32int)p[3];
}

static void
devtreeprop(char *path, char *key, void *val, int len)
{
	if(strncmp(path, "/chosen", 7) == 0 && strcmp(key, "bootargs") == 0){
		if(len > BOOTARGSLEN-1)
			len = BOOTARGSLEN-1;
		memmove(bootargsbuf, val, len);
		bootargsbuf[len] = '\0';
		plan9iniinit(bootargsbuf, 1);
		return;
	}
}

static uchar*
devtreenode(Devtree *t, uchar *p, char *cp)
{
	uchar *e = (uchar*)t->stab;
	char *s;
	int n;

	if(p+4 > e || beget4(p) != DtBeginNode)
		return nil;
	p += 4;
	if((s = memchr((char*)p, 0, e - p)) == nil)
		return nil;
	n = s - (char*)p;
	cp += n;
	if(cp >= &t->path[sizeof(t->path)])
		return nil;
	memmove(cp - n, (char*)p, n);
	*cp = 0;
	p += (n + 4) & ~3;
	while(p+12 <= e && beget4(p) == DtProp){
		n = beget4(p+4);
		if(p + 12 + n > e)
			return nil;
		s = t->stab + beget4(p+8);
		if(s < t->stab || s >= (char*)t->end
		|| memchr(s, 0, (char*)t->end - s) == nil)
			return nil;
		devtreeprop(t->path, s, p+12, n);
		p += 12 + ((n + 3) & ~3);
	}
	while(p+4 <= e && beget4(p) == DtBeginNode){
		*cp = '/';
		p = devtreenode(t, p, cp+1);
		if(p == nil)
			return nil;
	}
	if(p+4 > e || beget4(p) != DtEndNode)
		return nil;
	return p+4;
}

static int
parsedevtree(uchar *base)
{
	Devtree t[1];
	u32int total;

	if(base == nil || beget4(base) != DtHeader)
		return -1;
	total = beget4(base+4);
	if(total < 28)
		return -1;
	t->base = base;
	t->end = t->base + total;
	t->stab = (char*)base + beget4(base+12);
	if(t->stab >= (char*)t->end)
		return -1;
	devtreenode(t, base + beget4(base+8), t->path);
	return 0;
}

void
bootargsinit(void)
{
	if(dtbphys == 0 || dtbphys == (uintptr)-1){
		print("bootargsinit: no DTB pointer from firmware\n");
		return;
	}
	if(parsedevtree((uchar*)dtbphys) < 0){
		print("bootargsinit: no usable DTB at %#p\n", (void*)dtbphys);
		return;
	}
	print("bootargsinit: %d entries from DTB /chosen/bootargs\n", nconf);
}

char*
getconf(char *name)
{
	int i;

	if((i = findconf(name)) < 0)
		return nil;
	return confval[i];
}

void
setconfenv(void)
{
	int i;

	for(i = 0; i < nconf; i++){
		if(confname[i][0] != '*')
			ksetenv(confname[i], confval[i], 0);
		ksetenv(confname[i], confval[i], 1);
	}
}
