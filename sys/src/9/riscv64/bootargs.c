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
static char maxmem[32];
static int bootargs_set = 0;
static int preloadedini = 0;
char *bootconf;			/* BOOTARGS; see mem.h */

/*
 * Per-UART information harvested from the DTB.  Filled in by devtreeprop()
 * as it walks uart-like nodes (serial@..., uart@..., r_uart...).  After
 * the whole DTB is parsed, applyaliases() correlates them with the
 * /aliases section so serial0 → eia0, etc., and resolvedtbconsole()
 * picks the entry referenced by /chosen/stdout-path.
 */
#define MAXDTBUARTS 16
#define MAXDTBETHERS 4

static Dtbuart dtbuarts[MAXDTBUARTS];
static int ndtbuarts;
static Dtbether dtbethers[MAXDTBETHERS];
static int ndtbethers;
static char dtbalias[MAXDTBUARTS][64];	/* serial<N> → DT node path */
static char dtbethalias[MAXDTBETHERS][64]; /* ethernet<N> → DT node path */
static char dtbstdout[64];
static int dtbhasplic_;			/* root interrupt controller is a PLIC */

/*
 * Interrupt-controller node candidates harvested during the DTB walk.
 * Properties of a node can appear in any order, and the property we
 * use to recognise a PLIC ("compatible" = "riscv,plic*") may show up
 * before or after the "reg" we want to read; so we cache reg for any
 * node whose name shape is "interrupt-controller@..." or "plic@..."
 * and resolve the PLIC at end-of-parse.
 */
#define MAXDTBINTC 4
typedef struct Dtbintc Dtbintc;
struct Dtbintc {
	char	path[64];
	uvlong	base;
	uvlong	size;
};
static Dtbintc dtbintc[MAXDTBINTC];
static int ndtbintc;
static char dtbplicpath[64];		/* path of the node whose compatible = riscv,plic* */

int
dtbnuart(void)
{
	return ndtbuarts;
}

Dtbuart*
dtbuart(int i)
{
	if(i < 0 || i >= ndtbuarts)
		return nil;
	return &dtbuarts[i];
}

int
dtbhasplic(void)
{
	return dtbhasplic_;
}

int
dtbnether(void)
{
	return ndtbethers;
}

Dtbether*
dtbether(int i)
{
	if(i < 0 || i >= ndtbethers)
		return nil;
	return &dtbethers[i];
}

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

static int
isuartnode(char *path)
{
	char *base;

	base = strrchr(path, '/');
	if(base == nil)
		return 0;
	base++;
	return strncmp(base, "serial@", 7) == 0
	    || strncmp(base, "uart@", 5) == 0
	    || strncmp(base, "r_uart", 6) == 0;
}

static int
isethernetnode(char *path)
{
	char *base;

	base = strrchr(path, '/');
	if(base == nil)
		return 0;
	base++;
	/* A grandchild like mdio-bus/phy@0 must NOT match. */
	return strncmp(base, "ethernet@", 9) == 0;
}

static int
isintcnode(char *path)
{
	char *base;

	base = strrchr(path, '/');
	if(base == nil)
		return 0;
	base++;
	/*
	 * On K1 the PLIC node is named interrupt-controller@<addr>;
	 * other RISC-V boards prefer plic@<addr>.  Accept either.
	 */
	return strncmp(base, "interrupt-controller@", 21) == 0
	    || strncmp(base, "plic@", 5) == 0;
}

static Dtbintc*
findorallocintc(char *path)
{
	int i, n;
	Dtbintc *ic;

	for(i = 0; i < ndtbintc; i++)
		if(strcmp(dtbintc[i].path, path) == 0)
			return &dtbintc[i];
	if(ndtbintc >= MAXDTBINTC)
		return nil;
	ic = &dtbintc[ndtbintc++];
	memset(ic, 0, sizeof(*ic));
	n = strlen(path);
	if(n >= (int)sizeof(ic->path))
		n = sizeof(ic->path) - 1;
	memmove(ic->path, path, n);
	ic->path[n] = 0;
	return ic;
}

static Dtbether*
findorallocether(char *path)
{
	int i, n;
	Dtbether *e;

	for(i = 0; i < ndtbethers; i++)
		if(strcmp(dtbethers[i].path, path) == 0)
			return &dtbethers[i];
	if(ndtbethers >= MAXDTBETHERS)
		return nil;
	e = &dtbethers[ndtbethers++];
	memset(e, 0, sizeof(*e));
	n = strlen(path);
	if(n >= (int)sizeof(e->path))
		n = sizeof(e->path) - 1;
	memmove(e->path, path, n);
	e->path[n] = 0;
	e->irq = -1;
	e->aliasidx = -1;
	e->ok = 1;		/* per DT spec §2.3.4, absent status = enabled */
	e->txphase = -1;
	e->rxphase = -1;
	e->rstpin = -1;
	return e;
}

static Dtbuart*
findorallocuart(char *path)
{
	int i, n;
	Dtbuart *u;

	for(i = 0; i < ndtbuarts; i++)
		if(strcmp(dtbuarts[i].path, path) == 0)
			return &dtbuarts[i];
	if(ndtbuarts >= MAXDTBUARTS)
		return nil;
	u = &dtbuarts[ndtbuarts++];
	memset(u, 0, sizeof(*u));
	n = strlen(path);
	if(n >= (int)sizeof(u->path))
		n = sizeof(u->path) - 1;
	memmove(u->path, path, n);
	u->path[n] = 0;
	u->irq = -1;
	u->regshift = 0;	/* ns16550 binding default; PXA/DW APB set reg-shift=<2> */
	u->aliasidx = -1;
	u->ok = 1;		/* per DT spec §2.3.4, absent status = enabled */
	return u;
}

static void
devtreeprop(char *path, char *key, void *val, int len)
{
	uvlong addr;
	uchar *p = val;

	if((strncmp(path, "/memory", 7) == 0 || strncmp(path, "/memory@0", 9) == 0)
	   && strcmp(key, "reg") == 0){
		if(findconf("*maxmem") < 0 && len == 16){
			/* <base size>, two BE cells each: limit = base + size */
			addr = (uvlong)beget4(p)<<32 | beget4(p+4);
			addr += (uvlong)beget4(p+8)<<32 | beget4(p+12);
			snprint(maxmem, sizeof(maxmem), "%#llux", addr);
			print("devtreeprop: maxmem %s\n", maxmem);
			addconf("*maxmem", maxmem);
		}
		return;
	}
	if(strncmp(path, "/cpus/cpu", 9) == 0 && strcmp(key, "reg") == 0){
		hartcnt++;		/* count harts the DTB lists (see confinit) */
		return;
	}
	/*
	 * /cpus/timebase-frequency = CLINT mtime tick rate in Hz.  All
	 * kernel timer math (HZ scheduling, tsleep, IP-stack retransmit
	 * timers) depends on this being correct.  On the K1 it's 24 MHz;
	 * hard-coding 10 MHz makes every kernel timer fire 2.4x too
	 * quickly, which warps everything downstream including ping RTT.
	 */
	if(strcmp(path, "/cpus") == 0
	   && strcmp(key, "timebase-frequency") == 0 && len >= 4)
		timebase = beget4(p);
	if(strncmp(path, "/chosen", 7) == 0){
		if(strcmp(key, "bootargs") == 0){
			if(preloadedini)	/* don't clobber a plan9.ini */
				return;
			if(len > BOOTARGSLEN)
				len = BOOTARGSLEN;
			memmove(BOOTARGS, val, len);
			bootargs_set = 1;
			return;
		}
		if(strcmp(key, "stdout-path") == 0 || strcmp(key, "stdout_path") == 0){
			int n = len;
			if(n >= (int)sizeof(dtbstdout))
				n = sizeof(dtbstdout) - 1;
			memmove(dtbstdout, val, n);
			dtbstdout[n] = 0;
			return;
		}
	}
	if(strcmp(path, "/aliases") == 0 && strncmp(key, "serial", 6) == 0){
		int n = atoi(key + 6);
		int ln = len;
		if(n < 0 || n >= MAXDTBUARTS)
			return;
		if(ln >= (int)sizeof(dtbalias[0]))
			ln = sizeof(dtbalias[0]) - 1;
		memmove(dtbalias[n], val, ln);
		dtbalias[n][ln] = 0;
		return;
	}
	if(strcmp(path, "/aliases") == 0 && strncmp(key, "ethernet", 8) == 0){
		int n = atoi(key + 8);
		int ln = len;
		if(n < 0 || n >= MAXDTBETHERS)
			return;
		if(ln >= (int)sizeof(dtbethalias[0]))
			ln = sizeof(dtbethalias[0]) - 1;
		memmove(dtbethalias[n], val, ln);
		dtbethalias[n][ln] = 0;
		return;
	}
	if(isethernetnode(path)){
		Dtbether *e = findorallocether(path);
		int n;
		if(e == nil)
			return;
		if(strcmp(key, "reg") == 0 && len >= 16){
			e->base = ((uvlong)beget4(p) << 32) | beget4(p + 4);
			e->size = ((uvlong)beget4(p + 8) << 32) | beget4(p + 12);
		} else if(strcmp(key, "interrupts") == 0 && len >= 4){
			e->irq = beget4(p);
		} else if(strcmp(key, "interrupts-extended") == 0 && len >= 8){
			/* <phandle irq> — second cell is the source number */
			e->irq = beget4(p + 4);
		} else if(strcmp(key, "compatible") == 0){
			n = len;
			if(n >= (int)sizeof(e->compatible))
				n = sizeof(e->compatible) - 1;
			memmove(e->compatible, val, n);
			e->compatible[n] = 0;
		} else if(strcmp(key, "status") == 0){
			e->ok = (len >= 5 && strcmp((char*)val, "okay") == 0);
		} else if(strcmp(key, "local-mac-address") == 0 && len == 6){
			memmove(e->mac, val, 6);
			e->hasmac = 1;
		} else if(strcmp(key, "mac-address") == 0 && len == 6 && !e->hasmac){
			memmove(e->mac, val, 6);
			e->hasmac = 1;
		} else if(strcmp(key, "k1x,apmu-base-reg") == 0 && len >= 4){
			/* SpacemiT extension: APMU MMIO base shared by all EMACs. */
			e->apmubase = beget4(p);
		} else if(strcmp(key, "ctrl-reg") == 0 && len >= 4){
			/* SpacemiT extension: per-EMAC offset into APMU for
			 * clock/reset control. */
			e->ctrlregoff = beget4(p);
		} else if(strcmp(key, "dline-reg") == 0 && len >= 4){
			/* SpacemiT extension: per-EMAC offset into APMU for
			 * the RGMII delay-line register. */
			e->dlineregoff = beget4(p);
		} else if(strcmp(key, "tx-phase") == 0 && len >= 4){
			/* SpacemiT extension: RGMII delay in degrees of the
			 * 125MHz clock; per-port board tuning. */
			e->txphase = beget4(p);
		} else if(strcmp(key, "rx-phase") == 0 && len >= 4){
			e->rxphase = beget4(p);
		} else if(strcmp(key, "emac,reset-gpio") == 0 && len >= 12){
			/* <phandle pin flags> — the PHY's reset line */
			e->rstpin = beget4(p + 4);
		}
		return;
	}
	if(strcmp(key, "compatible") == 0){
		/*
		 * Spot the SoC's external interrupt controller.  A "riscv,plic*"
		 * compatible means plic.c can drive it; the PLIC's reg base is
		 * captured separately (see isintcnode branch below).  APLIC /
		 * IMSIC / MSI-only parts (e.g. K3) need a different driver and
		 * we leave dtbhasplic_ false for them.
		 */
		char *c;
		int rem = len;
		int n;
		for(c = (char*)val; rem > 0; ){
			int sl = strlen(c) + 1;
			if(strncmp(c, "riscv,plic", 10) == 0){
				dtbhasplic_ = 1;
				n = strlen(path);
				if(n >= (int)sizeof(dtbplicpath))
					n = sizeof(dtbplicpath) - 1;
				memmove(dtbplicpath, path, n);
				dtbplicpath[n] = 0;
			}
			c += sl;
			rem -= sl;
		}
	}
	if(isintcnode(path) && strcmp(key, "reg") == 0 && len >= 8){
		Dtbintc *ic = findorallocintc(path);
		if(ic != nil){
			/*
			 * /soc on K1 is #address-cells=2 #size-cells=2, so reg is
			 * 16 bytes; older qemu-virt uses 8.  Cover both.
			 */
			if(len >= 16){
				ic->base = ((uvlong)beget4(p) << 32) | beget4(p + 4);
				ic->size = ((uvlong)beget4(p + 8) << 32) | beget4(p + 12);
			} else {
				ic->base = beget4(p);
				ic->size = len >= 8? beget4(p + 4): 0;
			}
		}
	}
	if(isuartnode(path)){
		Dtbuart *u = findorallocuart(path);
		int n;
		if(u == nil)
			return;
		if(strcmp(key, "reg") == 0 && len >= 8){
			/*
			 * Assume #address-cells=2, #size-cells=2 (true for /soc on K1).
			 * base = upper 64 bits; the next 64 bits are size, which we don't need.
			 */
			if(len >= 16)
				u->base = ((uvlong)beget4(p) << 32) | beget4(p + 4);
			else
				u->base = beget4(p);
		} else if(strcmp(key, "interrupts") == 0 && len >= 4){
			u->irq = beget4(p);
		} else if(strcmp(key, "reg-shift") == 0 && len == 4){
			u->regshift = beget4(p);
		} else if(strcmp(key, "compatible") == 0){
			n = len;
			if(n >= (int)sizeof(u->compatible))
				n = sizeof(u->compatible) - 1;
			memmove(u->compatible, val, n);
			u->compatible[n] = 0;
		} else if(strcmp(key, "status") == 0){
			u->ok = (len >= 5 && strcmp((char*)val, "okay") == 0);
		}
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

/*
 * After the whole tree is walked, link each Dtbuart back to its serial<N>
 * alias, mark the entry that /chosen/stdout-path picks as the console, and
 * resolve the SoC PLIC physical base from the candidates we cached on the
 * way through.  soc.plic ends up holding the PLIC base if and only if the
 * DTB told us where it is; otherwise socidentify() falls back to a vendor-
 * specific default.
 */
static void
finishdtbuarts(void)
{
	int i, j, k, n;
	char name[32], *path, *colon;

	if(dtbhasplic_ && dtbplicpath[0] != 0){
		for(i = 0; i < ndtbintc; i++)
			if(strcmp(dtbintc[i].path, dtbplicpath) == 0){
				soc.plic = (uchar*)(uintptr)dtbintc[i].base;
				break;
			}
	}

	for(i = 0; i < MAXDTBUARTS; i++){
		if(dtbalias[i][0] == 0)
			continue;
		for(j = 0; j < ndtbuarts; j++){
			if(strcmp(dtbuarts[j].path, dtbalias[i]) == 0){
				dtbuarts[j].aliasidx = i;
				break;
			}
		}
	}
	for(i = 0; i < MAXDTBETHERS; i++){
		if(dtbethalias[i][0] == 0)
			continue;
		for(j = 0; j < ndtbethers; j++){
			if(strcmp(dtbethers[j].path, dtbethalias[i]) == 0){
				dtbethers[j].aliasidx = i;
				break;
			}
		}
	}
	/*
	 * aliases are optional: hand enabled ether nodes that got no
	 * ethernetN alias the free indexes, in node order, so a DTB that
	 * only aliases the first port still yields the others.
	 */
	for(j = 0; j < ndtbethers; j++){
		if(dtbethers[j].aliasidx >= 0 || !dtbethers[j].ok)
			continue;
		for(i = 0; i < MAXDTBETHERS; i++){
			for(k = 0; k < ndtbethers; k++)
				if(dtbethers[k].aliasidx == i)
					break;
			if(k == ndtbethers){
				dtbethers[j].aliasidx = i;
				break;
			}
		}
	}

	if(dtbstdout[0] == 0)
		return;
	n = strlen(dtbstdout);
	if(n >= (int)sizeof(name))
		n = sizeof(name) - 1;
	memmove(name, dtbstdout, n);
	name[n] = 0;
	if((colon = strchr(name, ':')) != nil)
		*colon = 0;
	if(strncmp(name, "serial", 6) == 0){
		int a = atoi(name + 6);
		if(a < 0 || a >= MAXDTBUARTS || dtbalias[a][0] == 0)
			return;
		path = dtbalias[a];
	} else {
		path = name;
	}
	for(j = 0; j < ndtbuarts; j++)
		if(strcmp(dtbuarts[j].path, path) == 0){
			dtbuarts[j].isconsole = 1;
			return;
		}
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
	finishdtbuarts();
	if(preloadedini)
		plan9iniinit(BOOTARGS, 0);
	else if(bootargs_set)
		plan9iniinit(BOOTARGS, 1);
	return 0;
}

/*
 * a bootloader (or writeconf across a reboot) may have left a plan9.ini
 * at CONFADDR, the same convention u-boot boot scripts use on the arm
 * boards: zero the region, then load the file over it.  Accept it only
 * if it's NUL-terminated printable text containing a '='; anything else
 * is leftover memory, and configuration comes from the DTB instead.
 */
static int
inipreloaded(void)
{
	char *s;
	int n, eq;

	s = BOOTARGS;
	eq = 0;
	for(n = 0; n < BOOTARGSLEN; n++, s++){
		if(*s == 0)
			return n > 0 && eq;
		if(*s == '=')
			eq = 1;
		else if(*s == '\r')
			*s = '\n';	/* tolerate DOS line ends */
		else if((*s < ' ' && *s != '\n' && *s != '\t') || (uchar)*s >= 0x80)
			return 0;
	}
	return 0;
}

void
bootargsinit(void)
{
	static char fenced[BOOTARGSLEN];

	bootconf = (char*)CONFADDR;
	if(peekb(bootconf) < 0)
		bootconf = fenced;
	preloadedini = inipreloaded();
	if(dtbphys == 0 || dtbphys == (uintptr)-1){
		print("bootargsinit: no DTB pointer from firmware\n");
		if(preloadedini)
			plan9iniinit(BOOTARGS, 0);
		return;
	}
	if(parsedevtree((uchar*)dtbphys) < 0){
		print("bootargsinit: no usable DTB at %#p\n", (void*)dtbphys);
		if(preloadedini)
			plan9iniinit(BOOTARGS, 0);
		return;
	}
	{
		int i, ndis;

		ndis = 0;
		for(i = 0; i < ndtbethers; i++)
			if(!dtbethers[i].ok)
				ndis++;
		print("bootargsinit: %d entries from %s, %d uart %d ether nodes (%d disabled)\n",
			nconf, preloadedini? "plan9.ini": "DTB /chosen/bootargs",
			ndtbuarts, ndtbethers, ndis);
	}
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

	if(nconf < 0){
		/* use defaults when there was no configuration */
		ksetenv("console", "0", 1);
		return;
	}

	for(i = 0; i < nconf; i++){
		if(confname[i][0] != '*')
			ksetenv(confname[i], confval[i], 0);
		ksetenv(confname[i], confval[i], 1);
	}
}

void
writeconf(void)
{
	char *p, *q;
	int n;

	p = getconfenv();
	if(waserror()) {
		free(p);
		nexterror();
	}

	/* convert to name=value\n format */
	for(q=p; *q; q++) {
		q += strlen(q);
		*q = '=';
		q += strlen(q);
		*q = '\n';
	}
	n = q - p + 1;
	if(n >= BOOTARGSLEN)
		error("kernel configuration too large");
	memmove(BOOTARGS, p, n);
	memset(BOOTARGS+n, 0, BOOTARGSLEN-n);
	poperror();
	free(p);
}
