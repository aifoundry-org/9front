#include <u.h>
#include <libc.h>

/*
 * Comparisons and truth tests of values converted to
 * char, uchar, short and ushort (and from vlong to int).
 * The results of each test are hashed with FNV-1a and
 * compared with the hash of 8c's results.
 */

void	fail(char*, ...);
void	ok(char*);

char	gc;
uchar	guc;
short	gs;
ushort	gus;
int	gi;

int	vals[] = {
	0, 1, -1, 127, 128, -128, -129, 255, 256, 384,
	32767, 32768, -32768, -32769, 65535, 65536,
	0x7fffffff, 0x80000000, 0x12345680,
};

#define	REL(x, y)\
	switch(op){\
	case 0:	return x == y;\
	case 1:	return x != y;\
	case 2:	return x < y;\
	case 3:	return x <= y;\
	case 4:	return x > y;\
	case 5:	return x >= y;\
	}\
	return -1;

#define	BREL(x, y)\
	switch(op){\
	case 0:	if(x == y) return 1; break;\
	case 1:	if(x != y) return 1; break;\
	case 2:	if(x < y) return 1; break;\
	case 3:	if(x <= y) return 1; break;\
	case 4:	if(x > y) return 1; break;\
	case 5:	if(x >= y) return 1; break;\
	}\
	return 0;

#define	TRUTH(x)\
	switch(op){\
	case 0:	return !x;\
	case 1:	if(x) return 1; break;\
	case 2:	return x && j;\
	case 3:	return x || j;\
	case 4:	return x ? 2 : 3;\
	case 5:	if(!x) return 1; break;\
	}\
	return 0;

/* global vs cast, cast vs global, cast vs cast, cast vs constant; branch forms; truth */
#define	FORMS(T, g, K, fa, fb, fc, fd, fe, ff, fg, ft)\
int fa(int op, int i, int j) { USED(j); REL(g, (T)i) }\
int fb(int op, int i, int j) { USED(j); REL((T)i, g) }\
int fc(int op, int i, int j) { REL((T)i, (T)j) }\
int fd(int op, int i, int j) { USED(j); REL((T)i, K) }\
int fe(int op, int i, int j) { USED(j); BREL(g, (T)i) }\
int ff(int op, int i, int j) { BREL((T)i, (T)j) }\
int fg(int op, int i, int j) { USED(j); BREL(K, (T)i) }\
int ft(int op, int i, int j) { TRUTH((T)i) }

FORMS(char, gc, -128, ca, cb, cc, cd, ce, cf, cg, ct)
FORMS(uchar, guc, 0x80, uca, ucb, ucc, ucd, uce, ucf, ucg, uct)
FORMS(short, gs, -32768, sa, sb, sc, sd, se, sf, sg, st)
FORMS(ushort, gus, 0x8000, usa, usb, usc, usd, use, usf, usg, ust)

char
retc(int i)
{
	return i;
}

ushort
retus(int i)
{
	return i;
}

int
parc(char c, int j)
{
	return c == gc && c <= j;
}

int
parus(ushort u, int j)
{
	USED(j);
	return u == gus;
}

/* narrow values that are not casts */
int
misc(int op, int i, int j)
{
	char c;
	uchar uc;
	short s;
	int k;

	switch(op){
	case 0:	return (c = i) == gc;
	case 1:	return (gc = i) == (char)j;
	case 2:	c = gc; return (c += i) == (char)j;
	case 3:	c = gc; return ++c == (char)(j+i);
	case 4:	uc = guc; return (uc -= i) < (uchar)j;
	case 5:	s = gs; return (s *= i) == (short)j;
	case 6:	return retc(i) == gc;
	case 7:	return retus(i) < gus;
	case 8:	return parc(i, j);
	case 9:	return parus(i, j);
	case 10:	return (j&1 ? (char)i : gc) == gc;
	case 11:	return (uchar)(char)i == 0x80;
	case 12:	return (char)(short)i == gc;
	case 13:	return (short)(char)i == gs;
	case 14:	return (uchar)(short)i < guc;
	case 15:	return (char)(i>>1) == gc;
	case 16:	return (char)(i+j) == gc;
	case 17:	return (char)(i&j) < 0;
	case 18:	return (uchar)~i == guc;
	case 19:	return (char)-i == gc;
	case 20:
		switch((char)i){
		case -128:	return 1;
		case 0:	return 2;
		case -1:	return 3;
		}
		return 0;
	case 21:
		switch(c = i){
		case -128:	return 1;
		case 0:	return 2;
		}
		return 0;
	case 22:
		switch((ushort)i){
		case 0x8000:	return 1;
		case 0xffff:	return 2;
		}
		return 0;
	case 23:
		for(k = 0, c = i; c != gc && k < 300; c++)
			k++;
		return k;
	case 24:	c = i; if(c) return 1; return 0;
	case 25:	return (char)i == (uchar)j;
	case 26:	return (short)i < (ushort)j;
	case 27:	return (char)i < (short)j;
	}
	return -1;
}

/* 32-bit values narrowed from vlong */
int
vint(int op, int i, int j)
{
	vlong v, w;

	v = ((vlong)j<<32) | (uint)i;
	w = ((vlong)i<<32) | (uint)j;
	switch(op){
	case 0:	return (int)v == (int)w;
	case 1:	return (int)v < (int)w;
	case 2:	return (uint)v < (uint)w;
	case 3:	return (long)v <= gi;
	case 4:	return (ulong)v >= (ulong)gi;
	case 5:	return (int)v < 0;
	case 6:	return !(int)v;
	case 7:	if((uint)w) return 1; return 0;
	case 8:	return (int)(v>>16) == (int)(w>>16);
	case 9:	return (uint)(v>>16) > (uint)(w>>16);
	case 10:	return (int)v == i && (uint)w == j;
	case 11:	return (char)v == (char)w;
	case 12:	return (ushort)v < (ushort)w;
	}
	return -1;
}

typedef struct Test Test;
struct Test
{
	char	*name;
	int	(*f)(int, int, int);
	int	nop;
	uvlong	sum;
};

Test	tests[] = {
	"ca", ca, 6, 0xb63f53a0214b3704ULL,
	"cb", cb, 6, 0x9ba46ef3290f2ed4ULL,
	"cc", cc, 6, 0x9ba46ef3290f2ed4ULL,
	"cd", cd, 6, 0xbdd454750b832e40ULL,
	"ce", ce, 6, 0xb63f53a0214b3704ULL,
	"cf", cf, 6, 0x9ba46ef3290f2ed4ULL,
	"cg", cg, 6, 0x1671dd519ae15210ULL,
	"ct", ct, 6, 0xd9917f3a72c7a9dbULL,
	"uca", uca, 6, 0xddaafc1705c1bd44ULL,
	"ucb", ucb, 6, 0xbe1568b88cdefa54ULL,
	"ucc", ucc, 6, 0xbe1568b88cdefa54ULL,
	"ucd", ucd, 6, 0xb5f9748f88ffd068ULL,
	"uce", uce, 6, 0xddaafc1705c1bd44ULL,
	"ucf", ucf, 6, 0xbe1568b88cdefa54ULL,
	"ucg", ucg, 6, 0x88c66ed9caba2658ULL,
	"uct", uct, 6, 0xd9917f3a72c7a9dbULL,
	"sa", sa, 6, 0xd9b9863ba455c33cULL,
	"sb", sb, 6, 0x1e313d2ea4f4d0ecULL,
	"sc", sc, 6, 0x1e313d2ea4f4d0ecULL,
	"sd", sd, 6, 0x70a0d2df4b075548ULL,
	"se", se, 6, 0xd9b9863ba455c33cULL,
	"sf", sf, 6, 0x1e313d2ea4f4d0ecULL,
	"sg", sg, 6, 0x8cf568e4600e9958ULL,
	"st", st, 6, 0xabb9c512ea88f5f0ULL,
	"usa", usa, 6, 0xf1d66bdb505a015cULL,
	"usb", usb, 6, 0x14319b81a155b80cULL,
	"usc", usc, 6, 0x14319b81a155b80cULL,
	"usd", usd, 6, 0xe9b49723ea4ee25cULL,
	"use", use, 6, 0xf1d66bdb505a015cULL,
	"usf", usf, 6, 0x14319b81a155b80cULL,
	"usg", usg, 6, 0xdd2ff816ee6ba30cULL,
	"ust", ust, 6, 0xabb9c512ea88f5f0ULL,
	"misc", misc, 28, 0xeabdc3242e7abce4ULL,
	"vint", vint, 13, 0xc97ddd8756cb6774ULL,
};

void
narrow(void)
{
	Test *t;
	int op, a, b;
	uvlong h;

	for(t = tests; t < tests+nelem(tests); t++){
		h = 0xcbf29ce484222325ULL;
		for(op = 0; op < t->nop; op++)
			for(a = 0; a < nelem(vals); a++)
				for(b = 0; b < nelem(vals); b++){
					gc = vals[b];
					guc = vals[b];
					gs = vals[b];
					gus = vals[b];
					gi = vals[b];
					h ^= (*t->f)(op, vals[a], vals[b]);
					h *= 0x100000001b3ULL;
				}
		if(h != t->sum)
			fail("narrow %s: hash %llux, want %llux", t->name, h, t->sum);
	}
	ok("narrow");
}
