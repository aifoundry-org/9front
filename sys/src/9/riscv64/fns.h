#include "../port/portfns.h"

/* l.s */
extern void sev(void);
extern int tas(void *);
extern int cmpswap(long*, long, long);
extern void coherence(void);
extern void idlehands(void);
extern uvlong vcycles(void);
#define cycles(ip) *(ip) = vcycles()
extern int splfhi(void);
extern void splflo(void);
extern void touser(uintptr sp);
extern void sysrforkret(void);
extern void noteret(void);
extern void returnto(void*);
extern void fpconstset(void);
extern void fpon(void);
extern void fpoff(void);
extern int isfpon(void);
extern void fpsaveregs(void*);
extern void fploadregs(void*);
void fpsts2ureg(Ureg*ureg);

extern void hvccall(Ureg*);

extern void setttbr(uintptr pa);
extern uintptr getfar(void);

extern void flushasidva(uintptr asidva);
extern void tlbivae1is(uintptr asidva);

extern void flushasidvall(uintptr asidva);
extern void tlbivale1is(uintptr asidva);

extern void flushasid(uintptr asid);
extern void tlbiaside1is(uintptr asid);

extern void flushtlb(void);
extern void tlbivmalle1(void);

extern void flushlocaltlb(void);
extern void tlbivmalle1(void);

extern void sbiputc(uvlong);
extern int sbigetc(void);

/* cache */
extern ulong cachesize(int level);

extern void cacheiinvse(void*, int);
extern void cacheuwbinv(void);
extern void cacheiinv(void);

extern void cachedwbse(void*, int);
extern void cacheduwbse(void*, int);
extern void cachedinvse(void*, int);
extern void cachedwbinvse(void*, int);

extern void cachedwb(void);
extern void cachedinv(void);
extern void cachedwbinv(void);

extern void l2cacheuwb(void);
extern void l2cacheuinv(void);
extern void l2cacheuwbinv(void);

/* mmu */
#define	getpgcolor(a)	0
extern uintptr paddr(void*);
#define PADDR(a) paddr((void*)(a))
extern uintptr cankaddr(uintptr);
extern void* kaddr(uintptr);
#define KADDR(a) kaddr(a)
extern void kmapinval(void);
#define	VA(k)	((uintptr)(k))
extern KMap *kmap(Page*);
extern void kunmap(KMap*);
extern uintptr mmukmap(uintptr, uintptr, usize);
extern void kmapram(uintptr, uintptr);
extern void* vmap(uvlong, vlong);
extern void vunmap(void*, vlong);
extern void mmu1init(void);
extern void* mmuuncache(void*, usize);
extern void putasid(Proc*);

extern void* ucalloc(usize);
extern void* ucallocalign(usize size, int align, usize span);
extern void ucfree(void*);

/* mem */
extern void mmuidmap(uintptr*);
extern void mmu0init(uintptr*);
extern void meminit(void);

/* clock */
extern void clockinit(void);
extern void synccycles(void);
extern void clockshutdown(void);

/* fpu */
extern void fpuinit(void);
extern void fpuprocsetup(Proc*);
extern void fpuprocfork(Proc*);
extern void fpuprocsave(Proc*);
extern void fpuprocrestore(Proc*);
extern FPsave* fpukenter(Ureg*);
extern void fpukexit(Ureg*, FPsave*);
extern void mathtrap(Ureg*);

/* fpu from 9k */
void	fpconstset(void);
void	fpoff(void);
void	fpon(void);
int	fpudevprocio(Proc*, void*, long, uintptr, int);
void	_fpuinit(void);
void	fpuinit(void);
void	fptrap(Ureg *, void *);
void	fpuprocrestore(Proc*);
void	fpuprocsave(Proc*);
void	fpurestore(uchar *);
void	fpusave(uchar *);
void	fpusysprocsetup(Proc*);
void	fpusysrforkchild(Proc*, Proc*);
void	fpusysrfork(Ureg*);

/* trap */
extern void trapinit(void);
extern int userureg(Ureg*);
extern void evenaddr(uintptr);
extern void setkernur(Ureg*, Proc*);
extern void procfork(Proc*);
extern void procsetup(Proc*);
extern void procsave(Proc*);
extern void procrestore(Proc *);
extern void trap(Ureg*);
extern void syscall(Ureg*);
extern void dumpstack(void);
extern void dumpregs(Ureg*);

/* irq */
extern void intrinit(void);
extern void socidentify(void);
extern void intrcpushutdown(void);
extern void intrsoff(void);
extern void intrenable(int, void (*)(Ureg*, void*), void*, int, char*);
extern void intrdisable(int, void (*)(Ureg*, void*), void*, int, char*);
extern int irq(Ureg*);
extern void fiq(Ureg*);
extern void plicinit(void);
extern int plicirq(Ureg*);
extern void plicintrenable(int, void (*)(Ureg*, void*), void*, int, char*);
extern void plicintrdisable(int, void (*)(Ureg*, void*), void*, int, char*);
extern void aplicinit(void);
extern int aplicirq(Ureg*);
extern void aplicintrenable(int, void (*)(Ureg*, void*), void*, int, char*);
extern void aplicintrdisable(int, void (*)(Ureg*, void*), void*, int, char*);
extern void imsicswr(uint, uvlong);
extern uvlong stopeiclaim(void);

/* sysreg */
extern uvlong	sysrd(ulong);
extern void	syswr(ulong, uvlong);

/* uartimx */
extern void uartconsinit(void);

/* dma */
extern void dmaflush(int, void*, ulong);

/* main */
extern char *getconf(char *name);
extern void setconfenv(void);
extern void writeconf(void);

extern int isaconfig(char*, int, ISAConf*);
extern void links(void);

/* ccm */
extern void setclkgate(char *name, int on);
extern void setclkrate(char *name, char *source, int freq);
extern int getclkrate(char *name);

/* gpc */
extern void powerup(char *dom);

/* lcd */
extern void lcdinit(void);

/* iomux */
extern void iomuxpad(char *pads, char *sel, char *cfg);
extern uint iomuxgpr(int gpr, uint set, uint mask);

/* gpio */
#define GPIO_PIN(n, m)	((n)<<5 | (m))
extern void gpioout(uint pin, int set);
extern int gpioin(uint pin);
void gpiointrenable(uint pin, int mode, void (*f)(uint pin, void *a), void *a);
void gpiointrdisable(uint pin);

/* gpiok3 */
extern void gpiok3init(void);

/* k1reset */
extern int k1pmicread(int reg);
extern int k1pmicwrite(int reg, int val);

/* k1thermal */
extern void k1thermalinit(void);

/* k3thermal */
extern void k3thermalinit(void);

/* espik3 -- eSPI controller + cros_ec host-command transport, board-gated and
   lazily brought up on first call. Any future EC consumer uses these three */
extern int k3espiready(void);
extern int k3eccmd(int cmd, int ver, void *tx, int txlen, void *rx, int rxlen);
extern int k3ecmemmapread(int off, void *buf, int n);

/* espik3fan */
extern void k3ecfaninit(void);
/* screenpicoitx.c: DP1 framebuffer driver entry point plus the two
 * accessors screen.c's screeninit() needs. */
extern void dpuk3(void);
extern uvlong dpuk3fbaddr(void);
extern void* dpuk3fbkaddr(void);
/* screen.c */
extern void screeninit(void);

/* devarch */
extern Dirtab* addarchfile(char *name, int perm,
	long(*rdfn)(Chan*,void*,long,vlong), long(*wrfn)(Chan*,void*,long,vlong));

/* fan */
extern int fanfmt(char *buf, int nbuf, char *unit, long val, int isauto);
extern long fanparse(Cmdbuf *cb, char *unit, int hasauto, int *isauto);

/* main */
extern void poweroff(void);

/* pciqemu */
extern int pcicfgrw8(int tbdf, int rno, int data, int read);
extern int pcicfgrw16(int tbdf, int rno, int data, int read);
extern int pcicfgrw32(int tbdf, int rno, int data, int read);
extern void pciintrenable(int tbdf, void (*f)(Ureg*, void*), void *a);
extern void pciintrdisable(int tbdf, void (*f)(Ureg*, void*), void *a);

/* bootargs */
extern void bootargsinit(void);
extern int dtbnuart(void);
extern Dtbuart *dtbuart(int);
extern int dtbhasplic(void);
extern int dtbnether(void);
extern Dtbether *dtbether(int);
extern int dtbnufs(void);
extern Dtbufs *dtbufs(int);
extern int dtbsocmatch(char*);

// new
void wsatp(u64int);
void wstvec(u64int);
void putscounteren(uintptr);
u64int rsatp(void);
u64int rseed(void);
void *usertokernel(void *v);
u64int *userpte(void *v);
void flushalltlb(void);
void flushvatlb(uintptr);

// not all implemented
void	clearipi(void);
void	clockenable(void);
void	clockintr(Ureg* ureg, void *);
void	clockoff(void);
void	timerset(uvlong);
int	clocksanity(void);
void	clrreserv(void);
uintptr	clrsie(uintptr);
ulong	clrsipbit(ulong);
void	clrstie(void);
void	cboinval(void *);
void	cboflush(void *);
void	cboclean(void *);
void	l2init(void);

// more good 9k stuff: m mode bits.
// Let's keep them until we're sure we don't want them.
ulong	getfcsr(void);
ulong	setfcsr(uintptr);
uintptr	getmie(void);
uintptr	getmip(void);
uintptr	getmsts(void);
void*	getmtvec(void);
void	putmie(uintptr);
void	putmip(uintptr);
void	putmsts(uintptr);
void*	putmtvec(void *);
uvlong	rdtime(void);
uvlong	rdtsc(void);
void	sbisettimer(uvlong);

/* SBI */
vlong	sbicall(uvlong, uvlong, uvlong, Sbiret *, uvlong *);
vlong	sbiclearipi(void);
vlong	sbiecall(uvlong, uvlong, uvlong, Sbiret *, uvlong *);
vlong	sbigetimplid(void);
vlong	sbigetimplvers(void);
uvlong	sbigetmarchid(void);
uvlong	sbigetmimpid(void);
uvlong	sbigetmvendorid(void);
vlong	sbihartstart(uvlong hartid, uvlong phys_start, uvlong private);
vlong	sbihartstatus(uvlong hartid);
vlong	sbihartstop(void);
vlong	sbihartsuspend(void);
vlong	sbiprobeext(uvlong);
vlong	sbiremotesfencevma(uvlong *hartmask, uvlong start, uvlong size);
void	sbireset(ulong type, ulong reason);
vlong	sbisendipi(uvlong map[]);
void	sbisettimer(uvlong);
void	sbishutdown(void);

int	aadd(int *, int);
ulong	amoswapw(ulong *a, ulong nv);
uintptr	setsie(uintptr);
uintptr	getsie(void);
uintptr	getsip(void);
uintptr	getsp(void);
uintptr	getsts(void);
void*	getstvec(void);
void	putsie(uintptr);
void	putsip(uintptr);
void	putsscratch(uintptr);
void	putsts(uintptr);
void*	putstvec(void *);

/* reloc */
void	relocfixup(void);

int	peekb(void*);
int peekl(void *addr, u32int *dst, int numwords);

/* archrv */
void calibrate(void);
uint mach2context(Mach *);
void cpuinit(int);
void clintipitohart(int hart);
int ipitohart(int hart);
void countipi(void);
void idlewake(void);
void mpinit(void);
void hsmstartall(void);
void archreset(void);
void wfi(void);
void cpustart(int);
extern void cpumain(void);

/* rva23 */
void wrstimecmp(u64int);
u64int rdstimecmp(void);

void soft(void);
int clz(ulong);
