#include "u.h"
#include "tos.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "pool.h"
#include "io.h"
#include "sysreg.h"
#include "ureg.h"

//#include "rebootcode.i"

Conf conf;

int
isaconfig(char *, int, ISAConf *)
{
	return 0;
}

/*
 *  starting place for first process
 */
void
init0(void)
{
	char buf[2*KNAMELEN], **sp;
	Page *p;

	print("chandevinit ...\n");
	chandevinit();

	if(!waserror()){
		snprint(buf, sizeof(buf), "%s %s", "RISCV64", conffile);
		ksetenv("terminal", buf, 0);
		ksetenv("cputype", "riscv64", 0);
		if(cpuserver)
			ksetenv("service", "cpu", 0);
		else
			ksetenv("service", "terminal", 0);
		setconfenv();
		poperror();
	}
	kproc("alarm", alarmkproc, 0);

	p = newpage(USTKTOP-BY2PG, nil);
	segpage(up->seg[SSEG], p);
	sp = (char**)(p->pa + BY2PG - sizeof(Tos) - 8 - sizeof(sp[0])*4);
	print("sp is %p for usp %p\n", sp, sp);
	sp[3] = sp[2] = sp[1] = nil;
	strcpy(sp[1] = (char*)&sp[4], "boot");
	sp[0] = (void*)&sp[1];

	splhi();
	fpukexit(nil, nil);
	mmuswitch(up);
	print("fault %d sp %d\n", fault(UTZERO, UTZERO, 1), fault(p->va, UTZERO, 0));
	u64int* pte = userpte((void *)UTZERO);
	print("pte is %p *pte %llx\n", pte, *pte);
	print("touser baby MACH m is %p mmuto p%p\n", m, m->mmutop);

	print("islo %d, now enable clock\n", islo());
	clockenable();
	// I am not sure this should be here ...
	fpoff();
	print("SP will be USTKTOP-BY2PG %p\n", (uintptr)USTKTOP-BY2PG);
	touser((uintptr)USTKTOP-BY2PG);
}

void
confinit(void)
{
	int userpcnt;
	ulong kpages;
	char *p;
	int i;
	
	sys = &asys;
	sys->nonline = 1;		/* the boot cpu; secondaries bump this */

	conf.nmach = 1;
	if(p = getconf("*ncpu"))
		conf.nmach = strtol(p, 0, 0);
	if(conf.nmach > MAXMACH)
		conf.nmach = MAXMACH;

	if(p = getconf("service")){
		if(strcmp(p, "cpu") == 0)
			cpuserver = 1;
		else if(strcmp(p,"terminal") == 0)
			cpuserver = 0;
	}

	if(p = getconf("*kernelpercent"))
		userpcnt = 100 - strtol(p, 0, 0);
	else
		userpcnt = 0;

	if(userpcnt < 10)
		userpcnt = 60 + cpuserver*10;
	print("confinit start npage crap\n");
	conf.npage = 0;
	for(i = 0; i < nelem(conf.mem); i++)
		conf.npage += conf.mem[i].npage;

	print("conf.npage 0x%lx\n", conf.npage);
	kpages = conf.npage - (conf.npage*userpcnt)/100;
//	if(kpages > ((uintptr)-VDRAM)/BY2PG)
//		kpages = ((uintptr)-VDRAM)/BY2PG;

	print("kpages 0x%lx\n", kpages);
	conf.upages = conf.npage - kpages;
	conf.ialloc = (kpages/2)*BY2PG;
	print("kpages 0x%lx conf.upages 0x%lx\n", kpages, conf.upages);

	/* set up other configuration parameters */
	conf.nproc = 100 + ((conf.npage*BY2PG)/MB)*5;
	if(cpuserver)
		conf.nproc *= 3;
	if(conf.nproc > 4000)
		conf.nproc = 4000;
	conf.nswap = conf.npage*3;
	conf.nswppo = 4096;
	conf.nimage = 200;

	conf.copymode = conf.nmach > 1;

	/*
	 * Guess how much is taken by the large permanent
	 * datastructures. Mntcache and Mntrpc are not accounted for.
	 */
	kpages = conf.npage - conf.upages;
	print("kpages to start is %lx = %lx - %lx \n", kpages, conf.npage, conf.upages);
	kpages *= BY2PG;
	print("%lx -= %lx + %lx + %lx + %lx + %x\n", 
		kpages,conf.upages*sizeof(Page)
		, conf.nproc*sizeof(Proc*)
		, conf.nimage*sizeof(Image)
		, conf.nswap
		, conf.nswppo*sizeof(Page*));
	kpages -= conf.upages*sizeof(Page)
		+ conf.nproc*sizeof(Proc*)
		+ conf.nimage*sizeof(Image)
		+ conf.nswap
		+ conf.nswppo*sizeof(Page*);
	print("kpages now is ... %lx\n", kpages);
	//kpages *= BY2PG;
	print("exit with kpages %lx\n", kpages);
	mainmem->maxsize = kpages;
	imagmem->maxsize = kpages;
}

void
machinit(void)
{
	extern void strap(void);
	wstvec((uintptr)strap);
	m->ticks = 1;
	m->perf.period = 1;
	active.machs[m->machno] = 1;
}

void
mpinit(void)
{
	/*
	 * Uniprocessor unless *ncpu>1 was set, so the default boot is
	 * unchanged.  hsmstartall() starts the other harts at _main (l.s)
	 * via SBI HSM; each lands in cpustart().
	 */
	if(conf.nmach <= 1)
		return;
	if(nosbi){
		print("mpinit: no SBI HSM; staying uniprocessor\n");
		return;
	}
	soc.idlewake = 1;	/* enable cross-cpu wakeup via software ipis */
	hsmstartall();
}

void
cpuidprint(void)
{
	iprint("cpu%d: 1000MHz QEMU\n", m->machno);
}

int i = 1;
int once = 0;
void
main(void)
{
	if(!once){
		memset(MACHP(0), 0, KTZERO-(uintptr)MACHP(0));
	}

	if (once) panic("main entered twice");
	once++;
	while(i == 0); // BUG: does not reload i
	i = 1; // in case you use GDB to bump PC, make sure i is updated.
	if (!m->machno) {
		extern char bdata[], edata[], end[], etext[];
		static ulong vfy = 0xcafebabe;
	
		if (vfy != 0xcafebabe){
			sbiputc('Z');
			memmove(bdata, etext, edata - bdata);
		}
		if (vfy != 0xcafebabe) {
			sbiputc('?');
			panic("misaligned data segment");
		}
		vfy = 0;
	}
	memset(edata, 0, end - edata);		/* zero bss */
	machinit();
#ifdef XXX
	if(m->machno){
		trapinit();
		fpuinit();
		intrinit();
		clockinit();
		cpuidprint();
		synccycles();
		timersinit();
		mmu1init();
		m->ticks = MACHP(0)->ticks;
		schedinit();
		return;
	}
#endif
	uartconsinit();
	quotefmtinstall();
	bootargsinit();
	meminit();
	print("meminit done\n");
	confinit();
	print("confinit done\n");
	xinit();
	print("xinit done\n");
	xsummary();
	printinit();
	print("\nPlan 9\n");
	print("conf.mem[0].base %p, conf.mem[0].limit %p, conf.mem[0].npage %lud\n", conf.mem[0].base, conf.mem[0].limit, conf.mem[0].npage);
#ifdef xxx
	trapinit(); print("DONE 	trapinit();\n");
	fpuinit(); print("DONE 	fpuinit();\n");
#endif
	socidentify();		/* must precede intrinit: sets soc.plic */
	intrinit();
	clockinit();
	if(timebase == 0){
		print("timebase not in DTB, falling back to 10MHz\n");
		timebase = 10*Mhz;
	} else
		print("timebase from DTB: %lld Hz\n", timebase);
	calibrate();
	clocksanity();
	cpuidprint();
	timersinit();
	xsummary();
	pageinit();
	procinit0();
	initseg();
	links();
	chandevreset();
	userinit();
	mpinit();
	u64int tm = rdtime();
	u64int cm = rdstimecmp();
	print("tm %#llx cm %#llx\n", tm, cm);
	mmu1init();
	cpuinit(0); print("DONE cpuinit(0) -- FIXME\n");
	/*
	 * cpuinit() does clockoff() (clears Stie) as its last timer act, and
	 * for the boot cpu it runs AFTER timersinit(), so unlike a secondary
	 * (which timersinit()s after cpuinit in cpustart) cpu0 would enter the
	 * scheduler with its timer disabled — its stale periodic Timer then
	 * keeps any later timeradd() from re-arming, so the clock never restarts
	 * and timeout-only tsleeps (e.g. SD init) hang once other harts idle.
	 * Re-arm the clock here, matching the secondary path.
	 */
	timerset(0);
	schedinit();
}

void
exit(int)
{
	splhi();
	/* TODO: cpushutdown() to halt other harts cleanly via SBI HSM */
	if(m->machno == 0){
		/* clear secrets */
		zeroprivatepages();
		poolreset(secrmem);
	}
	sbireset(1, 0);		/* SRSTCOLD: works on QEMU; ignored on K1 */
	if (rvreset)
		rvreset();	/* board-specific reset, e.g. K1 WDT */
	sbireset(2, 0);		/* SRSTWARM */
	sbishutdown();		/* legacy ext 8 */
	for(;;)
		;
}

void
poweroff(void)
{
	splhi();
	if(m->machno == 0){
		/* clear secrets */
		zeroprivatepages();
		poolreset(secrmem);
	}
	sbireset(0, 0);		/* SRSTSHUT: works on QEMU; ignored on K1 */
	if (rvpoweroff)
		rvpoweroff();	/* board-specific, e.g. K1 PMIC shutdown */
	sbishutdown();		/* legacy ext 8 */
	for(;;)
		;
}

static void
rebootjump(void *entry, void *code, ulong size)
{
	print("rebootjump: %p, %p, %ld\n", entry, code, size);
	panic("rebootjmp");
#ifdef XXX
	void (*f)(void*, void*, ulong);

	intrcpushutdown();

	/* redo identity map */
	/* not needed. -- satp never changes*/
	//setttbr(PADDR(L1BOT));

	extern void strap(void);
	wstvec((uintptr)strap);
	/* setup reboot trampoline function */
	f = (void*)REBOOTADDR;
	memmove(f, rebootcode, sizeof(rebootcode));

	cachedwbinvse(f, sizeof(rebootcode));
	cacheiinvse(f, sizeof(rebootcode));

	(*f)(entry, code, size);
#endif
}

void
reboot(void*p, void *code, ulong size)
{
	print("reboot %p %p %ld\n", p, code, size);
	panic("reboot");
#ifdef XXX
	writeconf();
	while(m->machno != 0){
		procwired(up, 0);
		sched();
	}

	cpushutdown();
	delay(2000);

	splfhi();

	/* turn off buffered serial console */
	serialoq = nil;

	/* shutdown devices */
	chandevshutdown();

	/* stop the clock */
	clockshutdown();
	intrsoff();

	/* clear secrets */
	zeroprivatepages();
	poolreset(secrmem);

	/* off we go - never to return */
	rebootjump((void*)(KTZERO-KZERO), code, size);
#endif
}

void
dmaflush(int clean, void *p, ulong len)
{
	print("dmaflush %d %p %ld\n", clean, p, len);
	panic("dmaflush");
#ifdef XXX
	uintptr s = (uintptr)p;
	uintptr e = (uintptr)p + len;

	if(clean){
		s &= ~(BLOCKALIGN-1);
		e += BLOCKALIGN-1;
		e &= ~(BLOCKALIGN-1);
		cachedwbse((void*)s, e - s);
		return;
	}
	if(s & BLOCKALIGN-1){
		s &= ~(BLOCKALIGN-1);
		cachedwbinvse((void*)s, BLOCKALIGN);
		s += BLOCKALIGN;
	}
	if(e & BLOCKALIGN-1){
		e &= ~(BLOCKALIGN-1);
		if(e < s)
			return;
		cachedwbinvse((void*)e, BLOCKALIGN);
	}
	if(s < e)
		cachedinvse((void*)s, e - s);
#endif
}
/* setconfenv() now lives in bootargs.c */
