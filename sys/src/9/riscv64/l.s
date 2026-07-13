#include "riscv64l.h"

#include "mem.h"

#undef SFENCE
#undef WFI

#define SFENCE	WORD	$0x12000073
#define WFI	WORD	$0x10500073

#define SATP_SV39	(8ULL<<60)
#define MAKE_SATP(x)	(SATP_SV39|((x)>>12))

/*
	MOV	$_start(SB), R12
	MOV $0, R16
	MOV $1, R17
	MOV $65, R10
	ECALL
	JMP (R12)

*/
// THIS MUST BE FIRST FUNCTION IN THIS FILE
TEXT	_start(SB), 1, $-4
	/*
	 * S-mode entry state guaranteed by SBI, for both the cold-boot entry
	 * and an HSM hart_start resumption (see sbihartstart):
	 *   a0 (R10) = hartid of this hart
	 *   a1 (R11) = cold boot: DTB physical address
	 *              hart_start: the opaque value the starter passed (we
	 *                          pass the DTB, so it's the DTB either way)
	 *   pc       = this entry address; privilege = S-mode
	 *   satp     = 0 (paging off, so VA == PA here)
	 *   sstatus.SIE = 0, sie = 0 (interrupts masked)
	 * Everything else (sp, gp, tp, saved/temp regs) is UNSPECIFIED; we set
	 * up SB, stack and Mach ourselves below.
	 */
	/*
	 * RISC-V PE/COFF header (64 bytes) so that u-boot's booti can load
	 * a plain kernel binary without a uImage wrapper.
	 * Layout per Linux Documentation/riscv/boot-image-header.rst:
	 *   code0        first instruction (must be executable)
	 *   code1        second instruction
	 *   text_offset  load offset relative to image start (0)
	 *   image_size   effective image size (0 = infer from file)
	 *   flags        endianness/page-size (0 = LE, default page)
	 *   version      header version, currently 2
	 *   res1..res3   reserved
	 *   magic        deprecated (leave 0)
	 *   magic2       "RSC\5" — what booti actually validates
	 * code0 jumps 64 bytes forward, past the header, into the relocator below.
	 * ELF entry is _start for bootelf and kexec: start at code0 and take the jump.
	 *
	 * The kernel runs in place wherever the loader put it and fixes up
	 * its data pointers via the jl -u relocation table (relocfixup),
	 * so text_offset only steers the loader, it is not a promise.
	 */
	WORD $0x040000ef	/* code0: jal x1, +64 */
	WORD $0x00000013	/* code1: nop */
	WORD $0x40200000	/* text_offset[31:0] (bianbu u-boot treats it absolute; spec-compliant loaders add their DRAM base) */
	WORD $0			/* text_offset[63:32] */
	WORD $0x01000000	/* image_size[31:0] = 16 MiB (upper bound; booti rejects 0) */
	WORD $0			/* image_size[63:32] */
	WORD $0			/* flags[31:0] */
	WORD $0			/* flags[63:32] */
	WORD $2			/* version */
	WORD $0			/* res1 */
	WORD $0			/* res2[31:0] */
	WORD $0			/* res2[63:32] */
	WORD $0			/* deprecated magic[31:0] */
	WORD $0			/* deprecated magic[63:32] */
	WORD $0x05435352	/* magic2 = "RSC\5" */
	WORD $0			/* res3 */

	MOV	$setSB(SB), R3
	JAL	R1, mmudisable<>(SB)
	/*
	 * The memory layout hangs off the runtime address of _start
	 * (== ktzero): Mach pages grow down from it, one per hart, and
	 * the boot stack sits below all of them so it never grows into a
	 * secondary hart's page.  cpu0's Mach is the highest, just under
	 * _start.
	 */
	MOV	$_start(SB), R2
	MOV	$((MAXMACH+1)*MACHSIZE), R8
	SUB	R8, R2
	/*
	 * fix up absolute pointers in initialized data if we are not
	 * running at the linked address (jl -u relocation table).
	 * a0/a1 survive in stack slots above the call's argument area.
	 */
	ADD	$-48, R2
	MOV	R10, 32(R2)
	MOV	R11, 40(R2)
	JAL	R1, relocfixup(SB)
	MOV	32(R2), R10
	MOV	40(R2), R11
	ADD	$48, R2
	// clear out UP and set MACH.
	MOV $0, R6
	MOV	$_start(SB), R7
	MOV	$MACHSIZE, R8
	SUB	R8, R7
	// allow kernel to read/write user memory.
	MOV CSR(SSTATUS), R8
	OR $0x40000, R8, R8
	MOV R8, CSR(SSTATUS)
	/*
	 * hand a0/a1 to main(hartid, dtb).  The Plan 9 riscv ABI passes the
	 * first argument in RARG and later ones on the stack; arg0 is RARG
	 * (== 1(SP)) and arg1 goes in the next slot at 2*XLEN(SP).
	 */
	MOV R10, RARG			/* arg0 = hartid */
	MOV R11, (2*XLEN)(SP)		/* arg1 = dtb */

	JAL	R1, main(SB)
NDNR:
	MOV $0x30, R10
		MOV $1, R17
		MOV $0, R16
		ECALL
	JMP NDNR

TEXT	stop<>(SB), 1, $-4
_stop:
	WFI
	JMP	_stop

/*
 * Secondary hart entry, started by SBI HSM (hsmstartall) at PADDR(cpumain).
 * We arrive in S-mode with paging OFF; SBI passes a0 (R10) = hartid and
 * a1 (R11) = the opaque value we supplied (also the hartid).  The kernel
 * is already relocated and its page table (sv39, identity-mapped
 * gigapages) is built by the boot hart, so we only enable paging, set up
 * this hart's Mach and stack, and enter C at cpustart().
 *
 * Note: not named _main — that symbol is libc's C-runtime entry point
 * (crt0), and defining our own clobbers kernel startup.
 */
TEXT	cpumain(SB), 1, $-4
	MOV	R10, R12		/* keep hartid across everything */
	MOV	$setSB(SB), R3

	/*
	 * turn on paging with the shared kernel table.  the kernel range is
	 * identity-mapped, so the PC and SB-relative loads keep working
	 * across the switch (the sv39 load below already relies on VA==PA
	 * while paging is off).
	 */
	MOV	sv39(SB), R8		/* L1 table root (phys == virt) */
	SRL	$12, R8
	MOV	$SATP_SV39, R9
	OR	R9, R8, R8
	MOV	R8, CSR(SATP)
	SFENCE

	/* m (MACH == R7) = MACHP(hartid) = ktzero - (hartid+1)*MACHSIZE */
	MOV	$_start(SB), R7
	ADD	$1, R12, R8
	SLL	$13, R8			/* (hartid+1) * MACHSIZE (8 KiB) */
	SUB	R8, R7, R7

	/*
	 * per-hart stack: top of this hart's dedicated cpustack[hartid] slot,
	 * NOT its 8K Mach page (which would overflow when the scheduler runs).
	 */
	MOV	$cpustack(SB), R2	/* base of stack array */
	ADD	$1, R12, R8		/* hartid + 1 slots */
	MOV	$CPUSTACKSIZE, R9
	MUL	R9, R8, R8		/* (hartid+1) * CPUSTACKSIZE */
	ADD	R8, R2, R2		/* top of this hart's stack */
	AND	$~15, R2, R2		/* 16-byte align the SP */

	MOV	$0, R6			/* up = nil */

	/* permit kernel access to user pages (SSTATUS.SUM), as in _start */
	MOV	CSR(SSTATUS), R8
	OR	$0x40000, R8, R8
	MOV	R8, CSR(SSTATUS)

	MOV	R12, R(ARG)		/* cpustart(hartid) */
	JAL	R1, cpustart(SB)
_parkhart:				/* cpustart must not return */
	WFI
	JMP	_parkhart


/* fault-proof memcpy */
/* returns number of bytes not copied in R8. dtpeek seems to want that */
TEXT peek(SB), 1, $-4
	MOV	RARG, R1
	MOV	dst+8(FP), R2
	MOVWU	len+16(FP), R8
TEXT _peekinst(SB), 1, $-4
	MOV	$recktrap(SB), R9
	MOV	R9, CSR(STVEC)		/* if we fault here, report it */
_peekloop:
	MOVB 0(R1), R3
	MOVB R3, 0(R2)
	ADD $1, R2, R2
	ADD $1, R1, R1
	ADD $-1, R8, R8
	BNE R4, _peekloop
_endpeekcopy:
	MOV	R9, CSR(STVEC)
	RET

TEXT peekl(SB), 1, $-4
        MOV     RARG, R1
        MOV     dst+8(FP), R2
        MOVWU   numwords+16(FP), R8
TEXT _peeklinst(SB), 1, $-4
        MOV     $recktrap(SB), R9
        MOV     R9, CSR(STVEC)          /* if we fault here, report it */
_peeklloop:
        MOVW 0(R1), R3
        MOVW R3, 0(R2)
        ADD $4, R2, R2
        ADD $4, R1, R1
        ADD $-1, R8, R8 
        BNE R4, _peekloop
_endpeeklcopy:
        MOV     R9, CSR(STVEC)
        RET

TEXT 	soft(SB), 1, $-4
	RET

//#define SATP 0x180
TEXT	rsatp(SB), 1, $-4
	MOV	CSR(SATP), R8
	RET

TEXT	wsatp(SB), 1, $-4
	LOADSATP(RARG)
	RET

TEXT	rseed(SB), 1, $-4
	MOV	CSR(SEED), R8
	RET

TEXT	putsscratch(SB), 1, $-4
	MOV	R(ARG), CSR(SSCRATCH)
	RET

TEXT	wstvec(SB), 1, $-4
	SFENCE
	MOV	RARG, CSR(STVEC)
	SFENCE
	RET

/*
 * read the byte at RARG under a temporary trap vector; returns the
 * byte, or -1 if the access faults.  CONFADDR may sit inside the SBI
 * firmware's PMP fence (it does on qemu virt), so bootargsinit probes
 * before scanning for a preloaded plan9.ini.  On a fault the handler
 * runs with R9 (saved stvec) and R1 (return pc) still intact.
 */
TEXT	peekb(SB), 1, $-4
	MOV	CSR(STVEC), R9
	MOV	$peekbfault<>(SB), R12
	MOV	R12, CSR(STVEC)
	MOVBU	(RARG), RARG
	MOV	R9, CSR(STVEC)
	RET

TEXT	peekbfault<>(SB), 1, $-4
	MOV	R9, CSR(STVEC)
	MOV	$-1, RARG
	RET

TEXT	mmudisable<>(SB), 1, $-4
	SFENCE
	MOV	R0, CSR(SATP)
	SFENCE
	RET

TEXT	mmuenable<>(SB), 1, $-4
//	MOV	$MAKE_SATP(L1-KZERO), R8
	SFENCE
	MOV	R8, CSR(SATP)
	SFENCE
	RET

TEXT flushalltlb(SB), 1, $-4
	WORD $0x12000073
	RET

TEXT flushvatlb(SB), 1, $-4
	WORD $0x12040073
	RET

TEXT	cas(SB), 1, $-4
TEXT	cmpswap(SB), 1, $-4
	MOVWU	ov+8(FP), R12
	MOVWU	nv+(8+4)(FP), R13
	MOV	R0, R11
	FENCE_RW
_spincas:
	LRW(ARG, 14)
	SLL	$32, R14
	SRL	$32, R14
	BNE	R12, R14, _fail
	FENCE_RW
	SCW(13, ARG, 14)
	BNE	R14, _spincas
	MOV	$1, R11
_fail:
	FENCE_RW
	MOV	R11, R(ARG)
	RET

/* This version of tas stores RA (caller PC) in the location.
 * More useful than just '1' and the code tests for it being
 * non-zero, not just '1'. */
TEXT	tas(SB), 1, $-4
TEXT	_tas(SB), 1, $-4
	FENCE_RW
	AMOW(Amoswap, AQ|RL, 1, ARG, ARG)
	FENCE_RW
	RET

/*
 * wait/atomic primitives used by the idle + cross-cpu-ipi path (archrv.c).
 * ported from the unlinked M-mode mch.s; these are privilege-neutral.
 */
TEXT	halt(SB), 0, $-4
	WFI			/* may be a no-op or hint */
	RET

TEXT	amoswapw(SB), 0, $(2*XLEN)	/* ulong amoswapw(ulong *a, ulong nv) */
	MOVWU	nv+XLEN(FP), R9
	AMOW(Amoswap, AQ|RL, 9, ARG, ARG)	/* *a = nv; old *a -> ret */
	RET

TEXT	aadd(SB), 1, $-4		/* int aadd(int *a, int addend) */
	MOVW	addend+XLEN(FP), R9
	AMOW(Amoadd, AQ|RL, 9, ARG, 10)	/* *a += addend; old *a -> R10 */
	ADDW	R9, R10, R(ARG)		/* return new value */
	SEXT_W(R(ARG))
	RET

/*
 * Label consists of a stack pointer and a programme counter.
 * Restore them and RET. For a forked child the target is forkret(),
 * so on entry SP points at the Ureg but forkret builds its own frame below it;
 * sysrforkret recomputes the Ureg from R(USER) rather than trusting SP.
 */
TEXT	gotolabel(SB), 1, $-4			/* gotolabel(Label *) */
	MOV	0(R(ARG)), R2			/* restore sp */
	MOV	XLEN(R(ARG)), LINK		/* restore return pc */
	MOV	$1, R(ARG)
	RET

TEXT	setlabel(SB), 1, $-4
	MOV	R2, 0(R(ARG))			/* store sp */
	MOV	LINK, XLEN(R(ARG))		/* store return pc */
	MOV	R0, R(ARG)
	RET

/* NOTE: this is the SHADOW register of the memory-mapped
 * MTIME register. We will now use STIMECMP for interrupts. */
#define TIME	0xc01
TEXT	rdtime(SB), 1, $-4
	FENCE
	MOV	CSR(TIME), R8
	RET

#define STIMECMP 0x14D
TEXT wrstimecmp(SB), 1, $-4
	FENCE
	MOV R(ARG), CSR(STIMECMP)
	RET

TEXT rdstimecmp(SB), 1, $-4
	FENCE
	MOV CSR(STIMECMP), R(ARG)
	RET

/*
 * this hart's S-level IMSIC interrupt file (AIA).  Its registers are
 * reached indirectly: a register number goes in CSR siselect and the
 * data moves through CSR sireg.
 */
#define SISELECT	0x150
#define SIREG		0x151
#define STOPEI		0x15c

TEXT imsicswr(SB), 1, $-4		/* imsicswr(sel, val) */
	MOV	R(ARG), CSR(SISELECT)
	MOV	val+XLEN(FP), R9
	MOV	R9, CSR(SIREG)
	RET

/* claim the top pending external interrupt: (id<<16|prio), 0 if none */
TEXT stopeiclaim(SB), 1, $-4
	CSRRW	CSR(STOPEI), R0, R(ARG)
	RET

TEXT	coherence(SB), 1, $-4
	FENCE
	RET

TEXT	wfi(SB), 1, $-4
	RET

TEXT	flushtlb(SB), 1, $-4
	RET
TEXT	flushasidvall(SB), 1, $-4
	RET
TEXT	flushasidva(SB), 1, $-4
	RET
TEXT	flushasid(SB), 1, $-4
	RET

/*
 * cycle counter (CSR 0xc00): per core (not per hart) and may pause during WFI.
 * S-mode access needs mcounteren.CY (OpenSBI sets it); user mode needs scounteren.CY.
 */
TEXT	vcycles(SB), 1, $-4
	MOV	CSR(CYCLO), R(ARG)
	RET

/*
 * elapsed time for the scheduler: reads the time counter (CSR 0xc01),
 * not the cycle counter. Time is monotonic, doesn't pause during WFI.
 */
TEXT	perfticks(SB), 1, $-4
	MOV	CSR(TIME), R(ARG)
	RET

/* takes scounteren bits to let user mode read the cycle/time/instret CSRs */
TEXT	putscounteren(SB), 1, $-4
	MOV	R(ARG), CSR(SCOUNTEREN)
	RET

/*
 * switch to user mode with stack pointer from R(ARG), at start of text.
 * used to start process 1 (init).
 */
TEXT touser(SB), 1, $-4
	//JAL R1, _fpuinit(SB)
	FENCE
	FENCE_I
	CSRRW	CSR(SSCRATCH), R(MACH), R(MACH) /* restore R7, reload saved m */
	/*
	 * Drop sstatus.SPP to 0 so SRET returns to user mode (not supervisor),
	 * and set SPIE so SIE = 1 after SRET (user can be preempted by clock).
	 * Read-modify-write the whole register — same idiom as _start uses
	 * for setting SUM.
	 */
	MOV	CSR(SSTATUS), R9
	MOV	$~0x100, R10		/* ~Spp */
	AND	R10, R9, R9		/* clear SPP */
	OR	$0x20, R9, R9		/* set SPIE */
	MOV	R9, CSR(SSTATUS)
	MOV	$(UTZERO+0x28), R12	/* skip unextended exec hdr of init */
	MOV	R12, CSR(SEPC)		/* new pc */
	MOV	RARG, R2		/* new sp */
	WORD $0x10200073 //SRET

// Courtesy of Geoff
/*
 * Turn the Sie bit of SSTATUS on or off; leave the individual enables alone.
 * NB: the spl* functions contain barriers and it is safe to rely upon that.
 * Do we need these? Not sure yet. Leave it here for now.
 */

TEXT splhi(SB), 1, $-4
TEXT gsplhi(SB), 1, $-4
_splhi:
	CSRRC	CSR(SSTATUS), $Sie, R(ARG) /* disable super intrs */
	AND	$Sie, R(ARG)		/* return old state */
	BEQ	R(ARG), _spldone	/* intrs were disabled? */
	MOV	LINK, SPLPC(R(MACH)) 	/* save PC in m->splpc for kprof */
	FENCE
_spldone:
	RET

TEXT spllo(SB), 1, $-4
TEXT gspllo(SB), 1, $-4			/* marker for devkprof */
_spllo:
	MOV	CSR(SSTATUS), R(ARG)
	AND	$Sie, R(ARG)		/* return old state */
	BNE	R(ARG), _spldone	/* Sie=1, intrs are enabled? */
	/* they are disabled, so enable them */
	FENCE
	/*
	 * strictly speaking, SPLLO should be after zeroing SPLPC,
	 * but since even the profiling clock can't interrupt splhi,
	 * we get more meaningful profiling results with SPLLO before it.
	 */
	SPLLO				/* enable super intrs; expect intr. */
	MOV	R0, SPLPC(R(MACH))	/* enabling; clear m->splpc */
	RET

/* assumed between spllo and spldone by devkprof */
TEXT gsplx(SB), 1, $-4
TEXT splx(SB), 1, $-4
	MOV	CSR(SSTATUS), R9
	AND	$Sie, R9
	AND	$Sie, R(ARG), R10
	BEQ	R9, R10, _spldone	/* already in desired state? */
	BEQ	R10, _splhi		/* want intrs disabled? */
	JMP	_spllo			/* want intrs enabled */

TEXT spldone(SB), 1, $-4		/* marker for devkprof */
	RET

TEXT gislo(SB), 1, $-4
TEXT islo(SB), 1, $-4
	MOV	CSR(SSTATUS), R(ARG)
	AND	$Sie, R(ARG)
	RET

#define CBOINVAL 0
#define CBOCLEAN 1		/* write back iff dirty */
#define CBOFLUSH 2		/* clean then invalidate */

#define CBOZERO 4		/* Zicboz */

/* Zicbom extension; takes virtual addresses */
TEXT cboclean(SB), 1, $-4			/* void cboclean(uintptr) */
	WORD $(0xf | 2<<12 | ARG<<15 | CBOCLEAN<<20)
	RET

/* Zicbom extension; takes virtual addresses */
TEXT cboflush(SB), 1, $-4			/* void cboflush(uintptr) */
	WORD $(0xf | 2<<12 | ARG<<15 | CBOFLUSH<<20)
	RET

/* Zicbom extension; takes virtual addresses */
TEXT cboinval(SB), 1, $-4			/* void cboinval(uintptr) */
	WORD $(0xf | 2<<12 | ARG<<15 | CBOINVAL<<20)
	RET

// laterz
TEXT	cacheiinvse(SB), 1, $-4
	RET

TEXT clockenable(SB), 1, $-4
	MOV	$Stie, R9			/* super timer intr enable */
	CSRRS	CSR(SIE), R9, R0
	FENCE
	RET

TEXT clrstie(SB), 1, $-4
	MOV	$Stie, R(ARG)			/* super timer intr enable */
	CSRRC	CSR(SIE), R(ARG), R0
TEXT clrsipbit(SB), 1, $-4			/* uvlong clrsipbit(ulong bit) */
	CSRRC	CSR(SIP), R(ARG), R0
	FENCE
	MOV	CSR(SIP), R(ARG)
	RET

// The 9k port is amazing, it supports M mode. 

TEXT getmsts(SB), 1, $-4
	MOV	CSR(MSTATUS), R(ARG)
	RET
TEXT putmsts(SB), 1, $-4
	BARR_SFENCE_VMA(0, 0)
	MOV	R(ARG), CSR(MSTATUS)
	BARR_SFENCE_VMA(0, 0)
	FENCE
	FENCE_I
	RET

TEXT getsie(SB), 1, $-4
	MOV	CSR(SIE), R(ARG)
	RET
TEXT putsie(SB), 1, $-4
	MOV	R(ARG), CSR(SIE)
	FENCE
	RET
TEXT clrsie(SB), 1, $-4				/* returns old SIE */
	CSRRC	CSR(SIE), R(ARG), R(ARG)
	RET
TEXT setsie(SB), 1, $-4				/* returns old SIE */
	CSRRS	CSR(SIE), R(ARG), R(ARG)
	RET

/*
 * the cycle counters are per core (not per hart) and may stop during WFI;
 * avoid them when measuring elapsed time.
 */
TEXT rdtsc(SB), 1, $-4				/* Time Stamp Counter */
	FENCE
	MOV	CSR(CYCLO), R(ARG)
	RET

/* from Geoff */
TEXT getsip(SB), 1, $-4
	MOV	CSR(SIP), R(ARG)
	RET
TEXT putsip(SB), 1, $-4
	MOV	R(ARG), CSR(SIP)
	FENCE
	RET
TEXT getsp(SB), 1, $-4
	MOV	R2, R(ARG)
	RET
TEXT setsp(SB), 1,  $-4
	MOV	R(ARG), R2
