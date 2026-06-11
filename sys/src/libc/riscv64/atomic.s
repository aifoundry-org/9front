#include <atom.h>

#define ARG 8

/*  get variants */
TEXT agetl+0(SB),1,$0
	FENCE_RW
	LRW(ARG, ARG)
	FENCE_RW
	RET	

/*  set variants */
TEXT asetl+0(SB),1,$0
	MOVW	val+XLEN(FP), R12
	FENCE_RW
	AMOW(Amoswap, AQ|RL, 12, ARG, 10)
	FENCE_RW
	MOVW	R10, RARG
	RET

/*  inc variants */
TEXT aincl+0(SB),1,$0
	MOVW    d+XLEN(FP), R9
	FENCE_RW	/* flush changes to ram in case releasing a lock */
	/* after: value before add in R10, value after add in memory */
	AMOW(Amoadd, AQ|RL, 9, ARG, 10)
	FENCE_RW
	ADDW    R9, R10, RARG		/* old value ±1 for ainc/adec */
	RET

/*  cas variants */
TEXT acasl+0(SB),1,$0
	MOVWU	ov+XLEN(FP), R12
	MOVWU	nv+(XLEN+4)(FP), R13
	MOV	R0, R11		/* default to failure */
	FENCE_RW
spincas:
	LRW(ARG, 14)		/* (RARG) -> R14 */
	SLL	$32, R14
	SRL	$32, R14	/* don't sign extend */
	BNE	R12, R14, fail
	FENCE_RW
	SCW(13, ARG, 14)	/* R13 -> (RARG) maybe, R14=0 if ok */
	BNE	R14, spincas	/* R14 != 0 means store failed */
ok:
	MOV	$1, R11
fail:
	FENCE_RW
	MOV	R11, RARG
	RET

/*
 * swap variants
 *
 *	long	aswapl(Along *, long);
 *	void*	aswapp(Aptr *,  void *);
 */
TEXT aswapl(SB), 1, $-4
	MOVW	new+XLEN(FP), R9
	FENCE_RW
	AMOW(Amoswap, AQ|RL, 9, ARG, ARG)
	FENCE_RW
	RET

TEXT aswapp(SB), 1, $-4
	MOV	new+XLEN(FP), R9
	FENCE_RW
	AMOD(Amoswap, AQ|RL, 9, ARG, ARG)
	FENCE_RW
	RET

/* barriers */
TEXT coherence+0(SB),1,$0
	FENCE_RW
	RET

