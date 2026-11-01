/*
 * compare-and-swap with LR/SC (the A extension).  A board whose harts
 * must not use LR provides its own cmpswap in cas.c instead.
 */
#include "riscv64l.h"

#include "mem.h"

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
