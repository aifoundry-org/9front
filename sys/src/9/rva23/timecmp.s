/*
 * this hart's supervisor timer compare register, stimecmp (Sstc).  A
 * board without Sstc provides wrstimecmp and rdstimecmp in timecmp.c
 * instead.
 */
#include "riscv64l.h"

#include "mem.h"

#define STIMECMP 0x14D
TEXT wrstimecmp(SB), 1, $-4
	FENCE
	MOV R(ARG), CSR(STIMECMP)
	RET

TEXT rdstimecmp(SB), 1, $-4
	FENCE
	MOV CSR(STIMECMP), R(ARG)
	RET
