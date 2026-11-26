/*
 * this hart's supervisor timer compare register, stimecmp (Sstc);
 * only used when soc.sstc is set, see wrstimecmp
 */
#include "riscv64l.h"

#include "mem.h"

#define STIMECMP 0x14D
TEXT csrwrstimecmp(SB), 1, $-4
	FENCE
	MOV R(ARG), CSR(STIMECMP)
	RET

TEXT csrrdstimecmp(SB), 1, $-4
	FENCE
	MOV CSR(STIMECMP), R(ARG)
	RET
