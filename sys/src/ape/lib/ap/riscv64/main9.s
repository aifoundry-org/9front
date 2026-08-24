/*
 * APE startup following exec.  hand off to _callmain (callmain.c),
 * which sets _privates, _nprivates, _errnoloc and _plan9err before
 * running _apemain.  the old hand-written startup omitted _errnoloc
 * and _plan9err, so the first failed syscall faulted in _syserrno.
 *
 * the kernel enters with the tos pointer in R8 (REGARG) and the stack
 * laid out as [argc][argv0][argv1]...  _callmain(f, argc, arg0) wants
 * its stack args at 2*XLEN(SP) and 3*XLEN(SP), so SP must drop by 2*XLEN
 * before the call to line argc/argv up; f is passed in R8.
 *
 * because _main calls out with JAL, the linker gives it an automatic
 * XLEN-byte frame to save LR, which already accounts for one of the two
 * slots.  so we only SUB the remaining XLEN by hand.  (don't "fix" this
 * to 2*XLEN: arm64 uses 2*XLEN only because it tail-jumps and is a leaf.)
 */
#define FCSR	3

GLOBL	_tos(SB), $XLEN

TEXT	_main(SB), 1, $0
	MOV	$setSB(SB), R3
	MOV	R0, CSR(FCSR)

	/* _tos = arg */
	MOV	R8, _tos(SB)

	SUB	$XLEN, R2		/* + XLEN auto LR-save frame = 2*XLEN */
	MOV	$_apemain(SB), R8	/* f, first arg in REGARG */
	JAL	R1, _callmain(SB)	/* does not return */

	JAL	R1, exit(SB)
	RET
