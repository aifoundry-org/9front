/*
 * _mainp - APE startup with profiling.  same hand-off as main9.s, but
 * _callmain runs _profmain (which enables profiling, then _apemain).
 */
#define FCSR	3

GLOBL	_tos(SB), $XLEN

TEXT	_mainp(SB), 1, $0
	MOV	$setSB(SB), R3
	MOV	R0, CSR(FCSR)

	/* _tos = arg */
	MOV	R8, _tos(SB)

	SUB	$XLEN, R2		/* + XLEN auto LR-save frame = 2*XLEN; see main9.s */
	MOV	$_profmain(SB), R8	/* f, first arg in REGARG */
	JAL	R1, _callmain(SB)	/* does not return */

	JAL	R1, exit(SB)
	RET

TEXT	_savearg(SB), 1, $0
TEXT	_saveret(SB), 1, $0
	RET

TEXT	_callpc(SB), 1, $0
	MOV	argp+0(FP), R8
	MOV	XLEN(R8), R8
	RET
