#ifndef __SETJMP_H
#define __SETJMP_H
#pragma lib "/$M/lib/ape/libap.a"

/*
 * riscv64: jmp_buf holds 8-byte words (sp, pc, ...) and is 8-aligned,
 * matching the ABI and real compilers.  the generic int[10] would be
 * 4-aligned and indexed as 4-byte ints, truncating the saved sp/pc and
 * (via genarch measuring jmp_buf's alignment) under-aligning gs structs.
 */
typedef long long jmp_buf[10];
#ifdef _POSIX_SOURCE
typedef long long sigjmp_buf[10];
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int setjmp(jmp_buf);
extern void longjmp(jmp_buf, int);

#ifdef _POSIX_SOURCE
extern int sigsetjmp(sigjmp_buf, int);
extern void siglongjmp(sigjmp_buf, int);
#endif

#ifdef __cplusplus
}
#endif

#endif /* __SETJMP_H */
