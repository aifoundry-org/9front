#ifndef __SETJMP_H
#define __SETJMP_H
#pragma lib "/$M/lib/ape/libap.a"

/*
 * jmp_buf is architecture-specific; /$objtype/include/ape/setjmp.h may
 * define _SETJMP_ARCH_H_ and its own jmp_buf (e.g. 8-byte words on
 * 64-bit) before including this file.  the default suits 32-bit.
 */
#ifndef _SETJMP_ARCH_H_
typedef int jmp_buf[10];
#ifdef _POSIX_SOURCE
typedef int sigjmp_buf[10];
#endif
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
