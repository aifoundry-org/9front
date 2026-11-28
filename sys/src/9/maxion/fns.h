#include "../rva23/fns.h"

/* satp's mode is Sv48, for the four-level tables (mem.h, memmap.c) */
#define wsatp(v)	wsatp48(v)
void	wsatp48(u64int);
