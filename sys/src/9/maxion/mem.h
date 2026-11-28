#include "../rva23/mem.h"

/* four-level page tables, for Sv48 (memmap.c) */
#undef EVASHIFT
#define EVASHIFT	48

/* uniprocessor */
#undef MAXMACH
#define MAXMACH		1
