/*
 * Microchip PolarFire SoC: the rva23 memory layout, booted by u-boot's
 * booti at 0x80200000 on a U54 hart.  Hart 0 is the E51 monitor core,
 * which only runs M-mode (the HSS), so cpu0 is hart 1, whose S-mode
 * PLIC context is 2: the E51 has an M context only.
 */
#include "../rva23/mem.h"

#undef FIRSTHART
#define FIRSTHART	1

#undef PLICSCTXT
#define PLICSCTXT	2

#undef TEXTOFF
#define TEXTOFF		0x200000

/*
 * DDR the HSS maps non-cached at 0xc0000000: the DTB's
 * non-cached-low-buffer, kept out of the RAM banks by its
 * alloc-ranges.  The U54s have no cache-block operations, so
 * device DMA goes through here.
 */
#define NCDMABASE	0xc4000000ULL
#define NCDMASIZE	(64*MiB)
