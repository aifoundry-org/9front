#include "../rva23/fns.h"

/* gpiok3 */
extern void gpiok3init(void);

/* k3thermal */
extern void k3thermalinit(void);

/* espik3 -- eSPI controller + cros_ec host-command transport, board-gated and
   lazily brought up on first call. Any future EC consumer uses these three */
extern int k3espiready(void);
extern int k3eccmd(int cmd, int ver, void *tx, int txlen, void *rx, int rxlen);
extern int k3ecmemmapread(int off, void *buf, int n);

/* espik3fan */
extern void k3ecfaninit(void);
/* screenpicoitx.c: DP1 framebuffer driver entry point plus the two
 * accessors screen.c's screeninit() needs (shared with screencom260.c). */
extern void dpuk3(void);
extern uvlong dpuk3fbaddr(void);
extern void* dpuk3fbkaddr(void);
/* screencom260.c: DP1 framebuffer driver entry point for the K3 CoM260
 * IFX carrier board -- see that file's own header for how it differs
 * from screenpicoitx.c's dpuk3(). */
extern void dpuk3com260(void);
/* screen.c */
extern void screeninit(void);
