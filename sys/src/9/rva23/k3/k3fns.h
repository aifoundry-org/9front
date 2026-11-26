/* k3 */
extern void gpiok3init(void);
extern void k3poweroff(void);
extern void k3thermalinit(void);

/* eSPI and cros_ec host commands, brought up on first use */
extern int k3espiready(void);
extern int k3eccmd(int cmd, int ver, void *tx, int txlen, void *rx, int rxlen);
extern int k3ecmemmapread(int off, void *buf, int n);
extern void k3ecfaninit(void);

/* CTF2301 fan controller of the CoM260 family, over i2c */
extern void i2ck3faninit(void);

/* DP1 framebuffer: Pico-ITX, and the CoM260 IFX carrier */
extern void dpuk3(void);
extern uvlong dpuk3fbaddr(void);
extern void* dpuk3fbkaddr(void);
extern void dpuk3com260(void);
