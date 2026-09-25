/*
 * screen.h -- adapted for the K3 Pico-ITX from riscv64-video-branch's
 * own screen.h. Removed: the hdmi.c/i2cddc.c/v2d.c declarations
 * (including struct Mode, the EDID-derived timing type) -- none of
 * those files were ported, since this board's video path (DP1, real
 * link training) is entirely different from that reference kernel's
 * own HDMI/firmware-inherited one. See screen.c's own header for the
 * full explanation.
 */

/* devmouse.c */
typedef struct Cursor Cursor;
extern Cursor cursor;
extern void mousetrack(int, int, int, ulong);
extern void absmousetrack(int, int, int, ulong);
extern Point mousexy(void);
extern void mouseaccelerate(int);

/* screen.c */
extern Memimage *gscreen;
extern void	screeninit(void);
extern char*	screensettilt(char*);
extern int	screentilt(void);
extern char	*tiltstr[4];
extern void	blankscreen(int);
extern void	flushmemscreen(Rectangle);
extern Memdata*	attachscreen(Rectangle*, ulong*, int*, int*, int*);
extern void	cursoron(void);
extern void	cursoroff(void);
extern void	setcursor(Cursor*);

extern void mousectl(Cmdbuf*);
extern void mouseresize(void);
extern void mouseredraw(void);

/* devdraw.c */
extern QLock	drawlock;

#define ishwimage(i)	1		/* for ../port/devdraw.c */

/* swcursor.c */
void		swcursorhide(int);
void		swcursoravoid(Rectangle);
void		swcursordraw(Point);
void		swcursorload(Cursor *);
void		swcursorinit(void);
