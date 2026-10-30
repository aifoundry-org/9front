#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

/*
 * Board device setup for the SpacemiT K1/K3 boards and QEMU virt, run
 * from archinit.  A SoC directory with other devices supplies its own.
 */
void
boardinit(void)
{
	gpiok3init();
	k1thermalinit();
	k3thermalinit();
	k3ecfaninit();

	/* REVERTED: this sequence was briefly moved into archdevtab's own
	 * "reset" slot (a separate archdevreset() function), on the
	 * theory that Plan 9 runs every device's reset() before any
	 * device's init(), and that devdraw.c's own reset() refuses to
	 * register the #v device character without a screen exisiting
	 * yet. Both premises were wrong, confirmed by reading the real
	 * source: chandevinit() (chan.c) only ever calls init() on each
	 * device -- reset() is never called from there at all, so
	 * archdevreset() never ran, silently dropping this entire
	 * sequence. And devdraw.c's own drawdevtab uses plain devreset/
	 * devinit with no gscreen check anywhere -- its device character
	 * is 'i' (bind #i, not #v), which is what actually caused
	 * "unknown device": #v was never draw's letter in the first
	 * place. Back in archinit() (devtab[]'s first entry, so this
	 * still runs before every other device's own init(), which is
	 * all that was ever needed).
	 */
	/* ADDENDUM: one plain call each, no condition visible here at
	 * all -- matching how riscv64-video-branch's own devarch.c calls
	 * its own hardware bring-up ("dpuinit(); v2dinit(); screeninit();"
	 * in its own archinit(), no "if" around any of them). dpuk3()
	 * (screenpicoitx.c) checks the board itself, at its own very
	 * first line, and does nothing at all if this isn't a K3
	 * Pico-ITX -- the same pattern that reference kernel's own
	 * dpuinit() uses for its own board check. dpuk3com260()
	 * (screencom260.c) does the same for the CoM260 IFX carrier; at
	 * most one of the two does anything on a given boot. screeninit()
	 * (screen.c) is unconditional here for the same reason the
	 * reference kernel's own screeninit() call is: it has its own "is
	 * there really a framebuffer" check (dpuk3fbaddr() returning 0), so it
	 * doesn't need devarch.c to gate it either. See
	 * k3-9front-display-investigation.md for the full history. */
	dpuk3();
	dpuk3com260();
	screeninit();

	/* Diagnostic-only calls, not needed for normal boot now that DP1
	 * is confirmed working end to end -- kept defined in
	 * screenpicoitx.c for future debugging, just not called here:
	 *   dpuk3fbtest()             paints a test pattern into the
	 *                             framebuffer (overwrites whatever
	 *                             the real graphics stack puts there)
	 *   dpuk3dpdump()             dumps all ~190 DP1 registers
	 *   dpuk3auxtest()            one-byte DPCD_REV read (superseded
	 *                             by dpuk3linktrain()'s own real,
	 *                             larger AUX reads)
	 *   dpuk3ecpdtest()
	 *   dpuk3ectypecstatustest()  cros_ec USB-C status commands --
	 *   dpuk3ecucsitest()         all three always returned 0 bytes
	 *                             (stub pattern) in every test run
	 *                             this whole investigation, no real
	 *                             data ever obtained from any of them
	 */
}
