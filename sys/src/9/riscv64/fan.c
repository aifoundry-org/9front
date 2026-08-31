/*
 * Shared parse/format for the fanNctl / fanNstatus files, so every
 * board's fan driver speaks the same grammar even though the unit it
 * regulates in differs with the hardware:
 *
 *	espik3fan.c	EC closes the loop on rpm		unit "rpm"
 *	i2ck3fan.c	CTF2301 is open-loop pwm duty		unit "%"
 *	k1thermal.c	raw SoC pwm to a mosfet, no feedback	unit "%"
 *
 * A file reads back "auto\n", or "<value> <unit>\n". A write takes the
 * literal "auto", a bare number (interpreted in the board's own unit,
 * so `cp fan0status fan0ctl` works where they share a unit), or a
 * number followed by an explicit unit. The explicit unit is an
 * assertion: it must match the unit this board controls in, otherwise
 * the write fails rather than silently doing the wrong thing -- e.g.
 * "2400 rpm" written to a pwm-only board errors instead of being
 * mistaken for a duty. Likewise "auto" on a board that has no hardware
 * auto (hasauto == 0) errors rather than being a no-op.
 *
 * fanparse() does no unit conversion and no range clamping: it returns
 * the number in the board's unit and leaves the driver to clamp to
 * whatever its hardware accepts. Nothing here fakes a control loop the
 * silicon can't do.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"

/* format one ctl/status value: "auto\n", or "<val> <unit>\n" */
int
fanfmt(char *buf, int nbuf, char *unit, long val, int isauto)
{
	if(isauto)
		return snprint(buf, nbuf, "auto\n");
	return snprint(buf, nbuf, "%ld %s\n", val, unit);
}

/*
 * Parse a fanNctl write. unit is the board's native unit ("%"/"rpm"),
 * hasauto says whether "auto" is accepted. Sets *isauto and returns 0
 * for the "auto" request; otherwise returns the numeric setpoint in
 * unit. Calls error() -- so never returns -- for anything this board
 * can't honour.
 */
long
fanparse(Cmdbuf *cb, char *unit, int hasauto, int *isauto)
{
	char err[ERRMAX], *s;

	*isauto = 0;
	if(cb->nf < 1)
		error(Ebadctl);
	if(cistrcmp(cb->f[0], "auto") == 0){
		if(!hasauto)
			error("fan: auto unsupported");
		*isauto = 1;
		return 0;
	}
	s = cb->f[0];
	if(*s == '+' || *s == '-')
		s++;
	if(*s < '0' || *s > '9')
		error(Ebadctl);
	if(cb->nf >= 2 && cistrcmp(cb->f[1], unit) != 0){
		snprint(err, sizeof err, "fan: %s unsupported", cb->f[1]);
		error(err);
	}
	return strtol(cb->f[0], nil, 10);
}
