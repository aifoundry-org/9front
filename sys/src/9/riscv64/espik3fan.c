/*
 * espik3fan.c -- fan control for k3-pico-itx, over the CSCE2010 EC.
 *
 * This is the one thing in this tree today that needs the EC (see
 * espik3.c for the eSPI controller + cros_ec host-command transport
 * itself, which knows nothing about fans specifically). If something
 * else on this board ever needs the EC -- battery status, a switch
 * state, whatever else lives in EC_MEMMAP_*, another host command --
 * it's a new file like this one, calling k3espiready()/k3eccmd()/
 * k3ecmemmapread() from espik3.c, not a reason to touch that file.
 *
 * DESIGN: one ctl/status pair per fan, Plan 9 style, instead of one
 * file per concept. This board has exactly one fan, but the naming
 * (fanNctl/fanNstatus, N starting at 0) leaves room for more without
 * a rename later -- cheap to reserve, awkward to retrofit. Today N is
 * always 0 (FanIdx0 below); a second fan would mean looping
 * k3ecfaninit() over however many the EC reports and publishing
 * fan1ctl/fan1status alongside these, not restructuring anything.
 *
 *   #P/fan0status  r   measured RPM (EC_MEMMAP_FAN, plain memmap
 *                      read, no command/busy-wait), or the literal
 *                      string "absent"/"stalled" for the EC's own
 *                      sentinel values.
 *   #P/fan0ctl     rw  read: "auto", or the last target RPM this
 *                      driver commanded (see FIELD NOTE below for why
 *                      it's not a live readback). write: the literal
 *                      string "auto" hands the fan back to the EC's
 *                      own thermal curve; a decimal number takes
 *                      manual control and sets that target RPM.
 *
 * By construction, fan0status always reads back exactly one bare
 * token in the grammar fan0ctl's write side accepts (a decimal
 * number, or one of the sentinel words, which parse as 0 -- harmless
 * either way: "no fan" or "not spinning" both make 0 a reasonable
 * target). So `cat fan0status > fan0ctl` is well-defined: it pins the
 * fan at whatever RPM it happens to be doing right now. Deliberately
 * did NOT make fan0status multi-field (e.g. a "mode" line plus an
 * "rpm" line): parsecmd()/Cmdbuf tokenize across ALL whitespace
 * including newlines, not line by line, so a multi-field status would
 * still "work" when piped into ctl (extra tokens are silently
 * ignored, since ctl only looks at cb->f[0]) but would gain nothing
 * and give up the single-token round-trip property for no benefit.
 *
 * FIELD NOTE (real hardware, k3-pico-itx): the underlying transport
 * (espik3.c) and SET_FAN_TARGET_RPM/THERMAL_AUTO_FAN_CTRL(SET) are
 * confirmed working end to end. But this particular EC firmware image
 * has a consistent SET-works/GET-doesn't pattern: both
 * EC_CMD_PWM_GET_FAN_TARGET_RPM and the GET direction of
 * EC_CMD_THERMAL_AUTO_FAN_CTRL either fail outright or answer with a
 * stub value (0) that never reflects reality, while their SET
 * counterparts genuinely work (the physical fan responds, confirmed
 * via fan0status). So fan0ctl's read side does NOT call either GET
 * command -- it reports isauto/lasttarget, this driver's own record
 * of the last SET it successfully issued, which is the trustworthy
 * source of truth on this hardware. See ecsetfanauto() and
 * fan0ctlwrite() below.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"

enum {
	MemmapFan		= 0x10,		/* EC_MEMMAP_FAN */
	EcCmdPwmSetFanTargetRpm	= 0x0021,	/* CONFIRMED, cros_ec_commands.h */
	EcCmdThermalAutoFanCtrl	= 0x0052,	/* CONFIRMED, cros_ec_commands.h */
	EcAutoFanCtrlCmdSet	= 0,
	FanIdx0			= 0,		/* this board: one fan; see DESIGN above */
};

static long
fan0statusread(Chan*, void *a, long n, vlong off)
{
	char str[32];
	uchar b[2];
	int rpm;

	if(k3ecmemmapread(MemmapFan, b, 2) < 0)
		error(Eio);
	rpm = b[0] | b[1]<<8;
	if(rpm == 0xFFFF)
		snprint(str, sizeof str, "absent\n");
	else if(rpm == 0xFFFE)
		snprint(str, sizeof str, "stalled\n");
	else
		snprint(str, sizeof str, "%d\n", rpm);
	return readstr(off, a, n, str);
}

/*
 * struct ec_params_auto_fan_ctrl_v2 { uint8_t fan_idx; uint8_t cmd;
 * uint8_t set_auto; }, CONFIRMED against the real cros_ec_commands.h.
 * autoval: 1 = automatic (EC thermal curve owns the fan), 0 = manual
 * (the last SET_FAN_TARGET_RPM value sticks). Best-effort: not every
 * EC firmware necessarily implements this command at this version,
 * and if it's missing we'd still rather go ahead and try
 * SET_FAN_TARGET_RPM than refuse outright -- so failure here is not
 * fatal to the caller. Updates isauto on success, which is what
 * fan0ctlread() trusts instead of the (confirmed broken on this EC)
 * GET direction of this same command.
 */
static int isauto;
static ulong lasttarget;

static int
ecsetfanauto(int fanidx, int autoval)
{
	uchar req[3];
	int ret;

	req[0] = fanidx;
	req[1] = EcAutoFanCtrlCmdSet;
	req[2] = autoval != 0;
	ret = k3eccmd(EcCmdThermalAutoFanCtrl, 2, req, sizeof req, nil, 0);
	if(ret >= 0)
		isauto = (autoval != 0);
	return ret;
}

static long
fan0ctlread(Chan*, void *a, long n, vlong off)
{
	char str[32];

	if(isauto)
		snprint(str, sizeof str, "auto\n");
	else
		snprint(str, sizeof str, "%lud\n", lasttarget);
	return readstr(off, a, n, str);
}

static long
fan0ctlwrite(Chan*, void *a, long n, vlong)
{
	Cmdbuf *cb;
	uchar req[4];
	ulong rpm;

	cb = parsecmd(a, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	if(cb->nf < 1)
		error(Ebadctl);

	if(strcmp(cb->f[0], "auto") == 0){
		if(ecsetfanauto(FanIdx0, 1) < 0)
			error(Eio);
		poperror();
		free(cb);
		return n;
	}

	/*
	 * Take manual control first -- on real hardware, a target
	 * written while the EC's thermal engine still owned the fan got
	 * silently overwritten back to 0 within one control tick.
	 * Best-effort: proceed to SET_FAN_TARGET_RPM regardless of
	 * whether this particular EC firmware implements the switch.
	 */
	ecsetfanauto(FanIdx0, 0);

	rpm = strtoul(cb->f[0], nil, 10);
	/* struct ec_params_pwm_set_fan_target_rpm_v0 { uint32_t rpm; },
	 * CONFIRMED against the real cros_ec_commands.h -- 4 bytes. */
	req[0] = rpm & 0xFF;
	req[1] = (rpm>>8) & 0xFF;
	req[2] = (rpm>>16) & 0xFF;
	req[3] = (rpm>>24) & 0xFF;
	if(k3eccmd(EcCmdPwmSetFanTargetRpm, 0, req, sizeof req, nil, 0) < 0)
		error(Eio);
	lasttarget = rpm;
	poperror();
	free(cb);
	return n;
}

void
k3ecfaninit(void)
{
	if(!k3espiready())
		return;
	/*
	 * Boot state: explicitly put the fan under automatic control
	 * rather than trusting whatever the EC happened to power up
	 * already at. Best-effort, same as every other ecsetfanauto()
	 * call.
	 */
	ecsetfanauto(FanIdx0, 1);
	addarchfile("fan0status", 0444, fan0statusread, nil);
	addarchfile("fan0ctl", 0664, fan0ctlread, fan0ctlwrite);
}
