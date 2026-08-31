/*
 * Fan control for k3-pico-itx, over the CSCE2010 EC
 *
 * DESIGN: one ctl/status pair per fan, Plan 9 style, instead of one
 * file per concept. This board has exactly one fan, but the naming
 * (fanNctl/fanNstatus, N starting at 0) leaves room for more without
 * a rename later -- cheap to reserve, awkward to retrofit. Today N is
 * always 0 (FanIdx0 below); a second fan would mean looping
 * k3ecfaninit() over however many the EC reports and publishing
 * fan1ctl/fan1status alongside these, not restructuring anything
 *
 * Both use the shared fanparse()/fanfmt() grammar (fan.c); the unit
 * here is "rpm" because the EC closes the loop on rpm.
 *
 *   #P/fan0status  r   measured "<rpm> rpm" (EC_MEMMAP_FAN, plain
 *                      memmap read, no command/busy-wait), or the
 *                      literal string "absent"/"stalled" for the EC's
 *                      own sentinel values
 *   #P/fan0ctl     rw  read: "auto", or "<rpm> rpm" -- the last target
 *                      this driver commanded (see FIELD NOTE below for
 *                      why it's not a live readback). write: "auto"
 *                      hands the fan back to the EC's own thermal
 *                      curve; a number (bare, or "<rpm> rpm") takes
 *                      manual control and sets that target rpm
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
 * fan0ctlwrite() below
 *
 * Based on:
 *   espik3.c - K3 eSPI controller driver
 *   cros_ec_commands.h - ABI constants
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
	FanIdx0			= 0,		/* this board: one fan */
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
		fanfmt(str, sizeof str, "rpm", rpm, 0);
	return readstr(off, a, n, str);
}

/*
 * struct ec_params_auto_fan_ctrl_v2 { uint8_t fan_idx; uint8_t cmd;
 * uint8_t set_auto; }, CONFIRMED against the real cros_ec_commands.h.
 * autoval: 1 = automatic (EC thermal curve owns the fan), 0 = manual
 * (the last SET_FAN_TARGET_RPM value sticks)
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

	fanfmt(str, sizeof str, "rpm", lasttarget, isauto);
	return readstr(off, a, n, str);
}

static long
fan0ctlwrite(Chan*, void *a, long n, vlong)
{
	Cmdbuf *cb;
	uchar req[4];
	ulong rpm;
	int au;

	cb = parsecmd(a, n);
	if(waserror()){
		free(cb);
		nexterror();
	}

	rpm = fanparse(cb, "rpm", 1, &au);
	if(au){
		if(ecsetfanauto(FanIdx0, 1) < 0)
			error(Eio);
		poperror();
		free(cb);
		return n;
	}

	/*
	 * Take manual control first -- on real hardware, a target
	 * written while the EC's thermal engine still owned the fan got
	 * silently overwritten back to 0 within one control tick
	 */
	ecsetfanauto(FanIdx0, 0);

	/* struct ec_params_pwm_set_fan_target_rpm_v0 { uint32_t rpm; },
	 * CONFIRMED against the real cros_ec_commands.h -- 4 bytes */
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
	 * already at
	 */
	ecsetfanauto(FanIdx0, 1);
	addarchfile("fan0status", 0444, fan0statusread, nil);
	addarchfile("fan0ctl", 0664, fan0ctlread, fan0ctlwrite);
}
