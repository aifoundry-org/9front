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
 * FIELD NOTE (first real hardware test, k3-pico-itx): reading #P/fan
 * and #P/fantarget worked (both 0, plausible: fan idle, no target
 * set), and `echo 2651 > #P/fan` returned success (no Eio -- the EC
 * accepted the SET_FAN_TARGET_RPM v0 request, checksum and all: the
 * whole eSPI+cros_ec transport stack in espik3.c is confirmed working
 * end to end on real hardware). But #P/fantarget read back 0 again
 * right after. That's not a transport bug -- it's the expected
 * ChromeOS EC behaviour when the fan is still under AUTOMATIC
 * (thermal-curve-driven) control: the EC's own thermal engine
 * recomputes and overwrites the fan target on its own periodic tick,
 * so a manual SET_FAN_TARGET_RPM sticks for at most one tick unless
 * automatic control is turned off first. EC_CMD_THERMAL_AUTO_FAN_CTRL
 * (0x0052, CONFIRMED against cros_ec_commands.h) is that switch; see
 * ecsetfanauto() below. fanwrite() now takes manual control before
 * setting a target, and writing the literal string "auto" hands
 * control back to the EC's thermal curve.
 *
 * Publishes three #P files, each single-purpose (matching this
 * tree's own #P/cputype, #P/cpuid, #P/pmctl convention rather than
 * folding everything into one file with modes):
 *
 *   #P/fan        rw   measured RPM (EC_MEMMAP_FAN, plain memmap
 *                      read, no command/busy-wait) on read; on
 *                      write: a decimal number takes manual control
 *                      and sets that target RPM
 *                      (EC_CMD_PWM_SET_FAN_TARGET_RPM); the literal
 *                      string "auto" hands control back to the EC's
 *                      own thermal curve (EC_CMD_THERMAL_AUTO_FAN_CTRL).
 *   #P/fantarget  r    the RPM the EC's own control loop is currently
 *                      steering toward (EC_CMD_PWM_GET_FAN_TARGET_RPM).
 *   #P/fanauto    r    "1" if the fan is under the EC's automatic
 *                      thermal control, "0" if under manual control
 *                      from the last #P/fan write
 *                      (EC_CMD_THERMAL_AUTO_FAN_CTRL, GET).
 *
 * fan and fantarget can legitimately disagree for a bit right after a
 * manual #P/fan write -- spin-up/spin-down isn't instant, fantarget
 * reflects the EC's setpoint immediately but fan won't catch up until
 * the fan physically gets there.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"

enum {
	MemmapFan	= 0x10,			/* EC_MEMMAP_FAN */
	EcCmdPwmGetFanTargetRpm = 0x0020,	/* CONFIRMED, cros_ec_commands.h */
	EcCmdPwmSetFanTargetRpm = 0x0021,	/* CONFIRMED, cros_ec_commands.h */
	EcCmdThermalAutoFanCtrl	= 0x0052,	/* CONFIRMED, cros_ec_commands.h */
	EcAutoFanCtrlCmdSet	= 0,
	EcAutoFanCtrlCmdGet	= 1,	/* unused: confirmed broken on this EC, see fanautoread() */
	FanIdx0			= 0,		/* this board: one fan, index 0 */
};

static long
fanread(Chan*, void *a, long n, vlong off)
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
 * #P/fantarget -- read-only.
 *
 * FIELD NOTE (second real-hardware finding): on this board's EC
 * firmware, EC_CMD_PWM_GET_FAN_TARGET_RPM round-trips cleanly (no
 * transport/checksum error, real EC_RES_SUCCESS) but always answers
 * rpm=0, regardless of what SET_FAN_TARGET_RPM was actually asked
 * for -- confirmed NOT a transport bug, since #P/fan's physically
 * measured RPM does track what gets written to #P/fan. The simplest
 * explanation is that this particular EC image's GET handler for
 * this command is a stub that was never wired to the same state
 * SET writes to (SET has proper versioned request structs in
 * cros_ec_commands.h; GET has none at all -- it reads as the less-
 * used, more likely to bitrot of the pair on any given EC build).
 *
 * Given that, this file no longer trusts the live GET as the primary
 * source: it prefers the driver's own record of the last RPM it
 * successfully asked the EC for via #P/fan (lasttarget below), which
 * is empirically the trustworthy number on this hardware, and only
 * falls back to the live GET when there's no cached value yet (e.g.
 * right after boot, before any #P/fan write) or if a future EC
 * firmware update makes GET start answering for real -- a nonzero
 * live answer always wins, since a genuine live value beats a stale
 * cache.
 */
static ulong lasttarget;
static int haslasttarget;

static long
fantargetread(Chan*, void *a, long n, vlong off)
{
	char str[32];
	uchar b[4];
	ulong rpm;

	rpm = 0;
	if(k3eccmd(EcCmdPwmGetFanTargetRpm, 0, nil, 0, b, sizeof b) >= 4)
		rpm = b[0] | b[1]<<8 | (ulong)b[2]<<16 | (ulong)b[3]<<24;
	if(rpm == 0 && haslasttarget)
		rpm = lasttarget;
	snprint(str, sizeof str, "%lud\n", rpm);
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
 * fatal to the caller. Updates isauto on success so #P/fanauto (see
 * below) has something trustworthy to report.
 */
static int isauto;

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

/*
 * #P/fanauto -- read-only, "1"/"0" for whether the fan is currently
 * under the EC's own automatic thermal control or under manual
 * control from the last #P/fan write.
 *
 * FIELD NOTE (third real-hardware finding): EC_CMD_THERMAL_AUTO_FAN_CTRL
 * with cmd=EC_AUTO_FAN_CONTROL_CMD_GET fails outright on this board's
 * EC (real i/o error, not just a wrong answer) -- the same SET-works/
 * GET-doesn't asymmetry already confirmed for PWM_{GET,SET}_FAN_
 * TARGET_RPM. So, same fix as #P/fantarget: don't call the live GET
 * at all, just report isauto, the driver's own record of the last
 * value it successfully asked ecsetfanauto() for (set at boot in
 * k3ecfaninit(), and on every #P/fan write/"auto").
 */
static long
fanautoread(Chan*, void *a, long n, vlong off)
{
	char str[4];

	snprint(str, sizeof str, "%d\n", isauto);
	return readstr(off, a, n, str);
}

static long
fanwrite(Chan*, void *a, long n, vlong)
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
		haslasttarget = 0;	/* EC owns the target now, drop our cache */
		poperror();
		free(cb);
		return n;
	}

	/*
	 * Take manual control first -- see the FIELD NOTE at the top of
	 * this file for why: on real hardware, a target written while
	 * the EC's thermal engine still owns the fan got silently
	 * overwritten back to 0 within one control tick. Best-effort:
	 * proceed to SET_FAN_TARGET_RPM regardless of whether this
	 * particular EC firmware implements the auto-control switch.
	 */
	ecsetfanauto(FanIdx0, 0);

	rpm = strtoul(cb->f[0], nil, 10);
	/* struct ec_params_pwm_set_fan_target_rpm_v0 { uint32_t rpm; },
	 * CONFIRMED against the real cros_ec_commands.h -- this is 4
	 * bytes, NOT 2. An earlier revision of this file sent uint16
	 * and would have sent the EC a malformed/truncated request. */
	req[0] = rpm & 0xFF;
	req[1] = (rpm>>8) & 0xFF;
	req[2] = (rpm>>16) & 0xFF;
	req[3] = (rpm>>24) & 0xFF;
	if(k3eccmd(EcCmdPwmSetFanTargetRpm, 0, req, sizeof req, nil, 0) < 0)
		error(Eio);
	lasttarget = rpm;
	haslasttarget = 1;
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
	 * Boot state: explicitly put the fan under automatic (EC
	 * thermal-curve) control rather than trusting whatever the EC
	 * happened to power up already at -- deterministic starting
	 * point instead of an assumed default. Best-effort, same as
	 * every other ecsetfanauto() call: if this EC firmware doesn't
	 * implement EC_CMD_THERMAL_AUTO_FAN_CTRL, this is a silent
	 * no-op and whatever the EC powered up in stands as-is.
	 */
	ecsetfanauto(FanIdx0, 1);
	addarchfile("fan", 0664, fanread, fanwrite);
	addarchfile("fantarget", 0444, fantargetread, nil);
	addarchfile("fanauto", 0444, fanautoread, nil);
}
