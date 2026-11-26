/*
 * i2ck3fan.c -- fan control for the K3 CoM260 module and the boards
 * built around it, over the Sensylink CTF2301 (LM63-register-
 * compatible temperature sensor + fan controller) sitting on i2c6
 * at address 0x4c.
 *
 * This is the CoM260 analogue of espik3fan.c: i2ck3.c is the I2C bus
 * transport for this SoC (see port/i2c.h -- I2Cbus/I2Cdev, addi2cbus(),
 * i2creadbyte()/i2cwritebyte()) and knows nothing about fans or the
 * CTF2301 specifically, same as espik3.c knows nothing about fans on
 * the eSPI side. If something else on this board ever needs the
 * CTF2301 -- the remote/local temperature inputs, the alarm bits, the
 * 12-entry temperature/PWM lookup table -- that's a new file calling
 * i2cbus("i2c6")/i2creadbyte()/i2cwritebyte() directly, same as
 * i2ck3faninit() does below, not a reason to touch this one, and
 * definitely not a reason to touch i2ck3.c.
 *
 * The CTF2301 is register-compatible with the LM63 family
 * (LM63/LM64/LM96163), whose register map its original vendor
 * documents publicly; the offsets and bit meanings below are that map.
 *
 * UNCONFIRMED, unlike espik3fan.c: none of this has been run against
 * real CTF2301 hardware yet. espik3fan.c could say "confirmed working
 * end to end" because it was bring-up-tested on a real k3-pico-itx.
 * Nothing here has had that pass yet -- treat every "CONFIRMED" below
 * as "matches the documented register map", not "confirmed on this
 * board's hardware".
 *
 * DESIGN: same #P/fanNstatus + #P/fanNctl pair as espik3fan.c, and the
 * same shared fanparse()/fanfmt() grammar (fan.c). The one difference
 * from espik3fan.c is the ctl unit, driven by what the two chips offer:
 *
 *   - The EC's manual mode takes a target RPM (SET_FAN_TARGET_RPM)
 *     and runs its own closed loop to get there; fan0ctl on that
 *     driver is in "rpm".
 *   - The CTF2301's manual mode takes a raw 0-255 PWM duty
 *     (RegPwmValue below) with no closed loop of its own attached
 *     to that mode -- it either free-runs the duty you give it, or
 *     (auto mode) walks its own 12-point temperature/PWM lookup
 *     table with no host involvement at all. There's no "target RPM"
 *     concept for manual mode on this chip. So fan0ctl here is in "%"
 *     (scaled to/from the chip's raw 0-255 duty), not rpm; fan0status
 *     is still rpm (that part the chip does report directly, via the
 *     tachometer, in both modes). The raw range differs from the K1's
 *     pwm block, so percent -- not a raw count -- is what makes the
 *     two pwm boards mean the same thing.
 *
 *   #P/fan0status  r   measured "<rpm> rpm", converted from the
 *                      tachometer count registers (tachtorpm());
 *                      "0 rpm" for "not spinning or not connected" --
 *                      the chip doesn't distinguish those two the way
 *                      the EC's MEMMAP sentinels did, so neither do we.
 *   #P/fan0ctl     rw  read: "auto" if REG_CONFIG_FAN's manual bit is
 *                      clear (LUT owns the fan), else "<pct> %" from the
 *                      current REG_PWM_VALUE. write: "auto" hands the
 *                      fan back to the onboard lookup table; a number
 *                      (bare, or "<pct> %") takes manual control and
 *                      sets that duty.
 *
 * Unlike fan0ctlread() in espik3fan.c, this one does NOT keep its own
 * isauto/lasttarget shadow state -- there's no known GET-side quirk
 * on this chip forcing that workaround (nothing here has the EC's
 * confirmed SET-works/GET-doesn't split), so fan0ctlread() just reads
 * REG_CONFIG_FAN and REG_PWM_VALUE live off the bus. Simpler, and
 * more honest about what's actually on the chip right now. If real
 * hardware turns up a reason this needs shadow state too, that's the
 * time to add it, not before.
 *
 * BOOT ORDER, CONFIRMED (main.c): links() -- which dispatches
 * i2ck3link(), registering "i2c6" -- runs at main.c:316, well before
 * archinit() ever gets a chance to run. archinit() is archdevtab's
 * ->init(), reached only via chandevinit() (main.c:33, inside
 * init0()); init0() is the body of the first process, created by
 * userinit() at main.c:318 but not actually SCHEDULED until
 * schedinit() at main.c:344 hands off to it. So the real ordering is
 * links() -> chandevreset() -> userinit() -> ... -> schedinit() ->
 * (first process runs) -> init0() -> chandevinit() -> archinit() ->
 * i2ck3faninit(): i2c6 exists long before this file's init ever runs,
 * with an entire scheduler start-up in between. No overlap possible.
 *
 * i2ck3faninit() calls getfandev() once, eagerly, so the boot default
 * applies before anything reads fan0ctl; i2c6 is confirmed to exist by
 * then.  getfandev() still does its own lookup, which keeps this file
 * tolerant of i2ck3.c moving its own bring-up later.
 *
 * The duty register is scaled by REG_PWM_FREQ: twice that register is
 * what means full speed, and the chip keeps only as many bits as its
 * resolution allows, so fan0ctl converts through both rather than
 * assuming 0-255.  The tachometer must be read low byte first, which
 * latches the high byte; the other order yields mismatched pairs.
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "../port/i2c.h"

enum {
	Ctf2301Addr		= 0x4c,		/* CONFIRMED, i2ck3.c cfg[] comment + the DT patch adding this node */

	/* the LM63-family register map */
	RegConfig1		= 0x03,
	RegConvrate		= 0x04,
	RegConfigFan		= 0x4A,
	RegTachCountLsb		= 0x46,
	RegTachCountMsb		= 0x47,
	RegPwmValue		= 0x4C,
	RegPwmFreq		= 0x4D,

	Config1Standby		= 1<<6,		/* 1 = chip in standby, not converting */
	Config1TachEn		= 1<<2,		/* ALERT/TACH pin configured as tach input;
						 * CTF2301 needs this forced on explicitly,
						 * per the "Configuring ALERT/TACH pin for
						 * tachometer input" hunk in the series */

	ConfigFanManual		= 1<<5,		/* 1 = manual, REG_PWM_VALUE writable;
						 * 0 = automatic, chip runs its own LUT */

	PwmCap			= 255,		/* widest duty register value */
	Fandefault		= 50,		/* percent, when the LUT can't be trusted */

	Nlut			= 8,		/* LUT entries */
	RegLutPwm		= 0x51,		/* first pwm entry, then every other byte */

	TachStalled		= 0xFFFC,	/* the count the tach reads when the fan is not turning */
};
/*
 * No FanIdx enum here unlike espik3fan.c: the CTF2301's registers
 * have no fan-index byte anywhere (it's a single implicit fan at
 * fixed addresses), so there's nothing to parametrize yet. The
 * fan0status/fan0ctl naming still reserves room for a second
 * *chip* (a second CTF2301 at a different address, fan1status/
 * fan1ctl) the same way espik3fan.c reserves room for a second
 * *fan index* on one EC -- same DESIGN reasoning, different axis.
 */

static QLock fanlock;	/* protects fandev's one-time init AND the RegConfigFan read-modify-write in fan0ctlwrite() below -- a QLock, not a Lock/ilock: the critical sections here do multi-transaction I2C bus I/O, which is not something to hold with interrupts disabled (ilock's whole point) across */

/*
 * fandev is nil until the first successful getfandev() call, then
 * stays pointing at fandevbuf forever.  i2ck3faninit() calls it
 * eagerly so the boot default applies before the first read.
 */
static I2Cdev fandevbuf;
static I2Cdev *fandev;
static int pwmmax = PwmCap;	/* duty value that means full speed */
static int lutempty;		/* LUT all zeroes: auto would stop the fan */

/*
 * getfandev -- look up the CTF2301 on i2c6 and bring it up, once.
 * Returns nil (leaving fandev untouched) if i2c6 isn't registered
 * yet; every caller below already handles that as a plain Eio, same
 * as any other bus fault.
 *
 * Double-checked locking: the unlocked check up front is the fast
 * path once fandev is set (every call after the first, forever), so
 * fan0statusread()/fan0ctlread() don't pay a qlock on every read.
 * The lock only matters for the narrow window before the first
 * caller finishes bring-up -- without it, two callers racing in
 * before fandev is set would both run memset()+addi2cdev() on the
 * same static fandevbuf and both do their own bring-up I2C traffic
 * concurrently. Re-checking fandev again after taking the lock is
 * what makes the second (losing) racer just take the already-
 * published result instead of redoing any of this.
 */
static I2Cdev*
getfandev(void)
{
	I2Cbus *bus;
	int cfg, freq, i, v;

	if(fandev != nil)
		return fandev;
	qlock(&fanlock);
	if(fandev != nil){
		qunlock(&fanlock);
		return fandev;
	}
	bus = i2cbus("i2c6");
	if(bus == nil){
		qunlock(&fanlock);
		return nil;
	}
	memset(&fandevbuf, 0, sizeof fandevbuf);
	fandevbuf.bus = bus;
	fandevbuf.addr = Ctf2301Addr;
	fandevbuf.subaddr = 1;	/* the chip's registers are one address byte */
	addi2cdev(&fandevbuf);

	/*
	 * Bring-up, best-effort: don't trust the chip's power-on state,
	 * put it somewhere known instead.  i2ck3faninit() calls this at
	 * boot so the fan is already at its default before the first
	 * fan0ctl read.
	 *
	 * Config1TachEn is forced on unconditionally here. That is only
	 * right where the pin is wired to a fan tachometer: on a board
	 * using ALERT/TACH as an interrupt line to the host instead,
	 * forcing this bit would be wrong. This driver has no "does this
	 * device have an irq" concept to check, so it always forces it. Justified here
	 * by the schematic net name (FAN_TACH, wired to the physical fan
	 * connector, not to any host alert/interrupt net) but that's a
	 * board-level inference, not something read back from the chip
	 * or the DT -- flagging it rather than presenting it as settled.
	 */
	cfg = i2creadbyte(&fandevbuf, RegConfig1);
	if(cfg >= 0 && (cfg & Config1Standby))
		i2cwritebyte(&fandevbuf, RegConfig1, cfg & ~Config1Standby);
	cfg = i2creadbyte(&fandevbuf, RegConfig1);
	if(cfg >= 0 && (cfg & Config1TachEn) == 0)
		i2cwritebyte(&fandevbuf, RegConfig1, cfg | Config1TachEn);
	/*
	 * Take manual control before probing the duty register: while
	 * the chip is in auto that register is read-only.
	 */
	cfg = i2creadbyte(&fandevbuf, RegConfigFan);
	if(cfg < 0)
		cfg = ConfigFanManual;
	if((cfg & ConfigFanManual) == 0){
		cfg |= ConfigFanManual;
		i2cwritebyte(&fandevbuf, RegConfigFan, cfg);
	}

	/*
	 * Full duty is twice REG_PWM_FREQ, and the register keeps only
	 * the low bits of what it is given; probe both so percent means
	 * the same thing going in and coming back out.
	 */
	freq = i2creadbyte(&fandevbuf, RegPwmFreq);
	if(freq <= 0)
		freq = 1;
	pwmmax = 2*freq;
	if(pwmmax > PwmCap)
		pwmmax = PwmCap;
	i2cwritebyte(&fandevbuf, RegPwmValue, 0xff);
	v = i2creadbyte(&fandevbuf, RegPwmValue);
	if(v > 0 && v < pwmmax)
		pwmmax = v;
	if(pwmmax < 2)
		pwmmax = 2;

	/*
	 * An all-zero lookup table makes "auto" stop the fan, which is
	 * worse than a fixed duty.  When the table is empty stay in
	 * manual at a default that a governor or fan0ctl can change.
	 */
	lutempty = 1;
	for(i = 0; i < Nlut; i++)
		if(i2creadbyte(&fandevbuf, RegLutPwm + 2*i) > 0){
			lutempty = 0;
			break;
		}
	if(lutempty)
		i2cwritebyte(&fandevbuf, RegPwmValue,
			(Fandefault*pwmmax + 50)/100);
	else
		i2cwritebyte(&fandevbuf, RegConfigFan,
			cfg & ~ConfigFanManual);

	fandev = &fandevbuf;
	qunlock(&fanlock);
	return fandev;
}

/* the tach counts a fixed-frequency clock over two fan poles */
static int
tachtorpm(int reg)
{
	if(reg == TachStalled || reg == 0)
		return 0;
	return 5400000 / reg;
}

static long
fan0statusread(Chan*, void *a, long n, vlong off)
{
	char str[32];
	I2Cdev *dev;
	int lsb, msb, reg;

	dev = getfandev();
	if(dev == nil)
		error(Eio);
	/*
	 * Read the low byte first: that latches the high byte, so the
	 * pair comes from one sample.  The low two bits are not part of
	 * the count.
	 */
	lsb = i2creadbyte(dev, RegTachCountLsb);
	msb = i2creadbyte(dev, RegTachCountMsb);
	if(lsb < 0 || msb < 0)
		error(Eio);
	reg = msb<<8 | (lsb & 0xFC);
	fanfmt(str, sizeof str, "rpm", tachtorpm(reg), 0);
	return readstr(off, a, n, str);
}

static long
fan0ctlread(Chan*, void *a, long n, vlong off)
{
	char str[32];
	I2Cdev *dev;
	int cfg, pct, pwm;

	dev = getfandev();
	if(dev == nil)
		error(Eio);
	cfg = i2creadbyte(dev, RegConfigFan);
	if(cfg < 0)
		error(Eio);
	if((cfg & ConfigFanManual) == 0)
		fanfmt(str, sizeof str, "%", 0, 1);
	else{
		pwm = i2creadbyte(dev, RegPwmValue);
		if(pwm < 0)
			error(Eio);
		pct = pwm >= pwmmax ? 100 :
			(pwm*100 + pwmmax/2)/pwmmax;
		fanfmt(str, sizeof str, "%", pct, 0);
	}
	return readstr(off, a, n, str);
}

static long
fan0ctlwrite(Chan*, void *a, long n, vlong)
{
	Cmdbuf *cb;
	I2Cdev *dev;
	int cfg, pwm, au;
	long pct;

	dev = getfandev();
	if(dev == nil)
		error(Eio);

	cb = parsecmd(a, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	/* "auto" is the chip's own LUT; a number is a percent */
	pct = fanparse(cb, "%", !lutempty, &au);

	qlock(&fanlock);
	if(waserror()){
		qunlock(&fanlock);
		nexterror();
	}

	cfg = i2creadbyte(dev, RegConfigFan);
	if(cfg < 0)
		error(Eio);

	if(au){
		cfg &= ~ConfigFanManual;
		if(i2cwritebyte(dev, RegConfigFan, cfg) < 0)
			error(Eio);
	}else{
		if(pct < 0)
			pct = 0;
		if(pct > 100)
			pct = 100;
		pwm = (pct*pwmmax + 50)/100;
		if(pwm > pwmmax)
			pwm = pwmmax;
		cfg |= ConfigFanManual;
		if(i2cwritebyte(dev, RegConfigFan, cfg) < 0)
			error(Eio);
		if(i2cwritebyte(dev, RegPwmValue, pwm) < 0)
			error(Eio);
	}

	poperror();
	qunlock(&fanlock);
	poperror();
	free(cb);
	return n;
}

void
i2ck3faninit(void)
{
	/*
	 * Bring the fan up now so the boot default applies before
	 * anything reads fan0ctl.  i2c6 is registered here already.
	 */
	getfandev();
	addarchfile("fan0status", 0444, fan0statusread, nil);
	addarchfile("fan0ctl", 0664, fan0ctlread, fan0ctlwrite);
}
