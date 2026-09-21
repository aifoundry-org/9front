/*
 * i2ck3fan.c -- fan control for k3-com260-ifx, over the Sensylink
 * CTF2301 (LM63-register-compatible temperature sensor + fan
 * controller) sitting on i2c6 at address 0x4c.
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
 * getfandev() below is still written lazily rather than doing the
 * i2cbus("i2c6") lookup once in i2ck3faninit() -- now confirmed
 * unnecessary for correctness on this boot path, but cheap to keep:
 * it's also what makes this file tolerant of i2ck3.c someday moving
 * its own bring-up later (a bus reset added to i2ck3link(), a retry
 * loop, whatever), without this file needing another look.
 *
 * TODO, flagged rather than silently assumed: manual-mode duty is
 * written to REG_PWM_VALUE as a raw 0-255 value on the assumption the
 * chip is in (or defaults to) 8-bit high-resolution PWM mode. In the
 * chip's low-resolution mode, REG_PWM_VALUE is instead a duty scaled
 * by REG_PWM_FREQ, and a raw 0-255 write here would not mean what the
 * comment above says it means. This driver does not read REG_PWM_FREQ or
 * touch REG_CONFIG_ENHANCED to force high-res mode -- confirm the
 * chip's power-on default on real hardware before trusting fan0ctl's
 * numbers.
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

	PwmMax			= 255,		/* full duty */

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
 * stays pointing at fandevbuf forever -- see the BOOT ORDER comment
 * above for why this is looked up lazily instead of once, eagerly,
 * in i2ck3faninit().
 */
static I2Cdev fandevbuf;
static I2Cdev *fandev;

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
	int cfg;

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
	addi2cdev(&fandevbuf);

	/*
	 * Bring-up, best-effort, same spirit as espik3fan.c's boot-time
	 * ecsetfanauto(FanIdx0, 1): don't trust the chip's power-on
	 * state, put it somewhere known instead. Runs once, here, on
	 * first access rather than in i2ck3faninit() -- not because it
	 * has to (see BOOT ORDER: i2c6 is confirmed to exist by the time
	 * i2ck3faninit() runs), just because it's free insurance against
	 * that ordering ever changing.
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
	cfg = i2creadbyte(&fandevbuf, RegConfigFan);
	if(cfg >= 0 && (cfg & ConfigFanManual) != 0)
		i2cwritebyte(&fandevbuf, RegConfigFan, cfg & ~ConfigFanManual);

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
	 * MSB read before LSB, not the other way around: UNCONFIRMED
	 * whether this chip latches the LSB on the MSB read the way many
	 * dual-byte sensor registers do (nothing in either source cited
	 * at the top of this file documents it either way), but MSB-
	 * first is the overwhelmingly common convention for chips that
	 * do latch, and costs nothing if this one doesn't -- pure
	 * downside-free defensive ordering, not a confirmed fix.
	 */
	msb = i2creadbyte(dev, RegTachCountMsb);
	lsb = i2creadbyte(dev, RegTachCountLsb);
	if(lsb < 0 || msb < 0)
		error(Eio);
	reg = msb<<8 | lsb;
	fanfmt(str, sizeof str, "rpm", tachtorpm(reg), 0);
	return readstr(off, a, n, str);
}

static long
fan0ctlread(Chan*, void *a, long n, vlong off)
{
	char str[32];
	I2Cdev *dev;
	int cfg, pwm;

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
		/* raw 0-PwmMax duty back to a rounded percent */
		fanfmt(str, sizeof str, "%", (pwm*100 + PwmMax/2)/PwmMax, 0);
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
	/* "auto" is the chip's own 12-point LUT; a number is a percent */
	pct = fanparse(cb, "%", 1, &au);

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
		/* percent to the chip's raw 0-PwmMax duty, rounded */
		pwm = (pct*PwmMax + 50)/100;
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
	if(!dtbsocmatch("spacemit,k3-com260-ifx"))
		return;

	/*
	 * Deliberately does NOT call getfandev() here. Not required --
	 * see BOOT ORDER: i2c6 is confirmed to exist by this point -- but
	 * keeping file registration unconditional on just the board match
	 * costs nothing and stays correct even if that ordering changes.
	 */
	addarchfile("fan0status", 0444, fan0statusread, nil);
	addarchfile("fan0ctl", 0664, fan0ctlread, fan0ctlwrite);
}
