#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"
#include "../port/error.h"
#include "io.h"

/*
 * devgpiok3.c -- named-pin #G/gpio Chan device for SpacemiT K3
 * boards, covering every board this tree has a pin table for
 * (currently: pico-itx, schematic SCH-146-V10_K3_DEB1_P1_LP5315B;
 * K3-CoM260/BPI-SM10, schematic k3-com260_kit_v02_20260306.pdf).
 *
 * THIS FILE REPLACES devgpiopicoitx.c AND devgpiocom260.c
 * -----------------------------------------------------------
 * Those two were separate, near-identical files, each defining its
 * own "Dev ...devtab = {'G', "gpio", ...}". That's fine for
 * usbxhcibpif3.c/usbxhcipicoitx.c/usbxhcicom260.c, which all build
 * into the same rv64 image and self-select at runtime via
 * dtbsocmatch() -- but that trick works there because they register
 * into a SHARED "xhci" hci-type table (addhcitype()), where several
 * entries can coexist. A Dev can't do that: dc='G' is claimed by
 * exactly one Dev struct per kernel image, so the old two-file setup
 * could only ever be built one board at a time (whichever one was
 * listed in rv64's "dev" section), never both together. This file
 * merges them into ONE Dev, "gpiok3devtab", with per-board pin
 * tables selected inside gpioinit() by trying each board's
 * dtbsocmatch() string in turn -- see BOARD SELECTION below. Add
 * "gpiok3" (not "gpiopicoitx" or "gpiocom260") to rv64's "dev"
 * section for this file.
 *
 * Modelled directly on 9front's bcm devgpio.c: same qid-packing
 * idiom, same #G/gpio/{ctl,event,<name>} namespace shape, same
 * Cmdtab-driven ctl file, same single-reader "event" file -- but
 * driving gpiok3.c's real gpioout()/gpioin()/gpiointrenable() API
 * instead of bcm's own raw register pokes (gpiok3.c IS the register-
 * level driver here, already mapped/clocked from gpiok3init(),
 * called once from devarch.c's archinit() -- this file never touches
 * MMIO itself).
 *
 * BOARD SELECTION
 * -----------------
 * This is functionally the same problem bcm devgpio.c's "scheme"
 * (bcm/board/wpi) solves -- several different name sets for the same
 * underlying pin space -- except there the person picks the scheme
 * by hand with a ctl write, because all three names are valid at
 * once on the same running board. Here the "scheme" is really "which
 * board is this", which dtbsocmatch() can already answer, so
 * gpioinit() picks automatically instead of exposing a CMscheme
 * command: on a real board exactly one of "spacemit,k3-pico-itx" /
 * "spacemit,k3-com260-ifx" will match, and only that board's names
 * get populated. If neither matches (bpif3, qemu, or a board this
 * file has no table for yet), the directory still exists but only
 * shows ctl/event, same policy the old two files used for "wrong
 * board" -- see gpioinit() below.
 *
 * DELIBERATE DIFFERENCES FROM THE BCM VERSION, AND WHY
 * ------------------------------------------------------
 *   - PinBits is 7 and the pin table has 128 slots, not bcm's 32:
 *     K3 has 4 banks of 32 GPIOs (gpiok3.c's Nbank/Npin), not one
 *     bank of 32.
 *
 *   - No "function"/"pull" ctl commands. gpiok3.c's own header says
 *     plainly that pin-mux and pull-up/down live in a SEPARATE
 *     "spacemit,k3-pinctrl" block this driver doesn't touch: there
 *     is no gpiosel()/gpiopull() to call, unlike bcm where the GPIO
 *     block itself owns FSEL/PUD. CMfunc/CMpull are simply not in
 *     gpiocmd[], so lookupcmd() rejects those writes on its own --
 *     no dead code pretending to support them.
 *
 *   - gpioinit() gates each table on its own dtbsocmatch() string,
 *     same policy usbxhcipicoitx.c/usbxhcicom260.c's reset() and
 *     gpiok3init()/k1thermalinit()/k3thermalinit() use: on a
 *     non-matching board that board's pin-name table is left empty
 *     rather than being populated with names that don't apply there.
 *     It still calls no gpiomeminit()-equivalent and reads no
 *     getboardrev(): gpiok3.c already maps its own MMIO and gates
 *     its own clock from gpiok3init(), called separately from
 *     devarch.c's archinit(); this file's gpioinit() only ever
 *     builds (or skips) the pin table, it never touches hardware
 *     itself.
 *
 *   - "event" is driven by gpiok3.c's real per-pin edge interrupt
 *     (gpiointrenable()/gpiointrdisable(), dispatched per-pin inside
 *     gpiok3.c off its one shared aplic IRQ) instead of a raw ISR
 *     that scans every pin's status bit the way bcm's interrupt()/
 *     gpiogetevent() do. eventvalue is correspondingly Nbankwords*32
 *     = 128 bits, read back as 16 bytes instead of bcm's 4.
 *
 *   - Each pin-name table is built at gpioinit() time from a
 *     {pin,name} list instead of a positional array with holes
 *     written out ("0, 0, "2", "3", ..."): this compiler doesn't
 *     have C99 designated initializers, and hand-aligning 128
 *     positional slots by hand invites off-by-one bugs that a
 *     {pin,name} pair doesn't.
 *
 *   - getpin() uses strcmp(), not bcm's strncmp(table[i], name,
 *     strlen(table[i])) prefix match: every board's names here are
 *     already unambiguous, so an exact match is strictly safer and
 *     no less convenient.
 *
 * Everything else -- the #G/gpio/{ctl,event,<name>} shape, PATH()/
 * PIN_NUMBER()/FILE_TYPE() qid packing, the Cmdtab ctl parser, the
 * exclusive "event" reader -- is carried over as-is from devgpio.c.
 *
 * PIN TABLE SOURCE AND SCOPE -- PICO-ITX
 * -----------------------------------------
 * Every {pin,name} entry in picoitxpins[] below comes from the
 * schematic's "GPIO ASSIGNMENT" sheet (page 10) and the
 * SOC-GPIO-1..5 sheets (pages 12-16), which agree with each other on
 * every entry used here. The "GPIO[x]_NN" label's NN suffix *is* the
 * flat 0..127 pin number gpiok3.c/fns.h's GPIO_PIN() expects
 * (bank=NN/32); the "[x]" prefix is just the vendor ballout table's
 * own pad-group label, unrelated to the register bank.
 *
 * Included: signals wired as a plain control/status/wake/interrupt
 * line (module enable/reset, CC/HPD/WAKE interrupts, PCIe PERST#/
 * CLKREQ#/WAKE#, eDP power/backlight enable, EC handshake lines).
 *
 * Excluded, on purpose:
 *   - Any net with an "R-" prefix (R-GMAC3_*, R-CAN0_*, R-I2C1_*,
 *     ...): those are R.GPIO[n] pins, a SEPARATE RCPU-owned GPIO
 *     controller gpiok3.c never touches (see its own header on
 *     APMASK_REG vs CPMASK_REG). Out of scope for this file too.
 *   - Dedicated-function pins with no plain-GPIO use on this board:
 *     RGMII (GMAC0/2/3), LPDDR5, eSPI0 data/clk/cs lines, MIPI/eDP/
 *     UFS/QSPI lanes, and JTAG/boot-strap pins (Strap0-5, PRI_T*),
 *     which are sampled once at reset, not runtime GPIO.
 *   - CPU_EC_SWDIO/CPU_EC_SWCLK (pins 82/83) are the one exception:
 *     they carry the EC's SWD debug port, not a general-purpose
 *     signal, but the schematic does route them through plain
 *     GPIO[4]_82/83 pads, so they're listed for anyone who wants to
 *     bit-bang SWD before EC firmware exists.
 *
 * PIN TABLE SOURCE AND SCOPE -- K3-CoM260 -- MUCH THINNER, AND WHY
 * --------------------------------------------------------------
 * Unlike the pico-itx schematic, the CoM260 PDF's own "GPIO
 * ASSIGNMENT" sheet (page 8) carries no extractable pin-table text --
 * title only, no data (presumably an image/graphic table that didn't
 * OCR). Without that sheet, there is no page that pairs a net name
 * directly with its GPIO[x]_NN the way pico-itx's page 10 did.
 *
 * What IS available is the SODIMM edge connector's own pinout (pages
 * 9-10, connectors J2A/J2B), which lists every
 * "GPIO[x]_NN { alt-func-1 / alt-func-2 / ... } <connector-pin>"
 * entry -- the K3 ballout's own alternate-function table. Each entry
 * in com260pins[] below was accepted ONLY if:
 *
 *   1. its net name appears as a literal alternate-function string
 *      inside exactly ONE GPIO[x]_NN entry across all of J2A/J2B --
 *      if the same name shows up as an alt-func of two or more
 *      different pins (this happens a lot: I2C0_SCL, PCIeA_PERSTn,
 *      PCIeA_WAKEn, PCIeD_CLKREQn, PCIeD_PERSTn, UART6_TXD/RXD, ...
 *      all appear more than once), there's no way from this text
 *      alone to know which physical pin the board actually muxed to
 *      that function, so it's left out rather than guessed at;
 *
 *   2. it isn't a generic multiplexed bus (I2C0/I2C1/I2C3/I2C4/I2C5,
 *      UART1/UART6, SPI0/R-SPI0, MMC2, I2S0, CAN) -- those are
 *      carrier-passthrough buses on the 40-pin GPIO Expansion Header
 *      and SODIMM connector, properly the domain of devuart.c/i2c/
 *      mmc drivers, not plain GPIO, same exclusion rule applied to
 *      pico-itx's eSPI/RGMII/etc above;
 *
 *   3. it isn't one of the BMCU_* / PMIC_RST_OUTn/SHUTDOWN_REQ/
 *      POWER_EN/SLEEP-WAKE/MODULE_ID family -- those go to the
 *      on-module power-sequencing MCU (EFM8SB10F2G, U1402, see the
 *      "MCU DEBUG" sheet), not to a K3 GPIO[x]_NN pin at all, so
 *      gpiok3.c has nothing to drive there regardless.
 *
 * That leaves a short list: two PCIe PERST# lines and three PCIe
 * CLKREQ# lines, each confirmed unique. This table is intentionally
 * conservative -- it's very likely the real "GPIO ASSIGNMENT" sheet
 * has more, but guessing at ambiguous matches would be worse than
 * listing fewer, correct ones.
 */


enum {
	PinBits		= 7,		/* 0..127: K3 has 4 banks of 32 GPIOs */
	PinMask		= (1<<PinBits)-1,
	Ngpiopin	= 128,		/* == gpiok3.c's Npin */
	Nbankwords	= 4,		/* == gpiok3.c's Nbank; eventvalue size in u32ints */
};

#define PIN_OFFSET	(PARENT_OFFSET + PARENT_BITS)
#define PIN_NUMBER(q)	(((q).path >> PIN_OFFSET) & PinMask)

#define PARENT_OFFSET	(FILE_OFFSET + FILE_BITS)
#define PARENT_BITS	3
#define PARENT_MASK	((1 << PARENT_BITS) - 1)
#define PARENT_TYPE(q)	(((q).path >> PARENT_OFFSET) & PARENT_MASK)

#define FILE_OFFSET	0
#define FILE_BITS	3
#define FILE_MASK	((1 << FILE_BITS) - 1)
#define FILE_TYPE(q)	(((q).path >> FILE_OFFSET) & FILE_MASK)

/* pin is valid only when file==Qdata, otherwise 0 */
#define PATH(pin, parent, file) \
	(((uvlong)(pin) & PinMask) << PIN_OFFSET) \
	| (((uvlong)(parent) & PARENT_MASK) << PARENT_OFFSET) \
	| (((uvlong)(file) & FILE_MASK) << FILE_OFFSET)

#define SET_BIT(f, pin, value) \
	((f)[(pin)/32] = ((f)[(pin)/32] & ~(1UL<<((pin)%32))) | ((ulong)(value)<<((pin)%32)))
#define GET_BIT(f, pin) \
	(((f)[(pin)/32] >> ((pin)%32)) & 1)

enum {
	/* parent types */
	Qtopdir = 0,
	Qgpiodir,
	/* file types */
	Qdir,
	Qdata,
	Qctl,
	Qevent,
};

/* commands: CMzero/CMone are written straight to a pin's Qdata file,
 * CMevent goes to Qctl -- exactly like bcm's shared gpiocmd[]. */
enum {
	CMzero,
	CMone,
	CMevent,
};

static Cmdtab gpiocmd[] = {
	CMzero,	"0",		1,
	CMone,	"1",		1,
	CMevent,"event",	4,	/* event <edge> <enable|disable> <pinname> */
};

/* dat.h's GpioRising/GpioFalling/GpioEdge, in the order evtypename[] names them */
static char *evtypename[] = {
	"edge-rising", "edge-falling", "edge-both",
};
static int evmode[] = {
	GpioRising, GpioFalling, GpioEdge,
};

Dirtab topdir = { "#G", {PATH(0, Qtopdir, Qdir), 0, QTDIR}, 0, 0555 };
Dirtab gpiodir = { "gpio", {PATH(0, Qgpiodir, Qdir), 0, QTDIR}, 0, 0555 };

static Dirtab typedir[] = {
	"ctl",		{ PATH(0, Qgpiodir, Qctl), 0, QTFILE }, 0, 0666,
	"event",	{ PATH(0, Qgpiodir, Qevent), 0, QTFILE }, 0, 0444,
};

typedef struct Pinname Pinname;
struct Pinname
{
	int	pin;	/* fns.h GPIO_PIN() value, 0..127 */
	char	*name;	/* schematic net name == file name */
};

/*
 * pin                       name                     schematic sheet(s)
 */
static Pinname picoitxpins[] = {
{18,	"MODULE_TURN_ON_1V8"},		/* 10,12,37: WWAN module power-on, out */
{19,	"WWAN_RESET#_1V8"},		/* 10,12,37: WWAN module reset, out, active-low */
{23,	"SOC_eDP0_HPD_3V3"},		/* 10,19,46: eDP0 hot-plug detect, in */
{24,	"DP1_HPD_3V3"},			/* 10,19: DP1 hot-plug detect, in */
{25,	"PCIE2_PERSTN_3V3_CN"},		/* 10,37: PCIe2 (M.2 B-key/LTE) PERST#, out */
{26,	"BT_WAKE_HOST_3V3"},		/* 10,47: Bluetooth wake host, in */
{27,	"PCIE2_CLKREQN_33_CN"},		/* 10,37: PCIe2 CLKREQ#, in */
{30,	"BT_EN_3V3"},			/* 10,47: Bluetooth enable, out */
{35,	"WLAN_EN_3V3"},			/* 10,47: WLAN enable, out */
{41,	"HOST_WAKE_BT_3V3"},		/* 10,47: host wake BT, out */
{42,	"EC_CPU_ALLPG"},		/* 10,43: EC all-power-good, in */
{43,	"EC_CPU_INT_N"},		/* 10,43: EC interrupt, active low, in */
{76,	"PCIE4_PERSTN"},		/* 10,47: PCIe4 (WiFi) PERST#, out */
{77,	"PCIE4_WAKEN"},			/* 10,47: PCIe4 WAKE#, in */
{78,	"PCIE4_CLKREQN"},		/* 10,47: PCIe4 CLKREQ#, in */
{79,	"PCIE0_PERSTN"},		/* 10,38: PCIe0 (M.2 M-key SSD) PERST#, out */
{80,	"SOC_EC_RST"},			/* 10,43: reset to EC, out */
{81,	"PCIE0_CLKREQN"},		/* 10,38: PCIe0 CLKREQ#, in */
{82,	"CPU_EC_SWDIO"},		/* 10,43: EC SWD data -- see header note */
{83,	"CPU_EC_SWCLK"},		/* 10,43: EC SWD clock -- see header note */
{84,	"CPU_W_DISABLE#1"},		/* 10,37: M.2 W_DISABLE# (radio kill), out */
{85,	"CC_INT_N"},			/* 10,34: USB-C CC ctlr (FUSB301) interrupt, in */
{86,	"USB30_DRD_DIR"},		/* 10,34: USB3 DRD direction select, out */
{88,	"PCIE1_PERSTN"},		/* 10,37: PCIe1 (M.2 A/B x2) PERST#, out */
{89,	"MUX_CPU_M2B2"},		/* 10,36: PCIe1 2:4 mux select (B-key path), out */
{90,	"PCIE1_CLKREQN"},		/* 10,37: PCIe1 CLKREQ#, in */
{101,	"SOC_eDP0_PWR-EN_3V3"},		/* 10,46: eDP panel power enable, out */
{106,	"SOC_eDP0_BL-PWM_3V3"},		/* 10,46: eDP backlight PWM/enable, out */
{118,	"SOC_eDP0_BL-EN_3V3"},		/* 10,46: eDP backlight enable, out */
};

/*
 * pin                   name                    schematic sheet(s), notes
 *
 * Every one of these appears as an alternate-function name inside
 * exactly one GPIO[x]_NN entry on connectors J2A/J2B (pages 9-10) --
 * see the PIN TABLE SOURCE AND SCOPE -- K3-CoM260 header comment for
 * why so few entries met that bar.
 */
static Pinname com260pins[] = {
{44,	"PCIeA_CLKREQn"},	/* 9,22: GPIO[3]_44, M.2 Key-M[A] NVMe CLKREQ#, in */
{58,	"PCIeC_PERSTn"},	/* 9,23: GPIO[3]_58, M.2 Key-M[B] NVMe PERST#, out */
{60,	"PCIeC_CLKREQn"},	/* 9,21,23: GPIO[3]_60, M.2 Key-M[B]/Key-E CLKREQ#, in */
{76,	"PCIeE_PERSTn"},	/* 10,21: GPIO[4]_76, M.2 Key-E (WiFi/BT) PERST#, out */
{78,	"PCIeE_CLKREQn"},	/* 10,21: GPIO[4]_78, M.2 Key-E (WiFi/BT) CLKREQ#, in */
};

static char *gpiotable[Ngpiopin];	/* filled in by gpioinit() */

static Rendez rend;
static u32int eventvalue[Nbankwords];
static long eventinuse;
static Lock eventlock;

static void
mkdeventry(Chan *c, Qid qid, Dirtab *tab, Dir *db)
{
	mkqid(&qid, tab->qid.path, tab->qid.vers, tab->qid.type);
	devdir(c, qid, tab->name, tab->length, eve, tab->perm, db);
}

static int
gpiogen(Chan *c, char*, Dirtab*, int, int s, Dir *db)
{
	Qid qid;
	int parent, l;

	qid.vers = 0;
	parent = PARENT_TYPE(c->qid);

	if(s == DEVDOTDOT){
		switch(parent){
		case Qtopdir:
		case Qgpiodir:
			mkdeventry(c, qid, &topdir, db);
			break;
		default:
			return -1;
		}
		return 1;
	}

	if(parent == Qtopdir){
		switch(s){
		case 0:
			mkdeventry(c, qid, &gpiodir, db);
			break;
		default:
			return -1;
		}
		return 1;
	}

	if(parent == Qgpiodir){
		l = nelem(typedir);
		if(s < l){
			mkdeventry(c, qid, &typedir[s], db);
		} else if(s < l+Ngpiopin){
			s -= l;
			if(gpiotable[s] == nil)
				return 0;
			mkqid(&qid, PATH(s, Qgpiodir, Qdata), 0, QTFILE);
			devdir(c, qid, gpiotable[s], 0, eve, 0666, db);
		} else
			return -1;
		return 1;
	}

	return 1;
}

/* gpiointrenable()'s per-pin callback: mark the bit and wake any reader. */
static void
gpioevent(uint pin, void*)
{
	SET_BIT(eventvalue, pin, 1);
	wakeup(&rend);
}

static void
gpioinit(void)
{
	int i;

	/*
	 * Try each known board's compatible string in turn, same gate
	 * every board-exclusive driver in this tree uses per board
	 * (usbxhcipicoitx.c/usbxhcicom260.c's reset(), gpiok3init()/
	 * k1thermalinit()/k3thermalinit()) -- except here, instead of
	 * one file per board each refusing on the wrong one, a single
	 * gpioinit() just picks whichever table matches. Exactly one of
	 * these matches on real hardware; on a board this file has no
	 * table for (bpif3, qemu, ...) neither matches and gpiotable[]
	 * is left all-nil -- #G/gpio/ still exists, but only shows ctl
	 * and event, no per-pin files, instead of offering names that
	 * don't apply.
	 */
	if(dtbsocmatch("spacemit,k3-pico-itx")){
		for(i = 0; i < nelem(picoitxpins); i++)
			gpiotable[picoitxpins[i].pin] = picoitxpins[i].name;
	} else if(dtbsocmatch("spacemit,k3-com260-ifx")){
		for(i = 0; i < nelem(com260pins); i++)
			gpiotable[com260pins[i].pin] = com260pins[i].name;
	} else
		print("gpiok3: unrecognized k3 board, no pin files\n");
}

static void
gpioshutdown(void)
{
}

static Chan*
gpioattach(char *spec)
{
	return devattach('G', spec);
}

static Walkqid*
gpiowalk(Chan *c, Chan *nc, char **name, int nname)
{
	return devwalk(c, nc, name, nname, 0, 0, gpiogen);
}

static int
gpiostat(Chan *c, uchar *db, int n)
{
	return devstat(c, db, n, 0, 0, gpiogen);
}

static Chan*
gpioopen(Chan *c, int omode)
{
	c = devopen(c, omode, 0, 0, gpiogen);

	switch(FILE_TYPE(c->qid)){
	case Qdata:
		c->iounit = 1;
		break;
	case Qevent:
		lock(&eventlock);
		if(eventinuse != 0){
			c->flag &= ~COPEN;
			unlock(&eventlock);
			error(Einuse);
		}
		eventinuse = 1;
		unlock(&eventlock);
		memset(eventvalue, 0, sizeof eventvalue);
		c->iounit = sizeof eventvalue;
		break;
	}

	return c;
}

static void
gpioclose(Chan *c)
{
	if(FILE_TYPE(c->qid) == Qevent && (c->flag & COPEN))
		eventinuse = 0;
}

static int
isset(void*)
{
	int i;

	for(i = 0; i < Nbankwords; i++)
		if(eventvalue[i])
			return 1;
	return 0;
}

static long
gpioread(Chan *c, void *va, long n, vlong off)
{
	int type;
	uint pin;
	char *a;
	int v;

	a = va;

	if(c->qid.type & QTDIR)
		return devdirread(c, va, n, 0, 0, gpiogen);

	type = FILE_TYPE(c->qid);

	switch(type){
	case Qdata:
		/*
		 * Fixed-size (1 byte) file: only hand back data at
		 * off==0, EOF (n=0) at any later offset. Without this,
		 * gpioin() gets re-read on every call regardless of
		 * off, cat() never sees EOF, and the pin is polled
		 * forever printing its (possibly still-'1') value on
		 * every loop iteration -- this is the exact bug the
		 * unpatched bcm devgpio.c this file was modelled on
		 * also has.
		 */
		if(off > 0){
			n = 0;
			break;
		}
		pin = PIN_NUMBER(c->qid);
		v = gpioin(pin);
		if(v < 0)
			error(Eio);	/* gpiok3init() hasn't run, or wrong board */
		a[0] = v? '1': '0';
		n = 1;
		break;
	case Qctl:
		n = 0;
		break;
	case Qevent:
		if(off >= sizeof eventvalue){
			off %= sizeof eventvalue;
			memset(eventvalue, 0, sizeof eventvalue);
		}
		sleep(&rend, isset, 0);
		if(off+n > sizeof eventvalue)
			n = sizeof eventvalue - off;
		memmove(a, (uchar*)eventvalue+off, n);
		break;
	}

	return n;
}

static int
getpin(char *pinname)
{
	int i;

	for(i = 0; i < Ngpiopin; i++)
		if(gpiotable[i] != nil && strcmp(gpiotable[i], pinname) == 0)
			return i;
	return -1;
}

static long
gpiowrite(Chan *c, void *va, long n, vlong)
{
	int type, i, mode;
	uint pin;
	char *arg;
	Cmdbuf *cb;
	Cmdtab *ct;

	if(c->qid.type & QTDIR)
		error(Eisdir);

	type = FILE_TYPE(c->qid);

	cb = parsecmd(va, n);
	if(waserror()){
		free(cb);
		nexterror();
	}
	ct = lookupcmd(cb, gpiocmd, nelem(gpiocmd));
	if(ct == nil)
		error(Ebadctl);

	switch(type){
	case Qdata:
		pin = PIN_NUMBER(c->qid);
		switch(ct->index){
		case CMzero:
			gpioout(pin, 0);
			break;
		case CMone:
			gpioout(pin, 1);
			break;
		default:
			error(Ebadctl);
		}
		break;
	case Qctl:
		switch(ct->index){
		case CMevent:
			mode = getpin(cb->f[3]);
			if(mode == -1)
				error(Ebadctl);
			pin = mode;
			arg = cb->f[1];
			mode = -1;
			for(i = 0; i < nelem(evtypename); i++)
				if(strcmp(evtypename[i], arg) == 0){
					mode = evmode[i];
					break;
				}
			if(mode == -1)
				error(Ebadctl);
			if(cb->f[2][0] == 'e')
				gpiointrenable(pin, mode, gpioevent, nil);
			else if(cb->f[2][0] == 'd')
				gpiointrdisable(pin);
			else
				error(Ebadctl);
			break;
		default:
			error(Ebadctl);
		}
		break;
	default:
		error(Ebadctl);
	}

	free(cb);
	poperror();
	return n;
}

/*
 * Symbol name matches the "gpiok3" keyword this file is listed under
 * in rv64's "dev" section (mkdevlist's devKEYWORD.c <-> KEYWORDdevtab
 * convention -- same one devarch.c/archdevtab uses), NOT bcm
 * devgpio.c's "gpio"/gpiodevtab, and NOT the old per-board
 * gpiopicoitxdevtab/gpiocom260devtab symbols this file replaces. The
 * exposed namespace name below ("gpio") is unrelated and can stay
 * the same as bcm's -- it's just the string under #G.
 */
Dev gpiok3devtab = {
	'G',
	"gpio",

	devreset,
	gpioinit,
	gpioshutdown,
	gpioattach,
	gpiowalk,
	gpiostat,
	gpioopen,
	devcreate,
	gpioclose,
	gpioread,
	devbread,
	gpiowrite,
	devbwrite,
	devremove,
	devwstat,
};
