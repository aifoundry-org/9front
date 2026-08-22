enum {
	SpacemiT	= 0x710,	/* soc.vendor for the K1 (and K3) */

	WdtPhys		= 0xD4080000ull,
	WdtSize		= 0x100,
	WdtWfar		= 0xb0,		/* write-access magic 1: 0xbaba */
	WdtWsar		= 0xb4,		/* write-access magic 2: 0xeb10 */
	WdtWmer		= 0xb8,		/* match enable */
	WdtWmr		= 0xbc,		/* match value */
	WdtWsr		= 0xc0,		/* status */
	WdtWcr		= 0xc8,		/* counter reset */

	MpmuPhys	= 0xD4050000ull,
	MpmuSize	= 0x2000,
	MpmuWdtpcr	= 0x200,	/* WDT power/clock/reset gate */
	MpmuAprr	= 0x1020,	/* APRR_WDTR (bit 4) enables reset */

	ApmuPhys	= 0xD4282800ull,
	ApmuSize	= 0x400,
	ApmuSdh0	= 0x54,		/* SDH0 clock/reset control */
	ApmuSdh2	= 0xe0,		/* SDH2 clock/reset control */
	SdhPorval	= 0x100,	/* reset value: block in reset, clocks off, div 1 */

	ApbcPhys	= 0xD4015000ull,
	ApbcSize	= 0x100,
	ApbcTwsi8	= 0x20,		/* write-only: bit 2 reset, 1 fnclk, 0 apbclk */

	Twsi8Phys	= 0xD401D800ull,
	Twsi8Size	= 0x40,
	TwsiIcr		= 0x00,
	 Iue		= 1<<14,	/* unit enable */
	 Scle		= 1<<13,	/* drive SCL (master) */
	 Tb		= 1<<3,		/* transfer byte */
	 Acknak		= 1<<2,		/* NAK after received byte */
	 Stop		= 1<<1,
	 Start		= 1<<0,
	TwsiIsr		= 0x04,
	 Irf		= 1<<20,	/* IDBR receive full (W1C) */
	 Ite		= 1<<19,	/* IDBR transmit empty (W1C) */
	TwsiIdbr	= 0x0c,

	PmicAddr	= 0x41,		/* P1 power-management chip on TWSI8 */
	PmicPwrctrl2	= 0x7e,
	 PmicShutdown	= 1<<2,		/* shutdown request */
	 PmicRstreq	= 1<<1,		/* reset request */
};
