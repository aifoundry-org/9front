/*
 * GPIO pins, on whichever controller the SoC driver registered
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

static Gpioifc *gpio;

void
addgpio(Gpioifc *g)
{
	if(gpio == nil)
		gpio = g;
}

void
gpioout(uint pin, int set)
{
	if(gpio != nil && pin < gpio->npin)
		gpio->out(pin, set);
}

/* the pin's level, or -1 */
int
gpioin(uint pin)
{
	if(gpio == nil || pin >= gpio->npin)
		return -1;
	return gpio->in(pin);
}

/* the pin's level without changing its direction, or -1 */
int
gpiopeek(uint pin)
{
	if(gpio == nil || pin >= gpio->npin)
		return -1;
	return gpio->peek(pin);
}

void
gpiointrenable(uint pin, int mode, void (*f)(uint, void*), void *a)
{
	if(gpio != nil && pin < gpio->npin)
		gpio->intrenable(pin, mode, f, a);
}

void
gpiointrdisable(uint pin)
{
	if(gpio != nil && pin < gpio->npin)
		gpio->intrdisable(pin);
}
