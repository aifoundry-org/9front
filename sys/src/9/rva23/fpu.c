#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

#include "ureg.h"
#include "../rva23/sysreg.h"
#include "riscv64.h"

/* from 9k */
enum {						/* FCSR */
	I		= FPAINEX,
	D		= FPAINVAL,		/* Denormalized-Operand */
	Z		= FPAZDIV,
	O		= FPAOVFL,		/* Overflow */
	U		= FPAUNFL,		/* Underflow */
};

enum {						/* PFPU.state */
	Hold		= 1<<2,			/* Handling an FPU note */
	TrapSpew		= 0,
};

static char *fpstnames[] = {
	"Init", "Active", "Inactive",
};

/* The rules:
 * 9front kernels allow kernel and note handler use of FPU.
 * Exceptions can nest, requiring a stack of saved FPU register values.
 ** This stack is implemented via a linked list.
 * CPU takes a trap on ANY access to FPU.
 ** All setting is done at SPLHI in assembly. No code in this file should touch the FP registers.
 * Registers F28-F31 hold constants.
 * FPU registers should never be used in this file.
 * FPU is enabled by 2 bits in SSTATUS (FS). If those bits are 0, FP use traps.
 * SSTATUS in the ureg is a saved copy of SSTATUS
 ** Weirdly, the FPU bits are not cleared to 0 when strap occurs.
 * The FPU status is restored when we leave the kernel to user mode.
 * Or when we leave this trap to go back to kernel.
 ** fpsts2ureg sets FS for the return to user mode: on only while the
 ** process's registers are live on this hart, which a process switch ends.
 */

/* initial contents of high fp regs.  compiler assumes such initialization. */
/* See: fpconstset: 28, 29, 30, and 31 are set to these values */
double fpzero = 0, fphalf = 0.5, fpone = 1, fptwo = 2;
/* thank you 9k! */

/* FPalloc allocates an FPsave structure and sets its link to the passed in FPalloc *.
 * This linked list basically forms a stack. */
static FPalloc*
fpalloc(FPalloc *link)
{
	FPalloc *a;

	while((a = mallocalign(sizeof(FPalloc), 16, 0, 0)) == nil){
		int x = spllo();
		if(up != nil && !waserror()){
			resrcwait("no memory for FPalloc");
			poperror();
		}
		splx(x);
	}
	memset(a, 0, sizeof(*a));
	a->link = link;
	return a;
}

static void
fpfree(FPalloc*a)
{
	free(a);
}

static FPsave fpsave0;

static void
fpsave(FPsave *p)
{
	if (TrapSpew)print("fpsave\n");
	p->fcsr = getfcsr();
	fpsaveregs(p->regs);
	fpoff();
}

static void
fprestore(FPsave *p)
{
	if (TrapSpew)print("fprestore\n");
	fpon();
	setfcsr(p->fcsr);
	fploadregs(p->regs);
}

/* fpinit loads the registers from fpsave0.
 * Critically, the constant registers will be set.
 * This used to load from fpsave0, but that does not
 * have f28-31 set, which in actual use did not end well.
 */
static void
fpinit(void)
{
	if (TrapSpew)print("fpinit\n");
	_fpuinit();
}

/* notefpsave saves processor FPU state if the process has been using the FPU. */
FPsave*
notefpsave(Proc *p)
{
	if(p->fpsave == nil)
		return nil;
	if(p->fpstate == (FPinactive|FPnotify)){
		p->fpsave = fpalloc(p->fpsave);
		memmove(p->fpsave, p->fpsave->link, sizeof(FPsave));
		p->fpstate = FPinactive;
	}
	return p->fpsave->link;
}

void
fpuprocsave(Proc *p)
{
	if (TrapSpew)print("fpuprocsave\n");
	// Process dead? nothing do to.
	if(p->state == Moribund){
		FPalloc *a;

		if(p->fpstate > FPinit || p->kfpstate > FPinit)
			fpoff();
		p->fpstate = p->kfpstate = FPinit;
		while((a = p->fpsave) != nil) {
			p->fpsave = a->link;
			fpfree(a);
		}
		while((a = p->kfpsave) != nil) {
			p->kfpsave = a->link;
			fpfree(a);
		}
		return;
	}
	// kfpstate is kernel fp state? Kernel using FPU I guess? or ...
	if(p->kfpstate == FPactive){
		fpsave(p->kfpsave);
		p->kfpstate = FPinactive;
		return;
	}
	if(p->fpstate != FPactive)
		return;
	fpsave(p->fpsave);
	p->fpstate = FPinactive;
}

/* from here on down, ron is uncertain. */
void
fpuprocrestore(Proc*)
{
	if (TrapSpew)print("fpuprocrestore\n");
	/*
	 * when the scheduler switches,
	 * we can discard its fp state.
	 */
	switch(m->fpstate){
	case FPactive:
		fpoff();
		/* wet floor */
	case FPinactive:
		fpfree(m->fpsave);
		m->fpsave = nil;
		m->fpstate = FPinit;
	}
}

/* exec: the new image starts without fp state */
void
fpuprocsetup(Proc *p)
{
	FPalloc *a;

	p->fpstate = FPinit;
	while((a = p->fpsave) != nil) {
		p->fpsave = a->link;
		fpfree(a);
	}
}

/* fork the fpu state from parent to child.
 * If the parent was inactive, then the child will be --
 * they resume at the same PC. Otherwise, save the parent
 * FPU state, such that it can be recreated in the child,
 * such that they both can exit with the same FPU state.
 * We need a test for this. */
void
fpuprocfork(Proc *p)
{
	int s;

	s = splhi();
	switch(up->fpstate & ~FPnotify){
	case FPactive:
		fpsave(up->fpsave);
		up->fpstate = FPinactive;
		/* wet floor */
	case FPinactive:
		/*
		 * The parent's state is saved in up->fpsave; give the child
		 * its own copy.  Falling through from FPactive is not
		 * enough: an already-inactive parent must copy too,
		 * otherwise the child is left FPinactive with a nil fpsave and
		 * faults in fpsave() the first time it is scheduled out.
		 */
		if(p->fpsave == nil)
			p->fpsave = fpalloc(nil);
		memmove(p->fpsave, up->fpsave, sizeof(FPsave));
		p->fpstate = FPinactive;
		break;
	}
	splx(s);
}

void
fpunotify(Proc *p)
{
	fpuprocsave(p);
	p->fpstate |= FPnotify;
}

void
fpunoted(Proc *p)
{
	FPalloc *o;

	if(p->fpstate & FPnotify) {
		p->fpstate &= ~FPnotify;
	} else if((o = p->fpsave->link) != nil) {
		fpfree(p->fpsave);
		p->fpsave = o;
		p->fpstate = FPinactive;
	} else {
		p->fpstate = FPinit;
	}
}

/*
 * You get here out of the illegal instruction handler, when FPU is turned off.
 * Your task is to figure out what mode the trap occured in, if the FPU was in use before it was turned
 * off, and how to construct a return from as you exit the trap handler back to kernel, note handler, or
 * process. During trap entry, FPU is turned off, but its state as of the trap is preserved.
 */
void
mathtrap(Ureg *ureg)
{
	if (TrapSpew)print("mathtrap %p\n", ureg->pc);
	/* we were not in user mode. So, the kernel decided to use the FPU,
	 * and if there is state to be preserved, here is where we do it.
	 * We will set up the FPU here and just return. */

	/* no user stuff going on. Ron is unsure about this code. */
	if(!userureg(ureg)){
		if (TrapSpew)print("kernel mode\n");
		/* there was no process active. So we need only worry about the kernel FP state. */
		if(up == nil){
			/* This step pushes a new fpsave onto the stack. */
			switch(m->fpstate){
			/* It was not used. Set up an FPsave and it will be popped on the way out of the trap handler */
			case FPinit:
				m->fpsave = fpalloc(m->fpsave);
				m->fpstate = FPactive;
				fpinit();
				break;
			case FPinactive:
				fprestore(m->fpsave);
				m->fpstate = FPactive;
				break;
			default:
				panic("floating point error in irq for FP state %d", m->fpstate);
			}
			return;
		}
		if (TrapSpew)print("fpstate %d\n", up->fpstate);
		if(up->fpstate == FPactive){
			if (TrapSpew)print("fp active. save it\n");
			fpsave(up->fpsave);
			up->fpstate = FPinactive;
		}

		switch(up->kfpstate){
		case FPinit:
			up->kfpsave = fpalloc(up->kfpsave);
			up->kfpstate = FPactive;
			fpinit();
			break;
		case FPinactive:
			fprestore(up->kfpsave);
			up->kfpstate = FPactive;
			break;
		default:
			panic("floating point error in trap");
		}
		return;
	}

	if (TrapSpew)print("user trap: up -> fpstate %d\n", up->fpstate);
	switch(up->fpstate){
	case FPinit|FPnotify:
		/* wet floor */
	case FPinit:
		if (TrapSpew)print("init: turn it on\n");
		if(up->fpsave == nil)
			up->fpsave = fpalloc(nil);
		up->fpstate = FPactive;
		fpinit();
		break;
	case FPinactive|FPnotify:
		if (TrapSpew)print("idle | notify\n");
		spllo();
		qlock(&up->debug);
		notefpsave(up);
		qunlock(&up->debug);
		splhi();
		/* wet floor */
	case FPinactive:
		if (TrapSpew)print("inactive\n");
		fprestore(up->fpsave);
		up->fpstate = FPactive;
		break;
	case FPactive:
		if (TrapSpew)print("active\n");
		postnote(up, 1, "sys: floating point error", NDebug);
		break;
	}
	if (TrapSpew)print("isfpon: %s\n", up->dbgreg->status & Fsst ? "yes" : "no");
}

/* FS for the return to user mode */
void
fpsts2ureg(Ureg*ureg)
{
	ureg->status &= ~Fsst;
	if(up->fpstate == FPactive)
		ureg->status |= Dirty<<Fsshft;
}

void
fpukexit(Ureg*, FPsave*)
{
	FPalloc *a;
	if (TrapSpew)print("fpukexit\n");

	/* if there is no process, just turn the FPU off. */
	if(up == nil){
		/* if it was used by the kernel, save state */
		switch(m->fpstate){
		case FPactive:
			fpoff();
			/* wet floor */
		case FPinactive:
			a = m->fpsave;
			m->fpsave = a->link;
			fpfree(a);
		}
		m->fpstate = m->fpsave != nil? FPinactive: FPinit;
		return;
	}

	switch(up->kfpstate){
	case FPactive:
		fpoff();
		/* wet floor */
	case FPinactive:
		a = up->kfpsave;
		up->kfpsave = a->link;
		fpfree(a);
	}
	up->kfpstate = up->kfpsave != nil? FPinactive: FPinit;
}
