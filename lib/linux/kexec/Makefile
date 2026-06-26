# Build on the target board.  Needs gcc and libfdt-dev:
#   apt-get install gcc libfdt-dev
# Static linking pulls in libfdt's .a from libfdt-dev.
#
# Working invocation on a 2 GB BPI-F3 (Spacemit K1):
#   ./kexec -b "*maxmem=0x80000000 uartbase=0xd4017000 \
#                k1plicmask=42 k1ier=0x55 k1fcr=0x41 console=0" \
#           9bpif3.elf
#
# *maxmem can be different if you've got a 4/8/16 GB board.
# k1plicmask/k1ier/k1fcr/console are needed to make UART RX work on the K1;
# see sys/src/9/riscv64/uarti8250.c for the per-knob explanation.

CFLAGS := -Wall -O2

kexec: kexec.c
	gcc $(CFLAGS) -o $@ ./kexec.c -lfdt

clean:
	rm -f kexec
