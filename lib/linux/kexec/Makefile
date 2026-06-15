# Build on the target board.  Needs gcc and libfdt-dev:
#   apt-get install gcc libfdt-dev
# Static linking pulls in libfdt's .a from libfdt-dev.
# Then run it like this on a 2GB BPI-F3:
# ./kexec -b "*maxmem=0x80000000" 9qemu.elf
# maxmem can be different if you've got more memory

CFLAGS := -Wall -O2

kexec: kexec.c
	gcc $(CFLAGS) -o $@ ./kexec.c -lfdt

clean:
	rm -f kexec
