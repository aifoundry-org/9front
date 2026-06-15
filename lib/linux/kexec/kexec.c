#include <unistd.h>
#include <sys/syscall.h>
#include <linux/reboot.h>
#include <sys/reboot.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>

/* RISC-V 64-bit syscall numbers */
#ifndef __NR_kexec_load
#define __NR_kexec_load 283
#endif

#ifndef __NR_reboot
#define __NR_reboot 142
#endif

/* KEXEC Architecture flag for RISC-V */
#define KEXEC_ARCH_RISCV (243 << 16)

#include "fdt-bpif3.c"

/* 64-byte segment, 32-bit aligned */
/* Instructions:
 * 0x04800513 : li a0, 72 (0x48)
 * 0x00100893 : li a7, 1
 * 0x00000073 : ecall
 * 0xff5ff06f : j -12 (Infinite loop to the start)
 * Remaining bytes are 0 padding.
 */
static unsigned int loop_payload[1048576] __attribute__((aligned(1024))) = {
	0x04800513, 0x00100893, 0x00000073, 0xff5ff06f,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

struct kexec_segment {
	const void *buf;
	size_t bufsz;
	unsigned long mem;
	size_t memsz;
};

int main(int argc, char *argv[]) {
	struct kexec_segment segs[2];

	segs[0].buf = loop_payload;
	segs[0].bufsz = sizeof(loop_payload);
	segs[0].mem = 0x40200000ULL; // BPI-F3: 1GiB + 2MB, matches KTZERO
	segs[0].memsz = sizeof(loop_payload);

	segs[1].buf = bpif3_dtb;
	segs[1].bufsz = sizeof(bpif3_dtb);
	segs[1].mem = segs[0].mem + segs[0].memsz;
	/* memsz must be page-aligned or kexec_load returns EADDRNOTAVAIL */
	segs[1].memsz = (sizeof(bpif3_dtb) + 0xfffUL) & ~0xfffUL;

	if (argc > 1) {
		int fd = open(argv[1], 0);
		if (fd < 0) {
			perror("open");
			exit(1);
		}
		if (read(fd, (void *)loop_payload, sizeof(loop_payload)) < 0){
			perror("read");
			exit(2);
		}
	}

	/* 1. Load the segment into the kernel's kexec slots */
	long load_ret = syscall(__NR_kexec_load,
				(unsigned long)segs[0].mem, // Entry point
				2UL,                    // Number of segments
				&segs,
				(unsigned long)KEXEC_ARCH_RISCV);

	if (load_ret != 0) {
		perror("failed to load the segment into the kexec slots");
		return 1; // Load failed (likely permissions or memory conflict)
	}

	/* 2. Execute the reboot into the loaded image */
	/* Note: This requires the LINUX_REBOOT_CMD_KEXEC (0x45584543) magic */
	syscall(__NR_reboot,
		LINUX_REBOOT_MAGIC1,
		LINUX_REBOOT_MAGIC2,
		LINUX_REBOOT_CMD_KEXEC,
		NULL);

	perror("failed to reboot into Plan 9 kernel");
	/* If reboot returns, something went wrong */
	return 2;
}
