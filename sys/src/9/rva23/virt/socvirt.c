/*
 * QEMU virt: PCIe (pciqemu), virtio and the PLIC come from the
 * device tree; reset and power-off go through SBI
 */
#include "u.h"
#include "../port/lib.h"
#include "mem.h"
#include "dat.h"
#include "fns.h"

Socmod socvirt = {
	"virt",
	"riscv-virtio",
};
