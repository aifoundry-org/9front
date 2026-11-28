# QEMU virt boots to a shell; virt is a SoC module; PLIC from the DT
script() {
	cat <<'X'
want "plic: 95 sources, context 1"
rc
has {cat '#P/archctl'} "soc virt riscv-virtio"
X
}
