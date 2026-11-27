# the AIBOX-K3 shares the CoM260 compatible and is told apart by the
# root model.  The module then drives K3 hardware, so stop at the choice.
dts() {
	sed 's/compatible = "riscv-virtio";/compatible = "spacemit,k3-com260", "spacemit,k3", "riscv-virtio";/
		s/model = "riscv-virtio,qemu";/model = "Firefly AIBOX-K3";/'
}
script() {
	cat <<'X'
wantnot "soc: k3-com260-ifx" "soc: aibox-k3"
X
}
