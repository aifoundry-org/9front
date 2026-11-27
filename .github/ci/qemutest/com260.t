# the CoM260 IFX compatibles prefix the AIBOX module's too, but its
# model is not an AIBOX
dts() {
	sed 's/compatible = "riscv-virtio";/compatible = "spacemit,k3-com260-ifx", "spacemit,k3-com260", "spacemit,k3", "riscv-virtio";/
		s/model = "riscv-virtio,qemu";/model = "SpacemiT K3 CoM260 IFX";/'
}
script() {
	cat <<'X'
wantnot "soc: aibox-k3" "soc: k3-com260-ifx"
X
}
