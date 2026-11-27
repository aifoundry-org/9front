# a root compatible naming the K1 selects its SoC module.  The module
# then drives K1 hardware that virt lacks, so stop at the choice.
dts() {
	sed 's/compatible = "riscv-virtio";/compatible = "spacemit,k1-x", "riscv-virtio";/'
}
script() {
	cat <<'X'
wantnot "soc: virt" "soc: k1"
X
}
