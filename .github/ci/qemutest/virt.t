# QEMU virt boots to a shell; no SoC module claims it
script() {
	cat <<'X'
rc
has {cat '#P/archctl'} "soc none"
X
}
