# virtio-net on PCIe gets a BAR; its interrupts through the PLIC carry DHCP
SMP=2
QEMUARGS="-netdev user,id=n0 -device virtio-net-pci,netdev=n0,disable-legacy=on,romfile="
script() {
	cat <<'X'
rc
has {cat '#$/pci/'*ctl} "1:40000000"
run {bind -a '#I' /net; bind -a '#l0' /net}
run {ip/ipconfig ether /net/ether0}
has {cat /net/ndb} "ip=10.0.2.15"
X
}
