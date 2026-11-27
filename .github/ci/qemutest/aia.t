# virt with an APLIC and IMSICs instead of the PLIC; DHCP through MSIs
MACHINE=virt,aia=aplic-imsic
SMP=2
QEMUARGS="-netdev user,id=n0 -device virtio-net-pci,netdev=n0,disable-legacy=on,romfile="
script() {
	cat <<'X'
want "aplic: 96 sources"
rc
run {bind -a '#I' /net; bind -a '#l0' /net}
run {ip/ipconfig ether /net/ether0}
has {cat /net/ndb} "ip=10.0.2.15"
X
}
