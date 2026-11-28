# the kernel's bank holds 64 MiB, the rest of RAM is elsewhere,
# so user pages and much of the kernel heap come from the second bank
MEM=2G
QEMUARGS="-netdev user,id=n0 -device virtio-net-pci,netdev=n0,disable-legacy=on,romfile="
dts() {
	sed -e 's/reg = <0x00 0x80000000 0x00 0x80000000>;/reg = <0x00 0x80000000 0x00 0x4000000>;/' -e '$d'
	cat <<'Y'
	memory@c0000000 {
		device_type = "memory";
		reg = <0x00 0xc0000000 0x00 0x40000000>;
	};
};
Y
}
script() {
	cat <<'X'
want "meminit: uzero 0x100000000"
want "meminit: bank 0xc0000000-0x100000000"
rc
has {awk 'BEGIN { s = "0123456789abcdef"; for(i = 0; i < 20; i++) s = s s; print length(s), gsub(/0123456789abcdef/, "", s) }'} "16777216 1048576"
run {bind -a '#I' /net; bind -a '#l0' /net}
run {ip/ipconfig ether /net/ether0}
has {cat /net/ndb} "ip=10.0.2.15"
X
}
