# *e820 replaces the device tree's RAM: 0xb0000000-0xc0000000 is in
# the tree but not used, 0xc0000000-0xd0000000 is used but not in the
# tree.  Its type 2 region and the tree's reserved memory are holes.
MEM=2G
APPEND="console=0 *e820='1 0x80000000 0xb0000000 2 0xa0000000 0xa1000000 1 0xc0000000 0xd0000000'"
dts() {
	sed -e 's/reg = <0x00 0x80000000 0x00 0x80000000>;/reg = <0x00 0x80000000 0x00 0x40000000>;/' -e '$d'
	cat <<'Y'
	reserved-memory {
		#address-cells = <0x02>;
		#size-cells = <0x02>;
		ranges;
		fixed@98000000 {
			reg = <0x00 0x98000000 0x00 0x1000000>;
		};
	};
};
Y
}
script() {
	cat <<'X'
want "meminit: uzero 0x100000000"
want "meminit: bank 0x80800000-0x98000000"
want "meminit: bank 0x99000000-0xa0000000"
want "meminit: bank 0xa1000000-0xb0000000"
want "meminit: bank 0xc0000000-0xd0000000"
rc
has {awk '$2 == "memory" { if($1 > 1031000000 && $1 <= 1031798784) print "memory ok"; else print "memory " $1 }' /dev/swap} "memory ok"
X
}
