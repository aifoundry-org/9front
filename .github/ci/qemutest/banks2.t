# two adjacent /memory nodes make one bank; with half of RAM for the
# kernel, user pages come from the second node
MEM=2G
APPEND="console=0 *kernelpercent=50"
dts() {
	sed -e 's/reg = <0x00 0x80000000 0x00 0x80000000>;/reg = <0x00 0x80000000 0x00 0x40000000>;/' -e '$d'
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
want "meminit: bank 0x80800000-0x100000000"
rc
has {awk '$2 == "memory" { if($1 > 2000000000 && $1 <= 2147483648) print "memory ok"; else print "memory " $1 }' /dev/swap} "memory ok"
has {awk 'BEGIN { s = "0123456789abcdef"; for(i = 0; i < 20; i++) s = s s; print length(s), gsub(/0123456789abcdef/, "", s) }'} "16777216 1048576"
X
}
