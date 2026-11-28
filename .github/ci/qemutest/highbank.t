# a bank above the first 6 GiB, with a hole below it: uzero moves up,
# and user pages come from that bank
MEM=5G
APPEND="console=0 *kernelpercent=50"
dts() {
	sed -e 's/reg = <0x00 0x80000000 0x01 0x40000000>;/reg = <0x00 0x80000000 0x00 0x40000000>;/' -e '$d'
	cat <<'Y'
	memory@180000000 {
		device_type = "memory";
		reg = <0x01 0x80000000 0x00 0x40000000>;
	};
};
Y
}
script() {
	cat <<'X'
want "meminit: uzero 0x1c0000000"
want "meminit: bank 0x180000000-0x1c0000000"
rc
has {awk 'BEGIN { s = "0123456789abcdef"; for(i = 0; i < 20; i++) s = s s; print length(s), gsub(/0123456789abcdef/, "", s) }'} "16777216 1048576"
X
}
