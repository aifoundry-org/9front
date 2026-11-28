# one /memory node with two <base size> pairs, as U-Boot writes them:
# 2 GiB of RAM, used once
MEM=2G
dts() {
	sed 's/reg = <0x00 0x80000000 0x00 0x80000000>;/reg = <0x00 0x80000000 0x00 0x40000000 0x00 0xc0000000 0x00 0x40000000>;/'
}
script() {
	cat <<'X'
want "meminit: bank 0x80800000-0x100000000"
rc
has {awk '$2 == "memory" { if($1 > 2000000000 && $1 <= 2147483648) print "memory ok"; else print "memory " $1 }' /dev/swap} "memory ok"
X
}
