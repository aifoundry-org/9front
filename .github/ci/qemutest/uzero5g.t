# 5 GiB of RAM from 2 GiB: user space starts at 7 GiB, above it
MEM=5G
SMP=2
script() {
	cat <<'X'
want "meminit: uzero 0x1c0000000"
rc
has {cat /proc/$pid/segment} "Text" "1c0000000"
X
}
