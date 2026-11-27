# 1 GiB of RAM: user space starts at the 4 GiB floor
script() {
	cat <<'X'
want "meminit: uzero 0x100000000"
rc
has {cat /proc/$pid/segment} "Text" "100000000"
X
}
