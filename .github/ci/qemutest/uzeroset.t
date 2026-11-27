# *uzero moves user space to 64 GiB
APPEND="console=0 *uzero=0x1000000000"
script() {
	cat <<'X'
want "meminit: uzero 0x1000000000"
rc
has {cat /proc/$pid/segment} "Text" "1000000000"
X
}
