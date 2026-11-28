# the device tree header's /memreserve/ entries are left out of RAM,
# rounded to whole pages
dts() {
	sed '1a\
/memreserve/ 0x90000000 0x1000000;\
/memreserve/ 0xa0000800 0x1000;'
}
script() {
	cat <<'X'
want "meminit: bank 0x80800000-0x90000000"
want "meminit: bank 0x91000000-0xa0000000"
want "meminit: bank 0xa0002000-0xc0000000"
rc
X
}
