# all harts of a 4-way virt come up
SMP=4
script() {
	cat <<'X'
rc
has {cat /dev/sysstat} "          0 " "          3 "
X
}
