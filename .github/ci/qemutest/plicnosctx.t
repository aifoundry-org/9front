# a PLIC with M-mode contexts only is refused, not guessed at
SMP=2
dts() {
	sed '/interrupts-extended/s/ 0x[0-9a-f]* 0x09//g'
}
script() {
	cat <<'X'
want "panic: plic: no S-mode context for hart 0"
X
}
