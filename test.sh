#!/bin/sh
# Decode simulated captures and check the ITM counter comes through intact.
# Usage: ./test.sh   (needs python3 with numpy, and a built ./ch32trace)
set -e
T=$(mktemp -d)
trap 'rm -rf "$T"' EXIT
fail=0

check() {	# name, generator args, decoder args
	name=$1; gen=$2; dec=$3
	python3 gen_trace.py "$T/c.bin" $gen > /dev/null
	line=$(./ch32trace -i "$T/c.bin" -C 1 $dec 2>&1 | grep "counter check")
	breaks=$(echo "$line" | sed 's/.*values, \([0-9]*\) breaks.*/\1/')
	if [ "$breaks" = 0 ]; then r=PASS; else r=FAIL; fail=1; fi
	printf '%-5s %-28s %s\n' "$r" "$name" "${line#*: }"
}

check "4-bit 50 MHz"           "--clk 50e6"                         ""
check "4-bit 45 MHz"           "--clk 45e6"                         ""
check "4-bit 60 MHz"           "--clk 60e6 --skew 0.3 --jitter 0.1" ""
check "4-bit 20 MHz, skew 2"   "--clk 20e6 --skew 2"                ""
check "4-bit centered data"    "--center"                           ""
check "4-bit 2 MHz"            "--clk 2e6 --counter 20000"          ""
check "4-bit forced -p 2"      ""                                   "-p 2"
check "4-bit forced -p -1"     ""                                   "-p -1"
check "2-bit 50 MHz"           "--width 2 --counter 60000"          "-d 1,2"
check "1-bit 25 MHz"           "--width 1 --clk 25e6 --counter 30000" "-d 1"

# a capture with samples cut out must report one break and the loss
python3 gen_trace.py "$T/c.bin" --drop 123457 --sync-every 16 > /dev/null
line=$(./ch32trace -i "$T/c.bin" -C 1 2>&1 | grep "counter check")
case "$line" in
	*" 1 breaks"*) r=PASS ;;
	*) r=FAIL; fail=1 ;;
esac
printf '%-5s %-28s %s\n' "$r" "dropped samples detected" "${line#*: }"

# SWO: UART (auto-baud, given baud with clock error) and Manchester
swo() {	# name, generator args, decoder args, expected values
	python3 gen_trace.py "$T/s.bin" --counter 20000 $2 > /dev/null
	line=$(./ch32trace -i "$T/s.bin" -C 1 $3 2>&1 | grep "counter check")
	case "$line" in
		*": $4 values, 0 breaks, 0 missing"*) r=PASS ;;
		*) r=FAIL; fail=1 ;;
	esac
	printf '%-5s %-28s %s\n' "$r" "$1" "${line#*: }"
}
swo "SWO uart 6M, auto-baud"     "--swo uart --baud 6e6"                     "-W 5"                 20000
swo "SWO uart 2M +2%, -b 2M"     "--swo uart --baud 2e6 --baud-err 0.02"     "-W 5 -b 2000000"      20000
swo "SWO uart 37.5M (8 s/bit)"   "--swo uart --baud 37.5e6"                  "-W 5"                 20000
swo "SWO manchester 4M"          "--swo manchester --baud 4e6"               "-W 5 -M manchester"   20000
swo "SWO manchester 37.5M"       "--swo manchester --baud 37.5e6 --seed 2"   "-W 5 -M manchester"   20000
swo "SWO uart + TPIU formatter"  "--swo uart --baud 6e6 --swo-tpiu"          "-W 5 -F"              19998

# no clock for a while first, and a clock that stops and comes back faster:
# it must wait, then decode (the second run restarts the counter: 1 break)
python3 gen_trace.py "$T/a.bin" --clk 20e6 --counter 30000 > /dev/null
python3 gen_trace.py "$T/b.bin" --clk 50e6 --counter 30000 --seed 3 > /dev/null
python3 - "$T" <<'PY'
import sys, numpy as np
T = sys.argv[1]
a = np.fromfile(T + "/a.bin", np.uint8); b = np.fromfile(T + "/b.bin", np.uint8)
idle = lambda n, v: np.full(n, v, np.uint8)
np.concatenate([idle(40_000_000, 0x80), a, idle(20_000_000, a[-1]), b]).tofile(T + "/w.bin")
PY
for isa in fast scalar; do
	out=$(CH32TRACE_ISA=$isa ./ch32trace -i "$T/w.bin" -C 1 2>&1)
	line=$(echo "$out" | grep "counter check")
	case "$out" in
		*"waiting for TRACECLK"*"stopped"*"found"*) ;;
		*) line="no wait message: $line" ;;
	esac
	case "$line" in
		*": 599"[89]?" values, 1 breaks, 0 missing"*) r=PASS ;;
		*) r=FAIL; fail=1 ;;
	esac
	printf '%-5s %-28s %s\n' "$r" "wait for clock ($isa)" "${line#*: }"
done

# every stage-A code path must give the same ITM stream, glitches included
python3 gen_trace.py "$T/c.bin" --clk 40e6 --glitch 2000 --jitter 0.3 --counter 60000 > /dev/null
ref=
for isa in fast avx2 sse2 scalar; do
	for p in "" "-p -1" "-p 1"; do
		CH32TRACE_ISA=$isa ./ch32trace -i "$T/c.bin" $p -w "$T/itm_$isa" -q > /dev/null 2>&1
		head -c 150000 "$T/itm_$isa" > "$T/itm_cut"	# the last frame may differ
		sum=$(cksum < "$T/itm_cut")
		[ -z "$ref" ] && ref=$sum
		if [ "$sum" = "$ref" ] || [ "$p" != "" ]; then r=PASS; else r=FAIL; fail=1; fi
		[ "$p" = "" ] && printf '%-5s %-28s %s\n' "$r" "same result: $isa" ""
	done
done

exit $fail
