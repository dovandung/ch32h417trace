#!/bin/sh
# Decode throughput on simulated captures (file in page cache, best of 3).
# Usage: ./bench.sh [binary] [capture dir]
B=${1:-./ch32trace}
D=${2:-${TMPDIR:-/tmp}/ch32bench}
mkdir -p "$D"
[ -f "$D/b4.bin" ] || python3 gen_trace.py "$D/b4.bin" --clk 50e6 --counter 3000000 --sync-every 256 > /dev/null
[ -f "$D/b4c.bin" ] || python3 gen_trace.py "$D/b4c.bin" --clk 60e6 --counter 3000000 --sync-every 256 --skew 0.3 --jitter 0.1 > /dev/null
[ -f "$D/b1.bin" ] || python3 gen_trace.py "$D/b1.bin" --width 1 --clk 37.5e6 --counter 700000 --sync-every 256 > /dev/null

run() {	# name, file, args
	n=$(stat -L -c %s "$2")
	cat "$2" > /dev/null
	best=
	for k in 1 2 3; do
		t0=$(date +%s.%N)
		"$B" -i "$2" -q $3 > /dev/null 2>&1
		t1=$(date +%s.%N)
		t=$(echo "$t1 - $t0" | bc)
		if [ -z "$best" ] || [ "$(echo "$t < $best" | bc)" = 1 ]; then best=$t; fi
	done
	printf '%-36s %7.0f MS/s\n' "$1" "$(echo "$n / $best / 1000000" | bc -l)"
}

run "4-bit 50 MHz, -o"                "$D/b4.bin"  "-o /dev/null"
run "4-bit 50 MHz, -o + ITM check -C"  "$D/b4.bin"  "-o /dev/null -C 1"
run "4-bit 60 MHz, -o"                "$D/b4c.bin" "-o /dev/null"
run "1-bit 37.5 MHz, -o"              "$D/b1.bin"  "-d 1 -o /dev/null"
