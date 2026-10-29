# ch32trace

Cortex-M **parallel trace** (TPIU, 1/2/4-bit, TRACECLK + TRACEDATA) capture
with the CH32H417 USB 3 logic analyzer, as a low-cost stand-in for the
ORBTrace mini's parallel trace input. It also captures **SWO** (UART/NRZ or
Manchester) on any analyzer channel. Output is an aligned TPIU (or SWO)
byte stream for **Orbuculum**, plus a built-in ITM decoder for quick checks.

It needs the OpenSourceLogic-CH32H417 APP firmware, version 3 or later for
300 MS/s, and talks the same USB protocol as the libsigrok `wch-ch32h417`
driver (EP1 commands, EP2 8-channel stream, 1 byte per sample, bit k =
channel k).

## How it works

```
analyzer (300 MS/s, 8 ch) ──USB3──► edge thread ──► bits thread ──► TPIU file / FIFO ──► orbuculum
                                    find TRACECLK    bit + byte          ITM decode:
                                    edges, pick one  alignment from      print a port,
                                    sample per half  0x7FFFFFFF sync,    check a counter
                                    period           TPIU deframing
```

1. Trace is DDR: one TRACEDATA value per clock **edge**. The tool finds
   the TRACECLK edges and takes one data sample per half period, at a fixed
   distance from an edge (clamped when two edges are closer than that), so
   a half period one sample shorter or longer (quantization, jitter,
   glitches) never drops or doubles a value.
2. **Sampling point is automatic.** The first ~1M samples are used to
   measure TRACECLK and where the data lines change relative to the clock
   edges. The tool then picks the sample furthest from those transitions,
   counted either after an edge or before the next one. This works for data
   that changes on the clock edge and for centre-aligned data. Override it
   with `-p N` (N after the edge) or `-p -N` (N before the next edge).
3. **Alignment.** The TPIU full sync (31 ones then a zero) gives bit and
   byte alignment for any port width. If the stream slips (lost samples),
   the next full sync re-aligns it, and the slip is counted as a resync.
4. The aligned bytes go to `-o`. The tool also deframes them itself (16-byte
   TPIU frames, stream IDs) and decodes ITM on stream 1.
5. **Waiting for the clock.** If TRACECLK is not running yet (target held in
   reset, trace not set up yet), ch32trace prints `waiting for TRACECLK on
   channel 0 ...` and waits as long as it takes; samples with no clock are
   dropped. If the clock later stops for about 28 ms (8M samples: target
   reset, debugger reconfiguring the trace port), it prints `TRACECLK
   stopped`, waits again, and measures the clock again when it comes back,
   in case its frequency changed. For SWO without `-b`, it waits the same
   way for traffic to measure the baud rate. Stop it with Ctrl-C. Only a
   file with no clock at all ends with `No clock found` (exit code 1).

## Limits

At 300 MS/s you need about 3 samples per half period:

| TRACECLK | samples per half period | result in simulation |
|---|---|---|
| ≤ 60 MHz | ≥ 2.5 | error-free (tested 2 – 60 MHz, 1/2/4-bit) |
| 75 MHz | 2 | fails, as expected |

- **Host CPU.** See *Speed* below: about 4–5× faster than 300 MS/s on a
  2.8 GHz cloud VM with 2 cores. If the host ever falls behind, the tool
  says so (`drop … B`) and marks a gap; it never decodes across a gap
  silently.
- **Device-side losses.** The firmware stream has no sequence numbers or
  overflow flag. If the analyzer itself loses data, it shows up only as
  TPIU frame errors, resyncs and counter breaks. Adding a block counter or
  overflow flag to the firmware stream would make these visible directly.
- **STM32F4/F7 TRACECLK.** These typically run TRACECLK at HCLK/2: 84 MHz at
  168 MHz HCLK is too fast. Lower HCLK while tracing (100 MHz HCLK gives
  50 MHz TRACECLK). On STM32H7 the trace clock comes from a PLL output and
  can be set independently.

## Speed (x86-64)

Decoding is split over two threads: **stage A** (clock edges → TRACEDATA
bits) and **stage B** (TPIU sync and alignment, frames, ITM). The USB
reads run in a third thread.

Stage A picks its code at run time from the CPU:

| CPU | stage A | how |
|---|---|---|
| AVX2 + BMI2 (Intel Haswell and later, AMD Zen 3 and later) | `avx2+bmi2 (pext)` | 64 samples per step: clock edges as a 64-bit mask (vpmovmskb), the sampling points as that mask shifted by the offset (clamped picks are masks too), and `pext` moves the TRACEDATA bits of all of them into the output in one go. No loop over edges, no branches per value. |
| AVX2 | `avx2` | 64-sample edge mask, then a short loop per edge |
| SSE2 (any x86-64) | `sse2` | same, 4 × 16 samples |
| other | `scalar` | same, multiply-based mask |

On AMD before Zen 3, `pext`/`pdep` are microcoded and slow, so the `avx2`
path is used there. The fast path also needs a sampling offset ≤ 32 and a
half period under ~40 samples (TRACECLK above ~4 MHz at 300 MS/s); below
that the per-edge path is used, and its speed is no issue at such clocks.
The startup line `edge finder: …` says which one runs.

Stage B works on bytes. It finds the full sync at byte granularity (one
compare per byte; only bytes after three 0xFF can end a sync), handles
whole 16-byte frames at once when they cannot hold a sync, and decodes ITM
per block, with a fast path for 32-bit stimulus writes. ITM is decoded only
when `-P`, `-C` or `-w` asks for it.

**Test the speed on your own PC** (Windows or Linux, no analyzer needed):

```
ch32trace -B
```

It generates a 0.5 s worst-case capture in memory (4-bit, TRACECLK 50 MHz
at 300 MS/s, continuous trace, clock jitter), decodes it with every code
path your CPU supports, checks the result, and prints MS/s and how many
times real time that is. Anything above 1× keeps up with the analyzer. On
the 2.8 GHz cloud VM (2 vCPU):

```
edge finder            ITM          MS/s   realtime   check
avx2+bmi2 (pext)       decoded      2310       7.7x   ok: 4463217 of 4463219 values, 0 breaks
avx2+bmi2 (pext)       off          2319       7.7x   1491310 frames, 0 errors
avx2                   decoded      1716       5.7x   ok: 4463214 of 4463219 values, 0 breaks
sse2                   decoded      1672       5.6x   ok: 4463214 of 4463219 values, 0 breaks
scalar                 decoded      1453       4.8x   ok: 4463214 of 4463219 values, 0 breaks
```

`bench.sh` (Linux / MSYS2) measures whole runs on files instead, including
start-up and reading the file. With the analyzer connected, the 1-second
status line also shows the live USB rate and any bytes dropped on the PC.

Measured on a 2.8 GHz Xeon cloud VM (2 vCPU), with `./bench.sh`. The figures
are whole runs from a file in the page cache, so they include startup and
calibration:

| capture | before | now |
|---|---|---|
| 4-bit, 50 MHz, `-o` | 236 MS/s | 1400 MS/s |
| 4-bit, 50 MHz, `-o -C 1` | 253 MS/s | 1194 MS/s |
| 4-bit, 60 MHz, `-o` | 243 MS/s | 1444 MS/s |
| 1-bit, 37.5 MHz, `-o` | 341 MS/s | 1559 MS/s |

Without AVX2 (`CH32TRACE_ISA=sse2`) it runs at 700–1300 MS/s. A desktop
Intel CPU should be faster still. Set `CH32TRACE_ISA=fast|avx2|sse2|scalar`
to force a path; `test.sh` checks that all four give the same result.

All paths give the same bits as the original per-edge decoder. The fast
path was checked value-for-value against a Python model of the per-edge
rule on glitch-heavy captures, for forward and backward offsets 0–20 and
1/2/4-bit. The only difference is at the very end of a capture: the fast
path also decodes the last half-finished frame.

## Build

Linux:

```sh
sudo apt install build-essential pkg-config libusb-1.0-0-dev
make
```

The default `CFLAGS` use `-march=native`. To build a binary for other
machines, use `make CFLAGS=-O2`. The AVX2/BMI2 code is still built in and
picked at run time.

On Linux, give your user access to the device (udev rule for 1a86:5538) or
run as root.

### Windows

ch32trace runs on 64-bit Windows 10/11, with the same speed paths
(AVX2/BMI2 detection works the same way).

1. **Driver:** install WinUSB for the analyzer (`1a86:5538`) with
   [Zadig](https://zadig.akeo.ie/), as for the sigrok driver / ALL LOGIC.
2. **Program:** either
   - use the prebuilt `ch32trace.exe` (static, no DLLs), or
   - cross-build it on Linux with `./build-windows.sh` (downloads llvm-mingw
     and libusb 1.0.30), or
   - build it natively in an MSYS2 **UCRT64** shell:
     `pacman -S make mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-libusb mingw-w64-ucrt-x86_64-pkgconf`,
     then `make`.
3. **libusb ≥ 1.0.30 matters on Windows.** It adds WinUSB RAW_IO, which
   ch32trace turns on so several 1 MB reads stay queued. Older libusb keeps
   only one read in flight under WinUSB, which cannot keep up with
   300 MB/s. The prebuilt .exe and `build-windows.sh` use 1.0.30. If MSYS2's
   libusb is older, use one of those instead.
4. **Orbuculum:** Windows has no FIFOs, so use the TCP output (it works on
   Linux too):

   ```bat
   ch32trace.exe -S 3450
   orbuculum -s localhost:3450 -T -t 1
   ```

   `-o file` and `-o -` (stdout, binary) work as on Linux.

What was checked: the program builds for Windows without warnings
(llvm-mingw clang 23, static, libusb 1.0.30) and is a valid PE32+ console
program. The TCP output was tested on Linux: it gives the same bytes as `-o`,
Orbuculum's TPIU decoder reads it, and clients can drop and reconnect. The
.exe itself has **not** been run on Windows yet, so please try it with
`-i` on a capture file first, then with the analyzer.

## Wiring (analyzer board)

| analyzer channel | target |
|---|---|
| D0 | TRACECLK |
| D1 – D4 | TRACEDATA0 – TRACEDATA3 (D1 – D2 for 2-bit, D1 for 1-bit) |
| D5 (any spare) | SWO, for SWO capture (`-W 5`) |
| GND | GND (several wires, next to TRACECLK) |

Keep the wires short and of equal length.

## SWO capture

SWO is one wire, so it goes on any spare analyzer channel and is decoded on
the PC from the samples, like the parallel trace. One run does either
parallel trace or SWO: the TPIU has one output, set to one or the other.

```sh
./ch32trace -W 5                      # SWO on D5, UART/NRZ, baud rate measured
./ch32trace -W 5 -b 4000000           # ... at a given baud rate
./ch32trace -W 5 -M manchester        # Manchester
./ch32trace -W 5 -S 3450              # for: orbuculum -s localhost:3450
./ch32trace -W 5 -C 1 -P 0            # built-in ITM checks, as for parallel
```

- **UART / NRZ** (`TPI->SPPR = 2`): idle high, start bit, 8 data bits,
  stop bit. Without `-b`, the bit time is measured from the first ~1M
  samples (run lengths between edges). Each byte re-syncs on its start bit,
  so a few % clock error is fine; ±3 % passed in the tests.
- **Manchester** (`TPI->SPPR = 1`): ARM's coding (idle low, packets start
  with a 1, a 1 is high then low), as in Black Magic Probe. It is
  self-clocking: the bit time is measured at the start, then followed on
  every mid-bit edge, so it needs no `-b` and tolerates clock drift.
- **Formatter:** SWO is normally the ITM stream itself (`TPI->FFCR = 0x100`,
  formatter off). Send it to Orbuculum **without** `-T`:
  `orbuculum -s localhost:3450`. If the formatter is on (TPIU frames over
  SWO, e.g. to carry ETM too), add `-F` to ch32trace and `-T -t 1` to
  Orbuculum. Then the same deframer as for parallel trace runs.
- **Speed:** at 300 MS/s, UART works from about 8 samples per bit up
  (37.5 Mbaud; 6 samples per bit passed with a given `-b`), Manchester
  from 4 samples per bit (37.5 Mbaud). For slow SWO a lower `-r` is
  fine, e.g. `-r 50` for up to ~6 Mbaud. Decoding runs at about 2.5 GS/s on
  the test VM, so it is no load.

Target setup for SWO (CMSIS; SWO pin muxing is MCU specific):

```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
TPI->CSPSR = 1;                      /* port size 1 */
TPI->SPPR  = 2;                      /* 2: NRZ/UART, 1: Manchester */
TPI->ACPR  = traceclkin_hz / swo_hz - 1;   /* NRZ; Manchester sends one bit
                                        per two prescaler clocks: use
                                        / (2 * swo_hz), check your MCU manual */
TPI->FFCR  = 0x100;                  /* formatter off: plain ITM */
/* then ITM / DWT as for parallel trace */
```

## Input level: VIO18

The analyzer has no comparators. Its inputs are UHSIF pins (PC1–PC3,
PB0–PB1, PE10–PE12 for D0–D7), and these are in the chip's **VIO18** bank,
so they switch at about **VIO18 / 2**. The "threshold" is therefore set by
setting VIO18, and `-V` takes the VIO18 voltage. Set it to the target's I/O
voltage:

| target I/O | `-V` | inputs switch at |
|---|---|---|
| 3.3 V | `3.3` (default) | ~1.65 V |
| 2.5 V | `2.5` | ~1.25 V |
| 1.8 V | `1.8` | ~0.9 V |
| 1.2 V | `1.2` | ~0.6 V |

Never set VIO18 below the target's signal swing. A 3.3 V signal into pins
supplied at 1.8 V drives the protection diodes and back-feeds the rail.

`-V` is rounded up to one of 1.2 / 1.8 / 2.5 / 3.3 V and sent as the DAC
code of `CMD_SET_LOGIC_LEVEL` (0xA1), through `vth_lut.h`. That works on
both boards:

- **Q2H2 analyzer board.** The internal VIO18 regulator is off; the DAC
  trims the external regulator that supplies VIO18.
- **nanoCH32H417, with the OpenSourceLogic-CH32H417-nano firmware**
  (`BOARD_NANO_CH32H417 = 1`). The firmware turns the same DAC code back
  into volts and sets the internal VIO18 regulator to the *nearest* level.
  ch32trace rounds up first, so a 2.0 V request becomes 2.5 V rather than
  being rounded down to 1.8 V by the firmware.

| `-V` | DAC code | nano firmware computes | nano VIO18 |
|---|---|---|---|
| 1.2 | 757 | 1162 mV | 1.2 V |
| 1.8 | 557 | 1759 mV | 1.8 V |
| 2.5 | 322 | 2461 mV | 2.5 V |
| 3.3 | 52 | 3267 mV | 3.3 V |

On the nano, R12 (VIO18 to VDDIO) must stay unfitted. VIO18 powers up at
1.8 V (XO floating) until the firmware sets 3.3 V, so connect a 3.3 V target
after the analyzer has enumerated.

Use `-c` and `-d` for other channel mappings, e.g. `-c 7 -d 0,1,2,3`.

## Target setup (CMSIS, 4-bit)

```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

/* trace pins: MCU specific. STM32F4/F7, for example:
 * DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN | (3u << DBGMCU_CR_TRACE_MODE_Pos);
 * plus PE2..PE6 as alternate function 0 (check your reference manual) */

TPI->CSPSR = 1u << (4 - 1);        /* port width 4 (bit n-1 for n bits) */
TPI->SPPR  = 0;                    /* parallel (synchronous) trace port  */
TPI->FFCR  = 0x102;                /* formatter on, continuous mode      */

DWT->CTRL |= (1u << DWT_CTRL_SYNCTAP_Pos) | DWT_CTRL_CYCCNTENA_Msk;
ITM->LAR = 0xC5ACCE55;
ITM->TCR = (1u << ITM_TCR_TRACEBUSID_Pos) | ITM_TCR_SYNCENA_Msk
         | ITM_TCR_ITMENA_Msk;
ITM->TER = 0x3;                    /* stimulus ports 0 (text) and 1      */

/* counter for the loss test */
for (uint32_t n = 0;; n++) {
	while (ITM->PORT[1].u32 == 0)
		;
	ITM->PORT[1].u32 = n;
}
```

## Use

First, check the capture with the built-in decoder. The counter test should
show `0 breaks, 0 missing`:

```sh
./ch32trace -C 1 -P 0 -t 10
```

With Orbuculum, through TCP (Linux and Windows):

```sh
./ch32trace -S 3450 &
orbuculum -s localhost:3450 -T -t 1       # -T: strip TPIU framing, -t 1: stream 1
orbcat -c 0,"%c"                           # e.g. print ITM port 0
```

`-S` listens on localhost only; add `-A` to accept connections from other
machines. One client at a time; a new one replaces the old one. Orbuculum
locks on at the next TPIU sync.

Or through a FIFO (Linux):

```sh
mkfifo /tmp/trace.fifo
./ch32trace -o /tmp/trace.fifo &
orbuculum -f /tmp/trace.fifo -T -t 1
```

Offline: capture with sigrok-cli (8 channels, `-O binary`) or `-R raw.bin`,
then decode the file with `-i raw.bin -r 300`.

Options:

```
-i FILE    raw capture input instead of the device ('-' = stdin)
-r MHZ     sample rate (300, 275, 250, 225, 200, …), default 300
-c CH      TRACECLK channel, default 0
-d LIST    TRACEDATA channels, TRACEDATA0 first, default 1,2,3,4
-p N|-N    force the sampling point (default automatic)
-V VOLT    VIO18 = target I/O voltage, default 3.3 (inputs switch at ~VOLT/2)
-o FILE    aligned TPIU byte stream (file, FIFO or '-')
-S PORT    serve the TPIU stream on TCP PORT (for orbuculum -s localhost:PORT)
-A         with -S: listen on all interfaces, not only localhost
-I ID      TPIU stream ID carrying ITM, default 1
-w FILE    deframed ITM stream bytes
-P PORT    print an ITM stimulus port as text
-C PORT    check a +1 counter on an ITM stimulus port (32-bit writes)
-R FILE    also save raw samples (needs a 300 MB/s disk)
-t SEC     stop after SEC seconds
-q         quiet: no status line every second, only the final summary
-s         silent: no status line, no summary, no info messages; only errors
           and warnings (ITM text from -P still goes to stdout)
-B         speed test (generated trace, every code path)
-W CH      SWO capture on channel CH instead of parallel trace
-M MODE    SWO coding: uart (default) or manchester
-b BAUD    SWO bit rate (default: measured)
-F         SWO stream has TPIU frames (formatter on)
```

## What was tested

There was no hardware to test with. `gen_trace.py` builds simulated
analyzer captures: TPIU frames with two interleaved stream IDs; ITM with a
counter, text, timestamps and sync packets; clock jitter and data skew;
optional sample loss. `test.sh` runs the matrix above. Also checked:

- the `-o` stream is byte-identical to the generated TPIU stream for
  1/2/4-bit;
- Orbuculum's own `tpiuDecoder.c`, compiled into a small harness, gets the
  same ITM stream from the `-o` file as ch32trace's deframer;
- waiting: 0.5 s with no clock before the trace, and a clock that stops
  for 0.1 s then comes back at a different frequency (20 → 50 MHz), with
  every edge-finder path; SWO auto-baud after 0.5 s of idle line;
- the code was run under AddressSanitizer and UBSan, with read sizes from
  37 bytes to 1 MiB and on random noise.

SWO, with `gen_trace.py --swo uart|manchester` (edge jitter, idle gaps,
baud error, random Manchester packet lengths):

- the decoded bytes are byte-identical to the generated SWO stream for
  UART at 1–50 Mbaud (6–100 samples per bit, non-integer bit times,
  ±2–3 % clock error, measured or given baud) and Manchester at
  1–37.5 Mbaud (4–50 samples per bit, +5 % clock error), at 50, 100 and
  300 MS/s, with read sizes of 37 bytes and 1 MiB, over three seeds;
- the ITM counter comes through complete; with the formatter (`-F`) the
  output is the generated TPIU stream and Orbuculum's TPIU decoder reads
  it without errors;
- sanitizers clean on SWO input, TPIU-formatted SWO, parallel-trace
  captures and noise fed to the SWO decoder.

The USB capture path follows the sigrok driver's protocol, but it has not
been run against a real analyzer yet.

## Files

- `ch32trace.c` – the tool
- `patches/libsigrok-wch-ch32h417-vio18-threshold.patch` – sigrok driver:
  threshold → VIO18 = 2 × threshold, so the default 1.65 V gives a 3.3 V
  rail
- `vth_lut.h` – VIO18 regulator DAC calibration table, from the libsigrok
  `wch-ch32h417` driver
- `gen_trace.py`, `test.sh` – simulated captures and tests
  (`gen_trace.py --glitch N` adds clock glitches)
- `bench.sh` – decode speed on simulated captures
- `build-windows.sh` – cross-build `ch32trace.exe` on Linux
- `ch32trace.exe` – prebuilt for 64-bit Windows (static)

License: GPL-3.0-or-later.
