#!/usr/bin/env python3
"""
Make a simulated CH32H417 analyzer capture of Cortex-M parallel trace, to
test ch32trace without hardware.

Output: raw samples, 1 byte per sample, bit k = channel k (like the
analyzer's 8-channel stream). TRACECLK on channel 0, TRACEDATA0.. on
channels 1..width.

The ITM stream (TPIU ID 1) carries a +1 counter on stimulus port 1 and a
text message on port 0, plus timestamps and sync packets. A second TPIU
stream (ID 2) is mixed in to exercise ID switching.
"""
import argparse
import numpy as np


def itm_stream(n_counter, text, rng):
    out = bytearray()
    t = text.encode()
    ti = 0
    for v in range(n_counter):
        if v % 500 == 0:
            out += b"\x00" * 5 + b"\x80"            # ITM sync packet
        out += bytes([(1 << 3) | 3]) + int(v).to_bytes(4, "little")
        if v % 7 == 0 and ti < len(t):
            out += bytes([(0 << 3) | 1, t[ti]])      # port 0, 1 byte
            ti += 1
        if v % 13 == 0:                              # local timestamp, 2 bytes
            out += bytes([0xc0, 0x80 | rng.integers(0, 127), rng.integers(0, 127)])
    return bytes(out)


def mux_streams(itm, rng):
    """Interleave ID 1 (ITM) with bursts of ID 2."""
    items = []
    i = 0
    while i < len(itm):
        n = int(rng.integers(1, 40))
        items += [(1, b) for b in itm[i:i + n]]
        i += n
        if rng.random() < 0.3:
            items += [(2, int(x)) for x in rng.integers(0, 256, int(rng.integers(1, 6)))]
    return items


def tpiu_frames(items, sync_every):
    out = bytearray(b"\xff\xff\xff\x7f")
    cur = None
    q = items
    p = 0
    nframes = 0
    while len(q) - p >= 16:
        f = bytearray(16)
        aux = 0
        for s in range(8):
            id0, b0 = q[p]
            if s == 7:
                if id0 == cur:
                    f[14] = b0 & 0xfe
                    aux |= (b0 & 1) << 7
                    p += 1
                else:
                    f[14] = (id0 << 1) | 1
                    cur = id0
                continue
            if id0 != cur:
                f[2 * s] = (id0 << 1) | 1           # new ID now (aux 0)
                f[2 * s + 1] = b0
                cur = id0
                p += 1
            elif q[p + 1][0] == cur:
                f[2 * s] = b0 & 0xfe
                aux |= (b0 & 1) << s
                f[2 * s + 1] = q[p + 1][1]
                p += 2
            else:                                    # ID change, next byte old ID
                f[2 * s] = (q[p + 1][0] << 1) | 1
                aux |= 1 << s
                f[2 * s + 1] = b0
                cur = q[p + 1][0]
                p += 1
        f[15] = aux
        out += f
        nframes += 1
        if nframes % sync_every == 0:
            out += b"\xff\xff\xff\x7f"
    return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--rate", type=float, default=300e6)
    ap.add_argument("--clk", type=float, default=50e6, help="TRACECLK in Hz")
    ap.add_argument("--width", type=int, default=4, choices=(1, 2, 4))
    ap.add_argument("--counter", type=int, default=200000)
    ap.add_argument("--sync-every", type=int, default=64, help="frames between full syncs")
    ap.add_argument("--skew", type=float, default=0.5, help="max data delay after the clock edge, samples")
    ap.add_argument("--jitter", type=float, default=0.15, help="clock edge jitter, samples (uniform +/-)")
    ap.add_argument("--center", action="store_true", help="data changes mid half period")
    ap.add_argument("--drop", type=int, default=0, help="remove this many samples mid capture")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--tpiu-out", help="also write the TPIU byte stream")
    ap.add_argument("--glitch", type=int, default=0, help="add this many 1-sample clock glitches")
    ap.add_argument("--swo", choices=("uart", "manchester"), help="make SWO instead of parallel trace")
    ap.add_argument("--baud", type=float, default=4e6, help="SWO bit rate")
    ap.add_argument("--baud-err", type=float, default=0.0, help="SWO clock error, e.g. 0.01 = +1%%")
    ap.add_argument("--swo-ch", type=int, default=5, help="SWO channel")
    ap.add_argument("--swo-tpiu", action="store_true", help="SWO carries TPIU frames (formatter on)")
    ap.add_argument("--swo-out", help="also write the SWO byte stream")
    a = ap.parse_args()

    rng = np.random.default_rng(a.seed)
    text = "Hello from ITM port 0 over 4-bit parallel trace! " * 20
    itm = itm_stream(a.counter, text, rng)
    if a.swo:
        return make_swo(a, rng, itm)
    tpiu = tpiu_frames(mux_streams(itm, rng), a.sync_every)
    # idle syncs before the trace starts
    tpiu = b"\xff\xff\xff\x7f" * 64 + tpiu
    if a.tpiu_out:
        open(a.tpiu_out, "wb").write(tpiu)

    bits = np.unpackbits(np.frombuffer(tpiu, np.uint8), bitorder="little")
    nib = np.zeros(len(bits) // a.width, np.uint8)
    for k in range(a.width):
        nib |= bits[k::a.width][:len(nib)] << k

    H = a.rate / (2 * a.clk)
    n_edges = len(nib)
    T = 50.0 + np.arange(n_edges) * H + rng.uniform(-a.jitter, a.jitter, n_edges)
    launch = T + (H / 2 if a.center else 0) + rng.uniform(0, a.skew, n_edges)
    n_samp = int(T[-1] + 3 * H)
    i = np.arange(n_samp, dtype=np.float64)

    clk = (np.searchsorted(T, i, side="right") & 1).astype(np.uint8)
    di = np.searchsorted(launch, i, side="right") - 1
    data = np.where(di >= 0, nib[np.clip(di, 0, None)], 0).astype(np.uint8)

    s = clk.copy()
    for k in range(a.width):
        s |= ((data >> k) & 1) << (k + 1)
    s |= 0x80  # an unrelated channel held high

    if a.glitch:
        pos = rng.integers(1000, len(s) - 1000, a.glitch)
        s[pos] ^= 1
    if a.drop:
        cut = n_samp // 2
        s = np.concatenate([s[:cut], s[cut + a.drop:]])

    s.tofile(a.out)
    print(f"{a.out}: {len(s)} samples, TRACECLK {a.clk/1e6:.2f} MHz, "
          f"{H:.2f} samples/half period, {len(tpiu)} TPIU bytes, "
          f"{a.counter} counter values")


def make_swo(a, rng, itm):
    """SWO line on channel a.swo_ch, everything else idle (channel 7 high)."""
    data = itm
    if a.swo_tpiu:
        data = tpiu_frames(mux_streams(itm, rng), a.sync_every)
        data = b"\xff\xff\xff\x7f" * 16 + data
    if a.swo_out:
        open(a.swo_out, "wb").write(data)
    T = a.rate / (a.baud * (1 + a.baud_err))        # samples per bit
    lv, ln = [], []                                  # levels, lengths in bits

    def seg(level, bits):
        if ln and lv[-1] == level:
            ln[-1] += bits
        else:
            lv.append(level)
            ln.append(bits)

    b = np.frombuffer(data, np.uint8)
    if a.swo == "uart":
        seg(1, 20)
        gaps = rng.choice([0, 0, 0, 0, 1, 2, 5, 30], len(b))
        for x, g in zip(b, gaps):
            seg(0, 1)                                # start
            for k in range(8):
                seg((x >> k) & 1, 1)
            seg(1, 1 + g)                            # stop + idle
        seg(1, 20)
    else:
        seg(0, 20)
        i = 0
        while i < len(b):
            n = int(rng.integers(1, 9))              # bytes in this packet
            bits = [1] + [(int(x) >> k) & 1 for x in b[i:i + n] for k in range(8)]
            for bit in bits:                         # 1: high-low, 0: low-high
                seg(bit, 0.5)
                seg(1 - bit, 0.5)
            seg(0, 1 + int(rng.choice([0, 0, 1, 3, 20])))   # stop: low
            i += n
        seg(0, 20)
    t = np.concatenate([[0.0], np.cumsum(np.array(ln) * T)])
    t[1:-1] += rng.uniform(-0.2, 0.2, len(t) - 2)   # edge jitter, samples
    n = int(t[-1])
    idx = np.searchsorted(t, np.arange(n) + 0.0, side="right") - 1
    line = np.array(lv, np.uint8)[np.clip(idx, 0, len(lv) - 1)]
    s = (line << a.swo_ch) | 0x80
    s.astype(np.uint8).tofile(a.out)
    print(f"{a.out}: {n} samples, SWO {a.swo} {a.baud * (1 + a.baud_err):.0f} baud "
          f"({T:.2f} samples/bit), {len(data)} bytes, {a.counter} counter values")


if __name__ == "__main__":
    main()
