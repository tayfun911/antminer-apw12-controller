#!/usr/bin/env python3
"""Dump the raw bits of each I2C transaction.

Useful for checking that the decoder's byte boundaries and ACK bits line up
with the waveform, and for eyeballing the clock period.

Usage:
    python i2c_bits.py
    python i2c_bits.py capture.csv bits.txt 8
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from i2c_decode import load, transitions, detect_lines  # noqa: E402


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "digital.csv")
    outpath = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "digital_bits.txt")
    ntx = int(sys.argv[3]) if len(sys.argv) > 3 else 8

    _, points = load(path)
    tr = transitions(points)
    sda_i, scl_i = detect_lines(tr)

    lines = [f"SDA=Ch{sda_i} SCL=Ch{scl_i}", ""]

    segments = []
    seg = None
    counter = [0]

    for i in range(1, len(tr)):
        t, c0, c1 = tr[i]
        sda = c0 if sda_i == 0 else c1
        scl = c1 if scl_i == 1 else c0
        _, p0, p1 = tr[i - 1]
        psda = p0 if sda_i == 0 else p1
        pscl = p1 if scl_i == 1 else p0

        if pscl == 1 and scl == 1 and psda == 1 and sda == 0:      # start
            if seg is not None:
                segments.append(seg)
            seg = {"t0": t, "bits": [], "n": counter[0]}
            counter[0] += 1
            continue
        if pscl == 1 and scl == 1 and psda == 0 and sda == 1:      # stop
            if seg is not None:
                seg["t1"] = t
                segments.append(seg)
                seg = None
            continue
        if pscl == 0 and scl == 1 and seg is not None:
            seg["bits"].append((t, sda))

    if seg is not None:
        seg["t1"] = tr[-1][0]
        segments.append(seg)

    for k, s in enumerate(segments[:ntx]):
        lines.append(f"--- segment {k}  t={s['t0']:.6f}  clock pulses={len(s['bits'])} ---")
        for g in range(0, len(s["bits"]), 9):
            grp = s["bits"][g:g + 9]
            dbits = "".join(str(b) for _, b in grp[:8])
            if len(grp) == 9:
                ack = grp[8][1]
                byte = int(dbits, 2) if len(dbits) == 8 else None
                tag = f"byte=0x{byte:02X} " + ("ACK" if ack == 0 else "NACK")
            else:
                tag = f"partial ({len(grp)} bits)"
            ts = " ".join(f"{t:.6f}:{b}" for t, b in grp)
            lines.append(f"    {tag:22} bits={dbits if dbits else '-'}  | {ts}")
        lines.append(f"    total bits: {len(s['bits'])}")
        lines.append("")

    with open(outpath, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"wrote {outpath} ({len(segments)} segments)")


if __name__ == "__main__":
    main()
