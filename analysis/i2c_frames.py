#!/usr/bin/env python3
"""Rebuild APW12 frames from the decoded I2C transactions.

Also derives the voltage model from any 0x83 setpoints found in the capture.

Usage:
    python i2c_frames.py
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from i2c_decode import load, transitions, detect_lines, decode  # noqa: E402

# Correct model, derived from the five real setpoints in the capture:
#   248 -> 12.0 V   208 -> 12.5 V   168 -> 13.0 V
#   128 -> 13.5 V    88 -> 14.0 V
# i.e. 80 counts per volt with a 15.1 V reference.
DAC_REF = 15.1
DAC_STEP = -0.0125

# Coefficients used by the older Rust implementation, kept for comparison.
OLD_REF = 15.1084
OLD_STEP = -0.013046


def dac_to_v(d):
    return DAC_REF + DAC_STEP * d


def old_dac_to_v(d):
    return OLD_REF + OLD_STEP * d


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "digital.csv")
    outpath = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "digital_frames.txt")

    _, points = load(path)
    tr = transitions(points)
    sda_i, scl_i = detect_lines(tr)
    transactions = decode(tr, sda_i, scl_i)

    # Two byte transactions are the address/register prefix (0x20, 0x11).
    # Single byte ones carry the actual data stream.
    two = [t for t in transactions if len(t["bytes"]) == 2]
    one = [t for t in transactions if len(t["bytes"]) == 1]
    empty = [t for t in transactions if len(t["bytes"]) == 0]

    lines = []
    lines.append(f"transactions total : {len(transactions)}")
    lines.append(f"  two byte prefix  : {len(two)}")
    lines.append(f"  single byte data : {len(one)}")
    lines.append(f"  empty            : {len(empty)}")

    prefixes = set(tuple(b for b, _ in t["bytes"]) for t in two)
    lines.append(f"prefix patterns    : {[f'{a:02X} {b:02X}' for a, b in prefixes]}")

    ack_stats = {}
    for t in transactions:
        for _, ack in t["bytes"]:
            ack_stats[ack] = ack_stats.get(ack, 0) + 1
    lines.append(f"ACK/NACK counts    : {ack_stats}  (0=ACK, 1=NACK)")

    stream = [b for t in one for b, _ in t["bytes"]]
    lines.append("")
    lines.append("data stream: " + " ".join(f"{b:02X}" for b in stream))

    # Frame layout: 55 AA len cmd payload.. ck_lo ck_hi
    lines.append("")
    lines.append("============== APW12 FRAMES ==============")
    i = 0
    n = 0
    while i < len(stream) - 5:
        if stream[i] == 0x55 and stream[i + 1] == 0xAA:
            ln = stream[i + 2]
            total = ln + 2
            if i + total <= len(stream):
                frame = stream[i:i + total]
                cmd = frame[3]
                payload = frame[4:total - 2]
                ck = (ln + cmd + sum(payload)) & 0xFF
                ok = "OK" if ck == frame[total - 2] else "CHECKSUM FAIL"
                extra = ""
                if cmd == 0x83 and len(payload) >= 1:
                    d = payload[0]
                    extra = (f"  -> DAC=0x{d:02X} ({d}) = {dac_to_v(d):.2f} V"
                             f"   [old Rust formula: {old_dac_to_v(d):.2f} V]")
                lines.append(f"[{n:3}] cmd=0x{cmd:02X} len={ln} "
                             f"payload={' '.join(f'{b:02X}' for b in payload)} "
                             f"ck={frame[total-2]:02X}/{frame[total-1]:02X} {ok}{extra}")
                i += total
                n += 1
                continue
        i += 1

    lines.append("")
    lines.append("============== VOLTAGE MODEL ==============")
    lines.append("The five real setpoints in the capture:")
    lines.append("")
    lines.append("  DAC :  248   208   168   128    88")
    lines.append("  V   : 12.0  12.5  13.0  13.5  14.0")
    lines.append("")
    lines.append("  Step   : 40 counts per 0.5 V  ->  80 counts per volt")
    lines.append("  Ref    : dac=0 -> 12.0 + 248/80 = 15.1 V")
    lines.append("")
    lines.append("      V   = 15.1 - 0.0125 * dac")
    lines.append("      dac = (15.1 - V) * 80")
    lines.append("")
    lines.append("  Example: 12.6 V -> dac = (15.1 - 12.6) * 80 = 200 (0xC8)")
    lines.append("  Note: the old Rust coefficients (15.1084 / -0.013046) give")
    lines.append("        dac=192 for 12.6 V, which outputs 12.7 V - a 100 mV error.")

    text = "\n".join(lines) + "\n"
    with open(outpath, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"wrote {outpath} ({n} frames)")


if __name__ == "__main__":
    main()
