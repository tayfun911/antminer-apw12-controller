#!/usr/bin/env python3
"""Decode a Saleae digital export into I2C transactions.

Usage:
    python i2c_decode.py
    python i2c_decode.py capture.csv report.txt

The CSV must have a time column followed by two channel columns. SDA and SCL
are identified automatically from the start condition, so the channel order
does not matter.
"""
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def load(path):
    points = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if len(row) < 3:
                continue
            try:
                points.append((float(row[0]), int(row[1]), int(row[2])))
            except ValueError:
                continue
    return header, points


def transitions(points):
    """Keep only the samples where something changed."""
    out = [points[0]]
    for p in points[1:]:
        if p[1] != out[-1][1] or p[2] != out[-1][2]:
            out.append(p)
    return out


def detect_lines(tr):
    """Work out which channel is SDA from the first start condition."""
    for i in range(1, len(tr)):
        _, p0, p1 = tr[i - 1]
        _, c0, c1 = tr[i]
        # Data falls while the clock is high: that is a start.
        if p0 == 1 and c0 == 0 and p1 == 1 and c1 == 1:
            return 0, 1
        if p1 == 1 and c1 == 0 and p0 == 1 and c0 == 1:
            return 1, 0
    return 0, 1


def decode(tr, sda_i, scl_i):
    transactions = []
    txn = None
    bits = []
    ack_pending = False
    pending = None

    for i in range(1, len(tr)):
        t, c0, c1 = tr[i]
        sda = c0 if sda_i == 0 else c1
        scl = c1 if scl_i == 1 else c0
        _, p0, p1 = tr[i - 1]
        psda = p0 if sda_i == 0 else p1
        pscl = p1 if scl_i == 1 else p0

        # Start or repeated start
        if pscl == 1 and scl == 1 and psda == 1 and sda == 0:
            if txn is not None:
                txn["end"] = t
                transactions.append(txn)
            txn = {"start": t, "bytes": [], "end": None, "repeated": bool(transactions)}
            bits = []
            ack_pending = False
            continue

        # Stop
        if pscl == 1 and scl == 1 and psda == 0 and sda == 1:
            if txn is not None:
                txn["end"] = t
                transactions.append(txn)
                txn = None
            bits = []
            ack_pending = False
            continue

        # Sample on the rising clock edge
        if pscl == 0 and scl == 1:
            if txn is None:
                continue
            if not ack_pending:
                bits.append(sda)
                if len(bits) == 8:
                    pending = int("".join(str(b) for b in bits), 2)
                    bits = []
                    ack_pending = True
            else:
                txn["bytes"].append((pending, sda))
                ack_pending = False

    if txn is not None:
        txn["end"] = tr[-1][0]
        transactions.append(txn)
    return transactions


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "digital.csv")
    outpath = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "digital_i2c.txt")

    header, points = load(path)
    tr = transitions(points)
    sda_i, scl_i = detect_lines(tr)
    transactions = decode(tr, sda_i, scl_i)

    lines = []
    lines.append(f"file       : {path}")
    lines.append(f"header     : {header}")
    lines.append(f"samples    : {len(points)}  (transitions: {len(tr)})")
    lines.append(f"SDA=Ch{sda_i}  SCL=Ch{scl_i}")
    lines.append(f"transactions: {len(transactions)}")
    if transactions and transactions[0]["end"]:
        span = transactions[0]["end"] - transactions[0]["start"]
        nbits = sum(9 * len(t["bytes"]) for t in transactions if t is transactions[0])
        if nbits and span > 0:
            lines.append(f"first transaction: {span*1000:.2f} ms  -> SCL ~{nbits/span:.0f} Hz")
    lines.append("")
    lines.append("================ TRANSACTIONS ================")

    for n, t in enumerate(transactions):
        dur = (t["end"] - t["start"]) * 1000.0 if t["end"] else 0.0
        tag = "  (repeated start)" if t["repeated"] else ""
        lines.append("")
        lines.append(f"[{n}] start={t['start']:.6f}s  duration={dur:.2f} ms{tag}")
        raw = []
        for j, (b, ack) in enumerate(t["bytes"]):
            lines.append(f"    byte{j}: 0x{b:02X}  {'ACK ' if ack == 0 else 'NACK'}")
            raw.append(b)
        lines.append("    raw: " + " ".join(f"{b:02X}" for b in raw))

    text = "\n".join(lines) + "\n"
    with open(outpath, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"wrote {outpath} ({len(transactions)} transactions)")


if __name__ == "__main__":
    main()
