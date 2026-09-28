# Protocol analysis

How the APW12 protocol and the voltage model were recovered.

## What is here

| File | Purpose |
|---|---|
| `digital.csv` | the raw capture: SDA on channel 0, SCL on channel 1 |
| `i2c_decode.py` | turns the capture into I2C transactions |
| `i2c_frames.py` | rebuilds APW12 frames and fits the voltage model |
| `i2c_bits.py` | dumps raw bits per transaction, for checking the decoding |
| `digital_i2c.txt` | output of `i2c_decode.py` - 109 transactions |
| `digital_frames.txt` | output of `i2c_frames.py` - 5 frames and the model |
| `digital_bits.txt` | output of `i2c_bits.py` - bit level dump |

## The capture

Two logic analyser channels on the SDA and SCL lines while the stock controller
set the supply to 12 V, 12.5 V, 13 V, 13.5 V then 14 V. The CSV is a
transition-based export, so it only contains the samples where a line changed:
3364 rows, about 59 kB.

Channel order does not matter - the scripts find SDA from the first start
condition, where the data line falls while the clock is high.

## Running the scripts

Python 3 only, no third party packages:

```
python i2c_decode.py
python i2c_frames.py
python i2c_bits.py
```

Each reads `digital.csv` from this directory and writes a `digital_*.txt`
report next to it. A different capture can be passed as the first argument:

```
python i2c_frames.py my_capture.csv report.txt
```

## Finding 1: frame layout

The data stream recovered from the capture:

```
55 AA 06 83 F8 00 81 01 | 55 AA 06 83 D0 00 59 01 | ...
```

```
Address   0x10, write byte 0x20, read byte 0x21
Register  0x11
Transport each frame byte is sent as its own write:
          START, 0x20, 0x11, <byte>, STOP
Frame     55 AA | len | cmd | payload | ck_lo | ck_hi    (len = payload + 4)
Checksum  len + cmd + payload, 16-bit, low byte verified
```

Evidence: 140 ACKs and no NACKs anywhere in the capture, and every frame's
checksum validates (`ck=81/01`, `ck=59/01` and so on).

## Finding 2: the voltage model

Five real `set-voltage` commands (0x83) went out in the capture:

| DAC (hex) | DAC (dec) | Voltage |
|---|---|---|
| 0xF8 | 248 | 12.0 V |
| 0xD0 | 208 | 12.5 V |
| 0xA8 | 168 | 13.0 V |
| 0x80 | 128 | 13.5 V |
| 0x58 | 88 | 14.0 V |

### Fitting it

1. Consecutive setpoints differ by exactly 40 counts and 0.5 V:

   40 counts = 0.5 V, so 80 counts = 1 V

2. Step size:

   dV = 0.5 / 40 = 0.0125 V per count

3. Reference, by extrapolating back from 12.0 V at 248 counts:

   V0 = 12.0 + 248 * 0.0125 = 15.1 V

### Result

```
V   = 15.1 - 0.0125 * dac
dac = (15.1 - V) * 80
```

Check: DAC 200 gives 15.1 - 2.5 = 12.6 V, which is the value verified against a
multimeter on the ESP32-C3 and the Pico.

The whole useful range:

| Voltage | 12.0 V | 12.5 V | 13.0 V | 13.5 V | 14.0 V | 14.5 V | 15.0 V |
|---|---|---|---|---|---|---|---|
| DAC | 248 | 208 | 168 | 128 | 88 | 48 | 8 |
| Hex | `0xF8` | `0xD0` | `0xA8` | `0x80` | `0x58` | `0x30` | `0x08` |

The first five are measured. 14.5 V and 15.0 V follow from the fit. DAC 0 is
15.10 V and DAC 255 is 11.91 V, so that is the reachable range - the DAC is
8 bit and cannot go past either end.

### The older coefficients are wrong

`amlogic-cb-tools/src/protocol.rs` uses:

```rust
pub const DAC_REF_VOLTS: f32 = 15.1084;
pub const DAC_OFFSET_VOLTS_PER_COUNT: f32 = -0.013046;
```

That is 13.05 mV per count instead of 12.5 mV, and it drifts further the
further you get from the fitted range. `digital_frames.txt` prints both models
side by side:

```
[0] cmd=0x83 ... DAC=0xF8 (248) = 12.00 V   [old Rust formula: 11.87 V]
[1] cmd=0x83 ... DAC=0xD0 (208) = 12.50 V   [old Rust formula: 12.39 V]
[2] cmd=0x83 ... DAC=0xA8 (168) = 13.00 V   [old Rust formula: 12.92 V]
[3] cmd=0x83 ... DAC=0x80 (128) = 13.50 V   [old Rust formula: 13.44 V]
[4] cmd=0x83 ... DAC=0x58 (88)  = 14.00 V   [old Rust formula: 13.96 V]
```

In practice, asking the Rust tool for 12.6 V computes DAC 192, which puts
12.7 V on the output - a 100 mV error. The correct value is 200.

## Finding 3: the bus runs at 330 Hz

Clock edges from the capture:

```
5.083328, 5.086370, 5.089412, 5.092453, 5.095495, 5.098536, ...
```

The gap is constant at 3.042 ms:

f = 1 / 3.042 ms = 329 Hz

The first write transaction confirms it independently: 55.77 ms for two bytes
plus their ACKs, i.e. 18 bit periods, which is 3.1 ms per bit.

| Measurement | Value |
|---|---|
| SCL period | 3.04 ms |
| High time | 1.013 ms |
| Low time | 2.030 ms |
| Frequency | 329 Hz |

This is the reason the firmware bit-bangs the bus. Attempts and outcomes:

| Method | Result |
|---|---|
| `Wire` at 100 kHz, 50 kHz, 10 kHz | no ACK |
| SDA/SCL swapped | no ACK |
| bus recovery, nine clocks plus stop | no ACK |
| fast software bit-banging, ~100 kHz | no ACK |
| software bit-banging at ~330 Hz | ACK from 0x10 |

`DEFAULT_T_HIGH_US` and `DEFAULT_T_LOW_US` in both sketches come straight from
these numbers.

## Finding 4: the measurement command

The conversion comes from `src/protocol.rs`:

V = (raw + 0.8615) / 63.017

On real hardware with no load, `measure` (0x04) returns `adc=0x0000`. The
supply does not appear to refresh its telemetry until it is delivering current.
Use a meter, or a load, to confirm the output.

## Taking your own capture

1. Put the two analyser channels on the **SDA and SCL** lines, not on enable,
   and make sure ground is shared.
2. Sample at 1 MHz or more. The bus is slow, but the edges need to be resolved.
3. Export as a transition-based CSV.

Saleae Logic 2: Export > CSV, digital. The resulting file starts with
`Time [s],Channel 0,Channel 1`.

## What the scripts do

```
i2c_decode.py
  reads the CSV
  identifies SDA and SCL from the start condition
  finds start, repeated start, stop and rising clock edges
  assembles 9 bit groups (8 data plus ACK) into bytes
  writes digital_i2c.txt

i2c_frames.py
  takes the transaction list from i2c_decode
  collects the single byte transactions into a data stream
  extracts 55 AA frames and verifies their checksums
  converts the 0x83 DAC values with both the new and the old model
  writes digital_frames.txt, including the model derivation

i2c_bits.py
  prints the sampled bits of each transaction with timestamps
  writes digital_bits.txt, mainly for verifying ACK bits
```
