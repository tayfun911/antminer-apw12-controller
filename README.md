# APW12 PSU Controller

Standalone firmware for controlling an Antminer APW12 power supply over I2C,
without the stock control board or LuxOS running.

Two targets are provided:

| Target | Status | Directory |
|---|---|---|
| ESP32-C3 | tested on hardware | `esp32c3-psu/` |
| Raspberry Pi Pico | tested on hardware, with OLED | `pico-psu/` |

Both set a 12.6 V output at power-up, expose a serial console for changing the
voltage and reading telemetry, and drive the PSU's enable line.

---

## Background

The APW12 normally hangs off the miner's control board. The control board talks
to it over a dedicated I2C bus (`/dev/i2c-1` on the Amlogic board), address
`0x10`, register `0x11`. This project replaces the control board with a small
microcontroller so the supply can be used on its own.

The protocol was recovered from a logic analyser capture of the stock
controller setting 12 V through 14 V. The capture, the decoding scripts and the
derivation are in `analysis/`.

## The one thing that matters: bus speed

The APW12's I2C bus runs at about **330 Hz** - roughly 3 ms per bit, with SCL
high for 1.013 ms and low for 2.030 ms. This is measured directly from the
capture and is not a misreading: a two byte transaction takes 55.8 ms, which
matches 18 bit periods at 3.04 ms.

Hardware I2C does not work here. At 100 kHz, 50 kHz or even 10 kHz the supply
never acknowledges its own address. The following were all tried and all
failed:

- `Wire` at 100 kHz, 50 kHz and 10 kHz
- SDA/SCL swapped
- bus recovery (nine clock pulses followed by a stop)
- fast software bit-banging at roughly 100 kHz

Dropping the bit rate to the speed the stock controller uses works immediately:

```
software I2C scan (tHigh=1013 tLow=2030 us, ~328 Hz)
  ACK from 0x10
```

So both firmwares drive the PSU bus in software, with configurable delays.
The OLED on the Pico is a separate, ordinary 400 kHz hardware bus.

## Voltage model

Five real setpoints were recovered from the capture:

| Voltage | 12.0 V | 12.5 V | 13.0 V | 13.5 V | 14.0 V |
|---|---|---|---|---|---|
| DAC | 248 | 208 | 168 | 128 | 88 |
| Hex | `0xF8` | `0xD0` | `0xA8` | `0x80` | `0x58` |

Each step is exactly 40 counts for 0.5 V, so 80 counts per volt, and the
reference works out to 15.1 V at DAC 0:

```
V   = 15.1 - 0.0125 * dac
dac = (15.1 - V) * 80
```

Values for the full useful range:

| Voltage | 12.0 V | 12.5 V | 13.0 V | 13.5 V | 14.0 V | 14.5 V | 15.0 V |
|---|---|---|---|---|---|---|---|
| DAC | 248 | 208 | 168 | 128 | 88 | 48 | 8 |
| Hex | `0xF8` | `0xD0` | `0xA8` | `0x80` | `0x58` | `0x30` | `0x08` |

The first five are measured; 14.5 V and 15.0 V are calculated from the fit.
`12.6 V -> 200 (0xC8)` was verified against a multimeter on both boards and the
output read exactly 12.600 V.

Achievable range is roughly **11.91 V to 15.10 V**. DAC 0, the value the supply
boots with, is 15.1 V - worth knowing before enabling the output.

Note that the Rust implementation in `amlogic-cb-tools` uses `15.1084` and
`-0.013046`. Those coefficients are slightly off: they compute DAC 192 for a
12.6 V target, which produces 12.7 V at the terminals, a 100 mV error.

## Protocol

```
Address   0x10, write byte 0x20, read byte 0x21
Register  0x11
Transport each frame byte is sent as its own write:
          START, 0x20, 0x11, <byte>, STOP
Frame     55 AA | len | cmd | payload | ck_lo | ck_hi    (len = payload + 4)
Checksum  len + cmd + payload, 16-bit, low byte verified
NAK       0xF5
```

| Command | Code | Reply |
|---|---|---|
| get firmware | `0x01` | `[0x15, 0x00]` |
| get hardware | `0x02` | version |
| get voltage | `0x03` | `[dac]` |
| measure | `0x04` | `[adc_lo, adc_hi]` |
| read state | `0x05` | `[lo, hi]`, 1 = on |
| read calibration | `0x06` | calibration page |
| watchdog | `0x81` | `[0x00, 0x00]` disables it |
| set voltage | `0x83` | `[dac, 0x00]` |

Measurement conversion: `V = (raw + 0.8615) / 63.017`.

One caveat: `0x04` returns zero with no load attached. The supply does not
appear to update its telemetry until it is actually delivering current, so
verify the output with a meter or a real load.

## Wiring

All of this lives on the connector between the supply and the control board:
SDA, SCL, ENABLE and ground. The enable line is **active low** - pulling it to
ground turns the output on.

ESP32-C3:

| ESP32-C3 | PSU |
|---|---|
| GPIO4 | SDA |
| GPIO5 | SCL |
| GPIO6 | ENABLE |
| GND | GND |

Pico:

| Pico | PSU | | Pico | OLED |
|---|---|---|---|---|
| GP2 | SDA | | GP4 | SDA |
| GP3 | SCL | | GP5 | SCL |
| GP6 | ENABLE | | 3V3 | VCC |
| GND | GND | | GND | GND |

Things that will otherwise waste an afternoon:

- Both boards are 3.3 V, which matches the PSU bus. A 5 V Arduino will not do.
- Ground must be common.
- The bus needs pull-ups. If the control board is unpowered its pull-ups are
  dead, so add 4.7k from each line to 3V3.
- A 10k pull-up on the enable line keeps the output off while the
  microcontroller boots.
- One master per bus. Do not leave both boards connected, and do not run the
  stock controller's PSU tools at the same time.

## Power-up behaviour

On reset the firmware:

1. probes address `0x10`
2. reads the firmware version
3. disables the watchdog
4. writes the setpoint and checks it was accepted
5. only then asserts enable

The setpoint goes first because the supply boots at DAC 0, i.e. 15.1 V.
Enabling before writing the setpoint would briefly put 15.1 V on the load. If
the setpoint is rejected the output stays off.

Change the boot voltage in the sketch:

```c
#define BOOT_VOLTAGE       12.6f
#define BOOT_ENABLE_OUTPUT 1      // 0 to leave the output off at power-up
```

## Building

### ESP32-C3

1. Arduino IDE, Boards Manager, install **esp32** (Espressif)
2. Board: **ESP32C3 Dev Module**
3. **Tools > USB CDC On Boot > Enabled** - without this the serial output goes
   to the UART pins and the monitor stays empty
4. Open `esp32c3-psu/esp32c3-psu.ino` and upload
5. Serial monitor at 115200

With arduino-cli: `--fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc`

### Pico

1. Boards Manager: **Raspberry Pi Pico/RP2040** (Earle F. Philhower)
2. Library Manager: **Adafruit SSD1306** and **Adafruit GFX Library**
3. Board: **Raspberry Pi Pico**
4. Open `pico-psu/pico-psu.ino` and upload; hold BOOTSEL if it does not enter
   the bootloader on its own
5. Serial monitor at 115200

Each sketch must stay in a folder matching its filename - that is an Arduino
requirement, not a suggestion.

## Commands

```
help              command list
status            setpoint, state and measurement
v <volts>         set voltage, e.g. v 12.6
d <dac>           raw DAC value (0..255 or 0x..)
g / m / state     read setpoint / measure / read state
on / off          enable or disable the output
wd off | wd <v>   watchdog off, or send a ping
scan              software I2C scan
speed <hi> <lo>   bit timing in microseconds, for finding the usable rate
raw <cmd> [b..]   send a raw frame
```

Pico only:

```
oled              redraw the display
size 32 | 64      panel height
big 1|2|3         size of the main readout
```

Typical session:

```
v 12.6
on
status
off
```

## Troubleshooting

| Symptom | Cause |
|---|---|
| No ACK from the PSU address | bit rate too high, SDA/SCL swapped, no ground, no pull-ups, or another master on the bus |
| Address ACKs but no reply to get-firmware | frame rejected; slow the bus down |
| State stays OFF | `on` was not issued, or the enable line is not connected |
| Voltage does not change | output is off |
| Measurement reads zero | no load attached; check with a meter |
| Supply shuts down after a while | watchdog is running; send `wd off` |
| No serial output on ESP32-C3 | USB CDC On Boot is not enabled |
| OLED blank | try `OLED_ADDR 0x3D`, check wiring, or try `size 32` / `size 64` |
| OLED text cramped or half-height | wrong panel height |

## Safety

The APW12 delivers 12-15 V at tens of amps. Do the wiring with the supply
disconnected from mains, and stay away from the high current output. Keep the
target within the hashboard's limits - the firmware clamps input to
11.5-15.0 V. The voltage model is empirical; verify against a meter before
trusting it.

## Credits

The protocol work builds on the APW12 reverse engineering already done for the
`amlogic-cb-tools` project, and on
[Hashsource](https://github.com/HashSource/Antminer-APW12-Firmware) for APW12
firmware dumps. The coefficients and bus timing used here were re-derived from
a fresh capture and verified on hardware.

## License

MIT. See `LICENSE`.
