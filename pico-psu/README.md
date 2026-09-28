# Raspberry Pi Pico target

Controls an APW12 supply from a Pico, with live status on an SSD1306 OLED.

Same idea as the ESP32-C3 version. The PSU bus is bit-banged in software at
roughly 330 Hz because hardware I2C is ignored by the supply; the OLED sits on
an ordinary 400 kHz hardware bus.

## Wiring

PSU, bit-banged:

| Pico | PSU |
|---|---|
| GP2 | SDA |
| GP3 | SCL |
| GP6 | ENABLE, active low |
| GND | GND |

OLED, hardware I2C0:

| Pico | SSD1306 |
|---|---|
| GP4 | SDA |
| GP5 | SCL |
| 3V3 | VCC |
| GND | GND |

The OLED address is `0x3C` by default; some modules ship as `0x3D`.

Add 4.7k pull-ups to 3V3 on the PSU SDA and SCL lines if the control board is
not powering that bus, and a 10k pull-up on the enable line so the output stays
off while the Pico boots. The two buses are independent, so the PSU master and
the display do not interfere.

## Building

1. Boards Manager: **Raspberry Pi Pico/RP2040** (Earle F. Philhower)
2. Library Manager: **Adafruit SSD1306** and **Adafruit GFX Library**
3. Board: **Raspberry Pi Pico**
4. Upload; hold BOOTSEL while plugging in if it does not reset into the
   bootloader by itself

Command line equivalent:

```
arduino-cli compile --fqbn rp2040:rp2040:rpipico .
arduino-cli upload  -p COM18 --fqbn rp2040:rp2040:rpipico .
```

## Display

The main readout is drawn large. Two layouts are supported, selected by the
panel height.

128x64 (0.96 inch):

```
APW12 PSU    fw15

   12.60V

SET  dac200
OUT ON    MEAS 12.58V
```

128x32 (0.91 inch):

```
12.60V        ON
SET dac200  M 12.58V
```

If the panel height is set wrong the image looks squashed or only fills half
the screen. Switch at runtime to find the right setting:

```
size 64
size 32
big 3        # larger main readout
big 0        # back to automatic sizing
```

Set it permanently with `#define OLED_IS_64 1` for a 64 row panel.

The display refreshes every five seconds. Each PSU read takes about 1.5 s at
this bit rate, so it is not a fast instrument.

`MEAS --` means no measurement is available. The supply reports zero on `0x04`
with no load attached; check the output with a meter.

## Power-up behaviour

```
[OK] PSU answered at 0x10
[OK] firmware: 15 00
[WD] watchdog disabled
[BOOT] setting 12.60 V (dac=0xC8)
[EN] enable LOW, output ON
[BOOT] output enabled at 12.60 V
```

Setpoint first, then enable, for the reason described in the top level
`README.md`: the supply boots at DAC 0, which is 15.1 V.

Configuration at the top of the sketch:

```c
#define PSU_SDA_PIN        2
#define PSU_SCL_PIN        3
#define PSU_ENABLE_PIN     6

#define OLED_SDA_PIN       4
#define OLED_SCL_PIN       5
#define OLED_ADDR          0x3C
#define OLED_IS_64         0

#define BOOT_VOLTAGE       12.6f
#define BOOT_ENABLE_OUTPUT 1
```

## Commands

Serial console at 115200:

```
help              command list
status            setpoint, state and measurement
v <volts>         set voltage, e.g. v 12.6
d <dac>           raw DAC value (0..255 or 0x..)
g / m / state     read setpoint / measure / read state
on / off          enable or disable the output
wd off | wd <v>   watchdog off, or send a ping
scan              software I2C scan
speed <hi> <lo>   bit timing in microseconds
raw <cmd> [b..]   send a raw frame
oled              redraw the display
size 32 | 64      panel height
big 1|2|3         size of the main readout
```

## Implementation note

Voltages are formatted by hand rather than with `%f`. The RP2040 toolchain does
not link the floating point printf by default, so `snprintf("%.2f")` would print
garbage. `fmtVolts()` builds the string from an integer number of millivolts.
