# ESP32-C3 target

Controls an APW12 supply from an ESP32-C3 over a software I2C bus, and exposes
a serial console for setting the voltage and reading telemetry.

The PSU side has to be bit-banged. Hardware I2C does not work - the supply's bus
runs at roughly 330 Hz and never acknowledges anything faster. The reasoning and
the measurements are in the top level `README.md` and in `../analysis/`.

## Wiring

| ESP32-C3 | PSU |
|---|---|
| GPIO4 | SDA |
| GPIO5 | SCL |
| GPIO6 | ENABLE, active low |
| GND | GND |

Both sides are 3.3 V. Add 4.7k pull-ups to 3V3 on SDA and SCL if the control
board is not powering the bus, and a 10k pull-up on the enable line so the
output stays off while the chip boots. Only one master per bus.

## Building

In the Arduino IDE:

1. Boards Manager: install **esp32** (Espressif)
2. Board: **ESP32C3 Dev Module**
3. **Tools > USB CDC On Boot > Enabled**
4. Upload

The USB CDC setting is what makes `Serial` appear on the USB port. Left
disabled, the chip sends its output to the UART pins and the serial monitor
shows nothing at all.

Command line equivalent:

```
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc .
arduino-cli upload  -p COM10 --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc .
```

## Power-up behaviour

```
[OK] PSU answered at 0x10
[OK] firmware: 15 00
[WD] watchdog disabled
[BOOT] setting 12.60 V (dac=0xC8)
[BOOT] setpoint accepted (echo)
[EN] enable LOW, output ON
[BOOT] output enabled at 12.60 V
```

The setpoint is written before enable is asserted. The supply powers up at
DAC 0, which is 15.1 V on this model, so enabling first would briefly put that
on the load. If the setpoint is rejected the output is left off.

Configuration lives at the top of the sketch:

```c
#define I2C_SDA_PIN        4
#define I2C_SCL_PIN        5
#define PSU_ENABLE_PIN     6

#define BOOT_VOLTAGE       12.6f
#define BOOT_ENABLE_OUTPUT 1

#define DEFAULT_T_HIGH_US  1013
#define DEFAULT_T_LOW_US   2030
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
```

`v` accepts 11.5 V to 15.0 V. Anything outside that is refused.

## Notes

- `speed 500 1000` is worth trying once the link works. Faster timings may
  still be acknowledged, and a shorter frame means a much more responsive
  console.
- Resetting the board drops the output briefly while the firmware runs through
  its startup sequence - a few seconds, since each PSU command takes about
  1.5 s at this bit rate.
- `0x04` reports zero with no load connected. The supply does not update its
  telemetry until it is delivering current.
