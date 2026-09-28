#pragma once
#include <stdint.h>
#include <string.h>

// APW12 protocol definitions.
//
// Derived from a logic analyser capture of the stock controller talking to the
// supply (12 V through 14 V in 0.5 V steps). See analysis/README.md for the
// decoding scripts and the derivation.

#define PSU_ADDRESS            0x10
#define PSU_WRITE_REGISTER     0x11

#define CMD_GET_FW_VERSION     0x01
#define CMD_GET_HW_VERSION     0x02
#define CMD_GET_VOLTAGE        0x03
#define CMD_MEASURE_VOLTAGE    0x04
#define CMD_READ_STATE         0x05
#define CMD_READ_CAL           0x06
#define CMD_WATCHDOG           0x81
#define CMD_SET_VOLTAGE        0x83

#define PREAMBLE_LSB           0x55
#define PREAMBLE_MSB           0xAA
#define NAK_BYTE               0xF5

// Voltage model, fitted to five real setpoints from the capture:
//   248 -> 12.0 V   208 -> 12.5 V   168 -> 13.0 V
//   128 -> 13.5 V    88 -> 14.0 V
// i.e. 80 counts per volt with a 15.1 V reference.
#define DAC_REF_VOLTS          15.1f
#define DAC_STEP_VOLTS        -0.0125f

struct PsuFrame {
  uint8_t cmd;
  uint8_t payload[32];
  uint8_t len;
};

enum PsuResult { PSU_OK = 0, PSU_NAK, PSU_TIMEOUT, PSU_BAD_FRAME };
