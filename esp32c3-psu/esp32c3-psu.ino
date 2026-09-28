/*
 * esp32c3-psu.ino
 *
 * APW12 power supply control for the ESP32-C3.
 *
 * The APW12's I2C bus is unusually slow: roughly 330 Hz, about 3 ms per bit.
 * Hardware I2C (100 kHz or 400 kHz) does not work - the PSU never
 * acknowledges any address, at any speed the ESP32's I2C peripheral can
 * produce. The bus is therefore driven in software at the same rate the
 * stock firmware uses.
 *
 *   Bus        : address 0x10, register 0x11, one byte per I2C transaction
 *   Frame      : 55 AA | len | cmd | payload | ck_lo | ck_hi   (len = payload+4)
 *   Checksum   : len + cmd + payload, 16-bit, low byte verified
 *   Setpoint   : command 0x83, payload [dac, 0x00]
 *                dac = (15.1 - V) * 80
 *   Measurement: command 0x04 -> V = (raw + 0.8615) / 63.017
 *
 * Wiring: SDA = GPIO4, SCL = GPIO5, ENABLE = GPIO6 (active low), common GND.
 *
 * See README.md for the hardware notes and command reference.
 */

#include <Arduino.h>
#include "psu_protocol.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

#define I2C_SDA_PIN        4
#define I2C_SCL_PIN        5
#define PSU_ENABLE_PIN     6

#define SERIAL_BAUD        115200

// Voltage written at power-up, before the output is enabled.
#define BOOT_VOLTAGE       12.6f
#define BOOT_ENABLE_OUTPUT 1       // set to 0 to leave the output off at boot

// Bus timing taken from the logic analyser capture. The PSU will not answer
// if these are shortened much.
#define DEFAULT_T_HIGH_US  1013
#define DEFAULT_T_LOW_US   2030

#define RESPONSE_DELAY_MS      600
#define RESPONSE_TIMEOUT_MS    1500
#define MAX_RESPONSE_ATTEMPTS  2
#define SETTLE_MS              2000

// ---------------------------------------------------------------------------
// Slow software I2C
// ---------------------------------------------------------------------------

static uint16_t g_tHigh = DEFAULT_T_HIGH_US;
static uint16_t g_tLow  = DEFAULT_T_LOW_US;

static inline void sdaRelease() { pinMode(I2C_SDA_PIN, INPUT_PULLUP); }
static inline void sdaLow()     { pinMode(I2C_SDA_PIN, OUTPUT); digitalWrite(I2C_SDA_PIN, LOW); }
static inline void sclRelease() { pinMode(I2C_SCL_PIN, INPUT_PULLUP); }
static inline void sclLow()     { pinMode(I2C_SCL_PIN, OUTPUT); digitalWrite(I2C_SCL_PIN, LOW); }

void bbStart() {
  sdaRelease(); sclRelease();
  delayMicroseconds(g_tLow);
  sdaLow();
  delayMicroseconds(g_tLow);
  sclLow();
  delayMicroseconds(g_tLow);
}

void bbStop() {
  sdaLow();
  delayMicroseconds(g_tLow);
  sclRelease();
  delayMicroseconds(g_tHigh);
  sdaRelease();
  delayMicroseconds(g_tLow);
}

// Returns 0 on ACK, 1 on NACK.
int bbWriteByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) sdaRelease(); else sdaLow();
    delayMicroseconds(g_tLow);
    sclRelease();
    delayMicroseconds(g_tHigh);
    sclLow();
    delayMicroseconds(g_tLow);
  }
  sdaRelease();
  delayMicroseconds(g_tLow);
  sclRelease();
  delayMicroseconds(g_tHigh);
  int ack = digitalRead(I2C_SDA_PIN);
  sclLow();
  delayMicroseconds(g_tLow);
  return ack;
}

uint8_t bbReadByte(bool sendAck) {
  uint8_t v = 0;
  sdaRelease();
  for (int i = 0; i < 8; i++) {
    sclRelease();
    delayMicroseconds(g_tHigh);
    v = (uint8_t)((v << 1) | (digitalRead(I2C_SDA_PIN) ? 1 : 0));
    sclLow();
    delayMicroseconds(g_tLow);
  }
  if (sendAck) sdaLow(); else sdaRelease();
  delayMicroseconds(g_tLow);
  sclRelease();
  delayMicroseconds(g_tHigh);
  sclLow();
  delayMicroseconds(g_tLow);
  sdaRelease();
  return v;
}

// ---------------------------------------------------------------------------
// PSU transport
// ---------------------------------------------------------------------------

// START, addr+W, 0x11, data, STOP
bool psuWriteByte(uint8_t b) {
  bbStart();
  int a1 = bbWriteByte((uint8_t)(PSU_ADDRESS << 1));
  int a2 = bbWriteByte(PSU_WRITE_REGISTER);
  int a3 = bbWriteByte(b);
  bbStop();
  return (a1 == 0 && a2 == 0 && a3 == 0);
}

// START, addr+R, byte, NACK, STOP
bool psuReadByte(uint8_t &out) {
  bbStart();
  if (bbWriteByte((uint8_t)((PSU_ADDRESS << 1) | 1)) != 0) {
    bbStop();
    return false;
  }
  out = bbReadByte(false);
  bbStop();
  return true;
}

// ---------------------------------------------------------------------------
// Frame handling
// ---------------------------------------------------------------------------

void psuSendFrame(uint8_t cmd, const uint8_t *payload, uint8_t plLen) {
  uint8_t len = (uint8_t)(plLen + 4);
  uint16_t ck = (uint16_t)len + cmd;
  for (uint8_t i = 0; i < plLen; i++) ck += payload[i];

  psuWriteByte(PREAMBLE_LSB);
  psuWriteByte(PREAMBLE_MSB);
  psuWriteByte(len);
  psuWriteByte(cmd);
  for (uint8_t i = 0; i < plLen; i++) psuWriteByte(payload[i]);
  psuWriteByte((uint8_t)(ck & 0xFF));
  psuWriteByte((uint8_t)(ck >> 8));
}

PsuResult psuReadFrame(PsuFrame &f, uint32_t timeoutMs) {
  uint8_t raw[40];
  uint32_t t0 = millis();
  uint8_t b;

  // Resynchronise on the preamble. A stray 0xF5 means the PSU rejected
  // whatever we sent last.
  while (true) {
    if (!psuReadByte(b)) {
      if (millis() - t0 > timeoutMs) return PSU_TIMEOUT;
      continue;
    }
    if (b == NAK_BYTE) return PSU_NAK;
    if (b == PREAMBLE_LSB) { raw[0] = b; break; }
    if (millis() - t0 > timeoutMs) return PSU_TIMEOUT;
  }

  if (!psuReadByte(raw[1]) || raw[1] != PREAMBLE_MSB) return PSU_BAD_FRAME;
  if (!psuReadByte(raw[2])) return PSU_BAD_FRAME;

  uint8_t len = raw[2];
  if (len < 4 || len > (uint8_t)(sizeof(raw) - 2)) return PSU_BAD_FRAME;

  uint8_t total = (uint8_t)(len + 2);
  for (uint8_t i = 3; i < total; i++) {
    if (!psuReadByte(raw[i])) return PSU_BAD_FRAME;
  }

  uint8_t plLen = (uint8_t)(total - 6);
  uint16_t ck = (uint16_t)len + raw[3];
  for (uint8_t i = 0; i < plLen; i++) ck += raw[4 + i];
  if (raw[total - 2] != (uint8_t)(ck & 0xFF)) return PSU_BAD_FRAME;

  f.cmd = raw[3];
  f.len = plLen;
  memcpy(f.payload, &raw[4], plLen);
  return PSU_OK;
}

// Sends a command and reads the answer. The PSU occasionally stays quiet
// while it applies a new setpoint, so a couple of attempts are made.
PsuResult psuExchange(uint8_t cmd, const uint8_t *payload, uint8_t plLen, PsuFrame &out) {
  psuSendFrame(cmd, payload, plLen);
  delay(RESPONSE_DELAY_MS);
  for (uint8_t a = 0; a < MAX_RESPONSE_ATTEMPTS; a++) {
    PsuResult r = psuReadFrame(out, RESPONSE_TIMEOUT_MS);
    if (r == PSU_OK && out.cmd == cmd) return PSU_OK;
    delay(RESPONSE_DELAY_MS);
  }
  return PSU_TIMEOUT;
}

// ---------------------------------------------------------------------------
// Voltage conversion
// ---------------------------------------------------------------------------

uint8_t voltageToDac(float v) {
  float code = (v - DAC_REF_VOLTS) / DAC_STEP_VOLTS + 0.5f;
  if (code < 0.0f) code = 0.0f;
  if (code > 255.0f) code = 255.0f;
  return (uint8_t)code;
}

float dacToVoltage(uint8_t d) { return DAC_REF_VOLTS + DAC_STEP_VOLTS * (float)d; }

float measuredToVoltage(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  return (raw + 0.8615f) / 63.017f;
}

// ---------------------------------------------------------------------------
// PSU operations
// ---------------------------------------------------------------------------

void psuOutputOn() {
  digitalWrite(PSU_ENABLE_PIN, LOW);
  Serial.println(F("[EN] enable LOW, output ON"));
}

void psuOutputOff() {
  digitalWrite(PSU_ENABLE_PIN, HIGH);
  Serial.println(F("[EN] enable HIGH, output OFF"));
}

bool psuSetDac(uint8_t dac, String &note) {
  uint8_t pl[2] = { dac, 0x00 };
  PsuFrame f;
  if (psuExchange(CMD_SET_VOLTAGE, pl, 2, f) == PSU_OK) {
    note = F("accepted (echo)");
    return true;
  }

  // No usable reply. Read the setpoint back instead: if it matches, the
  // command landed and the PSU was simply busy while switching.
  PsuFrame rb;
  if (psuExchange(CMD_GET_VOLTAGE, nullptr, 0, rb) == PSU_OK &&
      rb.len >= 1 && rb.payload[0] == dac) {
    note = F("accepted (verified by readback)");
    return true;
  }
  note = F("rejected");
  return false;
}

void psuGetVoltage() {
  PsuFrame f;
  if (psuExchange(CMD_GET_VOLTAGE, nullptr, 0, f) != PSU_OK || f.len < 1) {
    Serial.println(F("get-voltage: no reply"));
    return;
  }
  uint8_t dac = f.payload[0];
  Serial.printf("dac=0x%02X (%u)  estimated=%.3f V\n", dac, dac, dacToVoltage(dac));
}

void psuMeasure() {
  PsuFrame f;
  if (psuExchange(CMD_MEASURE_VOLTAGE, nullptr, 0, f) != PSU_OK || f.len < 2) {
    Serial.println(F("measure: no reply"));
    return;
  }
  uint16_t raw = (uint16_t)f.payload[0] | ((uint16_t)f.payload[1] << 8);
  Serial.printf("measured=%.3f V (adc=0x%04X)\n",
                measuredToVoltage(f.payload[0], f.payload[1]), raw);
}

void psuReadState() {
  PsuFrame f;
  if (psuExchange(CMD_READ_STATE, nullptr, 0, f) != PSU_OK || f.len < 2) {
    Serial.println(F("state: no reply"));
    return;
  }
  uint16_t st = (uint16_t)f.payload[0] | ((uint16_t)f.payload[1] << 8);
  Serial.printf("state=0x%04X (%s)\n", st, st == 1 ? "ON" : "OFF");
}

void psuWatchdog(uint8_t val) {
  uint8_t pl[2] = { val, 0x00 };
  PsuFrame f;
  if (psuExchange(CMD_WATCHDOG, pl, 2, f) == PSU_OK)
    Serial.println(val == 0 ? F("[WD] watchdog disabled") : F("[WD] watchdog ping sent"));
  else
    Serial.println(F("[WD] no reply"));
}

void cmdSetVoltage(float volts) {
  if (volts < 11.5f || volts > 15.0f) {
    Serial.printf("error: %.3f V is outside the allowed range (11.5 - 15.0 V)\n", volts);
    return;
  }
  uint8_t dac = voltageToDac(volts);
  Serial.printf("target=%.3f V -> dac=0x%02X (%u) -> model=%.3f V\n",
                volts, dac, dac, dacToVoltage(dac));
  String note;
  if (!psuSetDac(dac, note)) { Serial.println(note); return; }
  Serial.println(note);
  delay(SETTLE_MS);
  psuMeasure();
}

void cmdScan() {
  Serial.printf("software I2C scan (tHigh=%u tLow=%u us, ~%lu Hz)\n",
                g_tHigh, g_tLow,
                (unsigned long)(1000000UL / (uint32_t)(g_tHigh + g_tLow)));
  uint8_t found = 0;
  for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
    bbStart();
    int ack = bbWriteByte((uint8_t)(addr << 1));
    bbStop();
    if (ack == 0) {
      Serial.printf("  ACK from 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println(F("  nothing on the bus"));
}

void cmdRaw(uint8_t cmd, const uint8_t *pl, uint8_t n) {
  PsuFrame f;
  PsuResult r = psuExchange(cmd, pl, n, f);
  if (r != PSU_OK) { Serial.printf("raw: failed (code %d)\n", (int)r); return; }
  Serial.printf("reply cmd=0x%02X len=%u payload=", f.cmd, f.len);
  for (uint8_t i = 0; i < f.len; i++) Serial.printf("%02X ", f.payload[i]);
  Serial.println();
}

// ---------------------------------------------------------------------------
// Serial console
// ---------------------------------------------------------------------------

int splitArgs(String line, String *args, int maxArgs) {
  int n = 0, start = 0;
  while (start < (int)line.length() && n < maxArgs) {
    int sp = line.indexOf(' ', start);
    if (sp < 0) sp = line.length();
    String tok = line.substring(start, sp);
    tok.trim();
    if (tok.length() > 0) args[n++] = tok;
    start = sp + 1;
  }
  return n;
}

bool parseByte(const String &s, uint8_t &out) {
  if (s.length() == 0) return false;
  if (s.startsWith("0x") || s.startsWith("0X"))
    out = (uint8_t)strtol(s.c_str() + 2, nullptr, 16);
  else
    out = (uint8_t)strtol(s.c_str(), nullptr, 10);
  return true;
}

void printHelp() {
  Serial.println(F("commands:"));
  Serial.println(F("  help              this list"));
  Serial.println(F("  status            setpoint, state and measurement"));
  Serial.println(F("  v <volts>         set voltage, e.g. v 12.6"));
  Serial.println(F("  d <dac>           raw DAC value (0..255 or 0x..)"));
  Serial.println(F("  g / m / state     read setpoint / measure / read state"));
  Serial.println(F("  on / off          enable or disable the output"));
  Serial.println(F("  wd off | wd <v>   watchdog off, or send a ping"));
  Serial.println(F("  scan              software I2C scan"));
  Serial.println(F("  speed <hi> <lo>   bit timing in microseconds"));
  Serial.println(F("  raw <cmd> [b..]   send a raw frame"));
}

void handleCommand(String line) {
  String a[12];
  int n = splitArgs(line, a, 12);
  if (n == 0) return;
  String c = a[0]; c.toLowerCase();

  if (c == "help" || c == "?") { printHelp(); return; }
  if (c == "on")  { psuOutputOn();  return; }
  if (c == "off") { psuOutputOff(); return; }

  if (c == "v") {
    if (n < 2) { Serial.println(F("usage: v <volts>")); return; }
    cmdSetVoltage(a[1].toFloat());
    return;
  }
  if (c == "d") {
    if (n < 2) { Serial.println(F("usage: d <dac>")); return; }
    uint8_t dac;
    if (!parseByte(a[1], dac)) { Serial.println(F("bad value")); return; }
    String note;
    if (psuSetDac(dac, note)) {
      Serial.printf("dac=0x%02X (%u) = %.3f V -> %s\n", dac, dac, dacToVoltage(dac), note.c_str());
      delay(SETTLE_MS);
      psuMeasure();
    } else Serial.println(note);
    return;
  }
  if (c == "g")     { psuGetVoltage(); return; }
  if (c == "m")     { psuMeasure();    return; }
  if (c == "state") { psuReadState();  return; }
  if (c == "scan")  { cmdScan();       return; }
  if (c == "status") {
    psuGetVoltage(); psuReadState(); psuMeasure(); return;
  }
  if (c == "speed") {
    if (n < 3) { Serial.println(F("usage: speed <tHigh_us> <tLow_us>")); return; }
    g_tHigh = (uint16_t)a[1].toInt();
    g_tLow  = (uint16_t)a[2].toInt();
    Serial.printf("timing now tHigh=%u tLow=%u (~%lu Hz)\n", g_tHigh, g_tLow,
                  (unsigned long)(1000000UL / (uint32_t)(g_tHigh + g_tLow)));
    return;
  }
  if (c == "wd") {
    uint8_t val = 0;
    if (n >= 2 && !a[1].equalsIgnoreCase("off")) parseByte(a[1], val);
    psuWatchdog(val);
    return;
  }
  if (c == "raw") {
    if (n < 2) { Serial.println(F("usage: raw <cmd> [bytes..]")); return; }
    uint8_t cmd, pl[18], plLen = 0;
    parseByte(a[1], cmd);
    for (int i = 2; i < n && plLen < sizeof(pl); i++) parseByte(a[i], pl[plLen++]);
    cmdRaw(cmd, pl, plLen);
    return;
  }
  Serial.println(F("unknown command, try 'help'"));
}

// ---------------------------------------------------------------------------
// Setup and loop
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  // Keep the output disabled until a setpoint has been accepted.
  pinMode(PSU_ENABLE_PIN, OUTPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);

  sdaRelease();
  sclRelease();
  delay(50);

  Serial.println();
  Serial.println(F("APW12 PSU controller (ESP32-C3)"));
  Serial.printf ("PSU : SDA=GPIO%d SCL=GPIO%d EN=GPIO%d, active low\n",
                 I2C_SDA_PIN, I2C_SCL_PIN, PSU_ENABLE_PIN);
  Serial.printf ("bus : address 0x%02X, register 0x%02X, tHigh=%u tLow=%u us (~%lu Hz)\n",
                 PSU_ADDRESS, PSU_WRITE_REGISTER, g_tHigh, g_tLow,
                 (unsigned long)(1000000UL / (uint32_t)(g_tHigh + g_tLow)));

  bbStart();
  int ack = bbWriteByte((uint8_t)(PSU_ADDRESS << 1));
  bbStop();

  if (ack == 0) {
    Serial.println(F("[OK] PSU answered at 0x10"));

    PsuFrame f;
    if (psuExchange(CMD_GET_FW_VERSION, nullptr, 0, f) == PSU_OK && f.len >= 1) {
      Serial.print(F("[OK] firmware: "));
      for (uint8_t i = 0; i < f.len; i++) Serial.printf("%02X ", f.payload[i]);
      Serial.println();
    } else {
      Serial.println(F("[!!] no reply to get-firmware"));
    }

    psuWatchdog(0x00);

    // Set the voltage before enabling. The PSU powers up with DAC 0, which
    // is 15.1 V on this model, so enabling first would briefly expose the
    // load to that.
    uint8_t dac = voltageToDac(BOOT_VOLTAGE);
    Serial.printf("[BOOT] setting %.2f V (dac=0x%02X)\n", BOOT_VOLTAGE, dac);
    String note;
    if (psuSetDac(dac, note)) {
      Serial.printf("[BOOT] setpoint %s\n", note.c_str());
      delay(300);
#if BOOT_ENABLE_OUTPUT
      psuOutputOn();
      Serial.printf("[BOOT] output enabled at %.2f V\n", BOOT_VOLTAGE);
#else
      Serial.println(F("[BOOT] setpoint ready, output left off"));
#endif
    } else {
      Serial.println(F("[BOOT] setpoint rejected, output stays off"));
    }
  } else {
    Serial.println(F("[!!] no ACK from the PSU address"));
    Serial.println(F("     run 'scan' and check the wiring, pull-ups and bus owner"));
  }

  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleCommand(line);
  }
}
