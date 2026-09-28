/*
 * pico-psu.ino
 *
 * APW12 power supply control for the Raspberry Pi Pico, with an SSD1306 OLED
 * showing live status.
 *
 * Same approach as the ESP32-C3 version. The one thing that matters: the
 * APW12's I2C bus runs at roughly 330 Hz (~3 ms per bit) and does not answer
 * to hardware I2C at any usable speed. The PSU side is therefore bit-banged in
 * software at the same rate the stock firmware uses.
 *
 *   PSU   : GP2 (SDA), GP3 (SCL), GP6 (ENABLE, active low)
 *   OLED  : GP4 (SDA), GP5 (SCL) on hardware I2C0, address 0x3C
 *
 * At power-up the sketch writes a 12.6 V setpoint and then enables the output.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "psu_protocol.h"

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

// PSU side, bit-banged
#define PSU_SDA_PIN        2
#define PSU_SCL_PIN        3
#define PSU_ENABLE_PIN     6

// OLED on hardware I2C0
#define OLED_SDA_PIN       4
#define OLED_SCL_PIN       5
#define OLED_ADDR          0x3C
// 0 for a 128x32 panel (0.91"), 1 for 128x64 (0.96"). Can also be switched at
// runtime with 'size 64' / 'size 32'.
#define OLED_IS_64         0

#define SERIAL_BAUD        115200

#define BOOT_VOLTAGE       12.6f
#define BOOT_ENABLE_OUTPUT 1       // set to 0 to leave the output off at boot

// Bus timing taken from the logic analyser capture.
#define DEFAULT_T_HIGH_US  1013
#define DEFAULT_T_LOW_US   2030

#define RESPONSE_DELAY_MS      600
#define RESPONSE_TIMEOUT_MS    1500
#define MAX_RESPONSE_ATTEMPTS  2
#define SETTLE_MS              2000
#define STATUS_PERIOD_MS       5000

// Two display objects; the active one is picked by oledBegin().
Adafruit_SSD1306 disp32(128, 32, &Wire, -1);
Adafruit_SSD1306 disp64(128, 64, &Wire, -1);
static bool g_is64 = OLED_IS_64;
static uint8_t g_bigSize = 0;      // 0 = auto (24px on 64-row panels, 16px on 32)
static Adafruit_SSD1306 *D = &disp32;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool    g_oledOk   = false;
static bool    g_psuOk    = false;
static bool    g_outputOn = false;
static uint8_t g_dac      = 0;
static float   g_setV     = 0.0f;
static bool    g_stateOn  = false;
static float   g_measV    = 0.0f;
static bool    g_haveMeas = false;
static uint8_t g_fw0      = 0;
static uint32_t g_lastStatus = 0;

static uint16_t g_tHigh = DEFAULT_T_HIGH_US;
static uint16_t g_tLow  = DEFAULT_T_LOW_US;

// ---------------------------------------------------------------------------
// Slow software I2C
// ---------------------------------------------------------------------------

static inline void sdaRelease() { pinMode(PSU_SDA_PIN, INPUT_PULLUP); }
static inline void sdaLow()     { pinMode(PSU_SDA_PIN, OUTPUT); digitalWrite(PSU_SDA_PIN, LOW); }
static inline void sclRelease() { pinMode(PSU_SCL_PIN, INPUT_PULLUP); }
static inline void sclLow()     { pinMode(PSU_SCL_PIN, OUTPUT); digitalWrite(PSU_SCL_PIN, LOW); }

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
  int ack = digitalRead(PSU_SDA_PIN);
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
    v = (uint8_t)((v << 1) | (digitalRead(PSU_SDA_PIN) ? 1 : 0));
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

bool psuWriteByte(uint8_t b) {
  bbStart();
  int a1 = bbWriteByte((uint8_t)(PSU_ADDRESS << 1));
  int a2 = bbWriteByte(PSU_WRITE_REGISTER);
  int a3 = bbWriteByte(b);
  bbStop();
  return (a1 == 0 && a2 == 0 && a3 == 0);
}

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

bool psuProbe() {
  bbStart();
  int ack = bbWriteByte((uint8_t)(PSU_ADDRESS << 1));
  bbStop();
  return (ack == 0);
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

// Formats a voltage without relying on %f. The RP2040 toolchain does not link
// the floating point printf by default, so "12.60" is built by hand.
void fmtVolts(char *out, size_t n, float v) {
  int mv = (int)(v * 1000.0f + (v >= 0.0f ? 0.5f : -0.5f));
  int a = mv / 1000;
  int b = mv % 1000;
  if (b < 0) b = -b;
  snprintf(out, n, "%d.%02d", a, b / 10);
}

// ---------------------------------------------------------------------------
// OLED
// ---------------------------------------------------------------------------

bool oledBegin() {
  D = g_is64 ? &disp64 : &disp32;
  bool ok = D->begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  if (ok) {
    D->clearDisplay();
    D->setTextColor(SSD1306_WHITE);
    D->display();
  }
  return ok;
}

void oledRedraw() {
  if (!g_oledOk || D == nullptr) return;
  const int H = D->height();
  const uint8_t big = g_bigSize ? g_bigSize : ((H >= 64) ? 3 : 2);

  D->clearDisplay();
  D->setTextColor(SSD1306_WHITE);

  if (!g_psuOk) {
    D->setTextSize(2);
    D->setCursor(0, (H >= 64) ? 10 : 0);
    D->print("NO PSU");
    D->setTextSize(1);
    D->setCursor(0, (H >= 64) ? 38 : 20);
    D->print("check wiring / run scan");
    D->display();
    return;
  }

  char n[12], vs[14], vm[14], line[26];
  fmtVolts(n, sizeof(n), g_setV);
  snprintf(vs, sizeof(vs), "%sV", n);

  if (g_haveMeas) {
    fmtVolts(n, sizeof(n), g_measV);
    snprintf(vm, sizeof(vm), "%sV", n);
  } else {
    strcpy(vm, "--");
  }

  if (H >= 64) {
    D->setTextSize(1);
    D->setCursor(0, 0);
    snprintf(line, sizeof(line), "APW12 PSU    fw%02X", g_fw0);
    D->print(line);

    D->setTextSize(big);
    D->setCursor(0, 12);
    D->print(vs);

    D->setTextSize(1);
    D->setCursor(0, 42);
    snprintf(line, sizeof(line), "SET  dac%u", g_dac);
    D->print(line);

    D->setCursor(0, 54);
    snprintf(line, sizeof(line), "OUT %s    MEAS %s", g_stateOn ? "ON" : "OFF", vm);
    D->print(line);
  } else {
    D->setTextSize(big);
    D->setCursor(0, 0);
    D->print(vs);

    D->setTextSize(1);
    D->setCursor(98, 4);
    D->print(g_stateOn ? "ON" : "OFF");

    D->setCursor(0, 20);
    snprintf(line, sizeof(line), "SET dac%u  M %s", g_dac, vm);
    D->print(line);
  }

  D->display();
}

// ---------------------------------------------------------------------------
// PSU operations
// ---------------------------------------------------------------------------

void psuOutputOn() {
  digitalWrite(PSU_ENABLE_PIN, LOW);
  g_outputOn = true;
  Serial.println(F("[EN] enable LOW, output ON"));
}

void psuOutputOff() {
  digitalWrite(PSU_ENABLE_PIN, HIGH);
  g_outputOn = false;
  Serial.println(F("[EN] enable HIGH, output OFF"));
}

bool psuSetDac(uint8_t dac, String &note) {
  uint8_t pl[2] = { dac, 0x00 };
  PsuFrame f;
  if (psuExchange(CMD_SET_VOLTAGE, pl, 2, f) == PSU_OK) {
    note = F("accepted (echo)");
    g_dac = dac;
    g_setV = dacToVoltage(dac);
    return true;
  }
  PsuFrame rb;
  if (psuExchange(CMD_GET_VOLTAGE, nullptr, 0, rb) == PSU_OK &&
      rb.len >= 1 && rb.payload[0] == dac) {
    note = F("accepted (verified by readback)");
    g_dac = dac;
    g_setV = dacToVoltage(dac);
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
  g_dac = f.payload[0];
  g_setV = dacToVoltage(g_dac);
  char v[12];
  fmtVolts(v, sizeof(v), g_setV);
  Serial.printf("dac=0x%02X (%u)  estimated=%s V\n", g_dac, g_dac, v);
}

void psuMeasure() {
  PsuFrame f;
  if (psuExchange(CMD_MEASURE_VOLTAGE, nullptr, 0, f) != PSU_OK || f.len < 2) {
    Serial.println(F("measure: no reply"));
    return;
  }
  uint16_t raw = (uint16_t)f.payload[0] | ((uint16_t)f.payload[1] << 8);
  g_measV = measuredToVoltage(f.payload[0], f.payload[1]);
  g_haveMeas = true;
  char v[12];
  fmtVolts(v, sizeof(v), g_measV);
  Serial.printf("measured=%s V (adc=0x%04X)\n", v, raw);
}

void psuReadState() {
  PsuFrame f;
  if (psuExchange(CMD_READ_STATE, nullptr, 0, f) != PSU_OK || f.len < 2) {
    Serial.println(F("state: no reply"));
    return;
  }
  uint16_t st = (uint16_t)f.payload[0] | ((uint16_t)f.payload[1] << 8);
  g_stateOn = (st == 1);
  Serial.printf("state=0x%04X (%s)\n", st, g_stateOn ? "ON" : "OFF");
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
    Serial.println(F("error: outside the allowed range (11.5 - 15.0 V)"));
    return;
  }
  uint8_t dac = voltageToDac(volts);
  char v[12];
  fmtVolts(v, sizeof(v), volts);
  Serial.printf("target=%s V -> dac=0x%02X (%u)\n", v, dac, dac);
  String note;
  if (!psuSetDac(dac, note)) { Serial.println(note); oledRedraw(); return; }
  Serial.println(note);
  delay(SETTLE_MS);
  psuMeasure();
  oledRedraw();
}

void cmdScan() {
  Serial.printf("software I2C scan (tHigh=%u tLow=%u us)\n", g_tHigh, g_tLow);
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
  Serial.println(F("  oled              redraw the display"));
  Serial.println(F("  size 32 | 64      panel height"));
  Serial.println(F("  big 1|2|3         size of the main readout"));
}

void handleCommand(String line) {
  String a[12];
  int n = splitArgs(line, a, 12);
  if (n == 0) return;
  String c = a[0]; c.toLowerCase();

  if (c == "help" || c == "?") { printHelp(); return; }
  if (c == "on")  { psuOutputOn();  oledRedraw(); return; }
  if (c == "off") { psuOutputOff(); oledRedraw(); return; }
  if (c == "oled") { oledRedraw(); return; }

  if (c == "size") {
    if (n < 2) { Serial.println(F("usage: size 32 | 64")); return; }
    bool want64 = (a[1].toInt() == 64);
    if (want64 != g_is64) {
      g_is64 = want64;
      g_oledOk = oledBegin();
    }
    Serial.printf("panel height %d -> %s\n", g_is64 ? 64 : 32,
                  g_oledOk ? "ok" : "not found");
    oledRedraw();
    return;
  }

  if (c == "big") {
    if (n < 2) { Serial.println(F("usage: big 1 | 2 | 3")); return; }
    g_bigSize = (uint8_t)constrain(a[1].toInt(), 1, 3);
    Serial.printf("readout size %u\n", g_bigSize);
    oledRedraw();
    return;
  }

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
      Serial.printf("dac=0x%02X -> %s\n", dac, note.c_str());
      delay(SETTLE_MS);
      psuMeasure();
    } else Serial.println(note);
    oledRedraw();
    return;
  }
  if (c == "g")     { psuGetVoltage(); oledRedraw(); return; }
  if (c == "m")     { psuMeasure();    oledRedraw(); return; }
  if (c == "state") { psuReadState();  oledRedraw(); return; }
  if (c == "scan")  { cmdScan();       return; }
  if (c == "status") {
    psuGetVoltage(); psuReadState(); psuMeasure(); oledRedraw(); return;
  }
  if (c == "speed") {
    if (n < 3) { Serial.println(F("usage: speed <tHigh> <tLow>")); return; }
    g_tHigh = (uint16_t)a[1].toInt();
    g_tLow  = (uint16_t)a[2].toInt();
    Serial.printf("timing now tHigh=%u tLow=%u\n", g_tHigh, g_tLow);
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
  delay(500);

  // Keep the output disabled until a setpoint has been accepted.
  pinMode(PSU_ENABLE_PIN, OUTPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);

  sdaRelease();
  sclRelease();
  delay(50);

  Wire.setSDA(OLED_SDA_PIN);
  Wire.setSCL(OLED_SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);
  g_oledOk = oledBegin();
  if (g_oledOk) {
    D->setTextSize(1);
    D->setCursor(0, 0);  D->print("APW12 PSU (Pico)");
    D->setCursor(0, 10); D->print("starting up");
    D->display();
  }

  Serial.println();
  Serial.println(F("APW12 PSU controller (Raspberry Pi Pico)"));
  Serial.printf ("PSU : SDA=GP%d SCL=GP%d EN=GP%d, active low\n",
                 PSU_SDA_PIN, PSU_SCL_PIN, PSU_ENABLE_PIN);
  Serial.printf ("OLED: SDA=GP%d SCL=GP%d addr=0x%02X %s\n",
                 OLED_SDA_PIN, OLED_SCL_PIN, OLED_ADDR,
                 g_oledOk ? "ok" : "not found");
  Serial.printf ("bus : tHigh=%u tLow=%u us (~%lu Hz)\n",
                 g_tHigh, g_tLow,
                 (unsigned long)(1000000UL / (uint32_t)(g_tHigh + g_tLow)));

  g_psuOk = psuProbe();
  if (g_psuOk) {
    Serial.println(F("[OK] PSU answered at 0x10"));

    PsuFrame f;
    if (psuExchange(CMD_GET_FW_VERSION, nullptr, 0, f) == PSU_OK && f.len >= 1) {
      g_fw0 = f.payload[0];
      Serial.print(F("[OK] firmware: "));
      for (uint8_t i = 0; i < f.len; i++) Serial.printf("%02X ", f.payload[i]);
      Serial.println();
    }
    psuWatchdog(0x00);

    // Set the voltage before enabling. The PSU powers up with DAC 0, which is
    // 15.1 V on this model.
    uint8_t dac = voltageToDac(BOOT_VOLTAGE);
    char v[12];
    fmtVolts(v, sizeof(v), BOOT_VOLTAGE);
    Serial.printf("[BOOT] setting %s V (dac=0x%02X)\n", v, dac);
    String note;
    if (psuSetDac(dac, note)) {
      Serial.printf("[BOOT] setpoint %s\n", note.c_str());
      delay(300);
#if BOOT_ENABLE_OUTPUT
      psuOutputOn();
      Serial.printf("[BOOT] output enabled at %s V\n", v);
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

  oledRedraw();
  g_lastStatus = millis();
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) handleCommand(line);
    return;                       // do not start a slow poll right after a command
  }

  // The PSU reads take about 1.5 s each, so the display refreshes every few
  // seconds rather than continuously.
  if (millis() - g_lastStatus >= STATUS_PERIOD_MS) {
    g_lastStatus = millis();
    bool wasOk = g_psuOk;
    g_psuOk = psuProbe();
    if (g_psuOk) {
      psuGetVoltage();
      psuReadState();
    } else if (wasOk) {
      Serial.println(F("[!!] lost contact with the PSU"));
    }
    oledRedraw();
  }
}
