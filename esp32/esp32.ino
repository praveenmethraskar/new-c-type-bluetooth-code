// ============================================================
//  ESP32 (Type-C devkit)  <->  Bluetooth SPP  <->  I2C master
//  Works with: lockers_25_mega.ino  (Mega, I2C address 8)
//
//  - Polls the Mega over I2C every 100 ms (no D5 trigger pin)
//  - Door event -> sent to Bluetooth immediately
//  - The LAST door event is repeated every REPEAT_MS, like old code:
//      {'ln':'l01','ls':'0','ds':'1'}
//      {'ln':'l01','ls':'0','ds':'1'}
//      ...
//  - Before any door event since power-on, it cycles through all
//    25 lockers' current status in the same format.
// ============================================================
#include <Wire.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

BluetoothSerial SerialBT;

const uint8_t  ARDUINO_ADDR = 8;
const int      SDA_PIN      = 21;
const int      SCL_PIN      = 22;
const uint32_t POLL_MS      = 100;    // how often to ask the Mega
const uint32_t REPEAT_MS    = 1000;   // how often to repeat the last log

String   lastEvent  = "";     // last {'ln':..,'ls':..,'ds':..} message
String   lastStatus = "";     // "0111101111..." from the Mega
uint8_t  rrIndex    = 0;      // round robin before first event
uint32_t lastPoll = 0, lastRepeat = 0;

// Read one reply (max 32 bytes) from the Mega
String readArduino() {
  String s;
  uint8_t n = Wire.requestFrom((int)ARDUINO_ADDR, 32);
  for (uint8_t k = 0; k < n && Wire.available(); k++) {
    char c = Wire.read();
    if (c >= 32 && c <= 126) s += c;   // drop 0xFF padding
  }
  s.trim();
  return s;
}

void sendOut(const String &msg) {
  Serial.println(msg);
  if (SerialBT.hasClient()) SerialBT.println(msg);
}

void pollArduino() {
  // Drain all queued events first
  for (int guard = 0; guard < 40; guard++) {
    String r = readArduino();
    if (r.length() == 0) {
      Serial.println("I2C: no reply (check SDA/SCL/GND)");
      return;
    }
    if (r.startsWith("{")) {            // door event / device msg
      sendOut(r);
      if (r.indexOf("'ln'") >= 0) lastEvent = r;
      lastRepeat = millis();            // restart repeat timer
      continue;
    }
    if (r.startsWith("S:")) {           // idle -> status of all doors
      lastStatus = r.substring(2);
    }
    return;
  }
}

void repeatLog() {
  if (millis() - lastRepeat < REPEAT_MS) return;
  lastRepeat = millis();

  if (lastEvent.length() > 0) {
    sendOut(lastEvent);                 // same log again and again
    return;
  }
  // No door event yet since power-on: show each locker in turn
  if (lastStatus.length() >= 25) {
    char buf[40];
    snprintf(buf, sizeof(buf), "{'ln':'l%02u','ls':'%c','ds':'1'}",
             (unsigned)(rrIndex + 1), lastStatus[rrIndex]);
    sendOut(String(buf));
    rrIndex = (rrIndex + 1) % 25;
  }
}

void forwardBluetoothCommand() {
  if (!SerialBT.available()) return;
  String cmd = SerialBT.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  if (cmd.length() > 32) {
    sendOut("{'err':'cmd too long'}");
    return;
  }
  Serial.println("BT -> " + cmd);
  Wire.beginTransmission(ARDUINO_ADDR);
  Wire.write((const uint8_t *)cmd.c_str(), cmd.length());
  uint8_t err = Wire.endTransmission();
  if (err) sendOut("{'err':'i2c " + String(err) + "'}");
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("secundrabad_locker");
  SerialBT.setTimeout(50);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  Serial.println("Bridge started, pair via Bluetooth");
}

void loop() {
  forwardBluetoothCommand();
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();
    pollArduino();
  }
  repeatLog();
}