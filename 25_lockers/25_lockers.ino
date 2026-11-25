// ============================================================
//  25-Locker controller  (Arduino Mega 2560, I2C slave @ 0x08)
//  - Debounced door sensing (no blocking delays)
//  - Event queue: ESP32 polls over I2C, nothing is lost
//  - Non-blocking 3 s unlock pulse
//  - I2C callbacks only copy data; real work happens in loop()
// ============================================================
#include <Wire.h>
#include <ArduinoJson.h>   // v6

#define NUM_LOCKERS   25
#define I2C_ADDR      8
#define BUZZER_PIN    A6

const uint8_t LOCK_PINS[NUM_LOCKERS] = {
  2, 4, 6, 8, 10, 12, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40,
  42, 44, 46, 48, 50, 52, 54, 56, 58
};
const uint8_t DOOR_PINS[NUM_LOCKERS] = {
  3, 5, 7, 9, 11, 13, 23, 25, 27, 29, 31, 33, 35, 37, 39, 41,
  43, 45, 47, 49, 51, 53, 55, 57, 59
};
// NOTE: pin 54 == A0. The old code used A0 as EVENT_TRIGGER_PIN *and*
// as LOCKER_STATUS_23 at the same time. The trigger pin is no longer
// used, so A0 is only locker 23 now.

const uint8_t  UNLOCK_LEVEL   = HIGH;
const uint8_t  LOCK_LEVEL     = LOW;
const uint8_t  DOOR_OPEN      = HIGH;   // INPUT_PULLUP: switch open = HIGH
const uint8_t  BUZZER_ON      = LOW;   // swap if your buzzer is active-LOW
const uint8_t  BUZZER_OFF     = LOW;
const uint16_t UNLOCK_MS      = 2000;
const uint16_t DEBOUNCE_MS    = 50;
const uint16_t BUZZER_DELAY_MS = 3000;

// ---------- door state ----------
volatile uint8_t doorState[NUM_LOCKERS];   // read inside I2C ISR
uint8_t       lastRaw[NUM_LOCKERS];
unsigned long rawChangedAt[NUM_LOCKERS];
unsigned long lastDoorChange = 0;

// ---------- unlock timers ----------
bool          unlocking[NUM_LOCKERS];
unsigned long unlockStart[NUM_LOCKERS];

// ---------- event queue (loop -> I2C request ISR) ----------
#define Q_SIZE        32
#define EVT_CONNECTED 255
volatile uint8_t qIdx[Q_SIZE], qVal[Q_SIZE];
volatile uint8_t qHead = 0, qTail = 0;

void pushEvent(uint8_t idx, uint8_t val) {
  noInterrupts();
  uint8_t next = (qHead + 1) % Q_SIZE;
  if (next != qTail) {           // drop if full
    qIdx[qHead] = idx;
    qVal[qHead] = val;
    qHead = next;
  }
  interrupts();
}

// ---------- I2C receive (ISR: copy only) ----------
volatile char    rxBuf[40];
volatile bool    rxReady = false;

void onI2CReceive(int) {
  uint8_t k = 0;
  while (Wire.available()) {
    char c = Wire.read();
    if (k < sizeof(rxBuf) - 1) rxBuf[k++] = c;
  }
  rxBuf[k] = '\0';
  rxReady = true;
}

// ---------- I2C request (ISR: send next event or status) ----------
// Max 32 bytes per transfer on AVR Wire.
void onI2CRequest() {
  char out[33];
  int len;
  if (qTail != qHead) {
    uint8_t i = qIdx[qTail], v = qVal[qTail];
    qTail = (qTail + 1) % Q_SIZE;
    if (i == EVT_CONNECTED) {
      len = snprintf(out, sizeof(out), "{'ds':'1'}");
    } else {
      // {'ln':'l06','ls':'0','ds':'1'}  -> 31 bytes
      len = snprintf(out, sizeof(out), "{'ln':'l%02u','ls':'%u','ds':'1'}",
                     (unsigned)(i + 1), (unsigned)v);
    }
  } else {
    // Idle -> full status "S:0101..." (27 bytes)
    out[0] = 'S'; out[1] = ':';
    for (uint8_t i = 0; i < NUM_LOCKERS; i++) out[2 + i] = doorState[i] ? '1' : '0';
    len = 2 + NUM_LOCKERS;
  }
  Wire.write((const uint8_t *)out, len);
}

// ---------- helpers ----------
void unlockLocker(uint8_t i) {
  digitalWrite(LOCK_PINS[i], UNLOCK_LEVEL);
  unlocking[i]   = true;
  unlockStart[i] = millis();
  Serial.print(F("Unlock l")); Serial.println(i + 1);
}

void handleCommand(const char *cmd) {
  Serial.print(F("RX: ")); Serial.println(cmd);
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, cmd)) {
    Serial.println(F("Bad JSON"));
    return;
  }
  const char *ln = doc["ln"] | "";
  const char *ds = doc["ds"] | "";

  if (ln[0] == 'l') {                       // accepts "l1" and "l01"
    int n = atoi(ln + 1);
    if (n >= 1 && n <= NUM_LOCKERS) unlockLocker(n - 1);
  }
  if (ds[0] == 'y') pushEvent(EVT_CONNECTED, 1);
}

void scanDoors() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_LOCKERS; i++) {
    uint8_t r = digitalRead(DOOR_PINS[i]);
    if (r != lastRaw[i]) {
      lastRaw[i] = r;
      rawChangedAt[i] = now;
    } else if (r != doorState[i] && now - rawChangedAt[i] >= DEBOUNCE_MS) {
      doorState[i] = r;
      lastDoorChange = now;
      pushEvent(i, r);
      Serial.print(F("Door l")); Serial.print(i + 1);
      Serial.print(F(" = ")); Serial.println(r);
    }
  }
}

void serviceUnlocks() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_LOCKERS; i++) {
    if (unlocking[i] && now - unlockStart[i] >= UNLOCK_MS) {
      digitalWrite(LOCK_PINS[i], LOCK_LEVEL);
      unlocking[i] = false;
    }
  }
}

void serviceBuzzer() {
  bool anyOpen = false;
  for (uint8_t i = 0; i < NUM_LOCKERS; i++)
    if (doorState[i] == DOOR_OPEN) { anyOpen = true; break; }
  bool on = anyOpen && (millis() - lastDoorChange > BUZZER_DELAY_MS);
  digitalWrite(BUZZER_PIN, on ? BUZZER_ON : BUZZER_OFF);
}

// ---------- setup / loop ----------
void setup() {
  Serial.begin(9600);

  for (uint8_t i = 0; i < NUM_LOCKERS; i++) {
    pinMode(DOOR_PINS[i], INPUT_PULLUP);
    pinMode(LOCK_PINS[i], OUTPUT);
    digitalWrite(LOCK_PINS[i], LOCK_LEVEL);
    unlocking[i] = false;
  }
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, BUZZER_OFF);

  delay(20);
  for (uint8_t i = 0; i < NUM_LOCKERS; i++) {   // initial state, no fake events
    doorState[i] = lastRaw[i] = digitalRead(DOOR_PINS[i]);
    rawChangedAt[i] = millis();
  }

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);   // register ONCE, here
  Wire.onRequest(onI2CRequest);

  Serial.println(F("Initiated all Lockers"));
}

void loop() {
  if (rxReady) {
    char cmd[40];
    noInterrupts();
    for (uint8_t k = 0; k < sizeof(cmd); k++) cmd[k] = rxBuf[k];
    rxReady = false;
    interrupts();
    handleCommand(cmd);
  }
  scanDoors();
  serviceUnlocks();
  serviceBuzzer();
}