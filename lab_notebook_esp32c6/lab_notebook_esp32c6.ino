/**
 * mmWave Lab Notebook — ESP32C6 (Seeed XIAO ESP32C6 + MR60BHA2) firmware
 * =========================================================================
 * Samples presence/distance once per second, always storing locally,
 * regardless of BLE connection state. The connected phone is responsible
 * for pulling data off the device and telling it what's safe to delete —
 * the Google Sheet (via the phone) is the source of truth, never this
 * device. This firmware NEVER deletes data on its own initiative.
 *
 * STORAGE LAYOUT (LittleFS)
 * --------------------------
 * /state.txt                     - persisted experiment state (survives reboot)
 * /<exp_id>/<index>.dat          - one file per sample: "timestamp,distance,presence"
 *
 * Each sample gets its own tiny file (instead of one big CSV) so a single
 * row can be deleted in O(1) via LittleFS.remove() without rewriting a
 * whole file. Index numbers are per-experiment, starting at 1.
 *
 * BLE PROTOCOL
 * ------------
 * Same "TYPE:VAL" text style as your VESC tuner, one Service with a
 * write characteristic (commands) and a notify characteristic (reports).
 *
 * Phone -> ESP32 (write to CMD_UUID):
 *   TIME:<unix_sec>            Sync wall clock (send right after connecting)
 *   START:<exp_id>,<name>      Begin a new experiment. exp_id is generated
 *                              phone-side, e.g. exp_20260803_143012. name
 *                              is optional and may be empty.
 *   STOP                       End the currently active experiment. Only
 *                              meaningful while connected (by design).
 *   LISTEXP                    Ask for all experiment IDs that still have
 *                              undeleted data on the device.
 *   SYNCREQ:<exp_id>           Ask for up to SYNC_BATCH_MAX undeleted rows
 *                              from that experiment.
 *   DEL:<exp_id>:<index>       Delete one specific sample file. Only ever
 *                              sent by the phone after it has confirmed
 *                              the row is safely in the Sheet.
 *
 * ESP32 -> Phone (notify on READ_UUID). Most reports are one per
 * notification, but B: backlog rows are batched several-per-notification
 * (joined by '|', VESC-tuner style) to cut BLE overhead on large syncs --
 * the phone splits each incoming notification on '|' before parsing:
 *   S:<active>:<exp_id>:<pending_count>:<storage_pct>
 *                              Heartbeat, ~every 2s. active is 0/1.
 *                              pending_count = undeleted rows in the
 *                              CURRENTLY ACTIVE experiment only.
 *   D:<index>:<distance>:<presence>
 *                              Live sample, sent right after capture,
 *                              only while connected AND an experiment
 *                              is active. Presence is 0/1.
 *   X:<exp_id1>,<exp_id2>,...  Reply to LISTEXP. Empty string if none.
 *   B:<exp_id>:<index>:<timestamp>:<distance>:<presence>
 *                              One backlog row, reply to SYNCREQ.
 *   E:<exp_id>:<remaining>     End of this SYNCREQ batch. remaining = how
 *                              many undeleted rows are still left in that
 *                              experiment (phone should call SYNCREQ again
 *                              if remaining > 0).
 *   ACK:DEL:<exp_id>:<index>   Confirms a DEL was carried out.
 *
 * STATUS LED (XIAO ESP32C6 "USER LED", GPIO15, active-LOW on most XIAO
 * boards -- verify polarity on your unit and flip LED_ACTIVE_LOW if the
 * blink looks inverted). Reflects LittleFS usage:
 *   <=20% full   : blink @ 0.5 Hz
 *   20-50% full  : blink @ 1 Hz
 *   50-80% full  : blink @ 2 Hz
 *   80-100% full : solid on
 */

#include <Arduino.h>
#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "Seeed_Arduino_mmWave.h"

#ifdef ESP32
#include <HardwareSerial.h>
HardwareSerial mmWaveSerial(0);
#else
#define mmWaveSerial Serial1
#endif

SEEED_MR60BHA2 mmWave;

// ---------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------
#define USER_LED_PIN         15
#define LED_ACTIVE_LOW       true    // flip if your board's LED is inverted

#define SAMPLE_INTERVAL_MS   1000UL
#define HEARTBEAT_INTERVAL_MS 2000UL
#define SYNC_BATCH_MAX         20    // max rows sent per single SYNCREQ
#define BLE_NOTIFY_GAP_MS     15     // spacing between back-to-back notifications
#define STORAGE_PAUSE_PCT     95     // stop sampling above this usage to protect flash

// Do not use hex-shorthand UUIDs -- Bluefy's Web Bluetooth bridge needs full strings.
#define SERVICE_UUID  "0000ab00-0000-1000-8000-00805f9b34fb"
#define CMD_UUID      "0000ab01-0000-1000-8000-00805f9b34fb"
#define READ_UUID     "0000ab02-0000-1000-8000-00805f9b34fb"

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------
BLEServer *bleServer = nullptr;
BLECharacteristic *cmdChar = nullptr;
BLECharacteristic *readChar = nullptr;
volatile bool deviceConnected = false;

bool expActive = false;
String currentExpId = "";
String currentExpName = "";
uint32_t nextIndex = 1;

// Wall-clock sync: unixSecAtSync corresponds to millis()==millisAtSync
uint32_t unixSecAtSync = 0;
uint32_t millisAtSync = 0;
bool timeSynced = false;

// Cached latest sensor reading, refreshed as fast as the sensor reports
float lastDistance = 0.0f;
bool lastPresence = false;
bool haveSensorReading = false;

unsigned long lastSampleAt = 0;
unsigned long lastHeartbeatAt = 0;

// Storage-based LED blink state
unsigned long lastLedToggleAt = 0;
bool ledOn = false;

// LittleFS is touched from two different FreeRTOS tasks -- the Arduino
// main loop (sampling, once/sec) and the BLE command callback (LISTEXP/
// SYNCREQ/DEL, on the BLE stack's own task). Without serializing access,
// a directory scan running concurrently with a new sample being written
// corrupts LittleFS's directory iteration (duplicate/skipped entries).
// Recursive so nested calls (e.g. handleSyncReq calling countPending)
// from the same task don't deadlock.
SemaphoreHandle_t fsMutex;
void fsLock() { xSemaphoreTakeRecursive(fsMutex, portMAX_DELAY); }
void fsUnlock() { xSemaphoreGiveRecursive(fsMutex); }

// ---------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------
void ledWrite(bool on) {
  digitalWrite(USER_LED_PIN, (on != LED_ACTIVE_LOW) ? HIGH : LOW);
}

uint32_t nowUnixSec() {
  if (!timeSynced) return 0;
  return unixSecAtSync + (uint32_t)((millis() - millisAtSync) / 1000UL);
}

float storagePct() {
  size_t total = LittleFS.totalBytes();
  size_t used = LittleFS.usedBytes();
  if (total == 0) return 0.0f;
  return (100.0f * (float)used) / (float)total;
}

// Splits "TYPE:REST" into type and rest (rest may itself contain ':' or ',').
void splitCommand(const String &cmd, String &type, String &rest) {
  int idx = cmd.indexOf(':');
  if (idx < 0) {
    type = cmd;
    rest = "";
  } else {
    type = cmd.substring(0, idx);
    rest = cmd.substring(idx + 1);
  }
}

void notifyLine(const String &line) {
  if (!deviceConnected || readChar == nullptr) return;
  readChar->setValue((uint8_t *)line.c_str(), line.length());
  readChar->notify();
  delay(BLE_NOTIFY_GAP_MS); // give the BLE stack room to breathe between packets
}

// ---------------------------------------------------------------------
// Persisted state (/state.txt) -- simple key=value lines, no JSON lib needed
// ---------------------------------------------------------------------
void saveState() {
  fsLock();
  File f = LittleFS.open("/state.txt", "w");
  if (!f) {
    Serial.println("ERROR: failed to open /state.txt for write");
    fsUnlock();
    return;
  }
  f.printf("active=%d\n", expActive ? 1 : 0);
  f.printf("exp_id=%s\n", currentExpId.c_str());
  f.printf("exp_name=%s\n", currentExpName.c_str());
  f.printf("next_index=%lu\n", (unsigned long)nextIndex);
  f.close();
  fsUnlock();
}

void loadState() {
  fsLock();
  if (!LittleFS.exists("/state.txt")) { fsUnlock(); return; }
  File f = LittleFS.open("/state.txt", "r");
  if (!f) { fsUnlock(); return; }
  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    int eq = line.indexOf('=');
    if (eq < 0) continue;
    String key = line.substring(0, eq);
    String val = line.substring(eq + 1);
    if (key == "active") expActive = (val.toInt() == 1);
    else if (key == "exp_id") currentExpId = val;
    else if (key == "exp_name") currentExpName = val;
    else if (key == "next_index") nextIndex = (uint32_t) val.toInt();
  }
  f.close();
  fsUnlock();
  Serial.printf("Loaded state: active=%d exp_id=%s next_index=%lu\n",
                expActive, currentExpId.c_str(), (unsigned long)nextIndex);
}

// ---------------------------------------------------------------------
// Experiment folder / sample file helpers
// ---------------------------------------------------------------------
String expDir(const String &expId) {
  return "/" + expId;
}

String sampleFilePath(const String &expId, uint32_t index) {
  return expDir(expId) + "/" + String(index) + ".dat";
}

void ensureExpDir(const String &expId) {
  fsLock();
  String dir = expDir(expId);
  if (!LittleFS.exists(dir)) {
    LittleFS.mkdir(dir);
  }
  fsUnlock();
}

// Counts undeleted sample files in an experiment folder.
uint32_t countPending(const String &expId) {
  fsLock();
  uint32_t count = 0;
  File dir = LittleFS.open(expDir(expId));
  if (!dir || !dir.isDirectory()) { fsUnlock(); return 0; }
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) count++;
    entry = dir.openNextFile();
  }
  fsUnlock();
  return count;
}

// ---------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------
void captureSample() {
  if (!expActive) return;
  if (storagePct() >= STORAGE_PAUSE_PCT) {
    Serial.println("WARNING: storage above pause threshold, skipping sample");
    return;
  }
  if (!haveSensorReading) return; // nothing from the sensor yet

  uint32_t idx = nextIndex;
  uint32_t ts = nowUnixSec(); // 0 if never time-synced -- phone should sync ASAP on connect

  fsLock();
  ensureExpDir(currentExpId);
  File f = LittleFS.open(sampleFilePath(currentExpId, idx), "w");
  if (!f) {
    Serial.println("ERROR: failed to write sample file");
    fsUnlock();
    return;
  }
  f.printf("%lu,%.1f,%d\n", (unsigned long)ts, lastDistance, lastPresence ? 1 : 0);
  f.close();

  nextIndex++;
  saveState(); // small file, cheap enough to persist every sample for reboot-safety
  fsUnlock();

  if (deviceConnected) {
    String line = "D:" + String(idx) + ":" + String(lastDistance, 1) + ":" + String(lastPresence ? 1 : 0);
    notifyLine(line);
  }
}

// ---------------------------------------------------------------------
// BLE command handling
// ---------------------------------------------------------------------
void handleStart(const String &rest) {
  int comma = rest.indexOf(',');
  String expId = (comma < 0) ? rest : rest.substring(0, comma);
  String name = (comma < 0) ? "" : rest.substring(comma + 1);
  expId.trim();
  name.trim();
  if (expId.length() == 0) {
    Serial.println("START ignored: empty exp_id");
    return;
  }
  currentExpId = expId;
  currentExpName = name;
  expActive = true;
  nextIndex = 1;
  ensureExpDir(currentExpId);
  saveState();
  Serial.printf("Experiment started: %s (%s)\n", currentExpId.c_str(), currentExpName.c_str());
}

void handleStop() {
  expActive = false;
  saveState();
  Serial.printf("Experiment stopped: %s\n", currentExpId.c_str());
}

void handleListExp() {
  fsLock();
  String result = "";
  File root = LittleFS.open("/");
  File entry = root.openNextFile();
  bool first = true;
  while (entry) {
    if (entry.isDirectory()) {
      String name = String(entry.name());
      // LittleFS core versions differ on whether directory entries come
      // back with a leading slash ("/exp_...") or bare ("exp_..."row).
      // Normalize to bare so it matches expDir()'s "/" + expId convention.
      if (name.startsWith("/")) name = name.substring(1);
      uint32_t pending = countPending(name);
      Serial.printf("LISTEXP: found dir '%s' pending=%lu\n", name.c_str(), (unsigned long)pending);
      if (pending > 0) {
        if (!first) result += ",";
        result += name;
        first = false;
      }
    }
    entry = root.openNextFile();
  }
  fsUnlock();
  Serial.println("LISTEXP result: [" + result + "]");
  notifyLine("X:" + result);
}

void handleSyncReq(const String &expId) {
  String dirPath = expDir(expId);
  Serial.printf("SYNCREQ received for '%s' (dirPath='%s')\n", expId.c_str(), dirPath.c_str());

  fsLock();

  if (!LittleFS.exists(dirPath)) {
    fsUnlock();
    Serial.println("SYNCREQ: dir does not exist, replying E:0");
    notifyLine("E:" + expId + ":0");
    return;
  }

  // Snapshot the pending count up front, before the scan, so a sample
  // written mid-scan (blocked by the mutex until we're done anyway,
  // but conceptually) doesn't skew what "remaining" means for this pass.
  uint32_t totalPending = countPending(expId);

  File dir = LittleFS.open(dirPath);
  File entry = dir.openNextFile();
  uint32_t sent = 0;

  // Batch several rows into each BLE notification instead of one
  // notify() per row -- each notify() call has real overhead, and at
  // large backlog sizes that overhead alone was making a sync pass take
  // longer than new samples take to accumulate (a diverging backlog).
  const uint32_t MAX_ROWS_PER_NOTIFY = 6;
  const size_t MAX_BATCH_CHARS = 160; // stay well under typical BLE MTU
  String batch = "";
  uint32_t rowsInBatch = 0;

  while (entry && sent < SYNC_BATCH_MAX) {
    if (!entry.isDirectory()) {
      String fname = String(entry.name()); // e.g. "42.dat" or full path depending on core
      // Extract just the numeric index regardless of whether name() returns
      // a bare filename or a full path.
      int slashIdx = fname.lastIndexOf('/');
      String base = (slashIdx >= 0) ? fname.substring(slashIdx + 1) : fname;
      int dotIdx = base.indexOf('.');
      String idxStr = (dotIdx >= 0) ? base.substring(0, dotIdx) : base;

      String content = entry.readStringUntil('\n');
      content.trim();
      // content = "timestamp,distance,presence"
      int c1 = content.indexOf(',');
      int c2 = content.indexOf(',', c1 + 1);
      if (c1 > 0 && c2 > c1) {
        String ts = content.substring(0, c1);
        String dist = content.substring(c1 + 1, c2);
        String pres = content.substring(c2 + 1);
        String rowStr = "B:" + expId + ":" + idxStr + ":" + ts + ":" + dist + ":" + pres;

        if (rowsInBatch >= MAX_ROWS_PER_NOTIFY ||
            (batch.length() > 0 && batch.length() + 1 + rowStr.length() > MAX_BATCH_CHARS)) {
          notifyLine(batch);
          batch = "";
          rowsInBatch = 0;
        }
        if (batch.length() > 0) batch += "|";
        batch += rowStr;
        rowsInBatch++;
        sent++;
      }
    }
    entry = dir.openNextFile();
  }
  if (batch.length() > 0) {
    notifyLine(batch);
  }

  fsUnlock();

  uint32_t remaining = (totalPending > sent) ? (totalPending - sent) : 0;
  Serial.printf("SYNCREQ '%s': sent=%lu totalPending=%lu remaining=%lu\n",
                expId.c_str(), (unsigned long)sent, (unsigned long)totalPending, (unsigned long)remaining);
  notifyLine("E:" + expId + ":" + String(remaining));
}

void handleDelete(const String &rest) {
  int colon = rest.indexOf(':');
  if (colon < 0) return;
  String expId = rest.substring(0, colon);
  String idxStr = rest.substring(colon + 1);
  String path = sampleFilePath(expId, (uint32_t) idxStr.toInt());
  fsLock();
  if (LittleFS.exists(path)) {
    LittleFS.remove(path);
  }
  fsUnlock();
  notifyLine("ACK:DEL:" + expId + ":" + idxStr);
}

void handleCommand(const String &raw) {
  Serial.println("CMD RX: " + raw);
  String type, rest;
  splitCommand(raw, type, rest);

  if (type == "TIME") {
    unixSecAtSync = (uint32_t) rest.toInt();
    millisAtSync = millis();
    timeSynced = true;
    Serial.printf("Time synced: %lu\n", (unsigned long)unixSecAtSync);
  } else if (type == "START") {
    handleStart(rest);
  } else if (type == "STOP") {
    handleStop();
  } else if (type == "LISTEXP") {
    handleListExp();
  } else if (type == "SYNCREQ") {
    handleSyncReq(rest);
  } else if (type == "DEL") {
    handleDelete(rest);
  } else {
    Serial.println("Unknown command: " + raw);
  }
}

// ---------------------------------------------------------------------
// BLE callbacks
// ---------------------------------------------------------------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    deviceConnected = true;
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer *server) override {
    deviceConnected = false;
    Serial.println("BLE disconnected, resuming advertising");
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = String(characteristic->getValue().c_str());
    if (value.length() > 0) {
      handleCommand(value);
    }
  }
};

// ---------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  fsMutex = xSemaphoreCreateRecursiveMutex();

  pinMode(USER_LED_PIN, OUTPUT);
  ledWrite(false);

  if (!LittleFS.begin(true)) {
    Serial.println("ERROR: LittleFS mount failed");
  }
  loadState();

  mmWaveSerial.begin(115200);
  mmWave.begin(&mmWaveSerial);

  BLEDevice::init("mmWaveTester");
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService *service = bleServer->createService(SERVICE_UUID);

  cmdChar = service->createCharacteristic(
      CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CmdCallbacks());

  readChar = service->createCharacteristic(
      READ_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  readChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as mmWaveTester");
}

void loop() {
  // Keep the mmWave parser fed as often as possible; cache latest reading.
  if (mmWave.update(50)) {
    float distance;
    if (mmWave.getDistance(distance)) {
      lastDistance = distance;
      lastPresence = true;
      haveSensorReading = true;
    } else {
      // update() returned true but no target -> treat as no presence,
      // keep last known distance for reference.
      lastPresence = false;
      haveSensorReading = true;
    }
  }

  unsigned long now = millis();

  // 1 Hz sampling, independent of BLE connection state
  if (now - lastSampleAt >= SAMPLE_INTERVAL_MS) {
    lastSampleAt = now;
    captureSample();
  }

  // Heartbeat status report
  if (deviceConnected && (now - lastHeartbeatAt >= HEARTBEAT_INTERVAL_MS)) {
    lastHeartbeatAt = now;
    uint32_t pending = expActive ? countPending(currentExpId) : 0;
    String line = "S:" + String(expActive ? 1 : 0) + ":" + currentExpId + ":" +
                  String(pending) + ":" + String(storagePct(), 1);
    notifyLine(line);
  }

  // Storage-usage LED
  float pct = storagePct();
  unsigned long blinkPeriodMs;
  bool solid = false;
  if (pct >= 80.0f) {
    solid = true;
    blinkPeriodMs = 0;
  } else if (pct >= 50.0f) {
    blinkPeriodMs = 250;  // 2 Hz -> full cycle 500ms -> toggle every 250ms
  } else if (pct >= 20.0f) {
    blinkPeriodMs = 500;  // 1 Hz -> toggle every 500ms
  } else {
    blinkPeriodMs = 1000; // 0.5 Hz -> toggle every 1000ms
  }

  if (solid) {
    ledWrite(true);
  } else if (now - lastLedToggleAt >= blinkPeriodMs) {
    lastLedToggleAt = now;
    ledOn = !ledOn;
    ledWrite(ledOn);
  }
}
