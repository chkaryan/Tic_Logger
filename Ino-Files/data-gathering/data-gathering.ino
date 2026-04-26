// ============================================================
//  TIC LOGGER — Continuous Data Collection Firmware
//  ESP32 + MPU6500 + MAX30102
//  Streams to Firebase Firestore via WiFi (batched)
// ============================================================

#include <Wire.h>
#include "MAX30105.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>   // v6 — install via Library Manager

// ===================== CONFIGURATION ========================
// --- WiFi ---
#define WIFI_SSID       "YOUR_SSID"
#define WIFI_PASSWORD   "YOUR_PASSWORD"

// --- Firebase ---
// Project ID from Firebase console (not the full URL)
#define FIREBASE_PROJECT_ID   "your-project-id"
// Web API key from Firebase console → Project Settings → General
#define FIREBASE_API_KEY      "your-web-api-key"
// Firestore collection name
#define COLLECTION_NAME       "tic_sessions"

// --- Pins ---
#define SDA_PIN           8
#define SCL_PIN           9
#define TFT_DC            7
#define TFT_CS            3
#define MPU6500_ADDR      0x68

#define YES_BUTTON_PIN    10
#define NO_BUTTON_PIN     20
#define MANUAL_BUTTON_PIN 5

// --- Sampling ---
#define SAMPLE_RATE_HZ    100          // target sample rate
#define SAMPLE_INTERVAL_MS 10          // 1000 / SAMPLE_RATE_HZ

// --- Batching ---
#define BATCH_SIZE        50           // samples per Firebase POST
#define SEND_INTERVAL_MS  500          // send every 500ms (= 50 samples at 100Hz)
#define MAX_RETRY         3            // retry failed POSTs this many times

// --- Spike detection (same thresholds as before) ---
#define SPIKE_DELTA_THRESHOLD 0.8f
#define SPIKE_MIN_MAG         1.5f

// --- Heart rate ---
#define AC_THRESHOLD    8
#define MIN_BPM         45
#define MAX_BPM         120
#define DC_ALPHA        0.80
#define PEAK_LOCKOUT_MS 450
#define BPM_HISTORY     4

// --- SpO2 ---
#define SPO2_ALPHA      0.95f
#define SPO2_AC_ALPHA   0.70f
#define SPO2_HISTORY    8
// ============================================================

// ==================== SAMPLE STRUCT ========================
// One row of the dataset
struct Sample {
  uint32_t timestamp;
  float    ax, ay, az;
  float    gx, gy, gz;
  float    mag;
  float    gyro_mag;
  float    jerk;
  float    bpm;
  float    spo2;
  uint8_t  tic_event;   // 0=normal, 1=tic confirmed, 2=tic denied, 3=manual
};

// ==================== CIRCULAR BUFFER ======================
#define BUFFER_SIZE 200                // holds 2 seconds at 100Hz — safety margin
Sample   sampleBuffer[BUFFER_SIZE];
uint16_t writeIdx   = 0;              // next slot to write
uint16_t sendIdx    = 0;              // next slot to send
uint16_t buffered() {                 // how many unsent samples waiting
  return (writeIdx >= sendIdx)
    ? (writeIdx - sendIdx)
    : (BUFFER_SIZE - sendIdx + writeIdx);
}
// ============================================================

// ==================== SENSOR STATE =========================
MAX30105 sensor;
Adafruit_GC9A01A tft(TFT_CS, TFT_DC);

// Heart rate
long  dcAvg       = 0;
bool  aboveThresh = false;
unsigned long lastBeatTime = 0;
float currentBPM  = 0;
float avgBPM      = 0;
float bpmHist[BPM_HISTORY] = {0};
int   bpmIdx      = 0;
long  lastAC      = 0;

// SpO2
long  dcAvgRed = 0, dcAvgIR = 0;
float acRmsRed = 0, acRmsIR = 0;
float avgSpO2  = 0;
float spo2Hist[SPO2_HISTORY] = {0};
int   spo2Idx  = 0;

// IMU
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;
float   ax_g, ay_g, az_g;
float   gx_dps, gy_dps, gz_dps;
float   accel_mag     = 0;
float   last_accel_mag = 0;
float   jerk          = 0;

// Buttons / tic state
bool          ticPromptActive  = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 300;
uint8_t       pendingTicEvent  = 0;   // set when spike detected, cleared after label

// Timing
unsigned long lastSampleTime = 0;
unsigned long lastSendTime   = 0;

// Session ID — unique per boot, used as Firestore document group
String sessionId;

// WiFi / upload stats for display
bool     wifiOK        = false;
uint32_t totalSent     = 0;
uint32_t totalDropped  = 0;
// ============================================================

// ==================== FIREBASE HELPERS =====================

// Build the Firestore REST URL for a batch write
// We use the runQuery / batchWrite endpoint approach via simple documents
String firestoreUrl() {
  return String("https://firestore.googleapis.com/v1/projects/")
       + FIREBASE_PROJECT_ID
       + "/databases/(default)/documents/"
       + COLLECTION_NAME
       + "/" + sessionId
       + "/samples?key="
       + FIREBASE_API_KEY;
}

// Serialize one sample into a Firestore Value map field object
void sampleToFirestoreFields(const Sample& s, JsonObject fields) {
  auto addDouble = [&](const char* key, double val) {
    fields[key]["doubleValue"] = val;
  };
  auto addInt = [&](const char* key, int val) {
    fields[key]["integerValue"] = String(val);
  };

  addInt   ("timestamp",  s.timestamp);
  addDouble("ax",         s.ax);
  addDouble("ay",         s.ay);
  addDouble("az",         s.az);
  addDouble("gx",         s.gx);
  addDouble("gy",         s.gy);
  addDouble("gz",         s.gz);
  addDouble("mag",        s.mag);
  addDouble("gyro_mag",   s.gyro_mag);
  addDouble("jerk",       s.jerk);
  addDouble("bpm",        s.bpm);
  addDouble("spo2",       s.spo2);
  addInt   ("tic_event",  s.tic_event);
}

// POST a batch of samples to Firestore using batchWrite
// Each sample becomes its own document with auto-generated ID
bool postBatch(Sample* samples, uint16_t count) {
  if (!wifiOK || count == 0) return false;

  // Firestore batchWrite endpoint
  String url = String("https://firestore.googleapis.com/v1/projects/")
             + FIREBASE_PROJECT_ID
             + "/databases/(default)/documents:batchWrite?key="
             + FIREBASE_API_KEY;

  // Build JSON: { "writes": [ { "update": { "name": "...", "fields": {...} } }, ... ] }
  // Each sample doc name includes session + timestamp for uniqueness
  DynamicJsonDocument doc(8192);
  JsonArray writes = doc.createNestedArray("writes");

  for (uint16_t i = 0; i < count; i++) {
    JsonObject write  = writes.createNestedObject();
    JsonObject update = write.createNestedObject("update");

    // Document name (full resource path)
    update["name"] = String("projects/") + FIREBASE_PROJECT_ID
                   + "/databases/(default)/documents/"
                   + COLLECTION_NAME + "/" + sessionId
                   + "/samples/" + String(samples[i].timestamp)
                   + "_" + String(i);   // i disambiguates same-ms samples

    sampleToFirestoreFields(samples[i], update.createNestedObject("fields"));
  }

  String body;
  serializeJson(doc, body);

  HTTPClient http;
  http.begin(url);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(5000);

  int code = http.POST(body);
  http.end();

  return (code == 200);
}

// ==================== SPO2 HELPER ==========================
float calculateSpO2(float acRed, float dcRed, float acIR, float dcIR) {
  if (dcRed < 1 || dcIR < 1 || acIR < 0.001f) return 0;
  float R    = (acRed / dcRed) / (acIR / dcIR);
  float spo2 = 110.0f - 25.0f * R;
  if (spo2 < 70.0f || spo2 > 100.0f) return 0;
  return spo2;
}

// ==================== SETUP ================================
void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // bump to 400kHz — gives more headroom for 100Hz + display

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(GC9A01A_BLACK);

  // --- WiFi ---
  drawStatus("Connecting WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    attempts++;
  }
  wifiOK = (WiFi.status() == WL_CONNECTED);

  // Session ID = boot timestamp (ms) as hex string — unique enough
  sessionId = String(millis(), HEX);

  // --- MAX30102 ---
  drawStatus("Init sensors...");
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    drawStatus("MAX30102 FAIL");
    while(1);
  }
  sensor.setup(0x3F, 4, 2, 200, 411, 4096);
  sensor.setPulseAmplitudeIR(0xFF);
  sensor.setPulseAmplitudeRed(0xBB);

  // --- MPU6500 ---
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B); Wire.write(0x00);   // wake up
  Wire.endTransmission(true);
  delay(10);

  // Gyro range ±500 dps (0x08), Accel range ±8g (0x10)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission(true);

  // --- Buttons ---
  pinMode(YES_BUTTON_PIN,    INPUT_PULLUP);
  pinMode(NO_BUTTON_PIN,     INPUT_PULLUP);
  pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);

  // --- Serial CSV header (backup local log) ---
  Serial.println("timestamp,ax,ay,az,gx,gy,gz,mag,gyro_mag,jerk,bpm,spo2,tic_event");

  drawMainUI();
  lastSampleTime = millis();
  lastSendTime   = millis();
}

// ==================== MAIN LOOP ============================
void loop() {
  unsigned long now = millis();

  // ---- 1. SAMPLE at fixed rate ----
  if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = now;
    takeSample(now);
  }

  // ---- 2. SEND batch when enough samples or time elapsed ----
  if ((buffered() >= BATCH_SIZE || now - lastSendTime >= SEND_INTERVAL_MS)
       && buffered() > 0) {
    sendBatch();
    lastSendTime = now;
  }

  // ---- 3. BUTTONS ----
  handleButtons(now);

  // ---- 4. DISPLAY update (every 200ms — don't let it eat sample budget) ----
  static unsigned long lastDisplay = 0;
  if (now - lastDisplay >= 200) {
    lastDisplay = now;
    if (!ticPromptActive) {
      updateBPMDisplay(avgBPM, sensor.getIR() > 15000);
      updateSpO2Display(avgSpO2, sensor.getIR() > 15000);
      updateIMUDisplay();
      updateWifiStatus();
    }
  }
}

// ==================== SAMPLE COLLECTION ====================
void takeSample(unsigned long now) {
  // -- Read IMU (14 bytes: accel + temp_skip + gyro) --
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 14, true);

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();              // temperature bytes — skip
  gx_raw = (Wire.read() << 8) | Wire.read();
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();

  // ±8g range → 4096 LSB/g
  ax_g = ax_raw / 4096.0f;
  ay_g = ay_raw / 4096.0f;
  az_g = az_raw / 4096.0f;

  // ±500 dps range → 65.5 LSB/dps
  gx_dps = gx_raw / 65.5f;
  gy_dps = gy_raw / 65.5f;
  gz_dps = gz_raw / 65.5f;

  float new_mag  = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  float gyro_mag = sqrt(gx_dps*gx_dps + gy_dps*gy_dps + gz_dps*gz_dps);
  jerk           = (new_mag - last_accel_mag) / (SAMPLE_INTERVAL_MS / 1000.0f);
  last_accel_mag = new_mag;
  accel_mag      = new_mag;

  // -- Read optical sensor --
  long rawIR  = sensor.getIR();
  long rawRed = sensor.getRed();
  bool contact = (rawIR > 15000);

  if (contact) {
    // DC tracking
    dcAvgIR  = SPO2_ALPHA * dcAvgIR  + (1.0f - SPO2_ALPHA) * rawIR;
    dcAvgRed = SPO2_ALPHA * dcAvgRed + (1.0f - SPO2_ALPHA) * rawRed;

    long acIR  = rawIR  - dcAvgIR;
    long acRed = rawRed - dcAvgRed;

    acRmsIR  = SPO2_AC_ALPHA * acRmsIR  + (1.0f - SPO2_AC_ALPHA) * abs((float)acIR);
    acRmsRed = SPO2_AC_ALPHA * acRmsRed + (1.0f - SPO2_AC_ALPHA) * abs((float)acRed);

    // Heart rate
    dcAvg = DC_ALPHA * dcAvg + (1.0 - DC_ALPHA) * rawIR;
    long ac = rawIR - dcAvg;

    if (ac > AC_THRESHOLD && lastAC <= AC_THRESHOLD && !aboveThresh) {
      if (lastBeatTime == 0) {
        lastBeatTime = now;
      } else if (now - lastBeatTime > PEAK_LOCKOUT_MS) {
        unsigned long interval = now - lastBeatTime;
        float newBPM = 60000.0f / interval;

        if (newBPM >= MIN_BPM && newBPM <= MAX_BPM) {
          bool outlier = (avgBPM > 0 && abs(newBPM - avgBPM) > 15);
          if (!outlier) {
            bpmHist[bpmIdx] = newBPM;
            bpmIdx = (bpmIdx + 1) % BPM_HISTORY;
            float sum = 0; int cnt = 0;
            for (int i = 0; i < BPM_HISTORY; i++) if (bpmHist[i] > 0) { sum += bpmHist[i]; cnt++; }
            avgBPM = cnt ? sum / cnt : 0;

            float sp = calculateSpO2(acRmsRed, dcAvgRed, acRmsIR, dcAvgIR);
            if (sp > 0) {
              spo2Hist[spo2Idx] = sp;
              spo2Idx = (spo2Idx + 1) % SPO2_HISTORY;
              float ssum = 0; int scnt = 0;
              for (int i = 0; i < SPO2_HISTORY; i++) if (spo2Hist[i] > 0) { ssum += spo2Hist[i]; scnt++; }
              avgSpO2 = scnt ? ssum / scnt : 0;
            }
          }
        }
        lastBeatTime = now;
      }
      aboveThresh = true;
    }
    if (ac < 0) aboveThresh = false;
    lastAC = ac;
  }

  // -- Spike detection --
  if (accel_mag > SPIKE_MIN_MAG &&
      jerk > (SPIKE_DELTA_THRESHOLD / (SAMPLE_INTERVAL_MS / 1000.0f)) &&
      !ticPromptActive) {
    ticPromptActive = true;
    pendingTicEvent = 0;
    drawTicPrompt();
  }

  // -- Write sample to circular buffer --
  Sample& s    = sampleBuffer[writeIdx];
  s.timestamp  = now;
  s.ax         = ax_g;
  s.ay         = ay_g;
  s.az         = az_g;
  s.gx         = gx_dps;
  s.gy         = gy_dps;
  s.gz         = gz_dps;
  s.mag        = accel_mag;
  s.gyro_mag   = gyro_mag;
  s.jerk       = jerk;
  s.bpm        = avgBPM;
  s.spo2       = avgSpO2;
  s.tic_event  = pendingTicEvent;

  // Also echo to Serial as CSV backup
  Serial.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%d\n",
    now, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps,
    accel_mag, gyro_mag, jerk, avgBPM, avgSpO2, pendingTicEvent);

  writeIdx = (writeIdx + 1) % BUFFER_SIZE;

  // If buffer is completely full, advance sendIdx to avoid overwrite
  // (drops oldest unsent sample — better than corrupting newer ones)
  if (writeIdx == sendIdx) {
    sendIdx = (sendIdx + 1) % BUFFER_SIZE;
    totalDropped++;
  }

  // Clear tic event flag after it's been written into the sample
  pendingTicEvent = 0;
}

// ==================== BATCH SEND ===========================
void sendBatch() {
  if (!wifiOK) {
    // Reconnect attempt
    if (WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
      return;
    }
    wifiOK = true;
  }

  uint16_t count = min((uint16_t)BATCH_SIZE, buffered());
  if (count == 0) return;

  // Copy batch into a flat array (handles buffer wraparound)
  Sample batch[BATCH_SIZE];
  for (uint16_t i = 0; i < count; i++) {
    batch[i] = sampleBuffer[(sendIdx + i) % BUFFER_SIZE];
  }

  bool ok = false;
  for (uint8_t attempt = 0; attempt < MAX_RETRY && !ok; attempt++) {
    ok = postBatch(batch, count);
    if (!ok) delay(100);
  }

  if (ok) {
    sendIdx    = (sendIdx + count) % BUFFER_SIZE;
    totalSent += count;
  } else {
    // Give up on this batch rather than block indefinitely
    // Samples stay in buffer and will retry on next send cycle
    // unless overwritten by new data first
    totalDropped += count;
    sendIdx = (sendIdx + count) % BUFFER_SIZE;
  }
}

// ==================== BUTTON HANDLING ======================
void handleButtons(unsigned long now) {
  if (now - lastDebounceTime < DEBOUNCE_DELAY) return;

  if (digitalRead(MANUAL_BUTTON_PIN) == LOW) {
    pendingTicEvent = 3;   // will be written on next sample tick
    lastDebounceTime = now;
    return;
  }

  if (ticPromptActive) {
    if (digitalRead(YES_BUTTON_PIN) == LOW) {
      pendingTicEvent  = 1;
      ticPromptActive  = false;
      lastDebounceTime = now;
      drawMainUI();
    }
    else if (digitalRead(NO_BUTTON_PIN) == LOW) {
      pendingTicEvent  = 2;
      ticPromptActive  = false;
      lastDebounceTime = now;
      drawMainUI();
    }
  }
}

// ==================== UI FUNCTIONS =========================
void drawStatus(const char* msg) {
  tft.fillScreen(GC9A01A_BLACK);
  tft.setCursor(20, 100);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(2);
  tft.println(msg);
}

void drawMainUI() {
  tft.fillScreen(GC9A01A_BLACK);
  tft.drawCircle(120, 120, 100, GC9A01A_BLUE);
  tft.drawCircle(120, 120, 98,  GC9A01A_BLUE);

  tft.setCursor(85, 30);
  tft.setTextColor(GC9A01A_CYAN);
  tft.setTextSize(2);
  tft.println("BPM");

  tft.setCursor(10, 155);
  tft.setTextColor(GC9A01A_MAGENTA);
  tft.setTextSize(1);
  tft.println("SpO2:");

  // WiFi indicator label
  tft.setCursor(10, 200);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(1);
  tft.println("WiFi:");

  // Sent counter label
  tft.setCursor(10, 215);
  tft.println("Sent:");

  drawHeartIcon(120, 180, GC9A01A_RED);

  tft.setCursor(10, 10);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(1);
  tft.println("IMU:");
}

void drawTicPrompt() {
  tft.fillRect(15, 55, 210, 130, GC9A01A_BLACK);
  tft.setTextColor(GC9A01A_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(68, 75);
  tft.println("TIC?");
  tft.setTextSize(2);
  tft.setCursor(28, 115);
  tft.println("Was that a tic?");
}

void updateBPMDisplay(float bpm, bool contact) {
  tft.fillRect(40, 70, 160, 60, GC9A01A_BLACK);
  if (!contact) {
    tft.setCursor(70, 95); tft.setTextColor(GC9A01A_YELLOW); tft.setTextSize(2); tft.println("---");
  } else if (bpm == 0) {
    tft.setCursor(50, 95); tft.setTextColor(GC9A01A_CYAN); tft.setTextSize(2); tft.println("WAIT");
  } else {
    tft.setTextSize(5); tft.setTextColor(GC9A01A_GREEN);
    String s = String((int)bpm);
    tft.setCursor(120 - (s.length() * 30) / 2, 85);
    tft.println(s);
  }
}

void updateSpO2Display(float spo2, bool contact) {
  tft.fillRect(45, 163, 80, 10, GC9A01A_BLACK);
  tft.setTextSize(1);
  if (!contact || spo2 == 0) {
    tft.setTextColor(GC9A01A_YELLOW); tft.setCursor(45, 163); tft.println("---");
  } else {
    tft.setTextColor(GC9A01A_MAGENTA); tft.setCursor(45, 163);
    tft.print((int)spo2); tft.println("%");
  }
}

void updateIMUDisplay() {
  tft.fillRect(35, 10, 185, 10, GC9A01A_BLACK);
  tft.setCursor(35, 10); tft.setTextColor(GC9A01A_WHITE); tft.setTextSize(1);
  tft.print("X="); tft.print(ax_g, 1);
  tft.print(" Y="); tft.print(ay_g, 1);
  tft.print(" Z="); tft.print(az_g, 1);
}

void updateWifiStatus() {
  tft.fillRect(40, 200, 180, 20, GC9A01A_BLACK);
  tft.setTextSize(1);

  // WiFi indicator
  tft.setCursor(40, 200);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(GC9A01A_GREEN); tft.println("OK");
  } else {
    tft.setTextColor(GC9A01A_RED); tft.println("DOWN");
  }

  // Sent / dropped counter
  tft.setCursor(40, 215);
  tft.setTextColor(GC9A01A_WHITE);
  tft.print(totalSent);
  if (totalDropped > 0) {
    tft.setTextColor(GC9A01A_RED);
    tft.print(" -"); tft.print(totalDropped);
  }
}

void drawHeartIcon(int x, int y, uint16_t color) {
  tft.fillCircle(x - 8, y - 5, 6, color);
  tft.fillCircle(x + 8, y - 5, 6, color);
  tft.fillTriangle(x - 14, y - 2, x, y + 10, x + 14, y - 2, color);
}