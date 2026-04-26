#include <Wire.h>
#include "MAX30105.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// ================= PIN CONFIGURATION =================
#define SDA_PIN 8
#define SCL_PIN 9
#define TFT_DC 7
#define TFT_CS 3
#define MPU6500_ADDR 0x68

#define YES_BUTTON_PIN     10
#define NO_BUTTON_PIN      20
#define MANUAL_BUTTON_PIN  5

#define SPIKE_DELTA_THRESHOLD 0.8f
#define SPIKE_MIN_MAG         1.5f

// ================= HEART RATE SETTINGS ===============
#define AC_THRESHOLD 8
#define MIN_BPM 45
#define MAX_BPM 120
#define DC_ALPHA 0.80
#define PEAK_LOCKOUT_MS 450
#define BPM_HISTORY 4
// =====================================================

long dcAvg = 0;
bool aboveThreshold = false;
unsigned long lastBeatTime = 0;
float currentBPM = 0;
float avgBPM = 0;
float bpmHist[BPM_HISTORY];
int bpmIdx = 0;
long lastAC = 0;

int pulseRadius = 0;
bool pulsing = false;
unsigned long pulseStartTime = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temp;
float ax_g, ay_g, az_g;
float gx_dps, gy_dps, gz_dps;
float temperature;
float accel_magnitude;

float last_accel_magnitude = 0.0;
bool ticPromptActive = false;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 300;

MAX30105 sensor;
Adafruit_GC9A01A tft(TFT_CS, TFT_DC);

void setup() {
  Serial.begin(115200);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(GC9A01A_BLACK);
  
  drawWelcomeScreen();
  delay(2000);
  
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    while(1);
  }

  // Wrist-optimized config
  sensor.setup(0x3F, 4, 2, 200, 411, 4096);
  sensor.setPulseAmplitudeIR(0xFF);
  sensor.setPulseAmplitudeRed(0x00);

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B); Wire.write(0); Wire.endTransmission(true);
  delay(100);

  pinMode(YES_BUTTON_PIN, INPUT_PULLUP);
  pinMode(NO_BUTTON_PIN, INPUT_PULLUP);
  pinMode(MANUAL_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("timestamp,ax,ay,az,gx,gy,gz,mag,temp,bpm,label");

  drawMainUI();
}

void loop() {
  unsigned long currentTime = millis();

  // ================= HEART RATE =================
  long rawIR = sensor.getIR();
  bool hasContact = (rawIR > 15000);

  if (hasContact) {
    dcAvg = DC_ALPHA * dcAvg + (1.0 - DC_ALPHA) * rawIR;
    long ac = rawIR - dcAvg;

    // Rising edge detection
    if (ac > AC_THRESHOLD && lastAC <= AC_THRESHOLD && !aboveThreshold) {

      if (lastBeatTime == 0) {
        lastBeatTime = currentTime;
      } 
      else if (currentTime - lastBeatTime > PEAK_LOCKOUT_MS) {

        unsigned long interval = currentTime - lastBeatTime;
        float newBPM = 60000.0 / interval;

        if (newBPM >= MIN_BPM && newBPM <= MAX_BPM) {

          bool isOutlier = (avgBPM > 0 && abs(newBPM - avgBPM) > 15);

          if (!isOutlier) {
            bpmHist[bpmIdx] = newBPM;
            bpmIdx = (bpmIdx + 1) % BPM_HISTORY;

            float sum = 0; int count = 0;
            for (int i = 0; i < BPM_HISTORY; i++) {
              if (bpmHist[i] > 0) {
                sum += bpmHist[i];
                count++;
              }
            }

            avgBPM = count ? sum / count : 0;
            currentBPM = newBPM;

            pulsing = true;
            pulseStartTime = currentTime;
          }
        }

        lastBeatTime = currentTime;
      }

      aboveThreshold = true;
    }

    if (ac < 0) aboveThreshold = false;

    lastAC = ac;
  }

  // ================= MANUAL BUTTON (ALWAYS WORKS) =================
  if (digitalRead(MANUAL_BUTTON_PIN) == LOW &&
      (currentTime - lastDebounceTime > DEBOUNCE_DELAY)) {

    logTicData("manual");
    lastDebounceTime = currentTime;
  }

  // ================= IMU =================
  Wire.beginTransmission(MPU6500_ADDR); Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 6, true);

  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  ax_g = ax / 4096.0;
  ay_g = ay / 4096.0;
  az_g = az / 4096.0;

  accel_magnitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

  // ================= TIC DETECTION =================
  bool isSpike = false;

  if (last_accel_magnitude > 0 && !ticPromptActive) {
    if (accel_magnitude > SPIKE_MIN_MAG &&
        (accel_magnitude - last_accel_magnitude) > SPIKE_DELTA_THRESHOLD) {
      isSpike = true;
    }
  }

  last_accel_magnitude = accel_magnitude;

  if (isSpike) {
    ticPromptActive = true;
    drawTicPrompt();
  }

  // YES / NO only during prompt
  if (ticPromptActive && (currentTime - lastDebounceTime > DEBOUNCE_DELAY)) {

    if (digitalRead(YES_BUTTON_PIN) == LOW) {
      logTicData("yes");
      ticPromptActive = false;
      lastDebounceTime = currentTime;
      drawMainUI();
    }

    else if (digitalRead(NO_BUTTON_PIN) == LOW) {
      logTicData("no");
      ticPromptActive = false;
      lastDebounceTime = currentTime;
      drawMainUI();
    }
  }

  // ================= DISPLAY =================
  static unsigned long lastDisplayUpdate = 0;

  if (currentTime - lastDisplayUpdate > 100) {
    lastDisplayUpdate = currentTime;

    if (!ticPromptActive) {
      updateBPMDisplay(avgBPM, hasContact);
      updateSignalIndicator(hasContact, rawIR);
      updateIMUDisplay();
    }
  }

  delay(10);
}

// ================= LOGGING =================
void logTicData(const char* label) {
  Serial.print(millis()); Serial.print(",");
  Serial.print(ax_g, 3); Serial.print(",");
  Serial.print(ay_g, 3); Serial.print(",");
  Serial.print(az_g, 3); Serial.print(",");
  Serial.print(accel_magnitude, 3); Serial.print(",");
  Serial.print(temperature, 2); Serial.print(",");
  Serial.print(currentBPM, 1); Serial.print(",");
  Serial.println(label);
}

// ====================== UI FUNCTIONS ======================
void drawWelcomeScreen() {
  tft.fillScreen(GC9A01A_BLACK);
  tft.fillCircle(100, 80, 20, GC9A01A_RED);
  tft.fillCircle(140, 80, 20, GC9A01A_RED);
  tft.fillTriangle(80, 85, 120, 140, 160, 85, GC9A01A_RED);

  tft.setCursor(55, 160);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(2);
  tft.println("HEART");

  tft.setCursor(50, 185);
  tft.println("MONITOR");
}

void drawMainUI() {
  tft.fillScreen(GC9A01A_BLACK);

  tft.drawCircle(120, 120, 100, GC9A01A_BLUE);
  tft.drawCircle(120, 120, 98, GC9A01A_BLUE);

  tft.setCursor(85, 30);
  tft.setTextColor(GC9A01A_CYAN);
  tft.setTextSize(2);
  tft.println("BPM");

  drawHeartIcon(120, 180, GC9A01A_RED);

  tft.setCursor(10, 10);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(1);
  tft.println("Accel:");

  tft.setCursor(10, 25);
  tft.println("Gyro:");

  tft.setCursor(10, 40);
  tft.println("Temp:");
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

void updateBPMDisplay(float bpm, bool hasContact) {
  tft.fillRect(40, 70, 160, 60, GC9A01A_BLACK);

  if (!hasContact) {
    tft.setCursor(70, 95);
    tft.setTextColor(GC9A01A_YELLOW);
    tft.setTextSize(2);
    tft.println("---");
  } 
  else if (bpm == 0) {
    tft.setCursor(50, 95);
    tft.setTextColor(GC9A01A_CYAN);
    tft.setTextSize(2);
    tft.println("WAIT");
  } 
  else {
    tft.setTextSize(5);
    tft.setTextColor(GC9A01A_GREEN);

    String bpmStr = String((int)bpm);
    int16_t x = 120 - (bpmStr.length() * 30) / 2;

    tft.setCursor(x, 85);
    tft.println(bpmStr);
  }
}

void updateSignalIndicator(bool hasContact, long signal) {
  int startX = 80;
  int startY = 220;

  tft.fillRect(startX, startY - 20, 80, 20, GC9A01A_BLACK);

  if (!hasContact) {
    tft.setCursor(startX - 10, startY - 15);
    tft.setTextColor(GC9A01A_RED);
    tft.setTextSize(1);
    tft.println("NO SIGNAL");
    return;
  }

  int strength = 0;
  if (signal > 15000) strength = 1;
  if (signal > 30000) strength = 2;
  if (signal > 50000) strength = 3;
  if (signal > 80000) strength = 4;
  if (signal > 100000) strength = 5;

  for (int i = 0; i < 5; i++) {
    int barHeight = (i + 1) * 3;
    uint16_t color = (i < strength) ? GC9A01A_GREEN : GC9A01A_DARKGREY;

    tft.fillRect(startX + i * 12, startY - barHeight, 8, barHeight, color);
  }
}

void updateIMUDisplay() {
  tft.fillRect(50, 10, 180, 35, GC9A01A_BLACK);

  tft.setCursor(50, 10);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(1);

  tft.print("X="); tft.print(ax_g, 1);
  tft.print(" Y="); tft.print(ay_g, 1);
  tft.print(" Z="); tft.print(az_g, 1);
}

void drawHeartIcon(int x, int y, uint16_t color) {
  tft.fillCircle(x - 8, y - 5, 6, color);
  tft.fillCircle(x + 8, y - 5, 6, color);
  tft.fillTriangle(x - 14, y - 2, x, y + 10, x + 14, y - 2, color);
}