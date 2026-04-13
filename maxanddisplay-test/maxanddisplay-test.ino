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

// =====================================================

MAX30105 sensor;
Adafruit_GC9A01A tft(TFT_CS, TFT_DC);

// ================= HEART RATE SETTINGS ===============
#define AC_THRESHOLD 15
#define MIN_BPM 45
#define MAX_BPM 160
#define DC_ALPHA 0.95
#define PEAK_LOCKOUT_MS 300
#define BPM_HISTORY 5
// =====================================================

// Heart rate data
long dcAvg = 0;
bool aboveThreshold = false;
unsigned long lastBeatTime = 0;
float currentBPM = 0;
float avgBPM = 0;
float bpmHist[BPM_HISTORY];
int bpmIdx = 0;

// Display animation
int pulseRadius = 0;
bool pulsing = false;
unsigned long pulseStartTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Heart Rate Monitor with Display");
  
  // ========== INITIALIZE I2C ==========
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  
  // ========== INITIALIZE DISPLAY ==========
  Serial.print("Initializing display... ");
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(GC9A01A_BLACK);
  Serial.println("OK!");
  
  // Welcome screen
  drawWelcomeScreen();
  delay(2000);
  
  // ========== INITIALIZE MAX30102 ==========
  Serial.print("Initializing MAX30102... ");
  if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
    Serial.println("FAILED!");
    tft.fillScreen(GC9A01A_BLACK);
    tft.setCursor(40, 100);
    tft.setTextColor(GC9A01A_RED);
    tft.setTextSize(2);
    tft.println("SENSOR");
    tft.setCursor(50, 130);
    tft.println("ERROR!");
    while (1);
  }
  Serial.println("OK!");
  
  // Configure sensor for wrist detection
  sensor.setup(0xFF, 4, 2, 400, 411, 16384);
  sensor.setPulseAmplitudeIR(0xFF);
  sensor.setPulseAmplitudeRed(0x00);
  
  // Warm up sensor
  Serial.print("Warming up");
  for (int i = 0; i < 100; i++) {
    long raw = sensor.getIR();
    dcAvg = raw;
    if (i % 20 == 0) Serial.print(".");
    delay(10);
  }
  Serial.println(" Done!");
  
  // Draw main UI
  drawMainUI();
  
  Serial.println("\nReady! Place finger/wrist on sensor");
}

void loop() {
  unsigned long currentTime = millis();
  
  // ========== READ HEART RATE SENSOR ==========
  long rawIR = sensor.getIR();
  bool hasContact = (rawIR > 30000);
  
  if (hasContact) {
    // DC removal
    dcAvg = DC_ALPHA * dcAvg + (1.0 - DC_ALPHA) * rawIR;
    long ac = rawIR - dcAvg;
    
    // Peak detection
    if (!aboveThreshold && ac > AC_THRESHOLD) {
      if (currentTime - lastBeatTime > PEAK_LOCKOUT_MS) {
        if (lastBeatTime > 0) {
          unsigned long interval = currentTime - lastBeatTime;
          float newBPM = 60000.0 / interval;
          
          if (newBPM >= MIN_BPM && newBPM <= MAX_BPM) {
            bool isOutlier = false;
            if (avgBPM > 0 && abs(newBPM - avgBPM) > 25) {
              isOutlier = true;
            }
            
            if (!isOutlier) {
              bpmHist[bpmIdx] = newBPM;
              bpmIdx = (bpmIdx + 1) % BPM_HISTORY;
              
              float sum = 0;
              int count = 0;
              for (int i = 0; i < BPM_HISTORY; i++) {
                if (bpmHist[i] > 0) {
                  sum += bpmHist[i];
                  count++;
                }
              }
              avgBPM = sum / count;
              currentBPM = newBPM;
              
              // Trigger pulse animation
              pulsing = true;
              pulseStartTime = currentTime;
              
              Serial.print("💓 ");
              Serial.println(avgBPM, 1);
            }
          }
        }
        lastBeatTime = currentTime;
      }
      aboveThreshold = true;
    }
    
    if (ac < -AC_THRESHOLD / 2) {
      aboveThreshold = false;
    }
  }
  
  // ========== UPDATE DISPLAY ==========
  static unsigned long lastDisplayUpdate = 0;
  if (currentTime - lastDisplayUpdate > 100) { // Update at 10Hz
    lastDisplayUpdate = currentTime;
    
    // Update BPM number
    updateBPMDisplay(avgBPM, hasContact);
    
    // Pulse animation
    if (pulsing) {
      unsigned long elapsed = currentTime - pulseStartTime;
      if (elapsed < 300) {
        // Expand
        pulseRadius = map(elapsed, 0, 150, 0, 30);
        if (elapsed > 150) {
          // Contract
          pulseRadius = map(elapsed, 150, 300, 30, 0);
        }
        drawPulseAnimation(pulseRadius);
      } else {
        pulsing = false;
        drawPulseAnimation(0); // Clear
      }
    }
    
    // Update signal indicator
    updateSignalIndicator(hasContact, rawIR);
  }
  
  delay(10);
}

void drawWelcomeScreen() {
  tft.fillScreen(GC9A01A_BLACK);
  
  // Draw heart icon
  tft.fillCircle(100, 80, 20, GC9A01A_RED);
  tft.fillCircle(140, 80, 20, GC9A01A_RED);
  tft.fillTriangle(80, 85, 120, 140, 160, 85, GC9A01A_RED);
  
  // Title
  tft.setCursor(55, 160);
  tft.setTextColor(GC9A01A_WHITE);
  tft.setTextSize(2);
  tft.println("HEART");
  
  tft.setCursor(50, 185);
  tft.println("MONITOR");
}

void drawMainUI() {
  tft.fillScreen(GC9A01A_BLACK);
  
  // Draw decorative circle
  tft.drawCircle(120, 120, 100, GC9A01A_BLUE);
  tft.drawCircle(120, 120, 98, GC9A01A_BLUE);
  
  // Label
  tft.setCursor(85, 30);
  tft.setTextColor(GC9A01A_CYAN);
  tft.setTextSize(2);
  tft.println("BPM");
  
  // Heart icon
  drawHeartIcon(120, 180, GC9A01A_RED);
}

void updateBPMDisplay(float bpm, bool hasContact) {
  // Clear previous number
  tft.fillRect(40, 70, 160, 60, GC9A01A_BLACK);
  
  if (!hasContact) {
    // No contact message
    tft.setCursor(70, 95);
    tft.setTextColor(GC9A01A_YELLOW);
    tft.setTextSize(2);
    tft.println("---");
  } else if (bpm == 0) {
    // Detecting
    tft.setCursor(50, 95);
    tft.setTextColor(GC9A01A_CYAN);
    tft.setTextSize(2);
    tft.println("WAIT");
  } else {
    // Show BPM
    tft.setTextSize(5);
    tft.setTextColor(GC9A01A_GREEN);
    
    // Center the text
    String bpmStr = String((int)bpm);
    int16_t x = 120 - (bpmStr.length() * 30) / 2;
    tft.setCursor(x, 85);
    tft.println(bpmStr);
  }
}

void drawPulseAnimation(int radius) {
  // Clear previous
  static int lastRadius = 0;
  if (lastRadius > 0) {
    tft.drawCircle(120, 180, lastRadius, GC9A01A_BLACK);
    tft.drawCircle(120, 180, lastRadius + 1, GC9A01A_BLACK);
  }
  
  // Draw new pulse ring
  if (radius > 0) {
    uint8_t alpha = map(radius, 0, 30, 255, 50);
    uint16_t color = tft.color565(255, 0, alpha);
    tft.drawCircle(120, 180, radius, color);
    tft.drawCircle(120, 180, radius + 1, color);
  }
  
  lastRadius = radius;
}

void drawHeartIcon(int x, int y, uint16_t color) {
  // Small heart
  tft.fillCircle(x - 8, y - 5, 6, color);
  tft.fillCircle(x + 8, y - 5, 6, color);
  tft.fillTriangle(x - 14, y - 2, x, y + 10, x + 14, y - 2, color);
}

void updateSignalIndicator(bool hasContact, long signal) {
  // Signal strength bars at bottom
  int barWidth = 8;
  int barGap = 4;
  int startX = 80;
  int startY = 220;
  
  // Clear area
  tft.fillRect(startX, startY - 20, 80, 20, GC9A01A_BLACK);
  
  if (!hasContact) {
    // Show "NO SIGNAL"
    tft.setCursor(startX - 10, startY - 15);
    tft.setTextColor(GC9A01A_RED);
    tft.setTextSize(1);
    tft.println("NO SIGNAL");
    return;
  }
  
  // Calculate signal strength (0-5 bars)
  int strength = 0;
  if (signal > 30000) strength = 1;
  if (signal > 50000) strength = 2;
  if (signal > 80000) strength = 3;
  if (signal > 100000) strength = 4;
  if (signal > 120000) strength = 5;
  
  // Draw bars
  for (int i = 0; i < 5; i++) {
    int barHeight = (i + 1) * 3;
    uint16_t color = (i < strength) ? GC9A01A_GREEN : GC9A01A_DARKGREY;
    
    tft.fillRect(startX + i * (barWidth + barGap), 
                 startY - barHeight, 
                 barWidth, 
                 barHeight, 
                 color);
  }
}
