#include <Wire.h>
#include "MAX30105.h"

MAX30105 sensor;

// ================= CALIBRATION SETTINGS =================
#define SDA_PIN 8
#define SCL_PIN 9

#define SAMPLE_DELAY_MS 10
#define MIN_BPM 45
#define MAX_BPM 160

// TUNE THESE FOR YOUR WRIST:
#define AC_THRESHOLD 15        // Lower = more sensitive (try 10-30)
#define PEAK_LOCKOUT_MS 300    // Minimum time between beats (300ms = 200 BPM max)
#define DC_ALPHA 0.95          // DC filter: 0.95=slower adapt, 0.90=faster adapt

// LED settings (try different values if not working)
#define LED_BRIGHTNESS 0xFF    // 0x3F=low, 0x7F=med, 0xFF=max
#define USE_RED_LED false      // false=IR (default), true=RED (try if IR fails)

// Advanced filtering
#define USE_BANDPASS true      // Enable bandpass filter (reduces noise)
#define BPM_HISTORY 5          // More history = smoother but slower
#define BPM_OUTLIER_REJECT 25  // Reject BPM that differs by more than this from avg
// ========================================================

// DC removal filter
long dcAvg = 0;

// Bandpass filter (optional - removes low freq drift and high freq noise)
long bp1 = 0, bp2 = 0, bp3 = 0;
const float BP_ALPHA = 0.9;

// Peak detection
bool aboveThreshold = false;
unsigned long lastBeatTime = 0;
float currentBPM = 0;
float avgBPM = 0;

// BPM history
float bpmHist[BPM_HISTORY];
int bpmIdx = 0;
int validBeats = 0;

// Signal quality tracking
long minAC = 0, maxAC = 0;
unsigned long lastQualityCheck = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  if (!sensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30102 not found!");
    while (1);
  }

  // Optimized wrist setup
  sensor.setup(
    LED_BRIGHTNESS,
    4,        // sample average
    2,        // mode 2 = RED+IR
    400,      // sample rate (higher = better for HR)
    411,      // pulse width
    16384     // ADC range (max sensitivity)
  );

  if (USE_RED_LED) {
    sensor.setPulseAmplitudeRed(LED_BRIGHTNESS);
    sensor.setPulseAmplitudeIR(0x00);
    Serial.println("Using RED LED");
  } else {
    sensor.setPulseAmplitudeIR(LED_BRIGHTNESS);
    sensor.setPulseAmplitudeRed(0x00);
    Serial.println("Using IR LED");
  }

  Serial.println("\n=== MAX30102 Wrist HR Monitor ===");
  Serial.println("Settings:");
  Serial.print("  AC Threshold: "); Serial.println(AC_THRESHOLD);
  Serial.print("  LED Brightness: 0x"); Serial.println(LED_BRIGHTNESS, HEX);
  Serial.print("  Bandpass Filter: "); Serial.println(USE_BANDPASS ? "ON" : "OFF");
  Serial.println("\n📍 Keep wrist STILL for accurate reading!");
  Serial.println("Warming up sensor...\n");
  
  // Warm up and initialize DC filter
  for (int i = 0; i < 200; i++) {
    long raw = USE_RED_LED ? sensor.getRed() : sensor.getIR();
    dcAvg = raw;
    delay(10);
  }
  
  Serial.println("Ready!\n");
}

void loop() {
  // Read sensor
  long raw = USE_RED_LED ? sensor.getRed() : sensor.getIR();

  // Check for valid contact
  if (raw < 20000) {
    Serial.println("⚠️  No contact detected");
    delay(500);
    return;
  }

  // === DC REMOVAL (High-pass filter) ===
  dcAvg = DC_ALPHA * dcAvg + (1.0 - DC_ALPHA) * raw;
  long ac = raw - dcAvg;

  // === OPTIONAL BANDPASS FILTER ===
  // Removes very slow drift and very fast noise
  if (USE_BANDPASS) {
    bp3 = bp2;
    bp2 = bp1;
    bp1 = ac;
    ac = (bp1 + bp2 + bp3) / 3;  // Simple 3-point moving average
  }

  // === PEAK DETECTION ===
  unsigned long now = millis();
  
  if (!aboveThreshold && ac > AC_THRESHOLD) {
    // Rising edge detected
    if (now - lastBeatTime > PEAK_LOCKOUT_MS) {
      if (lastBeatTime > 0) {
        unsigned long interval = now - lastBeatTime;
        float newBPM = 60000.0 / interval;

        // Validate BPM range
        if (newBPM >= MIN_BPM && newBPM <= MAX_BPM) {
          
          // Outlier rejection (if we have history)
          bool isOutlier = false;
          if (avgBPM > 0 && abs(newBPM - avgBPM) > BPM_OUTLIER_REJECT) {
            isOutlier = true;
            Serial.print("⚠️  Outlier rejected: ");
            Serial.print(newBPM, 1);
            Serial.print(" (avg: ");
            Serial.print(avgBPM, 1);
            Serial.println(")");
          }
          
          if (!isOutlier) {
            // Add to history
            bpmHist[bpmIdx] = newBPM;
            bpmIdx = (bpmIdx + 1) % BPM_HISTORY;
            validBeats++;

            // Calculate average
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

            Serial.print("💓 Beat! Instant: ");
            Serial.print(currentBPM, 1);
            Serial.print(" | Avg: ");
            Serial.print(avgBPM, 1);
            Serial.print(" | Beats: ");
            Serial.println(validBeats);
          }
        } else {
          Serial.print("⚠️  Out of range: ");
          Serial.print(newBPM, 1);
          Serial.println(" BPM");
        }
      }
      lastBeatTime = now;
    }
    aboveThreshold = true;
  }

  // Reset threshold flag when signal goes negative
  if (ac < -AC_THRESHOLD / 2) {
    aboveThreshold = false;
  }

  // === TRACK SIGNAL QUALITY ===
  if (ac < minAC) minAC = ac;
  if (ac > maxAC) maxAC = ac;

  // === PERIODIC STATUS ===
  if (now - lastQualityCheck > 3000) {
    lastQualityCheck = now;
    
    long acRange = maxAC - minAC;
    
    Serial.print("\n📊 Status | Raw: ");
    Serial.print(raw);
    Serial.print(" | AC Range: ");
    Serial.print(acRange);
    Serial.print(" | Threshold: ");
    Serial.print(AC_THRESHOLD);
    
    if (avgBPM > 0) {
      Serial.print(" | ❤️ ");
      Serial.print(avgBPM, 1);
      Serial.println(" BPM");
    } else {
      Serial.println(" | Searching...");
    }
    
    // Signal quality feedback
    if (acRange < AC_THRESHOLD * 2) {
      Serial.println("⚠️  AC signal too weak!");
      Serial.println("   Try: Tighter fit, different position, or increase LED_BRIGHTNESS");
    } else if (acRange < AC_THRESHOLD * 5) {
      Serial.println("⚠️  Weak signal. Optimize placement for better results.");
    } else if (acRange > AC_THRESHOLD * 10) {
      Serial.println("✓ Good signal strength!");
    }
    
    // Calibration suggestion
    if (acRange > 0) {
      int suggestedThreshold = acRange / 4;
      if (suggestedThreshold != AC_THRESHOLD) {
        Serial.print("💡 Suggested AC_THRESHOLD: ");
        Serial.println(suggestedThreshold);
      }
    }
    
    minAC = 0;
    maxAC = 0;
    Serial.println();
  }

  // === DEBUG OUTPUT (comment out for cleaner serial) ===
  /*
  Serial.print("Raw: ");
  Serial.print(raw);
  Serial.print(" | DC: ");
  Serial.print(dcAvg);
  Serial.print(" | AC: ");
  Serial.print(ac);
  Serial.print(" | BPM: ");
  if (avgBPM > 0) {
    Serial.println(avgBPM, 1);
  } else {
    Serial.println("---");
  }
  */

  delay(SAMPLE_DELAY_MS);
}


// =====================================================
// CALIBRATION GUIDE FOR YOUR WRIST:
// =====================================================
//
// 1. AC_THRESHOLD (most important!):
//    - Watch the "AC Range" in status output
//    - Set AC_THRESHOLD to about 25-30% of AC Range
//    - Example: AC Range = 60 → try AC_THRESHOLD = 15-20
//    - Too low = false beats, too high = missed beats
//
// 2. LED_BRIGHTNESS:
//    - Start at 0xFF (maximum)
//    - If Raw value > 150,000 = too bright, lower to 0x7F
//    - If Raw value < 50,000 = too dim, keep at 0xFF
//    - Sweet spot: Raw around 80,000-120,000
//
// 3. DC_ALPHA:
//    - 0.95 = slow adaptation (stable but slow to adjust)
//    - 0.90 = faster adaptation (tracks changes better)
//    - For wrist: 0.92-0.95 is usually best
//
// 4. USE_RED_LED vs IR:
//    - IR (false) usually better for wrist
//    - Try RED (true) if IR gives poor results
//    - Different people respond differently
//
// 5. BPM_OUTLIER_REJECT:
//    - 25 = moderate filtering
//    - 15 = strict (only accept very consistent readings)
//    - 40 = loose (accept more variation)
//
// 6. POSITIONING (most critical!):
//    - Inner wrist, 1-2 fingers from wrist bone
//    - Snug but not too tight
//    - Keep COMPLETELY still for 30+ seconds
//    - Try different spots - even 5mm can matter!
//
// TROUBLESHOOTING:
// - "AC signal too weak" → Increase LED_BRIGHTNESS or reposition
// - Many outliers → Increase BPM_OUTLIER_REJECT
// - No beats detected → Lower AC_THRESHOLD
// - False beats → Raise AC_THRESHOLD
// - Use the "Suggested AC_THRESHOLD" as starting point
// =====================================================