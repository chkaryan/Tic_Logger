// ========================================
// MPU6500 Working Test - CONFIRMED WORKING
// ========================================
// Address: 0x68
// SDA: GPIO6, SCL: GPIO7

#include <Wire.h>

#define MPU6500_ADDR 0x68
#define SDA_PIN 8
#define SCL_PIN 9

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t temp;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== MPU6500 Test ===");
  
  // CRITICAL: Specify the I2C pins!
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Wake up MPU6500
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up
  Wire.endTransmission(true);
  
  delay(100);
  
  // Configure accelerometer (±8g range)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG register
  Wire.write(0x10);  // ±8g range (AFS_SEL=2)
  Wire.endTransmission(true);
  
  // Configure gyroscope (±500°/s range)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG register
  Wire.write(0x08);  // ±500°/s range (FS_SEL=1)
  Wire.endTransmission(true);
  
  // Verify WHO_AM_I
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 1, true);
  byte whoAmI = Wire.read();
  
  Serial.print("WHO_AM_I: 0x");
  Serial.println(whoAmI, HEX);
  
  if (whoAmI == 0x70) {
    Serial.println("✓ MPU6500 initialized successfully!");
  } else {
    Serial.println("⚠ Unexpected WHO_AM_I value");
  }
  
  Serial.println("\nAccel range: ±8g");
  Serial.println("Gyro range: ±500°/s");
  Serial.println("\nReading data...\n");
  delay(500);
}

void loop() {
  // Read accelerometer (6 bytes)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 6, true);
  
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  
  // Read temperature (2 bytes)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x41);  // TEMP_OUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 2, true);
  
  temp = (Wire.read() << 8) | Wire.read();
  
  // Read gyroscope (6 bytes)
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x43);  // GYRO_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 6, true);
  
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
  
  // Convert to physical units
  // Accel: ±8g range → 4096 LSB/g
  float ax_g = ax / 4096.0;
  float ay_g = ay / 4096.0;
  float az_g = az / 4096.0;
  
  // Gyro: ±500°/s range → 65.5 LSB/(°/s)
  float gx_dps = gx / 65.5;
  float gy_dps = gy / 65.5;
  float gz_dps = gz / 65.5;
  
  // Temp: (TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity + 21°C
  float temperature = (temp / 333.87) + 21.0;
  
  // Calculate total acceleration magnitude
  float accel_magnitude = sqrt(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  
  // Print data
  Serial.print("Accel (g): ");
  Serial.print("X="); Serial.print(ax_g, 2);
  Serial.print(" Y="); Serial.print(ay_g, 2);
  Serial.print(" Z="); Serial.print(az_g, 2);
  Serial.print(" |Mag|="); Serial.print(accel_magnitude, 2);
  
  Serial.print(" | Gyro (°/s): ");
  Serial.print("X="); Serial.print(gx_dps, 1);
  Serial.print(" Y="); Serial.print(gy_dps, 1);
  Serial.print(" Z="); Serial.print(gz_dps, 1);
  
  Serial.print(" | Temp: ");
  Serial.print(temperature, 1);
  Serial.println("°C");
  
  delay(100);  // 10Hz update rate
}

