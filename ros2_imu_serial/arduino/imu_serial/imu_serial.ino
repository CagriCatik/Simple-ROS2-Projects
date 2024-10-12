#include <Wire.h>

const int MPU = 0x68;  // MPU6050 I2C address
const float ACCEL_SCALE = 16384.0;  // Scale factor for accelerometer (±2g)
const float GYRO_SCALE = 131.0;  // Scale factor for gyroscope (±250 deg/s)
const int ACCEL_REG_START = 0x3B;  // Accelerometer register start address
const int GYRO_REG_START = 0x43;   // Gyroscope register start address

// IMU variables
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY;
float gyroAngleX = 0, gyroAngleY = 0, yaw = 0;
float roll, pitch;
float AccErrorX = 0, AccErrorY = 0, GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;
float elapsedTime, currentTime, previousTime = 0;

int errorStatus = 0;  // Status of I2C communication

void setup() {
  Serial.begin(19200);
  Wire.begin();
  
  // Initialize MPU6050 and check status
  errorStatus = init_MPU6050();
  if (errorStatus != 0) {
    Serial.println("Failed to initialize MPU6050!");
    while (1); // Stop execution if failed
  } else {
    Serial.println("MPU6050 initialized successfully!");
  }

  // Calculate and log IMU error offsets
  calculate_IMU_error();
  delay(20);
}

void loop() {
  // Time management for sensor fusion
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Time in seconds
  previousTime = currentTime;

  // Read IMU data
  readAccelerometerData();
  readGyroscopeData();

  // Complementary filter - combine accelerometer and gyroscope data
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Print values for Serial Plotter with labels
  Serial.print("Roll: ");   // Label for Roll
  Serial.print(roll);
  Serial.print(", Pitch: "); // Label for Pitch
  Serial.print(pitch);
  Serial.print(", Yaw: ");   // Label for Yaw
  Serial.println(yaw);

  // Add a small delay for better visualization
  delay(10);
}



int init_MPU6050() {
  Wire.beginTransmission(MPU);
  Wire.write(0x6B); // Power management register
  Wire.write(0x00); // Wake up MPU6050
  errorStatus = Wire.endTransmission(true);  // End I2C transmission and return error status

  if (errorStatus != 0) {
    return errorStatus; // Return error code if communication fails
  }

  // Optionally, configure gyro and accelerometer ranges (±250deg/s and ±2g already set by default)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B); // Gyro config register
  Wire.write(0x00); // ±250deg/s
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU);
  Wire.write(0x1C); // Accelerometer config register
  Wire.write(0x00); // ±2g
  return Wire.endTransmission(true);
}

void readAccelerometerData() {
  Wire.beginTransmission(MPU);
  Wire.write(ACCEL_REG_START); // Start with ACCEL_XOUT register
  errorStatus = Wire.endTransmission(false);
  if (errorStatus != 0) {
    Serial.println("Failed to read accelerometer data!");
    return;
  }

  Wire.requestFrom(MPU, 6, true);  // Request 6 bytes of data from accelerometer
  AccX = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  AccY = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  AccZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;

  // Calculating angles using accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX;
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
}

void readGyroscopeData() {
  Wire.beginTransmission(MPU);
  Wire.write(GYRO_REG_START);  // Start with GYRO_XOUT register
  errorStatus = Wire.endTransmission(false);
  if (errorStatus != 0) {
    Serial.println("Failed to read gyroscope data!");
    return;
  }

  Wire.requestFrom(MPU, 6, true);  // Request 6 bytes of data from gyroscope
  GyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - GyroErrorX;
  GyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - GyroErrorY;
  GyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - GyroErrorZ;

  // Integrating gyroscope data to get angles
  gyroAngleX += GyroX * elapsedTime;
  gyroAngleY += GyroY * elapsedTime;
  yaw += GyroZ * elapsedTime;
}

void calculate_IMU_error() {
  int c = 0;

  // Accumulate accelerometer error over 200 samples
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(ACCEL_REG_START);  // ACCEL_XOUT register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
    AccY = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
    AccZ = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
    AccErrorX += (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI);
    AccErrorY += (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI);
    c++;
  }
  AccErrorX /= 200;
  AccErrorY /= 200;

  c = 0; // Reset counter

  // Accumulate gyroscope error over 200 samples
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(GYRO_REG_START);  // GYRO_XOUT register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    GyroY = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    GyroZ = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  GyroErrorX /= 200;
  GyroErrorY /= 200;
  GyroErrorZ /= 200;

  // Log error values to Serial Monitor
  Serial.println("IMU Errors Calculated:");
  Serial.print("AccErrorX: "); Serial.println(AccErrorX);
  Serial.print("AccErrorY: "); Serial.println(AccErrorY);
  Serial.print("GyroErrorX: "); Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: "); Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: "); Serial.println(GyroErrorZ);
}
