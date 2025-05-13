#include "MPU6050Manager.h"

#include <Arduino.h>
#include <Wire.h>

#define MPU_ADDRESS 0x68

#define X      0
#define Y      1
#define Z      2

#define ROLL  0
#define PITCH 1
#define YAW   2

#define LED 13

#define ESC1_pin 5
#define ESC2_pin 6
#define ESC3_pin 9
#define ESC4_pin 3

static constexpr float SF_Gyro = 65.5;    // gyro scale factor for ±500°/s
static constexpr float SF_Accel = 4096;   // accel scale factor for ±8g

static constexpr float stabilization_coeff = 3.0;

MPU6050Manager::MPU6050Manager() {
  initializeMPU6050();
  calibrateMPU6050();
  m_time_storage = micros();
  readMPU6050();
}

void MPU6050Manager::initializeMPU6050() {
  Serial.println("MPU6050 initialization began ...");

  Wire.begin();
  TWBR = 12; // Set I2C to 400kHz

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Gyroscope config (±500°/s)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Accelerometer config (±8g)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();

  // Set DLPF to ~20Hz (cut off high-frequency noise)
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x04); // DLPF setting (20Hz bandwidth, 1kHz rate)
  Wire.endTransmission();

  delay(250);
  Serial.println("MPU6050 had been initialized !");
}

void MPU6050Manager::calibrateMPU6050() {
  Serial.println("MPU6050 Calibration began ...");

  const int nb_values = 2000;

  for (int i = 0; i < nb_values; ++i) {
    if (i % 20 == 0) digitalWrite(LED, !digitalRead(LED));
    readMPU6050();

    m_gyro_offset[ROLL] += m_gyro_raw[ROLL];
    m_gyro_offset[PITCH] += m_gyro_raw[PITCH];
    m_gyro_offset[YAW] += m_gyro_raw[YAW];

    m_accel_offset[X] += m_accel_raw[X];
    m_accel_offset[Y] += m_accel_raw[Y];
    m_accel_offset[Z] += m_accel_raw[Z];

    digitalWrite(ESC1_pin, HIGH);
    digitalWrite(ESC2_pin, HIGH);
    digitalWrite(ESC3_pin, HIGH);
    digitalWrite(ESC4_pin, HIGH);
    delayMicroseconds(1000);
    digitalWrite(ESC1_pin, LOW);
    digitalWrite(ESC2_pin, LOW);
    digitalWrite(ESC3_pin, LOW);
    digitalWrite(ESC4_pin, LOW);
    delay(3);
  }

  m_gyro_offset[ROLL] /= nb_values;
  m_gyro_offset[PITCH] /= nb_values;
  m_gyro_offset[YAW] /= nb_values;

  m_accel_offset[X] /= nb_values;
  m_accel_offset[Y] /= nb_values;
  m_accel_offset[Z] = m_accel_offset[Z] / nb_values - 4096;

  Serial.println("MPU6050 had been calibrated !");
}

void MPU6050Manager::readMPU6050() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(MPU_ADDRESS, 14);

  while (Wire.available() < 14);

  m_accel_raw[X] = Wire.read() << 8 | Wire.read();
  m_accel_raw[Y] = Wire.read() << 8 | Wire.read();
  m_accel_raw[Z] = Wire.read() << 8 | Wire.read();
  m_temperature = Wire.read() << 8 | Wire.read();
  m_gyro_raw[ROLL] = Wire.read() << 8 | Wire.read();
  m_gyro_raw[PITCH] = Wire.read() << 8 | Wire.read();
  m_gyro_raw[YAW] = Wire.read() << 8 | Wire.read();

  m_gyro_raw[PITCH] = -m_gyro_raw[PITCH];
  m_gyro_raw[YAW] = -m_gyro_raw[YAW];
  m_accel_raw[X] = -m_accel_raw[X];
}

void MPU6050Manager::calculateAnglesFusion() {
    // Calculate time delta in seconds
    unsigned long currentTime = micros();
    float dt = (currentTime - m_time_storage) / 1000000.0f;
    m_time_storage = currentTime;

    // Raw angular velocity (deg/s)
    float rawGyroX = (m_gyro_raw[ROLL]  - m_gyro_offset[ROLL])  / SF_Gyro;
    float rawGyroY = (m_gyro_raw[PITCH] - m_gyro_offset[PITCH]) / SF_Gyro;
    float rawGyroZ = (m_gyro_raw[YAW]   - m_gyro_offset[YAW])   / SF_Gyro;

    // Low-pass filter: smoother gyro angular velocities
    const float smoothing = 0.7f;

    m_gyro[ROLL]  = smoothing * m_gyro[ROLL]  + (1 - smoothing) * rawGyroX;
    m_gyro[PITCH] = smoothing * m_gyro[PITCH] + (1 - smoothing) * rawGyroY;
    m_gyro[YAW]   = smoothing * m_gyro[YAW]   + (1 - smoothing) * rawGyroZ;

    // Deadband: suppress tiny twitchy values
    if (abs(m_gyro[ROLL])  < 0.2f) m_gyro[ROLL] = 0;
    if (abs(m_gyro[PITCH]) < 0.2f) m_gyro[PITCH] = 0;
    if (abs(m_gyro[YAW])   < 0.2f) m_gyro[YAW]   = 0;

    // Accelerometer: convert to g
    float accelX = (m_accel_raw[X] - m_accel_offset[X]) / SF_Accel;
    float accelY = (m_accel_raw[Y] - m_accel_offset[Y]) / SF_Accel;
    float accelZ = (m_accel_raw[Z] - m_accel_offset[Z]) / SF_Accel;

    // Store scaled accel values
    m_accel[X] = accelX;
    m_accel[Y] = accelY;
    m_accel[Z] = accelZ;

    // Compute roll & pitch from accelerometer
    float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;
    float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

    m_angle_accel[ROLL]  = accelAngleX;
    m_angle_accel[PITCH] = accelAngleY;

    // Integrate filtered gyro values
    m_angle_gyro[ROLL]  += m_gyro[ROLL]  * dt;
    m_angle_gyro[PITCH] += m_gyro[PITCH] * dt;
    m_angle_gyro[YAW]   += m_gyro[YAW]   * dt;

    // Sensor fusion using complementary filter
    const float alpha = 0.98f;

    if (m_init_gyro_angles) {
        m_angle[ROLL]  = alpha * (m_angle[ROLL]  + m_gyro[ROLL]  * dt) + (1 - alpha) * accelAngleX;
        m_angle[PITCH] = alpha * (m_angle[PITCH] + m_gyro[PITCH] * dt) + (1 - alpha) * accelAngleY;
    } else {
        m_angle[ROLL]  = accelAngleX;
        m_angle[PITCH] = accelAngleY;
        m_init_gyro_angles = true;
    }

    // Yaw angle from gyro only
    m_angle[YAW] = m_angle_gyro[YAW];

    // Apply stabilization correction
    m_roll_adjustment = m_angle[ROLL] * stabilization_coeff;
    m_pitch_adjustment = m_angle[PITCH] * stabilization_coeff;

    if (!m_selflevel_mode) {
        m_roll_adjustment = 0;
        m_pitch_adjustment = 0;
    }
}


float const * const MPU6050Manager::getGyro() const {
  return m_gyro;
}

float MPU6050Manager::getRollAdjustment() const {
  return m_roll_adjustment;
}

float MPU6050Manager::getPitchAdjustment() const {
  return m_pitch_adjustment;
}

void MPU6050Manager::printAngles() {
  Serial.print("ROLL:");
  Serial.print(m_angle[ROLL]);
  Serial.print(",");
  Serial.print("PITCH:");
  Serial.print(m_angle[PITCH]);
  Serial.print(",");
  Serial.print("YAW:");
  Serial.println(m_angle[YAW]);
}

void MPU6050Manager::printAnglularVelocity() {
  Serial.print("Angular Velocity [deg/s] => ");
  
  Serial.print("ROLL:");
  Serial.print(m_gyro[ROLL]);  
  Serial.print(",");
  
  Serial.print("PITCH:");
  Serial.print(m_gyro[PITCH]);
  Serial.print(",");

  Serial.print("YAW:");
  Serial.println(m_gyro[YAW]);
}
