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
  unsigned long current_time = micros();
  float dt = (current_time - m_time_storage) / 1000000.0f;
  m_time_storage = current_time;

  // Remove offsets and scale
  float gyroX = (m_gyro_raw[ROLL] - m_gyro_offset[ROLL]) / SF_Gyro;
  float gyroY = (m_gyro_raw[PITCH] - m_gyro_offset[PITCH]) / SF_Gyro;
  float gyroZ = (m_gyro_raw[YAW] - m_gyro_offset[YAW]) / SF_Gyro;

  float accelX = (m_accel_raw[X] - m_accel_offset[X]) / SF_Accel;
  float accelY = (m_accel_raw[Y] - m_accel_offset[Y]) / SF_Accel;
  float accelZ = (m_accel_raw[Z] - m_accel_offset[Z]) / SF_Accel;

  // Integrate gyro angles
  m_angle_gyro[ROLL] += gyroX * dt;
  m_angle_gyro[PITCH] += gyroY * dt;
  m_angle_gyro[YAW] += gyroZ * dt;

  // Compute accel angles
  float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

  const float alpha = 0.98;
  if (m_init_gyro_angles) {
    m_angle[ROLL] = alpha * (m_angle[ROLL] + gyroX * dt) + (1 - alpha) * accelAngleX;
    m_angle[PITCH] = alpha * (m_angle[PITCH] + gyroY * dt) + (1 - alpha) * accelAngleY;
  } else {
    m_angle[ROLL] = accelAngleX;
    m_angle[PITCH] = accelAngleY;
    m_init_gyro_angles = true;
  }

  m_angle[YAW] = m_angle_gyro[YAW]; // YAW from gyro only

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
