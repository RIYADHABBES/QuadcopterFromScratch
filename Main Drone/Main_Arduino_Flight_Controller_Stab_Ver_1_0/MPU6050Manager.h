#ifndef MPU6050MANAGER_H
#define MPU6050MANAGER_H

#include <Wire.h> // library for I2C communication with the GY-521/MPU-6050


class MPU6050Manager {

public:



private:

void initializeMPU6050(); // MPU6050 circuit initialization function by I2C

void calibrateMPU6050();  // MPU6050 calibration function : calculation of gyroscope and accelerometer offsets

void readMPU6050();      // reading MPU6050 data

void MPU6050Manager::calculateAnglesFusion(); // calculation of Roll and Pitch angles by "Sensor Fusion"    
};

#endif //MPU6050MANAGER_H