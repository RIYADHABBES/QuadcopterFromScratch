
// ============================================================================
// ===                        SENSOR FUNCTIONS                              ===
// ============================================================================

void sensorInit(){
  //  ----------------------All Sensor Initialization----------------------------
  
  #ifdef I2C_FAST_MODE
  // set the TWI Bit Rate Register to 12 for 400kHz SCL speed
  // Default is 100kHz.
    TWBR = 12;  
  #endif
  
  Wire.begin();
  accInit();
  
  #if !defined(ARMED)
    // if ARMED, gyroInit will be called in motorInit
    gyroInit();  // if not ARMED, initialize gyro
  #endif
  
  #ifdef MAG
    magInit();
  #endif
}


void accInit(){
  // ----------------------ADXL345 Initialization------------------------------
  
  writeToSensor(ADXL345, 0x2D, 0);    // reset POWER_CTL
  writeToSensor(ADXL345, 0x2D, 16);   // POWER_CTL in standby mode
  writeToSensor(ADXL345, 0x2D, 8);    // POWER_CTL in measure mode
  
  writeToSensor(ADXL345, ADXL345_DATA_FORMAT, ADXL345_DATA_FORMAT_W);  // set resolution
  writeToSensor(ADXL345, ADXL345_BW_RATE, ADXL345_DATA_RATE);  // set bandwidth
  
  delay(500);
  
  #if defined CALIBRATE_ACC
    // calibration variables
    float accXReadings, accYReadings, accZReadings;
    
    Serial.print("Calibrating Accelerometer... ");
    
    for (byte i=0; i<ACC_CAL_SMPL_NUM; i++){
      updateAcc();
      accXReadings += accX;
      accYReadings += accY;
      accZReadings += accZ;
      delay(1000/ACC_SMPL_RATE);
    }
    
    delay(250);
    
    accXOffset = (int)(accXReadings/ACC_CAL_SMPL_NUM + 0.5);
    accYOffset = (int)(accYReadings/ACC_CAL_SMPL_NUM + 0.5);
    accZOffset = (int)(accZReadings/ACC_CAL_SMPL_NUM - 250 + 0.5);
    
    Serial.println("Done.");   
    
    Serial.println(accZReadings);
    
    Serial.print(accXOffset); Serial.print('\t');
    Serial.print(accYOffset); Serial.print('\t');
    Serial.println(accZOffset); 
  #else
    accXOffset = ACC_OFFSET_X;
    accYOffset = ACC_OFFSET_Y;
    accZOffset = ACC_OFFSET_Z;
  #endif
}


void gyroInit(){
  // ----------------------ITG3200 Initialization-----------------------------
  
  // set the gyroscope low pass filter cutoff
  writeToSensor(ITG3200, DLPF_FS, (DLPF_FS_SEL_1 | DLPF_FS_SEL_0 | DLPF_CFG_2 | DLPF_CFG_1 | DLPF_CFG_0));
  // set the gyro sample rate
  writeToSensor(ITG3200, SMPLRT_DIV, SMPLRT_DIV_VAL);
  
  delay(1000); // give the quad time to settle
  
  #ifndef OVERRIDE_GYRO_CALIBRATE
  // Calibrate the gyro offset by taking readings and averaging them
  
    // Calibration variables
    float gyroXReadings, gyroYReadings, gyroZReadings;
  
    Serial.print("Calibrating Gyro... ");
  
    for (byte i=0; i<GYRO_CAL_SMPL_NUM; i++){
      updateGyro();
      gyroXReadings += gyroX;
      gyroYReadings += gyroY;
      gyroZReadings += gyroZ;
      delay(1000/GYRO_SMPL_RATE);
    }
    
    gyroXOffset = gyroXReadings / GYRO_CAL_SMPL_NUM;
    gyroYOffset = gyroYReadings / GYRO_CAL_SMPL_NUM;
    gyroZOffset = gyroZReadings / GYRO_CAL_SMPL_NUM;
    
    
    Serial.println("Done.");
    Serial.print(gyroXOffset); Serial.print('\t');
    Serial.print(gyroYOffset); Serial.print('\t');
    Serial.println(gyroZOffset);
    
  #endif
  
  #ifdef OVERRIDE_GYRO_CALIBRATE  
    gyroXOffset = GYRO_OFFSET_X;
    gyroYOffset = GYRO_OFFSET_Y;
    gyroZOffset = GYRO_OFFSET_Z;
  #endif
}


#ifdef MAG
void magInit(){
  // ----------------------HMC5883L Initialization----------------------------
  writeToSensor(HMC5883L, CONFIG_A, CONFIG_A_SETTING);
  writeToSensor(HMC5883L, CONFIG_B, CONFIG_B_SETTING);
  writeToSensor(HMC5883L, MODE_REGISTER, MODE_SETTING);
}
#endif


void updateSensors(){
  /*
  -------------------------Sensor Update Function-----------------------------
  
  Function to gather new sensor readings from the accelerometer, gyroscope,
  and (maybe) magnetometer. 
  
  The readings are updated at the following frequencies:
    Accelerometer:   100 Hz
    Gyroscope:       500 Hz
    Magnetometer:    50 Hz
  
  */
  
  // variables for last update time of each sensor
  static unsigned long timeAcc, timeGyro, timeMag;
  
  if((loopTimeNow - timeAcc)>1000/ACC_SMPL_RATE){    // Accelerometer
    updateAcc();
    timeAcc = loopTimeNow;
    newSensorFlag |= NEW_ACC_FLAG;
  }
  
  if((loopTimeNow - timeGyro)>1000/GYRO_SMPL_RATE){  // Gyroscope
    updateGyro();
    timeGyro = loopTimeNow;
    newSensorFlag |= NEW_GYRO_FLAG;
  }
  
  #ifdef MAG
  if((loopTimeNow - timeMag)>1000/MAG_SMPL_RATE){    // Magnetometer
    updateMag();
    timeMag = loopTimeNow;
    newSensorFlag |= NEW_GYRO_FLAG;
  }
  #endif
}

void updateAcc(){
  // ----------------------Update Accelerometer-------------------------------
  
  int accBuffer[3];  // buffer array
  
  getNewAcc(accBuffer);
  
  // extract values
  accX = accBuffer[0];
  accY = accBuffer[1];
  accZ = accBuffer[2];
}

void updateGyro(){
  // ----------------------Update Gyroscope-----------------------------------
  
  float gyroBuffer[3];    // buffer array
  
  getNewGyro(gyroBuffer);
  
  gyroX = gyroBuffer[0];
  gyroY = gyroBuffer[1];
  gyroZ = gyroBuffer[2];
}

#ifdef MAG
void updateMag(){
  // ----------------------Update Magnetometer--------------------------------
  
  int magBuffer[3];      // buffer array
  
  getNewMag(magBuffer);
  
  magX = magBuffer[0];
  magY = magBuffer[1];
  magZ = magBuffer[2];
}
#endif


void getNewAcc(int acc_3D[]) {
  // ----------------------Accel. Read Function-------------------------------
  
  byte buff[ADXL345_TO_READ];  // buffer array
  
  // write data to buffer
  readFromSensor(ADXL345, ADXL345_DATAX0, ADXL345_TO_READ, buff);
  
  // extract from buffer
  acc_3D[0] = -(((int)(buff[1]) << 8) | buff[0]);  // invert X axis
  acc_3D[1] = (((int)(buff[3]) << 8) | buff[2]);
  acc_3D[2] = (((int)(buff[5]) << 8) | buff[4]);  
  
  // Correct readings with offsets
  acc_3D[0] = acc_3D[0] - accXOffset;
  acc_3D[1] = acc_3D[1] - accYOffset;
  acc_3D[2] = acc_3D[2] - accZOffset;
}

void getNewGyro(float gyro_3D[]){
  // ----------------------Gyro Read Function---------------------------------
  
  byte buffer[ITG3200_TO_READ];  // buffer array
  
  // write data to buffer
  readFromSensor(ITG3200, GYRO_XOUT_H, ITG3200_TO_READ, buffer);
  
  // extract from buffer and scale to deg/s
  gyro_3D[0] = (float)((int)(buffer[0]<<8) | buffer[1])/14.375;  //divide by 14 to scale to deg/s
  gyro_3D[1] = (float)((int)(buffer[2]<<8) | buffer[3])/14.375;
  gyro_3D[2] = (float)((int)(buffer[4]<<8) | buffer[5])/14.375;
  
  // correct readings with offsets
  gyro_3D[0] = (gyro_3D[0] - gyroXOffset);
  gyro_3D[1] = (gyro_3D[1] - gyroYOffset);
  gyro_3D[2] = (gyro_3D[2] - gyroZOffset);
}

#ifdef MAG
void getNewMag(int mag_3D[]){
  // ----------------------Mag Read Function----------------------------------
  
  byte buff[MAG_TO_READ];
  
  readFromSensor(HMC5883L, MAG_OUT_X_H, MAG_TO_READ, buff);
  
  mag_3D[1] = (((int)(buff[0]) << 8) | buff[1]);  // X data in buff[0], buff[1]
  mag_3D[0] = (((int)(buff[4]) << 8) | buff[5]);  // Y data in buff[4], buff[5]
  mag_3D[2] = (((int)(buff[2]) << 8) | buff[3]);  // Z data in buff[2], buff[3]
}
#endif

// ============================================================================
// ===                        STATE ESTIMATION                              ===
// ============================================================================

#ifndef COMP_FILTER
void updateStateEst(){
  /*
  -----------------------Attitude Estimation Function--------------------------
  
  Combines accelerometer and gyro data to estimate the roll and pitch.
  Accelerometer gain adjusted with ACCEL_GAIN. 
 
  Roll Directions:
      Positive roll: craft rolled to the right
      Negative roll: craft rolled to the left
      
  Pitch Directions:
      Positive pitch: craft pitched with nose up
      Negative pitch: craft pitched with nose down
  */
  
  static unsigned long lastStateEstTime; 
  static float pitch_acc, roll_acc;
  float errorPitch, errorRoll;
  
  if(newSensorFlag){
    // only run if new sensor reading has been taken
    
    unsigned long t = millis();
    float dt = (float)(t-lastStateEstTime)/1000.0;
    lastStateEstTime = t;
    
    if(newSensorFlag & NEW_ACC_FLAG){
      // calculate attitude from accelerometer
      pitch_acc = -atan2(accY, accZ) * 180.0 / PI;
      roll_acc = -atan2(accX, accZ) * 180.0 / PI;
    }
    
    // compare accel attitude to last estimate to get error
    errorPitch = pitch - pitch_acc;
    errorRoll = roll - roll_acc;
    
    if(newSensorFlag & NEW_GYRO_FLAG){
      // combine with gyro reading and integrate
      pitch = pitch + (-gyroX - errorPitch*ACCEL_GAIN)*dt;
      roll = roll + (-gyroY - errorRoll*ACCEL_GAIN)*dt;
    }
    
    #ifdef MAG
    if(newSensorFlag & NEW_MAG_FLAG){
      heading = (int)((atan2(magX, magY) * 180 / PI)+0.5);
      if(heading<0){
        heading += 360;
      }
    }
    #endif
    
    newSensorFlag = 0;

  }
}
#endif

#ifdef COMP_FILTER
void updateStateEst(){
  // EXPERIMENTAL. Uses simple complementary to estimate state.
  float accRoll, accPitch;
  float errorRoll, errorPitch;
  static unsigned long tm;
  
  unsigned long t = micros();
  float dt = (float)(t-tm)/1000000.0;
  tm = t;
  
  accRoll  = -atan2(accX, accZ) * 180 / PI;
  accPitch = -atan2(accY, accZ) * 180 / PI;
  
  roll  = COMP_SPLIT * (-gyroY*dt +  roll) + (1-COMP_SPLIT) * accRoll;
  pitch = COMP_SPLIT * (-gyroX*dt + pitch) + (1-COMP_SPLIT) * accPitch; 
}
#endif
  
  
// ============================================================================
// ===                General Purpose Sensor Functions                      ===
// ============================================================================

void writeToSensor(int device, byte address, byte val) {
  // -------------------Write to Sensor Funcion--------------------------------
  
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(val);
  Wire.endTransmission();
}


void readFromSensor(int device, byte address, int num, byte buff[]) {
  // -------------------Write to Sensor Funcion--------------------------------
  
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.beginTransmission(device);
  Wire.requestFrom(device, num);
  
  int i=0;
  while(Wire.available()){
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();
}
