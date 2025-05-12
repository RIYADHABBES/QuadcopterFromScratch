#include "MPU6050ManagerV2.h"

#include <Arduino.h>
#include <Wire.h> // library for I2C communication with the GY-521/MPU-6050
//#include <main.cpp>
#define MPU_ADDRESS 0x68  // MPU-6050 address = 0x68

#define X      0     // index 0 : X axis
#define Y      1     // index 1 : Y axis
#define Z      2     // index 2 : Z axis

#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3

#define LED 13    // LED equals 13 Built-in Nano board LED

// Wet Floor
#define ESC1_pin 5
#define ESC2_pin 6
#define ESC3_pin 9
#define ESC4_pin 3

// End Wet Floor
//--------------------------
// definition of constants
//--------------------------

static constexpr float SF_Gyro = 65.5;  // gyro scale factor
static constexpr float SF_Accel = 4096; // accelerometer scale factor


static constexpr int stabilization_coeff = 3.0;  // attitude correction coefficient, 3.00 is a good value

MPU6050Manager::MPU6050Manager(){
  
  initializeMPU6050();

  calibrateMPU6050();

  m_time_storage = millis();
  readMPU6050();

}
void MPU6050Manager::initializeMPU6050(){

    Serial.println("MPU6050 initialization began ...");
    // opening the I2C line as master
    
    Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU_ADDRESS);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Configure Accelerometer
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);

    delay(250);               // allow time for the MPU-6050 to start up

Serial.println("MPU6050 had been initialized !");
}

void MPU6050Manager::calibrateMPU6050(){
 Serial.println("MPU6050 Calibration began ...");

 // We can call this funtion in the setup section to calculate the accelerometer and gury data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  int nb_values = 200;
  for(int i = 0 ; i < nb_values ; ++i){
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);
    m_accel_raw[X] = (Wire.read() << 8 | Wire.read()) / 4096.0 ; // AccX
    m_accel_raw[Y] = (Wire.read() << 8 | Wire.read()) / 4096.0 ; // AccY
    m_accel_raw[Z] = (Wire.read() << 8 | Wire.read()) / 4096.0 ; // AccZ

    // m_accel_raw[X]=-m_accel_raw[X];

    // Sum all readings
    m_accel_offset[X] += ((atan((m_accel_raw[Y]) / sqrt(pow((m_accel_raw[X]), 2) + pow((m_accel_raw[Z]), 2))) * 180 / PI));
    m_accel_offset[Y] += ((atan(-1 * (m_accel_raw[X]) / sqrt(pow((m_accel_raw[Y]), 2) + pow((m_accel_raw[Z]), 2))) * 180 / PI));
    m_accel_offset[Z] += m_accel_raw[Z];
/*
    Serial.print("m_accel_offset : ");
    Serial.print(m_accel_offset[X]);
    Serial.print(" ");
    Serial.print(m_accel_offset[Y]);
    Serial.print(" ");
    Serial.print(m_accel_offset[Z]);
    Serial.println(" ");
*/
  }
  //Divide the sum by 200 to get the error value
  m_accel_offset[X] /= nb_values;
  m_accel_offset[X] /= nb_values;
  // calculation of the average offset
  m_accel_offset[Z] /= nb_values;
  // centering on 4096
  m_accel_offset[Z] -= 4096;

  // Read gyro values 200 nb_values times
  for(int i = 0 ; i < nb_values ; ++i) {
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDRESS, 6, true);
    m_gyro_raw[ROLL] = Wire.read() << 8 | Wire.read(); // GyroX
    m_gyro_raw[PITCH] = Wire.read() << 8 | Wire.read(); // GyroY
    m_gyro_raw[YAW] = Wire.read() << 8 | Wire.read();   // gyro yaw
    
    // m_gyro_raw[PITCH]=-m_gyro_raw[PITCH];
    // m_gyro_raw[YAW]=-m_gyro_raw[YAW];
    // Sum all readings
    m_gyro_offset[ROLL] += (m_gyro_raw[ROLL] / 32.8); // GyroErrorX
    m_gyro_offset[PITCH] += (m_gyro_raw[PITCH] / 32.8); // GyroErrorY
    m_gyro_offset[YAW] += (m_gyro_raw[YAW] / 32.8);
  }
  //Divide the sum by nb_values to get the error value
  m_gyro_offset[ROLL] /= nb_values;
  m_gyro_offset[PITCH] /= nb_values;
  m_gyro_offset[YAW] /= nb_values;
 
 
 
 /////////////////////////////////////////////////////////
 /*int nb_values = 2000;

    for (int i = 0; i < nb_values; i++) 
    {
     // Serial.println(i);
        // the LED is flashing rapidly during calibration
        // to do this, we make it change state every 20 iterations of the "for" loop.
        if(i % 20 == 0) digitalWrite(LED, !digitalRead(LED));   
      
        readMPU6050(); // MPU6050 data read function

        // sum of the measurement samples

        m_gyro_offset[ROLL] += m_gyro_raw[ROLL];
        m_gyro_offset[PITCH] += m_gyro_raw[PITCH];
        m_gyro_offset[YAW] += m_gyro_raw[YAW];

        m_accel_offset[X] += m_accel_raw[X];
        m_accel_offset[Y] += m_accel_raw[Y];
        m_accel_offset[Z] += m_accel_raw[Z];

    // to prevent the ESCs from beeping during calibration
    // they need to receive a PWM signal, so we send them the value
    // minimum : 1000us pulses
    
    // Original PORTD |= B11110000;       // setting to "1" of the outputs connected to the ESCs

    digitalWrite(ESC1_pin, HIGH);
    digitalWrite(ESC2_pin, HIGH);
    digitalWrite(ESC3_pin, HIGH);
    digitalWrite(ESC4_pin, HIGH);
    
    delayMicroseconds(1000);  // waiting time of 1000 microseconds (1ms)

    // Original PORTD &= B00001111;       // setting to "0" of the outputs connected to the ESCs
    
    digitalWrite(ESC1_pin, LOW);
    digitalWrite(ESC2_pin, LOW);
    digitalWrite(ESC3_pin, LOW);
    digitalWrite(ESC4_pin, LOW);
    
    delay(3);         // wait 3ms to complete the PWM signal period to about 4ms 
                      // (about because there are other instructions in the loop 
                      // that take time to execute) or a frequency close to 250 Hz,
                      // which is the upper limit acceptable for a standard ESC     
   }

    // calculation of the average offsets for angular velocities
    m_gyro_offset[ROLL] /= nb_values;
    m_gyro_offset[PITCH] /= nb_values;
    m_gyro_offset[YAW] /= nb_values;

    // calculation of average offsets for accelerations
    m_accel_offset[X] /= nb_values;
    m_accel_offset[Y] /= nb_values;

    // WARNING :
    // for the X and Y axes, we subtract the offset value from the raw value to re-center
    // it to zero because at rest and horizontally the MPU has zero acceleration in X and Y.
    // for the Z axis, the centering of the raw measurements is different from that of the X and Y axes
    // at rest and horizontally the MPU6050 is accelerated by 1g 
    // for an acceleration of 1g, the returned value must be 4096 so the offset to be subtracted
    // from the raw value to center it on 4096 is : average - 4096

        
    // calculation of the average offset
    m_accel_offset[Z] /= nb_values;
    // centering on 4096
    m_accel_offset[Z] -= 4096;
*/
Serial.println("MPU6050 had been calibrated !");    
}



    //#################################################
//  MPU6050 raw measurement reading function
//
//  "readMPU6050()"
//
//  - no input parameter
//
//  - no value returned
//
//#################################################
void MPU6050Manager::readMPU6050()
{
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
 
 // waiting until all 14 bytes are received
    while(Wire.available() < 6);
  //For a range of +-8g, we need to divide the raw values by 4096, according to the datasheet
  m_accel_raw[X] = (Wire.read() << 8 | Wire.read()) / 4096.0; // X-axis value  // AccX
  m_accel_raw[Y] = (Wire.read() << 8 | Wire.read()) / 4096.0; // Y-axis value  // AccY
  m_accel_raw[Z] = (Wire.read() << 8 | Wire.read()) / 4096.0; // Z-axis value  // AccZ
  // Calculating angle values using
  m_angle_accel[ROLL] = (atan(m_accel_raw[Y] / sqrt(pow(m_accel_raw[X], 2) + pow(m_accel_raw[Z], 2))) * 180 / PI) + 1.15; // AccErrorX ~(-1.15) See the calculate_IMU_error()custom function for more details
  m_angle_accel[PITCH] = (atan(-1 * m_accel_raw[X] / sqrt(pow(m_accel_raw[Y], 2) + pow(m_accel_raw[Z], 2))) * 180 / PI) - 0.52; // AccErrorX ~(0.5)
  //m_angle_accel[YAW] = ;
  
  Serial.print(" m_accel_raw : ");
    Serial.print( m_accel_raw[X]);
    Serial.print(" ");
    Serial.print( m_accel_raw[Y]);
       Serial.print(" ");
    Serial.print( m_accel_raw[Z]);
    Serial.println(" ");

  // === Read gyro data === //
  m_dT = (millis() - m_time_storage) / 1000;   // Divide by 1000 to get seconds
  m_time_storage = millis();            // Current time actual time read
Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  // waiting until all 14 bytes are received
    while(Wire.available() < 6);
 
  m_gyro_raw[ROLL] = (Wire.read() << 8 | Wire.read()) / 32.8; // For a 1000dps range we have to divide first the raw value by 32.8, according to the datasheet
  m_gyro_raw[PITCH] = (Wire.read() << 8 | Wire.read()) / 32.8;
  m_gyro_raw[YAW] = (Wire.read() << 8 | Wire.read()) / 32.8;

  m_gyro_raw[ROLL] = m_gyro_raw[ROLL] + 1.85; //// GyroErrorX ~(-1.85)
  m_gyro_raw[PITCH] = m_gyro_raw[PITCH] - 0.15; // GyroErrorY ~(0.15)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  m_angle_gyro[ROLL] = m_gyro_raw[ROLL] * m_dT;
  m_angle_gyro[PITCH] = m_gyro_raw[PITCH] * m_dT;
  // Complementary filter - combine acceleromter and gyro angle values
  m_angle[ROLL] = 0.98 * (m_angle[ROLL] + m_angle_gyro[ROLL]) + 0.02 * m_angle_accel[ROLL];
  m_angle[PITCH] = 0.98 * (m_angle[PITCH] + m_angle_gyro[PITCH]) + 0.02 * m_angle_accel[PITCH];
/*
    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B); // start address for reading
    Wire.endTransmission();             
    Wire.requestFrom(MPU_ADDRESS,14);// 14 byte reading request
  
    // waiting until all 14 bytes are received
    while(Wire.available() < 14);
  
    m_accel_raw[X]  = Wire.read() << 8 | Wire.read();   // X accel.
    m_accel_raw[Y]  = Wire.read() << 8 | Wire.read();   // Y accel. 
    m_accel_raw[Z]  = Wire.read() << 8 | Wire.read();   // Z accel.
    m_temperature = Wire.read() << 8 | Wire.read();     // temperature
    m_gyro_raw[ROLL] = Wire.read() << 8 | Wire.read();  // gyro roll 
    m_gyro_raw[PITCH] = Wire.read() << 8 | Wire.read(); // gyro pitch 
    m_gyro_raw[YAW] = Wire.read() << 8 | Wire.read();   // gyro yaw
*/
   
   /* // changes of signs for our rotation direction convention and
    // our senses of accelerations
    
    // for angular velocities
    m_gyro_raw[PITCH]=-m_gyro_raw[PITCH];
    m_gyro_raw[YAW]=-m_gyro_raw[YAW];
    
    // for accelerations
    m_accel_raw[X]=-m_accel_raw[X];
*/
    
}


//################################################################
//  Function to calculate Roll and Pitch angles by sensor fusion
//        
//  "calculateAnglesFusion()"
//
//  - no input parameter
//
//  - no value returned
//
//################################################################
void MPU6050Manager::calculateAnglesFusion()    
{
/*
    // calculation of raw angular velocities without offsets
    m_gyro_raw[ROLL] = m_gyro_raw[ROLL] - m_gyro_offset[ROLL];  
    m_gyro_raw[PITCH] = m_gyro_raw[PITCH] - m_gyro_offset[PITCH];  
    m_gyro_raw[YAW] = m_gyro_raw[YAW] - m_gyro_offset[YAW];
    
    // calculation of raw accelerations without offsets
    m_accel_raw[X] = m_accel_raw[X] - m_accel_offset[X];  
    m_accel_raw[Y] = m_accel_raw[Y] - m_accel_offset[Y];    
    m_accel_raw[Z] = m_accel_raw[Z] - m_accel_offset[Z];  

    // filtering of raw angular velocities without offsets
    m_gyro_filtered[ROLL] = 0.8*m_gyro_filtered[ROLL] + 0.2*m_gyro_raw[ROLL];    
    m_gyro_filtered[PITCH] = 0.8*m_gyro_filtered[PITCH] + 0.2*m_gyro_raw[PITCH];    
    m_gyro_filtered[YAW] = 0.8*m_gyro_filtered[YAW] + 0.2*m_gyro_raw[YAW];

    // filtering of raw accelerations without offsets
    m_accel_filtered[X] = 0.8*m_accel_filtered[X] + 0.2*m_accel_raw[X];
    m_accel_filtered[Y] = 0.8*m_accel_filtered[Y] + 0.2*m_accel_raw[Y];
    m_accel_filtered[Z] = 0.8*m_accel_filtered[Z] + 0.2*m_accel_raw[Z];   
    
    // scaling of raw angular velocities without offsets and filtered    
    m_gyro[ROLL] = m_gyro_filtered[ROLL] / SF_Gyro;
    m_gyro[PITCH] = m_gyro_filtered[PITCH] / SF_Gyro;
    m_gyro[YAW] = m_gyro_filtered[YAW] / SF_Gyro;    
    
    // scaling of raw acceleration without offsets and filtered  
    m_accel[X] = m_accel_filtered[X] / SF_Accel;
    m_accel[Y] = m_accel_filtered[Y] / SF_Accel;  
    m_accel[Z] = m_accel_filtered[Z] / SF_Accel;
    
    //---------------------------------------
    // calculation of angles from gyro data
    //---------------------------------------  
    m_angle_gyro[ROLL] += m_gyro[ROLL]*(m_dT/(float)1000000); // mandatory "cast" otherwise the results are constant
    
    m_angle_gyro[PITCH] += m_gyro[PITCH]*(m_dT/(float)1000000);                   
  
    m_angle_gyro[YAW] += m_gyro[YAW]*(m_dT/(float)1000000);
  
    // angle transfer ROLL <--> PITCH in case of YAW rotation
  
    m_angle_gyro[ROLL] += m_angle_gyro[PITCH] * sin(m_gyro[YAW]*(m_dT/(float)1000000)*0.0174533); // (PI/180=0,0174533)
  
    m_angle_gyro[PITCH] -= m_angle_gyro[ROLL] * sin(m_gyro[YAW]*(m_dT/(float)1000000)*0.0174533);

    m_dT = (micros() - m_time_storage);
    m_time_storage = micros();
    
    //------------------------------------------------
    // calculation of angles from accelerometer data
    //------------------------------------------------
    
    m_angle_accel[ROLL] = atan(m_accel[Y]/(sqrt(m_accel[X]*m_accel[X]+m_accel[Z]*m_accel[Z])))*(float)(180/PI);
    m_angle_accel[PITCH] = -atan(m_accel[X]/(sqrt(m_accel[Y]*m_accel[Y]+m_accel[Z]*m_accel[Z])))*(float)(180/PI);
 
    //------------------------------------------       
    // calculation of angles by "sensor fusion"
    //------------------------------------------
    
    if(m_init_gyro_angles)
    { 
        m_angle[ROLL] = 0.96*m_angle_gyro[ROLL]+0.04*m_angle_accel[ROLL];//0.9996*m_angle_gyro[ROLL]+0.0004*m_angle_accel[ROLL];
        m_angle[PITCH] = 0.96*m_angle_gyro[PITCH]+0.04*m_angle_accel[PITCH];//0.9996*m_angle_gyro[PITCH]+0.0004*m_angle_accel[PITCH];     
    }
    else
    {
      // alignment of the gyro angles with those of the accelerometer 
      // only once at startup
      m_angle_gyro[ROLL]=m_angle_accel[ROLL];
      m_angle_gyro[PITCH]=m_angle_accel[PITCH];
      m_init_gyro_angles = true; 
    }
*/
    // calculation of the correction for horizontal stabilization
    m_roll_adjustment = m_angle[ROLL] * stabilization_coeff;
    m_pitch_adjustment = m_angle[PITCH] * stabilization_coeff;    
    
    if(!m_selflevel_mode)
    {   // if the quadricopter is not in stabilized mode
        m_roll_adjustment = 0;   // sets the roll angle correction to zero 
        m_pitch_adjustment = 0;  // sets the pitch angle correction to zero        
    }
}

float const * const MPU6050Manager::getGyro() const 
{
  return m_gyro;
}


float MPU6050Manager::getRollAdjustment() const
{
  return m_roll_adjustment;
}
float MPU6050Manager::getPitchAdjustment() const
{
  return m_pitch_adjustment;
}

void MPU6050Manager::printAngles(){

  Serial.print("ROLL:");
  Serial.print(m_angle[ROLL]);
  Serial.print(",");
  Serial.print("PITCH:");
  Serial.print(m_angle[PITCH]);
  Serial.print(",");
  Serial.print("YAW:");
  Serial.println(m_angle[YAW]);
}