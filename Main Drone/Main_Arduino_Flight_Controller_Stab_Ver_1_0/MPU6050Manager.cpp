#include "MPU6050Manager.h"

#include <Arduino.h>

#define MPU_ADDRESS 0x68  // MPU-6050 address = 0x68

#define X      0     // index 0 : X axis
#define Y      1     // index 1 : Y axis
#define Z      2     // index 2 : Z axis

#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3

#define pi 3.14159265359 // Pi constant

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

const float SF_Gyro = 65.5;  // gyro scale factor
const float SF_Accel = 4096; // accelerometer scale factor


const int stabilization_coeff = 3.0;  // attitude correction coefficient, 3.00 is a good value


unsigned long dT; // variable for calculating the elapsed time between 2 MPU6050 readings


//-----------------------------------------------
// variables de lecture des données du MPU6050 
//-----------------------------------------------
int accel_raw[3] = {0,0,0};  // // table of raw accelerations

int gyro_raw[3] = {0,0,0};  // // table of raw angular velocities

float gyro_filtered[3] = {0,0,0}; // table of filtered raw angular velocities 80%/20%

float accel_filtered[3] = {0,0,0};// table of filtered accelerations
                             
int temperature = 0;

float gyro[3] = {0,0,0}; // table of angular velocities scaled and without offsets

float accel[3] = {0,0,0};// table of accelerations scaled and without offsets

float angle_gyro[3] = {0,0,0}; // table of angles calculated from angular velocities

float angle_accel[3] = {0,0,0}; // table of angles calculated from accelerations

float angle[3] = {0,0,0}; // table of angles calculated by "sensor fusion"

long gyro_offset[3] = {0,0,0}; // table of gyroscope offsets

long accel_offset[3] = {0,0,0}; // table of accelerometer offsets

bool init_gyro_angles = false; // flag allowing to know if the alignment of
                                // the angles calculated from the gyro values
                                // on the angles calculated from the accelerometer
                                // values has been done
                         
bool selflevel_mode = true; // stabilized mode (true) or acrobatic mode (false)

// adjustment variables for the correction of Pitch and Roll angular velocity setpoints
float pitch_adjustment = 0; 
float roll_adjustment = 0; 

void MPU6050Manager::initializeMPU6050(){

    // opening the I2C line as master
    Wire.begin(); 
    
    // setting the I2C clock to 400kHz instead of the default 100kHz
    TWBR = 12; 

    // internal clock configuration
    Wire.beginTransmission(MPU_ADDRESS);  // start communication 
    Wire.write(0x6B);                     // register PWR_MGMT_1
    Wire.write(0x00);                     // 8MHz internal clock                 
    Wire.endTransmission();               // end of transmission
  
    // gyroscope scale configuration
    Wire.beginTransmission(MPU_ADDRESS); 
    Wire.write(0x1B);                     // register GYRO_CONFIG
    Wire.write(0x08);                     // range ±500°/s
    Wire.endTransmission();  
  
  
    // accelerometer scale configuration
    Wire.beginTransmission(MPU_ADDRESS); 
    Wire.write(0x1C);                    // register ACCEL_CONFIG
    Wire.write(0x10);                    // range ±8g
    Wire.endTransmission(); 
  
    // low-pass filter configuration
    Wire.beginTransmission(MPU_ADDRESS); 
    Wire.write(0x1A);                     // register CONFIG 
    Wire.write(0x03);                     // cut-off at ~43Hz
    Wire.endTransmission(); 

    delay(250);               // allow time for the MPU-6050 to start up

}

void MPU6050Manager::calibrateMPU6050(){
 int nb_values = 2000;

    for (int i = 0; i < nb_values; i++) 
    {
        // the LED is flashing rapidly during calibration
        // to do this, we make it change state every 20 iterations of the "for" loop.
        if(i % 20 == 0) digitalWrite(LED, !digitalRead(LED));   
      
        readMPU6050(); // MPU6050 data read function

        // sum of the measurement samples

        gyro_offset[ROLL] += gyro_raw[ROLL];
        gyro_offset[PITCH] += gyro_raw[PITCH];
        gyro_offset[YAW] += gyro_raw[YAW];

        accel_offset[X] += accel_raw[X];
        accel_offset[Y] += accel_raw[Y];
        accel_offset[Z] += accel_raw[Z];

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
    gyro_offset[ROLL] /= nb_values;
    gyro_offset[PITCH] /= nb_values;
    gyro_offset[YAW] /= nb_values;

    // calculation of average offsets for accelerations
    accel_offset[X] /= nb_values;
    accel_offset[Y] /= nb_values;

    // WARNING :
    // for the X and Y axes, we subtract the offset value from the raw value to re-center
    // it to zero because at rest and horizontally the MPU has zero acceleration in X and Y.
    // for the Z axis, the centering of the raw measurements is different from that of the X and Y axes
    // at rest and horizontally the MPU6050 is accelerated by 1g 
    // for an acceleration of 1g, the returned value must be 4096 so the offset to be subtracted
    // from the raw value to center it on 4096 is : average - 4096

        
    // calculation of the average offset
    accel_offset[Z] /= nb_values;
    // centering on 4096
    accel_offset[Z] -= 4096;
    
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

    Wire.beginTransmission(MPU_ADDRESS);
    Wire.write(0x3B); // start address for reading
    Wire.endTransmission();             
    Wire.requestFrom(MPU_ADDRESS,14);// 14 byte reading request
  
    // waiting until all 14 bytes are received
    while(Wire.available() < 14);
  
    accel_raw[X]  = Wire.read() << 8 | Wire.read();   // X accel.
    accel_raw[Y]  = Wire.read() << 8 | Wire.read();   // Y accel. 
    accel_raw[Z]  = Wire.read() << 8 | Wire.read();   // Z accel.
    temperature = Wire.read() << 8 | Wire.read();     // temperature
    gyro_raw[ROLL] = Wire.read() << 8 | Wire.read();  // gyro roll 
    gyro_raw[PITCH] = Wire.read() << 8 | Wire.read(); // gyro pitch 
    gyro_raw[YAW] = Wire.read() << 8 | Wire.read();   // gyro yaw

    // changes of signs for our rotation direction convention and
    // our senses of accelerations
    
    // for angular velocities
    gyro_raw[PITCH]=-gyro_raw[PITCH];
    gyro_raw[YAW]=-gyro_raw[YAW];
    
    // for accelerations
    accel_raw[X]=-accel_raw[X];
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

    // calculation of raw angular velocities without offsets
    gyro_raw[ROLL] = gyro_raw[ROLL] - gyro_offset[ROLL];  
    gyro_raw[PITCH] = gyro_raw[PITCH] - gyro_offset[PITCH];  
    gyro_raw[YAW] = gyro_raw[YAW] - gyro_offset[YAW];
    
    // calculation of raw accelerations without offsets
    accel_raw[X] = accel_raw[X] - accel_offset[X];  
    accel_raw[Y] = accel_raw[Y] - accel_offset[Y];    
    accel_raw[Z] = accel_raw[Z] - accel_offset[Z];  

    // filtering of raw angular velocities without offsets
    gyro_filtered[ROLL] = 0.8*gyro_filtered[ROLL] + 0.2*gyro_raw[ROLL];    
    gyro_filtered[PITCH] = 0.8*gyro_filtered[PITCH] + 0.2*gyro_raw[PITCH];    
    gyro_filtered[YAW] = 0.8*gyro_filtered[YAW] + 0.2*gyro_raw[YAW];

    // filtering of raw accelerations without offsets
    accel_filtered[X] = 0.8*accel_filtered[X] + 0.2*accel_raw[X];
    accel_filtered[Y] = 0.8*accel_filtered[Y] + 0.2*accel_raw[Y];
    accel_filtered[Z] = 0.8*accel_filtered[Z] + 0.2*accel_raw[Z];   
    
    // scaling of raw angular velocities without offsets and filtered    
    gyro[ROLL] = gyro_filtered[ROLL] / SF_Gyro;
    gyro[PITCH] = gyro_filtered[PITCH] / SF_Gyro;
    gyro[YAW] = gyro_filtered[YAW] / SF_Gyro;    
    
    // scaling of raw acceleration without offsets and filtered  
    accel[X] = accel_filtered[X] / SF_Accel;
    accel[Y] = accel_filtered[Y] / SF_Accel;  
    accel[Z] = accel_filtered[Z] / SF_Accel;
    
    //---------------------------------------
    // calculation of angles from gyro data
    //---------------------------------------  
    angle_gyro[ROLL] += gyro[ROLL]*(dT/(float)1000000); // mandatory "cast" otherwise the results are constant
    
    angle_gyro[PITCH] += gyro[PITCH]*(dT/(float)1000000);                   
  
    angle_gyro[YAW] += gyro[YAW]*(dT/(float)1000000);
  
    // angle transfer ROLL <--> PITCH in case of YAW rotation
  
    angle_gyro[ROLL] += angle_gyro[PITCH] * sin(gyro[YAW]*(dT/(float)1000000)*0.0174533); // (pi/180=0,0174533)
  
    angle_gyro[PITCH] -= angle_gyro[ROLL] * sin(gyro[YAW]*(dT/(float)1000000)*0.0174533);

    //------------------------------------------------
    // calculation of angles from accelerometer data
    //------------------------------------------------
    
    angle_accel[ROLL] = atan(accel[Y]/(sqrt(accel[X]*accel[X]+accel[Z]*accel[Z])))*(float)(180/pi);
    angle_accel[PITCH] = -atan(accel[X]/(sqrt(accel[Y]*accel[Y]+accel[Z]*accel[Z])))*(float)(180/pi);
 
    //------------------------------------------       
    // calculation of angles by "sensor fusion"
    //------------------------------------------
    
    if(init_gyro_angles)
    { 
        angle[ROLL] = 0.9996*angle_gyro[ROLL]+0.0004*angle_accel[ROLL];
        angle[PITCH] = 0.9996*angle_gyro[PITCH]+0.0004*angle_accel[PITCH];     
    }
    else
    {
      // alignment of the gyro angles with those of the accelerometer 
      // only once at startup
      angle_gyro[ROLL]=angle_accel[ROLL];
      angle_gyro[PITCH]=angle_accel[PITCH];
      init_gyro_angles = true; 
    }

    // calculation of the correction for horizontal stabilization
    roll_adjustment = angle[ROLL] * stabilization_coeff;
    pitch_adjustment = angle[PITCH] * stabilization_coeff;    
    
    if(!selflevel_mode)
    {   // if the quadricopter is not in stabilized mode
        roll_adjustment = 0;   // sets the roll angle correction to zero 
        pitch_adjustment = 0;  // sets the pitch angle correction to zero        
    }
    
    Serial.print("Velocity Gyro Roll : ");
    Serial.print(gyro[ROLL]);
    Serial.print("    Velocity Gyro Pitch ");
    Serial.print(gyro[PITCH]);
    Serial.print(" Angle Roll : ");
    Serial.print(angle[ROLL]);
    Serial.print("    Angle Pitch ");
    Serial.println(angle[PITCH]);
}

