// ##################################################################
// ##################################################################
// ####                                                          ####
// ####       Program "Flight Controller" for Arduino V1.0       ####
// ####                                                          ####
// ####             PID control of angular velocities            ####
// ####                                                          ####
// ####           Horizontal stabilization by correction         ####
// ####                of angular velocity setpoints             ####
// ####                                                          ####
// ####                   Self-level mode control                ####
// ####                                                          ####
// ##################################################################
// ##################################################################

// Copyright (C) 2021 Olivier CHAU-HUU
// 
// This source program is the exclusive property of the author. It is distributed 
// only for personal use. It can in no case be copied or transformed in order to
// derive any commercial benefit.
//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!!       WARNING       !!!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// This program is delivered "as is" and without any warranty of any kind.
// It is given as an example to learn how to code a flight controller
// for Arduino UNO card.
// 
// A quadricopter UAV has dangerous parts such as the brushless motors
// equipped with their propellers.
//
// The author can in no way be held responsible for any injury or damage
// caused by the use of this program.
//
// It is up to the user to take all necessary safety precautions.
// 
//
// It is imperative to first test the operation of its UAV without
// propellers to validate its behavior. If and only if everything conforms,
// the propellers can be fixed. At this point the drone is potentially
// dangerous, it is the user's responsibility to take all necessary precautionary
// measures.
//
//
// MyDroneZone by Olivier CHAU-HUU :  website   --> https://sites.google.com/view/mydronezone
//                                    channel   --> https://www.youtube.com/channel/UCQMzPz1T8_Q2FvroI6nBwRg
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

////////////////////////////////////////////////////////////
////// Wiring for "Arduino UNO" microcontroller board //////
////////////////////////////////////////////////////////////

// Wiring of the Gyroscope :
//
// Remark :         the MPU6050 circuit used on the GY-521 board is powered by 3.3V 
//                  "MPU6050 VDD power supply voltage range = 2.375V to 3.46V" (Cf. Datasheet)
//
//                  but the GY-521 board integrates a 5V onboard LDO regulator, it can be powered
//                  between 3V and 5V.
//
//                              GY-521    -->  Arduino UNO
//                              --------------------------
//                              VCC       -->     5V 
//                              GND       -->     GND
//                              SDA       -->     PIN A4
//                              SCL       -->     PIN A5
//
// ESC wiring : D4, D5, D6, D7 and GND common
//
// Receiver wiring : 5V and GND
//
// Wiring of the RF receiver channels :   Channel 1 --> D8
//                                        Channel 2 --> D9
//                                        Channel 3 --> D10
//                                        Channel 4 --> D11
//
// Wiring of signal LED on D12
//
//
// Protective diode 1N4001 : between "+" battery and "VIN", passing direction
//
//

//////////////////////////////////////
////// Libraries to be included //////
//////////////////////////////////////
#include <Wire.h> // library for I2C communication with the GY-521/MPU-6050


////////////////////////////////////////
////// Declaration of the "define //////
////////////////////////////////////////

// the different "#define" to improve the readability of the code

#define MPU_ADDRESS 0x68  // MPU-6050 address = 0x68

#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3

#define X      0     // index 0 : X axis
#define Y      1     // index 1 : Y axis
#define Z      2     // index 2 : Z axis

#define STOP   0   // STOP equals 0
#define ARMED 1   // ARMED equals 1
#define RUN   2   // RUN equals 2

#define CHANNEL1 0    // CHANNEL1 equals 0 (ROLL control)
#define CHANNEL2 1    // CHANNEL2 equals 1 (PITCH control)
#define CHANNEL3 2    // CHANNEL3 equals 2 (GAS control)
#define CHANNEL4 3    // CHANNEL4 equals 3 (YAW control)

#define ESC1 0    // ESC1 equals 0
#define ESC2 1    // ESC2 equals 1
#define ESC3 2    // ESC3 equals 2
#define ESC4 3    // ESC4 equals 3

#define LED 12    // LED equals 12

#define pi 3.14159265359 // Pi constant

////////////////////////////////////////////////////
////// Declaration of constants and variables //////
////////////////////////////////////////////////////

//--------------------------
// definition of constants
//--------------------------

const float SF_Gyro = 65.5;  // gyro scale factor
const float SF_Accel = 4096; // accelerometer scale factor

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//\\\\                   PID SETTING PARAMETERS                    \\\\          
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


const float KpRoll = 1.80;   // value of proportional gain in roll
const float KiRoll = 0.03;   // value of integral gain in roll
const float KdRoll = 12.6;   // value of derivative gain in roll
const float PID_Roll_limit = 400; // limit in absolute value of the
                                  // PID correction in Roll

const float KpPitch = KpRoll;   // value of proportional gain in pitch
const float KiPitch = KiRoll;   // value of integral gain in pitch
const float KdPitch = KdRoll;   // value of derivative gain in pitch
const float PID_Pitch_limit = PID_Roll_limit; // limit in absolute value of the
                                              // PID correction in Pitch

const float KpYaw = 4.00;   // value of proportional gain in yaw
const float KiYaw = 0.02;   // value of integral gain in yaw
const float KdYaw = 0.00;   //  value of derivative gain in yaw
const float PID_Yaw_limit = 400; // limit in absolute value of the
                                 // PID correction in Yaw


const int stabilization_coeff = 3.0;  // attitude correction coefficient, 3.00 is a good value

boolean selflevel_mode = true; // stabilized mode (true) or acrobatic mode (false)

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

//----------------------------------
// declaration of global variables
//----------------------------------

//--------------------------------------------------------------------------
// variables used to calculate the elapsed time between 2 MPU6050 readings
//--------------------------------------------------------------------------

unsigned long time_storage; // memorization variable of the present moment
unsigned long dT; // variable for calculating the elapsed time between 2 MPU6050 readings

//---------------------------------------------------------------------------------------------------
// variables used to calculate the duration of the pulses delivered on the channels of the receiver
//---------------------------------------------------------------------------------------------------

volatile unsigned int pulse_duration[4]; // pulse durations on the receiver channels

volatile unsigned long current_time; // current time storage variable used in ISR
                                     // to calculate receiver pulse durations
                                        
volatile unsigned long pulse_start_t[4]; // table of pulse start times of the receiver channels

volatile byte ch_status_storage[4]; // storage of the states of the reception channels

//--------------------------------------------
// variables used to generate the ESC pulses
//--------------------------------------------

unsigned long present_time; // variable for storing the current time to know when 
                            // to generate the falling edges of the ESC control signals
                            
unsigned long ESC_pulse_start_time;   // start time of ESC pulses generation

unsigned int ESC_pulse_duration[4]= {1000,1000,1000,1000};  // duration of the control pulses 
                                                            // to be sent to the ESCs
                                                            
unsigned long ESC_pulse_end_time[4];   // end times of the ESC pulses

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

boolean init_gyro_angles=false; // flag allowing to know if the alignment of
                                // the angles calculated from the gyro values
                                // on the angles calculated from the accelerometer
                                // values has been done
                         
//-------------------------------------------
// variables for calculating PID corrections
//-------------------------------------------
                        
// variables containing flight setpoints : Roll, Pitch, Yaw, Gas/Throttle
float setpoint[4] = {0,0,0,0};

// calculated correction values for P, I and D

float P_correction[3] = {0,0,0}; // table of proportional corrections in roll, pitch and yaw  
float I_correction[3] = {0,0,0}; // table of integral corrections in roll, pitch and yaw
float D_correction[3] = {0,0,0}; // table of derivative corrections in roll, pitch and yaw  
float PID_correction[3] = {0,0,0}; // table of complete PID corrections in roll, pitch and yaw  
float PID_limit[3] = {PID_Roll_limit, PID_Pitch_limit, PID_Yaw_limit}; // limits in absolute values
                                                                       // of PID corrections in Roll, 
                                                                       // Pitch and Yaw

// variables needed for PID calculations
float error[3]= {0,0,0}; // error table for roll, pitch and yaw
float integral_error[3] = {0,0,0}; // table of error integrals (for PID calculations)
float derivative_error[3] = {0,0,0}; // error derivatives table (for PID calculations)
float error_storage[3] = {0,0,0}; // roll, pitch and yaw error memory table (for PID calculations)

// adjustment variables for the correction of Pitch and Roll angular velocity setpoints
float pitch_adjustment = 0; 
float roll_adjustment = 0; 

//-------------------------
// miscellaneous variables
//-------------------------

byte uav_status = 0; // variable containing the UAV's operating status

// variable allowing the measurement of the execution duration of the loop "loop()"
unsigned long loop_start;

////////////////////////////////////////////////////////////////////////////////
////// Declaration of the prototypes of the functions used in the program //////
////////////////////////////////////////////////////////////////////////////////


void configureOutputs();  // configuration function of the pins used as outputs

void initializeMPU6050(); // MPU6050 circuit initialization function by I2C

void calibrateMPU6050();  // MPU6050 calibration function : calculation of gyroscope and accelerometer offsets

void configureInterrupts(); // function to configure the pins that trigger an interrupt

void waitMinGas();       // waiting for the left joystick to be positioned at the bottom left : gas mini, yaw mini

void readMPU6050();      // reading MPU6050 data

void calculateAnglesFusion(); // calculation of Roll and Pitch angles by "Sensor Fusion"

void calculateSetpoints();    // calculation of the setpoint values from the PWM pulse durations of the receiver channels

void calculatePIDCommands();  // calculation of PID corrections

void calculateESCPulses();    // calculation of PWM pulse durations to be sent to the ESCs

void generateESCPulses();     // generation of ESC pulses and reading of MPU-6050 measurements

void resetPIDControllers();   // resetting the variables used to calculate PID corrections to zero

void stopMotors();            // sets the pulse duration of the ESCs to 1000 (minimum value)


float limit(float value, float min_value, float max_value); // function to limit a "float" type quantity between 2 values

//#######################################################
//####                     SETUP()                   ####
//#######################################################
void setup(int a) 
{  
    configureOutputs(); // configuration of the pins used as outputs 

    initializeMPU6050(); // MPU-6050 initialization function

    calibrateMPU6050(); // MPU6050 calibration function : calculation of gyroscope offsets
  
    // configuration of the pins used as interrupt triggers
    configureInterrupts();
     
    // waiting as long as the throttle joystick is not at the minimum and Yaw centered
    waitMinGas();
        
    // after executing "waitMinGas()", the quadcopter status corresponds to the joystick safety position
    // the variable "uav_status" is then set to "STOP".
  
    uav_status = STOP; // sets the status to "STOP".

    // taking the present moment when reading the MPU-6050 measurements to calculate the "dT" that will
    // be used to integrate the angular velocities
    time_storage = micros();

    readMPU6050();  // reading of angular velocities
    
}
//##### End of "SETUP()" #####


//####################################################
//####                    LOOP()                  ####
//####################################################

void loop(int a) 
{
        // take the start time of the "loop()" to manage the loop duration
        loop_start = micros();   
        
        // processing of gyro measurements to obtain reliable angular velocity values
        // and calculation of angles by merging gyro and accelerometer measurements
        calculateAnglesFusion();     

        ///////////////////////////////////////////////////////////////////////////////////////////
        //  Evolution of the states of the UAV according to the position of the left joystick.   //
        //                                                                                       //
        //  Reminder :  GAS   --> Channel 3                                                      //
        //              YAW   --> Channel 4                                                      //
        //              PITCH --> Channel 2                                                      //
        //              ROLL  --> Channel 1                                                      //
        //                                                                                       //
        ///////////////////////////////////////////////////////////////////////////////////////////
                        
        //-------------------------------
        // "STOP" STATE TO "ARMED" STATE 
        //-------------------------------
        //
        // after the "setup" the quadricopter is in the "STOP" state
        // if from this state the left joystick is set to low AND left, then the quadricopter goes to the "ARMED" state :
        // the motors are "armed" (ready to turn if the YAW is brought back to the middle position).
        //
        // pulse_duration[CHANNEL3] --> GAS
        // pulse_duration[CHANNEL4] --> YAW
        //
        
        if (uav_status == STOP && pulse_duration[CHANNEL3] < 1050 && pulse_duration[CHANNEL4] < 1050) 
        {
            uav_status = ARMED; // change to "ARMED" status
        }
             
        // Remark : after the "setup()", the state is necessarily "STOP", so one could be tempted to test only the position
        // of the joystick (what I had done at the beginning...). But if later on we are in "RUN" and we put the joystick down
        // on the left you can also switch to the "ARMED" state which cuts the motors : this is not the expected behavior.
        // Normally you can only get to "ARMED" state from the "STOP" state. It is therefore essential to set the condition
        // "uav_status == STOP" in the "if".

            
        //------------------------------
        // "ARMED" STATE TO "RUN" STATE 
        //------------------------------
        //
        // if the state is the "ARMED" state and the left joystick is brought back to the center (middle Yaw)
        // always with GAS at minimum then :   - "uav_status" is set to "RUN".
        //                                     - the PID calculation quantities are reset
        //
        // in the "RUN" state, the motors can be started as soon as the throttle is increased         
      
        
        if (uav_status == ARMED && pulse_duration[CHANNEL3] < 1050 && pulse_duration[CHANNEL4] > 1450) 
        {
            uav_status = RUN; // change to the "RUN" state
            
            resetPIDControllers(); // reset the PID calculation variables to prevent the motors
                                   // from racing at start up. 
                                   // at the first start there is no problem, but for example after
                                   // a first stop these variables are not null and at the next start
                                   // the program will take them into account and give wrong orders
                                   // to the motors.
        }
      
       
        //------------------------------
        // "RUN" STATE TO "STOP" STATE
        //------------------------------
        //  
        // if we are in the "RUN" state (i.e. motors running) and we put the left joystick on in
        // bottom right position i.e. :
        //
        //     - GAS at minimum
        //     - YAW at maximum (right)
        //
        // the status then returns to "STOP".
        //
      
            
        if (uav_status == RUN && pulse_duration[CHANNEL3] < 1050 && pulse_duration[CHANNEL4] > 1950) 
        {  
          uav_status = STOP;
        }

        // conversion of order orders received into data usable by PIDs
        calculateSetpoints();
      
        // setpoints and measurements are known, PID corrections can be calculated.
        calculatePIDCommands();
        
        ////////////////////////////////////////////////////////////////////
        //  Actions to be carried out according to the status of the UAV  //
        ////////////////////////////////////////////////////////////////////
            
        //--------------
        // "RUN" STATE
        //--------------
            
        if (uav_status == RUN) // ATTENTION : ONE FORGET of a "=" and the state was always at "RUN" !!!!!!!!!!!
        {
          
          calculateESCPulses(); // calculation of ESC pulse durations
           
        }
        else  
        {                     
          
          //-------------------------
          // "ARMED" or "STOP" STATE
          //-------------------------
        
          //  in all states other than RUN, the motors are switched off
      
          stopMotors(); // we set a 1000us pulse on the 4 ESCs 

          init_gyro_angles = false; // we authorize the assignment of the angles coming from the accelerometer in
                                    // the angle variables coming from the gyroscope so that the UAV does not take
                                    // off again with erroneous angle values coming from the gyroscope (the surface
                                    // where the stop took place is not horizontal).
        }   
      
        // ESCs pulse generation
        generateESCPulses();  // IMPORTANT: the function "generateESCPulses()" contains the function "readMPU6050()"
                              // in order to update the measurements.

        // we wait as long as the execution time of the "loop()" has not reached 4000us
        while(micros()-loop_start < 4000);

}
//##### End of "LOOP()" #####

/////////////////////////////////////
//// FUNCTIONS USED IN THE PROGRAM //
/////////////////////////////////////

//#####################################################
//  Arduino outputs initialization function
//
//  "configureOutputs()"
//
//  - no input parameter
//
//  - no value returned
//
//#####################################################
void configureOutputs()
{
  
    // declaration of pins used as outputs
    // Arduino pins are by default configured as inputs, only outputs have to be declared.
    DDRD |= B11110000;  // configuration of pins 4, 5, 6 and 7 of port D as outputs
    DDRB |= B00010000;  // configuring pin 12 of port B as output
    
}

//#####################################################
//  Interrupt pins initialization function
//
//  "configureInterrupts()"
//
//  - no input parameter
//
//  - no value returned
//
//#####################################################
void configureInterrupts()
{
  
    // interrupts configuration for receiver signals
    PCICR  |= (1 << PCIE0);     // condensed writing of PCICR = PCICR | (1 << PCIE0);
                                // PCIE0 is a variable defined in the "Arduino.h" library which is the rank of bit 0 (ordinal)
                                // in the PCICR registry
                                // so we shift the value 1 by 0 bit (PCIE0=0) to the left, i.e. "00000001"
                                // we make a logical "OR" of the PCICR register with the value "00000001"
                                // the result is placed in PCICR
                                // all of these operations have the effect of setting the PCICR zero-order bit (PCIE0) to "1"
                                // without touching the other bits of the PCICR register
  
    // the different interrupt PINS of port B are selected              
    PCMSK0 |= (1 << PCINT0);  // bit 0 (PCINT0) of register PCMSK0 is set to "1" in order to be able to receive an interrupt on
                              // a change of state of "digital input 8".
                
    PCMSK0 |= (1 << PCINT1);  // bit 1 (PCINT1) of register PCMSK0 is set to "1" in order to be able to receive an interrupt on
                              // a change of state of "digital input 9". 
                
    PCMSK0 |= (1 << PCINT2);  // bit 2 (PCINT2) of register PCMSK0 is set to "1" in order to be able to receive an interrupt on 
                              // a change of state of "digital input 10".
                
    PCMSK0 |= (1 << PCINT3);  // bit 3 (PCINT3) of register PCMSK0 is set to "1" in order to be able to receive an interrupt on
                              // a change of state of "digital input 11".
    
}


//#########################################
//  MPU6050 initialization function
//
//  "initializeMPU6050()"
//
//  - no input parameter
//
//  - no value returned
//
//#########################################

void initializeMPU6050()
{

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
void readMPU6050()
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

//##########################################################
//  Function to calculate the average offsets of the
//  gyroscope and the accelerometer
//
//  "calibrateMPU6050()"
//
//  the average of 2000 samples is calculated, 
//  during these measurements the MPU-6050 must remain
//  horizontal and stationary.    
//        
//  - no input parameter
//
//  - no value returned
//
//##########################################################

void calibrateMPU6050()
{
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
    
    PORTD |= B11110000;       // setting to "1" of the outputs connected to the ESCs
    delayMicroseconds(1000);  // waiting time of 1000 microseconds (1ms)
    PORTD &= B00001111;       // setting to "0" of the outputs connected to the ESCs
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

//##########################################################
//  Reset function for variables used to calculate PID
//  corrections.
//        
//  "resetPIDControllers()"
//
//  - no input parameter
//
//  - no value returned
//
//##########################################################
void resetPIDControllers()
{
    error[ROLL] = 0;
    error[PITCH] = 0;
    error[YAW] = 0;

    integral_error[ROLL] = 0;
    integral_error[PITCH] = 0;
    integral_error[YAW] = 0;
    
    derivative_error[ROLL] = 0;
    derivative_error[PITCH] = 0;
    derivative_error[YAW] = 0;
    
    error_storage[ROLL] = 0;
    error_storage[PITCH] = 0;
    error_storage[YAW] = 0;   

    P_correction[ROLL] = 0;
    I_correction[ROLL] = 0;
    D_correction[ROLL] = 0;

    P_correction[PITCH] = 0;
    I_correction[PITCH] = 0;
    D_correction[PITCH] = 0;
   
    P_correction[YAW] = 0;
    I_correction[YAW] = 0;
    D_correction[YAW] = 0;

    PID_correction[ROLL] = 0;
    PID_correction[PITCH] = 0;
    PID_correction[YAW] = 0;
    
}

//###################################################################
//  Function that sets the duration of the ESC control pulses to
//  the minimum value of 1000.
//        
//  "stopMotors()"
//
//  - no input parameter
//
//  - no value returned
//
//###################################################################
void stopMotors()
{
    ESC_pulse_duration[ESC1] = 1000;
    ESC_pulse_duration[ESC2] = 1000;
    ESC_pulse_duration[ESC3] = 1000;
    ESC_pulse_duration[ESC4] = 1000;
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
void calculateAnglesFusion()    
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
    
}



//####################################################################
//  Function for converting receiver signals to setpoint values
//        
//  "calculateSetpoints()"
//
//  - no input parameter
//
//  - no value returned
//
//####################################################################
void calculateSetpoints()
{
    // calculation of PIDs set points

    // Note: A small dead band of approx. 16us is required around the
    // middle points of the control joysticks, otherwise the control
    // is unstable when the joysticks are in the middle position.
    //
    // Reminder of channel assignment :
    //
    //      ROLL   --> Channel 1  
    //      PITCH  --> Channel 2
    //      GAS    --> Channel 3
    //      YAW    --> Channel 4
    //      


    //#######################
    //#### ROLL setpoint ####
    //#######################
    
    // the Roll setpoint in degrees/sec is determined from the pulse durations
    // of the receiver's Roll channel
    // these pulse durations must be converted to deg/sec to drive the PID input
    //
    // an angular velocity of 0,33tr/sec max seems reasonable, which corresponds
    // to 120deg/s
    //
    // the corresponding equations are :
    //
    //       for the interval [1000us - 1492us] : angular_velovity_setpoint = 0.244 x setpoint_pulse_duration - 363,90
    //
    //       for the interval [1508us - 2000us] : angular_velovity_setpoint = 0.244 x setpoint_pulse_duration - 367,80
    //  

    setpoint[ROLL] = 0;
    
    if(pulse_duration[CHANNEL1] > 1508)
    {
          setpoint[ROLL] = 0.244*pulse_duration[CHANNEL1] - 367.80;
    }
    else if(pulse_duration[CHANNEL1] < 1492)
    {
          setpoint[ROLL] = 0.244*pulse_duration[CHANNEL1] - 363,90;
    }

    //########################
    //#### PITCH setpoint ####
    //########################

    setpoint[PITCH] = 0;

    // the PITCH command has a negative slope because a PITCH command lower than 1492us must raise the nose of the UAV
    if(pulse_duration[CHANNEL2] > 1508)
    {	 
          setpoint[PITCH] = -0.244*pulse_duration[CHANNEL2] + 367.80;
    }
    else if(pulse_duration[CHANNEL2] < 1492)
    {
		  setpoint[PITCH] = -0.244*pulse_duration[CHANNEL2] + 363.90;
    }

    //######################
    //#### YAW setpoint ####
    //######################    

    setpoint[YAW] = 0;

    if(pulse_duration[CHANNEL3] > 1050)    // the YAW setpoint is only calculated if the GAS control is non-zero, otherwise
    {                                     // when you want to switch to the "STOP" state, the motors accelerate 
      

        if(pulse_duration[CHANNEL4] > 1508)
        {
              setpoint[YAW] = 0.244*pulse_duration[CHANNEL4] - 367.80;
        }
        else if(pulse_duration[CHANNEL4] < 1492)
        {
              setpoint[YAW] = 0.244*pulse_duration[CHANNEL4] - 363,90;
        }

    }

    //#######################
    //#### Stabilization ####
    //#######################
  
    // correction of Roll and Pitch angular speed setpoints to ensure Self-Level

    setpoint[ROLL] -= roll_adjustment;
    setpoint[PITCH] -= pitch_adjustment;
    

    //######################
    //#### GAS setpoint ####
    //######################

    setpoint[GAS] = pulse_duration[CHANNEL3];  // the GAS set point is the copy of Channel 3

    if (setpoint[GAS] > 1700) setpoint[GAS] = 1700; // we limit the GAS value, because PID corrections will be added to it
                                                    // to give the ESC pulse durations, without this limit the PID correction
                                                    // is not taken into account for GAS values higher than 1600us
                                                    // (PID limited to 400 and maximum pulse length = 2000)
}

//#########################################
//  PID commands calculation function
//        
//  "calculatePIDCommands()"
//
//  - no input parameter
//
//  - no value returned
//
//#########################################
void calculatePIDCommands()
{    
 
    // calculation of PID corrections

    //###############
    //   ROLL Axis
    //###############   

    //-------------------------
    // ROLL error calculation
    //-------------------------
    error[ROLL] = setpoint[ROLL] - gyro[ROLL] ;

    //---------------------------------------------
    // calculation of the ROLL proportional action 
    //---------------------------------------------
    P_correction[ROLL] = KpRoll * error[ROLL];

    //-----------------------------------------
    // calculation of the ROLL integral action 
    //-----------------------------------------
    integral_error[ROLL] += error[ROLL];    // condensed formula
  
    I_correction[ROLL] = KiRoll * integral_error[ROLL];  

    // limitation of the I correction value in Roll
    I_correction[ROLL]=limit(I_correction[ROLL], -PID_limit[ROLL], PID_limit[ROLL]);
        
   //----------------------------------------
    // calculating the ROLL derivative action 
    //----------------------------------------           
    
    // the variable "error_storage[ROLL]" is used to store the previous value of "error[ROLL]"
    derivative_error[ROLL] = error[ROLL] - error_storage[ROLL];
    D_correction[ROLL] = KdRoll * derivative_error[ROLL];   
         
    // update of the error storage for the calculation in the next iteration
    error_storage[ROLL] = error[ROLL];
    
    //------------------------------------------------
    // calculation of the complete PID action in ROLL
    //------------------------------------------------

    // complete PID correction in roll
    PID_correction[ROLL] =  P_correction[ROLL] + I_correction[ROLL] + D_correction[ROLL];

    // bounding of the PID correction value
    PID_correction[ROLL]=limit(PID_correction[ROLL], -PID_limit[ROLL], PID_limit[ROLL]);

    //################
    //   PITCH axis
    //################  

    //--------------------------
    // PITCH error calculation
    //--------------------------
    error[PITCH] = setpoint[PITCH] - gyro[PITCH] ;

    //----------------------------------------------
    // calculation of the proportional action PITCH
    //----------------------------------------------
    P_correction[PITCH] = KpPitch * error[PITCH];
  
    //------------------------------------------
    // calculation of the PITCH integral action
    //------------------------------------------
    integral_error[PITCH] += error[PITCH];    // condensed formula
  
    I_correction[PITCH] = KiPitch * integral_error[PITCH];

    // bounding of the I correction value in Pitch
    I_correction[PITCH]=limit(I_correction[PITCH], -PID_limit[PITCH], PID_limit[PITCH]);
        
    //-----------------------------------------
    // calculating the PITCH derivative action
    //-----------------------------------------    
    
    // the variable "error_storage[PITCH]" is used to store the previous value of "error[PITCH]".
    derivative_error[PITCH] = error[PITCH] - error_storage[PITCH];
    D_correction[PITCH] = KdPitch * derivative_error[PITCH];       
         
    // update of the error storage for the calculation in the next iteration
    error_storage[PITCH] = error[PITCH];

    //-------------------------------------------
    // calculating the PITCH complete PID action  
    //-------------------------------------------

    // PITCH complete PID correction
    PID_correction[PITCH] =  P_correction[PITCH] + I_correction[PITCH] + D_correction[PITCH];

    // bounding of the PID correction value
    PID_correction[PITCH]=limit(PID_correction[PITCH], -PID_limit[PITCH], PID_limit[PITCH]);


    //###############
    //   YAW axis
    //###############   

    //------------------------
    // YAW error calculation
    //------------------------
    error[YAW] = setpoint[YAW] - gyro[YAW] ;

    //-------------------------------------------
    // calculation of proportional action in YAW
    //-------------------------------------------
    P_correction[YAW] = KpYaw * error[YAW];

    //---------------------------------------
    // calculation of integral action in YAW
    //---------------------------------------
    integral_error[YAW] += error[YAW];    // condensed formula
  
    I_correction[YAW] = KiYaw * integral_error[YAW];

    // bounding of the I correction value in Yaw
    I_correction[YAW]=limit(I_correction[YAW], -PID_limit[YAW], PID_limit[YAW]);
        
        //--------------------------------------
    // calculating derivative action in YAW
    //--------------------------------------      
    
    // the variable "error_storage[YAW]" is used to store the previous value of "error[YAW]".
    derivative_error[YAW] = error[YAW] - error_storage[YAW];
    D_correction[YAW] = KdYaw * derivative_error[YAW];    
           
    // update of the error storage for the calculation in the next iteration
    error_storage[YAW] = error[YAW];

    //--------------------------------------------
    // calculating the complete action PID in YAW
    //--------------------------------------------

    // complete PID correction in YAW
    PID_correction[YAW] =  P_correction[YAW] + I_correction[YAW] + D_correction[YAW];

    // PID correction value boundary
    PID_correction[YAW]=limit(PID_correction[YAW], -PID_limit[YAW], PID_limit[YAW]);
}

//#############################################################
// Value bounding function
//
//  "float limit(float value, float min_value, float max_value)"
//
// input parameters :
//
// float value : real value to be limited
// float min_value : minimum limit
// float max_value : maximum limit
//
// returned value :
//
// bounded value in float type
//
//#############################################################
float limit(float value, float min_value, float max_value) 
{
    if (value > max_value) 
    {
        value = max_value;
    } 
    else if (value < min_value) 
    {
        value = min_value;
    }
 
    return value;
}

//###############################################
//  Function to calculate ESCs pulse durations
//        
//  "calculateESCPulses()"
//
//  - no input parameter
//
//  - no value returned
//
//###############################################
void calculateESCPulses()
{
    //---------------------
    // PID command mixing
    //---------------------

    ESC_pulse_duration[ESC1] = setpoint[GAS] + PID_correction[PITCH] - PID_correction[ROLL] + PID_correction[YAW];

    ESC_pulse_duration[ESC2] = setpoint[GAS] - PID_correction[PITCH] - PID_correction[ROLL] - PID_correction[YAW];

    ESC_pulse_duration[ESC3] = setpoint[GAS] - PID_correction[PITCH] + PID_correction[ROLL] + PID_correction[YAW];

    ESC_pulse_duration[ESC4] = setpoint[GAS] + PID_correction[PITCH] + PID_correction[ROLL] - PID_correction[YAW];

    // limit of pulse durations calculated between 1100 and 2000, if you start at 1000, some motors do not start.
    ESC_pulse_duration[ESC1] = limit(ESC_pulse_duration[ESC1], 1100, 2000);
    ESC_pulse_duration[ESC2] = limit(ESC_pulse_duration[ESC2], 1100, 2000);
    ESC_pulse_duration[ESC3] = limit(ESC_pulse_duration[ESC3], 1100, 2000);
    ESC_pulse_duration[ESC4] = limit(ESC_pulse_duration[ESC4], 1100, 2000);
}

//##################################################################
//  Function for generating ESC pulses and reading measurements
//  from the MPU-6050
//
//  Just after the generation of the rising edges of the ESC pulses
//  there is 1000us of free time left to read the measurements of the
//  MPU-6050 and calculate the time interval "dT" elapsed since the
//  last measurement was taken.
//        
//  "generateESCPulses()"
//
//  - no input parameter
//
//  - no value returned
//
//##################################################################
void generateESCPulses()
{
  
    ESC_pulse_start_time = micros();  // the start time of the elapsed time
                                      // counter is updated

    PORTD |= B11110000; // all outputs connected to the ESCs are set to "1"

    ESC_pulse_end_time[ESC1] = ESC_pulse_start_time + ESC_pulse_duration[ESC1];
    ESC_pulse_end_time[ESC2] = ESC_pulse_start_time + ESC_pulse_duration[ESC2];
    ESC_pulse_end_time[ESC3] = ESC_pulse_start_time + ESC_pulse_duration[ESC3];
    ESC_pulse_end_time[ESC4] = ESC_pulse_start_time + ESC_pulse_duration[ESC4];

    // after the start of the pulses, there is at least 1000us of free time
    // because the ESC pulses have a minimum duration of 1000us
    // the MPU6050 data can be read out during this time

    // calculation of the time elapsed since the last measurements,
    // necessary to precisely integrate the values of the angular velocities.
    dT = micros() - time_storage;
    
    // taken from the present moment at the time of measurement
    time_storage = micros();
        
    readMPU6050();

    // generation of falling edges
    while(PORTD >=16)
    {
        present_time = micros();
        if(ESC_pulse_end_time[ESC1] <= present_time) PORTD &= B11101111;
        if(ESC_pulse_end_time[ESC2] <= present_time) PORTD &= B11011111;
        if(ESC_pulse_end_time[ESC3] <= present_time) PORTD &= B10111111;
        if(ESC_pulse_end_time[ESC4] <= present_time) PORTD &= B01111111;
      
    }
    
}

//##############################################################################
//  Waiting function for "safety" positioning of the left joystick (mode 2) 
//  i.e. the left joystick positioned at the bottom left.
//        
//  "waitMinGas()"
//
//  - no input parameter
//
//  - no value returned
//
//##############################################################################
void waitMinGas()
{
        // function containing a "while" safety loop so that the motors do not start rotating as soon
        // as the quadricopter is switched on
        //
        // one waits for a safety position of the radio control joysticks before accepting orders from it
        // the chosen position is that of the left joystick (in radio control mode 2) : down and centered
        // which means: minimum GAS control and middle YAW control.
        //
        // we stay in the "while" security loop as long as :
        //
        //      - the GAS control is not in low position ("pulse_duration[CHANNEL3]" between 990 and 1020) 
        // 
        //      - with the median YAW command ("pulse_duration[CHANNEL4]" between 1450 and 1550)                                         
        //
        // Reminder of the channels :
        //
        //      ROLL   --> CHANNEL 1  
        //      PITCH  --> CHANNEL 2
        //      GAS    --> CHANNEL 3
        //      YAW    --> CHANNEL 4
        //
        //
        // Reminder of received value ranges :
        //
        //     for the joysticks of the 3 axes and the GAS control :
        //         - a central position returns a value of 1500
        //         - a low (or left) position returns a value less than 1500 (1000 minimum)
        //         - a high (or right) position returns a value greater than 1500 (2000 maximum)
        //
        
		    int cpt=0;

        // loop as long as the throttle control is above the minimum (with Yaw control in the center)
		
        while(pulse_duration[CHANNEL3] <990 || pulse_duration[CHANNEL3] > 1020 || pulse_duration[CHANNEL4] < 1450 || pulse_duration[CHANNEL4] > 1550 )   // boucle tant que la commande des GAS est supérieure au minimum      
        {
              cpt ++; // the variable "cpt" is incremented at each pass in the loop
              
              // we don't want the ESCs to beep continuously waiting for the correct positioning of the joystick,
              // so we send them short pulses while waiting for the right values from the RF receiver
              
              PORTD |= B11110000;       // sets PINS 4, 5, 6 and 7 of port D to "high" (ESC 1 to 4)
              delayMicroseconds(1000);  // wait for 1000us
                                                         
              PORTD &= B00001111;     // sets PINS 4, 5, 6 and 7 of port D to "low".
              delay(3);               // waiting 3 milliseconds before the next loop, i.e. a PWM frequency of about 250Hz

              // slow flashing of the LED while waiting for the correct positioning of the left joystick
              if(cpt == 125)  // every 125 loops (#500ms)
              {
                    digitalWrite(LED, !digitalRead(LED));  // inversion of the LED state with respect to its current state read on pin 12
                    cpt = 0;                               // resets the variable "cpt" to 0
              }
        }

        digitalWrite(LED,LOW);  // the LED is switched off
}


//##########################################################
//  Interrupt routine that detects state changes on
//  receiver channels
//
//  "ISR(PCINT0_vect)"
//
//  The calculated PWM pulse durations are placed in the
//  global variables :
//  
//        - pulse_duration[CHANNEL1]
//
//        - pulse_duration[CHANNEL2]
//
//        - pulse_duration[CHANNEL3]
//
//        - pulse_duration[CHANNEL4]
//        
//  - no input parameter
//
//  - no value returned
//
//##########################################################
ISR(PCINT0_vect) 
{
   current_time = micros();
   
   //########################
   // treatment of channel 1
   //########################
   if (PINB & B00000001) // if pin 8 is "HIGH"
   {                                       
       // we test if the memorized state of pin 8 was "LOW"
       if (ch_status_storage[CHANNEL1] == LOW) 
       {                     

          // then pin 8 made a rising edge

          // the memory variable of pin 8 state is updated

          ch_status_storage[CHANNEL1] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL1] = current_time;                        
       }
   } 

   // otherwise if pin 8 is at "LOW" and the storage of the state
   // of pin 8 is at "HIGH"

   else if (ch_status_storage[CHANNEL1] == HIGH) 
  
   {   // pin 8 went from "1" to "0" (falling edge)
       // the memory variable of pin 8 state is updated

       ch_status_storage[CHANNEL1] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL1] = current_time-pulse_start_t[CHANNEL1];   
   }

   //########################
   // treatment of channel 2
   //########################
   if (PINB & B00000010) // if pin 9 is "HIGH"
   {                                       
       // we test if the memorized state of pin 9 was "LOW"
       if (ch_status_storage[CHANNEL2] == LOW) 
       {                     

          // then pin 9 made a rising edge

          // the memory variable of pin 9 state is updated

          ch_status_storage[CHANNEL2] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL2] = current_time;                        
       }
   } 

   // otherwise if pin 9 is at "LOW" and the storage of the state
   // of pin 9 is at "HIGH"

   else if (ch_status_storage[CHANNEL2] == HIGH) 
  
   {   // pin 9 went from "1" to "0" (falling edge)
       // the memory variable of pin 9 state is updated

       ch_status_storage[CHANNEL2] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL2] = current_time-pulse_start_t[CHANNEL2];   
   }

   //########################
   // treatment of channel 3
   //########################
   if (PINB & B00000100) // if pin 10 is "HIGH"
   {                                       
       // we test if the memorized state of pin 10 was "LOW"
       if (ch_status_storage[CHANNEL3] == LOW) 
       {                     

          // // then pin 10 made a rising edge

          // the memory variable of pin 10 state is updated

          ch_status_storage[CHANNEL3] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL3] = current_time;                        
       }
   } 

   // otherwise if pin 10 is at "LOW" and the storage of the state
   // of pin 10 is at "HIGH"

   else if (ch_status_storage[CHANNEL3] == HIGH) 
  
   {   // pin 10 went from "1" to "0" (falling edge)
       // the memory variable of pin 10 state is updated

       ch_status_storage[CHANNEL3] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL3] = current_time-pulse_start_t[CHANNEL3];   
   }

   //########################
   // treatment of channel 4
   //########################
   if (PINB & B00001000) //  if pin 11 is "HIGH"
   {                                       
       // // we test if the memorized state of pin 11 was "LOW" 
       if (ch_status_storage[CHANNEL4] == LOW) 
       {                     

          // then pin 11 made a rising edge

          // the memory variable of pin 11 state is updated

          ch_status_storage[CHANNEL4] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL4] = current_time;                        
       }
   } 

   // otherwise if pin 11 is at "LOW" and the storage of the state
   // of pin 11 is at "HIGH"

   else if (ch_status_storage[CHANNEL4] == HIGH) 
  
   {   // pin 11 went from "1" to "0" (falling edge)
       // the memory variable of pin 11 state is updated

       ch_status_storage[CHANNEL4] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL4] = current_time-pulse_start_t[CHANNEL4];   
   }
   
}
