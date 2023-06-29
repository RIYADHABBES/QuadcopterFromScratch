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

#include "MPU6050Manager.h"
#define  MESSUP
////////////////////////////////////////
////// Declaration of the "define //////
////////////////////////////////////////

// the different "#define" to improve the readability of the code

// #define MPU_ADDRESS 0x68  // MPU-6050 address = 0x68  // Class copied

// copied
#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3

#define STOP   0   // STOP equals 0
#define ARMED 1   // ARMED equals 1
#define RUN   2   // RUN equals 2

#define ESC1 0    // ESC1 equals 0
#define ESC2 1    // ESC2 equals 1
#define ESC3 2    // ESC3 equals 2
#define ESC4 3    // ESC4 equals 3

// Wet Floor
#define ESC1_pin 5
#define ESC2_pin 6
#define ESC3_pin 9
#define ESC4_pin 3

// End Wet Floor

// Original #define LED 12    // LED equals 12
// In Both
#define LED 13    // LED equals 13 Built-in Nano board LED

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

//----------------------------------
// declaration of global variables
//----------------------------------

//--------------------------------------------------------------------------
// variables used to calculate the elapsed time between 2 MPU6050 readings
//--------------------------------------------------------------------------

unsigned long time_storage; // memorization variable of the present moment


//--------------------------------------------
// variables used to generate the ESC pulses
//--------------------------------------------

unsigned long present_time; // variable for storing the current time to know when 
                            // to generate the falling edges of the ESC control signals
                            
unsigned long ESC_pulse_start_time;   // start time of ESC pulses generation

unsigned int ESC_pulse_duration[4]= {1000,1000,1000,1000};  // duration of the control pulses 
                                                            // to be sent to the ESCs
                                                            
unsigned long ESC_pulse_end_time[4];   // end times of the ESC pulses
//-------------------------------------------
// variables for calculating PID corrections
//-------------------------------------------
                        


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

void configureInterrupts(); // function to configure the pins that trigger an interrupt


void calculatePIDCommands();  // calculation of PID corrections

void calculateESCPulses();    // calculation of PWM pulse durations to be sent to the ESCs

void generateESCPulses();     // generation of ESC pulses and reading of MPU-6050 measurements

void resetPIDControllers();   // resetting the variables used to calculate PID corrections to zero

void stopMotors();            // sets the pulse duration of the ESCs to 1000 (minimum value)


float limit(float value, float min_value, float max_value); // function to limit a "float" type quantity between 2 values

class MPU6050Manager;
//#######################################################
//####                     SETUP()                   ####
//#######################################################
void setup(int a) 
{  Serial.begin(9600);
    configureOutputs(); // configuration of the pins used as outputs 

    // copied MPU6050Manager::initializeMPU6050(); // MPU-6050 initialization function

    // copied MPU6050Manager::calibrateMPU6050(); // MPU6050 calibration function : calculation of gyroscope offsets
  
    // configuration of the pins used as interrupt triggers
    configureInterrupts();
     
    // waiting as long as the throttle joystick is not at the minimum and Yaw centered
    // JoyPadManager::waitMinGas();
    Serial.println("Wait 2 seconds");
       delay(2000) ;
    // after executing "waitMinGas()", the quadcopter status corresponds to the joystick safety position
    // the variable "uav_status" is then set to "STOP".
  
    uav_status = STOP; // sets the status to "STOP".

    // taking the present moment when reading the MPU-6050 measurements to calculate the "dT" that will
    // be used to integrate the angular velocities
    time_storage = micros();

    // copied MPU6050Manager::readMPU6050();  // reading of angular velocities
    
}
//##### End of "SETUP()" #####


//####################################################
//####                    LOOP()                  ####
//####################################################

void loop(int b) 
{
        // take the start time of the "loop()" to manage the loop duration
        loop_start = micros();   
        
        // processing of gyro measurements to obtain reliable angular velocity values
        // and calculation of angles by merging gyro and accelerometer measurements
        // copied MPU6050Manager::calculateAnglesFusion();     

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
        
        if (uav_status == STOP/* && JoyPadManager::pulse_duration[CHANNEL3] < 1050 && JoyPadManager::pulse_duration[CHANNEL4] < 1050*/) 
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
      
        
        if (uav_status == ARMED/* && JoyPadManager::pulse_duration[CHANNEL3] < 1050 && JoyPadManager::pulse_duration[CHANNEL4] > 1450*/) 
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
      
            
        if (uav_status == RUN/* && JoyPadManager::pulse_duration[CHANNEL3] < 1050 && JoyPadManager::pulse_duration[CHANNEL4] > 1950*/) 
        {  
          uav_status = STOP;
        }

        // conversion of order orders received into data usable by PIDs
        
        //JoyPadManager::calculateSetpoints();
      
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

         // MPU6050Manager::init_gyro_angles = false; // we authorize the assignment of the angles coming from the accelerometer in
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
   /* Original
    DDRD |= B11110000;  // configuration of pins 4, 5, 6 and 7 of port D as outputs
    DDRB |= B00010000;  // configuring pin 12 of port B as output
    */

    pinMode(ESC1_pin, OUTPUT); // ESC 1 -> 5
    pinMode(ESC2_pin, OUTPUT); // ESC 2 -> 6
    pinMode(ESC3_pin, OUTPUT); // ESC 3 -> 9
    pinMode(ESC4_pin, OUTPUT); // ESC 4 -> 3

    pinMode(LED, OUTPUT); // LED Not mounted yet

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
    
    // In both error[ROLL] = setpoint[ROLL] - gyro[ROLL] ;

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
    
    // In both error[PITCH] = setpoint[PITCH] - gyro[PITCH] ;

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
   
   // In both error[YAW] = setpoint[YAW] - gyro[YAW] ;

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

    ESC_pulse_duration[ESC1] = /*JoyPadManager::setpoint[GAS]*/ + PID_correction[PITCH] - PID_correction[ROLL] + PID_correction[YAW];

    ESC_pulse_duration[ESC2] = /*JoyPadManager::setpoint[GAS]*/ - PID_correction[PITCH] - PID_correction[ROLL] - PID_correction[YAW];

    ESC_pulse_duration[ESC3] = /*JoyPadManager::setpoint[GAS]*/ - PID_correction[PITCH] + PID_correction[ROLL] + PID_correction[YAW];

    ESC_pulse_duration[ESC4] = /*JoyPadManager::setpoint[GAS]*/ + PID_correction[PITCH] + PID_correction[ROLL] - PID_correction[YAW];

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
    
    // In both MPU6050Manager::set_dT(micros() - time_storage);
    
    // taken from the present moment at the time of measurement
    time_storage = micros();
        
    // copied MPU6050Manager::readMPU6050();

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
