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
#include "JoyPadManager.h"
#include "ControllerManager.h"

#define  MESSUP
////////////////////////////////////////
////// Declaration of the "define //////
////////////////////////////////////////

// the different "#define" to improve the readability of the code

// #define MPU_ADDRESS 0x68  // MPU-6050 address = 0x68  // Class copied



#define STOP   0   // STOP equals 0
#define ARMED 1   // ARMED equals 1
#define RUN   2   // RUN equals 2

// Wet Floor
#define ESC1_pin 5
#define ESC2_pin 6
#define ESC3_pin 9
#define ESC4_pin 3

// End Wet Floor

// Original #define LED 12    // LED equals 12
// In Both
#define LED 13    // LED equals 13 Built-in Nano board LED


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
    
    // copied ControllerManager::time_storage = micros();

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
            
           // copied ControllerManager::resetPIDControllers(); // reset the PID calculation variables to prevent the motors
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
       
        // copied ControllerManager::calculatePIDCommands();

        ////////////////////////////////////////////////////////////////////
        //  Actions to be carried out according to the status of the UAV  //
        ////////////////////////////////////////////////////////////////////
            
        //--------------
        // "RUN" STATE
        //--------------
            
        if (uav_status == RUN) // ATTENTION : ONE FORGET of a "=" and the state was always at "RUN" !!!!!!!!!!!
        {
          
          // copied ControllerManager::calculateESCPulses(); // calculation of ESC pulse durations
           
        }
        else  
        {                     
          
          //-------------------------
          // "ARMED" or "STOP" STATE
          //-------------------------
        
          //  in all states other than RUN, the motors are switched off
      
          // copied ControllerManager::stopMotors(); // we set a 1000us pulse on the 4 ESCs 

         // MPU6050Manager::init_gyro_angles = false; // we authorize the assignment of the angles coming from the accelerometer in
                                    // the angle variables coming from the gyroscope so that the UAV does not take
                                    // off again with erroneous angle values coming from the gyroscope (the surface
                                    // where the stop took place is not horizontal).
        }   
      
        // ESCs pulse generation

        // copied ControllerManager::generateESCPulses();  // IMPORTANT: the function "generateESCPulses()" contains the function "readMPU6050()"
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
  