#include "ControllerManager.h"

#include <Arduino.h>

#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3

#define ESC1 0    // ESC1 equals 0
#define ESC2 1    // ESC2 equals 1
#define ESC3 2    // ESC3 equals 2
#define ESC4 3    // ESC4 equals 3

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
void ControllerManager::resetPIDControllers()
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
void ControllerManager::stopMotors()
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
void ControllerManager::calculatePIDCommands()
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
float ControllerManager::limit(float value, float min_value, float max_value) 
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
void ControllerManager::calculateESCPulses()
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
void ControllerManager::generateESCPulses()
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
