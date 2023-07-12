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
    m_error[ROLL] = 0;
    m_error[PITCH] = 0;
    m_error[YAW] = 0;

    m_integral_error[ROLL] = 0;
    m_integral_error[PITCH] = 0;
    m_integral_error[YAW] = 0;
    
    m_derivative_error[ROLL] = 0;
    m_derivative_error[PITCH] = 0;
    m_derivative_error[YAW] = 0;
    
    m_error_storage[ROLL] = 0;
    m_error_storage[PITCH] = 0;
    m_error_storage[YAW] = 0;   

    m_P_correction[ROLL] = 0;
    m_I_correction[ROLL] = 0;
    m_D_correction[ROLL] = 0;

    m_P_correction[PITCH] = 0;
    m_I_correction[PITCH] = 0;
    m_D_correction[PITCH] = 0;
   
    m_P_correction[YAW] = 0;
    m_I_correction[YAW] = 0;
    m_D_correction[YAW] = 0;

    m_PID_correction[ROLL] = 0;
    m_PID_correction[PITCH] = 0;
    m_PID_correction[YAW] = 0;
    
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
    m_ESC_pulse_duration[ESC1] = 1000;
    m_ESC_pulse_duration[ESC2] = 1000;
    m_ESC_pulse_duration[ESC3] = 1000;
    m_ESC_pulse_duration[ESC4] = 1000;
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
    
    m_error[ROLL] = m_setpoint[ROLL] - m_angle[ROLL] ;

    //---------------------------------------------
    // calculation of the ROLL proportional action 
    //---------------------------------------------
    m_P_correction[ROLL] = m_KpRoll * m_error[ROLL];

    //-----------------------------------------
    // calculation of the ROLL integral action 
    //-----------------------------------------
    m_integral_error[ROLL] += m_error[ROLL];    // condensed formula
  
    m_I_correction[ROLL] = m_KiRoll * m_integral_error[ROLL];  

    // limitation of the I correction value in Roll
    m_I_correction[ROLL]=limit(m_I_correction[ROLL], -m_PID_limit[ROLL], m_PID_limit[ROLL]);
        
   //----------------------------------------
    // calculating the ROLL derivative action 
    //----------------------------------------           
    
    // the variable "error_storage[ROLL]" is used to store the previous value of "error[ROLL]"
    m_derivative_error[ROLL] = m_error[ROLL] - m_error_storage[ROLL];
    m_D_correction[ROLL] = m_KdRoll * m_derivative_error[ROLL];   
         
    // update of the error storage for the calculation in the next iteration
    m_error_storage[ROLL] = m_error[ROLL];
    
    //------------------------------------------------
    // calculation of the complete PID action in ROLL
    //------------------------------------------------

    // complete PID correction in roll
    m_PID_correction[ROLL] =  m_P_correction[ROLL] + m_I_correction[ROLL] + m_D_correction[ROLL];

    // bounding of the PID correction value
    m_PID_correction[ROLL]=limit(m_PID_correction[ROLL], -m_PID_limit[ROLL], m_PID_limit[ROLL]);

    //################
    //   PITCH axis
    //################  

    //--------------------------
    // PITCH error calculation
    //--------------------------
    
    m_error[PITCH] = m_setpoint[PITCH] - m_angle[PITCH] ;

    //----------------------------------------------
    // calculation of the proportional action PITCH
    //----------------------------------------------
    m_P_correction[PITCH] = m_KpPitch * m_error[PITCH];
  
    //------------------------------------------
    // calculation of the PITCH integral action
    //------------------------------------------
    m_integral_error[PITCH] += m_error[PITCH];    // condensed formula
  
    m_I_correction[PITCH] = m_KiPitch * m_integral_error[PITCH];

    // bounding of the I correction value in Pitch
    m_I_correction[PITCH]=limit(m_I_correction[PITCH], -m_PID_limit[PITCH], m_PID_limit[PITCH]);
        
    //-----------------------------------------
    // calculating the PITCH derivative action
    //-----------------------------------------    
    
    // the variable "error_storage[PITCH]" is used to store the previous value of "error[PITCH]".
    m_derivative_error[PITCH] = m_error[PITCH] - m_error_storage[PITCH];
    m_D_correction[PITCH] = m_KdPitch * m_derivative_error[PITCH];       
         
    // update of the error storage for the calculation in the next iteration
    m_error_storage[PITCH] = m_error[PITCH];

    //-------------------------------------------
    // calculating the PITCH complete PID action  
    //-------------------------------------------

    // PITCH complete PID correction
    m_PID_correction[PITCH] =  m_P_correction[PITCH] + m_I_correction[PITCH] + m_D_correction[PITCH];

    // bounding of the PID correction value
    m_PID_correction[PITCH]=limit(m_PID_correction[PITCH], -m_PID_limit[PITCH], m_PID_limit[PITCH]);


    //###############
    //   YAW axis
    //###############   

    //------------------------
    // YAW error calculation
    //------------------------
   
    m_error[YAW] = m_setpoint[YAW] - m_angle[YAW] ;

    //-------------------------------------------
    // calculation of proportional action in YAW
    //-------------------------------------------
    m_P_correction[YAW] = m_KpYaw * m_error[YAW];

    //---------------------------------------
    // calculation of integral action in YAW
    //---------------------------------------
    m_integral_error[YAW] += m_error[YAW];    // condensed formula
  
    m_I_correction[YAW] = m_KiYaw * m_integral_error[YAW];

    // bounding of the I correction value in Yaw
    m_I_correction[YAW]=limit(m_I_correction[YAW], -m_PID_limit[YAW], m_PID_limit[YAW]);
        
        //--------------------------------------
    // calculating derivative action in YAW
    //--------------------------------------      
    
    // the variable "error_storage[YAW]" is used to store the previous value of "error[YAW]".
    m_derivative_error[YAW] = m_error[YAW] - m_error_storage[YAW];
    m_D_correction[YAW] = m_KdYaw * m_derivative_error[YAW];    
           
    // update of the error storage for the calculation in the next iteration
    m_error_storage[YAW] = m_error[YAW];

    //--------------------------------------------
    // calculating the complete action PID in YAW
    //--------------------------------------------

    // complete PID correction in YAW
    m_PID_correction[YAW] =  m_P_correction[YAW] + m_I_correction[YAW] + m_D_correction[YAW];

    // PID correction value boundary
    m_PID_correction[YAW]=limit(m_PID_correction[YAW], -m_PID_limit[YAW], m_PID_limit[YAW]);
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

    m_ESC_pulse_duration[ESC1] = m_setpoint[GAS] + m_PID_correction[PITCH] - m_PID_correction[ROLL] + m_PID_correction[YAW];

    m_ESC_pulse_duration[ESC2] = m_setpoint[GAS] - m_PID_correction[PITCH] - m_PID_correction[ROLL] - m_PID_correction[YAW];

    m_ESC_pulse_duration[ESC3] = m_setpoint[GAS] - m_PID_correction[PITCH] + m_PID_correction[ROLL] + m_PID_correction[YAW];

    m_ESC_pulse_duration[ESC4] = m_setpoint[GAS] + m_PID_correction[PITCH] + m_PID_correction[ROLL] - m_PID_correction[YAW];

    // limit of pulse durations calculated between 1100 and 2000, if you start at 1000, some motors do not start.
    m_ESC_pulse_duration[ESC1] = limit(m_ESC_pulse_duration[ESC1], 1100, 2000);
    m_ESC_pulse_duration[ESC2] = limit(m_ESC_pulse_duration[ESC2], 1100, 2000);
    m_ESC_pulse_duration[ESC3] = limit(m_ESC_pulse_duration[ESC3], 1100, 2000);
    m_ESC_pulse_duration[ESC4] = limit(m_ESC_pulse_duration[ESC4], 1100, 2000);
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
  
    m_ESC_pulse_start_time = micros();  // the start time of the elapsed time
                                      // counter is updated

    PORTD |= B11110000; // all outputs connected to the ESCs are set to "1"

    m_ESC_pulse_end_time[ESC1] = m_ESC_pulse_start_time + m_ESC_pulse_duration[ESC1];
    m_ESC_pulse_end_time[ESC2] = m_ESC_pulse_start_time + m_ESC_pulse_duration[ESC2];
    m_ESC_pulse_end_time[ESC3] = m_ESC_pulse_start_time + m_ESC_pulse_duration[ESC3];
    m_ESC_pulse_end_time[ESC4] = m_ESC_pulse_start_time + m_ESC_pulse_duration[ESC4];

    // after the start of the pulses, there is at least 1000us of free time
    // because the ESC pulses have a minimum duration of 1000us
    // the MPU6050 data can be read out during this time

    // calculation of the time elapsed since the last measurements,
    // necessary to precisely integrate the values of the angular velocities.
    
    // In both MPU6050Manager::set_dT(micros() - time_storage);
    
    // taken from the present moment at the time of measurement
    m_time_storage = micros();
        
    // copied MPU6050Manager::readMPU6050();

    // generation of falling edges
    while(PORTD >=16)
    {
        m_present_time = micros();
        if(m_ESC_pulse_end_time[ESC1] <= m_present_time) PORTD &= B11101111;
        if(m_ESC_pulse_end_time[ESC2] <= m_present_time) PORTD &= B11011111;
        if(m_ESC_pulse_end_time[ESC3] <= m_present_time) PORTD &= B10111111;
        if(m_ESC_pulse_end_time[ESC4] <= m_present_time) PORTD &= B01111111;
      
    }
    
}
