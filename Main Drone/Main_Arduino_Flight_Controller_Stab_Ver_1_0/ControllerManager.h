#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H


class ControllerManager{
  public:


  private:

  
void calculatePIDCommands();  // calculation of PID corrections

void calculateESCPulses();    // calculation of PWM pulse durations to be sent to the ESCs

void generateESCPulses();     // generation of ESC pulses and reading of MPU-6050 measurements

void resetPIDControllers();   // resetting the variables used to calculate PID corrections to zero

void stopMotors();            // sets the pulse duration of the ESCs to 1000 (minimum value)


float limit(float value, float min_value, float max_value); // function to limit a "float" type quantity between 2 values



//--------------------------------------------------------------------------
// variables used to calculate the elapsed time between 2 MPU6050 readings
//--------------------------------------------------------------------------

unsigned long time_storage; // memorization variable of the present moment

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



//--------------------------------------------
// variables used to generate the ESC pulses
//--------------------------------------------

unsigned long present_time; // variable for storing the current time to know when 
                            // to generate the falling edges of the ESC control signals
                            
unsigned long ESC_pulse_start_time;   // start time of ESC pulses generation

unsigned int ESC_pulse_duration[4]= {1000,1000,1000,1000};  // duration of the control pulses 
                                                            // to be sent to the ESCs
                                                            
unsigned long ESC_pulse_end_time[4];   // end times of the ESC pulses




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


};
#endif // CONTROLLERMANAGER_H