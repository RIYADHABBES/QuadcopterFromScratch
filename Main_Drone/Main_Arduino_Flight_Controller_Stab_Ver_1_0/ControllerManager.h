#ifndef CONTROLLERMANAGER_H
#define CONTROLLERMANAGER_H


class ControllerManager{
  public:

  void setSetPointROLL(float setpointROLL){m_setpoint[0] = setpointROLL;};
  void setAngleROLL(float angleROLL){m_angle[0] = angleROLL;};
  float getSetPointROLL(){return m_setpoint[0];};
  float getAngleROLL(){return m_angle[0];};
  float getInputROLL(){return m_PID_correction[0];};

  void calculatePIDCommands();  // calculation of PID corrections
  private:

  


void calculateESCPulses();    // calculation of PWM pulse durations to be sent to the ESCs

void generateESCPulses();     // generation of ESC pulses and reading of MPU-6050 measurements

void resetPIDControllers();   // resetting the variables used to calculate PID corrections to zero

void stopMotors();            // sets the pulse duration of the ESCs to 1000 (minimum value)


float limit(float value, float min_value, float max_value); // function to limit a "float" type quantity between 2 values


float m_setpoint[3];
float m_angle[3];
//--------------------------------------------------------------------------
// variables used to calculate the elapsed time between 2 MPU6050 readings
//--------------------------------------------------------------------------

unsigned long m_time_storage; // memorization variable of the present moment

//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
//\\\\                   PID SETTING PARAMETERS                    \\\\          
//\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\


const float m_KpRoll = 0.80;   // value of proportional gain in roll
const float m_KiRoll = 0.03;   // value of integral gain in roll
const float m_KdRoll = 12.6;   // value of derivative gain in roll
const float m_PID_Roll_limit = 400; // limit in absolute value of the
                                  // PID correction in Roll

const float m_KpPitch = m_KpRoll;   // value of proportional gain in pitch
const float m_KiPitch = m_KiRoll;   // value of integral gain in pitch
const float m_KdPitch = m_KdRoll;   // value of derivative gain in pitch
const float m_PID_Pitch_limit = m_PID_Roll_limit; // limit in absolute value of the
                                              // PID correction in Pitch

const float m_KpYaw = 4.00;   // value of proportional gain in yaw
const float m_KiYaw = 0.02;   // value of integral gain in yaw
const float m_KdYaw = 0.00;   //  value of derivative gain in yaw
const float m_PID_Yaw_limit = 400; // limit in absolute value of the
                                 // PID correction in Yaw



//--------------------------------------------
// variables used to generate the ESC pulses
//--------------------------------------------

unsigned long m_present_time; // variable for storing the current time to know when 
                            // to generate the falling edges of the ESC control signals
                            
unsigned long m_ESC_pulse_start_time;   // start time of ESC pulses generation

unsigned int m_ESC_pulse_duration[4]= {1000,1000,1000,1000};  // duration of the control pulses 
                                                            // to be sent to the ESCs
                                                            
unsigned long m_ESC_pulse_end_time[4];   // end times of the ESC pulses




// calculated correction values for P, I and D

float m_P_correction[3] = {0,0,0}; // table of proportional corrections in roll, pitch and yaw  
float m_I_correction[3] = {0,0,0}; // table of integral corrections in roll, pitch and yaw
float m_D_correction[3] = {0,0,0}; // table of derivative corrections in roll, pitch and yaw  
float m_PID_correction[3] = {0,0,0}; // table of complete PID corrections in roll, pitch and yaw  
float m_PID_limit[3] = {m_PID_Roll_limit, m_PID_Pitch_limit, m_PID_Yaw_limit}; // limits in absolute values
                                                                       // of PID corrections in Roll, 
                                                                       // Pitch and Yaw

// variables needed for PID calculations
float m_error[3]= {0,0,0}; // error table for roll, pitch and yaw
float m_integral_error[3] = {0,0,0}; // table of error integrals (for PID calculations)
float m_derivative_error[3] = {0,0,0}; // error derivatives table (for PID calculations)
float m_error_storage[3] = {0,0,0}; // roll, pitch and yaw error memory table (for PID calculations)


};
#endif // CONTROLLERMANAGER_H