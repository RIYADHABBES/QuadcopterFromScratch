#ifndef JOYPADMANAGER_H
#define JOYPADMANAGER_H



class JoyPadManager {

public:


private:

void waitMinGas();       // waiting for the left joystick to be positioned at the bottom left : gas mini, yaw mini

void calculateSetpoints();    // calculation of the setpoint values from the PWM pulse durations of the receiver channels

void I_R(/*PCINT0_vect*/) ;



//---------------------------------------------------------------------------------------------------
// variables used to calculate the duration of the pulses delivered on the channels of the receiver
//---------------------------------------------------------------------------------------------------

volatile unsigned int m_pulse_duration[4]; // pulse durations on the receiver channels

// variables containing flight setpoints : Roll, Pitch, Yaw, Gas/Throttle
float m_setpoint[4] = {0,0,0,0};

volatile unsigned long m_current_time; // current time storage variable used in ISR
                                     // to calculate receiver pulse durations
                                        
volatile unsigned long m_pulse_start_t[4]; // table of pulse start times of the receiver channels


volatile bool m_ch_status_storage[4]; // storage of the states of the reception channels
// /!\/!\/!\/!\ /* !byte!  */ 
};

#endif // JOYPADMANAGER_H