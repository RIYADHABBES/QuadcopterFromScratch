#ifndef JOYPADMANAGER_H
#define JOYPADMANAGER_H

#include <RF24.h>    
//#include <nRF24L01.h>   
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};

class JoyPadManager {

public:

  JoyPadManager();

  void readRadio() ;
  
  void printData();

  void printSetPoints();

  void calculateSetpoints(float rollAdjustment = 0, float pitchAdjustment = 0);    // calculation of the setpoint values from the PWM pulse durations of the receiver channels

  float const * const getSetPoints() const;

  byte getButton1() const;
private:

void initializeReceiver();

void waitMinGas();       // waiting for the left joystick to be positioned at the bottom left : gas mini, yaw mini



void resetData();



RF24* m_radio/*(7, 8)*/;   // nRF24L01 (CE, CSN)

Data_Package m_data; //Create a variable with the above structure
//---------------------------------------------------------------------------------------------------
// variables used to calculate the duration of the pulses delivered on the channels of the receiver
//---------------------------------------------------------------------------------------------------

volatile unsigned int m_pulse_duration[4]; // pulse durations on the receiver channels

// variables containing flight setpoints : Roll, Pitch, Yaw, Gas/Throttle
float m_setpoint[4] = {0,0,0,0};

volatile unsigned long m_current_time = 0; // current time storage variable used in ISR
                                     // to calculate receiver pulse durations
volatile unsigned long m_lastReceiveTime = 0;  

volatile unsigned long m_pulse_start_t[4]; // table of pulse start times of the receiver channels


volatile bool m_ch_status_storage[4]; // storage of the states of the reception channels
// /!\/!\/!\/!\ /* !byte!  */ 
};

#endif // JOYPADMANAGER_H