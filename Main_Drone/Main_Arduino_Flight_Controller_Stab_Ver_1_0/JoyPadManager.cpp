#include "JoyPadManager.h"

#include <Arduino.h>
//#include <nRF24L01.h>    
//#include <RF24.h>    

#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3

#define LED 13    // LED equals 13 Built-in Nano board LED

const byte address[6] = "00001";

JoyPadManager::JoyPadManager()
{
  m_radio = new RF24(7, 8);//(10, 9);
  initializeReceiver();
  resetData();   
  readRadio();
}
void JoyPadManager::initializeReceiver()
{
  Serial.println("Initializing Receiver ...");
  m_radio->begin();
  m_radio->openReadingPipe(0, address);
  m_radio->setAutoAck(false);
  m_radio->setDataRate(RF24_250KBPS);
  m_radio->setPALevel(RF24_PA_LOW);
  m_radio->startListening(); //  Set the module as receiver  
  Serial.println("End Initializing Receiver.");
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
void JoyPadManager::waitMinGas()
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
        //      - the GAS control is not in low position ("pulse_duration[GAS]" between 990 and 1020) 
        // 
        //      - with the median YAW command ("pulse_duration[YAW]" between 1450 and 1550)                                         
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
		
        while(m_pulse_duration[GAS] <990 || m_pulse_duration[GAS] > 1020 || m_pulse_duration[YAW] < 1450 || m_pulse_duration[YAW] > 1550 )   // boucle tant que la commande des GAS est supérieure au minimum      
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
void JoyPadManager::calculateSetpoints(float rollAdjustment, float pitchAdjustment)
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

    m_setpoint[ROLL] = 0;
    
    if(m_pulse_duration[ROLL] > 1508)
    {
          m_setpoint[ROLL] = 0.244*m_pulse_duration[ROLL] - 367.80;
    }
    else if(m_pulse_duration[ROLL] < 1492)
    {
          m_setpoint[ROLL] = 0.244*m_pulse_duration[ROLL] - 363,90;
    }

    //########################
    //#### PITCH setpoint ####
    //########################

    m_setpoint[PITCH] = 0;

    // the PITCH command has a negative slope because a PITCH command lower than 1492us must raise the nose of the UAV
    if(m_pulse_duration[PITCH] > 1508)
    {	 
          m_setpoint[PITCH] = -0.244*m_pulse_duration[PITCH] + 367.80;
    }
    else if(m_pulse_duration[PITCH] < 1492)
    {
		  m_setpoint[PITCH] = -0.244*m_pulse_duration[PITCH] + 363.90;
    }

    //######################
    //#### YAW setpoint ####
    //######################    

    m_setpoint[YAW] = 0;

    if(m_pulse_duration[GAS] > 1050)    // the YAW setpoint is only calculated if the GAS control is non-zero, otherwise
    {                                     // when you want to switch to the "STOP" state, the motors accelerate 
      

        if(m_pulse_duration[YAW] > 1508)
        {
              m_setpoint[YAW] = 0.244*m_pulse_duration[YAW] - 367.80;
        }
        else if(m_pulse_duration[YAW] < 1492)
        {
              m_setpoint[YAW] = 0.244*m_pulse_duration[YAW] - 363,90;
        }

    }

    //#######################
    //#### Stabilization ####
    //#######################
  
    // correction of Roll and Pitch angular speed setpoints to ensure Self-Level

  m_setpoint[ROLL] -= rollAdjustment;
  m_setpoint[PITCH] -= pitchAdjustment;
    

    //######################
    //#### GAS setpoint ####
    //######################

    m_setpoint[GAS] = m_pulse_duration[GAS];  // the GAS set point is the copy of Channel 3

    if (m_setpoint[GAS] > 1700) m_setpoint[GAS] = 1700; // we limit the GAS value, because PID corrections will be added to it
                                                    // to give the ESC pulse durations, without this limit the PID correction
                                                    // is not taken into account for GAS values higher than 1600us
                                                    // (PID limited to 400 and maximum pulse length = 2000)
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
//        - pulse_duration[ROLL]
//
//        - pulse_duration[PITCH]
//
//        - pulse_duration[GAS]
//
//        - pulse_duration[YAW]
//        
//  - no input parameter
//
//  - no value returned
//
//##########################################################
void JoyPadManager::readRadio() 
{
    // Check whether there is data to be received
  if (m_radio->available()) {
    m_radio->read(&m_data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    m_lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Check whether we keep receving data, or we have a connection between the two modules
  m_current_time = millis();
  if ( m_current_time - m_lastReceiveTime > 3000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
   // Serial.print("/!!!!!!!! DATA RESET\n");
  }

  m_pulse_duration[ROLL] = map(m_data.j2PotY, 0, 255, 1000, 2000);
  m_pulse_duration[PITCH] = map(m_data.j2PotX, 0, 255, 1000, 2000);  
  m_pulse_duration[YAW] = map(m_data.j1PotX, 0, 255, 1000, 2000);
  m_pulse_duration[GAS] = map(m_data.j1PotY, 0, 255, 1000, 2000);
}

void JoyPadManager::printData()
{
 // Print the data in the Serial Monitor
  Serial.print("j1PotX: ");
  Serial.print(m_data.j1PotX);
  Serial.print("; j1PotY: ");
  Serial.print(m_data.j1PotY);
  Serial.print("; button1: ");
  Serial.print(m_data.button1);
  Serial.print("; j2PotX: ");
  Serial.print(m_data.j2PotX); 
  Serial.print("; j2PotY: ");
  Serial.print(m_data.j2PotY); 
  Serial.println(" ");
}

float const * const JoyPadManager::getSetPoints() const
{
  return m_setpoint;
}

byte JoyPadManager::getButton1() const
{
  return m_data.button1;
}

byte JoyPadManager::getTSwitch2() const
{
  return m_data.tSwitch2;
}

void JoyPadManager::printSetPoints()
{
 // Print the set points in the Serial Monitor
  Serial.print("Set Point:  ROLL: ");
  Serial.print(m_setpoint[ROLL]);
  Serial.print("; PITCH: ");
  Serial.print(m_setpoint[PITCH]);
  Serial.print("; GAS: ");
  Serial.print(m_setpoint[GAS]);
  Serial.print("; YAW: ");
  Serial.print(m_setpoint[YAW]);
  Serial.println(" ");


}

void JoyPadManager::resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  m_data.j1PotX = 127;
  m_data.j1PotY = 127;
  m_data.j2PotX = 127;
  m_data.j2PotY = 127;
  m_data.j1Button = 1;
  m_data.j2Button = 1;
  m_data.pot1 = 1;
  m_data.pot2 = 1;
  m_data.tSwitch1 = 1;
  m_data.tSwitch2 = 1;
  m_data.button1 = 1;
  m_data.button2 = 1;
  m_data.button3 = 1;
  m_data.button4 = 1;
}