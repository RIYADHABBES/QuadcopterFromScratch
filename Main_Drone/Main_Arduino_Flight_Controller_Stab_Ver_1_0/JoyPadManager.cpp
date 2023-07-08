#include "JoyPadManager.h"

#include <Arduino.h>

#define ROLL  0     // ROLL equals 0
#define PITCH 1     // PITCH equals 1
#define YAW   2     // YAW equals 2
#define GAS   3     // GAS equals 3


#define CHANNEL1 0    // CHANNEL1 equals 0 (ROLL control)
#define CHANNEL2 1    // CHANNEL2 equals 1 (PITCH control)
#define CHANNEL3 2    // CHANNEL3 equals 2 (GAS control)
#define CHANNEL4 3    // CHANNEL4 equals 3 (YAW control)

#define LED 13    // LED equals 13 Built-in Nano board LED

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
		
        while(m_pulse_duration[CHANNEL3] <990 || m_pulse_duration[CHANNEL3] > 1020 || m_pulse_duration[CHANNEL4] < 1450 || m_pulse_duration[CHANNEL4] > 1550 )   // boucle tant que la commande des GAS est supérieure au minimum      
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
void JoyPadManager::calculateSetpoints()
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
    
    if(m_pulse_duration[CHANNEL1] > 1508)
    {
          m_setpoint[ROLL] = 0.244*m_pulse_duration[CHANNEL1] - 367.80;
    }
    else if(m_pulse_duration[CHANNEL1] < 1492)
    {
          m_setpoint[ROLL] = 0.244*m_pulse_duration[CHANNEL1] - 363,90;
    }

    //########################
    //#### PITCH setpoint ####
    //########################

    m_setpoint[PITCH] = 0;

    // the PITCH command has a negative slope because a PITCH command lower than 1492us must raise the nose of the UAV
    if(m_pulse_duration[CHANNEL2] > 1508)
    {	 
          m_setpoint[PITCH] = -0.244*m_pulse_duration[CHANNEL2] + 367.80;
    }
    else if(m_pulse_duration[CHANNEL2] < 1492)
    {
		  m_setpoint[PITCH] = -0.244*m_pulse_duration[CHANNEL2] + 363.90;
    }

    //######################
    //#### YAW setpoint ####
    //######################    

    m_setpoint[YAW] = 0;

    if(m_pulse_duration[CHANNEL3] > 1050)    // the YAW setpoint is only calculated if the GAS control is non-zero, otherwise
    {                                     // when you want to switch to the "STOP" state, the motors accelerate 
      

        if(m_pulse_duration[CHANNEL4] > 1508)
        {
              m_setpoint[YAW] = 0.244*m_pulse_duration[CHANNEL4] - 367.80;
        }
        else if(m_pulse_duration[CHANNEL4] < 1492)
        {
              m_setpoint[YAW] = 0.244*m_pulse_duration[CHANNEL4] - 363,90;
        }

    }

    //#######################
    //#### Stabilization ####
    //#######################
  
    // correction of Roll and Pitch angular speed setpoints to ensure Self-Level

  // In Both setpoint[ROLL] -= MPU6050Manager::roll_adjustment;
   // In Both setpoint[PITCH] -= MPU6050Manager::pitch_adjustment;
    

    //######################
    //#### GAS setpoint ####
    //######################

    m_setpoint[GAS] = m_pulse_duration[CHANNEL3];  // the GAS set point is the copy of Channel 3

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
void JoyPadManager::I_R(/*PCINT0_vect*/) 
{
   m_current_time = micros();
   
   //########################
   // treatment of channel 1
   //########################
   if (PINB & B00000001) // if pin 8 is "HIGH"
   {                                       
       // we test if the memorized state of pin 8 was "LOW"
       if (m_ch_status_storage[CHANNEL1] == LOW) 
       {                     

          // then pin 8 made a rising edge

          // the memory variable of pin 8 state is updated

          m_ch_status_storage[CHANNEL1] = HIGH;
  
          // we memorize the moment of this rising edge

          m_pulse_start_t[CHANNEL1] = m_current_time;                        
       }
   } 

   // otherwise if pin 8 is at "LOW" and the storage of the state
   // of pin 8 is at "HIGH"

   else if (m_ch_status_storage[CHANNEL1] == HIGH) 
  
   {   // pin 8 went from "1" to "0" (falling edge)
       // the memory variable of pin 8 state is updated

       m_ch_status_storage[CHANNEL1] = LOW; 


       // PWM pulse duration is calculated

       m_pulse_duration[CHANNEL1] = m_current_time-m_pulse_start_t[CHANNEL1];   
   }

   //########################
   // treatment of channel 2
   //########################
   if (PINB & B00000010) // if pin 9 is "HIGH"
   {                                       
       // we test if the memorized state of pin 9 was "LOW"
       if (m_ch_status_storage[CHANNEL2] == LOW) 
       {                     

          // then pin 9 made a rising edge

          // the memory variable of pin 9 state is updated

          m_ch_status_storage[CHANNEL2] = HIGH;
  
          // we memorize the moment of this rising edge

          m_pulse_start_t[CHANNEL2] = m_current_time;                        
       }
   } 

   // otherwise if pin 9 is at "LOW" and the storage of the state
   // of pin 9 is at "HIGH"

   else if (m_ch_status_storage[CHANNEL2] == HIGH) 
  
   {   // pin 9 went from "1" to "0" (falling edge)
       // the memory variable of pin 9 state is updated

       m_ch_status_storage[CHANNEL2] = LOW; 


       // PWM pulse duration is calculated

       m_pulse_duration[CHANNEL2] = m_current_time-m_pulse_start_t[CHANNEL2];   
   }

   //########################
   // treatment of channel 3
   //########################
   if (PINB & B00000100) // if pin 10 is "HIGH"
   {                                       
       // we test if the memorized state of pin 10 was "LOW"
       if (m_ch_status_storage[CHANNEL3] == LOW) 
       {                     

          // // then pin 10 made a rising edge

          // the memory variable of pin 10 state is updated

          m_ch_status_storage[CHANNEL3] = HIGH;
  
          // we memorize the moment of this rising edge

          m_pulse_start_t[CHANNEL3] = m_current_time;                        
       }
   } 

   // otherwise if pin 10 is at "LOW" and the storage of the state
   // of pin 10 is at "HIGH"

   else if (m_ch_status_storage[CHANNEL3] == HIGH) 
  
   {   // pin 10 went from "1" to "0" (falling edge)
       // the memory variable of pin 10 state is updated

       m_ch_status_storage[CHANNEL3] = LOW; 


       // PWM pulse duration is calculated

       m_pulse_duration[CHANNEL3] = m_current_time-m_pulse_start_t[CHANNEL3];   
   }

   //########################
   // treatment of channel 4
   //########################
   if (PINB & B00001000) //  if pin 11 is "HIGH"
   {                                       
       // // we test if the memorized state of pin 11 was "LOW" 
       if (m_ch_status_storage[CHANNEL4] == LOW) 
       {                     

          // then pin 11 made a rising edge

          // the memory variable of pin 11 state is updated

          m_ch_status_storage[CHANNEL4] = HIGH;
  
          // we memorize the moment of this rising edge

          m_pulse_start_t[CHANNEL4] = m_current_time;                        
       }
   } 

   // otherwise if pin 11 is at "LOW" and the storage of the state
   // of pin 11 is at "HIGH"

   else if (m_ch_status_storage[CHANNEL4] == HIGH) 
  
   {   // pin 11 went from "1" to "0" (falling edge)
       // the memory variable of pin 11 state is updated

       m_ch_status_storage[CHANNEL4] = LOW; 


       // PWM pulse duration is calculated

       m_pulse_duration[CHANNEL4] = m_current_time-m_pulse_start_t[CHANNEL4];   
   }
   
}
