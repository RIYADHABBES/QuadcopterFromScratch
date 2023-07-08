///////////////////////////////////////////////////////////////////////////////////////////////
//        
//                RECEPTION TEST PROGRAM OF ORDERS FROM THE RADIO CONTROL
//
///////////////////////////////////////////////////////////////////////////////////////////////

// Copyright (C) 2021 Olivier CHAU-HUU
// 
// This source program is the exclusive property of the author. It is distributed 
// only for personal use. It can in no case be copied or transformed in order to
// derive any commercial benefit.
//

//!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!      WARNING      !!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// This program is delivered "as is" and without any warranty of any kind.
//
// It is given as an example to learn how to code for the Arduino card.
//
// The author can in no way be held responsible for any injury or damage
// caused by the misuse of this program.
//
// It is up to the user to take all necessary safety precautions.
//
//
//
// MyDroneZone by Olivier CHAU-HUU :  website   --> https://sites.google.com/view/mydronezone
//                                    channel   --> https://www.youtube.com/channel/UCQMzPz1T8_Q2FvroI6nBwRg
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  USAGE :
//
//  This program allows you to test that the Arduino board correctly receives commands from the radio control.
//  It is possible to detect a wiring error, a channel inversion, a value shift, etc... 
//
//  We remind you that, in our case, the radio control is in MODE 2.
//
//  In order to carry out the tests, the following operations must be performed :
//
//    - connect the channels from the receiver to the Arduino   CH1 --> PIN 8 Roll command
//                                                              CH2 --> PIN 9 Pitch command
//                                                              CH3 --> PIN 10 Gaz command
//                                                              CH4 --> PIN 11 Yaw command
//
//    - connect the power supply terminals of the receiver ("GND" and "5V" of Arduino)
// 
//    - Connect the Arduino to the computer with the USB cable, open the Arduino IDE and load this program
//      into the Arduino
//
//    - open the serial monitor, check that the flow rate is set to 9600 bauds
//
//    - switch on the radio control and move the joysticks
//
//    You can then see in the serial monitor the values of the pulse durations received on each channel.
//
//    In order are displayed : Roll, Pitch, Gas, Yaw.
//
//    The symbol "---" indicates that the values are between 1480 us and 1520 us.
//
//    For Roll and Yaw :  
//                        -  the symbol "<<<" indicates that the values are less than 1480 us
//                        -  the symbol ">>>" indicates that the values are higher than 1520 us.
//
//    For Pitch and Gas :  
//                        -  the symbol "^^^" indicates that the values are higher than 1520 us
//                        -  the symbol "vvv"  indicates that the values are less than 1480 us.
//    
//  If everything is correct, depending on the position of the joysticks the signals should have values
//  close to those shown below :
//  
//  Gas joystick to maximum (to the top)    : 2000 us
//  Gas joystick at median (in the middle)  : 1500 us
//  Gaz joystick to minimum (to the bottom) : 1000 us
//
//  Pitch joystick to maximum (to the top)    : 2000 us
//  Pitch joystick at median (in the middle)  : 1500 us
//  Pitch joystick to minimum (to the bottom) : 1000 us
//
//  Roll joystick to maximum (to the right)  : 2000 us
//  Roll joystick at median (in the middle)  : 1500 us
//  Roll joystick to minimum (to the left)   : 1000 us
//
//  Yaw joystick to maximum (to the right)   : 2000 us
//  Yaw joystick at median (in the middle)   : 1500 us
//  Yaw joystick to minimum (to the left)    : 1000 us
//
//  It is important to set the midpoints correctly so that their pulse durations are as close as possible
//  to the 1500 us value. The radio controls allow these settings to be made by means of "trim" adjustment
//  buttons. Please refer to the specific instructions of the radio control to find out how to adjust the
//  "trim" settings. 
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////
////// Declaration of the "define //////
////////////////////////////////////////

// the different "#define" to improve the readability of the code

#define CHANNEL1 0    // CHANNEL1 equals 0 (ROLL control)
#define CHANNEL2 1    // CHANNEL2 equals 1 (PITCH control)
#define CHANNEL3 2    // CHANNEL3 equals 2 (GAS control)
#define CHANNEL4 3    // CHANNEL4 equals 3 (YAW control)

/////////////////////////////////////////////
////// Déclaration of global variables //////
/////////////////////////////////////////////

//---------------------------------------------------------------------------------------------------
// variables used to calculate the duration of the pulses delivered on the channels of the receiver
//---------------------------------------------------------------------------------------------------

volatile unsigned int pulse_duration[4]; // pulse durations on the receiver channels

volatile unsigned long current_time; // current time storage variable used in ISR
                                     // to calculate receiver pulse durations
                                        
volatile unsigned long pulse_start_t[4]; // table of pulse start times of the receiver channels

volatile byte ch_status_storage[4]; // storage of the states of the reception channels

////////////////////////////////////////////////////////////////////////////////
////// Declaration of the prototypes of the functions used in the program //////
////////////////////////////////////////////////////////////////////////////////

void configureInterrupts(); // function to configure the pins that trigger an interrupt

void displayRCSignals();  // function that displays on the serial line the durations of
                          // the pulses received via the RC receiver.

//####################################################
//####                  SETUP()                   ####
//####################################################
void setup()
{

  // configuration of the pins used as interrupt sources
  configureInterrupts();
  
  // opening of the serial line with a baud rate of 9600 bauds
  Serial.begin(9600); 
}

//####################################################
//####                    LOOP()                  ####
//####################################################
void loop()
{
  delay(250);
  displayRCSignals();
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
ISR(PCINT0_vect) 
{
   current_time = micros();
   
   //########################
   // treatment of channel 1
   //########################
   if (PINB & B00000001) // if pin 8 is "HIGH"
   {                                       
       // we test if the memorized state of pin 8 was "LOW"
       if (ch_status_storage[CHANNEL1] == LOW) 
       {                     

          // then pin 8 made a rising edge

          // the memory variable of pin 8 state is updated

          ch_status_storage[CHANNEL1] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL1] = current_time;                        
       }
   } 

   // otherwise if pin 8 is at "LOW" and the storage of the state
   // of pin 8 is at "HIGH"

   else if (ch_status_storage[CHANNEL1] == HIGH) 
  
   {   // pin 8 went from "1" to "0" (falling edge)
       // the memory variable of pin 8 state is updated

       ch_status_storage[CHANNEL1] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL1] = current_time-pulse_start_t[CHANNEL1];   
   }

   //########################
   // treatment of channel 2
   //########################
   if (PINB & B00000010) // if pin 9 is "HIGH"
   {                                       
       // we test if the memorized state of pin 9 was "LOW"
       if (ch_status_storage[CHANNEL2] == LOW) 
       {                     

          // then pin 9 made a rising edge

          // the memory variable of pin 9 state is updated

          ch_status_storage[CHANNEL2] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL2] = current_time;                        
       }
   } 

   // otherwise if pin 9 is at "LOW" and the storage of the state
   // of pin 9 is at "HIGH"

   else if (ch_status_storage[CHANNEL2] == HIGH) 
  
   {   // pin 9 went from "1" to "0" (falling edge)
       // the memory variable of pin 9 state is updated

       ch_status_storage[CHANNEL2] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL2] = current_time-pulse_start_t[CHANNEL2];   
   }

   //########################
   // treatment of channel 3
   //########################
   if (PINB & B00000100) // if pin 10 is "HIGH"
   {                                       
       // we test if the memorized state of pin 10 was "LOW"
       if (ch_status_storage[CHANNEL3] == LOW) 
       {                     

          // // then pin 10 made a rising edge

          // the memory variable of pin 10 state is updated

          ch_status_storage[CHANNEL3] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL3] = current_time;                        
       }
   } 

   // otherwise if pin 10 is at "LOW" and the storage of the state
   // of pin 10 is at "HIGH"

   else if (ch_status_storage[CHANNEL3] == HIGH) 
  
   {   // pin 10 went from "1" to "0" (falling edge)
       // the memory variable of pin 10 state is updated

       ch_status_storage[CHANNEL3] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL3] = current_time-pulse_start_t[CHANNEL3];   
   }

   //########################
   // treatment of channel 4
   //########################
   if (PINB & B00001000) //  if pin 11 is "HIGH"
   {                                       
       // // we test if the memorized state of pin 11 was "LOW" 
       if (ch_status_storage[CHANNEL4] == LOW) 
       {                     

          // then pin 11 made a rising edge

          // the memory variable of pin 11 state is updated

          ch_status_storage[CHANNEL4] = HIGH;
  
          // we memorize the moment of this rising edge

          pulse_start_t[CHANNEL4] = current_time;                        
       }
   } 

   // otherwise if pin 11 is at "LOW" and the storage of the state
   // of pin 11 is at "HIGH"

   else if (ch_status_storage[CHANNEL4] == HIGH) 
  
   {   // pin 11 went from "1" to "0" (falling edge)
       // the memory variable of pin 11 state is updated

       ch_status_storage[CHANNEL4] = LOW; 


       // PWM pulse duration is calculated

       pulse_duration[CHANNEL4] = current_time-pulse_start_t[CHANNEL4];   
   }
   
}

//#####################################################
//  Display function of the pulse durations received
//  via the radio receiver
//
//  "displayRCSignals()"
//
//  - no input parameter
//
//  - no value returned
//
//#####################################################
void displayRCSignals()
{
  Serial.print("Roll :");
  if(pulse_duration[CHANNEL1] < 1480)Serial.print(" <<< ");
  else if(pulse_duration[CHANNEL1] > 1520)Serial.print(" >>> ");
  else Serial.print(" --- ");
  Serial.print(pulse_duration[CHANNEL1]);
  
  Serial.print("  Pitch :");
  if(pulse_duration[CHANNEL2] < 1480)Serial.print(" vvv ");
  else if(pulse_duration[CHANNEL2] > 1520)Serial.print(" ^^^ ");
  else Serial.print(" --- ");
  Serial.print(pulse_duration[CHANNEL2]);
  
  Serial.print("  Gas :");
  if(pulse_duration[CHANNEL3] < 1480)Serial.print(" vvv ");
  else if(pulse_duration[CHANNEL3] > 1520)Serial.print(" ^^^ ");
  else Serial.print(" --- ");
  Serial.print(pulse_duration[CHANNEL3]);
  
  Serial.print("  Yaw :");
  if(pulse_duration[CHANNEL4] < 1480)Serial.print(" <<< ");
  else if(pulse_duration[CHANNEL4] > 1520)Serial.print(" >>> ");
  else Serial.print(" --- ");
  Serial.println(pulse_duration[CHANNEL4]);
}
