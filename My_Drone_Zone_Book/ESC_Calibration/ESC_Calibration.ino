//#####################################################################################
//##                                                                                 ##
//##                    ESC CALIBRATION PROGRAM for ESC type HW30A                   ##
//##                                                                                 ##
//#####################################################################################

//
// Copyright (C) 2021 Olivier CHAU-HUU
// 
// This source program is the exclusive property of the author. It is distributed 
// only for personal use. It can in no case be copied or transformed in order to
// derive any commercial benefit.
//

//!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!     WARNING     !!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!
//
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// This program is delivered "as is" and without any warranty of any kind.
//
// It is given as an example to learn how to code for the Arduino card.
// 
// A brushless motor equipped with a propeller is an element that can be very
// dangerous.
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
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//!!!!     DANGER : DO NOT PUT A PROPELLER ON THE MOTOR     !!!!
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//################################################################################################################################
//##																															                                                              ##
//##						                                           USAGE                                                                ##
//##																															                                                              ##
//##	  The Arduino must be connected to the ESC beforehand :   - pin 4 on ESC white control wire                               ##
//##                                                            - GND on black ESC control wire                                 ##
//##                                                            - WARNING: red ESC control wire not connected                   ##
//##																															                                                              ##
//##    The ESC is connected to the brushless motor (DO NOT FIX HELICE) but not yet to the battery.                             ##
//##                                                                                                                            ##
//##                                                                                                                            ##
//##  	1. Connect the Arduino to the computer with the USB cable, launch the Arduino IDE, open and upload the program.         ##
//##       The program starts sending PWM pulses of maximum duration to the ESC (2ms).	 							                          ##
//##																															                                                              ##
//##    2. Open the Arduino IDE serial monitor, set the baud rate to 9600 baud. A text appears in the monitor to remind         ##
//##       you how to proceed. 													                                                                        ##
//##                                                                                                                            ##
//##    3. Connect the ESC to the battery : you will then hear a melody of 3 rising "beeps" followed by 2 identical "beeps".    ## 
//##       The 2 "beeps" confirm that the maximum speed control point has been memorized.                                       ##
//##																															                                                              ##
//##    4. Within 2 seconds after the 2 "beeps", type in the serial monitor the letter "v" followed by "enter". This sends      ##
//##       a minimum speed command (PWM pulses of 1ms) to the ESC. The ESC then emits 3 short "beeps" followed by a long        ##
//##       "beep" indicating confirmation that the minimum speed command has been acknowledged. Calibration is now complete.    ##                                                                                   ##
//##                                                                                                                            ##
//##   	5. Press "t" to perform a test with an increasing/decreasing speed control : the motor starts to run up to its          ##
//##       maximum speed and then slows down to a stop.															                                            ##
//## 																															                                                              ##
//##    Note : If step "4" is not performed quickly enough then the ESC enters the configuration mode.                          ##
//##                                                                                                                            ##
//##    In this case, in order to be able to perform the calibration sequence it is necessary to :                              ##
//## 																															                                                              ##
//##  	- disconnect the ESC from the battery																				                                            ##
//##    - close the serial monitor                                                                                              ##
//## 	  - Reset the Arduino (press the push button on the card provided for this purpose or disconnect/reconnect its USB link)  ##  					                                                                    ##
//##    - reopen the serial monitor                                                                                             ##
//## 	  - reconnect the ESC to the battery.  																					                                          ##
//##	  - Resume calibration in step "4.".																				                                              ##
//## 																															                                                              ##
//##    After calibration, disconnect the ESC from the battery, disconnect the Arduino from the PC, disconnect the Arduino/ESC  ##         
//##    link, close the serial monitor and the Arduino IDE.                                                                     ##
//##                                                                                                                            ##
//################################################################################################################################


// inclusion of the library to generate PWM signals, it is called "Servo.h" because it is this
// type of signals that are used to control servomotors.
#include <Servo.h>

Servo motor;	// we declare a 'variable' motor of type "Servo" (it is an object instance)
char data;		// variable of type "char" which will make it possible to read a character typed on the keyboard


//####################################################
//####                  SETUP()                   ####
//####################################################

void setup() // initialization routine
{
    Serial.begin(9600);	// initialization of the serial communication with the computer at 9600 bauds

    motor.attach(4, 1000, 2000); // ESC control on pin 4, min and max PWM durations : 1000us and 2000us

    motor.write(180); // send GAS MAX command (180 corresponds to pulses of 2ms duration)

    displayInstructions();	// function that displays user instructions in the serial monitor 
}


//####################################################
//####                    LOOP()                  ####
//####################################################

void loop() // main program loop
{
    if (Serial.available()) 
    {
        // reading of characters typed in the serial monitor
        data = Serial.read();

        // execution of different actions according to the character received
        switch (data) 
        {
            // v' for "validation" : sending a maximum command
            case 118 : Serial.println("Sending MINI GAS command");
                       motor.write(0);            
                     
            break;

            // m' for "maximum" : sending a minimum command
            case 109 : Serial.println("Sending MAX GAS command");
                       motor.write(180);            
                     
            break;

            // t' for "test" : send a variable increasing/decreasing command
            case 116 : Serial.print("Motor test start in 3s");
                      delay(1000);
                      Serial.print(" 2s");
                      delay(1000);
                      Serial.println(" 1s ...");
                      delay(1000);
                      ESC_test();
            break;
        }
    }
}


//#####################################################
//
// ESC test function : ESC setpoint varying from min.
// to max. and then from max. to min.
//
// "ESC_test()"
//
//  - no input parameter
//
//  - no value returned
//
//#####################################################
void ESC_test()
{
    for (int i=0; i<=180; i++) 
    {
        Serial.print("Speed = ");
        Serial.println(i);
        motor.write(i);        
        delay(10);
    }
    
    for (int i=180; i>=0; i--) 
    {
        Serial.print("Speed = ");
        Serial.println(i);
        motor.write(i);        
        delay(10);
    }

    Serial.println("STOP");
    motor.write(0);
}

//#####################################################
//
// Display fonction of instructions for the user
//
// "displayInstructions()"
//
//  - no input parameter
//
//  - no value returned
//
//#####################################################
void displayInstructions()
{
    Serial.println("READY FOR CALIBRATION\n");
    Serial.println("Please read the following instructions in their entirety before you begin :\n");
    Serial.println("Connect the ESC to the battery.");
    Serial.println("The ESC emits a melody of 3 rising beeps, then 2 beeps of the same tone.");
    Serial.println("Within 2 seconds, you must validate by typing the letter \"v\" followed by");
    Serial.println("\"enter\" in the serial monitor.");
    Serial.println("The ESC then emits 3 identical short beeps followed by a long beep to signal");
    Serial.println("that the calibration is done.");
    Serial.println("Start a test in variable speed by typing in the serial monitor the letter");
    Serial.println("\"t\" then \"enter\".\n");    
}
