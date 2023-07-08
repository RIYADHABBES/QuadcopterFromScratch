//################################################################################################################################
//##																															                                                              ##
//##						                           ESC CONFIGURATION PROGRAM for ESC type HW30A 																        ##
//##                                                                                                                            ##
//################################################################################################################################

//
// Copyright (C) 2021 Olivier CHAU-HUU
// 
// This source program is the exclusive property of the author. It is distributed 
// only for personal use. It can in no case be copied or transformed in order to
// derive any commercial benefit.
//

//################################################################################################################################
//##                                                                                                                            ##                                                                                                                          
//##  !!!!!!!!!!!!!!!!!!!                                                                                                       ##  
//##  !!!!  CAUTION  !!!!                                                                                                       ##
//##  !!!!!!!!!!!!!!!!!!!                                                                                                       ##
//##                                                                                                                            ##  
//##  This program is delivered "as is" and without any warranty of any kind.                                                   ##
//##  It is given as an example to learn how to code for the Arduino card.                                                      ##
//##                                                                                                                            ##   
//##  A brushless motor equipped with a propeller is an element that can be very                                                ## 
//##  dangerous.                                                                                                                ##
//##                                                                                                                            ##
//##  The author can in no way be held responsible for any injury or damage                                                     ##
//##  caused by the misuse of this program.                                                                                     ##
//##                                                                                                                            ##
//##  It is up to the user to take all necessary safety precautions.                                                            ##
//##                                                                                                                            ##
//##                                                                                                                            ##
//##  MyDroneZone by Olivier CHAU-HUU :  website   --> https://sites.google.com/view/mydronezone                                ##
//##                                     channel   --> https://www.youtube.com/channel/UCQMzPz1T8_Q2FvroI6nBwRg                 ##
//################################################################################################################################
//##                                                                                                                            ##
//##	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                                                           ##
//##	!!!!     DANGER : DO NOT PUT A PROPELLER ON THE MOTOR      !!!!                                                           ##
//##	!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!                                                           ##
//##                                                                                                                            ##
//##                                                                                                                            ##  
//##	  Usage :																														                                                      ##
//##																															                                                              ##
//##  	1. Connect the Arduino UNO card to the computer with the USB cable, launch the Arduino IDE, then upload this program.   ## 
//##       This is then executed by starting to send a maximum speed setpoint to the ESC (PWM pulses of maximum duration        ##
//##       corresponds to 2ms)		 							                                                                                ##
//##																															                                                              ##
//##    2. Open the Arduino IDE serial monitor                                      																			      ##
//##																															                                                              ##
//##    3. Connect the ESC to the Arduino (ESC control on pin 4) then connect the ESC to the battery : the ESC emits a 3 beeps  ##
//##       rising melody, then 2 beeps of the same tone. After 5s, it enters programming mode and emits a 5 notes rising        ##
//##       melody. The ESC then starts to "list" the configuration items:                                                       ##
//##                                                                                                                            ##
//##            Menu 1    1 beep : Brake setting                                                                                ##
//##            Menu 2    2 beeps : Battery type setting                                                                        ##
//##            Menu 3    3 beeps : Cut-off mode setting                                                                        ##
//##            Menu 4    4 beeps : Cut-off Threshold setting                                                                   ##
//##            Menu 5    1 beeeep : Startup mode setting                                                                       ##
//##            Menu 6    1 beeeep 1 beep : Timing setting                                                                      ##
//##            Menu 7    1 beeeep 2 beep : Factory defaults setting                                                            ##
//##            Menu 8    2 beeeeps : exit programming mode                                                                     ##
//##                                                                                                                            ##
//##      The detail of the parameters of each menu is given in the section "ESC : calibration and configuration".              ##
//##                                                                                                                            ##
//##			To enter a menu, just after hearing his musical signature type in the serial monitor the letter "l"                   ##
//##       (lowercase "L") followed by "enter".                                                                                 ##
//##                                                                                                                            ##
//##      Inside the menu to validate a parameter, just after hearing its musical signature type in the serial monitor          ##
//##      the letter "h" followed by "enter". The ESC confirms that the parameter has been taken into account by emitting       ##
//##      2 beeps of descending tones twice. The ESC then goes to the next menu.                                                ##
//##                                                                                                                            ##
//##      To exit the configuration mode, immediately after hearing the validation melody of a parameter, type in the serial    ##
//##      monitor the letter "l" followed by "enter".                                                                           ##
//##                                                                                                                            ##
//##      You can also exit the configuration mode by going to menu N°8 which corresponds to "exit".                            ##                                       
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
    Serial.begin(9600);  // initialization of the serial communication with the computer at 9600 bauds

    motor.attach(4, 1000, 2000); // ESC control on pin 4, min and max PWM durations : 1000us and 2000us

    motor.write(180); // send GAS MAX command (180 corresponds to pulses of 2ms duration)

    displayInstructions();  // function that displays user instructions in the serial monitor 
}

//####################################################
//####                    LOOP()                  ####
//####################################################

void loop() // main program loop
{
    if (Serial.available()) 
    {
        data = Serial.read();

        switch (data) {
           
            // 'l' for "low"
            case 108 : Serial.println("Sending MINI GAS command");
                      motor.write(0);
            break;

            // 'h' for "high"
            case 104 : Serial.println("Sending MAX GAS command");
                      motor.write(180);
                      
            break;

            // 'm'for "medium"
            case 109 : Serial.println("Sending MEDIUM GAS command");
                      motor.write(90);
                      
            break;

            // 't' for "test"
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
    Serial.println("READY FOR CONFIGURATION\n");
    Serial.println("Please read all of the following instructions before proceeding. :\n");
    Serial.println("Connect the ESC to the battery. The ESC emits a melody of 3 rising beeps,");
    Serial.println("then 2 beeps of the same tone.");
    Serial.println("After 5 seconds, the ESC emits a melody of 5 rising beeps: this is the entry");
    Serial.println("in programming mode.");
    Serial.println("The ESC reviews the menus acoustically : 1 beep, 2 beeps etc...");
    Serial.println("To enter a menu, just after hearing its sound signature");
    Serial.println("type in the serial monitor the letter \"l\" followed by \"enter\".");
    Serial.println("Once in the menu, the options are listed acoustically : 1 beep");
    Serial.println("2 beeps etc... To validate an option, just after hearing its sound signature");
    Serial.println(" type the letter \"h\" then \"enter\".");
    Serial.println("To confirm, the ESC emits 2 beeps of descending tones twice and then returns");
    Serial.println("to the item listing. You can then enter another menu item,or exit the ");
    Serial.println("programming mode by typing the letter \"l\" followed by \"enter\".");
}
