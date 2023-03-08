// ============================================================================
// ===                        MOTOR FUNCTIONS                               ===
// ============================================================================

void motorInit(){
  /*
  -------------------------MOTOR INITIALIZATION--------------------------------
  
  Delays program until arming sequence is completed.
  
  To arm the motors: 
    1. Lower throttle to min
    2. Raise throttle to max (motors will not spin)
    3. Lower throttle to min again. ESCs will beep to indicate arming.
  
  */
  
  byte channelPins[] = {OUTPUT_PIN_M0, OUTPUT_PIN_M1, OUTPUT_PIN_M2, OUTPUT_PIN_M3};
  
  // pause program until motors armed.
  Serial.print("Motors safe. "); 
  
  while(rxValShared[CHANNEL_THROTTLE] > THROTTLE_CUTOFF){ }
  Serial.println("Throttle down. ");

  while(rxValShared[CHANNEL_THROTTLE] < THROTTLE_RMAX-50){ }
  Serial.print("Throttle up. ");

  // once throttle is raised, LED blue
  setLED(BLUE);
  // initialize and calibrate gyro 
  gyroInit();
  // reset LED to red when done calibrating
  setLED(RED);

  while(rxValShared[CHANNEL_THROTTLE] > THROTTLE_CUTOFF){ }
  Serial.println("Throttle down. MOTORS ARMED!");
  setLED(GREEN);  // motors armed, LED green
  motorsArmed = true;  // set arming flag

  #if defined(USE_MOTOR_PWM)
    // set each motor pin to output for PWM
    for(byte i=0; i<4; i++){
      pinMode(channelPins[i], OUTPUT);
      motorVal[i] = MOTOR_ARM_START - 50;  // also set the motor setpoint below arm
    }
    
    // Initialize PWM on each pin
    TCCR1A |= _BV(COM1A1);  // attach pin  9 to timer 1 channel A, write with OCR1A
    TCCR1A |= _BV(COM1B1);  // attach pin 10 to timer 1 channel B, write with OCR1B
    TCCR2A |= _BV(COM2A1);  // attach pin 11 to timer 2 channel A, write with OCR2A
    TCCR2A |= _BV(COM2B1);  // attach pin  3 to timer 2 channel B, write with OCR2B
    
    setMotors();  // set motors to below arm
    
  #elif defined(USE_SERVO_LIB)
    // attach servo objects to pins for each output channel and turn to zero throttle.
    for (int i=0; i<4; i++){
      motors[i].attach(channelPins[i]);
      motors[i].writeMicroseconds(MOTOR_ARM_START-50);
    }
  #endif
  
  // delay a half second to allow ESCs to arm
  delay(500);
}


// ============================================================================
// ===                          MOTOR SETTING                               ===
// ============================================================================


#if defined(USE_MOTOR_PWM)
void setMotors(){
  // motor control using hardware PWM at 490Hz
  // must divide motor setpoint by 8 to get correct pulse width in 8 bits
  // this is done using bitshift to be faster
  
  // directly accesses hardware PWM comparater registers instead of using 
  // analogWrite(), slightly faster
  OCR1A = motorVal[0]>>3;
  OCR1B = motorVal[1]>>3;
  OCR2A = motorVal[2]>>3;
  OCR2B = motorVal[3]>>3;
}

#elif defined(USE_SERVO_LIB)
void setMotors(){
  // Motor control using Servo library at 50 Hz
  for(byte i=0; i<4; i++){
    motors[i].writeMicroseconds(motorVal[i]);
  }
}
#endif


// ============================================================================
// ===                         MOTOR ARM/DISARM                             ===
// ============================================================================

void disarmMotors(){
  // function to disarm the motors at any time during the program
  for(byte i=0; i<4; i++){
    motorVal[i] = 0;
  }
  setMotors();
  
  setLED(RED); // set LED to red
}

void rearmMotors(){
  // function to rearm the motors after being disarmed mid-program
  for(byte i=0; i<4; i++){
    motorVal[i] = MOTOR_ARM_START - 50;
  }
  setMotors();
  
  setLED(GREEN); // set LED to green
}

