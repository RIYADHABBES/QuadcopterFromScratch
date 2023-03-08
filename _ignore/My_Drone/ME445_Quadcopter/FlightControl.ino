/*
  Quad motor configuration

   ->     <- 
    M0   M1
      \ /
      / \
    M3   M2
   ->     <-

*/

// ============================================================================
// ===                          FLIGHT CONTROL                              ===
// ============================================================================

void flightControl() {
  /*
  -------------------------FLIGHT CONTROL SEQUENCE-----------------------------
  
  Calls functions to update Rx values, calculate motor control values, and 
  sends values to the motors.
  */
  
  // Check if any new inputs have been received
  if (updateFlagsShared){
    // get any new RC inputs
    rxGetNewVals();
  }
  
  #ifdef DEBUG_RX_RAW
  Serial.print(rxVal[CHANNEL_THROTTLE]); Serial.print('\t'); 
  Serial.print(rxVal[CHANNEL_ROLL]);     Serial.print('\t');
  Serial.print(rxVal[CHANNEL_PITCH]);    Serial.print('\t');
  Serial.println(rxVal[CHANNEL_YAW]);
  #endif


  // motor setpoints for each axis of motion
  int setRoll, setPitch, setYaw;
  
    // flag for whether a disarm command was recieved last loop
  static boolean disarmRequested = false;
  // time for when disarm command was recieved
  static unsigned long disarmRequestTime;
  // flags for re-arming motors
  static boolean throDown = false; 
  static boolean throUp   = false;
  
  if(motorsArmed && rxVal[CHANNEL_THROTTLE] > THROTTLE_CUTOFF){  // only spin motors if armed and throttle is above cutoff
    
    disarmRequested = false;  // reset disarm request flag
    
    #ifdef DEBUG_RX_ANGLES
      Serial.print(rxThro); Serial.print('\t'); 
      Serial.print(rxRoll); Serial.print('\t');
      Serial.print(rxPitch); Serial.print('\t');
      Serial.println(rxYaw);
    #endif
    
    #ifdef TEST_ROLL
      Serial.print(millis()); Serial.print('\t');
      Serial.print(rxRoll);   Serial.print('\t');
      Serial.println(roll);
    #endif
    
    #if defined(PID_CONTROL)
      // compute controller output values
      setRoll  = (int)computeRollPID((float)rxRoll);
      setPitch = (int)computePitchPID((float)rxPitch);
      setYaw   = (int)computeYawPID((float)rxYaw);
    
    #elif defined(RX_MIXING_ONLY)
      // Mixes Rx outputs only, no PID control
      setRoll  = rxRoll;
      setPitch = rxPitch;
      setYaw   = rxYaw;
    #endif
    
    motorVal[0] = rxThro + setRoll + setPitch + setYaw;
    motorVal[1] = rxThro - setRoll + setPitch - setYaw;
    motorVal[2] = rxThro - setRoll - setPitch + setYaw;
    motorVal[3] = rxThro + setRoll - setPitch - setYaw;
    
    #ifdef FORWARD_INDIV
      // TESTING ONLY. Forward throttle to single motor.
      for(byte i=0; i<4; i++){
        if(i == FORWARD_INDIV){ motorVal[i] = rxThro; }
        else{ motorVal[i] = MOTOR_ARM_START - 50; }
      }          
    #endif
    
  }
  
  else if(motorsArmed){  
    // if motors armed but throttle not above cutoff, don't spin any motors.
    motorVal[0] = MOTOR_ARM_START - 50;
    motorVal[1] = MOTOR_ARM_START - 50;
    motorVal[2] = MOTOR_ARM_START - 50;
    motorVal[3] = MOTOR_ARM_START - 50;
    
    /*  MOTOR DISARM CHECK
    
    Motors may be disarmed mid-program by holding throttle down and yaw full left for 1 second.
    
    When throttle down, check for full left yaw. If it has been held for 1 second, disarm the motors.
    */
    
    if(rxVal[CHANNEL_YAW] > 1850){  // if yaw stick is fully left while motors armed and throttle low
      if(disarmRequested){  // if a disarm request was received in the prev. loop
        if((millis() - disarmRequestTime) > 1000){  // if 1 second have elapsed since first disarm request
          disarmMotors();           // disarm the motors
          motorsArmed = false;
          disarmRequested = false;  // reset disarm request flag
        }
      }
      else{  // this is the first disarm request
        disarmRequested = true;  // set the flag
        disarmRequestTime = millis();  // save the time
      }
    }
    else{  // yaw stick is not in disarm position (fully left yaw, throttle down)
      disarmRequested = false;
    }
  }
  
  else{  // if motors not armed
    /* MOTOR RE-ARM CHECK
    
    Once disarmed mid-program, motors may be re-armed by raising lowering throttle to minimum, 
    then raising throttle to full, then returning throttle to minimum.
    
    */
    
    if(rxVal[CHANNEL_THROTTLE] < THROTTLE_CUTOFF && !throDown){  // check for min throttle first
      throDown = true;  // save flag to proceed
    }
    else if(rxVal[CHANNEL_THROTTLE] > THROTTLE_RMAX - 50 && throDown && !throUp){  // next check for max throttle
      throUp = true;    // save flag to proceed
    }
    else if(rxVal[CHANNEL_THROTTLE] < THROTTLE_CUTOFF && throUp){  // finally check for throttle back at min
      rearmMotors();  // re-arm the motors
      motorsArmed = true;
      
      // reset the motor re-arm flags
      throDown = false;
      throUp = false;
    }
     
  }
  
  // write values to motors
  setMotors();
}
