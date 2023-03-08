
// ============================================================================
// ===                          PID CONTROL                                 ===
// ============================================================================

float computeRollPID(float setPoint){
  /*
  ---------------------------Roll PID Function---------------------------------
  
  General-purpose PID controller. 
  */
  
  static unsigned long lastTime;
  static float lastErr, errSum;
  float error;
  static float P, I, D;
  
  unsigned long now = millis();
  int dt = (int)(now - lastTime);
  
  if(dt >= PID_SMPL_TIME){
    error   = setPoint - roll;
    errSum += error;
    
    P = ROLL_PID_KP * error;
    
    // when elapsed time is greater than 10 times the PID sample rate:
    // omit the d term, which could cause a large output spike
    // also reset the errSum value to zero the I term
    // this prevents strange behavior when throttle is lowered below 
    // the cutoff threshold and then raise again
    if(dt > 10*PID_SMPL_TIME) { 
      D = 0; 
      errSum = 0;
    }
    else{
      D = ROLL_PID_KD * (error - lastErr);
    }
    
    I = ROLL_PID_KI * errSum;
    // constrain I term to max setting
    I = constrain(I, -I_MAX, I_MAX);    
    
    #ifdef DEBUG_PID_ROLL
      Serial.print(P); Serial.print('\t');
      Serial.print(I); Serial.print('\t');
      Serial.println(D);
    #endif
    
    lastErr = error;
    lastTime = now;
  }
  return (P + I + D);
}


float computePitchPID(float setPoint){
  /*
  --------------------------Pitch PID Function---------------------------------
  
  General-purpose PID controller. 
  */
  
  static unsigned long lastTime;
  static float lastErr, errSum;
  float error;
  static float P, I, D;
  
  unsigned long now = millis();
  int dt = (int)(now - lastTime);
  
  if(dt >= PID_SMPL_TIME){
    error   = setPoint - pitch;
    errSum += error;
    
    P = PITCH_PID_KP * error;
    
    // when elapsed time is greater than 10 times the PID sample rate:
    // omit the d term, which could cause a large output spike
    // also reset the errSum value to zero the I term
    // this prevents strange behavior when throttle is lowered below 
    // the cutoff threshold and then raise again
    if(dt > 10*PID_SMPL_TIME){ 
      D = 0; 
      errSum = 0;
    }
    else{
      D = PITCH_PID_KD * (error - lastErr);
    }
    
    I = PITCH_PID_KI * errSum;
    // constrain I term to max setting
    I = constrain(I, -I_MAX, I_MAX);    
    
    #ifdef DEBUG_PID_PITCH
      Serial.print(P); Serial.print('\t');
      Serial.print(I); Serial.print('\t');
      Serial.println(D);
    #endif
    
    lastErr = error;
    lastTime = now;
  }
  return (P + I + D);
}


float computeYawPID(float setPoint){
  /*
  ----------------------------Yaw PID Function---------------------------------
  
  General-purpose PID controller.
  */
  
  static unsigned long lastTime;
  static float lastErr, errSum;
  float error;
  static float P, I, D;
  
  float now = micros();
  float dt = (float)(now - lastTime);
  
  if(dt >= PID_SMPL_TIME){
    
    error   = setPoint - gyroZ;
    errSum += error;
    
    P = YAW_PID_KP * error;
    
    if(dt>10*PID_SMPL_TIME){
      D = 0;
      errSum = 0;
    }
    else{
      D = YAW_PID_KD * (error - lastErr);
    }
    
    I = YAW_PID_KI * errSum;
    // constrain I term to max setting
    I = constrain(I, -I_MAX, I_MAX);
    
    #ifdef DEBUG_PID_YAW
      Serial.print(P); Serial.print('\t');
      Serial.print(I); Serial.print('\t');
      Serial.println(D);
    #endif    
    
    lastErr = error;
    lastTime = now;
  
  }
  
  return (P + I + D);
}
