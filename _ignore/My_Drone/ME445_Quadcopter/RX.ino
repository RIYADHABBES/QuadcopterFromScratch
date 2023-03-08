// ============================================================================
// ===                          RX FUNCTIONS                                ===
// ============================================================================


void rxInit(){
  // initialize interface with Rx by attaching interrupts
  PCintPort::attachInterrupt(RX_PIN_CH0, chan0, CHANGE);
  PCintPort::attachInterrupt(RX_PIN_CH1, chan1, CHANGE);
  PCintPort::attachInterrupt(RX_PIN_CH2, chan2, CHANGE);
  PCintPort::attachInterrupt(RX_PIN_CH3, chan3, CHANGE);
}

void rxGetNewVals(){
  // --------------------Raw Rx Value Updater Function-------------------------
  // updates values in rxVal with any new RX signals that have been recieved
  
  noInterrupts();  // turn off interrupts while interacting with volatile vars
  
  byte updateFlags = updateFlagsShared;  // take local copy of update flags
   
  // copy the value of each channel into the buffer if it has been updated
  for (int i=0; i<4; i++){
    // use bitshift to check if the flag has been raised
    if (updateFlagsShared & 1<<i){
      // check if value is reasonable. 
      // Reject values greater than 2200 micros, these mean an interrupt was missed
      if(600 < rxValShared[i] && rxValShared[i] < 2200){
        rxVal[i] = rxValShared[i];
      }
    }
  }
    
  // now we have a copy of each changed variable. Reset the flag variable and turn interrupts on
  updateFlagsShared = 0;
  interrupts();
  
  rxMapping(updateFlags);
}

void rxMapping(byte updateFlags){
  // -----------------------Rx Value Mapper Function--------------------------
  /* 
  Converts any new Rx values into angle setpoints to be used by the PIDs.
  Only calculates for values that have been updated since last time, indicated
  by the updateFlags parameter. 
  
  For roll/pitch/yaw, uses floating point math instead of the map() function
  because it runs faster.
  */
  
  if(updateFlags & 1<<CHANNEL_THROTTLE){  // New throttle value
    // map throttle values directly 
    rxThro = map(rxVal[CHANNEL_THROTTLE], THROTTLE_CUTOFF, THROTTLE_RMAX, THROTTLE_WMIN, THROTTLE_WMAX);
  }
  if(updateFlags & 1<<CHANNEL_ROLL){  // New roll
    rxRoll  = (float)rxVal[CHANNEL_ROLL]*ROLL_RX_MULT - ROLL_RX_SUB;
    rxRoll  = constrain(rxRoll, -ROLL_A_MAX, ROLL_A_MAX);
  }
  if(updateFlags & 1<<CHANNEL_PITCH){  // New pitch
    rxPitch = (float)rxVal[CHANNEL_PITCH]*PITCH_RX_MULT - PITCH_RX_SUB;
    rxPitch = constrain(rxPitch, -PITCH_A_MAX, PITCH_A_MAX);
  }
  if(updateFlags & 1<<CHANNEL_YAW){  // New yaw
    rxYaw   = (float)rxVal[CHANNEL_YAW]*YAW_RX_MULT - YAW_RX_SUB;
    rxYaw   = constrain(rxYaw, -YAW_A_MAX, YAW_A_MAX);
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

void chan0()
// Function attached to RX_PIN_CH0 interrupt.
// Measures pulse lengths of input servo signals.
{
  // if pin is high, new pulse. Record time
  if (digitalRead(RX_PIN_CH0) == HIGH)
  {
    rxStart[0] = micros();
  }
  // if pin is low, end of pulse. Calculate pulse length and add channel flag. Also reset channel start.
  else
  {
    rxValShared[0] = micros() - rxStart[0];
    rxStart[0] = 0;
    updateFlagsShared |= RX_FLAG_CH0;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void chan1()
// Function attached to RX_PIN_CH1 interrupt.
// Measures pulse lengths of input servo signals.
{
  // if pin is high, new pulse. Record time
  if (digitalRead(RX_PIN_CH1) == HIGH)
  {
    rxStart[1] = micros();
  }
  // if pin is low, end of pulse. Calculate pulse length and add channel flag. Also reset channel start.
  else
  {
    rxValShared[1] = micros() - rxStart[1];
    rxStart[1] = 0;
    updateFlagsShared |= RX_FLAG_CH1;
  }
}

void chan2()
// Function attached to RX_PIN_CH2 interrupt.
// Measures pulse lengths of input servo signals.
{
  // if pin is high, new pulse. Record time
  if (digitalRead(RX_PIN_CH2) == HIGH)
  {
    rxStart[2] = micros();
  }
  // if pin is low, end of pulse. Calculate pulse length and add channel flag. Also reset channel start.
  else
  {
    rxValShared[2] = micros() - rxStart[2];
    rxStart[2] = 0;
    updateFlagsShared |= RX_FLAG_CH2;
  }
}

void chan3()
// Function attached to RX_PIN_CH3 interrupt.
// Measures pulse lengths of input servo signals.
{
  // if pin is high, new pulse. Record time
  if (digitalRead(RX_PIN_CH3) == HIGH)
  {
    rxStart[3] = micros();
  }
  // if pin is low, end of pulse. Calculate pulse length and add channel flag. Also reset channel start.
  else
  {
    rxValShared[3] = micros() - rxStart[3];
    rxStart[3] = 0;
    updateFlagsShared |= RX_FLAG_CH3;
  }
}
