// ============================================================================
// ===                         LED  FUNCTIONS                               ===
// ============================================================================


void ledInit(){
  // initialize the LED pins for output
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  
  // set the LED to red
  setLED(RED);
}

void setLED(byte color){
  // --------Set the LED color---------
  
  switch(color){
    case RED:
      digitalWrite(RED_LED, HIGH);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, LOW);
      break;
    case GREEN:
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(BLUE_LED, LOW);
      break;
    case BLUE:
      digitalWrite(RED_LED, LOW);
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, HIGH);
      break;
  }
}
