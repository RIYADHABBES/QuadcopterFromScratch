void setup() {
   Serial.begin(9600); //set up serial library baud rate to 9600
}

void loop() {
  Serial.print("I received:");
   if(Serial.available()) //if number of bytes (characters) available for reading from
    { //serial port
      Serial.print("I received:"); //print I received
      Serial.write(Serial.read()); //send what you read
   }
}