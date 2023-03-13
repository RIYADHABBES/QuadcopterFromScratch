/*
    DIY Arduino based RC Transmitter Project
              == Receiver Code ==

  by Dejan Nedelkovski, www.HowToMechatronics.com
  Library: TMRh20/RF24, https://github.com/tmrh20/RF24/
*/
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include <Servo.h>

Servo motor1, motor2, motor3, motor4;	// we declare a 'variable' motor of type "Servo" (it is an object instance)
#define MOTOR1_PIN 5
#define MOTOR2_PIN 6
#define MOTOR3_PIN 9
#define MOTOR4_PIN 3
#define GAS_MAX 180
#define GAS_MIN 0


int gasStickTouchZeroAtLeastOnce = 0;

RF24 radio(7, 8);   // nRF24L01 (CE, CSN)
constexpr byte address[6] = "00001";

static unsigned long lastReceiveTime = 0;
static unsigned long currentTime = 0;

static unsigned long lastPrintDataTime = 0;

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotX; 
  byte j1PotY; 
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};

Data_Package data; //Create a variable with the above structure

void setup() {
  Serial.begin(9600);

  setMotors();

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening(); //  Set the module as receiver

  resetData();
  
}
void loop() {
  // Check whether there is data to be received
  if (radio.available()) 
  {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data

/*
    if( currentTime - lastPrintDataTime > map(data.pot2, 0, 255, 0, 4000) )
    { // Control the speed of writing to the serial monitor using the potentiometer 2
      printDebug(data);
      lastPrintDataTime = currentTime;
    }
*/
    // Check if the gas stick is secured  to 0 before starting the propellers roatation
      if(data.j1PotX == 0 && gasStickTouchZeroAtLeastOnce == 0)
      {
        gasStickTouchZeroAtLeastOnce++;

        Serial.print("Securing the gas stick is done: ");
        Serial.println(gasStickTouchZeroAtLeastOnce);
      }

      // Start Rotating the propllers when acting on the gas stick
      if(gasStickTouchZeroAtLeastOnce)
      {
        motor1.write(map(data.j1PotX, 0, 255, GAS_MIN, GAS_MAX));
        motor2.write(map(data.j1PotX, 0, 255, GAS_MIN, GAS_MAX));
        motor3.write(map(data.j1PotX, 0, 255, GAS_MIN, GAS_MAX));
        motor4.write(map(data.j1PotX, 0, 255, GAS_MIN, GAS_MAX));

        Serial.print("Value on motor: ");
        Serial.println(map(data.j1PotX, 0, 255, GAS_MIN, GAS_MAX));
      }
    
  }
    
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone has a throttle up and we lose connection, it can keep flying unless we reset the values
    Serial.println("No Signal Recived");    
  }
}

void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 127;
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1; 
  data.j2Button = 1; 
  data.pot1 = 1; 
  data.pot2 = 1; 
  data.tSwitch1 = 1; 
  data.tSwitch2 = 1; 
  data.button1 = 1; 
  data.button2 = 1; 
  data.button3 = 1; 
  data.button4 = 1; 
  gasStickTouchZeroAtLeastOnce = 0;
}

void printDebug(const Data_Package& data) {
  // Print the data in the Serial Monitor
  Serial.print("j1PotX: ");
  Serial.print(data.j1PotX);
  Serial.print("; j1PotY: ");
  Serial.print(data.j1PotY);
  Serial.print("; j1Button: ");
  Serial.println(data.j1Button);
  
  Serial.print("j2PotX: ");
  Serial.print(data.j2PotX);
  Serial.print("; j2PotY: ");
  Serial.print(data.j2PotY);
  Serial.print("; j2Button: ");
  Serial.println(data.j2Button);

  Serial.print("button1: ");
  Serial.print(data.button1);
  Serial.print("; button2: ");
  Serial.print(data.button2);
  Serial.print("; button3: ");
  Serial.print(data.button3);
  Serial.print("; button4: ");
  Serial.println(data.button4);

  Serial.print("pot1: ");
  Serial.print(data.pot1); 
  Serial.print("; pot2: ");
  Serial.print(data.pot2); 
  Serial.print("; tSwitch1: ");
  Serial.print(data.tSwitch1); 
  Serial.print("; tSwitch2: ");
  Serial.println(data.tSwitch2); 
  Serial.println();
}

void setMotors()
{
    motor1.attach(MOTOR1_PIN, 1000, 2000); // ESC control on pin x, min and max PWM durations : 1000us and 2000us
    motor1.write(GAS_MIN); // send GAS MIn command (180 corresponds to pulses of 2ms duration)

    motor2.attach(MOTOR2_PIN, 1000, 2000);
    motor2.write(GAS_MIN);

    motor3.attach(MOTOR3_PIN, 1000, 2000);
    motor3.write(GAS_MIN);

    motor4.attach(MOTOR4_PIN, 1000, 2000);
    motor4.write(GAS_MIN);
}