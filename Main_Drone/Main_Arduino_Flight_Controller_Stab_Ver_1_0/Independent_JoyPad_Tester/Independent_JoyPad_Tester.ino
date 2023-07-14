#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/JoyPadManager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/JoyPadManager.cpp"

JoyPadManager* joyPadManager = nullptr;

void setup(int a) {
   Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("***************************");
  Serial.println("Welcom to joyPad Tester :) ");
  
  Serial.println("\nSetup began ");
  
  joyPadManager = new JoyPadManager();

  Serial.println("Wait few seconds ...");
  delay(2000);  

  Serial.println("\n Setup ended ");
  Serial.println("***************************\n\n");
}

void loop(int b) {
  // put your main code here, to run repeatedly:
joyPadManager->readRadio();

//joyPadManager->printData();

joyPadManager->calculateSetpoints();

joyPadManager->printSetPoints();
}
