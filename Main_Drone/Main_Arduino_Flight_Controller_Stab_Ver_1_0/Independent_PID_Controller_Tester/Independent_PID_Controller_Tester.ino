#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/ControllerManager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/ControllerManager.cpp"

#include "firstOrderSystem.h"

ControllerManager* controllerManager = nullptr;
FirstOrderSystem* firstOrderSystem = nullptr;

static int counter = 0 ;
void setup(int a) {
   Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("***************************");
  Serial.println("Welcom to PID Controller Tester :) ");
  
  Serial.println("\nSetup began ");
  
  controllerManager = new ControllerManager();
  firstOrderSystem = new FirstOrderSystem(0.9 , 1.0, 3.0);
  

  controllerManager->setSetPointROLL(2);
  controllerManager->setAngleROLL(2.0);

  Serial.println("Wait few seconds ...");
  delay(2000);  

  Serial.println("\n Setup ended ");
  Serial.println("***************************\n\n");
}

void loop(int b) {
  // put your main code here, to run repeatedly:

  if(++counter < 50)
  {
    controllerManager->calculatePIDCommands();
    controllerManager->setAngleROLL(firstOrderSystem->computeOutput(controllerManager->getInputROLL()));

    Serial.print("SetPoint ");
    Serial.print(controllerManager->getSetPointROLL());
    Serial.print(" ");
    Serial.print("Output ");
    Serial.println(controllerManager->getAngleROLL());
  }
  else
    counter = 100;

  delay(5);
}
