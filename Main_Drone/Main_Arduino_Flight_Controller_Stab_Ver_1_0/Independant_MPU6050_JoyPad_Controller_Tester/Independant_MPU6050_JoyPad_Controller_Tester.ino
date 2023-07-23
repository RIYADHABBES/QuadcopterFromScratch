#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/JoyPadManager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/JoyPadManager.cpp"

#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/MPU6050Manager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/MPU6050Manager.cpp"

#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/ControllerManager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/ControllerManager.cpp"

MPU6050Manager* mpu6050Manager = nullptr;
unsigned long time_storage; // memorization variable of the present moment
unsigned long loop_start;
JoyPadManager* joyPadManager = nullptr;

ControllerManager* controllerManager = nullptr;

void setup(int a) {
   Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("***************************");
  Serial.println("Welcom to MPU6050 + JoyPad Tester :) ");
  
  mpu6050Manager = new MPU6050Manager();
  joyPadManager = new JoyPadManager();
  controllerManager = new ControllerManager();

  Serial.println("\nSetup began ");
  Serial.println("Wait few seconds ...");
  delay(2000);  

  Serial.println("\n Setup ended ");
  Serial.println("***************************\n\n");
//time_storage = micros();
}

void loop(int b) {

  loop_start = micros();
  mpu6050Manager->calculateAnglesFusion();

  joyPadManager->calculateSetpoints(mpu6050Manager->getRollAdjustment(),mpu6050Manager->getPitchAdjustment());

  const float *setpoint = joyPadManager->getSetPoints();
  const float *gyro = mpu6050Manager->getGyro();
  
  controllerManager->setSetPoints(setpoint);
  controllerManager->setGyro(gyro);
  
  controllerManager->calculatePIDCommands();  
  
  controllerManager->calculateESCPulses();
  if(!joyPadManager->getTSwitch2())
  {
    controllerManager->generateESCPulses();
  }
  else
  {
    controllerManager->stopMotors();
  }
//  mpu6050Manager->set_dT(micros() - time_storage);
//  time_storage = micros();

  mpu6050Manager->readMPU6050();
  joyPadManager->readRadio();
  
  Serial.print((micros()-loop_start) / 1000.0);
}
