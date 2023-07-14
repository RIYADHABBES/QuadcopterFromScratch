#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/JoyPadManager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/JoyPadManager.cpp"

#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/MPU6050Manager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/MPU6050Manager.cpp"

MPU6050Manager* mpu6050Manager = nullptr;
unsigned long time_storage; // memorization variable of the present moment

JoyPadManager* joyPadManager = nullptr;

void setup(int a) {
   Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("***************************");
  Serial.println("Welcom to MPU6050 + JoyPad Tester :) ");
  
  mpu6050Manager = new MPU6050Manager();
  joyPadManager = new JoyPadManager();

  Serial.println("\nSetup began ");
  Serial.println("Wait few seconds ...");
  delay(2000);  

  Serial.println("\n Setup ended ");
  Serial.println("***************************\n\n");
time_storage = micros();
}

void loop(int b) {

  
  mpu6050Manager->calculateAnglesFusion();

  joyPadManager->calculateSetpoints(mpu6050Manager->getRollAdjustment(),mpu6050Manager->getPitchAdjustment());

  float *setpoint = joyPadManager->getSetPoints();
  float *gyro = mpu6050Manager->getGyro();
  float error[3] = {0, 0, 0};

  error[0] = setpoint[0] - gyro[0];
  error[1] = setpoint[1] - gyro[1];
  error[2] = setpoint[2] - gyro[2];
  
  //joyPadManager->printData();
  //if(!joyPadManager->getButton1())
  {
   // mpu6050Manager->printAngles();
   // joyPadManager->printSetPoints();
    Serial.print("error[ROLL]; ");
    Serial.print(error[0]);
    Serial.print("error[PITCH]; ");
    Serial.print(error[1]);
    Serial.print("error[YAW]; ");
    Serial.print(error[2]);
    Serial.println(" ");
  }
  mpu6050Manager->set_dT(micros() - time_storage);
  time_storage = micros();

  mpu6050Manager->readMPU6050();
  joyPadManager->readRadio();
  
}
