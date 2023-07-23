#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/MPU6050Manager.h"
#include "C:/Users/noufe/OneDrive/Bureau/QuadcopterFromScratch/Main_Drone/Main_Arduino_Flight_Controller_Stab_Ver_1_0/MPU6050Manager.cpp"

MPU6050Manager* mpu6050Manager = nullptr;
unsigned long time_storage; // memorization variable of the present moment

void setup(int a) {
   Serial.begin(9600);
  // put your setup code here, to run once:
  Serial.println("***************************");
  Serial.println("Welcom to MPU6050 Tester :) ");
  mpu6050Manager = new MPU6050Manager();
  Serial.println("\nSetup began ");
  Serial.println("Wait few seconds ...");
  delay(2000);  

  Serial.println("\n Setup ended ");
  Serial.println("***************************\n\n");
//time_storage = micros();
}

void loop(int b) {
  // put your main code here, to run repeatedly:

  mpu6050Manager->calculateAnglesFusion();

  // mpu6050Manager->set_dT(micros() - time_storage);
  // time_storage = micros();

  mpu6050Manager->readMPU6050();

  mpu6050Manager->printAngles();
}
