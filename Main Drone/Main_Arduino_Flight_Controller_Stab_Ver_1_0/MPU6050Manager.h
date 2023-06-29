#ifndef MPU6050MANAGER_H
#define MPU6050MANAGER_H




class MPU6050Manager {

public:

  MPU6050Manager();
  void set_dT(const unsigned long dT){ m_dT = dT; }

private:

  void initializeMPU6050(); // MPU6050 circuit initialization function by I2C

  void calibrateMPU6050();  // MPU6050 calibration function : calculation of gyroscope and accelerometer offsets

  void readMPU6050();      // reading MPU6050 data

  void calculateAnglesFusion(); // calculation of Roll and Pitch angles by "Sensor Fusion"    

// Attributes 

/*Needed*/unsigned long m_dT; // variable for calculating the elapsed time between 2 MPU6050 readings
  int m_temperature = 0;

  int m_accel_raw[3] = {0,0,0};  // // table of raw accelerations
  float m_accel_filtered[3] = {0,0,0};// table of filtered accelerations
  float m_accel[3] = {0,0,0};// table of accelerations scaled and without offsets

  int m_gyro_raw[3] = {0,0,0};  // // table of raw angular velocities
  float m_gyro_filtered[3] = {0,0,0}; // table of filtered raw angular velocities 80%/20%
/*Needed*/ float m_gyro[3] = {0,0,0}; // table of angular velocities scaled and without offsets



  float m_angle_gyro[3] = {0,0,0}; // table of angles calculated from angular velocities
  float m_angle_accel[3] = {0,0,0}; // table of angles calculated from accelerations
  float m_angle[3] = {0,0,0}; // table of angles calculated by "sensor fusion"

  long m_gyro_offset[3] = {0,0,0}; // table of gyroscope offsets
  long m_accel_offset[3] = {0,0,0}; // table of accelerometer offsets


  // adjustment variables for the correction of Pitch and Roll angular velocity setpoints
  float m_pitch_adjustment = 0; 
  float m_roll_adjustment = 0; 

  bool m_init_gyro_angles = false; // flag allowing to know if the alignment of
                                // the angles calculated from the gyro values
                                // on the angles calculated from the accelerometer
                                // values has been done

  bool m_selflevel_mode = true; // stabilized mode (true) or acrobatic mode (false)
};

#endif //MPU6050MANAGER_H