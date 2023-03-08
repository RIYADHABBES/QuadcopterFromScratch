
// ============================================================================
// ===                          CONFIGURATION                               ===
// ============================================================================


#define ARMED    // Quad safe if not defined.


// ============================================================================
// ===                            DEBUGGING                                 ===
// ============================================================================

// *****************Debug Printing********************

#define DEBUG_ANGLES          // print roll and pitch
//#define DEBUG_SENSOR_RAW      // print raw sensor vals
//#define DEBUG_MAG             // print mag readings
//#define DEBUG_RX_RAW          // print raw Rx channel values
//#define DEBUG_RX_ANGLES       // print mapped Rx setpoints
//#define DEBUG_PID_ROLL        // print roll PID output
//#define DEBUG_PID_PITCH       // print pitch PID output
//#define DEBUG_PID_YAW         // print yaw PID output
//#define DEBUG_MOTOR_SETPOINTS // print values to be sent to motors
//#define TEST_ROLL             // print roll setpoint and measured roll angle
//#define LOOP_TIMER            // print loop times

// *************Alternate Motor Control***************

//#define RX_MIXING_ONLY        // skip PIDs, directly mix 4 Rx channels into motor output
//#define FORWARD_INDIV 0       // forward throttle to one motor only, selected by number



// ============================================================================
// ===                          PID CONFIG                                  ===
// ============================================================================

#define KP 1.5
#define KD 0.3
#define KI 0

#define YAW_KP 3
#define YAW_KD 0
#define YAW_KI 0
  
#define PID_SMPLRT 500    // PID update rate in Hz

#define I_MAX 100         // Max output if integral control term

#define ROLL_A_MAX 20.0   // Max roll angle setpoint in deg. 
#define PITCH_A_MAX 20.0  // Max pitch angle setpoint in deg. 
#define YAW_A_MAX 60.0    // Max yaw rate setpoint in deg/s.



// ============================================================================
// ===                          SENSOR CONFIG                               ===
// ============================================================================

//#define I2C_FAST_MODE


// **************State Estimation*******************
//#define COMP_FILTER
#define COMP_SPLIT 0.99 // gyro bias for complimentary filter

#define ACCEL_GAIN 1    // accelerometer gain for state estimation

// *************ADXL435 Accelerometer***************
//#define CALIBRATE_ACC
#define ACC_CAL_SMPL_NUM 50

#define ACC_OFFSET_X -12
#define ACC_OFFSET_Y -15
#define ACC_OFFSET_Z -32

// **************ITG-3200 Gyro**********************
#define GYRO_CAL_SMPL_NUM 100  // average 100 readings for offset

#define GYRO_OFFSET_X 0.42
#define GYRO_OFFSET_Y 2.65
#define GYRO_OFFSET_Z 0.21

// **************HMC5883L Compass*******************

//#define MAG



// ============================================================================
// ===                          MOTOR CONFIG                                ===
// ============================================================================

//#define USE_SERVO_LIB
#define USE_MOTOR_PWM


#define MOTOR_ARM_START 1000    // minimum motor level for motors to spin
#define MOTOR_MAX_LEVEL 1800    // max motor power from throttle only



// ============================================================================
// ===                            RX CONFIG                                 ===
// ============================================================================

#define THROTTLE_CUTOFF 1050  // threshold for throttle input. Set to just above bottom stick.
#define THROTTLE_RMAX 1860    // maximum throttle value

// pins to be used to read reciever signals for each channel
#define RX_PIN_CH0 6
#define RX_PIN_CH1 7
#define RX_PIN_CH2 8
#define RX_PIN_CH3 4

// which channel corresponds to which input
#define CHANNEL_THROTTLE 0
#define CHANNEL_ROLL 1
#define CHANNEL_PITCH 2
#define CHANNEL_YAW 3

// ============================================================================
// ===                           LED CONFIG                                 ===
// ============================================================================

#define RED_LED   13
#define GREEN_LED 12
#define BLUE_LED  2

