
// ============================================================================
// ===                          GENERAL DEFS                                ===
// ============================================================================

//#define USE_SERIAL

// macro to enable Serial if any debugging is defined
#if defined(DEBUG_ANGLES)   || defined(DEBUG_SENSOR_RAW) || defined(DEBUG_MAG) ||         \
    defined(DEBUG_RX_RAW)   || defined(LOOP_TIMER)       || defined(DEBUG_RX_ANGLES) ||   \
    defined(DEBUG_PID_ROLL) || defined(DEBUG_PID_PITCH)  || defined(DEBUG_PID_YAW) ||     \
    defined(TEST_ROLL)      || defined(DEBUG_MOTOR_SETPOINTS)
  #ifndef USE_SERIAL
    #define USE_SERIAL
  #endif
#endif

// ============================================================================
// ===                             PID DEFS                                 ===
// ============================================================================

#define PID_SMPL_TIME 1000/PID_SMPLRT

#define ROLL_PID_KP KP
#define ROLL_PID_KI KI / PID_SMPLRT
#define ROLL_PID_KD KD * PID_SMPLRT

#define PITCH_PID_KP KP
#define PITCH_PID_KI KI / PID_SMPLRT
#define PITCH_PID_KD KD * PID_SMPLRT

#define YAW_PID_KP YAW_KP
#define YAW_PID_KI YAW_KD / PID_SMPLRT
#define YAW_PID_KD YAW_KI * PID_SMPLRT

#if !defined(RX_MIXING_ONLY)
  #define PID_CONTROL
#endif


// ============================================================================
// ===                          SENSOR DEFS                                 ===
// ============================================================================


#define NEW_ACC_FLAG 1
#define NEW_GYRO_FLAG 2
#define NEW_MAG_FLAG 4

// *************ADXL435 Accelerometer***************

#define ACC_SMPL_RATE 50  // 50Hz 

#define ADXL345 0x53         // B1010011 (ADXL345)
#define ADXL345_TO_READ 6    // read 6 bytes
#define ADXL345_DATAX0 0x32  // Acceleration data starts at register 0x32 (DEC 50)
#define ADXL345_BW_RATE 0x2C  // Data rate register
#define ADXL345_DATA_FORMAT 0x31    // data format register
#define ADXL345_DATA_RATE B00001001 // Set sample rate to 50Hz
#define ADXL345_DATA_FORMAT_W 0x0B  // set to full resolution and +/- 16g


// **************ITG-3200 Gyro**********************
#define GYRO_SMPL_RATE 1000  // 500Hz

#define WHO_AM_I 0x00
#define SMPLRT_DIV 0x15   // 
#define DLPF_FS 0x16      // address of the digital low pass filter
#define GYRO_XOUT_H 0x1D  // only really need this one, data starts from here
#define GYRO_XOUT_L 0x1E
#define GYRO_YOUT_H 0x1F
#define GYRO_YOUT_L 0x20
#define GYRO_ZOUT_H 0x21
#define GYRO_ZOUT_L 0x22

// Write B00011011 to DLPF_FS
#define SMPLRT_DIV_VAL 1      // Sample rate divider = 1, means sample rate of 500Hz
#define DLPF_CFG_0 (0<<0)     // DLPF_CFG Sets the low pass filter cutoff
#define DLPF_CFG_1 (1<<1)     // Set to 5 for 10Hz 
#define DLPF_CFG_2 (1<<2)
#define DLPF_FS_SEL_0 (1<<3)  // FS_SEL set to 3 for +/-2000 deg/s
#define DLPF_FS_SEL_1 (1<<4)

#define ITG3200 0x68    // B1101000
#define ITG3200_TO_READ 6  // read 6 bytes


// **************HMC5883L Compass*******************

#define MAG_SMPL_RATE 30  // 30Hz
#define MAG_SMPL_TIME 1000 / MAG_SMPL_RATE 

#define HMC5883L 0x1E  // B0011110

#define CONFIG_A 0x00
#define CONFIG_B 0x01
#define MODE_REGISTER 0x02
#define MAG_OUT_X_H 0x03
#define MAG_OUT_X_L 0x04
#define MAG_OUT_Z_H 0x05
#define MAG_OUT_Z_L 0x06
#define MAG_OUT_Y_H 0x07
#define MAG_OUT_Y_H 0x08

#define CONFIG_A_SETTING B01010100  // 30 Hz
#define CONFIG_B_SETTING B00000000  // Set to most sensitive gain
#define MODE_SETTING B00000000      // Set to continuous measurement

#define MAG_TO_READ 6


// ============================================================================
// ===                           MOTOR DEFS                                 ===
// ============================================================================

// define pins to be used to output servo controls
// these pin assignments may not be changed.
#define OUTPUT_PIN_M0 9
#define OUTPUT_PIN_M1 10
#define OUTPUT_PIN_M2 11
#define OUTPUT_PIN_M3 3


// ============================================================================
// ===                             RX DEFS                                  ===
// ============================================================================

#define NUMBER_CHANNELS 4

// define flags for channels in updateFlagsShared
#define RX_FLAG_CH0 1  //B00000001
#define RX_FLAG_CH1 2  //B00000010
#define RX_FLAG_CH2 4  //B00000100
#define RX_FLAG_CH3 8  //B00001000

#define THROTTLE_WMIN MOTOR_ARM_START
#define THROTTLE_WMAX MOTOR_MAX_LEVEL

#define YAW_RX_MULT YAW_A_MAX/400
#define YAW_RX_SUB 1500*YAW_RX_MULT

#define ROLL_RX_MULT ROLL_A_MAX/400
#define ROLL_RX_SUB 1500*ROLL_RX_MULT

#define PITCH_RX_MULT PITCH_A_MAX/400
#define PITCH_RX_SUB 1500*PITCH_RX_MULT

// For reference only, min/max values for channel readings
// currently, Rx reading deflection is based on 1500 +/- 350 
#define YAW_RMIN 1080  
#define YAW_RMAX 1850
#define ROLL_RMIN 1084
#define ROLL_RMAX 1856
#define PITCH_RMIN 1092
#define PITCH_RMAX 1860

// ============================================================================
// ===                            LED DEFS                                  ===
// ============================================================================

#define RED   0
#define GREEN 1
#define BLUE  2
