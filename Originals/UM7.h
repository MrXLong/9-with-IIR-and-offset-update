/* 
 * File:   UM6.h
 * Author: Don
 *
 * Created on 4. Januar 2015, 14:22
 */

#ifndef UM7_H
#define	UM7_H

#include "p33EP512MU810.h"
#include <libpic30.h>
#define FCY 40000000ULL
#include "PID.h"

#define BAUDRATE_UM7 57600
#define BRGVAL_UM7   ((FCY/BAUDRATE_UM7)/16)-1

// Size of the buffer for received data (in Byte)
#define UM7_TxBufSize                           7
// Size of the buffer for received data (in Byte)
#define UM7_RxBufSize                           28              // maximum required size: 20 bytes for 5 sent Register

// Factors for transmitted sensor data to obtain the actual values
#define GyroConstant        0.0610352       // processedGyro * GyroConstant     = [degrees/sec (º/s)]
#define AcelConstant        0.000183105     // processedAcel * AcelConstant     = [gravities (g)]
#define MagConstant         0.000305176     // processedMag * MagConstant       = unit-norm (assuming proper calibration) magnetic-field vector
#define EulerConstant       0.0109863       // eulerAngle * EulerConstant       = [Degrees (º)]

// Factors for adjusting the actual value to the setpoint levels level             (Faktor = ratio_x)
#define ratio_eulerAngle    0.733
#define ratio_eulerRate     1.0

// Offset for the position angle and angular velocities
#define offset_roll_angle   105
#define offset_pitch_angle  0
#define offset_yaw_rate     0.0

// Structure for holding received packet information
/*typedef struct UM6_packet_struct
{
    uint8_t Address;
    uint8_t PT;
    uint16_t Checksum;
    uint8_t data_length;
    uint8_t data[30];
} UM6_packet;
*/

extern int error;
extern int calibrateDriftError;//Variable to allow the calibration of the gyroscopes drift error
extern int detect;
extern int currentRoll;
extern int currentPitch;
extern int previousRoll;
extern int previousPitch;
extern int varianceRoll;
extern int variancePitch;

union Data{
    int data16[16];                         // 16*8 = 128
    float data32[8];                        // 32*4 = 128
    unsigned char Byte[32];                 // 8*16 = 128
};

typedef struct communicationUM7{// Struct to safe and use the sensor data from UM7
    unsigned char Address;
    unsigned char PacketType;
    union Data DataBytes;
    unsigned char Checksum1;
    unsigned char Checksum0;
}UM7_Commun;

typedef struct UM7Data{ // Struct to safe and use the sensor data from UM7
    int processedGyroX; //0x5C
    int processedGyroY; //0x5C
    int processedGyroZ; //0x5D
    int processedAcc_X; //0x5E
    int processedAcc_Y; //0x5E
    int processedAcc_Z; //0x5F
    int processedMagX;  //0x60
    int processedMagY;  //0x60
    int processedMagZ;  //0x61
    int eulerRollAngle; //0x70 (phi)
    int eulerPitchAngle;//0x70 (theta)
    int eulerYawAngle;  //0x71 (psi)
    int eulerYawRate;   // 0x73 (d_psi/d_t)
    float covariance[15];//0x66 - 0x75
    float temperature;  //0x76
    _Q16 varianceRoll;
    _Q16 variancePitch;
}UM7DataSensor;

//DMA buffers. They contain the received and send data through RS232.
extern unsigned char uart2RxBuff[UM7_RxBufSize];
extern unsigned char uart2TxBuff[UM7_TxBufSize];

extern UM7_Commun UM7dataSensors;// Struct to contain the data received from UM7 sensor
extern UM7DataSensor UM7_Sensors;// Struct to contain the UM7 data
extern UM7DataSensor *ptr_eulerAngle;

extern unsigned int ZeroGyrosWrong;// Flag to indicate when the calibrate gyros order was successful
extern volatile unsigned int incomingUM7Message;// Flag to indicate if there are a new UM6 message ready to use

extern float promediumYawError;//Yaw promedium error in half second. The IMU sends data 20 times / sec (20 Hz) and we store 10 values.
/****************************************************************************
Function:     initUM7communication
Parameters:   none
Return value: none
This function initializes the UART2 module to use the UM7 Sensor communication
via DMA channel 3,4.
****************************************************************************/
void cfgDMA3_UART_Rx(void);
void cfgDMA4_UART_Tx(void);
void cfgUART2 (void);
void initUM7communication(void);
void ZeroGyros(void);
void setAcc_Ref_Vec (void);
void setMag_Ref_Vec (void);
void resetEKF (void);
void determineVariance (UM7DataSensor *ptr_eulerAngle);

/****************************************************************************
Function:     ZeroGyros
Parameters:   none
Return value: none
This function recalibrates the gyroscope writing the UM7 sensor via DMA3.
 * The UM7 (IMU) is configured to calibrate the gyros every start. So right
 * now using this function is not so necessary.
 * IMPORTANT!! The function produces problems such as break execution or send
 * data to the slaves boards. These sendings sometime (not often) start
 * movements without control.
****************************************************************************/



/****************************************************************************
Function:     storeUM7data
Parameters:   data
Return value: none
This function store data from UM7 sensor to use it later.
****************************************************************************/
void storeUM7data(UM7_Commun data);

#endif	/* UM7_H */

