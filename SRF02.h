/****************************************************************************

 Header File:		Utrasonic.h

 In this Header file the functions to initialize the utrasonic sensors and
 get the measured distance are defined.

****************************************************************************/
#ifndef __UTRASONIC_H
#define __UTRASONIC_H

#include "PID.h"    //necessary to dereference s_controller_fixedpoint

/*** Number of sonar sensors ***********************************************/
#define Sonar_NumSens		1 //5

/*** I2C address definitions ***********************************************/
//address of sonar sensor 1
#define Sonar_SensAddr1 0xE0 //Note this is 0xE0 on boxcopter
/*
//address of sonar sensor 2
#define Sonar_SensAddr2		0xE4
//address of sonar sensor 3
#define Sonar_SensAddr3		0xE2
//address of sonar sensor 4
#define Sonar_SensAddr4		0xE8
//address of sonar sensor 5
#define Sonar_SensAddr5		0xEA
//address of an unprogrammed sonar sensor
#define Sonar_SensAddrU		0xE0
 */



/*** command definitions ***************************************************/
#define Sonar_CmdPingInch	0x50
#define Sonar_CmdPingCm		0x51
#define Sonar_CmdPingUs		0x52

/*** miscellaneous definitions *********************************************/
//Maximum distance of sonar sensor in cm
#define SONAR_MAX   200
//Minimum effective distance of sonar sensor in cm
#define SONAR_MIN   15        

// this value gives 80ms out of the timer6 calculation.
#define PeriodofChecking80ms    12500
//this constant array contains the I2C2 addresses of each sonar sensor
extern const unsigned char Sonar_I2C2Addr[Sonar_NumSens];
//this array contains the messured distances of each sonar sensor
extern unsigned int Sonar_Dist[Sonar_NumSens];
// this variable for the current active sonar sensor.
extern unsigned char HwCtrl_SonarCh;
extern unsigned int Sonar_AvoidObstaclesvalues[Sonar_NumSens];
extern unsigned int Sonar_ClimpingStairsvalues[Sonar_NumSens];

/****************************************************************************
Function:     initSonar
Parameters:   none
Return value: none
This function initializes the sonar sensor variables. The messured distance
of each sensor is set to Sonar_MaxDist. The sensors are marked as not
ready.
****************************************************************************/
void InitUtrasonic ( void );

/****************************************************************************
Function:     initSonarClimpingStairs
Parameters:   none
Return value: none
This function initializes the sonar values to use in the climping stairs mode.
****************************************************************************/
void initSonarClimpingStairs();

/****************************************************************************
Function:     initSonarAvoidObstacles
Parameters:   none
Return value: none
This function initializes the sonar values to use in the avoiding ostacles
mode.
****************************************************************************/
void initSonarAvoidObstacles();

/****************************************************************************
Function:     Sonar_ReadSensor
Parameters:   uint8 SensNo
Return value: none
This function reads the data from the sonar sensor with the number SensNo.
The messured value is stored in the corresponding shadow variable. If the
messurement has failed or the messured distance is greater than
Sonar_MaxDist, the value is set to Sonar_MaxDist.
****************************************************************************/
void Sonar_ReadSensor ( unsigned char SensNo );

/****************************************************************************
Function:     Sonar_SendCommand
Parameters:   uint8 SensNo
              uint8 Cmd
Return value: none
This function sends the command Cmd to the sonar sensor with the number
SensNo.
****************************************************************************/
void Sonar_SendCommand ( unsigned char SensNo, unsigned char Cmd );

/****************************************************************************
Function:     ReadSensors
Parameters:   none
Return value: none
This function reads the sonar sensors and calculates the messured distances
of the IR sensors. This function intilize the sensors all togther then if the
72ms has passed the sensors will be read togther.
****************************************************************************/
void ReadSensors ( void );

unsigned int ReadSensor();

/****************************************************************************
Function:     set_height
Parameters:   s_controller_fixedpoint* controller_fix
Return value: none
This function reads set the heigth position. Hold Z.
****************************************************************************/
void set_height(s_controller_fixedpoint* controller_fix);


void autoLand(s_controller_fixedpoint* ctrlr);
void holdHeight(unsigned int setpoint, s_controller_fixedpoint* ctrlr);

int hasLanded();

#endif //__UTRASONIC_H
