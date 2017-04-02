/****************************************************************************
 * File:   SRF02.c
 * Created by: Alejandro Perez on 31.08.2015

 * Description: This file includes functions
 * pertaining to data retreival from Ultrasonic unit and height control using
 * retreived data and user inputs. ??????????

 * Last Modified: 09.12.2015 (MW)
****************************************************************************/


#include "SRF02.h"
#include "I2C.h"
#include "p33EP512MU810.h"
#include "PID.h"
#include "Mixer.h"
#include "defines.h"

#define sensorPos 5    //static position of ultrasonic on quad relative to ground in cm
#define minThrottle 5500    //MW THESE ARE GUESSES!!!!!
#define hovThrottle 6800    //MW THESE ARE GUESSES!!!!!
#define maxThrottle 8500    //MW THESE ARE GUESSES!!!!!



/*** Global variables ******************************************************/
const unsigned char Sonar_I2C1Addr[Sonar_NumSens]
	= {	Sonar_SensAddr1 /*,
		Sonar_SensAddr2,
		Sonar_SensAddr3,
		Sonar_SensAddr4,
		Sonar_SensAddr5*/ };

unsigned int Sonar_Dist[Sonar_NumSens];



/*** Local variables *******************************************************/
//MW these are not local variables just because you commented that they are...

unsigned int Sonar_SensRdy;
//acutal channel of the sonar sensor which is messuring.
unsigned char HwCtrl_SonarCh;
//delay counter for sonar messurement.
unsigned char HwCtrl_SonarDelay;
unsigned int Sonar_AvoidObstaclesvalues[Sonar_NumSens];
unsigned int Sonar_ClimpingStairsvalues[Sonar_NumSens];

/*** Global functions ******************************************************/

//Function descriptions found in SRF02.h
void InitUtrasonic(void)
{
	int i;
	//set the measured distances of every sonar sensor to sonar min distance
	for (i=0; i<Sonar_NumSens; i++)
	{
		Sonar_Dist[i] = SONAR_MIN;
	}
	//mark all sensors as not ready.
	Sonar_SensRdy = 0;
}


void Sonar_ReadSensor ( unsigned char SensNo )
{
	unsigned char data[2];
	//test, if sensor is ready.
	if (I2C1_ReadReg(Sonar_I2C1Addr[SensNo],0)==0xFF )
	{
		//sensor is not ready; mark it and leave this function.
		Sonar_SensRdy &= ~(1<<SensNo);
		return;
	}
    
	//sensor is ready for an I2C1 communication, so read the measured value
	data[0] = I2C1_ReadReg(Sonar_I2C1Addr[SensNo], 2);
	data[1] = I2C1_ReadReg(Sonar_I2C1Addr[SensNo], 3);
	Sonar_Dist[SensNo] = (data[0]<<8)|data[1];
    
	//if the measured value is invalid (==0) or to large, set it to Sonar_MaxDist
	if ((Sonar_Dist[SensNo]==0) | (Sonar_Dist[SensNo]>SONAR_MAX))
	{
		Sonar_Dist[SensNo] = SONAR_MAX;
	}
    
	//mark sensor as ready.
	Sonar_SensRdy |= (1<<SensNo);
}

void Sonar_SendCommand ( unsigned char SensNo, unsigned char Cmd )
{
	//send the command Cmd to the sonar sensor SensNo.
	I2C1_WriteReg (Sonar_I2C1Addr[SensNo], 0, Cmd);
}

void ReadSensors ( void )
{
        for(HwCtrl_SonarCh=0; HwCtrl_SonarCh<1 ; HwCtrl_SonarCh++)
        {
            //Tell the sensors to read in centemeter and to be ready for reading.
            Sonar_SendCommand (HwCtrl_SonarCh, Sonar_CmdPingCm);
        }


        for( HwCtrl_SonarCh=0; HwCtrl_SonarCh<1 ; HwCtrl_SonarCh++)
        {
            // Read all the sensors in a loop.
            Sonar_ReadSensor (HwCtrl_SonarCh);
        }
}

//Doensn't work for some reason

//static unsigned int height=0;
//unsigned int ReadSensor()
//{
//    unsigned char data[2];
//    unsigned int result=0;
//    
//    I2C1_WriteReg(Sonar_SensAddr1, 0, Sonar_CmdPingCm);
//    
//    data[0] = I2C1_ReadReg(Sonar_SensAddr1, 2);
//	data[1] = I2C1_ReadReg(Sonar_SensAddr1, 3);
//    
//    result = (data[0]<<8)|data[1];
//    
//    if(result != 255)
//        height = result;
//
//    
//    Sonar_Dist[0]=height;
//	return height;
//}

//alejandro perez
/*
void set_height(s_controller_fixedpoint* controller_fix){

    int ld;
//    ReadSensors();
    controller_fix->setpoint_height = Sonar_Dist[0];
    
    //MW This is an averaging filter?
    for(ld=0;ld<10;ld++)
    {
//        ReadSensors();
        controller_fix->setpoint_height = (controller_fix->setpoint_height + Sonar_Dist[0])/2;
    }
}
 */

//Matt W
void holdHeight(unsigned int setpoint, s_controller_fixedpoint* ctrlr)
{
    int height=10000;
    float PID_out=0;
    
    //ReadSensors();
    height = Sonar_Dist[0];
    
    //setValueGas(ptr_pulsewidth);
    
    Bound(setpoint,SONAR_MAX,SONAR_MIN);
          
    PID_out = (int)PID_position(setpoint,height,ALTITUDE);
    //PID_out = map(PID_out,(Sonar_MinDist-Sonar_MaxDist),(Sonar_MaxDist-Sonar_MinDist),minThrottle,coeff_fixedpoint.gas_Max);
    //PID_out = map(PID_out,Sonar_MinDist,Sonar_MaxDist,0,9500);
    //if(PID_out < minThrottle)
        //PID_out = minThrottle;
    //if(PID_out > maxThrottle)
        //PID_out = maxThrottle;
    
    ctrlr->setpoint_gas = ctrlr->thr_holded + PID_out;
    
    if(ctrlr->setpoint_gas < minThrottle)
        ctrlr->setpoint_gas = minThrottle;
    if(ctrlr->setpoint_gas > maxThrottle)
        ctrlr->setpoint_gas = maxThrottle;
    
}

void autoLand(s_controller_fixedpoint* ctrlr)
{
    int height=10000;
    float PID_out=0;
    unsigned int gnd_setpoint=0;
    
   //ReadSensors();
    height = Sonar_Dist[0];
    
    //setValueGas(ptr_pulsewidth);
    
    
    if(sensorPos>SONAR_MIN)
        gnd_setpoint=sensorPos;
    else
        gnd_setpoint=SONAR_MIN;

    PID_out = (int)PID_position(gnd_setpoint,height,'h');
    
    PID_out = map(PID_out,(SONAR_MIN-SONAR_MAX),(SONAR_MAX-SONAR_MIN),minThrottle,coeff_fixedpoint.gas_Max);
    if(PID_out < minThrottle)
        PID_out = minThrottle;
    
    ctrlr->setpoint_gas = PID_out;
    
    
    //check accelerometers in intervals and react if value is too high
    //once z acceleromter reads 1 g and is height sensor is at expected value, we know quad has landed, remove power.
    
    //MUST RECONFIGURE IMU IF WANT ACCEL VALUES
}

int hasLanded()
{
    int i=0;
    int height=1000;
    
//   ReadSensors();
    height = Sonar_Dist[0];
    
    //averaging filter
    for(i=0;i<10;i++)
    {
//        ReadSensors();
        height = (height+Sonar_Dist[0])/2;
    }
    
    //height within landing height zone
    if(((height<sensorPos+5)||(height<=SONAR_MIN)) && (height>sensorPos-5))
    {
        return 1;
    }
    else
    {
        return 0;
    }
    
}
