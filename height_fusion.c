/*
 * File:   height_fusion.c
 * Author: LJ
 *
 * Created on January 19, 2017, 2:03 PM
 */
#include "SRF02.h"
#include "MPL3115A2_Barometer.h"
#include "p33EP512MU810.h"
#include "height_fusion.h"
#include "UM7.h"
#include <stdlib.h>
#include "UART.h"

#define Kalman_dt 0,15 // read the sensor every 150ms 
#define dt_hoch2_devide2 (0.15*0.15)/2

/* Global variable*/
long Fusion_Height;

/* local variable*/
float trust_on_Ultrasonic;
float turst_on_Barometer = 0.02;

void Height_Fusion_of_Baro_and_Ultra()
{
    if( (Sonar_Dist[0] <= 38) ){trust_on_Ultrasonic = 0.03;}
    if( (Sonar_Dist[0] == 0) ){trust_on_Ultrasonic = 1;}
    if( (Sonar_Dist[0] == 200)){trust_on_Ultrasonic = 0;}
    if( (Sonar_Dist[0] == 200) & (MPL_Barometer_Dist_cm < 160)  )
    {   
        Sonar_Dist[0] = 15;
        trust_on_Ultrasonic = 0.05;
    }
    if( (Sonar_Dist[0] < 200) & (MPL_Barometer_Dist_cm > 40) ){ trust_on_Ultrasonic = 1;}
    Fusion_Height = (Sonar_Dist[0] * trust_on_Ultrasonic + MPL_Barometer_Dist_cm * turst_on_Barometer) / ( trust_on_Ultrasonic + turst_on_Barometer ) -17;
}

void compare_Ultra_and_Barometer()
{
    int i = 0.0;
   
    i = Sonar_Dist[0] - MPL_Barometer_Dist_cm;
    //j = (int)(MPL_Barometer_Dist);
    //i = MPL_Barometer_Dist -j;
    if(i >= 0){
        U1TXREG = 0x3E;
        //U1TXREG = 0x31 ;
    }
    else 
        U1TXREG = 0x3C;
        //U1TXREG = 0x32 ;
}








