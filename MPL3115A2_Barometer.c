/*
 * File:   MPL3115A2_Barometer.c
 * Author: LJ
 *
 * Created on January 16, 2017, 3:10 PM
 */

#define FCY 40000000ULL
#define Baro_init_tuning    20
#define IIR_count 4

#include "MPL3115A2_Barometer.h"
#include "I2C.h"
#include "p33EP512MU810.h"
#include <math.h>
#include "Filter.h"
#include "matrix_calculator.h"
#include <stdlib.h>
#include "UART1_K6.h"
#include <libpic30.h>
#include "UM7.h"
#include "height_fusion.h"



/*** Global variables and static variables ******************************************************/
long MPL_Barometer_Dist_cm = 0;
float MPL_Barometer_Dist_M = 0.0;
float MPL_Dist = 0.0;
long MPL_Pressure = 0;
//static int MPL_Barometer_Dist_cm_pre1 = 0;
//static int MPL_Barometer_Dist_cm_pre2 = 0;
//static int MPL_Barometer_Dist_cm_pre3 = 0;
float MPL_Barometer_offset = 0.0;
int offset_update_MPL3115A2 =0;
unsigned char STA, MSB, CSB, LSB;// T_MSB, T_LSB;

IIR_Filter_Elements Baro_IIR_Elements;
IIR_Coeff Baro_IIR_Coeff_1;
IIR_Coeff Baro_IIR_Coeff_2;
IIR_Coeff Baro_IIR_Coeff_3;
//static float IIR_Coeff_1[4]  = { 0.2, 0.2, 0.3, 0.3};
static float IIR_Coeff_1[4]  = { 0.2, 0.2, 0.3, 0.3};
static float IIR_Coeff_2[4]  = {0.1, 0.1, 0.3, 0.5};
static float IIR_Coeff_3[4]  = { 0.2, 0.2, 0.25, 0.35};

// LJ: To set the sensor as BARO MODE, write to 0x26: 0x39
//     to sent the sensor as alti mode, write to 0x26:0xB9

void MPL3115A2_Barometer_initial()
{
    int i = 0;
    I2C1_WriteReg (SlaveAddressIIC, 0x26,0xB8);
    I2C1_WriteReg (SlaveAddressIIC, 0x13,0x07);
    I2C1_WriteReg (SlaveAddressIIC, 0x26,0xB9); //polling 
//    I2C1_WriteReg(SlaveAddressIIC, 0x28,0x11);// interrupt
//    I2C1_WriteReg(SlaveAddressIIC, 0x29,0x80);//interrupt
//    I2C1_WriteReg(SlaveAddressIIC, 0x26,0xB9);//interrupt
    
    for(i = 0;i < IIR_count; i++)
    {
        Baro_IIR_Elements.IIR_Eingang_float[i] = 0;
    }
    IIR_Coeff_Initial(&Baro_IIR_Coeff_1 , IIR_Coeff_1);
    IIR_Coeff_Initial(&Baro_IIR_Coeff_2 , IIR_Coeff_2);
    IIR_Coeff_Initial(&Baro_IIR_Coeff_3 , IIR_Coeff_3);
    
    for(i = 0; i < 50; i++){
//        Read_MPL_NoMatterWhat(); __delay_ms(10);
        Read_MPL_Barometer_polling(0);
    }
    
    MPL_Barometer_offset_function(Baro_init_tuning);

}

void MPL_Barometer_offset_function(int times)
{
    int i = 0;
    float temp[times];
    
    for(i=0; i < times ; i++){
        Read_MPL_NoMatterWhat();
        temp[i] = MPL_Barometer_Dist_cm;
        __delay_ms(100); 
    }
    
    Median_filter(temp, Baro_init_tuning);
    
    for(i = 0; i< times-15; i++){
        MPL_Barometer_offset = MPL_Barometer_offset + temp[i];
    }
    MPL_Barometer_offset = (MPL_Barometer_offset/(times-15)) ; 
    //U1TXREG = (int)(MPL_Barometer_offset/100 + 0,5);
}

void Read_MPL_Barometer_polling(int if_with_IIR )
{   
    STA = I2C1_ReadReg(SlaveAddressIIC, 0x00); //780ms really?! without the 3 reading lines, the whole function takes 452us 
    if( !(STA & 0x08) ){return;}                                             // 2nd TEST 404us
    
    Read_MCL_SB();
    Calculate_MCL_SB();
//          MPL_Barometer_Dist_cm = (long)((MPL_Dist * 100  )  + 0.5);
//           if (MPL_Barometer_Dist_cm < MPL_Barometer_offset )
//          {
//              MPL_Barometer_Dist_cm = 0;
//          }
//           else     
//    if(abs(UM7_Sensors.processedAcc_Z) < 8){return;}

    MPL_Barometer_Dist_cm = (long)(MPL_Dist *100 - MPL_Barometer_offset ) ; 
    if(MPL_Barometer_Dist_cm < 0){ MPL_Barometer_Dist_cm = 0;}
    
    if(if_with_IIR)
    {
        Array_for_the_filter( &Baro_IIR_Elements,  MPL_Barometer_Dist_cm);
        MPL_Barometer_Dist_cm = IIR_Filter(  Baro_IIR_Elements,  Baro_IIR_Coeff_1);
    }
         //if(MPL_Barometer_Dist_cm > -200 && MPL_Barometer_Dist_cm < 0 )
         //{
         //    MPL_Barometer_Dist_cm = abs(MPL_Barometer_Dist_cm);
          //   U1TXREG = 0x3C;
         //}
         //IIR_Coeffition_1
         //IIR_Filter(MPL_Barometer_Dist_cm, MPL_Barometer_Dist_cm_pre1, MPL_Barometer_Dist_cm_pre2, MPL_Barometer_Dist_cm_pre3,
         //          Baro_Filter);
         //MPL_Barometer_Dist_M = MPL_Barometer_Dist_cm /100;
         
         //calculating the pressure
         //Temp_Pressure = MSB;
         //Temp_Pressure <<= 8;
         //Temp_Pressure |= CSB;
         //Temp_Pressure <<= 8;
         //Temp_Pressure |= LSB;
         //Temp_Pressure >>= 4;
         //MPL_Pressure = Temp_Pressure;
         //LSB >>=4;
         //U1TXREG = MSB;
         //U1TXREG = CSB;
         //U1TXREG = LSB;
//    LATBbits.LATB5 = 0;
//         break;
//    }//while ends here

}

void Read_MCL_SB()
{
    //    LATBbits.LATB5 = 1;
    MSB = I2C1_ReadReg(SlaveAddressIIC, 0x01); //419 us 
    CSB = I2C1_ReadReg(SlaveAddressIIC, 0x02); // also 404us
//            LATBbits.LATB5 = 0; // the 2 lines together above take 804us
    LSB = I2C1_ReadReg(SlaveAddressIIC, 0x03); // 404us
//            LATBbits.LATB5 = 0; // the 3 lines above take 1.2ms
            //T_MSB = I2C1_ReadReg(SlaveAddressIIC, 0x04);
            //T_LSB = I2C1_ReadReg(SlaveAddressIIC, 0x05);
            // from here to the break point it takes 45.7us
}

void Calculate_MCL_SB()
{
    long Temp = 0;
    if (MSB > 0X7F)
    {
        Temp = ~((long)MSB << 16 | (long)CSB << 8 | (long)LSB) + 1; // 2's complement the data
        MPL_Dist = (float) (Temp >> 8) + (float) ((LSB >> 4)/16.0); // Whole number plus fraction altitude in meters for negative altitude
        MPL_Dist *= -1.;
    }
    else{   
                //Temp = (((MSB << 8) | CSB)<<4) | (LSB >> 4);
                //MPL_Dist = Temp/16;  // Whole number plus fraction altitude in meters
        MPL_Dist = (float) ((long) ((MSB << 8) | CSB)) + (float) (LSB >> 4) * 0.0625;
                
    }
}

void Read_MPL_NoMatterWhat()
{
    while(1)
    {
    STA = I2C1_ReadReg(SlaveAddressIIC, 0x00); //780ms really?! without the 3 reading lines, the whole function takes 452us 
    if( STA & 0x08 )
    {
        Read_MCL_SB(MSB, CSB, LSB);
        Calculate_MCL_SB(  MSB,   CSB,   LSB);
        MPL_Barometer_Dist_cm = (long)((MPL_Dist *100 - MPL_Barometer_offset)  + 0.5);
        break;
    }  
    }
}

void offset_update()
{
    if(abs(UM7_Sensors.processedAcc_Z) < 8){offset_update_MPL3115A2++;}
    if(offset_update_MPL3115A2 > 10)
    {
        MPL_Barometer_offset = MPL_Dist *100 - Fusion_Height;
        offset_update_MPL3115A2 = 0;
    }
}
