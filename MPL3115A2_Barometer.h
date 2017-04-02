
/* 
 * File:   MPL3115A2_Barometer
 * Author: LJ
 * Comments: Headfile for the MPL3115A2_Barometer
 * Revision history: 
 */
#ifndef _MPL3115A2_BAROMETER_H_
#define _MPL3115A2_BAROMETER_H_
#include "Filter.h"

extern long MPL_Barometer_Dist_cm;
extern float MPL_Barometer_Dist_M;
extern float MPL_Dist;
extern unsigned char STA;
extern unsigned char MSB;
extern unsigned char CSB;
extern unsigned char LSB;
extern float MPL_Barometer_offset;
extern int offset_update_MPL3115A2;
extern IIR_Filter_Elements Baro_IIR_Elements;
extern IIR_Coeff Baro_IIR_Coeff_3;
//extern long MPL_Pressure;

#define SlaveAddressIIC  0xC0

// This barometer could be set as Barometer or Altimeter, with this function 
// I have chosen it as a Altimeter. You could check the data-sheet to see more possibilities
void MPL3115A2_Barometer_initial();

//I write this function to convert the 0xFFFFFF into decimal, also, it should use the UART K6 to print the data on the monitor
void MPL_withK6();
void MPL_Barometer_Dist_cm_withK6();
void MPL_Barometer_Dist_M_withK6();
void MPL_Pressure_withK6();

// This function help to feedback the Altitude. Further functions may be developed with the existed function in the Ulrasonic
void witre_reading_command( unsigned char MSB, unsigned char CSB, unsigned char  LSB);
void Read_MPL_Barometer_polling(int if_with_IIR );
void Read_MCL_SB();
void Calculate_MCL_SB();
void Read_MPL_NoMatterWhat();
void MPL_Barometer_offset_function(int times);
void offset_update();



#endif