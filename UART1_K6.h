
/* 
 * File:   UART1_K6.h
 * Author: LJ Xie   
 * Comments: this file helps to read data from sensor through uart1 at k6. 
 *           it's use for testing initially
 *           the RX would be set to 0 in order to avoid sending message to the sensor.
 * Revision history: 
 */


extern void Configure_Uart1_K6(void);
void __attribute__((__interrupt__)) _U1TXInterrupt(void);
void output_with_K6(float to_output, int length, int scale, int sign_abs);





