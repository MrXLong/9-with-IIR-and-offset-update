/* 
 * File:   analog2digital.h
 * Author: Matt
 *
 * Created on 29. September 2016, 18:39
 */

#ifndef ANALOG2DIGITAL_H
#define	ANALOG2DIGITAL_H

#include "defines.h"

struct ADchannel 
{
	int16_t input;  // raw input
	int16_t value;  // average of the sum of inputs between report outs
	int16_t offset; // baseline at power up 
	int32_t sum;    // used as an integrator
}; // variables for processing an AD channel

struct BattInfo
{
    uint16_t millivolts;
    float  volts;
    int16_t  milliamps;
    int8_t   remaining;
    float safeMaxVolts;
    float safeMinVolts;
}; //variables for holding battery information


extern struct ADchannel AN1;
extern struct ADchannel AN2;
extern struct BattInfo  battery;


void init_ADC(void);
void init_DMA5(void);

void calc_battery_life(void);

#endif	/* ANALOG2DIGITAL_H */

