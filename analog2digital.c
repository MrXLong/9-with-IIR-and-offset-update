// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#include "Pins.h"
#include "p33EP512MU810.h"
#include "analog2digital.h"
#include "defines.h"
#include "options.h"    //info about battery for monitoring, airframe type

//Fp is internal clock
//Tp = 1/Fp
//Tad= mult*Tp

boolean a2d_read_flag=0;    //for debug can take out later
struct ADchannel AN1;
struct ADchannel AN2;
struct BattInfo battery;

#if (AIRFRAME_TYPE == QUAD)
    //Quad board has problem with 3.3V regulator ... it only gives 3.24V
    #define ADC_REF_VOLTAGE 3.24    //ADC Reference voltage in volts
#else
    #define ADC_REF_VOLTAGE 3.31    //ADC Reference voltage in volts
#endif //AIRFRAME_TYPE

// Number of locations for ADC buffer = 6 (AN0,15,16,17,18) x 1 = 6 words
// Align the buffer. This is needed for peripheral indirect mode
//#define  SAMP_BUFF_SIZE	 		8
#define NUM_AD_CHAN 4
int16_t BufferA[NUM_AD_CHAN];
int16_t BufferB[NUM_AD_CHAN];
int16_t sample_count;
uint8_t DmaBuffer = 0;

//TAD 117.6 min
#if 1   // these are the original/legacy values, switch this off to test Mark's new timing constants
#define ALMOST_ENOUGH_SAMPLES   216 // there are 222 or 223 samples in a sum
#define ADCLK_DIV_N_MINUS_1     63  // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*12 = 0.3us (3333.3Khz)
                                    // ADC Conversion Time for 12-bit Tc=14*Tad = 4.2us
#define ADSAMP_TIME_N           11   // No waiting between samples
#else

// dsPIC33FJXXXGPX06A/X08A/X10A
// minimum allowed 12-bit ADC clock period is 118ns or 8.47MHz
// minimum allowed 10-bit ADC clock period is 76ns, or 13.16MHz
// 12 bit mode conversion time is 14 TAD cycles
// total sample/conversion time is (14+SAMC) * TAD
// for 400nsec TAD, total is 10usec for 100KHz conversion rate with SAMC = 11
// *** observed 72usec interval between interrupts on scope => interrupt every sample

// desired adc clock is 625KHz and conversion rate is 25KHz
#if (MIPS == 16)
// ADC_CLK 640KHz
#define ADCLK_DIV_N_MINUS_1 24
// ADC_RATE 25.6KHz
#define ADSAMP_TIME_N 11

#elif (MIPS == 32)
// ADC_CLK 640KHz
#define ADCLK_DIV_N_MINUS_1 49
// ADC_RATE 25.6KHz
#define ADSAMP_TIME_N 11

#elif (MIPS == 40)
// ADC_CLK 625KHz
#define ADCLK_DIV_N_MINUS_1 63
// ADC_RATE 25KHz
#define ADSAMP_TIME_N 11

#elif (MIPS == 64)
// ADC_CLK 1MHz
#define ADCLK_DIV_N_MINUS_1 63
// ADC_RATE 25KHz
#define ADSAMP_TIME_N 26

#endif

// TAD is 1/ADC_CLK
#define ADC_CLK (MIPS / (ADCLK_DIV_N_MINUS_1 + 1))

// At FCY=40MHz, ADC_CLK=625KHz
// At FCY=40MHz, ADC_RATE = 25 KHz
// At 40MHz: 23.148KHz ADC rate and 8 channels seq. sampled, the per channel rate is
// about 2.894 KHz and lp2 3dB point is at 75Hz.
#define ADC_RATE (ADC_CLK / (ADSAMP_TIME_N + 14))

#define ALMOST_ENOUGH_SAMPLES ((ADC_RATE / (NUM_AD_CHAN * HEARTBEAT_HZ)) - 2)

#endif // 0/1

//NOT CURRENTLY WORKING, THE SAMPLING IS CAUSING A RESET. CHECK CONFIGURATION OF BITS AND INPUT PINS
//UNCOMMENT DMA CHEN AFTER DEBUGGING
void init_ADC(void)
{
	sample_count = 0;

	AD1CON1bits.FORM  = 3;      // Data Output Format: Signed Fraction (Q15 format)
	AD1CON1bits.SSRC  = 7;      // Sample Clock Source: Auto-conversion
	AD1CON1bits.ASAM  = 1;      // ADC Sample Control: Sampling begins immediately after conversion
	AD1CON1bits.AD12B = 1;      // 12-bit ADC operation

	AD1CON2bits.CSCNA = 1;      // Scan Input Selections for CH0+ during Sample A bit
	AD1CON2bits.CHPS  = 0;      // Converts CH0

	AD1CON3bits.ADRC = 0;       // ADC Clock is derived from System Clock
	AD1CON3bits.ADCS = ADCLK_DIV_N_MINUS_1;
	AD1CON3bits.SAMC = ADSAMP_TIME_N;

	AD1CON2bits.VCFG = 0;       // use supply as reference voltage

	AD1CON1bits.ADDMABM = 1;    // DMA buffers are built in sequential mode
	AD1CON2bits.SMPI    = (NUM_AD_CHAN-1);
	AD1CON4bits.DMABL   = 0;    // Each buffer contains 1 word
    
    AD1CON4bits.ADDMAEN = 1;    //MW Use DMA Buffer

	AD1CSSL = 0x0000;
	AD1CSSH = 0x0000;
    
    //AD1CHS0bits.CH0SA = 18;

    
	// include voltage monitor inputs
	_CSS19 = 1;                 // Enable AN19 for channel scan
	//_CSS18 = 1;                 // Enable AN18 for channel scan


	_AD1IF = 0;                 // Clear the A/D interrupt flag bit
	_AD1IP = 4;                 // Set the interrupt priority
	_AD1IE = 0;                 // Do Not Enable A/D interrupt
	AD1CON1bits.ADON = 1;       // Turn on the A/D converter

}

void init_DMA5(void)
{
	DMA5CONbits.AMODE = 2;      // Configure DMA for Peripheral indirect mode
	DMA5CONbits.MODE  = 2;      // Configure DMA for Continuous Ping-Pong mode
	DMA5PAD = (int16_t)&ADC1BUF0;
	DMA5CNT = NUM_AD_CHAN-1;
	DMA5REQ = 13;               // Select ADC1 as DMA Request source

	DMA5STAL = __builtin_dmaoffset(BufferA);
	DMA5STBL = __builtin_dmaoffset(BufferB);

	_DMA5IP = 4;     // Set the DMA ISR priority
	_DMA5IF = 0;                // Clear the DMA interrupt flag bit
	_DMA5IE = 1;                // Set the DMA interrupt enable bit

	DMA5CONbits.CHEN = 1;       // Enable DMA
    
}

void __attribute__((__interrupt__,__no_auto_psv__)) _DMA5Interrupt(void)
{
	//int16_t *CurBuffer = (DmaBuffer == 0) ? BufferA : BufferB;

	AN1.input = BufferA[0];
	AN2.input  = BufferB[0];
    
	//DmaBuffer ^= 1;                 // Switch buffers
	IFS3bits.DMA5IF = 0;            // Clear the DMA0 Interrupt Flag

	if (a2d_read_flag == 1)  // prepare for the next reading
	{
		a2d_read_flag = 0;
		AN1.sum = 0;
		AN2.sum = 0;
		sample_count = 0;
	}

	// perform the integration:
	AN1.sum += AN1.input;
	AN2.sum  += AN2.input;


	sample_count++;

	// When there is a chance that data will be read soon,
	// have the new average values ready.
	if (sample_count > ALMOST_ENOUGH_SAMPLES)
	{
		AN1.value = __builtin_divsd(AN1.sum, sample_count);
		AN2.value  = __builtin_divsd(AN2.sum,  sample_count);
        a2d_read_flag = 1;
	}

}

//calculates battery information using ADC and battery type information given in options.h
void calc_battery_life()
{
    battery.safeMinVolts = NUM_LIPO_CELLS*LIPO_CELL_MIN;
    battery.safeMaxVolts = NUM_LIPO_CELLS*LIPO_CELL_MAX;
    
    battery.volts = ((AN1.value + (int32_t)32768) * ADC_REF_VOLTAGE * VOLTAGE_RATIO)/ UINT16_MAX;
    
    battery.millivolts = (uint16_t)(battery.volts*1000);
    
    //battery percentage is between 0 and 100
    //with this calculation, running to 1.0 percent battery is still ok,
    //but don't run down to zero percent
    if(battery.volts <= battery.safeMinVolts)
    {
        battery.remaining = 0;
    }
    else
    {
    battery.remaining = (int8_t)((battery.volts-battery.safeMinVolts) * (100-0) /
            (battery.safeMaxVolts-battery.safeMinVolts) + 0);
    }
    
}
