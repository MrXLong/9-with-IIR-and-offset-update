/* 
 * File:   Pins.h
 * Author: Simon
 *
 * Created on December 18, 2014, 1:12 PM
 */

#ifndef PINS_H
#define	PINS_H

#define FCY 40000000ULL
//#define LATGbits.LATG2 (LATG |=0x0004)
//#define LATGbits.LATG3 (LATG |=0x0008)

//Defines are for SD_SPI.c 
#define MEDIA_SOFT_DETECT

#define SD_WE               0


#define SD_CS               LATFbits.LATF1
#define SD_CS_TRIS          TRISFbits.TRISF1
//            #define SD_CS_ANSEL         ANSELDbits.ANSD2 // no such bit on device

//            #define SD_CD               PORTAbits.RA12
//            #define SD_CD_TRIS          TRISAbits.TRISA12
//            #define SD_WE               PORTGbits.RG1
//            #define SD_WE_TRIS          TRISGbits.TRISG1

#define SPICON1             SPI1CON1
#define SPISTAT             SPI1STAT
#define SPIBUF              SPI1BUF
#define SPISTAT_RBF         SPI1STATbits.SPIRBF
#define SPICON1bits         SPI1CON1bits
#define SPISTATbits         SPI1STATbits
#define SPIENABLE           SPI1STATbits.SPIEN
#define SPIBRG              SPI1BRG


// Description: SD-SPI Analog/Digital Select ANSEL bit
#define SD_SCK_ANSEL	0
#define SD_SDI_ANSEL	0
#define SD_SDO_ANSEL	0
#define SD_CS_ANSEL     0

// Tris pins for SCK/SDI/SDO lines
// Description: The TRIS bit for the SCK pin
#define SPICLOCK            TRISGbits.TRISG0
// Description: The TRIS bit for the SDI pin
#define SPIIN               TRISGbits.TRISG1
// Description: The TRIS bit for the SDO pin
#define SPIOUT              TRISAbits.TRISA6

    //SPI-SD
//    TRISFbits.TRISF1 = 0;// Card Detect/ Data Line[bit3]
//    TRISGbits.TRISG1 = 0;// Command/Response
//    TRISGbits.TRISG0 = 1;// Clock
//    TRISAbits.TRISA6 = 0;// Data Line[bit0]

void init_Pins (void);
void remap_Pins (void);

#endif	/* PINS_H */
