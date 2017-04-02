/****************************************************************************

 Header File:		i2cxDrv.h

 In this Header file the functions from I2C1 and I2C2 are defined

****************************************************************************/

#ifndef __I2CXDRV_H
#define __I2CXDRV_H


/*** I2C1 addresses definitions of the two Other Modules********************/
//*** I2Cx bus definitions **************************************************
//baud rate of the I2C-Bus in Hz
#define CFG_I2CSPD		100000      // 100KHz
//baud rate generator register for the I2C1(CFG_CPUFRQ/CFG_I2CSPD/2-1)
#define CFG_I2C1BRG		0x188       // (40MHz/100KHz)-2 = 0x188
//this variable to enable the I2Cx module and to release SCLx
#define I2CENSCLREL             0x8000


/****************************************************************************
Function:     initI2C1Drv
Parameters:   none
Return value: none
This function initializes the I2C1 bus and set the I2C1 Bus Speed to 100KHz.
I2C1 used to communicate with the two other slave modules.
****************************************************************************/
void initI2C1Drv ( void );

/****************************************************************************
Function:     I2C1_ReadReg
Parameters:   uint8 Addr1
              uint8 Reg1
Return value: uint8 result1
This function reads and returns the value result1 one byte from the I2C1
devices via the I2C1 bus.
****************************************************************************/
unsigned char I2C1_ReadReg ( unsigned char Addr1, unsigned char Reg1 );

/****************************************************************************
Function:     I2C1_WriteReg
Parameters:   uint8 Addr1
              uint8 Reg1
              uint8 Cmd1
Return value: none
This function writes the Command Cmd1 into the register Reg1 of the I2C1
device with the address Addr1 via the I2C1 bus.
****************************************************************************/
void I2C1_WriteReg ( unsigned char Addr1, unsigned char Reg1, unsigned char Cmd1 );




#ifdef USE_I2C2_DRIVER
/*** I2C2 addresses definitions of the two Other Modules********************/
//*** I2Cx bus definitions **************************************************
//baud rate of the I2C-Bus in Hz
#define CFG_I2CSPD		100000      // 100KHz
//baud rate generator register for the I2C2(CFG_CPUFRQ/CFG_I2CSPD/2-1)
#define CFG_I2C2BRG		0x188       // (40MHz/100KHz)-2 = 0x188
//this variable to enable the I2Cx module and to release SCLx
#define I2CENSCLREL             0x8000


/****************************************************************************
Function:     initI2C1Drv
Parameters:   none
Return value: none
This function initializes the I2C1 bus and set the I2C1 Bus Speed to 100KHz.
I2C1 used to communicate with the two other slave modules.
****************************************************************************/
void initI2C2Drv ( void );

/****************************************************************************
Function:     I2C1_ReadReg
Parameters:   uint8 Addr1
              uint8 Reg1
Return value: uint8 result1
This function reads and returns the value result1 one byte from the I2C1
devices via the I2C1 bus.
****************************************************************************/
unsigned char I2C2_ReadReg ( unsigned char Addr1, unsigned char Reg1 );

/****************************************************************************
Function:     I2C1_WriteReg
Parameters:   uint8 Addr1
              uint8 Reg1
              uint8 Cmd1
Return value: none
This function writes the Command Cmd1 into the register Reg1 of the I2C1
device with the address Addr1 via the I2C1 bus.
****************************************************************************/
void I2C2_WriteReg ( unsigned char Addr1, unsigned char Reg1, unsigned char Cmd1 );
#endif //USE_I2C2_DRIVER

#endif //__I2CXDRV_H
