/****************************************************************************
*
* Name: 			i2cxDrv.c
* Version:			1.00
* Prosessor:                    dsPIC33E
* Compiler:			MPLAB® C30 Beta v3.21
* Header file:                  p33EP512MU810.h
* Linker file:                  p33EP512MU810.gld
*
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Autor                 Date			Comments
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Mohan Aleatbi		10/2011			I2Cx Initialization
* Huajie Wang           10/2012                 I2C Code optimieren
*                                               I2C Read Code
****************************************************************************/
#include <i2c.h>
#include "I2C.h"


/*********** Gleitkommazahl in Bytes  zerlegen ************/
/***************** Float in bytes divide ******************/

/*** Global functions for the I2C1***************************************** */

void initI2C1Drv ( void )
{
         OpenI2C1 ( I2CENSCLREL, CFG_I2C1BRG );
}

unsigned char I2C1_ReadReg ( unsigned char Addr1, unsigned char Reg1 )
{
	unsigned char result1;
	//self explaining
	IdleI2C1 ();
	StartI2C1 ();
	IdleI2C1 ();
	MasterWriteI2C1	( Addr1 );
	IdleI2C1 ();
	MasterWriteI2C1	( Reg1 );
	IdleI2C1 ();
	RestartI2C1 ();
	IdleI2C1 ();
	MasterWriteI2C1	( Addr1|0x01 );
	IdleI2C1 ();
	result1 = MasterReadI2C1 ();
	IdleI2C1 ();
	StopI2C1 ();
	return result1;
}

void I2C1_WriteReg ( unsigned char Addr1, unsigned char Reg1, unsigned char Cmd1 )
{
	//self explaining
	IdleI2C1 ();
	StartI2C1 ();
	IdleI2C1 ();
	MasterWriteI2C1	( Addr1 );
	IdleI2C1 ();
	MasterWriteI2C1	( Reg1 );
	IdleI2C1 ();
	MasterWriteI2C1	( Cmd1 );
	IdleI2C1 ();
	StopI2C1 ();
}

#ifdef USE_I2C2_DRIVER
/*** Global functions for the I2C2***************************************** */

void initI2C2Drv ( void )
{
         OpenI2C2 ( I2CENSCLREL, CFG_I2C2BRG );
}

unsigned char I2C2_ReadReg ( unsigned char Addr1, unsigned char Reg1 )
{
	unsigned char result1;
	//self explaining
	IdleI2C2 ();
	StartI2C2 ();
	IdleI2C2 ();
	MasterWriteI2C2	( Addr1 );
	IdleI2C2 ();
	MasterWriteI2C2	( Reg1 );
	IdleI2C2 ();
	RestartI2C2 ();
	IdleI2C2 ();
	MasterWriteI2C2	( Addr1|0x01 );
	IdleI2C2 ();
	result1 = MasterReadI2C2 ();
	IdleI2C2 ();
	StopI2C2 ();
	return result1;
}

void I2C2_WriteReg ( unsigned char Addr1, unsigned char Reg1, unsigned char Cmd1 )
{
	//self explaining
	IdleI2C2 ();
	StartI2C2 ();
	IdleI2C2 ();
	MasterWriteI2C2	( Addr1 );
	IdleI2C2 ();
	MasterWriteI2C2	( Reg1 );
	IdleI2C2 ();
	MasterWriteI2C2	( Cmd1 );
	IdleI2C2 ();
	StopI2C2 ();
}
#endif //USE_I2C2_DRIVER