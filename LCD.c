/*
 * File:   LCD.c
 * Author: Alejandro
 *
 * Created on 27. August 2015, 23:16
 */

#include <stdio.h>
#include <stdlib.h>
#include <p33EP512MU810.h>
#define FCY 40000000ULL
#include <libpic30.h>
#include "LCD.h"
#include "GPS.h"
#include "SRF02.h"
#include "InputCapture.h"
#include "Mixer.h"
#include "UM7.h"


/*
 *
 */
//LCD native Functions Developed by electroSome


void Lcd_Port(char a)
{
	if(a & 1)
		D4 = 1;
	else
		D4 = 0;

	if(a & 2)
		D5 = 1;
	else
		D5 = 0;

	if(a & 4)
		D6 = 1;
	else
		D6 = 0;

	if(a & 8)
		D7 = 1;
	else
		D7 = 0;
}
void Lcd_Cmd(char a)
{
	RS = 0;             // => RS = 0
	Lcd_Port(a);
	EN  = 1;             // => E = 1
        __delay_ms(4);
        EN  = 0;             // => E = 0
}

void Lcd_Clear(void)
{
	Lcd_Cmd(0);
	Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b)
{
	char temp,z,y;
	if(a == 1)
	{
	  temp = 0x80 + b - 1;
		z = temp>>4;
		y = temp & 0x0F;
		Lcd_Cmd(z);
		Lcd_Cmd(y);
	}
	else if(a == 2)
	{
		temp = 0xC0 + b - 1;
		z = temp>>4;
		y = temp & 0x0F;
		Lcd_Cmd(z);
		Lcd_Cmd(y);
	}
}

void Lcd_Init(void)
{
  Lcd_Port(0x00);
   __delay_ms(20);
  Lcd_Cmd(0x03);
	__delay_ms(5);
  Lcd_Cmd(0x03);
	__delay_ms(11);
  Lcd_Cmd(0x03);
  /////////////////////////////////////////////////////
  Lcd_Cmd(0x02);
  Lcd_Cmd(0x02);
  Lcd_Cmd(0x08);
  Lcd_Cmd(0x00);
  Lcd_Cmd(0x0C);
  Lcd_Cmd(0x00);
  Lcd_Cmd(0x06);

}

void Lcd_Write_Char(char a)
{
   char temp,y;
   temp = a&0x0F;
   y = a&0xF0;
   RS = 1;                      // => RS = 1
   Lcd_Port(y>>4);             //Data transfer
   EN = 1;
   __delay_us(40);
   EN = 0;
   Lcd_Port(temp);
   EN = 1;
   __delay_us(40);
   EN = 0;
}

void Lcd_Write_String(char *a)
{
	int i;
	for(i=0;a[i]!='\0';i++)
	   Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right(void)
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left(void)
{
	Lcd_Cmd(0x01);
	Lcd_Cmd(0x08);
}

void LCD_LongLat(void){
    
    //float lg = 10.555;
    char s[16];
    //float lt = GPSdata.latitude;

    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    //sprintf(s,"Long: %f",(double)lg);    //used to write formatted string to a variable
    Lcd_Write_Char(GPScommdata.longitude[0]);
    Lcd_Write_Char(GPScommdata.longitude[1]);
    Lcd_Write_Char(GPScommdata.longitude[2]);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(GPScommdata.longitude[3]);
    Lcd_Write_Char(GPScommdata.longitude[4]);
    Lcd_Write_Char('\'');
    Lcd_Write_Char(GPScommdata.longitude[6]);
    Lcd_Write_Char(GPScommdata.longitude[7]);
    Lcd_Write_Char(GPScommdata.longitude[8]);
    Lcd_Write_Char(GPScommdata.longitude[9]);
    Lcd_Write_Char('"');
    
    
    Lcd_Set_Cursor(2,1);
    sprintf(s,"Long: %f",GPSdata.longitude);
    Lcd_Write_String(s);
    /*
    Lcd_Write_Char(GPScommdata.latitude[0]);
    Lcd_Write_Char(GPScommdata.latitude[1]);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(GPScommdata.latitude[2]);
    Lcd_Write_Char(GPScommdata.latitude[3]);
    Lcd_Write_Char('\'');
    Lcd_Write_Char(GPScommdata.latitude[5]);
    Lcd_Write_Char(GPScommdata.latitude[6]);
    Lcd_Write_Char(GPScommdata.latitude[7]);
    Lcd_Write_Char(GPScommdata.latitude[8]);
    Lcd_Write_Char('"');
     */
}

void dispPWM(void)
{
    char a[6],b[6],c[6],d[6];
    
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    
    sprintf(a, "%d", pulsewidth.throCap);
    Lcd_Write_String(a);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(' ');
    sprintf(c, "%d", pulsewidth.elevCap);
    Lcd_Write_String(c);
    
    Lcd_Set_Cursor(2,1);
    
    sprintf(b, "%d", pulsewidth.ruddCap);
    Lcd_Write_String(b);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(' ');
    sprintf(d, "%d", pulsewidth.aileCap);
    Lcd_Write_String(d);
}


void dispMotors(void)
{
    char a[8],b[8],c[8],d[8];
    
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    
    sprintf(a,"M2:%d%%",(MIX.m2/95));
    Lcd_Write_String(a);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(' ');
    sprintf(b,"M1:%d%%",(MIX.m1/95));
    Lcd_Write_String(b);
    
    Lcd_Set_Cursor(2,1);
 
    sprintf(c,"M3:%d%%",(MIX.m3/95));
    Lcd_Write_String(c);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(' ');
    sprintf(d,"M4:%d%%",(MIX.m4/95));
    Lcd_Write_String(d);

}

void dispCompass(void)
{
    char a[16];
    
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    
    sprintf(a,"%0.2f degrees",(double)calcMagHeading());
    Lcd_Write_String(a);
}

/*
void LCD_LongLat(void){
    
    char s[4],t[4];
    
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_Char(GPScommdata.UTC_Time[0]);
    Lcd_Write_Char(GPScommdata.UTC_Time[1]);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(GPScommdata.UTC_Time[2]);
    Lcd_Write_Char(GPScommdata.UTC_Time[3]);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(GPScommdata.UTC_Time[4]);
    Lcd_Write_Char(GPScommdata.UTC_Time[5]);
    Lcd_Write_Char('.');
    Lcd_Write_Char(GPScommdata.UTC_Time[7]);
    Lcd_Write_Char(GPScommdata.UTC_Time[8]);
    
    Lcd_Set_Cursor(2,1);
    Lcd_Write_Char(GPScommdata.checksum[0]);
    Lcd_Write_Char(GPScommdata.checksum[1]);
    Lcd_Write_Char(GPScommdata.checksum[2]);
    Lcd_Write_Char(' ');
    Lcd_Write_Char(' ');
    
    
    sprintf(s, "%lu", check);
    sprintf(t, "%lu", temp);
    Lcd_Write_String(s);
    Lcd_Write_Char('/');
    Lcd_Write_String(t);
    //Lcd_Write_Char(' ');
    //Lcd_Write_Char(' ');
    //Lcd_Write_Char(checkSum[0]);
    //Lcd_Write_Char(checkSum[1]);
}*/

/*
void LCD_LongLat(void){
    int i = 0;
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    for (i=0;i<GPS_RxBufSize;i++)
        Lcd_Write_Char(GPSRx[i]);
}*/




void LCD_conf(void){

    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("   Quadcopter   ");
    Lcd_Set_Cursor(2,1);
    Lcd_Write_String("       RST      ");
    
}

void LCD_Ultra(void){

    char s[16];
    int us = Sonar_Dist[0];// ptr_FIX->actual_height;//
    //int ac = ptr_FIX->setpoint_height;

    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("Ground-Distance");
    Lcd_Set_Cursor(2,1);
    sprintf(s,"Set:%dcm",us);    //used to write formatted string to a variable
    Lcd_Write_String(s);
    Lcd_Set_Cursor(2,10);
   // sprintf(s,"Ac:%d",ac);
    Lcd_Write_String(s);
}

void LCD_Angles(void){

    char s[16];
    int Roll = (int)ptr_FIX->actual_angle_aile;
    int Pitch = (int)ptr_FIX->actual_angle_elev;
    int Yaw = (int)ptr_FIX->actual_angle_rudd;

     Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String("Roll Pitch  Yaw");
    Lcd_Set_Cursor(2,1);
    sprintf(s,"%d",Roll);    //used to write formatted string to a variable
    Lcd_Write_String(s);
    Lcd_Set_Cursor(2,7);
    sprintf(s,"%d",Pitch);
    Lcd_Write_String(s);
    Lcd_Set_Cursor(2,13);
    sprintf(s,"%d",Yaw);
    Lcd_Write_String(s);
}
