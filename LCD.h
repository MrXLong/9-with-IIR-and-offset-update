/*
 * File:   LCD.h
 * Author: Alejandro
 *
 * Created on 27. August 2015, 23:20
 */

#ifndef LCD_H
#define	LCD_H


#define RS LATFbits.LATF3
#define EN LATFbits.LATF8
#define D4 LATAbits.LATA5
#define D5 LATAbits.LATA14
#define D6 LATAbits.LATA15
#define D7 LATDbits.LATD11

void Lcd_Port(char a);
void Lcd_Cmd(char a);
void Lcd_Clear(void);
void Lcd_Set_Cursor(char a, char b);
void Lcd_Init(void);
void Lcd_Write_Char(char a);
void Lcd_Write_String(char *a);
void Lcd_Shift_Right(void);
void Lcd_Shift_Left(void);
void LCD_conf(void);
void LCD_LongLat(void);
void LCD_Ultra(void);
void LCD_Angles(void);

void dispPWM(void);     //MW 07.12.2015

void dispMotors(void);     //MW 07.12.2015
void dispCompass(void);

#endif	/* LCD_H */

