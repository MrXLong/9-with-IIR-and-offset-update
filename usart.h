/* 
 * File:   usart.h
 * Author: Matt
 *
 * Created on 25. Juli 2016, 11:54
 */

#ifndef USART_H
#define	USART_H

#ifdef	__cplusplus
extern "C" {
#endif

#define FCY 40000000ULL  
#define BAUDRATE_RS232 115200
#define BRGVAL_RS232   ((FCY/BAUDRATE_RS232)/16)-1
#define rx_buf_size    80
#define tx_buf_size    80
    
#include <LIBPIC30.h>
    
    
    
void testUart(void);
void Delay1Second(void); 
void initUart(void);
void putString(char* str);

    

#ifdef	__cplusplus
}
#endif

#endif	/* USART_H */

