
#include "p33EP512MU810.h"
#include "serial.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libpic30.h>

//*** Global variables ******************************************************
 char uart3RxBuff[RxBufSize];
 char uart3TxBuff[TxBufSize];


#define SERIAL_BUFFER_SIZE 256
char serial_buffer[SERIAL_BUFFER_SIZE+1];
int16_t sb_index = 0;
int16_t end_index = 0;
char serial_interrupt_stopped = 1;


void __attribute__((__interrupt__, __no_auto_psv__)) _U3TXInterrupt(void)
{
	_U3TXIF = 0; // clear the interrupt

	int16_t txchar = udb_serial_callback_get_byte_to_send();
	if (txchar != -1)
	{
		U3TXREG = (uint8_t)txchar;
	}
}

void __attribute__((__interrupt__, __no_auto_psv__)) _U3RXInterrupt(void)
{
	_U3RXIF = 0; // clear the interrupt

	while (U3STAbits.URXDA) //char still available
	{
		uint8_t rxchar = U3RXREG;   //get char
		mav_msg_receive(rxchar);
	}
	U3STAbits.OERR = 0;
}

void cfgUART3(void)
{
    U3BRG = BRGVAL;
	// configure U2MODE
	U3MODEbits.UARTEN = 0;      // Bit15 TX, RX DISABLED, ENABLE at end of func
	//                          // Bit14
	U3MODEbits.USIDL = 0;       // Bit13 Continue in Idle
	U3MODEbits.IREN = 0;        // Bit12 No IR translation
	U3MODEbits.RTSMD = 0;       // Bit11 Simplex Mode
	//                          // Bit10
	U3MODEbits.UEN = 0;         // Bits8,9 TX,RX enabled, CTS,RTS not
	U3MODEbits.WAKE = 0;        // Bit7 No Wake up (since we don't sleep here)
	U3MODEbits.LPBACK = 0;      // Bit6 No Loop Back
	U3MODEbits.ABAUD = 0;       // Bit5 No Autobaud (would require sending '55')
	U3MODEbits.URXINV = 0;      // Bit4 IdleState = 1  (for dsPIC)
	//U3MODEbits.BRGH = 1;        // Bit3 4 clocks per bit period
	U3MODEbits.PDSEL = 0;       // Bits1,2 8bit, No Parity
	U3MODEbits.STSEL = 0;       // Bit0 One Stop Bit

	// Load all values in for U2STA SFR
	U3STAbits.UTXISEL1 = 0;     //Bit15 Int when Char is transferred (1/2 config!)
	U3STAbits.UTXINV = 0;       //Bit14 N/A, IRDA config
	U3STAbits.UTXISEL0 = 1;     //Bit13 Other half of Bit15
	//                          //Bit12
	U3STAbits.UTXBRK = 0;       //Bit11 Disabled
	//U2STAbits.UTXEN = 1;        //Bit10 TX pins controlled by periph (handled below)
	//U2STAbits.UTXBF = 0;        //Bit9 *Read Only Bit*
	//U2STAbits.TRMT = 0;         //Bit8 *Read Only bit*
	U3STAbits.URXISEL = 0;      //Bits6,7 Int. on character recieved
	U3STAbits.ADDEN = 0;        //Bit5 Address Detect Disabled
	//U2STAbits.RIDLE = 0;        //Bit4 *Read Only Bit*
	//U2STAbits.PERR = 0;         //Bit3 *Read Only Bit*
	//U2STAbits.FERR = 0;         //Bit2 *Read Only Bit*
	U3STAbits.OERR = 0;         //Bit1 *Read Only Bit*
	//U2STAbits.URXDA = 0;        //Bit0 *Read Only Bit*

	//_U3TXIP = INT_PRI_U3TX;     // Mid Range Interrupt Priority level, no urgent reason
	//_U3RXIP = INT_PRI_U3RX;     // Mid Range Interrupt Priority level, no urgent reason

	_U3TXIF = 0;                // Clear the Transmit Interrupt Flag
	_U3TXIE = 1;                // Enable Transmit Interrupts
	_U3RXIF = 0;                // Clear the Receive Interrupt Flag
	_U3RXIE = 1;                // Enable Receive Interrupts

	U3MODEbits.UARTEN = 1;      // And turn the peripheral on
	U3STAbits.UTXEN = 1;
}


void initSerial()
{
    cfgUART3();
}


void echoString(char* str)
{
    while(U3STAbits.URXDA); //wait while data is receiving
    while(!U3STAbits.TRMT);

    sprintf(uart3TxBuff, "\r\n%s", str);
    //strcpy(uart3TxBuff, str);
    DMA1CNT = strlen(uart3TxBuff)-1;
    /* Load transmit buffer and transmit the same till null character is encountered */
    force_send();
}

void force_send()
{
    DMA1CONbits.CHEN = 1;
    DMA1REQbits.FORCE = 1;  
}


int16_t udb_serial_callback_get_byte_to_send(void)
{
	if (sb_index < end_index && sb_index < SERIAL_BUFFER_SIZE) // ensure never end up racing thru memory.
	{
		uint8_t txchar = serial_buffer[sb_index++];
		return txchar;
	}
	else
	{
		serial_interrupt_stopped = 1;
	}
	return -1;
}

int16_t mavlink_serial_send(const uint8_t buf[], uint16_t len) // RobD
{
	int16_t start_index;
	int16_t remaining;

	// Note at the moment, all channels lead to the one serial port
	if (serial_interrupt_stopped == 1)
	{
		sb_index = 0;
		end_index = 0;
	}
	start_index = end_index;
	remaining = SERIAL_BUFFER_SIZE - start_index;


	if (len > remaining)
	{
		// Chuck away the entire packet, as sending partial packet
		// will break MAVLink CRC checks, and so receiver will throw it away anyway.
		return (-1);
	}
	if (remaining > 1)
	{
		memcpy(&serial_buffer[start_index], buf, len);
		end_index = start_index + len;
	}
	if (serial_interrupt_stopped == 1)
	{
		serial_interrupt_stopped = 0;
        //DMA1CNT = len-1;
		udb_serial_start_sending_data(); //fire flag
	}
	return (1);
}

void mp_mavlink_transmit(uint8_t ch)
// routine to send a single character used by MAVlink standard include routines.
// We forward to multi-byte sending routine so that firmware can interleave
// ascii debug messages with MAVLink binary messages without them overwriting the buffer.
{
//printf("mp_mavlink_transmit(%u)\r\n", ch);
	mavlink_serial_send(&ch, 1);
}

void udb_serial_start_sending_data(void)
{
    _U3TXIF = 1;
}

