
#include "UART.h"

char uart1RxBuff[RS232_RxBufSize];
char uart1TxBuff[RS232_TxBufSize];

unsigned int messageFull = 0;// Flag what says if the message starts "0" or is in the middle "1"
unsigned int missingMessage = 0;// Variable what saves the number of bytes missing to complete the message
unsigned int incomingMessage = 0;// Flag to indicate if there are a new message ready to use
int positionDMA2TxBuffer = 0;// Pointer to indicate the position to write in the DMA2 buffer

Message MessageReceived;// Message received from the RS232
Message queue[SizeQueue];// Incoming message ready to store or execute in real time
Message messageToTransmit;// Message to send via RS232

unsigned int messagePosition = 0;// Pointer to indicate the position where the missing message begins
unsigned int wrongMessage = 0;// Variable that indicates if the data received are good or not
// 0 means good, 1 means for another customer, 2 means undefined error
unsigned int messageSize = 0;
unsigned int bufferAorB = 0;// 0 means bufferA, 1 means bufferB
//DATARS232[6] array of the maximum bytes that would be transmitted to the
//slave modules in one block
unsigned char DATARS232[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//len is the size of the data array that would be transfered to the slavef
//modules   2<len<7
unsigned char len=0;

void cfgDMA1_UART_Rx(void)//Receiving
// DMA1 configuration
// Direction: Read from UART1 Receive Register and write to DMA RAM
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART1 RX
{
	DMA1CON = 0x0002;// Continuous, Post-Inc., Periph-RAM
        DMA1CONbits.SIZE = 1;// 0 = Word, 1 = Byte
        DMA1CONbits.MODE = 0b00;// Continuous, PING PONG mode disabled
	DMA1CNT = 4 - 1;// Interrupt when the PayloadSize arrives
	DMA1REQ = 0x00B;// UART1 Receiver

	DMA1PAD = (volatile unsigned int) &U1RXREG;//Values to read from RS-232 (UART1)
        DMA1STAL = __builtin_dmaoffset(uart1RxBuff);

	IFS0bits.DMA1IF  = 0;			// Clear DMA interrupt
	IEC0bits.DMA1IE  = 1;			// Enable DMA interrupt 1 **************************************************************************
        DMA1CONbits.CHEN = 1;			// Enable DMA Channel
}

void cfgDMA2_UART_Tx(void)
// DMA2 configuration
// Direction: Read from DMA RAM and write to UART1 Transmit register
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART1 TX
{

	DMA2CON = 0x2001;// One-Shot, Post-Increment, RAM-to-Peripheral
        DMA2CONbits.SIZE = 1;// 0 = Word, 1 = Byte
	DMA2CNT = 7 - 1;// DMA requests, 7 bytes is the shorter message to send
	DMA2REQ = 0x00C;// UART1 Transmitter
        DMA2REQbits.FORCE = 1;// Manual Start

	DMA2PAD = (volatile unsigned int) &U1TXREG;// Values to write to RS-232 (UART)
	DMA2STAL = __builtin_dmaoffset(uart1TxBuff);// In that way one character is transmiting everytime

	IFS1bits.DMA2IF  = 0;			// Clear DMA interrupt
	IEC1bits.DMA2IE  = 1;			// Enable DMA interrupt
        DMA2CONbits.CHEN = 0;                   // Disable DMA channel

}

// UART1 Configuration
void cfgUART1(void)
{


// Disable Loop Back Mod
// When LPBACK is enable UART1 is sending data, the R1RX is disabled avoiding colitions
    	U1MODEbits.LPBACK  = 1;

// ?	Configure U1BRG register for 9600 bit rate, note Fcy=40Mhz
//  	U1BRD=((FCY/BAUDRATE)/16)-1
		U1BRG = BRGVAL;

// Configure U1MODE register to the following
// ?	No Parity and 8-bit data (U1MODEbits.PDSEL=?)
// ?	1-Stop Bit (U1MODEbits.STSEL=?)
// ?    Autobaud Disabled
		U1MODEbits.PDSEL = 0;
		U1MODEbits.STSEL = 0;
                U1MODEbits.ABAUD = 0;
                U1MODE	= 0x8001;      // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                U1STA	= 0x0000;

// Configure U1STA register to the following
// ?	Interrupt after one Tx character is transmitted (U1STAbits.UTXISEL=?)
// ?	Interrupt after one RX character is received (U1STAbits.URXISEL=?)
// ?	Enable UART Transmit Module (U2STAbits.UTXEN=?)
		U1STAbits.UTXISEL0 = 0;
		U1STAbits.UTXISEL1 = 0;
		U1STAbits.URXISEL = 0;

        U1MODEbits.UARTEN = 1; // Enable UART
        U1STAbits.UTXEN = 1; // Enable UART Tx

}

void initRS232()
{
    cfgDMA2_UART_Tx();
    cfgDMA1_UART_Rx();
    cfgUART1();
/*
        // Send covariance matrix values 0 - 1
        messageToTransmit.PayloadSize = 2 + 8;
        messageToTransmit.PacketType = 0xA9;
        messageToTransmit.Payload.Byte[0] = 1;// HH
        messageToTransmit.Payload.Byte[1] = 0;// HL
        messageToTransmit.Payload.Byte[2] = 0;// LH
        messageToTransmit.Payload.Byte[3] = 0;// LL
        messageToTransmit.Payload.Byte[4] = 0;// HH
        messageToTransmit.Payload.Byte[5] = 0;// HL
        messageToTransmit.Payload.Byte[6] = 0;// LH
        messageToTransmit.Payload.Byte[7] = 0;// LL
        //sendDataToRS232_DMA();
 * */
}

void sendDataToRS232_DMA()// Sending data over RS232 to de Raspberry Pi
{
    messageToTransmit.Payload.Byte[0] = 1;
    messageToTransmit.Payload.Byte[1] = 2;
    messageToTransmit.Payload.Byte[2] = 3;
    messageToTransmit.Payload.Byte[3] = 0;
    messageToTransmit.Payload.Byte[4] = 0;
    messageToTransmit.PayloadSize = 7;
    int Check;
    uart1TxBuff[positionDMA2TxBuffer] = ReportHeader;
    uart1TxBuff[positionDMA2TxBuffer + 1] = ReportHeader;
    uart1TxBuff[positionDMA2TxBuffer + 2] = IdRaspberryPi;
    uart1TxBuff[positionDMA2TxBuffer + 3] = messageToTransmit.PayloadSize;
    uart1TxBuff[positionDMA2TxBuffer + 4] = TrackMerlinIIIID;
    uart1TxBuff[positionDMA2TxBuffer + 5] = messageToTransmit.PacketType;

    positionDMA2TxBuffer = positionDMA2TxBuffer + 6;
    int i;
    int h = positionDMA2TxBuffer;
    for (i = 0; i < (messageToTransmit.PayloadSize - 2); i++)
    {
        uart1TxBuff[i + h] = messageToTransmit.Payload.Byte[i];
        positionDMA2TxBuffer++;
    }

    Check = TrackMerlinIIIID + messageToTransmit.PacketType;
    for (i = 0; i < (messageToTransmit.PayloadSize - 2); i++)
    {
        Check = Check + messageToTransmit.Payload.Byte[i];
    }

    Check = 256 - (Check %256);// Modulo 256 checksum
    uart1TxBuff[positionDMA2TxBuffer] = Check;
    positionDMA2TxBuffer++;
    DMA2CNT = positionDMA2TxBuffer - 1;// DMA requests

}

void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void)//Receiving data
{
    // The operationl time spends by the interrupt is 726ns

    if (messageFull == 0)// Begining of the message
    {
        MessageReceived.Header[0]= uart1RxBuff[0];
        MessageReceived.Header[1]= uart1RxBuff[1];
        MessageReceived.DestinationId = uart1RxBuff[2];
        MessageReceived.PayloadSize = uart1RxBuff[3];

        missingMessage = MessageReceived.PayloadSize + 1;// + 1 is the Checksum
        // Resizing the DMA interrupt to save the missing message
        DMA1CNT = missingMessage - 1;// Resizing the DMA buffer.
        messageFull = 1;// Message complete
    }

    else// Missing message
    {
        MessageReceived.SourceID = uart1RxBuff[0];
        MessageReceived.PacketType = uart1RxBuff[1];

        missingMessage = missingMessage - 2;
        int i;
        for (i = 0; i < (missingMessage - 1); i++)
        {
            MessageReceived.Payload.Byte[i] = uart1RxBuff[missingMessage - i];
        }
        MessageReceived.Checksum = uart1RxBuff[missingMessage + 1];

        messageFull = 0;// message finished
        DMA1CNT = 4 - 1;// Resizing the DMA buffer
        incomingMessage = 1;// Message ready to read
    }


   IFS0bits.DMA1IF = 0;// Clear the DMA1 Interrupt Flag
}

void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)//Transmiting data
{
    // The operationl time spends by the interrupt is 151ns
    //positionDMA2TxBuffer = 0;
    IFS1bits.DMA2IF = 0;// Clear the DMA2 Interrupt Flag;
}
