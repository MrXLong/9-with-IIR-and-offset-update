#include "Pins.h"
#include "p33EP512MU810.h"
#include "UM7.h"
#include <stdio.h>
#include "libq.h"
#include <libpic30.h>

// UART Paket-Struktur:
////////////////////////////////////////////////////////////////////////////////////////////
//  |"s"|"n"|"p"|Packet Type (PT)|Address|Data Bytes (D0...Dn-1)|Checksum 1|Checksum 0|   //
////////////////////////////////////////////////////////////////////////////////////////////

// Packet Type (PT) Struktur:
////////////////////////////////////////////////////
// |Has Data|Is Batch|BL3|BL2|BL1|BL0|Hidden|CF|  //
////////////////////////////////////////////////////

//*** Global variables ******************************************************
unsigned char uart2RxBuff[UM7_RxBufSize];       //#define UM7_RxBufSize 28
unsigned char uart2TxBuff[UM7_TxBufSize];       //#define UM7_TxBufSize 7

unsigned char missingUM7Message = 0;            // Variable what saves the number of bytes missing to complete the message
unsigned int messageUM7Full = 0;                // Flag what says if the message starts "0" or is in the middle "1"
unsigned int ZeroGyrosWrong = 0;                // Flag to indicate when the calibrate gyros order was successful
volatile unsigned int incomingUM7Message = 0;   // Flag to indicate if there are a new UM7 message ready to use

float yawError;                                 // Current yaw error
float CalculatingErrorPointer = 0;              // Variable to know how many times the promedium error was calculated
float promediumYawError = 0.03;                 // Yaw promedium error. The IMU sends data 20 times / sec (20 Hz)
                                                // 4 is more or less the expected value
float lastYawMeasure = 0;
int firsTime = 0;                               // First time in yaw calibration
int currentRoll = 0;
int currentPitch = 0;
int previousRoll = 0;
int previousPitch = 0;
int varianceRoll = 0;
int variancePitch = 0;
//***************************************************************************

void cfgDMA3_UART_Rx(void)//Receiving
// DMA3 configuration
// Direction: Read from UART1 Receive Register and write to DMA RAM
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART2 RX
{
	DMA3CON = 0x0002;                       // Continuous, Post-Inc., Periph-RAM
        DMA3CONbits.SIZE = 1;                   // 0 = Word, 1 = Byte
        DMA3CONbits.MODE = 0b00;                // Continuous, PING PONG mode disabled
	DMA3CNT = 5 - 1;
	DMA3REQ = 0x01E;                        // UART2 Receiver

	DMA3PAD = (volatile unsigned int) &U2RXREG;
        DMA3STAL = __builtin_dmaoffset(uart2RxBuff);

	IFS2bits.DMA3IF  = 0;			// Clear DMA interrupt
	IEC2bits.DMA3IE  = 1;			// Enable DMA interrupt
        DMA3CONbits.CHEN = 1;			// Enable Receiving-DMA Channel
}

void cfgDMA4_UART_Tx(void)//Transmitting
// DMA4 configuration
// Direction: Read from DMA RAM and write to UART1 Transmit register
// AMODE: Register Indirect with Post-Increment mode
// IRQ: UART2 TX
{
	DMA4CON = 0x2001;                           // One-Shot, Post-Increment, RAM-to-Peripheral (10000000000001)
        DMA4CONbits.SIZE = 1;                       // 0 = Word, 1 = Byte
	DMA4CNT = UM7_TxBufSize - 1;                // DMA4 Transfer Count Register, Anzahl DMA-Transfers = CNT +1 (7 DMA-Requests)
	DMA4REQ = 0x01F;                            // UART2 Transmitter (11111)
        DMA4REQbits.FORCE = 0;                      // Manual Start

	DMA4PAD = (volatile unsigned int) &U2TXREG; // Pheripal Address Register, siehe Datenblatt DMA, S.42, Example 22-10
	DMA4STAL = __builtin_dmaoffset(uart2TxBuff);// In that way one character is transmiting everytime, Start Address Register
        DMA4STAH = 0x0000;                          // noch hinzugefügt nach Expample 22-10

	IFS2bits.DMA4IF  = 0;                       // Clear DMA interrupt
	IEC2bits.DMA4IE  = 0;                       // Enable DMA interrupt
        DMA4CONbits.CHEN = 1;                       // Enable DMA channel (Transmit)
                                                    //nach Ende des Datenblocks wird das CHEN-Bit automatisch wieder auf 0 gesetzt
}

void cfgUART2(void)                                 // UART2 Konfiguration für RX und TX
{
    	U2MODEbits.LPBACK  = 1;                     // Enable Loop Back Mode (Avoiding transmitting/receiving colitions)
	U2BRG = BRGVAL_UM7;                         // U2BRD = ((FCY/BAUDRATE_UM7)/16)-1
                                                    // U2BRD = ((40000000/57600)/16)-1 = 42,4
	U2MODEbits.PDSEL = 0;                       // No Parity, 8-bit data (U1MODEbits.PDSEL=?)
	U2MODEbits.STSEL = 0;                       // 1-Stop Bit (U1MODEbits.STSEL=?)
        U2MODEbits.ABAUD = 0;                       // Autobaud Disabled
        U2MODE	= 0x8000;
        U2STA	= 0x0000;
	U2STAbits.UTXISEL0 = 0;                     // Interrupt after one Tx character is transmitted
	U2STAbits.UTXISEL1 = 0;
	U2STAbits.URXISEL = 0;                      // Interrupt after one RX character is received

        U2MODEbits.UARTEN = 1;                      // Enable UART2
        U2STAbits.UTXEN = 1;                        // Enable UART2 Tx
}

void initUM7communication(void)
{
    cfgUART2();
    cfgDMA4_UART_Tx();
    cfgDMA3_UART_Rx();
}

void ZeroGyros()                                    // Kalibration der Gyroskope
{
    unsigned int check = 0;
    int i;

    uart2TxBuff[0] = 's';
    uart2TxBuff[1] = 'n';
    uart2TxBuff[2] = 'p';
    uart2TxBuff[3] = 0x00;                          // Kommando senden: gesamtes PT byte = 0 und nur Adresse des Registers senden (nächste Zeile)
    uart2TxBuff[4] = 0xAC;                          // UM7_ZERO_GYROS, UM7 Command Register

    for (i = 0; i < 5; i++)
    {
        check = check + uart2TxBuff[i];
    }
    // Checksum calculation
    // The checksum is computed by summing the each unsigned character in the packet and storing the result in an unsigned 16-bit integer
    uart2TxBuff[5] = (check >> 8) & 0xFF;           // Checksum1 (High)
    uart2TxBuff[6]= check & 0xFF;                   // Checksum2 (Low)

    DMA4CONbits.CHEN = 1;                           // Enable DMA4 channel
    DMA4REQbits.FORCE = 1;                          // Forces a single DMA Transfer (Manual Mode)
}

void setAcc_Ref_Vec (void)
{
    unsigned int check = 0;
    int i;

    uart2TxBuff[0] = 's';
    uart2TxBuff[1] = 'n';
    uart2TxBuff[2] = 'p';
    uart2TxBuff[3] = 0x00;                          // Has Data = 0, Batch = 0 = no data
    uart2TxBuff[4] = 0xAF;                          //UM7_SET_ACCEL_REF, UM7 Command Register
    for (i = 0; i < 5; i++)
    {
        check = check + uart2TxBuff[i];
    }
    // Checksum calculation
    // The checksum is computed by summing the each unsigned character in the packet and storing the result in an unsigned 16-bit integer
    uart2TxBuff[5] = (check >> 8) & 0xFF;           // Checksum1 (High)
    uart2TxBuff[6]= check & 0xFF;                   // Checksum2 (Low)

    DMA4CONbits.CHEN = 1;                           // Enable DMA4 channel
    DMA4REQbits.FORCE = 1;                          // Manual Start
}

void setMag_Ref_Vec (void)
{
    unsigned int check = 0;
    int i;

    uart2TxBuff[0] = 's';
    uart2TxBuff[1] = 'n';
    uart2TxBuff[2] = 'p';
    uart2TxBuff[3] = 0x00;                          // Has Data = 0, Batch = 0 = no data
    uart2TxBuff[4] = 0xB0;                          //UM7_SET_MAG_REF, UM7 Command Register
    for (i = 0; i < 5; i++)
    {
        check = check + uart2TxBuff[i];
    }
    // Checksum calculation
    // The checksum is computed by summing the each unsigned character in the packet and storing the result in an unsigned 16-bit integer
    uart2TxBuff[5] = (check >> 8) & 0xFF;           // Checksum1 (High)
    uart2TxBuff[6]= check & 0xFF;                   // Checksum2 (Low)

    DMA4CONbits.CHEN = 1;                           // Enable DMA4 channel
    DMA4REQbits.FORCE = 1;                          // Manual Start
}

void resetEKF (void)
{
    unsigned int check = 0;
    int i;

    uart2TxBuff[0] = 's';
    uart2TxBuff[1] = 'n';
    uart2TxBuff[2] = 'p';
    uart2TxBuff[3] = 0x00;                          // Has Data = 0, Batch = 0 = no data
    uart2TxBuff[4] = 0xAD;                          //UM7_RESET_EKF, UM7 Command Register
    for (i = 0; i < 5; i++)
    {
        check = check + uart2TxBuff[i];
    }
    // Checksum calculation
    // The checksum is computed by summing the each unsigned character in the packet and storing the result in an unsigned 16-bit integer
    uart2TxBuff[5] = (check >> 8) & 0xFF;           // Checksum1 (High)
    uart2TxBuff[6]= check & 0xFF;                   // Checksum2 (Low)

    DMA4CONbits.CHEN = 1;                           // Enable DMA4 channel
    DMA4REQbits.FORCE = 1;                          // Manual Start
}

// Function to determine the deviation is not (more) currently required
void determineVariance (UM7DataSensor *ptr_eulerAngle)
{
    TMR6 = 0x00;
    while (detect < 41)
    {
    if (detect < 40)                                                        // detect = Timer6 Flag, Wertaufnahme über 1,6 Sekunden und Bildung der durchschnittlichen Abweichung
    {
        storeUM7data(UM7dataSensors);                                       // Eulerwinkel abspeichern
        currentRoll = ptr_eulerAngle->eulerRollAngle;
        varianceRoll = currentRoll - previousRoll;
        varianceRoll += varianceRoll;
        previousRoll = currentRoll;

        storeUM7data(UM7dataSensors);                                       // Eulerwinkel abspeichern
        currentPitch = ptr_eulerAngle->eulerPitchAngle;
        variancePitch = currentPitch - previousPitch;
        variancePitch += variancePitch;
        previousPitch = currentPitch;
    }
    if (detect == 40)
    {
        storeUM7data(UM7dataSensors);                                       // Eulerwinkel abspeichern
        currentRoll = ptr_eulerAngle->eulerRollAngle;
        varianceRoll = currentRoll - previousRoll;
        varianceRoll += varianceRoll;
        ptr_eulerAngle->varianceRoll = _Q16ftoi(varianceRoll/400*0.733);    // durchschnittliche Roll-Abweichung nach 10ms

        storeUM7data(UM7dataSensors);                                       // Eulerwinkel abspeichern
        currentPitch = ptr_eulerAngle->eulerPitchAngle;
        variancePitch = currentPitch - previousPitch;
        variancePitch += variancePitch;
        ptr_eulerAngle->variancePitch = _Q16ftoi(variancePitch/400*0.733);  // durchschnittliche Nick-Abweichung nach 10ms
    }
    }
}

void storeUM7data(UM7_Commun data)
{
    unsigned int check;
    unsigned char checksum1;
    unsigned char checksum0;
    unsigned int finish;

    //Checking the checksum
    check = 's' + 'n' + 'p' + data.PacketType + data.Address;
    if ((data.PacketType & 0b10000000) != 0)                        // Bit 7 in packet type (PT) = 1 = has data
    {
        if ((data.PacketType & 0b01000000) != 0)                    // Bit 6 in PT = 1 = is batch
        {
            finish = data.PacketType & 0b00111100;                  // Maske, um die Batch Length (Anzahl der Register im Batch) herauszufinden
                                                                    // hier: 0101 = 5 (Batch Length)
            finish = 4 * (missingUM7Message >> 2);                  // 4 * Batch Length = Datenlänge = 20 Bytes
        }
        else
        {
            finish = 4;                                             // 4 bytes, 4*8 = 32bit
        }
    }
    else
    {
        finish = 0;                                                 // Only missing the checksum, d.h. keine Daten vorhanden
    }

    int i;

    // aus der ermittelten Datenlänge wird die checksum aus den Daten ermittelt und mit der checksum von s.n.p,PT,address verrechnet,
    // um endgültige checksum (2*8 Bytes) zu erhalten
    for (i = 0; i < finish; i++)
    {
        check = check + data.DataBytes.Byte[i];
    }
    // If the checksum is correct, storage is executed
    checksum1 = (check >> 8) & 0xFF;                                        // speichert linken Teil (Bit 8-15), checksum1
    checksum0 = check & 0xFF;                                               // speichert rechten Teil (Bit 0-7), checksum0

    if ((data.Checksum1 == checksum1) && (data.Checksum0 == checksum0))     //wenn Schecksum 1 und Checksum 0 stimmen
    {
        switch (data.Address)                                               //Abfrage der Adresse der gesendeten Daten (S. 43 Datenblatt)
        {
            // Sensor sendet nicht nur die Daten, zu deren Register die Adresse angegeben ist, sondern auch noch die Register hinter dem angeforderten Register
            // also: Roll, Pitch, Yaw, Roll rate, Pitch rate, Yaw rate, Euler time (Register 0x70 bis 0x74)
            case 0x70:                                                      // Adresse Euler Winkel (Roll, Pitch), mit allen Eulerwinkel-Registern im Anschluss (s.Zeile 290 Speicherinhalt)
            {
                UM7_Sensors.eulerRollAngle  = offset_roll_angle -(data.DataBytes.data16[9]);     //91 entspricht 1° Offset nach rechts
                UM7_Sensors.eulerPitchAngle = -(data.DataBytes.data16[8]);
                //UM7_Sensors.eulerYawAngle = data.DataBytes.data16[7];
                UM7_Sensors.eulerYawRate    = -offset_yaw_rate-(data.DataBytes.data16[3]);
            } break;

            /* Speicherinhalt:
             * data.DataBytes.data16[x]:
             * [9] = roll
             * [8] = pitch
             * [7] = yaw
             * [6] = /
             * [5] = roll rate
             * [4] = pitch rate
             * [3] = yaw rate
             * [2] = /
             * [1] = Euler time
             * [0] = Euler time
             */
        }
    }
}

void __attribute__((interrupt, auto_psv)) _DMA3Interrupt(void)              //RECEIVING data
{
    // The operationl time spends by the interrupt is 1.6us

    if (messageUM7Full == 0)                                                            // Begining of the message
    {
        if (uart2RxBuff[0] == 's' && uart2RxBuff[1] == 'n' && uart2RxBuff[2] == 'p')    // Überprüfe, ob s, n, p vorhanden sind
        {
            UM7dataSensors.PacketType = uart2RxBuff[3];
            UM7dataSensors.Address = uart2RxBuff[4];

            if ((UM7dataSensors.PacketType & 0b10000000) != 0)                          // Überprüfe, ob Daten enthalten sind (Has Data = 1)
            {
                if ((UM7dataSensors.PacketType & 0b01000000) != 0)                      // Überprüfe, ob Batch-Operation (Batch = 1)
                {
                    missingUM7Message = UM7dataSensors.PacketType & 0b00111100;         // Auslesen der Batch-Länge (=Anzahl der enthaltenen Register)
                    missingUM7Message = (4 * (missingUM7Message >> 2)) + 2;             // 4 * Batch Length + checksum (checksum = 16 bit = 2 bytes)
                                                                                        // restliche Paketlänge (ohne s,n,p,PT,Address)
                }
                else
                {
                    missingUM7Message = 4 + 2;// 4 bytes + checksum                     // wenn kein Batch, sind nur 4 Bytes Daten vorhanden, plus die Checksum
                }
            }
            else                                                                        // Keine Daten vorhanden
            {
                missingUM7Message = 2;// Only missing the checksum
            }

            DMA3CNT = missingUM7Message - 1;// Resizing the DMA3 buffer.                // wichtige Zeile!!
                                                                                        // 22 DMA-Transfers (je 1 Byte) müssen übertragen werden,
                                                                                        // um einen (erneuten) Datenblock-Transfer als abgeschlossen zu betrachten
                                                                                        // d.h. uart2RxBuff fängt wieder bei Element [0] an, Sensordaten zu sammeln
                                                                                        // 20 Byte Sensordaten + 2 Byte checksum im Falle des UM7, 8 + 2 Byte beim UM6
                                                                                        // initialisiert wird DMA3CNT mit 5, was einem Datenblock mit 5 Byte entspricht (s,n,p,PT,address)

            messageUM7Full = 1;                                                         // Paket-Teil mit s.n.p,PT,address als eigenständiges Paket interpretiert und verarbeitet
                                                                                        // im nächsten Interrupt-Durchlauf wird der zweite Paket-Teil (mit den eigentlichen Sensorwerten)
                                                                                        // ebenfalls als eigenständiges Paket verarbeitet und gespeichert
                                                                                        // uart2RxBuff[] wird wieder zurückgesetzt, beim nächsten Interrupt-Durchlauf wird wieder ab [0]
                                                                                        // in uart2RxBuff[] geschrieben!
        }
    }
    else                                                                                // wenn Nachricht abgearbeitet
    {
        int i;
        // für die Länge der Daten - Länge checksum = 20 Durchläufe, 0 bis 19, also 20 Byte reine Daten auslesen
        for (i = 0; i < (missingUM7Message - 2); i++)                                   
        {
            UM7dataSensors.DataBytes.Byte[(missingUM7Message - 3) - i] = uart2RxBuff[i];// Byte 19 ist erstes Daten-Byte, Byte 0 letztes Daten-Byte
        }
        // Auslesen der letzten beiden Bytes (checksum-Bytes)
        UM7dataSensors.Checksum1 = uart2RxBuff[missingUM7Message - 2];                  // Checksum1, vorletzte Position = Byte 20 (da Buff bei [0] beginnt, ist die eigentliche letzte Position 21 (0 bis 21)))
        UM7dataSensors.Checksum0 = uart2RxBuff[missingUM7Message - 1];                  // Checksum2, letzte Position = Byte 21

        messageUM7Full = 0;                                                             // message finished, nächster Interrupt wird wieder s,n,p,PT,address erfasst
        DMA3CNT = 5 - 1;                                                                // Hier wird die Datenpaket-Größe wieder auf 5 reduziert, um im nächsten Durchlauf
                                                                                        // wieder nur s,n,p,PT,address in einem Paket übertragen zu bekommen
                                                                                        // anschließened wird der Buffer wieder auf die Größe der Daten (20 Bytes) erweitert, um
                                                                                        // nur die reinen Sensordaten als ein geschlossenes Paket übertragen zu bekommen
                                                                                        // plus die beiden checksum-Bytes

        incomingUM7Message = 1;                                                         // nach dem zweiten Interrupt-Durchlauf stehen nun die eigentlichen Sensordaten zur Verfügung
    }

    IFS2bits.DMA3IF = 0;                                                                // Clear the DMA3 Interrupt Flag;
}

void __attribute__((interrupt, no_auto_psv)) _DMA4Interrupt(void)                       //Transmiting data
{
    IFS2bits.DMA4IF = 0;                                                                // Clear the DMA4 Interrupt Flag
}