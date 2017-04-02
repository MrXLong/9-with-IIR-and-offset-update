/*
 * File:   SPI-SD.h
 * Author: Alejandro
 *
 * Created on 25. August 2015, 14:48
 */

#ifndef SPI_SD_H
#define	SPI_SD_H


// Size of the buffer for received data (in Byte)
#define SPISD_TxBufSize                           1//16
// Size of the buffer for received data (in Byte)
#define SPISD_RxBufSize                           1


typedef struct {
    int res;
    int v1;
    int acmd58;

}SD_DATA;

extern unsigned char SPISDRx[SPISD_RxBufSize];
extern unsigned char spi1RxBuff[SPISD_RxBufSize];       //#define UM7_RxBufSize 28

extern unsigned char SD_dataT[64]; //The size of data block at block addressing mode is fixed to 512 bytes.

void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _DMA9Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _DMA10Interrupt(void);
void cfgSPI1(void);
void cfgDMA9_SPISD_Tx(void);
void cfgDMA10_SPISD_Rx(void);
void initSPISDcommunication(void);
void send_data_spi_sd(int data);
void cfgSPIMD(void);

int SD_CMD0 (void); //GO_IDLE_STATE
int SD_CMD1 (void); //SEND_OP_COND
int SD_CMD8 (void); //SEND_IF_COND
int SD_CMD13 (void); //SEND_STATUS
int SD_CMD41 (void); //SEND_OP_COND
int SD_CMD45 (void);
int SD_CMD58 (void); //READ_OCR

void SD_init(SD_DATA data);

#endif	/* SPI_SD_H */
