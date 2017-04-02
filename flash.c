// This file is part of MatrixPilot.
//
//    http://code.google.com/p/gentlenav/
//
// Copyright 2009-2011 MatrixPilot Team
// See the AUTHORS.TXT file for a list of authors of MatrixPilot.
//
// MatrixPilot is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// MatrixPilot is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with MatrixPilot.  If not, see <http://www.gnu.org/licenses/>.


#include <p33EP512MU810.h>

//#include "options_file_system.h"


#ifdef USE_AUX_FLASH

#include <stdint.h>
#include "flash.h"
#include <spi.h>
#include <stdio.h>
#include <string.h>
#include "data_storage.h"

#define BYTE_PAGE_SIZE 2048  //page size in bytes for dspic33
#define BYTE_ROW_SIZE  256  //row size in bytes for dspic33

uint32_t auxFlashStartAddr = 0x7FC000;  //8372224
uint32_t auxFlashEndAddr = 0x7FFFF8;    //8388600
//difference of 16376

void init_dataflash(void)
{
    NVMCONbits.WREN = 1;
}

//unsigned int NVMUnlock (unsigned int nvmop)
//{
//  unsigned int status;
//  // Suspend or Disable all Interrupts
//  //asm volatile (?di %0? : ?=r? (status));
//  status = DI();
//  //DisableInterrupts();
//  // Enable Flash Write/Erase Operations (WREN bit) and Select
//  // Flash operation to perform
//  NVMCON = nvmop;
//  // Write Keys
//  NVMKEY = 0xAA996655;
//  NVMKEY = 0x556699AA;
//  // Start the operation using the Set Register (WR bit)
//  NVMCONSET = 0x8000;
//  // Wait for operation to complete
//  while (NVMCON & 0x8000);
//  // Restore Interrupts
//  if (status & 0x00000001)
//      asm ei;
//  //EnableInterrupts();
//  else
//      asm di;
//  //DisableInterrupts();
//  // Disable NVM write enable
//  NVMCONCLR = 0x0004000;
//  // Return WRERR and LVDERR Error Status Bits
//  return (NVMCON & 0x3800);
//}

//numBytes should be what size?
void UserFlashPageReadBytes(uint16_t userFlashPage, const uint16_t addr, uint8_t *buff, uint16_t numBytes)
{ 
   // Read numBytes from user page into buff. 
   memcpy((void *)buff, (void *)(userFlashPage + addr), numBytes); 
} 
  
void UserFlashPageWriteBytes(uint16_t userFlashPage, const uint16_t addr, uint8_t *data, uint16_t numBytes) 
{ 
   uint8_t pageBuff[BYTE_PAGE_SIZE]; //page size in bytes
   uint16_t numBytesLeft = numBytes; 
   uint16_t numBytesToWriteToRow; 
   uint16_t rowIndexInPageBuff; 
   
   if (addr + numBytes > sizeof(pageBuff)) //if data is more than one page
      return; 
  
   // Read entire 4096 byte page (sizeof(pageBuff)) bytes into pageBuff. 
   memcpy((void *)pageBuff, (void *)userFlashPage, sizeof(pageBuff)); 
  
   // Copy the range of source data bytes into pageBuff 
   uint16_t i; 
   for (i = addr; i < (addr + numBytes); i++) 
   { 
      pageBuff[i] = *data++; 
   } 
  
    // Erase one page (2048 Bytes) of User Program Flash 
    NVMErasePage((void *)userFlashPage); 
  
   // Now program the page with the new data in pageBuff, row by row. 
   rowIndexInPageBuff = 0; 
   while (numBytesLeft) 
   { 
      if (numBytesLeft > BYTE_ROW_SIZE) 
         numBytesToWriteToRow = BYTE_ROW_SIZE; 
      else 
         numBytesToWriteToRow = numBytesLeft; 
      
      // Write 256 bytes (one NVM row) from pageBuff. Writes a whole row always. 
      NVMWriteRow((void *)(userFlashPage + rowIndexInPageBuff), (void *)pageBuff); 
      numBytesLeft       -= numBytesToWriteToRow; 
      rowIndexInPageBuff += numBytesToWriteToRow; 
   } 
} 

void NVMWriteRow(uint16_t *rowAddr, uint8_t *pageBuff)
{
    NVMADRU = (uint8_t)(*rowAddr >> 7);
    NVMADR = (uint16_t)(*rowAddr << 1);
    
    NVMADR = *pageBuff;
    NVMCONbits.NVMOP = 0b0010; //Memory row write operation
    NVMCONbits.WR = 1;
}

void NVMErasePage(uint16_t *pageAddr)
{
    NVMADRU = (uint8_t)(*pageAddr >> 7);
    NVMADR = (uint16_t)(*pageAddr << 1);
    NVMCONbits.NVMOP = 0b0011; //Memory page erase operation
    NVMCONbits.WR = 1;
}

#endif // USE_AUX_FLASH

