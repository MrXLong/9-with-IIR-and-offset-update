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


#ifndef FLASH_H
#define FLASH_H

#include "defines.h"
//#define USE_AT45D_DMA   

void init_dataflash(void);
void NVMErasePage(uint16_t *pageAddr);
void NVMWriteRow(uint16_t *rowAddr, uint8_t *pageBuff);

//unsigned int NVMUnlock (unsigned int nvmop);
//unsigned int NVMWriteWord (void* address, unsigned int dat);
//unsigned int NVMWriteRow (void* address, void* dat);
//unsigned int NVMErasePage(void* address);
//unsigned int NVMErasePFM(void);

// code from microchip application note 61121E
//----------------------------------------------
/*
Run-Time Self-Programming (RTSP) allows the user code to modifyFlash program memory
contents. The device Flash memory is divided into two logical Flash partitions: the program Flash
memory (PFM), and the boot Flash memory (BFM). The last page in boot Flash memory contains
the debug page, which is reserved for use by the debugger tool while debugging.
The program Flash array for the PIC32 device is builtup of a series of rows. A row contains 128
32-bit instruction words or 512 bytes. A group of 8 rows compose a page; which, therefore,
contains 8 × 512 = 4096 bytes or 1024 instruction words. A page of Flash is the smallest unit of
memory that can be erased at a single time. The program Flash array can be programmed in one
of two ways:
? Row programming, with 128 instruction words at a time
? Word programming, with 1 instruction word at a time
Performing an RTSP operation while executing (fetching) instructions from program Flash
memory, the CPU stalls (waits) until the programming operation is finished. The CPU will not
execute any instruction, or respond to interrupts, during this time. If any interrupts occur during
the programming cycle, they remainpending until the cycle completes.
Performing an RTSP operation while executing (fetching) instructions from RAM memory, the
CPU can continue to execute instructions and respond to interrupts during the programming
operation. Note, any executable code scheduledto execute during the RTSP operation must be
placed in RAM memory. This includes the relevant interrupt vector, as well as the interrupt
service routine instructions.
*/

// this is the key function that allows writing to the program flash memory
//--------------------------------------------------------------------------
/*
To unlock Flash operations, steps 4 through 8 below must be performed exactly in order. If the
sequence is not followed exactly, WR is not set.
1. Suspend or disable all initiators that can access the Peripheral Bus and interrupt the
unlock sequence, e.g., DMA and interrupts.
2. Set WREN bit (NVMCON<14>) to allow writes to WR and set NVMOP<3:0> bit
(NVMCON<3:0>) to the desired operation with a single store instruction.
3. Wait for LVD to start-up.
4. Load 0xAA996655 to CPU register X.
5. Load 0x556699AA to CPU register Y.
6. Load 0x00008000 to CPU register Z.
7. Store CPU register X to NVMKEY.
8. Store CPU register Y to NVMKEY.
9. Store CPU register Z to NVMCONSET.
10. Wait for WR bit (NVMCON<15>) to be cleared.
11. Clear the WREN bit (NVMCON<14>).
12. Check the WRERR (NVMCON<13>) and LVDERR (NVMCON<12>) bits to ensure that
the program/erase sequence completed successfully.
When the WR bit is set, the program/erase sequence starts and the CPU is unable to execute
from Flash memory for the duration of the sequence.
*/

#endif // FLASH_H
