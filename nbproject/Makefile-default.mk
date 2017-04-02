#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=-mafrlcsj
else
COMPARISON_BUILD=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=MAVLink/include/bittest.c SD-SPI.c FSIO.c flash.c data_storage.c main.c I2C2.c events.c analog2digital.c I2C.c InputCapture.c Interrupts.c LCD.c LED.c MAVLink.c MAVParams.c Mixer.c Oszillator.c PID.c PWM.c Pins.c RPi.c SPI-SD.c SRF02.c Testfunktionen.c Timer.c UART.c UM7.c autopilot.c config.c data_services.c eeprom_udb4.c minIni.c nv_memory_table.c parameter_table.c parameter_table2.c parameter_table_init.c serial.c servoPrepare.c state.c usart.c MPL3115A2_Barometer.c Filter.c height_fusion.c matrix_calculator.c UART1_K6.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/MAVLink/include/bittest.o ${OBJECTDIR}/SD-SPI.o ${OBJECTDIR}/FSIO.o ${OBJECTDIR}/flash.o ${OBJECTDIR}/data_storage.o ${OBJECTDIR}/main.o ${OBJECTDIR}/I2C2.o ${OBJECTDIR}/events.o ${OBJECTDIR}/analog2digital.o ${OBJECTDIR}/I2C.o ${OBJECTDIR}/InputCapture.o ${OBJECTDIR}/Interrupts.o ${OBJECTDIR}/LCD.o ${OBJECTDIR}/LED.o ${OBJECTDIR}/MAVLink.o ${OBJECTDIR}/MAVParams.o ${OBJECTDIR}/Mixer.o ${OBJECTDIR}/Oszillator.o ${OBJECTDIR}/PID.o ${OBJECTDIR}/PWM.o ${OBJECTDIR}/Pins.o ${OBJECTDIR}/RPi.o ${OBJECTDIR}/SPI-SD.o ${OBJECTDIR}/SRF02.o ${OBJECTDIR}/Testfunktionen.o ${OBJECTDIR}/Timer.o ${OBJECTDIR}/UART.o ${OBJECTDIR}/UM7.o ${OBJECTDIR}/autopilot.o ${OBJECTDIR}/config.o ${OBJECTDIR}/data_services.o ${OBJECTDIR}/eeprom_udb4.o ${OBJECTDIR}/minIni.o ${OBJECTDIR}/nv_memory_table.o ${OBJECTDIR}/parameter_table.o ${OBJECTDIR}/parameter_table2.o ${OBJECTDIR}/parameter_table_init.o ${OBJECTDIR}/serial.o ${OBJECTDIR}/servoPrepare.o ${OBJECTDIR}/state.o ${OBJECTDIR}/usart.o ${OBJECTDIR}/MPL3115A2_Barometer.o ${OBJECTDIR}/Filter.o ${OBJECTDIR}/height_fusion.o ${OBJECTDIR}/matrix_calculator.o ${OBJECTDIR}/UART1_K6.o
POSSIBLE_DEPFILES=${OBJECTDIR}/MAVLink/include/bittest.o.d ${OBJECTDIR}/SD-SPI.o.d ${OBJECTDIR}/FSIO.o.d ${OBJECTDIR}/flash.o.d ${OBJECTDIR}/data_storage.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/I2C2.o.d ${OBJECTDIR}/events.o.d ${OBJECTDIR}/analog2digital.o.d ${OBJECTDIR}/I2C.o.d ${OBJECTDIR}/InputCapture.o.d ${OBJECTDIR}/Interrupts.o.d ${OBJECTDIR}/LCD.o.d ${OBJECTDIR}/LED.o.d ${OBJECTDIR}/MAVLink.o.d ${OBJECTDIR}/MAVParams.o.d ${OBJECTDIR}/Mixer.o.d ${OBJECTDIR}/Oszillator.o.d ${OBJECTDIR}/PID.o.d ${OBJECTDIR}/PWM.o.d ${OBJECTDIR}/Pins.o.d ${OBJECTDIR}/RPi.o.d ${OBJECTDIR}/SPI-SD.o.d ${OBJECTDIR}/SRF02.o.d ${OBJECTDIR}/Testfunktionen.o.d ${OBJECTDIR}/Timer.o.d ${OBJECTDIR}/UART.o.d ${OBJECTDIR}/UM7.o.d ${OBJECTDIR}/autopilot.o.d ${OBJECTDIR}/config.o.d ${OBJECTDIR}/data_services.o.d ${OBJECTDIR}/eeprom_udb4.o.d ${OBJECTDIR}/minIni.o.d ${OBJECTDIR}/nv_memory_table.o.d ${OBJECTDIR}/parameter_table.o.d ${OBJECTDIR}/parameter_table2.o.d ${OBJECTDIR}/parameter_table_init.o.d ${OBJECTDIR}/serial.o.d ${OBJECTDIR}/servoPrepare.o.d ${OBJECTDIR}/state.o.d ${OBJECTDIR}/usart.o.d ${OBJECTDIR}/MPL3115A2_Barometer.o.d ${OBJECTDIR}/Filter.o.d ${OBJECTDIR}/height_fusion.o.d ${OBJECTDIR}/matrix_calculator.o.d ${OBJECTDIR}/UART1_K6.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/MAVLink/include/bittest.o ${OBJECTDIR}/SD-SPI.o ${OBJECTDIR}/FSIO.o ${OBJECTDIR}/flash.o ${OBJECTDIR}/data_storage.o ${OBJECTDIR}/main.o ${OBJECTDIR}/I2C2.o ${OBJECTDIR}/events.o ${OBJECTDIR}/analog2digital.o ${OBJECTDIR}/I2C.o ${OBJECTDIR}/InputCapture.o ${OBJECTDIR}/Interrupts.o ${OBJECTDIR}/LCD.o ${OBJECTDIR}/LED.o ${OBJECTDIR}/MAVLink.o ${OBJECTDIR}/MAVParams.o ${OBJECTDIR}/Mixer.o ${OBJECTDIR}/Oszillator.o ${OBJECTDIR}/PID.o ${OBJECTDIR}/PWM.o ${OBJECTDIR}/Pins.o ${OBJECTDIR}/RPi.o ${OBJECTDIR}/SPI-SD.o ${OBJECTDIR}/SRF02.o ${OBJECTDIR}/Testfunktionen.o ${OBJECTDIR}/Timer.o ${OBJECTDIR}/UART.o ${OBJECTDIR}/UM7.o ${OBJECTDIR}/autopilot.o ${OBJECTDIR}/config.o ${OBJECTDIR}/data_services.o ${OBJECTDIR}/eeprom_udb4.o ${OBJECTDIR}/minIni.o ${OBJECTDIR}/nv_memory_table.o ${OBJECTDIR}/parameter_table.o ${OBJECTDIR}/parameter_table2.o ${OBJECTDIR}/parameter_table_init.o ${OBJECTDIR}/serial.o ${OBJECTDIR}/servoPrepare.o ${OBJECTDIR}/state.o ${OBJECTDIR}/usart.o ${OBJECTDIR}/MPL3115A2_Barometer.o ${OBJECTDIR}/Filter.o ${OBJECTDIR}/height_fusion.o ${OBJECTDIR}/matrix_calculator.o ${OBJECTDIR}/UART1_K6.o

# Source Files
SOURCEFILES=MAVLink/include/bittest.c SD-SPI.c FSIO.c flash.c data_storage.c main.c I2C2.c events.c analog2digital.c I2C.c InputCapture.c Interrupts.c LCD.c LED.c MAVLink.c MAVParams.c Mixer.c Oszillator.c PID.c PWM.c Pins.c RPi.c SPI-SD.c SRF02.c Testfunktionen.c Timer.c UART.c UM7.c autopilot.c config.c data_services.c eeprom_udb4.c minIni.c nv_memory_table.c parameter_table.c parameter_table2.c parameter_table_init.c serial.c servoPrepare.c state.c usart.c MPL3115A2_Barometer.c Filter.c height_fusion.c matrix_calculator.c UART1_K6.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512MU810
MP_LINKER_FILE_OPTION=,--script=p33EP512MU810.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/MAVLink/include/bittest.o: MAVLink/include/bittest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/MAVLink/include" 
	@${RM} ${OBJECTDIR}/MAVLink/include/bittest.o.d 
	@${RM} ${OBJECTDIR}/MAVLink/include/bittest.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MAVLink/include/bittest.c  -o ${OBJECTDIR}/MAVLink/include/bittest.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MAVLink/include/bittest.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MAVLink/include/bittest.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SD-SPI.o: SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/SD-SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SD-SPI.c  -o ${OBJECTDIR}/SD-SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SD-SPI.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SD-SPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FSIO.o: FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FSIO.o.d 
	@${RM} ${OBJECTDIR}/FSIO.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FSIO.c  -o ${OBJECTDIR}/FSIO.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FSIO.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/FSIO.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/flash.o: flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/flash.o.d 
	@${RM} ${OBJECTDIR}/flash.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  flash.c  -o ${OBJECTDIR}/flash.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/flash.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/flash.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/data_storage.o: data_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/data_storage.o.d 
	@${RM} ${OBJECTDIR}/data_storage.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  data_storage.c  -o ${OBJECTDIR}/data_storage.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/data_storage.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/data_storage.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/I2C2.o: I2C2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C2.o.d 
	@${RM} ${OBJECTDIR}/I2C2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  I2C2.c  -o ${OBJECTDIR}/I2C2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2C2.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/I2C2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/events.o: events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/events.o.d 
	@${RM} ${OBJECTDIR}/events.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  events.c  -o ${OBJECTDIR}/events.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/events.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/events.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/analog2digital.o: analog2digital.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/analog2digital.o.d 
	@${RM} ${OBJECTDIR}/analog2digital.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  analog2digital.c  -o ${OBJECTDIR}/analog2digital.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/analog2digital.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/analog2digital.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/I2C.o: I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C.o.d 
	@${RM} ${OBJECTDIR}/I2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  I2C.c  -o ${OBJECTDIR}/I2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2C.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/I2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/InputCapture.o: InputCapture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/InputCapture.o.d 
	@${RM} ${OBJECTDIR}/InputCapture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  InputCapture.c  -o ${OBJECTDIR}/InputCapture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/InputCapture.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/InputCapture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Interrupts.o: Interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Interrupts.o.d 
	@${RM} ${OBJECTDIR}/Interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Interrupts.c  -o ${OBJECTDIR}/Interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Interrupts.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/LCD.o: LCD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/LCD.o.d 
	@${RM} ${OBJECTDIR}/LCD.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  LCD.c  -o ${OBJECTDIR}/LCD.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/LCD.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/LCD.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/LED.o: LED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/LED.o.d 
	@${RM} ${OBJECTDIR}/LED.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  LED.c  -o ${OBJECTDIR}/LED.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/LED.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/LED.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MAVLink.o: MAVLink.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MAVLink.o.d 
	@${RM} ${OBJECTDIR}/MAVLink.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MAVLink.c  -o ${OBJECTDIR}/MAVLink.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MAVLink.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MAVLink.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MAVParams.o: MAVParams.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MAVParams.o.d 
	@${RM} ${OBJECTDIR}/MAVParams.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MAVParams.c  -o ${OBJECTDIR}/MAVParams.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MAVParams.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MAVParams.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Mixer.o: Mixer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Mixer.o.d 
	@${RM} ${OBJECTDIR}/Mixer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Mixer.c  -o ${OBJECTDIR}/Mixer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Mixer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Mixer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Oszillator.o: Oszillator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Oszillator.o.d 
	@${RM} ${OBJECTDIR}/Oszillator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Oszillator.c  -o ${OBJECTDIR}/Oszillator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Oszillator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Oszillator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PID.o: PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PID.o.d 
	@${RM} ${OBJECTDIR}/PID.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PID.c  -o ${OBJECTDIR}/PID.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PID.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/PID.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PWM.o: PWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PWM.o.d 
	@${RM} ${OBJECTDIR}/PWM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PWM.c  -o ${OBJECTDIR}/PWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PWM.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/PWM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pins.o: Pins.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Pins.o.d 
	@${RM} ${OBJECTDIR}/Pins.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pins.c  -o ${OBJECTDIR}/Pins.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pins.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pins.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/RPi.o: RPi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/RPi.o.d 
	@${RM} ${OBJECTDIR}/RPi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  RPi.c  -o ${OBJECTDIR}/RPi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/RPi.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/RPi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SPI-SD.o: SPI-SD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SPI-SD.o.d 
	@${RM} ${OBJECTDIR}/SPI-SD.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SPI-SD.c  -o ${OBJECTDIR}/SPI-SD.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SPI-SD.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SPI-SD.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SRF02.o: SRF02.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SRF02.o.d 
	@${RM} ${OBJECTDIR}/SRF02.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SRF02.c  -o ${OBJECTDIR}/SRF02.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SRF02.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SRF02.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Testfunktionen.o: Testfunktionen.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Testfunktionen.o.d 
	@${RM} ${OBJECTDIR}/Testfunktionen.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Testfunktionen.c  -o ${OBJECTDIR}/Testfunktionen.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Testfunktionen.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Testfunktionen.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Timer.o: Timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Timer.o.d 
	@${RM} ${OBJECTDIR}/Timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Timer.c  -o ${OBJECTDIR}/Timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Timer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UART.o: UART.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART.o.d 
	@${RM} ${OBJECTDIR}/UART.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART.c  -o ${OBJECTDIR}/UART.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UART.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/UART.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UM7.o: UM7.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UM7.o.d 
	@${RM} ${OBJECTDIR}/UM7.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UM7.c  -o ${OBJECTDIR}/UM7.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UM7.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/UM7.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/autopilot.o: autopilot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/autopilot.o.d 
	@${RM} ${OBJECTDIR}/autopilot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  autopilot.c  -o ${OBJECTDIR}/autopilot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/autopilot.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/autopilot.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/config.o: config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/config.o.d 
	@${RM} ${OBJECTDIR}/config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  config.c  -o ${OBJECTDIR}/config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/config.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/data_services.o: data_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/data_services.o.d 
	@${RM} ${OBJECTDIR}/data_services.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  data_services.c  -o ${OBJECTDIR}/data_services.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/data_services.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/data_services.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/eeprom_udb4.o: eeprom_udb4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eeprom_udb4.o.d 
	@${RM} ${OBJECTDIR}/eeprom_udb4.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  eeprom_udb4.c  -o ${OBJECTDIR}/eeprom_udb4.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/eeprom_udb4.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/eeprom_udb4.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/minIni.o: minIni.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/minIni.o.d 
	@${RM} ${OBJECTDIR}/minIni.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  minIni.c  -o ${OBJECTDIR}/minIni.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/minIni.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/minIni.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/nv_memory_table.o: nv_memory_table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/nv_memory_table.o.d 
	@${RM} ${OBJECTDIR}/nv_memory_table.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  nv_memory_table.c  -o ${OBJECTDIR}/nv_memory_table.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/nv_memory_table.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/nv_memory_table.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/parameter_table.o: parameter_table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/parameter_table.o.d 
	@${RM} ${OBJECTDIR}/parameter_table.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  parameter_table.c  -o ${OBJECTDIR}/parameter_table.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/parameter_table.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/parameter_table.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/parameter_table2.o: parameter_table2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/parameter_table2.o.d 
	@${RM} ${OBJECTDIR}/parameter_table2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  parameter_table2.c  -o ${OBJECTDIR}/parameter_table2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/parameter_table2.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/parameter_table2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/parameter_table_init.o: parameter_table_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/parameter_table_init.o.d 
	@${RM} ${OBJECTDIR}/parameter_table_init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  parameter_table_init.c  -o ${OBJECTDIR}/parameter_table_init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/parameter_table_init.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/parameter_table_init.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/serial.o: serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/serial.o.d 
	@${RM} ${OBJECTDIR}/serial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  serial.c  -o ${OBJECTDIR}/serial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/serial.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/serial.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/servoPrepare.o: servoPrepare.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/servoPrepare.o.d 
	@${RM} ${OBJECTDIR}/servoPrepare.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  servoPrepare.c  -o ${OBJECTDIR}/servoPrepare.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/servoPrepare.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/servoPrepare.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/state.o: state.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/state.o.d 
	@${RM} ${OBJECTDIR}/state.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  state.c  -o ${OBJECTDIR}/state.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/state.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/state.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/usart.o: usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/usart.o.d 
	@${RM} ${OBJECTDIR}/usart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  usart.c  -o ${OBJECTDIR}/usart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/usart.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/usart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MPL3115A2_Barometer.o: MPL3115A2_Barometer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MPL3115A2_Barometer.o.d 
	@${RM} ${OBJECTDIR}/MPL3115A2_Barometer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MPL3115A2_Barometer.c  -o ${OBJECTDIR}/MPL3115A2_Barometer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MPL3115A2_Barometer.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MPL3115A2_Barometer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Filter.o: Filter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Filter.o.d 
	@${RM} ${OBJECTDIR}/Filter.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Filter.c  -o ${OBJECTDIR}/Filter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Filter.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Filter.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/height_fusion.o: height_fusion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/height_fusion.o.d 
	@${RM} ${OBJECTDIR}/height_fusion.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  height_fusion.c  -o ${OBJECTDIR}/height_fusion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/height_fusion.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/height_fusion.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/matrix_calculator.o: matrix_calculator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/matrix_calculator.o.d 
	@${RM} ${OBJECTDIR}/matrix_calculator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  matrix_calculator.c  -o ${OBJECTDIR}/matrix_calculator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/matrix_calculator.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/matrix_calculator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UART1_K6.o: UART1_K6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART1_K6.o.d 
	@${RM} ${OBJECTDIR}/UART1_K6.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART1_K6.c  -o ${OBJECTDIR}/UART1_K6.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UART1_K6.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/UART1_K6.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/MAVLink/include/bittest.o: MAVLink/include/bittest.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}/MAVLink/include" 
	@${RM} ${OBJECTDIR}/MAVLink/include/bittest.o.d 
	@${RM} ${OBJECTDIR}/MAVLink/include/bittest.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MAVLink/include/bittest.c  -o ${OBJECTDIR}/MAVLink/include/bittest.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MAVLink/include/bittest.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MAVLink/include/bittest.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SD-SPI.o: SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/SD-SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SD-SPI.c  -o ${OBJECTDIR}/SD-SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SD-SPI.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SD-SPI.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/FSIO.o: FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/FSIO.o.d 
	@${RM} ${OBJECTDIR}/FSIO.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  FSIO.c  -o ${OBJECTDIR}/FSIO.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/FSIO.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/FSIO.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/flash.o: flash.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/flash.o.d 
	@${RM} ${OBJECTDIR}/flash.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  flash.c  -o ${OBJECTDIR}/flash.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/flash.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/flash.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/data_storage.o: data_storage.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/data_storage.o.d 
	@${RM} ${OBJECTDIR}/data_storage.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  data_storage.c  -o ${OBJECTDIR}/data_storage.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/data_storage.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/data_storage.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  main.c  -o ${OBJECTDIR}/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/I2C2.o: I2C2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C2.o.d 
	@${RM} ${OBJECTDIR}/I2C2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  I2C2.c  -o ${OBJECTDIR}/I2C2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2C2.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/I2C2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/events.o: events.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/events.o.d 
	@${RM} ${OBJECTDIR}/events.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  events.c  -o ${OBJECTDIR}/events.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/events.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/events.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/analog2digital.o: analog2digital.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/analog2digital.o.d 
	@${RM} ${OBJECTDIR}/analog2digital.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  analog2digital.c  -o ${OBJECTDIR}/analog2digital.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/analog2digital.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/analog2digital.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/I2C.o: I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/I2C.o.d 
	@${RM} ${OBJECTDIR}/I2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  I2C.c  -o ${OBJECTDIR}/I2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/I2C.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/I2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/InputCapture.o: InputCapture.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/InputCapture.o.d 
	@${RM} ${OBJECTDIR}/InputCapture.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  InputCapture.c  -o ${OBJECTDIR}/InputCapture.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/InputCapture.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/InputCapture.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Interrupts.o: Interrupts.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Interrupts.o.d 
	@${RM} ${OBJECTDIR}/Interrupts.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Interrupts.c  -o ${OBJECTDIR}/Interrupts.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Interrupts.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Interrupts.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/LCD.o: LCD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/LCD.o.d 
	@${RM} ${OBJECTDIR}/LCD.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  LCD.c  -o ${OBJECTDIR}/LCD.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/LCD.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/LCD.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/LED.o: LED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/LED.o.d 
	@${RM} ${OBJECTDIR}/LED.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  LED.c  -o ${OBJECTDIR}/LED.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/LED.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/LED.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MAVLink.o: MAVLink.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MAVLink.o.d 
	@${RM} ${OBJECTDIR}/MAVLink.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MAVLink.c  -o ${OBJECTDIR}/MAVLink.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MAVLink.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MAVLink.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MAVParams.o: MAVParams.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MAVParams.o.d 
	@${RM} ${OBJECTDIR}/MAVParams.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MAVParams.c  -o ${OBJECTDIR}/MAVParams.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MAVParams.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MAVParams.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Mixer.o: Mixer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Mixer.o.d 
	@${RM} ${OBJECTDIR}/Mixer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Mixer.c  -o ${OBJECTDIR}/Mixer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Mixer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Mixer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Oszillator.o: Oszillator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Oszillator.o.d 
	@${RM} ${OBJECTDIR}/Oszillator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Oszillator.c  -o ${OBJECTDIR}/Oszillator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Oszillator.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Oszillator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PID.o: PID.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PID.o.d 
	@${RM} ${OBJECTDIR}/PID.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PID.c  -o ${OBJECTDIR}/PID.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PID.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/PID.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/PWM.o: PWM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/PWM.o.d 
	@${RM} ${OBJECTDIR}/PWM.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  PWM.c  -o ${OBJECTDIR}/PWM.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/PWM.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/PWM.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Pins.o: Pins.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Pins.o.d 
	@${RM} ${OBJECTDIR}/Pins.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Pins.c  -o ${OBJECTDIR}/Pins.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Pins.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Pins.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/RPi.o: RPi.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/RPi.o.d 
	@${RM} ${OBJECTDIR}/RPi.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  RPi.c  -o ${OBJECTDIR}/RPi.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/RPi.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/RPi.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SPI-SD.o: SPI-SD.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SPI-SD.o.d 
	@${RM} ${OBJECTDIR}/SPI-SD.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SPI-SD.c  -o ${OBJECTDIR}/SPI-SD.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SPI-SD.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SPI-SD.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/SRF02.o: SRF02.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SRF02.o.d 
	@${RM} ${OBJECTDIR}/SRF02.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  SRF02.c  -o ${OBJECTDIR}/SRF02.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/SRF02.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/SRF02.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Testfunktionen.o: Testfunktionen.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Testfunktionen.o.d 
	@${RM} ${OBJECTDIR}/Testfunktionen.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Testfunktionen.c  -o ${OBJECTDIR}/Testfunktionen.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Testfunktionen.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Testfunktionen.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Timer.o: Timer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Timer.o.d 
	@${RM} ${OBJECTDIR}/Timer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Timer.c  -o ${OBJECTDIR}/Timer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Timer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Timer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UART.o: UART.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART.o.d 
	@${RM} ${OBJECTDIR}/UART.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART.c  -o ${OBJECTDIR}/UART.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UART.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/UART.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UM7.o: UM7.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UM7.o.d 
	@${RM} ${OBJECTDIR}/UM7.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UM7.c  -o ${OBJECTDIR}/UM7.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UM7.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/UM7.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/autopilot.o: autopilot.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/autopilot.o.d 
	@${RM} ${OBJECTDIR}/autopilot.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  autopilot.c  -o ${OBJECTDIR}/autopilot.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/autopilot.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/autopilot.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/config.o: config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/config.o.d 
	@${RM} ${OBJECTDIR}/config.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  config.c  -o ${OBJECTDIR}/config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/config.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/data_services.o: data_services.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/data_services.o.d 
	@${RM} ${OBJECTDIR}/data_services.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  data_services.c  -o ${OBJECTDIR}/data_services.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/data_services.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/data_services.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/eeprom_udb4.o: eeprom_udb4.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/eeprom_udb4.o.d 
	@${RM} ${OBJECTDIR}/eeprom_udb4.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  eeprom_udb4.c  -o ${OBJECTDIR}/eeprom_udb4.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/eeprom_udb4.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/eeprom_udb4.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/minIni.o: minIni.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/minIni.o.d 
	@${RM} ${OBJECTDIR}/minIni.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  minIni.c  -o ${OBJECTDIR}/minIni.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/minIni.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/minIni.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/nv_memory_table.o: nv_memory_table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/nv_memory_table.o.d 
	@${RM} ${OBJECTDIR}/nv_memory_table.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  nv_memory_table.c  -o ${OBJECTDIR}/nv_memory_table.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/nv_memory_table.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/nv_memory_table.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/parameter_table.o: parameter_table.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/parameter_table.o.d 
	@${RM} ${OBJECTDIR}/parameter_table.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  parameter_table.c  -o ${OBJECTDIR}/parameter_table.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/parameter_table.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/parameter_table.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/parameter_table2.o: parameter_table2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/parameter_table2.o.d 
	@${RM} ${OBJECTDIR}/parameter_table2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  parameter_table2.c  -o ${OBJECTDIR}/parameter_table2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/parameter_table2.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/parameter_table2.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/parameter_table_init.o: parameter_table_init.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/parameter_table_init.o.d 
	@${RM} ${OBJECTDIR}/parameter_table_init.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  parameter_table_init.c  -o ${OBJECTDIR}/parameter_table_init.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/parameter_table_init.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/parameter_table_init.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/serial.o: serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/serial.o.d 
	@${RM} ${OBJECTDIR}/serial.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  serial.c  -o ${OBJECTDIR}/serial.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/serial.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/serial.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/servoPrepare.o: servoPrepare.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/servoPrepare.o.d 
	@${RM} ${OBJECTDIR}/servoPrepare.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  servoPrepare.c  -o ${OBJECTDIR}/servoPrepare.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/servoPrepare.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/servoPrepare.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/state.o: state.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/state.o.d 
	@${RM} ${OBJECTDIR}/state.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  state.c  -o ${OBJECTDIR}/state.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/state.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/state.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/usart.o: usart.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/usart.o.d 
	@${RM} ${OBJECTDIR}/usart.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  usart.c  -o ${OBJECTDIR}/usart.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/usart.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/usart.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/MPL3115A2_Barometer.o: MPL3115A2_Barometer.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MPL3115A2_Barometer.o.d 
	@${RM} ${OBJECTDIR}/MPL3115A2_Barometer.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  MPL3115A2_Barometer.c  -o ${OBJECTDIR}/MPL3115A2_Barometer.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/MPL3115A2_Barometer.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/MPL3115A2_Barometer.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/Filter.o: Filter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Filter.o.d 
	@${RM} ${OBJECTDIR}/Filter.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  Filter.c  -o ${OBJECTDIR}/Filter.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/Filter.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/Filter.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/height_fusion.o: height_fusion.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/height_fusion.o.d 
	@${RM} ${OBJECTDIR}/height_fusion.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  height_fusion.c  -o ${OBJECTDIR}/height_fusion.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/height_fusion.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/height_fusion.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/matrix_calculator.o: matrix_calculator.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/matrix_calculator.o.d 
	@${RM} ${OBJECTDIR}/matrix_calculator.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  matrix_calculator.c  -o ${OBJECTDIR}/matrix_calculator.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/matrix_calculator.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/matrix_calculator.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/UART1_K6.o: UART1_K6.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/UART1_K6.o.d 
	@${RM} ${OBJECTDIR}/UART1_K6.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  UART1_K6.c  -o ${OBJECTDIR}/UART1_K6.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/UART1_K6.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -mlarge-code -mlarge-data -msmall-scalar -O0 -I"MAVLink" -msmart-io=1 -Wall -msfr-warn=off  
	@${FIXDEPS} "${OBJECTDIR}/UART1_K6.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip"  -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -legacy-libc  $(COMPARISON_BUILD)  -I"Microchip/Include" -I"Microchip" -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/1.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
