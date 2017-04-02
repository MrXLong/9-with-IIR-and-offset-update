/* 
 * First Version:
 * File:   main.c
 * Author: Simon Brehm
 * Created on 4. November 2014, 11:00.
 *
 * Second Version:
 * File:    main.c
 * Author: Alejandro Perez
 * Created on 01. July 2015, 15:30.
 *
 */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
	Typname 	Bitbreite 	Wertebereich 					Alias
	int8_t 		8 Bit 		-128..127 					signed char
	int16_t 	16 Bit 		-32768..32767 					signed int
	int32_t 	32 Bit 		-2147483648..2147483647 			signed long int
	int64_t 	64 Bit 		-9223372036854775808..9223372036854775807 	signed long long
	
	uint8_t 	8 Bit 		0..255 						unsigned char
	uint16_t 	16 Bit 		0..65535 					unsigned int
	uint32_t 	32 Bit 		0..4294967295 					unsigned long int
	uint64_t 	64 Bit 		0..18446744073709551615 			unsigned long long
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <xc.h>
#include "p33EP512MU810.h"
#define FCY 40000000ULL
#include <libpic30.h>
#include "Interrupts.h"
#include <stdlib.h>
#include <stdint.h>        //Includes uint16_t definition
#include "stdio.h"
#include "Pins.h"
#include "Oszillator.h"
#include "PWM.h"
#include "InputCapture.h"
#include "Timer.h"
#include "UART.h"
#include "Mixer.h"
#include "UM7.h"
#include "PID.h"
#include "libq.h"
#include "LED.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -----------------------------------------Main program functions------------------------------------------------//

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// -----------------------------------------Global Variables (main)---------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile int watch = 0;                                                     // Flag for defective sensor communication and emergency call program (Timer 6)
int detect = 0;                                                             // Flag to determine the permanent deviation (Timer 6)
int ResetCause = 0;
int active = 0;

volatile int initError = 1;                                                 // Flag for Initialization (maximum engine speed by programming)


// PID
s_controller_specifications aile_elev;
s_controller_specifications rudder;
s_controller_fixedpoint coeff_fixedpoint;
s_controller_fixedpoint *ptr_FIX = &coeff_fixedpoint;

// UM7
UM7_Commun UM7dataSensors;                                          // Struct to contain the data received from UM7 sensor
UM7DataSensor UM7_Sensors;                                          // Struct to contain the UM7 data
UM7DataSensor *ptr_eulerAngle = &UM7_Sensors;

// Mixer


// Select Internal FRC at POR
_FOSCSEL(FNOSC_FRC & IESO_OFF);

// Enable Clock Switching and Configure POSC in XT mode
_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_XT);

//Communication with ICSP Pins on PGEC3, JTAG is disabled, reset target vector to Primary Flash
_FICD(ICS_PGD3 & JTAGEN_OFF & RSTPRI_PF);
_FPOR(BOREN_OFF);
_FWDT(FWDTEN_OFF);

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//----------------------------------------------- main program--------------------------------------------------//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    // Initialization Oscillator
    initOscillator ();

    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0x03);                                  //COSC = 011, Primary Oscillator (XT, HS, EC) with PLL
    __builtin_write_OSCCONL(OSCCON | 0x01);                         //Requests Oscillator Switch to selection (by NOSC)

    while (OSCCONbits.COSC!= 0x03);                                 // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!= 1);                                    // Wait for PLL to lock

    __builtin_write_OSCCONL(OSCCON & ~(1<<6));                      //IOLOCK nicht aktiv (unlock Regisers)
    init_Pins ();
    remap_Pins ();
    __builtin_write_OSCCONL(OSCCON | (1<<6));                       //IOLOCK aktiv (lock Registers)

    enableInterrupts();
    intitPWMedge ();
    init_timers();
    __delay_ms(300);

    ////////////////////////////////////////////////////////////

    // Start values PWM output, needed to initialize the external motor driver 0.9ms
    SDC1 = 5400;        //M1
    PDC1 = 5400;       //M2
    SDC2 = 5400;      //M3
    PDC2 = 5400;     //M4

    // Initialization UART with DMA
    initUM7communication();
    startupLED ();

    // Initialization Input Capture modules 1-4
    InputCap1_Thro ();
    InputCap2_Aile ();
    InputCap3_Elev ();
    InputCap4_Rudd ();

    __delay_ms(100);

    // Software reset, if communication failed Sensor System
    if (incomingUM7Message != 1)
    {
        asm ("RESET");
    }
//---------------------------------------------------------------------------------------------
    // Kalibrierungs-Routine Sensorsystem, benötigt ca. 3 Sekunden
    // setzt Bezugsvektoren von Beschleunigungssensoren und Drehratensensoren bei aktueller Lage
    // und setzt den Kalman-Filter zurück
    /*
    setAcc_Ref_Vec ();
    __delay_ms(300);
    ZeroGyros();
    __delay_ms(300);
    resetEKF ();
    __delay_ms(1200);
    ZeroGyros();
    __delay_ms(300);
    setAcc_Ref_Vec ();
    __delay_ms(300);
    resetEKF ();
    __delay_ms(400);
    //determineVariance (ptr_eulerAngle);
    */
 //----------------------------------------------------------------------------------------------

    // Regler-Paramater (Fließkommazahlen) festlegen und in Festkommazahlen umwandeln
    set_PID_coefficients();
    get_PID_coefficients_fixedpoint(ptr_FIX, aile_elev, rudder);

    // Start TIMER6 and put register to 0; Start the monitoring carried out for sensor data reception
    T6CONbits.TON = 1;
    TMR6 = 0x00;

    // Search remote control data, sensor data, mixer, calculations and control calculations
	while(1)                                                    //while loop needs (currently) 1.12 µs
	{
            if (active == 0)
            {
            wait (ptr_pulsewidth);                                  // Were prevented engine start as long as both joystick not pulled down right
            active = 1;
            }
            ResetCause = RCON;                                      // Variable for debug mode to read reset basic

            if (active == 1)
            {
            // Active standby mode (ready to start)
            if (ptr_pulsewidth->thro <= 5840)
            {
                PDC1 = 5400; //5400
                SDC1 = 5400;
                PDC2 = 5400;
                SDC2 = 5400;

                rdy2goLED ();                                       // outer green LED
            }
            // Routine for operation with active position control
            else if ((incomingUM7Message == 1) && (watch == 0))     // every 10 ms are transmitted sensor data
            {                                                       // that is, Sampling time of the controller depends on the frequency of the UM7
                storeUM7data(UM7dataSensors);                       // Save Euler angles
                
                if ((ic_thr==1) && (ic_ail==1) && (ic_ele==1) && (ic_rud==1))           //wenn alle 4 Input Capture Signale neu anstehen
                {
                    setValue (ptr_pulsewidth);                      // Current setpoints for controllers
                }                                                   // otherwise perform with old setpoints and current sensor values Controller calculations
                actualValue_aile_elev (ptr_eulerAngle);            // actual values for controllers
                actualValue_rudd (ptr_eulerAngle, ptr_FIX);
                
                // Setpoint (old or new, from Input Capture) and actual value (new, from UM7) passed to controller
                // PID calculation
                PIDcal_fixedpoint_aile (ptr_FIX);
                PIDcal_fixedpoint_elev (ptr_FIX);
                PIDcal_fixedpoint_rudd (ptr_FIX);

                // Passed manipulated variables to the functions of mixer and calculate motor dependent values
                throMix (ptr_pulsewidth);
                aileMix (ptr_FIX);
                elevMix (ptr_FIX);
                ruddMix (ptr_FIX);

                // when new mixer values for thro, aile, pending elev (gas, roll, pitch, yaw)
                if ((thr_mix==1) && (ail_mix==1) && (ele_mix==1) && (rud_mix==1))
                {
                    motorMix (thro, aile, elev, rudd);              // final mixing of premixed engine values, resetting the flags of if condition
                }
                // If after uploading the software error function (one or more motors maximum speed), then reset security
                if ((ptr_MIX->m1 >= 7500 && initError==1) || (ptr_MIX->m4 >= 7500 && initError==1))
                {
                    asm ("RESET");
                }
                if ((ptr_MIX->m2 >= 7500 && initError==1) || (ptr_MIX->m3 >= 7500 && initError==1))            // IF after start a malfunction (Motor1 and Motor4 max. Rotational Speed)
                {
                    asm ("RESET");
                }
                PWM_out (ptr_MIX);                                  // Output to PWM
                initError = 0;
                TMR6 = 0x00;                                        // TIMER6 Register Reset (data reception takes place)
                incomingUM7Message = 0;                             // Reset variable for received sensor data
                watch = 0;                                          // Variable zur Überwachung fehlerhafter Sensorkommunikation zurücksetzen
                                                                    //Reset variable to monitor erroneous sensor communication

                // View the current roll or pitch angle to 0 °
                //rollLED ();
                //pitchLED ();
                //setValueTestLED (ptr_pulsewidth, ptr_eulerAngle);
                euler_test_LED(ptr_pulsewidth, ptr_eulerAngle);
            }

            // Routine für Handbetrieb (Notfallbetrieb)
            if (watch == 1)                                         // if 10 times in a row (= 100ms) was no sensor data reception
            {                                                       // then start an emergency program (pure manual operation, no position control)
                errorLED ();
                if ((ic_thr==1) && (ic_ail==1) && (ic_ele==1) && (ic_rud==1))
                {
                    setValueEmergency (ptr_pulsewidth);
                }
                throMix (ptr_pulsewidth);                           // For thro (gas) mixer
                emergencyMix (ptr_PWM);                             // For aile, elev, Rudd (roll, pitch, yaw) mixer

                if ((thr_mix==1) && (ail_mix==1) && (ele_mix==1) && (rud_mix==1))
                {
                    motorMix (thro, aile, elev, rudd);
                }
                PWM_out (ptr_MIX);
                watch = 0;
            }
        }
        }
}