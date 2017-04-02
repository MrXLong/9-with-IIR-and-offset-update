#include "p33EP512MU810.h"
#include "PWM.h"
#include "state.h"
#include <stdlib.h>   //abs function

//struct pwm PWM;
//struct pwm *ptr_PWM = &PWM;

void initPWMcenter (void)
{
//Independent PWM Mode - Independent Duty Cycles and Periods, No Phase-Shifting, Center-Aligned
//PHASEx/ SPHASEx Register Value Calculation in Center-Aligned Mode
 /*                                      Fosc
                PHASEx, SPHASEx = ---------------------------------------
                                    Fpwm * PWM input clock prescaler * 2

PHASEx         Period of the PWMxH
SPHASEx        Period of the PWMxL
Fpwm           Desired PWM Switching Frequency
prescaler      From PCLKDIV in PTCON2
2              Because Center aligned mode is used,so the period and
               duty cycle multiplied by 2
Fosc           2*Fcy

PHASEx/ SPHASEx Register: 2^16 = 65535 = max. Value
                                        80MHz
                PHASEx, SPHASEx = ---------------- = 12500
                                    400Hz * 8 * 2

PDCx/SDCx:
-> init = 4700 (0,94 ms)
-> min = 5500 (1,1 ms)
-> max = 9500 (1,9 ms)

*/
    /*

    //Periode, Fpwm = 400MHz
    PHASE1 = 12500;      //Motor1
    SPHASE1 = 12500;     //Motor2
    PHASE2 = 12500;      //Motor3
    SPHASE2 = 12500;     //Motor4 12500*4 = 50000

    //Initialisierungs-Pulsweite
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PDC1 = 4700;
    SDC1 = 4700;
    PDC2 = 4700;
    SDC2 = 4700;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Dead Time
    DTR1 = DTR2 = 0;
    ALTDTR1 = ALTDTR2 = 0;

    //Independent Mode
    IOCON1 = IOCON2 = 0xCC00;

    //Independent Time Bases, Center-Aligned Mode and Independent Duty Cycles
    PWMCON1 = PWMCON2 = 0x0204; //1000000100

    //FLTSTAT=0, no fault Interrupt is pending (Interrupt Status Bit is cleared)
    //CLSTAT=0
    //TRGSTA=0
    //FLTIEN=0
    //CLIEN=0
    //TRGIEN=0
    //ITB = 1, PHASEx/SPHASEx register provides timing for this PWM-Generator                   (Periode; PHASEx/SPHASEx)
    //MDCS, PDCx and SDCx registers provide duty cycle information for this PWM generator   (Pulsweite; PDCx/SDCx)
    //DTC=0, Positive dead time actively applied for all output modes
	//DTC
    //DTCP=0, Death Time Compensation ignored (if DTC=00)
    //=0
    //MTBS=0, PWM uses primary master time base for synchronization and as the clock source for the PWM generation logic
    //CAM = 1, Center-Aligned-Mode is enable
    //XPRES=0, External pins do not affect PWM time base
    //IUE=0, Updates to the active PDCx registers are synchronized to the PWM time base

    // Configure Faults
    FCLCON1 = FCLCON2 = 0x0003; //FLTMOD = 11; Fault input is disabled

    //Prescaler
    PTCON2 = 0b011;        //1:8

    // Enable PWM Module
    PTCON = 0x8000;     //PTEN = 1
     * */
}

// [DS70645C-page 14-66]
void intitPWMedge (void)
{
//Independent PWM Mode?Independent Duty Cycles and Periods, No Phase-Shifting, Edge-Aligned
/*
 Periode/ Pulsweite:
                             80MHz
PTPER, PHASEx, SPHASEx = ---------------- = 12500
                            400Hz * 16

PDCx/SDCx:
-> init = 4700 (0,94 ms)
-> min = 5500 (1,1 ms)
-> max = 9500 (1,9 ms)
 */

    
//PTPER =  50000;     //100Hz

PTPER =  11236;     //445Hz

//PTPER = 65535;      //78Hz

PHASE1 = 0;
SPHASE1 = 0;
PHASE2 = 0;
SPHASE2 = 0;    

PDC1 = 4700;
SDC1 = 4700;
PDC2 = 4700;
SDC2 = 4700;

DTR1 = DTR2 = DTR3 = 0;
ALTDTR1 = ALTDTR2 = ALTDTR3 = 0;
    
IOCON1 = IOCON2 = IOCON3 = 0xCC00;

PWMCON1 = PWMCON2 = PWMCON3 = 0x0000;

FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;
PTCON2 = 0b100;
PTCON = 0x8000;


///* Set PWM Periods on PHASEx Registers*/
////All PWM outputs should have the same period, just different Duty cycles respective
////to desired motor speed
//PHASE1 = 12500;
//SPHASE1 = 12500;
//PHASE2 = 12500;
//SPHASE2 = 12500;
//
//// Set Duty Cycles
//PDC1 = 4700;
//SDC1 = 4700;
//PDC2 = 4700;
//SDC2 = 4700;
//
///* Set Dead Time Values */
//DTR1 = DTR2 = DTR3 = 0;
//ALTDTR1 = ALTDTR2 = ALTDTR3 = 0;
//
///* Set PWM Mode to Independent */
//IOCON1 = IOCON2 = IOCON3 = 0xCC00;
//
///* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
//PWMCON1 = PWMCON2 = PWMCON3 = 0x0200;
//
///* Configure Faults */
//FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;
//
///* 1:16 Prescaler */
//PTCON2 = 0b100;
///* Enable PWM Module */
//PTCON = 0x8000;
}


void stickConfig(pw *ptr_pw)
{
    if(ptr_pw->thro<=5550 && ptr_pw->aile>=9460 && ptr_pw->elev<=5550 && ptr_pw->rudd<=5600)
    {
        changeState(ARMED, TRANSMITTER);
    }
        
    else if(ptr_pw->thro<=5550 && ptr_pw->aile<=5550 && ptr_pw->elev<=5550 && ptr_pw->rudd>=9460)
    {
        changeState(DISARMED, TRANSMITTER);
    }
}

// Output of motor values to the PWM modules
void PWM_out (mix *ptr_pwm)
{
    SDC1 = (unsigned int)ptr_pwm->m1;
    PDC1 = (unsigned int)ptr_pwm->m2;
    SDC2 = (unsigned int)ptr_pwm->m3;
    PDC2 = (unsigned int)ptr_pwm->m4;
}

//currently there is no good way to verify transmitter signal. an attempt
//was made here to use the jump in throttle signal when the signal is lost
//as an indicator, but this is not reliable and will cause problems
boolean checkTransmitter(void)
{
//    static int prev_thr_pwm = 0;
//    static int current_thr_pwm = 0;
//    static int thr_diff = 0;
//    static boolean first_pass = true;
//    
//    current_thr_pwm = ptr_pulsewidth->thro;
//    thr_diff = abs(current_thr_pwm - prev_thr_pwm);
//    
//    if(first_pass)
//        thr_diff = 0;
//    
//    first_pass = false;
//    prev_thr_pwm = current_thr_pwm;
//    if((thr_diff>800))
//        return false;   //loss of transmitter
//    else 
//    {
//        return true;    //transmitter ok
//    }
    
    return true;
}
