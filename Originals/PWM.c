#include "p33EP512MU810.h"
#include "PWM.h"
#include "Interrupts.h"

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

void intitPWMedge (void)
{
//Independent PWM Mode, Independent Duty Cycle and Phase, Fixed Primary Period, Edge-Aligned:
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
/* Set PWM Period on Primary Time Base */
//PTPER = 12500;
/* Set Phase Shift */
PHASE1 = 12500;
SPHASE1 = 12500;
PHASE2 = 12500;
SPHASE2 = 12500;

// Set Duty Cycles
PDC1 = 4700;
SDC1 = 4700;
PDC2 = 4700;
SDC2 = 4700;

/* Set Dead Time Values */
DTR1 = DTR2 = DTR3 = 0;
ALTDTR1 = ALTDTR2 = ALTDTR3 = 0;
/* Set PWM Mode to Independent */
IOCON1 = IOCON2 = IOCON3 = 0xCC00;
/* Set Primary Time Base, Edge-Aligned Mode and Independent Duty Cycles */
PWMCON1 = PWMCON2 = PWMCON3 = 0x0200;
/* Configure Faults */
FCLCON1 = FCLCON2 = FCLCON3 = 0x0003;
/* 1:8 Prescaler */
PTCON2 = 0b100;
/* Enable PWM Module */
PTCON = 0x8000;
}

void wait (pw *ptr_pw)
{
    int C = 0;
    while (C == 0){
        if (ptr_pw->aile >= 9160){
            if(ptr_pw->elev <= 5650){
                if(ptr_pw->rudd <= 5650){
                    C = 1;
                    PDC1 = 5400;
                    SDC1 = 5400;
                    PDC2 = 5400;
                    SDC2 = 5400;
                }
            }
        }
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