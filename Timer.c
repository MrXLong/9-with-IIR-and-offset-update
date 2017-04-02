#include "Timer.h"
#include "p33EP512MU810.h"
#include "UM7.h"
#include "RS232.h"
#include "LCD.h"
#include "analog2digital.h"
#include "options.h"
#include "MAVLink/include/common/mavlink.h" //access to mavlink types
#include "state.h"  //access to vehicle modes
#include "PWM.h"    //check pwm status

#define FCY 40000000ULL

    //				Fcy
    //  Schritte × Prescaler = -----
    //				F		

static uint16_t cpu_timer = 0;

uint8_t udb_cpu_load(void)
{
	// scale cpu_timer to seconds*100 for percent loading
	return (uint8_t)(__builtin_muluu(cpu_timer, CPU_LOAD_PERCENT) >> 16);
}

Timer timer;

unsigned int telemetry_ctr=0;
unsigned int autopilot_ctr=0;
boolean tx_watchdog_enabled = false;

int led_rs232 = 0;
int led_gps = 0;
int led_um7 = 0;

void init_timers (void)
{
    //Timer 1, Synchronisation der Input Capture Module
    T1CONbits.TON = 0;          //Disable Timer
    T1CONbits.TCS = 0;          //Select internal instruction cycle clock
    T1CONbits.TGATE = 0;        //Disable Gated Timer mode
    T1CONbits.TCKPS = 0b01;     //Select 1:256 Prescaler                                           // 0b11
    TMR1 = 0x00;                //Clear timer register
    PR1 = 40000;                //Load the period value, Timer Überlauf alle 1ms ////// 65000
    IPC0bits.T1IP = 2;          //Set Timer 1 Interrupt Priority Level (should be highest?, which is 7 per DS70000600D-p.29)
    IFS0bits.T1IF = 0;          //Clear Timer 1 Interrupt Flag
    T1CONbits.TON = 1;          //Start Timer


	//Timer 2, Input Capture 1 (Gas)
	T2CONbits.TON = 0;			//Disable Timer2
    T2CONbits.T32 = 0;          //MW-03.11.2015- [DS70616G-p.278] make T2 16 bit timer 
	T2CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
	T2CONbits.TGATE = 0;        //Disable Gated Timer mode
	T2CONbits.TCKPS = 0b01;     //Select 1:8 Prescaler
	TMR2 = 0x00;				//Clear timer2 register
	PR2 = 40000;				//Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
	IPC1bits.T2IP = 3;          //Set Timer2 Interrupt Priority Level = 
	IFS0bits.T2IF = 0;			//Clear Timer2 Interrupt Flag
	T2CONbits.TON = 1;			//Start Timer2

    //Timer 3, Input Capture 2 (Rollen)
    T3CONbits.TON = 0;			//Disable Timer3
    T3CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T3CONbits.TGATE = 0;		//Disable Gated Timer mode
    T3CONbits.TCKPS = 0b01;     //Select 1:8 Prescaler
    TMR3 = 0x00;                //Clear timer2 register
    PR3 = 40000;                //Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
    IPC2bits.T3IP = 6;          //Set Timer3 Interrupt Priority Level = 
    IFS0bits.T3IF = 0;			//Clear Timer3 Interrupt Flag
    T3CONbits.TON = 1;			//Start Timer3

	//Timer 4, Input Capture 3 (Nicken)
	T4CONbits.TON = 0;			//Disable Timer4
    T4CONbits.T32 = 0;          //MW-03.11.2015- [DS70616G-p.278] make T4 16 bit timer 
	T4CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
	T4CONbits.TGATE = 0;        //Disable Gated Timer mode
	T4CONbits.TCKPS = 0b01;     //Select 1:8 Prescaler
	TMR4 = 0x00;				//Clear timer4 register
	PR4 = 40000;				//Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
	IPC6bits.T4IP = 4;          //Set Timer4 Interrupt Priority Level = 
	IFS1bits.T4IF = 0;			//Clear Timer4 Interrupt Flag
	T4CONbits.TON = 1;			//Start Timer4
	
    //Timer 5, Input Capture 4 (Gieren)
    T5CONbits.TON = 0;			//Disable Timer5
    T5CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T5CONbits.TGATE = 0;		//Disable Gated Timer mode
    T5CONbits.TCKPS = 0b01;     //Select 1:8 Prescaler
    TMR5 = 0x00;                //Clear timer5 register
    PR5 = 40000;                //Period Value = 6ms (30000 Schritte); Auflösung = 0,2µs/Schritt
    IPC7bits.T5IP = 4;          //Set Timer5 Interrupt Priority Level = 3
    IFS1bits.T5IF = 0;			//Clear Timer5 Interrupt Flag
    T5CONbits.TON = 1;			//Start Timer5

    //Timer 6, Überwachung Sensordaten-Empfang
    T6CONbits.TON = 0;			//Disable Timer6
    T6CONbits.T32 = 0;          //MW-03.11.2015- [DS70616G-p.278] make T6 16 bit timer 
	T6CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
	T6CONbits.TGATE = 0;        //Disable Gated Timer mode
	T6CONbits.TCKPS = 0b01;     //Select 1:64 Prescaler
	TMR6 = 0x00;				//Clear timer6 register
	PR6 = 10000;//62500;		//Auflösung = 200ns/Schritt, Abtastzeit: 2ms
	IPC11bits.T6IP = 6;         //Set Timer6 Interrupt Priority Level = 5 x   ***********************************
	IFS2bits.T6IF = 0;			//Clear Timer6 Interrupt Flag
	T6CONbits.TON = 1;			//Start Timer6

    //Timer 7, RS232
    T7CONbits.TON = 0;			//Disable Timer7
    T7CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T7CONbits.TGATE = 0;		//Disable Gated Timer mode
    T7CONbits.TCKPS = 0b11;     //Select 1:256 Prescaler
    TMR7 = 0x00;                //Clear timer5 register
    PR7 = 65535;                //step = 6,4µs/Schritt, Abtastzeit: 419ms
    IPC12bits.T7IP = 5;         //Set Timer7 Interrupt Priority Level = 4
    IFS3bits.T7IF = 0;			//Clear Timer7 Interrupt Flag
    T7CONbits.TON = 1;			//Start Timer7
      
    //Timer 8, GPS
    T8CONbits.TON = 0;			//Disable Timer8
    T8CONbits.T32 = 0;          //MW-03.11.2015- [DS70616G-p.278] make T8 16 bit timer 
    T8CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T8CONbits.TGATE = 0;		//Disable Gated Timer mode
    T8CONbits.TCKPS = 0b11;     //Select 1:256 Prescaler
    TMR8 = 0x00;                //Clear timer8 register
    //PR8 = 2500;                //step = 6,4µs/Schritt, Abtastzeit: 400ms
    PR8 = 65535;   
    IPC12bits.T8IP = 1;         //Set Timer8 Interrupt Priority Level = 1 (lowest)
    IFS3bits.T8IF = 0;			//Clear Timer8 Interrupt Flag
    //T8CONbits.TON = 1;			//Start Timer8


    //Timer off; 16-bit mode; Prescaler 3; Input clk CPU/256

        // Timer6 period 419.424 ms = 2.384222171 Hz
    
        //PR8 = 0xFFFF-0x001;         //w:?
    
        // 40MHz/256 = 156.250KHz , 156.250KHz/65535 = 2.384222171 Hz
        //                                           = 419.424 ms
        // later on 80ms required form this timer for the utrasonic
        // initializing. the calculation as it shown below:
        //
        //  PeriodofChecking80ms      65535
        // --------------------- = --------- ===> PeriodtoCheck = 12500
        //          80ms           419.424ms

       
    //This timer is used to create a timer within the software
    //Timer 9. mm.ss.ms
    T9CONbits.TON = 0;			//Disable Timer9
    T9CONbits.TCS = 0;			//Internal Clock Source (Fosc/2)
    T9CONbits.TGATE = 0;                //Disable Gated Timer mode
    T9CONbits.TCKPS = 0b01;             //Select 1:8 Prescaler
    TMR9 = 0x00;			//Clear timer6 register
    PR9 = 5000;//62500;			//Auflösung = 200ns/Schritt, Abtastzeit: 1ms
    IPC13bits.T9IP = 2;                 //Set Timer9 Interrupt Priority Level = 5 x   ***********************************
    timer.ms = 0;
    timer.s = 0;
    timer.min = 0;
    IFS3bits.T9IF = 0;			//Clear Timer9 Interrupt Flag
    T9CONbits.TON = 1;			//Start Timer9
}

/*
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{
	IFS0bits.T2IF = 0; //Clear Timer2 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T2Interrupt(void)
{
	IFS0bits.T2IF = 0; //Clear Timer2 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T3Interrupt(void)
{
	IFS0bits.T3IF = 0; //Clear Timer3 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T4Interrupt(void)
{
	IFS1bits.T4IF = 0; //Clear Timer4 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T5Interrupt(void)
{
	IFS1bits.T5IF = 0; //Clear Timer5 interrupt flag
}
*/

void __attribute__((__interrupt__, no_auto_psv)) _T6Interrupt(void)
{
    if(tx_watchdog_enabled)
        if(!checkTransmitter()) //transmitter signal lost
            activateFailsafe();
    
    if (incomingUM7Message == 1)
    {
        if(led_um7 == 0) //store_data = 1
        {
            led_um7 = 1;
            store_data = 0;
            LATBbits.LATB2 = 1;
            //LATBbits.LATB5 = 1;     //LED3 ON
            //DMA1CONbits.CHEN = 1; //enable DMA1 channel
            //MW ... Wrong DMA channel?
        } 
        else //(led_um7 == 1, store_data = 0)
        {
            led_um7 = 0;
            //LATBbits.LATB5 = 0;     //LED3 OFF
            LATBbits.LATB2 = 0;
            store_data = 1;
            incomingUM7Message = 0;
        }
    }
    watch ++;
    if (watch >= 5)
    {    /// 5 times * 2ms = 10ms
        watch = 0;
        fly_var = 1;
    }

    IFS2bits.T6IF = 0;                      //Clear Timer6 interrupt flag
}

static unsigned int flashctr=0;
void __attribute__((__interrupt__, no_auto_psv)) _T7Interrupt(void)
{
    //LED should be solid ON if quad is armed and under control of RC Transmitter
    //LED should flash in a slow pattern if in other autonomous modes and armed
    switch(vehicle_base_mode)
    {
        case MAV_MODE_STABILIZE_ARMED:
            LATEbits.LATE8 = 1;
        break;
        case MAV_MODE_MANUAL_ARMED:
            LATEbits.LATE8 = 1;
        break;
        case MAV_MODE_GUIDED_ARMED:
            if(flashctr>0)LATEbits.LATE8=~LATEbits.LATE8;
        break;
        case MAV_MODE_AUTO_ARMED:
            if(flashctr>0)LATEbits.LATE8=~LATEbits.LATE8;
        break;
        default:
            LATEbits.LATE8 = 0;
        break;
    }
    
    flashctr++;
    
    if(flashctr>1)
        flashctr=0;
    IFS3bits.T7IF = 0;                      //Clear Timer7 interrupt flag
}

/*
void __attribute__((__interrupt__, no_auto_psv)) _T8Interrupt(void)
{
    
    if(led_gps == 0){
        led_gps = 1;
        //LATBbits.LATB4 = 1;
        DMA8CONbits.CHEN = 1;
        
    } else {
        led_gps = 0;
        //LATBbits.LATB4 = 0;
    }
    
    IFS3bits.T8IF = 0;                      //Clear Timer8 interrupt flag
}*/

//use this timer to change brightness of LEDs
static unsigned int pwmCounter = 0;
void __attribute__((__interrupt__, no_auto_psv)) _T8Interrupt(void)
{
//want 9500 flashing 20hz
//at 7450 want flashing 10 hz
//want 5400 flashing 1hz
//make timer 20hz and automatically flash unless led value is less than

//THIS CODE CHANGES THE BRIGHTNESS OF THE LED BASED ON MOTOR POWER
//HOWEVER WITH THE CURRENT BOARD IT IS NOT USEFUL BECAUSE LEDS ARE
//TOO DIM TO TELL DIFFERENCE IN BRIGHTNESS LEVEL    
//    if(motor1_led > pwmCounter)
//        LATEbits.LATE8=~LATEbits.LATE8;
//    if(motor2_led > pwmCounter)
//        LATEbits.LATE9=~LATEbits.LATE9;
//    if(motor3_led > pwmCounter)
//        LATBbits.LATB5=~LATBbits.LATB5;
//    if(motor4_led > pwmCounter)
//        LATBbits.LATB4=~LATBbits.LATB4;
    
//    motor1_led = 1;    
//    motor2_led = 10;
//    motor3_led = 20;
//    motor4_led = 40;
    
//    if(motor4_led > pwmCounter)
//        LATEbits.LATE8=1; else LATEbits.LATE8=0;
//    if(motor3_led > pwmCounter)
//        LATEbits.LATE9=1; else LATEbits.LATE9=0;
//    if(motor2_led > pwmCounter)
//        LATBbits.LATB5=1; else LATBbits.LATB5=0;
//    if(motor1_led > pwmCounter)
//        LATBbits.LATB4=1; else LATBbits.LATB4=0;
//    
//END LED BRIGHTNESS CODE    
    
    pwmCounter++;
    
    if(pwmCounter>0)
    {
        dispMotors();
        //dispCompass();
        pwmCounter=0;
    }
    
//used to be for GPS lock verification before GPS was moved to RPi
//    if(GPSlocked)
//        LATBbits.LATB3 = ~LATBbits.LATB3; else LATBbits.LATB3 = 1;
    
    if(battery.volts <= LOW_VOLTAGE_ALARM)
        LATBbits.LATB3 = ~LATBbits.LATB3; else LATBbits.LATB3 = 0;
    
    TMR8=0x00;
    IFS3bits.T8IF = 0;                      //Clear Timer8 interrupt flag
}

void __attribute__((__interrupt__, no_auto_psv)) _T9Interrupt(void)
{
    timer.ms++;
    if(timer.ms == 1000)
    {
        timer.ms = 0;
        timer.s++;
    }
    if(timer.s == 60)
    {
        timer.s = 0;
        timer.min++;
    }
    if(timer.min == 60)
        timer.min = 0;
    
    //timer.ctr is used as a reliable method to count milliseconds greater than
    //1000. If value becomes too high, it is reset to prevent overrun.
    timer.ctr++;
    if(timer.ctr == 65535)
        timer.ctr = 0;
    
    //there should be a better way to do call events periodically
    //rather than having an interrupt counter for each periodic function,
    //but I'm not sure how to do it.
    //NOTE: timer.ms % X is not reliable when called from main loop
    //the main loop is called more than once within a millisec
    //we would need a microsec counter.
    telemetry_ctr++;
    if(telemetry_ctr == 65535)
        telemetry_ctr = 0;
        
    autopilot_ctr++;
    if(autopilot_ctr == 65535)
        autopilot_ctr = 0;
    
    IFS3bits.T9IF = 0;                      //Clear Timer8 interrupt flag
}
