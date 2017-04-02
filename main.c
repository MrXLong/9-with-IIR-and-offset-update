/****************************************************************************
 * File:   main.c
 * Created by: Simon Brehm on 4.11.2014

 * Description: This is the main file of the Quadcopter project. It contains
 * calls to other functions in other files and handles the initialization 
 * calls for the hardware. It contains the main operating loop of the software,
 * which is a while(1) loop, such that once it is entered, it will repeat 
 * indefinitely.
 * 
 * Second Version
 * Created By: Alejandro Perez on 01.07.2015

 * Last Modified: 04.12.2015 (MW)
 * Last used Compiler: C30 v3.31
****************************************************************************/


// <editor-fold defaultstate="collapsed" desc="Includes">

#define FCY 40000000ULL
#include "MAVLink/include/matrixpilot_mavlink_bridge_header.h"
#include "MAVLink/include/common/mavlink.h"
#include <libpic30.h>
#include "Interrupts.h"
#include <stdlib.h>
#include "Pins.h"
#include "Oszillator.h"
#include "PWM.h"
#include "Timer.h"
#include "LED.h"
#include "serial.h"
#include "GPS.h"
#include "SPI-SD.h"
#include "LCD.h"
#include <i2c.h>
#include "I2C.h"
#include "SRF02.h"
#include <math.h>
#include "MAVLink.h"
#include "autopilot.h"
#include "state.h"
#include "options.h"
#include "analog2digital.h" //to init a2d
#include "SD-SPI.h"
//#include "usart.h"
#include "MPL3115A2_Barometer.h"
#include "Filter.h"
#include "height_fusion.h"
#include "matrix_calculator.h"
#include "UART1_K6.h"

// </editor-fold>

//DEBUG & OPTION SWITCHES
#define setPIDcoeffRS232    0       //set PID values over RS232
#define GPSsetRS232         0       //set GPS setpoint coordinates over RS232
#define uart3ReturnGPS      0       
#define useLCD              0       //Use the LCD 
#define uart3PIDout         0       
#define uart3MagOut         0
#define uart3out            0       //Allow for output on UART3 
#define debugTracking       0       //troubleshoot Tracking routine
#define debugAutoLand       0       //troubleshoot AutoLand routine
#define autoLandActive      0       //activate AutoLand functionality
#define debugMotorOutput    0   
#define useMAVLink                 //output messages to MAVLink 
#define Baro_init_tuning    20
//flight mode (acro or stabil) defined in PID.h


/*******************************************************************************
                       Global Variables (main)
 ******************************************************************************/

volatile int watch = 0;         // Flag for defective sensor communication and emergency call program (Timer 6)
volatile int fly_var = 0;       // Flag that allows the fly routines
volatile int store_data = 0;
int detect = 0;                 // Flag to determine the permanent deviation (Timer 6)
int ResetCause = 0;
int active = 0;
int initial_calc=1;

int CoordinatesSet = 0;
int heightIsSet=0;

unsigned int height_setpoint=0;

volatile int initError = 1;     // Flag for Initialization (maximum engine speed by programming)

unsigned long temp=0;        //debug
unsigned long check=0;      //debug
char checkSum[2];           //debug

int gpsRollCorrection = 0;
int gpsPitchCorrection = 0;

int GPSlocked=0;

requestor_t requestor;
// PID
s_controller_specifications aile_elev;
s_controller_specifications rudder;
s_controller_specifications gas;
s_controller_fixedpoint coeff_fixedpoint;
s_controller_fixedpoint *ptr_FIX = &coeff_fixedpoint;

// UM7
UM7_Commun UM7dataSensors;                                          // Struct to contain the data received from UM7 sensor
UM7DataSensor UM7_Sensors;                                          // Struct to contain the UM7 data
UM7DataSensor *ptr_eulerAngle = &UM7_Sensors;

// GPS
GPS_Commun GPScommdata;
GPS_Commun *ptr_GPScommdata = &GPScommdata;
GPSData GPSdata;
GPSData *ptr_GPSdata = &GPSdata;

//SPI-SD
SD_DATA sd;

 //Select Internal FRC at POR
//_FOSCSEL(FNOSC_FRC & IESO_OFF);
 //Enable Clock Switching and Configure POSC in XT mode
//_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_XT); //OSCIOFNC_ON -> OSC2 is general purpose digital I/O pin
//Communication with ICSP Pins on PGEC3, JTAG is disabled, reset target vector to Primary Flash
//_FICD(ICS_PGD3 & JTAGEN_OFF & RSTPRI_PF);
//_FWDT(FWDTEN_OFF);
//_FPOR( ALTI2C1_ON & ALTI2C2_ON & BOREN_OFF);    // Configure SPI pins

#pragma config FNOSC=FRC,IESO=OFF,FCKSM=CSECMD,OSCIOFNC=ON,POSCMD=XT,ICS=PGD3,JTAGEN=OFF,RSTPRI=PF,FWDTEN=OFF,ALTI2C1=ON,ALTI2C2=ON,BOREN=OFF


/*******************************************************************************
                                 Main Loop
*******************************************************************************/
int main(void)
{
    
    unsigned long ctr = 0;        //DEBUG
    
    //float PID_out = 0;

    //functions for GPS point to point position control
    /*
    int turnedToHeading=0;
    float distToTarget=0;
    int targetReached=0;
    float calcdHeading=888;
    float yawOffset=0;
    */

    
    initOscillator ();

    // Initiate Clock Switch to Primary Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONH(0x03);               //COSC = 011, Primary Oscillator (XT, HS, EC) with PLL
    __builtin_write_OSCCONL(OSCCON | 0x01);      //Requests Oscillator Switch to selection (by NOSC)

    while (OSCCONbits.COSC!= 0x03);         // Wait for Clock switch to occur
    while (OSCCONbits.LOCK!= 1);            // Wait for PLL to lock

    __builtin_write_OSCCONL(OSCCON & ~(1<<6));      //IOLOCK nicht aktiv (unlock Regisers)
    init_Pins();
    remap_Pins();
    __builtin_write_OSCCONL(OSCCON | (1<<6));       //IOLOCK aktiv (lock Registers)

    enableInterrupts();
    
#if (AIRFRAME_TYPE == QUAD)
        intitPWMedge();
#endif //AIRFRAME_TYPE
    
    init_timers();
    __delay_ms(50);
    
    // Start values PWM output, needed to initialize the external motor driver 0.9ms
    SDC1 = 5400;        //M1
    PDC1 = 5400;       //M2
    SDC2 = 5400;      //M3
    PDC2 = 5400;     //M4
    
    vehicle_state = DISARMED; //vehicle initializes as disarmed
    vehicle_mode = PREFLIGHT; //vehicle intitializes in preflight mode

    // Initialization UART with DMA
    initUM7communication();
    __delay_ms(10);  
    setMag_Ref_Vec();
    __delay_ms(10);  
    ZeroGyros();
    __delay_ms(10);  
    resetEKF();
    __delay_ms(10);  

    
    initSerial();
    //initRS232communication();
    //initGPScommunication();
    ptr_GPSdata->home_f = 0; // Flag to save GPS home latitude - longitude.
    initSPISDcommunication();
    __delay_ms(50);        //MW Why?

    
    if(useLCD)
    {
        Lcd_Init();
        LCD_conf();
    }

    //I2C Ultrasonic
    initI2C1Drv();      //Initializing I2C1
    InitUtrasonic();    //Initializing The Ultrasonic sensors and mark them as they

    // Initialization Input Capture modules 1-6
    InitInputCapture();

    // Software reset, if communication failed Sensor System
    if (incomingUM7Message != 1)
    {
        asm ("RESET");
    }

    // Controller Parameters
    set_PID_coefficients(); //initialize PID controller and gains
    get_PID_coefficients(ptr_FIX, aile_elev, rudder, gas);

    //Start TIMER6 and put register to 0; Start the monitoring carried out for sensor data reception
    T8CONbits.TON = 1;
    TMR8=0x00;
    T6CONbits.TON = 1;
    TMR6 = 0x00;
    calibrate_offsets();
    
    init_mavlink();
    init_ADC();
    init_DMA5();
    
    ptr_GPSdata->prev_latitude=0;
    ptr_GPSdata->prev_longitude=0;
    ptr_FIX->rudd_angle_target = 0;
    ptr_FIX->setpoint_angle_aile = 0; 
    ptr_FIX->setpoint_angle_elev = 0;
    ptr_FIX->setpoint_angle_rudd = 0;
    
//used for most basic possible debug (flash LEDs on and off)
//    while(1)
//    {
//            LATBbits.LATB2=1;
//            LATBbits.LATB3=1;
//            LATBbits.LATB4=1;
//            LATBbits.LATB5=1;
//            LATEbits.LATE9=1;
//            LATEbits.LATE8=1;
//            __delay_ms(500);
//            LATBbits.LATB2=0;
//            LATBbits.LATB3=0;
//            LATBbits.LATB4=0;
//            LATBbits.LATB5=0;
//            LATEbits.LATE9=0;
//            LATEbits.LATE8=0;
//            __delay_ms(500);
//    };
    Configure_Uart1_K6();
    MPL3115A2_Barometer_initial();
    Kalman_MPL3115A2_init();
    __delay_ms(200);
//    while(1)
//    {
//        Read_MPL_NoMatterWhat();
//        output_with_K6( MPL_Barometer_Dist_cm ,  5,  1,  1); //MPL_Barometer_Dist_cm  Kalman_Filter_Height Sonar_Dist[0] MPL_Barometer_offset
//    }
    int distance_count = 0;
    //---------------------------------------------------------------------------------------------------------------------------------------
    while(1)
    {   
//        LATBbits.LATB5 = 1;
        
        //NOTE: DEDICATED PIN FOR TIMING MEASUREMENTS IS TRISEbits.TRISE6
        //See Pins.c for further info
        
#ifdef useMAVLink
            if(telemetry_ctr > 25) //40Hz
            {
                mavlink_output_40hz();
                telemetry_ctr = 0;
            }
#endif  //useMAVLink
        
        
        
        if(store_data == 1) //store UM7 data, update angles with data
        {
            storeUM7data(UM7dataSensors);
            actualValue_aile_elev(ptr_eulerAngle);              //sensor pitch and roll value
            actualValue_rudd(ptr_eulerAngle, ptr_FIX);          //sensor yaw value
        }
            
       
        
        if((timer.ms % 150) == 0)    //If ultrasonic is read too often, problems occur
        {  
            //note ultrasonic address is different for this and for boxcopter
            //must be changed in SRF02
//            LATBbits.LATB5 = 1;
//            if( abs(UM7_Sensors.processedAcc_Z) < 8 )
//            {
//                Read_MPL_Barometer_polling();
//                MPL_Barometer_offset = MPL_Barometer_Dist_cm - Fusion_Height;
//            }
//            offset_update();
            ReadSensors(); // For reading the ultrasonic periodically. takes 1.51ms
            Read_MPL_Barometer_polling(1);
            Kalman_MPL3115A2_altitude();
            Height_Fusion_of_Baro_and_Ultra();
//            offset_update();
//            Sonar_Dist[0]  = (int)( Kalman_gain_MPL3115A2.matrix[0][0]*100); //Kalman_P.matrix[0][0]
            if(distance_count == 5)
            {
//                UM7_Sensors.processedAcc_Z = (int)(UM7_Sensors.processedAcc_Z);
                output_with_K6( MPL_Barometer_Dist_cm, 5, 1,  1);
                //MPL_Barometer_Dist_cm  height_after_kalman Sonar_Dist[0] MPL_Barometer_offset  Fusion_Height
//              LATBbits.LATB5 = 0; // about 2.2ms
                distance_count = 0;
            }
            distance_count++;
        }
        
        if(setPIDcoeffRS232)
        {
        //get_PID_coefficients_RS232(ptr_FIX); //calibrate PID using RS232
            if((timer.ms%500)==0)
            {
//                pollUserInput();
            }
        }
        //in boxCopter mode, device is armed on startup, since it works without remote
#if (AIRFRAME_TYPE == BOXCOPTER)
        changeState(ARMED, TRANSMITTER); //when boxcopter, assumed armed as if from transmitter
        changeMode(STABILIZE, TRANSMITTER);
#else
        stickConfig(ptr_pulsewidth); //check stick config every loop based on PW data
        main_Switch(ptr_pulsewidth);    //aux switch configuration
#endif //AIRFRAME_TYPE
        
            ResetCause = RCON;
            
        if(vehicle_state == ARMED)
        {
            // Active standby mode (ready to start)
            if(ptr_FIX->setpoint_gas <= MIN_THROTTLE_VALUE)       // Under 12% thro ptr_pulsewidth->thro 5840
            {                      
                PDC1 = 5400;                        // Motors stand by.
                SDC1 = 5400;
                PDC2 = 5400;
                SDC2 = 5400;

                reset_ipart(ptr_FIX);   //reset PID integrals when not flying
                
                //set yaw setpoint to current yaw value when not flying
                actualValue_rudd(ptr_eulerAngle, ptr_FIX);
                ptr_FIX->setpoint_angle_rudd = ptr_FIX->actual_angle_rudd;
                
            }
            
            if (fly_var ==  1)  //beginning switch
            {
                
#if (AIRFRAME_TYPE == BOXCOPTER)
                //in boxCopter mode, throttle is automatically set and switch mode is set
                ptr_FIX->setpoint_gas = 6800;
                Switch = 1;
#endif  //AIRFRAME_TYPE
                
                //cases depend on remote switch configuration
                switch(vehicle_mode)
                {
                    case STABILIZE://FREE MODE

                        if(Switch==1) //MAINTAIN HEIGHT
                        {
                        
                            //MW why do I need this if statement here?
                            /*if ((ic_thr==1) && (ic_ail==1) && (ic_ele==1) && (ic_rud==1))   //When all 4 Input Capture signals have a new value
                            { 
                                setValue (ptr_pulsewidth);
                            }*/

                            //alejandro code
                            /*if(setheight==0)    //If height setpoint not given
                            {
                                set_height(ptr_FIX);    //setpoint = current height
                                setheight = 1;
                                setValueGas(ptr_pulsewidth);    //set throttle to current throttle val
                                ptr_FIX->thr_holded = ptr_FIX->setpoint_gas;    //hold throttle = throttle setpoint
                            }
                             * 
                            ReadSensors(); //Read Ultrasonic
                            //LCD_Ultra();   //Display Ultrasonic Result
                            ptr_FIX->actual_height = Sonar_Dist[0];
                            PIDcal_height(ptr_FIX); //PID_Ultrasonic
                             */
                            //multiply_gains(0.5);
                            if(!heightIsSet)    //if height setpoint not given, then give
                            { 
                                ptr_FIX->thr_holded = ptr_FIX->setpoint_gas;
                                height_setpoint = Sonar_Dist[0];
                                heightIsSet=1; 
                            }
                            if(height_setpoint >= SONAR_MAX) //invalid setpoint
                            {
                                heightIsSet=0;
                            }
                            //ReadSensors();
                            //constantly try to hold the setpoint when in this mode
                            //holdHeight(height_setpoint, &coeff_fixedpoint); 

                            CoordinatesSet = 0;
                        }
                        else
                        {
                            if ((ic_thr==1) && (ic_ail==1) && (ic_ele==1) && (ic_rud==1))   //When all 4 Input Capture signals have a new value
                            { 
                                setValue (ptr_pulsewidth);
                            }
                            setValueGas(ptr_pulsewidth);

                            height_setpoint=0;    //reset height setpoint
                            heightIsSet=0;        //reset flag
                            CoordinatesSet=0;     //reset flag
                        }
                        
                        takeoff_requested = false; //flag for resetting hover PID
                        land_requested = false; //flag for resetting hover PID
                        break;

                        case GUIDED://GUIDED MODE
                        {      
                            
                            if(autopilot_ctr > 1000/AUTOPILOT_FREQ) //AUTOPILOT_FREQ Hz defined in options.h
                            {
                                autopilot_periodic();
                                autopilot_ctr = 0;
                            }
                        
                        //here we have to take care not to update reference values
                        //with those given by transmitter. Reference should
                        //be given by GCS
                        
                        heightIsSet = 0;
                        height_setpoint = 0;
                        
                        break;
                    }
                    case TEST://RETURN HOME -- this shouldn't be TEST
                        
                        //setpoints become home positions
                        ptr_GPSdata->latitude_setpoint = GPSdata.hlatitude;
                        ptr_GPSdata->longitude_setpoint = GPSdata.hlongitude;
                        ptr_GPSdata->altitude_setpoint = GPSdata.haltitude;
                        
                        //repeat case2 with new septoints
 
                        
                        heightIsSet = 0;
                        //CoordinatesSet = 0;
                        height_setpoint = 0;
                        
                        break;

                    default:
                        //we should never enter this case... but we are... why?????  MW 18.04.2016
                        //if ((ic_thr==1) && (ic_ail==1) && (ic_ele==1) && (ic_rud==1)){ //When all 4 Input Capture signals have a new value
                       //     setValue (ptr_pulsewidth);
                       //     setValueGas (ptr_pulsewidth);
                       //}
                       // ----------------------- ACQUIRE THE VALUES
                       // actualValue_aile_elev (ptr_eulerAngle);            // actual values for controllers
                       // actualValue_rudd (ptr_eulerAngle, ptr_FIX);
                       // setheight = 0;
                       // CoordinatesSet = 0;
                        //ptr_FIX->setpoint_height = 0;
                        break;
                }

          //end switch
                
                if(!AcroMode)
                {
                    //Stabilize PIDs
                    PIDcal_fixedpoint_aile_angle(ptr_FIX);
                    PIDcal_fixedpoint_elev_angle(ptr_FIX);
                    PIDcal_fixedpoint_rudd_angle(ptr_FIX);
                }

                //If pilot asking for yaw change overwrite yaw stabilizer PID
                if(fabs(ptr_FIX->rudd_angle_target) > 5.0)
                { 
                    ptr_FIX->rudd_output_a = ptr_FIX->rudd_angle_target;
                    ptr_FIX->setpoint_angle_rudd = ptr_FIX->actual_angle_rudd;
                }

                
                //Rate PIDs
                PIDcal_fixedpoint_aile_rate(ptr_FIX);
                PIDcal_fixedpoint_elev_rate(ptr_FIX);
                PIDcal_fixedpoint_rudd_rate(ptr_FIX);
                
                //Mix PID outputs and send to motors
                throMix(ptr_FIX);
                aileMix(ptr_FIX);
                elevMix(ptr_FIX);
                ruddMix(ptr_FIX);
                
                // when new mixer values for thro, aile, pending elev (gas, roll, pitch, yaw)
                //if ((thr_mix==1) && (ail_mix==1) && (ele_mix==1) && (rud_mix==1)){
                    motorMix (thro, aile, elev, rudd);              // final mixing of premixed engine values, resetting the flags of if condition
                //}
                // If after uploading the software error function (one or more motors maximum speed), then reset security
//                if ((ptr_MIX->m1 >= 7500 && initError==1) || (ptr_MIX->m4 >= 7500 && initError==1)){
//                    asm ("RESET");
//                }
//                if ((ptr_MIX->m2 >= 7500 && initError==1) || (ptr_MIX->m3 >= 7500 && initError==1)){            // IF after start a malfunction (Motor1 and Motor4 max. Rotational Speed)
//                    asm ("RESET");
//                }
                PWM_out(ptr_MIX) ;//Output to PWM   //SHOULD THIS BE PUT IN AN INTERRUPT? MW
                initError = 0;
                fly_var = 0;

                //aux1();

                
            }
            
        } //active = 1 end
        else
        {   
            //if quad is disarmed, can allow for sending of data and 
            //calibrating of sensors
        }
        ctr++;
        //if(ctr>3000)
        //{
        //    LATEbits.LATE9=0;
        //    ctr=0;
        //}
    } //while(1) end
} //main end
