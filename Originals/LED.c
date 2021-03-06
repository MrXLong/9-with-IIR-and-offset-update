#include "LED.h"
#include "Mixer.h"

void rollLED (void)
{
    if ((UM7_Sensors.eulerRollAngle*EulerConstant <= 5) && (UM7_Sensors.eulerRollAngle*EulerConstant >= 2))
    {
        LATBbits.LATB2 = 1;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((UM7_Sensors.eulerRollAngle*EulerConstant < 2) && (UM7_Sensors.eulerRollAngle*EulerConstant > 0.5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 1;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((UM7_Sensors.eulerRollAngle*EulerConstant <= 0.5) && (UM7_Sensors.eulerRollAngle*EulerConstant >= -0.5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((UM7_Sensors.eulerRollAngle*EulerConstant > -2) && (UM7_Sensors.eulerRollAngle*EulerConstant < -0.5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 1;
    }
    if ((UM7_Sensors.eulerRollAngle*EulerConstant <= -2) && (UM7_Sensors.eulerRollAngle*EulerConstant >= -5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 1;
        LATEbits.LATE9 = 0;
    }
}

void pitchLED (void)
{
    if ((UM7_Sensors.eulerPitchAngle*EulerConstant <= 5) && (UM7_Sensors.eulerPitchAngle*EulerConstant >= 2))
    {
        LATBbits.LATB2 = 1;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((UM7_Sensors.eulerPitchAngle*EulerConstant < 2) && (UM7_Sensors.eulerPitchAngle*EulerConstant > 0.5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 1;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((UM7_Sensors.eulerPitchAngle*EulerConstant <= 0.5) && (UM7_Sensors.eulerPitchAngle*EulerConstant >= -0.5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((UM7_Sensors.eulerPitchAngle*EulerConstant > -2) && (UM7_Sensors.eulerPitchAngle*EulerConstant < -0.5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 1;
    }
    if ((UM7_Sensors.eulerPitchAngle*EulerConstant <= -2) && (UM7_Sensors.eulerPitchAngle*EulerConstant >= -5))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 1;
        LATEbits.LATE9 = 0;
    }
}

void setValueTestLED (pw *ptr_PW, UM7DataSensor *ptr_UM7)
{
    ptr_PW->test_rudd_rate = (ptr_PW->elev - 7531);
    if ((ptr_PW->test_rudd_rate < 50) && (ptr_PW->test_rudd_rate > 1))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 1;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((ptr_UM7->eulerYawRate <= 1) && (ptr_UM7->eulerYawRate >= -1))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((ptr_PW->test_rudd_rate > -50) && (ptr_PW->test_rudd_rate < -1))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 1;
        LATEbits.LATE9 = 0;
    }
}

void euler_test_LED (pw *ptr_PW, UM7DataSensor *ptr_UM7)
{

    if ((ptr_UM7->eulerYawRate < 50) && (ptr_UM7->eulerYawRate > 1))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 1;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((ptr_UM7->eulerYawRate <= 1) && (ptr_UM7->eulerYawRate >= -1))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 1;
        LATBbits.LATB5 = 1;
        LATEbits.LATE8 = 0;
        LATEbits.LATE9 = 0;
    }
    if ((ptr_UM7->eulerYawRate > -50) && (ptr_UM7->eulerYawRate < -1))
    {
        LATBbits.LATB2 = 0;
        LATBbits.LATB3 = 0;
        LATBbits.LATB4 = 0;
        LATBbits.LATB5 = 0;
        LATEbits.LATE8 = 1;
        LATEbits.LATE9 = 0;
    }
}

void startupLED (void)
{
    LATBbits.LATB2 = 1;
    LATBbits.LATB3 = 1;
    LATBbits.LATB4 = 1;
    LATBbits.LATB5 = 1;
    LATEbits.LATE8 = 1;
    LATEbits.LATE9 = 1;
}

void errorLED (void)
{
    LATBbits.LATB2 = 0;
    LATBbits.LATB3 = 1;
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 1;
    LATEbits.LATE8 = 0;
    LATEbits.LATE9 = 0;
}

void rdy2goLED (void)
{
    LATBbits.LATB2 = 1;
    LATBbits.LATB3 = 0;
    LATBbits.LATB4 = 0;
    LATBbits.LATB5 = 0;
    LATEbits.LATE8 = 0;
    LATEbits.LATE9 = 0;
}