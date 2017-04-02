/****************************************************************************
*
* Name: 			RPi.c
* Version:			1.00
* Processor:        dsPIC33E
* Compiler:			XC16
* Header file:      p33EP512MU810.h
* Linker file:      p33EP512MU810.gld?
*
*
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
* Authors                     Date                Comments
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *ORIGINAL VERSION PCtoMaster.c 
* Mohan Aleatbi             11-10/2011          Communication Cases
*
* Wang,Huajie               7/2012              0xE Functions
*
* David López               11-04/2013          Real Time and sequential control.
*                                               Command list.
*
*ADAPTED TO RPi.c
*Matt Woodard               18.08.2016          Overhaul and adaptation of code 
 *                                              to communicate between
 *                                              quadcopter and RPi
*
****************************************************************************/

//#include "p33EP512MU810.h"
//#include "timer4.h"
//#include "timer8.h"
//#include "i2cxDrv.h"
#include "serial.h"
//#include "PwmFlipArm.h"
#include <qei32.h>  //?
#include "Rpi.h"
#include "UM7.h"
//#include "Rposition.h"
//#include "Functionalities.h"
//#include "UM6Sensor.h"

/**********Global Variables**************************************************/
extern Message MessageReceived;
extern Message queue[SizeQueue];   // Incoming message ready to store or execute in real time
extern Message messageToTransmit;  // Message to send via serial

unsigned int messagePosition = 0;   // Pointer to indicate the position where the missing message begins
unsigned int wrongMessage = 0;  // Variable that indicates if the data received are good or not
// 0 means good, 1 means for another customer, 2 means undefined error
unsigned int messageSize = 0;
unsigned int bufferAorB = 0;    // 0 means bufferA, 1 means bufferB
//DATARS232[6] array of the maximum bytes that would be transmitted to the
//slave modules in one block
unsigned char DATARS232[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//len is the size of the data array that would be transfered to the slavef
//modules   2<len<7
unsigned char len=0;

//***************************************************************************
//unsigned char EnableAvoidingObstaclesFunction = 0;
//unsigned char EnableDetectingClimpingStairsFunction = 0;
//unsigned char LabyrinthMode = 0;

//***************************************************************************
unsigned int Check = 0;            //Message checksum
unsigned int CheckSum = 0;         //Summation of the values of the every message byte

unsigned int store = 0;              //Flag to enabled "1" or disabled "0" the access to the queue
int queuePosition = 0;
int emptyQueue = 1;                  //Flag to indicate the queue is empty
int currentCommandDuration = 0;     //Last command duration [space, time or degrees]
//float startAngle = 0;               //Angle at which the robot starts moving
//int rightOrLeft = 0;                //Angular velocity direction: 0 = right, 1 = left
int calibrateDriftError = 0;        //Variable to allow the calibration of the gyroscopes drift error

int key = 0;            //Flag that enabled the use of the commands which can change
                        //the robot parameters, like the PID constants, etc
int hold = 0;           //Flag to hold the list
//int distanceTimeOrDegrees = 0;      //Distance = 0, Time = 1, Degrees = 2
//float labyrinthSpeed = 0;
//****************************************************************************


void Commun_RobotRaspberryPi()//Function to received data from the Raspberry Pi
{
    if (queue[queuePosition].Header[0] == CommandHeader)
    {
        if (queue[queuePosition].Header[1] == CommandHeader)
        {
            if (queue[queuePosition].DestinationId == VEHICLE_ID)
            {
                if (queue[queuePosition].SourceID == RPI_ID)
                {
                    // Checksum calculation
                    CheckSum = queue[queuePosition].SourceID + queue[queuePosition].PacketType;
                    int i;
                    for (i = 0; i < (queue[queuePosition].PayloadSize - 2); i++)
                    {
                        CheckSum = CheckSum + queue[queuePosition].Payload.Byte[i];
                    }
                    Check = 256 - (CheckSum %256);              // Modulo 256 checksum

                    if (queue[queuePosition].Checksum == Check) // Checking if the message is ok
                    {
                        if (store == 1 && queue[queuePosition].PacketType != 0x16 && queue[queuePosition].PacketType != 0x71
                                && queue[queuePosition].PacketType != 0x72 && queue[queuePosition].PacketType != 0x11 && queue[queuePosition].PacketType != 0x12)
                        {
                            queuePosition++;
                            emptyQueue = 0;
                        }
                        else    // Execution of the command
                        {
                            CommandList(queuePosition);
                        }
                    }
                }
            }
        }
    }
}

//Commands to change the robot parameters, such as PID constants, etc
void ConfigParametersCommands(unsigned int commandPosition)
{
        switch (queue[commandPosition].PacketType)
        {

            case 0x52:  // change all the parameters back to defualt values
                            // of the main adn flipper driver module.
                {
                       DATARS232[0]=0x29;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x01;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x53:  // change all the parameters back to defualt values
                            // of the main driver module.
                {
                       DATARS232[0]=0x29;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x01;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

            case 0x54:     // change all the parameters back to defualt values
                            // of the flipper driver module.
                {
                       DATARS232[0]=0x29;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x01;
                       len=4;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x55:     // Set  PID-Regler nach Stellungsalgorithmus
                            // Set PID control algorithm for position
                            // of the main driver module.
                {
                       DATARS232[0]=0x2F;
                       DATARS232[1]=69;
                       DATARS232[2]=0;
                       DATARS232[3]=0;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x56:     // Set  PID-Regler nach Differenzengleichung
                            // of the main driver module.
                {
                       DATARS232[0]=0x2F;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x01;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x57:     // Set  PID-Regler nach Stellungsalgorithmus
                            // of the flipper driver module.
                {
                       DATARS232[0]=0x2F;
                       DATARS232[1]=69;
                       DATARS232[2]=0;
                       DATARS232[3]=0;
                       len=4;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x58:     // Set  PID-Regler nach Differenzengleichung
                            // of the flipper driver module.
                {
                       DATARS232[0]=0x2F;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x01;
                       len=4;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x59:     // Setting PID-Controller of the main driver module
                           // as : Kp = 0.03, Tn = 0,2, Tv = 0.
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0;
                       DATARS232[3]=0;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x5A:     // Setting PID-Controller of the main driver module
                           // as : Kp = 0.088, Tn = 0,066, Tv = 0.
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0;
                       DATARS232[3]=1;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x5B:     // Setting PID-Controller of the flipper driver module
                           // as : Kp = 0.03, Tn = 0,2, Tv = 0.
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0;
                       DATARS232[3]=0;
                       len=4;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x5C:    // Setting PID-Controller of the flipper driver module
                           // as : Kp = 0.088, Tn = 0,066, Tv = 0.
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0;
                       DATARS232[3]=1;
                       len=4;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x5D:     // Setting Sampling time of the PID-Controller of
                            // the main motors to 0.005 (10ms).
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0x0A;
                       DATARS232[3]=0xD7;
                       DATARS232[4]=0xA3;
                       DATARS232[5]=0x3B;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x5E:     // Setting Sampling time of the PID-Controller of
                            // the Flipper motors to 0.005 (10ms).
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0x0A;
                       DATARS232[3]=0xD7;
                       DATARS232[4]=0xA3;
                       DATARS232[5]=0x3B;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x5F:     // Setting PID-Parameters of the main motors
                            // individually.
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x02;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x60:     // Setting KP = 0.03 of the both main motors
                            // if other value is required then it should be
                            // entered manually here.
                {
                       DATARS232[0]=0x54;
                       DATARS232[1]=69;
                       DATARS232[2]=0x8F;
                       DATARS232[3]=0xC2;
                       DATARS232[4]=0xF5;
                       DATARS232[5]=0x3C;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                       DATARS232[0]=0x5A;
                       DATARS232[1]=69;
                       DATARS232[2]=0x8F;
                       DATARS232[3]=0xC2;
                       DATARS232[4]=0xF5;
                       DATARS232[5]=0x3C;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x61:     // Setting Tn = 0.2 of the both main motors
                            // if other value is required then it should be
                            // entered manually here.
                {
                       DATARS232[0]=0x56;
                       DATARS232[1]=69;
                       DATARS232[2]=0xCD;
                       DATARS232[3]=0xCC;
                       DATARS232[4]=0x4C;
                       DATARS232[5]=0x3E;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                       DATARS232[0]=0x5C;
                       DATARS232[1]=69;
                       DATARS232[2]=0xCD;
                       DATARS232[3]=0xCC;
                       DATARS232[4]=0x4C;
                       DATARS232[5]=0x3E;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x62:     // Setting Tv = 0.00 of the both main motors
                            // if other value is required then it should be
                            // entered manually here.
                {
                       DATARS232[0]=0x58;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x00;
                       DATARS232[4]=0x00;
                       DATARS232[5]=0x00;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                       DATARS232[0]=0x5E;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x00;
                       DATARS232[4]=0x00;
                       DATARS232[5]=0x00;
                       len=6;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                }break;

            case 0x63:     // Setting PID-Parameters of the Flipper motors
                            // individually.
                {
                       DATARS232[0]=0x30;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x02;
                       len=4;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x64:     // Setting KP = 0.03 of the both flipper motors
                            // if other value is required then it should be
                            // entered manually here.
                {
                       DATARS232[0]=0x54;
                       DATARS232[1]=69;
                       DATARS232[2]=0x8F;
                       DATARS232[3]=0xC2;
                       DATARS232[4]=0xF5;
                       DATARS232[5]=0x3C;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                       DATARS232[0]=0x5A;
                       DATARS232[1]=69;
                       DATARS232[2]=0x8F;
                       DATARS232[3]=0xC2;
                       DATARS232[4]=0xF5;
                       DATARS232[5]=0x3C;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x65:     // Setting Tn = 0.2 of the both Flipper motors
                            // if other value is required then it should be
                            // entered manually here.
                {
                       DATARS232[0]=0x56;
                       DATARS232[1]=69;
                       DATARS232[2]=0xCD;
                       DATARS232[3]=0xCC;
                       DATARS232[4]=0x4C;
                       DATARS232[5]=0x3E;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                       DATARS232[0]=0x5C;
                       DATARS232[1]=69;
                       DATARS232[2]=0xCD;
                       DATARS232[3]=0xCC;
                       DATARS232[4]=0x4C;
                       DATARS232[5]=0x3E;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x66:     // Setting Tv = 0.00 of the both Flipper motors
                            // if other value is required then it should be
                            // entered manually here.
                {
                       DATARS232[0]=0x58;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x00;
                       DATARS232[4]=0x00;
                       DATARS232[5]=0x00;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                       DATARS232[0]=0x5E;
                       DATARS232[1]=69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x00;
                       DATARS232[4]=0x00;
                       DATARS232[5]=0x00;
                       len=6;
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

             case 0x67:     //Encoder1 signal phase A with B change, for the main
                {           // and flipper arm motors
                       DATARS232[0]=0x1E;
                       DATARS232[1]=0x69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x00;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);

                }break;

             case 0x68:     //Encoder2 signal phase A with B change, for the main
                {           // and flipper arm motors
                       DATARS232[0]=0x1E;
                       DATARS232[1]=0x69;
                       DATARS232[2]=0x00;
                       DATARS232[3]=0x01;
                       len=4;
                       //I2C1_WriteData(MainMotMod_I2C1_Addr, DATARS232, len);
                       //I2C1_WriteData(FlipMotMod_I2C1_Addr, DATARS232, len);
                }break;

        }//end switch

        //distanceTimeOrDegrees = 1;//Time
        currentCommandDuration = 0;

}

void CommandList(unsigned int commandPosition)
{    
//*****************************************************************************
//Commands to configurate parameters

    if (key != 0)
    {
        //Commands to change the robot parameters, such as PID constants, etc
        ConfigParametersCommands(commandPosition);
        key = 0;//disables the key
    }

    switch (queue[commandPosition].PacketType)
    {

        case 0x50://Enables the key to change the robot parameters
            {
                    key = 1;
                    //distanceTimeOrDegrees = 1;//Time
                    currentCommandDuration = 0;

            }break;

        case 0x51://Enables the key to change the robot parameters
            {
                    key = 0;
                    //distanceTimeOrDegrees = 1;//Time
                    currentCommandDuration = 0;

            }break;

//*****************************************************************************
//Commands to program the robot

        case 0x10://Await the queue execution
        {
                   //StopMainArmDriFlip(0xC8);//Stop
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;

        }break;

        case 0x11: //Hold sequential process
        {
                    hold = 1;

        } break;

        case 0x12: //Continue sequential process
        {
                    hold = 0;

        } break;

        case 0x13://Finish program (deleting sequence)
        {
                    //StopMainArmDriFlip(0xC8);//Stop
                    queuePosition = 0;//Restarts the queue
                    emptyQueue = 1;//The queue is empty
                    //distanceTimeOrDegrees = 1;//Time
                    currentCommandDuration = 0;

        }break;

        case 0x14://Delete last command
        {
            queuePosition--;
            if(queuePosition == 0)//If the list is empty set the flag
            {
                emptyQueue = 1;
            }

        }break;

        case 0x15://Store in the queue ON
        {
            store = 1;

        }break;

        case 0x16://Store in the queue OFF
        {
            store = 0;

        }break;

//*****************************************************************************
//Automatic modes

        case 0x20://Enable Climbing Stairs automode function.
        {
                    //EnableDetectingClimpingStairsFunction = 1;

        } break;

        case 0x21://Disable Climbing Stairs automode function.
        {
                    //EnableDetectingClimpingStairsFunction = 0;

        } break;

        case 0x22://Enable Avoiding Obstacles Mode (automode function).
        {
                    //EnableAvoidingObstaclesFunction = 1;

        } break;

        case 0x23://Disable Avoiding Obstacles Mode (automode function).
        {
                    //EnableAvoidingObstaclesFunction = 0;

        } break;

        case 0x24://Enables way out of the labyrinth mode (automode function).
        {
                    //labyrinthSpeed = speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity);
                    //LabyrinthMode = 1;
                    //EnableAvoidingObstaclesFunction = 0;
                    hold = 1;//Hold the queue execution

        } break;

        case 0x25://Disable way out of the labyrinth mode (automode function).
        {
                    //LabyrinthMode = 0;
                    //StopMainArmDriFlip(0xC8);

        } break;

//*****************************************************************************
//Commands to setup the ramps

        case 0x33://Sets up the time duration the increasing ramp of the chosen motor during at desired time [ms] (Time limit: 65535ms).
        {//Time limit: 65535ms
                   //ConfigRampInc(queue[commandPosition].Payload.Byte[1], queue[commandPosition].Payload.Byte[0], queue[commandPosition].Payload.Byte[2]);
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 0;

            }break;

        case 0x34://Sets up the time duration the decreasing ramp of the chosen motor during at desired time [ms](Time limit: 65535ms).
            {//Time limit: 65535ms
                   //ConfigRampDec(queue[commandPosition].Payload.Byte[1], queue[commandPosition].Payload.Byte[0], queue[commandPosition].Payload.Byte[2]);
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 0;

            }break;

//*****************************************************************************
//Data request to robot from the raspberry

        case 0x40:  //Request sensors data
        {
                   unsigned long int aux;
                   IEC3bits.T9IE = 0;//Disable timer9 interrupt
                   T8CONbits.TON = 0;//Switch timer 8 OFF
                   aux = queue[commandPosition].Payload.Byte[2] & 0x000000FF;//H
                   aux = aux << 8;
                   aux = aux + (queue[commandPosition].Payload.Byte[1] & 0x000000FF);//LH
                   aux = aux << 8;
                   aux = aux + (queue[commandPosition].Payload.Byte[0] & 0x000000FF);//LL
                   aux = aux * 625000 / 1000;// Frequency * T[ms] / 1000 [ms / sec]
                   PR8 = aux & 0x0000FFFF;
                   PR9 = (aux >> 16) & 0x000000FF;
                   T8CONbits.TON = 1;       //Switch timer 8 ON
                   IFS3bits.T9IF = 0;       //clear old interrupt request.
                   IEC3bits.T9IE = 1;       // Enable timer9 interrupt

        } break;

        case 0x41://Don´t request sensors data
        {
                   T8CONbits.TON = 0;//Switch timer 8 OFF

        } break;

//*****************************************************************************
//Movement Command
        
//        case 0x70: //Command to describe every movement: go forward, backward, turn or follow a curve
//        {
//                   //float linearVel;
//                   //float angularVel;
//                   float Vleft;// Left track velocity
//                   float Vright;// Right track velocity
//
//                   //linearVel = speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity);
//                   //angularVel = speedCalculating(queue[commandPosition].Payload.Features.AngularVelocity);
//
//                   Vleft = queue[commandPosition].Payload.Features.LinearVelocity - (queue[commandPosition].Payload.Features.AngularVelocity * widthBetweenWheels / 2.0);
//                   Vright = queue[commandPosition].Payload.Features.LinearVelocity + (queue[commandPosition].Payload.Features.AngularVelocity * widthBetweenWheels / 2.0);
//
//                   if (queue[commandPosition].Payload.Features.LinearVelocity >= -0.001 && queue[commandPosition].Payload.Features.LinearVelocity <= 0.001)
//                   {
//                       //distanceTimeOrDegrees = 2;//Degrees
//                       //startAngle = (float)(UM6_Sensors.eulerYawAngle * EulerConstant);
//                       if (queue[commandPosition].Payload.Features.AngularVelocity >= 0x80000000)//Angular velocity
//                       {
//                           //rightOrLeft = 0;//Right
//                       }
//                       else
//                           //rightOrLeft = 1;//Left
//                   }
//                   else
//                       //distanceTimeOrDegrees = 0;//Distance
//
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;//Last Command duration = RunCommand
//                   //moveRobot(-Vleft, -Vright);
//
//        } break;

//Stop commands

        case 0x71: //Emergency Stop command (Main and flipper arms motors)
        {
                   //StopMainArmDriFlip(0x71);
                   queuePosition = 0;//Restarts the queue
                   emptyQueue = 1;//The queue is empty
                   currentCommandDuration = 0;
                   //LabyrinthMode = 0;
                   //EnableAvoidingObstaclesFunction = 0;

        } break;

        case 0x72: //Stop command (Main and flipper arms motors with ramp)
        {
                   //StopMainArmDriFlip(0x72);
                   emptyQueue = 1;//The queue is empty
                   currentCommandDuration = 30000;
                   //LabyrinthMode = 0;
                   //EnableAvoidingObstaclesFunction = 0;

        } break;

//*****************************************************************************
//IMU Command

        case 0x80: //Calibrating drift error. It takes 30 seconds
        {
                   //StopMainArmDriFlip(0x72);
                   currentCommandDuration = 0;
                   calibrateDriftError = 1;
        } break;

//*****************************************************************************
//Flipper arms commands

        case 0xD2:// Drives the flipper rotator motors forward with 75% of the full speed and achieves the angle of 184 degree.
            {
                   //FlipArm_Forward_SetSpeedAngle( 75 , 184.00 );
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 5000;// 5 sec

            } break;

        case 0xD3:// Drives the flipper rotator motors forward with 99% of the full speed and achieves the angle of 184 degree.
            {
                   //FlipArm_Backward_SetSpeedAngle( 99 , 184.00 );
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 5000;// 5 sec

            } break;

        case 0xD4:// Drives the flipper rotator motors forward with 99% of the full speed and achieves the angle 360 degree.
            {
                   //FlipArm_Forward_SetSpeedAngle( 99 , 360.00 );
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 5000;// 5 sec

            } break;

        case 0xD5:// Drives the flipper rotator motors backward with 75% of the full speed and achieves the angle 360 degree.
            {
                   //FlipArm_Backward_SetSpeedAngle( 75 , 360.00 );
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 5000;// 5 sec

            } break;

        case 0xD6:// Finds the home position of the flippers by driving them forward 30 degree and backward until the home position is found.
            {
                   //FlipArm_Home();
                   //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 5000;// 5 sec

            } break;

        case 0xD7:// Stops the flipper rotator motors.
            {
                    //FlipArm1_Stop();
                    //FlipArm2_Stop();
                    //distanceTimeOrDegrees = 1;//Time
                   currentCommandDuration = 0;//Last Command duration = RunCommand

            } break;
        
// <editor-fold defaultstate="collapsed" desc="Commands to move the robot at desired speed. Created by Huajie Wang">
//*****************************************************************************
//Right now are disabled

//        case 0xE1://Beide Mainmotor fahren nach vorne mit der bestimmten Geschwindigkeit
//            {    //The robot moves straight ahead at the desired speed.
//                //Main motor speed setting(execute speed)
//                MainMotormitGeschwindkeit(speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity));
//                distanceTimeOrDegrees = 0;//Distance
//                currentCommandDuration = queue[commandPosition].Payload.Features.Duration;//Last Command duration = RunCommand
//
//            } break;
//
//         case 0xE2://The robot moves bacwards at the desired speed.
//            {
//                MainMotormitGeschwindkeit(- speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity));//Main motor speed setting(execute speed)
//                distanceTimeOrDegrees = 0;//Distance
//                currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xE3://The robot turn right at the desired speed.
//            {
//                MainMotorRechtsAbbiegen(- speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity));//Main Motor turning right (Executed speed)
//                distanceTimeOrDegrees = 1;//Time
//                currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            }break;
//
//        case 0xE4://The robot turns left at the desired speed.
//            {
//                MainMotorLinksAbbiegen(- speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity));//Main Motor turning left (Executed speed)
//                distanceTimeOrDegrees = 1;//Time
//                currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            }break;
//
//        case 0xE5://The robot turn right along a curve with the desired speed and the given radius.
//            {
//                float DrehRadius,DrehWinkelGeschwindigkeit,MerlinGeschlink,MerlinGeschRechts,MerlinGesch;
//                //Turning radius, Rotational angular velocity
//                DrehRadius = - speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity);
//                DrehWinkelGeschwindigkeit = (float) queue[commandPosition].Payload.Features.AngularVelocity;
//
//                //Radiant in Degree wechseln
//                DrehWinkelGeschwindigkeit = DrehWinkelGeschwindigkeit/180 * PI;
//                MerlinGesch = DrehWinkelGeschwindigkeit * DrehRadius;
//
//                //Die Geschwindigkeit von linkem und rechtem Rad ermitteln
//                MerlinGeschlink = MerlinGesch * ( 1 - Robotbreit/( 2 * DrehRadius));
//
//                MerlinGeschRechts = MerlinGesch * ( 1 + Robotbreit/( 2 * DrehRadius));
//                MainMotorAbbiegenKurve(MerlinGeschlink,MerlinGeschRechts);
//
//                distanceTimeOrDegrees = 1;//Time
//                currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            }break;
//
//        case 0xE6://The robot turns left along a curve with the desired speed and the given radius.
//            {
//                float DrehRadius,DrehWinkelGeschwindigkeit,MerlinGeschlink,MerlinGeschRechts,MerlinGesch;
//                //Turning radius, Rotational angular velocity
//                DrehRadius = - speedCalculating(queue[commandPosition].Payload.Features.LinearVelocity);
//                DrehWinkelGeschwindigkeit = (float) queue[commandPosition].Payload.Features.AngularVelocity;
//
//                //Radiant in Degree wechseln
//                DrehWinkelGeschwindigkeit = DrehWinkelGeschwindigkeit/180 * PI;//Angular velocity
//                MerlinGesch = DrehWinkelGeschwindigkeit * DrehRadius;
//
//                //Die Geschwindigkeit von linkem und rechtem Rad ermitteln
//                MerlinGeschlink = MerlinGesch * ( 1 + Robotbreit/( 2 * DrehRadius));
//
//                MerlinGeschRechts = MerlinGesch * ( 1 - Robotbreit/( 2 * DrehRadius));
//
//                MainMotorAbbiegenKurve(MerlinGeschlink,MerlinGeschRechts);
//
//                distanceTimeOrDegrees = 1;//Time
//                currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            }break;

 // </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Commands to move the robot at fixed speed">
//*****************************************************************************
//Commands to move the robot at fixed speed

//        case 0xC0:   //both main motors run forward with 1/min
//            {
//                   GoForwardMainIncDec(0xC0, 0);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC1:   //increase speed of both main motors forward
//            {
//                   GoForwardMainIncDec(0xC1, 1);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC2:   //decrease speed of both main motors forward
//            {
//                   GoForwardMainIncDec(0xC2, 1);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC3:   // both main and flipper arm driver motors run
//                     // forward with 1/min
//            {
//                   GoForwardMainFlipIncDec(0xC3, 0);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC4:   // increase speed of both main and flipper arm
//                     // driver motors forward
//            {
//                   GoForwardMainFlipIncDec(0xC4, 1);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC5:   // decrease speed of both main and flipper arm
//                     // driver motors forward
//            {
//                   GoForwardMainFlipIncDec(0xC5, 1);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC6:   // Stop both main motors
//            {
//                   StopMainArmDriFlip(0xC6);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = 0;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC7:   //Stop flipper arm driver motors.
//            {
//                   StopMainArmDriFlip(0xC7);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = 0;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC8:   //Stop both main and flipper arm driver motors.
//            {
//                   StopMainArmDriFlip(0xC8);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = 0;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xC9:   // both main motors run backward with 1/min
//            {
//                   GoBackwardMainIncDec(0xC9, 0);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xCA:   //increase speed of both main motors backward
//            {
//                   GoBackwardMainIncDec(0xCA, 1);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xCB:   //decrease speed of both main motors backward
//            {
//                   GoBackwardMainIncDec(0xCB, 1);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xCC:   // turn the robot left with 1/min, left motor backward
//                     // right motor forward.
//            {
//                   TurnLeftMainIncDec(0xCC, 0);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xCD:   // increase speed of turning left.
//            {
//                   TurnLeftMainIncDec(0xCD, 3);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xCE:   // decrease speed of turning left.
//            {
//                   TurnLeftMainIncDec(0xCE, 3);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//
//        case 0xCF:   // turn the robot right with 1/min, right motor backward
//                     // left motor forward.
//            {
//                   TurnRightMainIncDec(0xCF, 0);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xD0:   // increase speed of turning right.
//            {
//                   TurnRightMainIncDec(0xD0, 3);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
//        case 0xD1:   // decrease speed of turning right.
//            {
//                   TurnRightMainIncDec(0xD1, 3);
//                   distanceTimeOrDegrees = 1;//Time
//                   currentCommandDuration = queue[commandPosition].Payload.Features.Duration;;//Last Command duration = RunCommand
//
//            } break;
//
// </editor-fold>

         default:
            {

            }break;

    }//end switch

}

void send_attitude()
{
        messageToTransmit.PayloadSize = 2 + 6;  //
        messageToTransmit.PacketType = 0xA6;
        messageToTransmit.Payload.Byte[0] = UM7_Sensors.eulerRollAngle  >> 8;// High
        messageToTransmit.Payload.Byte[1] = UM7_Sensors.eulerRollAngle;// Low
        messageToTransmit.Payload.Byte[2] = UM7_Sensors.eulerYawAngle >> 8;
        messageToTransmit.Payload.Byte[3] = UM7_Sensors.eulerYawAngle;
        messageToTransmit.Payload.Byte[4] = UM7_Sensors.eulerPitchAngle >> 8;
        messageToTransmit.Payload.Byte[5] = UM7_Sensors.eulerPitchAngle;
        //sendDataToSerial_DMA();
}





