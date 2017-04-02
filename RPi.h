/****************************************************************************

 Header File:		RPi.h

 In this Header file the functions for the commands that are sent from the
 PC through the RS232 or the USB are defined.

****************************************************************************/
#ifndef __RPI_H
#define __RPI_H


/*** Adjusting Factors *****************************************************/
// Defining the factor for increasing or decreasing the speed
#define Speed_Factor         4

/*** Queue parameters ******************************************************/
#define SizeQueue         128

/*** Mobile Station variables definitions **********************************/
// Header protocol to send a command
#define CommandHeader               0x23
// Header protocol to send a report
#define ReportHeader                0x2A
// Vehicle identification number
#define VEHICLE_ID                  0x03
// Device identification  number
#define RPI_ID                      0x00
// Maximum message size
#define MaxDataSize                 0x0E

/*** Automode variables definition *****************************************/
// Creating the struct Features, inside of the union Payload, inside of a struct is possible get the data in different ways
typedef struct PayloadFeatures
{   //8 Bytes
    int Duration;
    float LinearVelocity;
    float AngularVelocity;
    float generic_float;
}Features;

union PayloadType
{   //8 Bytes
    //struct PayloadFeatures Features;
    Features Features;
    char Byte[10];
};

typedef struct MessageRS232
{// Auxiliary variable to safe and use the first message received by DMA
    unsigned char Header[2];
    unsigned char DestinationId;
    unsigned char PayloadSize;
    unsigned char SourceID;
    unsigned char PacketType;   //Command
    union PayloadType Payload;
    unsigned char Checksum;
}Message;

extern Message MessageReceived;
extern Message queue[SizeQueue];    // Incoming message ready to store or execute in real time
extern Message messageToTransmit;   // Message to send

//extern int queuePosition;
//extern int emptyQueue;      //Flag to indicate the queue is empty
//extern unsigned int store;  //Flag to enabled "1" or disabled "0" the access to the queue

extern unsigned int messagePosition;    // Pointer to indicate the position where the missing message begins
extern unsigned int messageSize;
//extern unsigned int bufferAorB;         // 0 means bufferA, 1 means bufferB

// this variable for Enabling Avoiding Cruching Walls Function
//extern unsigned char EnableAvoidingObstaclesFunction;
//extern unsigned char EnableDetectingClimpingStairsFunction;
//extern unsigned char LabyrinthMode;
//extern int currentCommandDuration;
//extern float startAngle;      //Angle at which the robot starts moving
//extern int rightOrLeft;       //Angular velocity direction: 0 = right, 1 = left
//extern int calibrateDriftError; //Variable to allow the calibration of the gyroscopes drift error
//xtern unsigned char len;
//extern unsigned char DATARS232[6];

//extern int key;     //Flag that enabled the use of the commands which can change
                    //the robot parameters, like the PID constants, etc
//extern int hold;    //Flag to hold the list
//extern int distanceTimeOrDegrees;//Space = 0, Time = 1, Degrees = 2
//extern float labyrinthSpeed;

/****************************************************************************
Function:     Commun_RobotRaspberryPi
Parameters:   none
Return value: none
This function implementates the transmission protocol with the main RaspberryPi.
****************************************************************************/
void Commun_RobotRaspberryPi();

/****************************************************************************
Function:     CommandList
Parameters:   commandPosition
Return value: none
This function is switch cases  function used to check the sequential commands.
****************************************************************************/
void CommandList(unsigned int commandPosition);

void send_gyro(void);
void send_mag(void);
void send_attitude(void);



#endif //__RPI_H
