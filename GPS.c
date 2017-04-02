/****************************************************************************
 * File:   GPS.c
 * Created by: Alejandro Perez on 20.08.2015

 * Description: This file contains set up of UART communication between
 * the microcontroller and the GPS unit. This file also includes functions
 * pertaining to data retreival from GPS unit and position control using
 * retreived data and user inputs.

 * Last Modified: 14.06.2016 (MW)
****************************************************************************/

#include "GPS.h"
#include "p33EP512MU810.h"
#include "RS232.h"
#include "UM7.h"

#define _ADDED_C_LIB 1 // Needed to get snprintf()
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libpic30.h>
#include <math.h>
#include "Mixer.h"
#include "Timer.h"
#include <stdarg.h>

#define MAX_NAV_SPEED 3            //knots   1 knot = 1.852 km/hr
#define MAX_ANGLE_CORRECTION 30     //degrees maximum axis correction

//switches
#define useGndCourse 0
#define useEulerYaw 1

//*** Global variables ******************************************************
extern char uart1RxBuff[GPS_RxBufSize];       
extern char uart1TxBuff[GPS_TxBufSize];

char GPSRx[GPS_RxBufSize];

float phi1=0;           //latitude position
float phi2=0;           //latitude setpoint
float lamda1=0;         //longitude position
float lamda2=0;         //longitude position
float d_phi = 0;        //difference in lattitude
float d_lamda = 0;      //difference in longitude
float craftRollSpeed = 0;
float craftPitchSpeed = 0;


/****************************************************************************
 * Function Name: _DMA7Interrupt
 * Created by: Alejandro Perez on 01.08.2015
 
 * Description:  DMA Interrupt is called when entire UART buffer is received.
 * After buffer is received, the data can be parsed and stored.
 
 * Inputs:  void 
 * Returns: void
 * Calls:   parseNMEA(), storeGPScommdata()
 
 * Last Modified: 28.11.2015 (MW)
****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA7Interrupt(void)
{
    //if (parseNMEA())
    parseNMEA();
        storeGPScommdata(ptr_GPScommdata);
    
    IFS4bits.DMA7IF = 0;        // Clear the DMA7 Interrupt Flag
}

/****************************************************************************
  Function Name: _DM71Interrupt
  Description:   UART1 transmission complete
  Inputs:        None
  Returns:       None
****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA8Interrupt(void)
{
    IFS7bits.DMA8IF = 0;        // Clear the DMA8 Interrupt Flag
    
}

/****************************************************************************
 * Function Name: cfgDMA7_UART_Rx
 * Created by: Alejandro Perez on ??.08.2015
 
 * Description:  Configuration for DMA7 in conjunction with UART1 Rx Channel.
 * Read from UART1 Receive register and write to DMA RAM. 
 * NOTE: All DMAxCON bits are 0 value by default.
 
 * Inputs:  void 
 * Returns: void
 * Calls:   none
 
 * Last Modified: 11.22.2015 (MW)
****************************************************************************/
void cfgDMA7_UART_Rx(void)
{
    
    DMA7CONbits.CHEN = 0;           //Disable DMA Channel			
    DMA7CONbits.SIZE = 1;           //0: Word, 1: Byte
    IEC4bits.DMA7IE  = 0;			//Disable DMA interrupt
    IFS4bits.DMA7IF  = 0;			//Clear DMA interrupt
    
	DMA7CNT = GPS_RxBufSize-1;        //DMA interrupt occurs after receiving of entire buffer
	DMA7REQbits.IRQSEL = 0b00001011;    // UART1RX  UART1 Receiver (00001011) [DS70616G-p160]
	DMA7PAD = (volatile unsigned int) &U1RXREG;
    DMA7STAL = __builtin_dmaoffset(uart1RxBuff);
    
    //_DMA7IP = 4;                  //Interrupt Priority
	IEC4bits.DMA7IE  = 1;			// Enable DMA interrupt
    DMA7CONbits.CHEN = 1;			// Enable Receiving-DMA Channel
}


/****************************************************************************
 * Function Name: cfgDMA8_UART_Tx
 * Created by: Alejandro Perez on 01.08.2015
 
 * Description:  Configuration for DMA8 in conjunction with UART1 Tx Channel.
 * Read from DMA RAM and write to UART1 Transmit register
 * NOTE: All DMAxCON bits are 0 value by default.
 
 * Inputs:  void 
 * Returns: void
 * Calls:   none
 
 * Last Modified: 11.22.2015 (MW)
****************************************************************************/
void cfgDMA8_UART_Tx(void)
{
    IEC7bits.DMA8IE  = 0;
    IFS7bits.DMA8IF  = 0;                       // Clear DMA interrupt

	DMA8CON = 0x2001;                           // One-Shot, Post-Increment, RAM-to-Peripheral (10000000000001)
    DMA8CONbits.SIZE = 1;                       // 0 = Word, 1 = Byte
	DMA8CNT = GPS_TxBufSize-1;                // DMA1 Transfer Count Register, Anzahl DMA-Transfers = CNT +1 (7 DMA-Requests)
                
	DMA8REQbits.IRQSEL = 0b00001100;            // UART1TX 3 UART1 Transmitter (00001100) [DS70616G-p160]
    DMA8REQbits.FORCE = 0;                      // Automatic/Manual Start

	DMA8PAD = (volatile unsigned int) &U1TXREG; // Peripheral Address Register, siehe Datenblatt DMA, S.42, Example 22-10
    
	DMA8STAL = __builtin_dmaoffset(uart1TxBuff);// In that way one character is transmiting everytime, Start Address Register
    DMA8STAH = 0x0000;                          // primary Start Address bits (source or destination) Expample 22-10
    
    IEC7bits.DMA8IE  = 1;                       // Enable DMA interrupt
    DMA8CONbits.CHEN = 1;                       // Enable DMA channel (Transmit)
                                                //nach Ende des Datenblocks wird das CHEN-Bit automatisch wieder auf 0 gesetzt
}

/****************************************************************************
 * Function Name: cfgUART1
 * Created by: Alejandro Perez on 01.08.2015
 
 * Description:  Configuration for UART1 Channel.
 * NOTE: All writeable U1MODE bits and U1STA bits are 0 value by default.
 
 * Inputs:  void 
 * Returns: void
 * Calls:   none
 
 * Last Modified: 11.22.2015 (MW)
****************************************************************************/
void cfgUART1(void)                     //UART1 Configuration for RX und TX
{

    U1MODEbits.UARTEN = 0;              //Disable UART1 Channel
    //U1MODEbits.LPBACK  = 1;           //Enable Loop Back Mode (DEBUG ONLY)
    
	U1BRG = BRGVAL_GPS;                 //U1BRD = ((FCY/BAUDRATE_GPS)/16)-1
    
    U1MODEbits.UARTEN = 1;              //Enable UART1
    U1STAbits.UTXEN = 1;                //Enable UART1 Tx
}

/****************************************************************************
 * Function Name: initGPScommunication
 * Created by: Alejandro Perez on 01.08.2015
 
 * Description:  Initialize UART1 configuration and DMA7 and DMA8 for 
 * communication with GPS Receiver
 
 * Inputs:  void 
 * Returns: void
 * Calls:   cfgUART1(), cfgDMA8_UART_Tx, cfgDMA7_UART_Rx
 
 * Last Modified: 11.22.2015 (MW)
****************************************************************************/
void initGPScommunication(void)
{
    cfgUART1();
    cfgDMA8_UART_Tx();
    cfgDMA7_UART_Rx();
}


/****************************************************************************
 * Function Name: parseNMEA
 * Created by: Matt Woodard on 28.11.2015
 
 * Description:  Uses "circular buffer" approach to remove data directly
 * from UART1 Rx Buffer starting with first NMEA character '$' and ending with
 * last NMEA character '*'. The NMEA sentence is then placed in GPSRx buffer
 * starting with position 0. Returns 1 or 0 depending on whether the calculated
 * checksum matches the checksum read from the NMEA sentence or not.
 
 * Inputs:  void 
 * Returns: int
 * Calls:   calcChecksum()
 
 * Last Modified: 04.12.2015 (MW)
****************************************************************************/
int parseNMEA(void)
{
    unsigned int i = 0;
    unsigned int j = 0;
    unsigned int startCharFound = 0;
    unsigned int startCharPos = 0;
    unsigned int endCharFound = 0;
    unsigned int endCharPos = 0;
    int size=0;
    //char checkSum[2];
    //unsigned long check;

    
    //loop through buffer until start and end character both found
    while(!startCharFound || !endCharFound)
    {
        if(uart1RxBuff[i]=='$')
        {
            startCharFound = 1;
            startCharPos = i;
        }
        if (!startCharFound)
        {
            startCharPos++;
        }
        
        if((startCharFound) &&  uart1RxBuff[i]=='*')
        {
           endCharFound = 1; 
           endCharPos = i;
        }
        if (!endCharFound)
        {
            endCharPos++;
        }  
        
        if(i==GPS_RxBufSize)
            i=0;
        else
            i++;
    }
    
    size = endCharPos-startCharPos;   //size of message including '$' and '*'
    
    //if end char is in a position lower than start char, size will be negative
    if (size < 0)
        size = GPS_RxBufSize+size;
    
    //get all bytes including checksum (2 bytes after '*')
    for(j=0; j<=size+2; j++)      
    {
        GPSRx[j] = uart1RxBuff[startCharPos+j];
    }
    
    checkSum[0]=GPSRx[size+1];
    checkSum[1]=GPSRx[size+2];
    check = strtoul(checkSum, NULL, 16); //convert checksum character to long int
   
    
    //if calculated checksum matches read checksum, return 1, else return 0
    if (calcChecksum(size) == check)
        return 1;
    else
        return 0;
    
}

/****************************************************************************
 * Function Name: calcChecksum
 * Created by: Matt Woodard on 29.11.2015
 
 * Description:  Logical OR every character in NMEA sentences between, but not
 * including, the characters '$' and '*'. Return the checksum as an integer. 
 * Input parameter is the size of the buffer.
 
 * Inputs:  int 
 * Returns: int 
 * Calls:   None
 
 * Last Modified: 04.12.2015 (MW)
****************************************************************************/
long int calcChecksum(int mySize)
{
    int i = 0;
    unsigned int temp=0;       
    
    //loop through buffer chacters, starting at second character
    for(i=1; i<(mySize); i++)     
    {
        temp ^= GPSRx[i];   //XOR all characters between $ and *
    }
    
    return (temp);        

}

/****************************************************************************
 * Function Name: storeGPScommdata
 * Created by: Alejandro on 01.08.2015
 
 * Description: Breaks NMEA sentence into portions depending on type of data
 * and places it into structure 'GPS_Commun' so that the data can be more
 * easily referenced later in the code.
 
 * Inputs:  *GPS_Commun
 * Returns: void 
 * Calls:   None
 
 * Last Modified: 20.11.2015 (MW)
****************************************************************************/
void storeGPScommdata (GPS_Commun *data)
{
    char *pch;                              //character pointer
    const char d[2] = ",";                  //seperation character
    int GPRMCstring = 0;                    //message type flag
    
    
    //Copy Header
    pch = strtok(GPSRx,d);
    strcpy(data->header,pch);

    //NOTE: Further Development... make this an if, else structure to include
    //all possible NMEA sentence headers, with each having its own int value.
    //Is this GPGGA or GPRMC message
    if(!strcmp(data->header,"$GPRMC")) //if header = GPRMC
    {
        GPRMCstring = 1;
    }

    //Copy UTC Time
    pch = strtok(NULL,d);
    strcpy(data->UTC_Time,pch);

    //before 'latitude' the strings become different
    if(GPRMCstring != 1) //GPGGA string
    {
        pch = strtok(NULL,d);   //latitude
        strcpy(data->latitude,pch);
    }
    else
    {
        pch = strtok(NULL,d); //status
        pch = strtok(NULL,d);   //latitude
        strcpy(data->latitude,pch);
    }
    //after 'latitude' they are equal again


    pch = strtok(NULL,d);  //North or South
    strcpy(data->NS_ind,pch);
    pch = strtok(NULL,d);  //longitutde 
    strcpy(data->longitude,pch);

    pch = strtok(NULL,d); //East or West
    strcpy(data->EW_ind,pch);

    //here the strings become different
    if(!GPRMCstring)
    {
        pch = strtok(NULL,d); //Quality Indicator
        strcpy(data->GPS_quality_ind,pch);
        pch = strtok(NULL,d); //Satellites Used
        strcpy(data->Nsatellites,pch);
        pch = strtok(NULL,d); //HDOP
        strcpy(data->HDOP,pch);
        pch = strtok(NULL,d);
        strcpy(data->altitude,pch); //altitude
    }
    else
    {
        pch = strtok(NULL,d); //Ground Speed
        strcpy(data->groundSpeed,pch);
        pch = strtok(NULL,d); //Ground Course
        strcpy(data->groundCourse,pch);
    }
             
    //checksum data still neeeds to be added
    
}


/****************************************************************************
 * Function Name: storeGPSdata
 * Created by: Matt Woodard on 29.1.2015
 
 * Description: This function sets up the resultant data from the GPS so that
 * it is ready for manipulation by the control algorithm. i.e. changing 
 * characters to floats, etc.
 
 * Inputs:  *GPSData 
 * Returns: None 
 * Calls:   None
 
 * Last Modified: 22.03.2016 (MW)
****************************************************************************/
void storeGPSdata (GPSData *data, GPS_Commun *comm)
{
    char deg_la[3];
    char deg_lo[4];
    char min[8];
    int degrees=0;
    float minutes=0;
    
    if((timer.s % 15) == 0) //every 20 seconds save current data to prev data to compare later
    {
        data->prev_longitude = data->longitude;
        data->prev_latitude = data->latitude;
    }
    
    //need to convert to decimal degrees format and convert to float
    snprintf(deg_lo,4,comm->longitude);
    degrees = atoi(deg_lo)/100;     //not sure why this won't work without dividing by 100
    snprintf(min,8,&comm->longitude[3]);
    minutes = atof(min);
    data->longitude = degrees + minutes/60;
    
    snprintf(deg_la,3,comm->latitude);
    degrees = atoi(deg_la);
    snprintf(min,8,&comm->latitude[2]);
    minutes = atof(min);
    data->latitude = degrees + minutes/60;
    
    
    //data->latitude = 50.905206;                  //DEBUG ONLY
    //data->longitude = 8.029039;                 //DEBUG ONLY
    
    
    data->altitude = atof(comm->altitude);
    
    data->groundCourse = atof(comm->groundCourse);
    //put in +/-180° format
    if(data->groundCourse>180)
      data->groundCourse = (data->groundCourse-360);   
    
    data->groundSpeed =  atof(comm->groundSpeed);
    
    if(comm->EW_ind[0] == 'W')
        data->longitude = data->longitude * -1;
    
    if(comm->NS_ind[0] == 'S')
        data->latitude = data->latitude * -1;

}

/****************************************************************************
 * Function Name: setCoordinates
 * Created by: Matt Woodard on 29.12.2015
 
 * Description: stores current GPS values as setpoints for home position
 
 * Inputs:  *GPSData 
 * Returns: None 
 * Calls:   None
 
 * Last Modified: 04.01.2016 (MW)
****************************************************************************/
void setCoordinates(GPSData *dataStore)
{
    //dataStore->latitude_setpoint = dataStore->latitude;
    //dataStore->longitude_setpoint = dataStore->longitude;
    //dataStore->latitude_setpoint = 50.906618;         //DEBUG ONLY
    //dataStore->longitude_setpoint = 8.024381;        //DEBUG ONLY 
    dataStore->latitude_setpoint = 50.903095;         //DEBUG ONLY
    dataStore->longitude_setpoint = 8.029454;        //DEBUG ONLY 
    dataStore->altitude_setpoint = dataStore->altitude;
    //dataStore->direction_setpoint = dataStore->groundCourse;
    dataStore->speed_setpoint = dataStore->groundSpeed;
}


/****************************************************************************
 * Function Name: calcDistance
 * Created by: Matt Woodard on 29.02.2016
 
 * Description:  This function calculates the distance between the quad's
 * current position and the setpoint position. This is done using the haversine
 * formula, which is as follows:
 * 
 * d = R*c, where 
 * c = 2*atan2(sqrt(a),sqrt(1-a)), where
 * a = sin²(dphi/2)+cos(phi1)*cos(phi2)*sin²(dlamda/2)
 * 
 * for further reference visit:
 * http://www.movable-type.co.uk/scripts/latlong.html
 * 
 
 * Inputs:  None 
 * Returns: float 
 * Calls:   None
 
 * Last Modified: 28.03.2016 (MW)
****************************************************************************/
float calcDistance()
{
    float distance=0;
    float R=6371000;              //earth radius
    float a=0;                    //'a' part of haversine formula
    float c=0;                    //'c' part of haversine formula
    
    //ptr_GPSdata->latitude = 50.905206;                  //DEBUG ONLY
    //ptr_GPSdata->longitude = 8.029039;                 //DEBUG ONLY
    //ptr_GPSdata->latitude_setpoint = 50.906618;         //DEBUG ONLY
   //ptr_GPSdata->longitude_setpoint = 8.024381;        //DEBUG ONLY 
    
    phi1=GPSdata.latitude*(M_PI/180);             //lat1 in radians
    phi2=GPSdata.latitude_setpoint*(M_PI/180);    //lat2 in radians
    lamda1=GPSdata.longitude*(M_PI/180);          //lon1 in radians
    lamda2=GPSdata.longitude_setpoint*(M_PI/180); //lon2 in radians
 

    d_phi = (GPSdata.latitude_setpoint-GPSdata.latitude)*(M_PI/180);
    d_lamda = (GPSdata.longitude_setpoint-GPSdata.longitude)*(M_PI/180);
    
    a = sin(d_phi/2)*sin(d_phi/2)
        + 
        cos(phi1)*cos(phi2)*sin(d_lamda/2)*sin(d_lamda/2);
    
    c = 2*atan2(sqrt(a),sqrt(1-a));
    
    distance = R*c;
    
    return distance;
}

/****************************************************************************
 * Function Name: calcHeading
 * Created by: Matt Woodard on 29.02.2016
 
 * Description:  This function calculates the heading between the quad's
 * current position and the setpoint position.
 * 
 * for further reference visit:
 * http://www.movable-type.co.uk/scripts/latlong.html
 * 
 
 * Inputs:  None 
 * Returns: float 
 * Calls:   None
 
 * Last Modified: 28.03.2016 (MW)
****************************************************************************/
float calcHeading()
{
    double heading = 0;
  
    heading = atan2f(
            (sin(d_lamda)*cos(phi2))
            ,
            (cos(phi1)*sin(phi2)-sin(phi1)*cos(phi2)*cos(d_lamda))
            )*(180/M_PI);
    //heading is value between -180 and +180 degrees
    
    return heading;
}




/****************************************************************************
 * Function Name: turnToHeading
 * Created by: Matt Woodard on 15.03.2016
 
 * Description:  This function turns the quad to the heading which is calculated
 * in calcHeading by turning the quad about the Z axis (yaw rotation). The 
 * magnetic heading is compared against the desired heading. If the magnetic
 * heading is within a certain limit, then the quad is considered turned to the
 * proper heading and the function returns 1. If the quad is not turned to
 * the desired heading, a PID function is called whose output causes a yaw angle
 * movement, and 0 is returned.
 * 
 
 * Inputs:  *s_controller_fixedpoint 
 * Returns: int 
 * Calls:   calcDistance, calcHeading, calcMagHeading, PID_position
 
 * Last Modified: 28.03.2016 (MW)
 * 
 * NOTE: Magnetic heading doesn't necessarily match calculated ground course
 * heading. However, in Germany, the magnetic deviation is small enough that
 * we can considerate it negligible for this application of turning the quad to
 * a rough approximation of the correct direction.
****************************************************************************/
int turnToHeading(s_controller_fixedpoint *ctrlr)
{
//    float PID_out=0;
    float magHeading=0;
    float bearing=0;
    float correctedYawAngle=0;
    
    calcDistance(); //to get delta values
    bearing = calcHeading();  //to get bearing in +/-180°
    magHeading = calcMagHeading(); //to get compass value in +/-180°
    correctedYawAngle=ctrlr->actual_angle_rudd;
    
    
    if((correctedYawAngle < bearing+5) && (correctedYawAngle > bearing-5))
    {
        ptr_FIX->setpoint_angle_rudd = correctedYawAngle;
        return 1;
    }
    else
    {
        //PID_out = PID_position(bearing,magHeading,'c',ctrlr);
        //ctrlr->rudd_angle_target = PID_out; //correct heading error with yaw
        //ctrlr->actual_angle_rudd = magHeading;
        
        //how many degrees to turn?
        ctrlr->rudd_angle_target = 0;   //Any Pilot Requested Yaw is overridden
        ctrlr->setpoint_angle_rudd = bearing;
       // LATEbits.LATE8=0;
        
//        if(PID_out > ctrlr->rudd_Max_a)
//        {
//            ctrlr->rudd_output_a = ctrlr->rudd_Max_a;
//        }
//        else if(PID_out < -ctrlr->rudd_Max_a)
//        {
//            ctrlr->rudd_output_a = -ctrlr->rudd_Max_a;
//        }
//        else
//        {
//            ctrlr->rudd_output_a = PID_out;
//        }
        
        return 0;
    }
}

/****************************************************************************
 * Function Name: followHeading
 * Created by: Matt Woodard on 29.02.2016
 
 * Description:  This function follows the calculated required heading
 * between the quad and the desired setpoint. This is done by using the 
 * ground course feedback given from the GPS. The GPS ground course feedback
 * is much more stable than the calculated magnetometer heading.

 * Inputs:  *s_controller_fixedpoint 
 * Returns: int 
 * Calls:   calcDistance, calcHeading, PID_position
 
 * Last Modified: 28.03.2016 (MW)
****************************************************************************/
int followHeading(s_controller_fixedpoint *ctrlr)
{
//    float PID_out=0;
    float bearing=0;
    float gndCourse=0;
    float correctedYawAngle=0;
    
   //ptr_GPSdata->groundCourse = -65;      //DEBUG ONLY
    calcDistance(); //to get delta values
    bearing = calcHeading();  //to get bearing in +/-180°
    gndCourse = GPSdata.groundCourse;
    correctedYawAngle=ctrlr->actual_angle_rudd;
    
    if(useGndCourse)
    {
    
        if((gndCourse <= bearing+2) && (gndCourse >= bearing-2))
        {
            return 1;   //dead zone for following bearing
        }
        else if((gndCourse >= bearing+20)||(gndCourse <= bearing-20))
        {
            return 2;   //something is way off, quad not tracking correctly
        }
        else
        {
            //PID_out = PID_position(bearing,gndCourse,'c',ptr_FIX);
            ptr_FIX->rudd_angle_target = 0;     //Any Pilot Requested Yaw is overridden
            ctrlr->setpoint_angle_rudd = bearing;
            return 0;
        }
    }
    
    if(useEulerYaw)
    {
        if((correctedYawAngle <= bearing+5) && (correctedYawAngle >= bearing-5))
        {
            return 1;   //dead zone for following bearing
        }
        else if((correctedYawAngle >= bearing+20)||(correctedYawAngle <= bearing-20))
        {
            return 2;   //something is way off, quad not tracking correctly
        }
        else
        {
            //PID_out = PID_position(bearing,gndCourse,'c',ptr_FIX);
            ptr_FIX->rudd_angle_target = 0;     //Any Pilot Requested Yaw is overridden
            ctrlr->setpoint_angle_rudd = bearing;
            return 0;
        }
        
    }
    
}

int setHome(int GPSlockedStatus, GPSData *dataGPS)
{
    if(GPSlockedStatus)
    {
        //set current position as home position
        dataGPS->haltitude = dataGPS->altitude;
        dataGPS->hlatitude = dataGPS->latitude;
        dataGPS->hlongitude = dataGPS->longitude;     
        return 1;
    }
    else
    {
        //set home position to default position
        dataGPS->haltitude = defaultAlt;
        dataGPS->hlatitude = defaultLat;
        dataGPS->hlongitude = defaultLon;
        return 0;
    }
}

int checkGPSlock()
{
    //NOTE: these are the default values that the GPS gives
    //when it has no satellite lock. This is a real location
    //in the middle of Taiwan.
    if((int)GPSdata.latitude != 24 || (int)GPSdata.longitude != 121)
    {
        if((timer.s % 29) == 0)
        {
            if(fabs(GPSdata.latitude-GPSdata.prev_latitude)<latDeviation
            && fabs(GPSdata.longitude-GPSdata.prev_longitude)<lonDeviation)
            {
                return 1;
            }
        }
    }

    return 0;
}

/****************************************************************************
 *NOTE: The following two functions (GPSnavigate(),PitchRollCorrection())
 * are part of the Translational GPS Control Method investigated during
 * the studienarbeit of Matt W.
****************************************************************************/
/****************************************************************************
 * Function Name: GPSnavigate
 * Created by: Matt Woodard on 29.02.2016
 
 * Description:  This function is used as part of a sort of State Controller
 * in order to control the position of vehicle using GPS feedback.

 * Inputs:  void 
 * Returns: void 
 * Calls:   PitchRollCorrection()
 
 * Last Modified: 28.03.2016 (MW)
****************************************************************************/
void GPSnavigate()
{
    //PitchRollCorrection(calcDistance(),calcHeading());
}

/****************************************************************************
 * Function Name: PitchRollCorrection
 * Created by: Matt Woodard on 29.02.2016
 
 * Description:  This function makes the appropriate calculations to 
 * determine the course of the vehicle relative to true north and to determine
 * the current "pitch" and "roll" speed of the vehicle. Note taht these "pitch"
 * and "roll" speeds are only the true pitch and roll speed of the vehicle when
 * the front of the vehicle faces true north.

 * Inputs:  double, double 
 * Returns: void 
 * Calls:   None
 
 * Last Modified: 28.03.2016 (MW)
****************************************************************************/
void PitchRollCorrection(double distance, double heading)
{
    float radGndCourse = 0;
    float desiredRollSpeed = 0;
    float desiredPitchSpeed = 0;

    radGndCourse = (GPSdata.groundCourse)*(M_PI/180); 
    craftRollSpeed = sin((radGndCourse-heading)*GPSdata.groundSpeed);     
    craftPitchSpeed = cos((radGndCourse-heading)*GPSdata.groundSpeed); 
    
//    desiredRollSpeed = MAX_NAV_SPEED*sin(bearing-heading)*distance;
//    if(desiredRollSpeed>MAX_NAV_SPEED)
//        desiredRollSpeed = MAX_NAV_SPEED;
//    if(desiredRollSpeed<-MAX_NAV_SPEED)
//        desiredRollSpeed = -MAX_NAV_SPEED;
     
//    desiredPitchSpeed = MAX_NAV_SPEED*cos(bearing-heading)*distance;
//    if(desiredPitchSpeed>MAX_NAV_SPEED)
//        desiredPitchSpeed = MAX_NAV_SPEED;
//    if(desiredPitchSpeed<-MAX_NAV_SPEED)
//        desiredPitchSpeed = -MAX_NAV_SPEED;
//   
    
    gpsRollCorrection = desiredRollSpeed - craftRollSpeed;
    if(gpsRollCorrection>MAX_ANGLE_CORRECTION)
        gpsRollCorrection = MAX_ANGLE_CORRECTION;
    if(gpsRollCorrection<-MAX_ANGLE_CORRECTION)
        gpsRollCorrection = -MAX_ANGLE_CORRECTION;
    
    gpsPitchCorrection = desiredPitchSpeed - craftPitchSpeed;
    if(gpsPitchCorrection>MAX_ANGLE_CORRECTION)
        gpsPitchCorrection = MAX_ANGLE_CORRECTION;
    if(gpsPitchCorrection<-MAX_ANGLE_CORRECTION)
        gpsPitchCorrection = -MAX_ANGLE_CORRECTION;
 
}
//END GPS CONTROL TRANSLATIONAL METHOD 

/****************************************************************************
 *NOTE: The following commented code was an orignal GPS position controller
 * designed by Alejandro. It has not been tested, and this method was not
 * investigated or used during the creation of the other two methods for 
 * the GPS controller.
 * 
 * However, it should be noted that this method will not work if the yaw
 * orientation of the vehicle is unknown with respect to true north.
****************************************************************************/
/*
void move_right(s_controller_fixedpoint *controller_fix){

    controller_fix->setpoint_angle_aile = 10; //Set move on 10 degrees.
}

void move_left(s_controller_fixedpoint *controller_fix){

    controller_fix->setpoint_angle_aile = -10;
}

void move_front(s_controller_fixedpoint *controller_fix){

    controller_fix->setpoint_angle_elev = 10;
}

void move_back(s_controller_fixedpoint *controller_fix){

    controller_fix->setpoint_angle_elev = -10;
}


void latitude_controller(GPSData *data, s_controller_fixedpoint *controller_fix){

    data->pre_error_latitude = data->set_latitude - data->latitude;

    do
    {
        move_right(controller_fix);
        data->error_latitude = data->set_latitude - data->latitude;
    } while (data->error_latitude < data->pre_error_latitude);
    data->pre_error_latitude = data->error_latitude;

    do
    {
        move_back(controller_fix);
        data->error_latitude = data->set_latitude - data->latitude;
    } while (data->error_latitude < data->pre_error_latitude);
    data->pre_error_latitude = data->error_latitude;

    do
    {
        move_left(controller_fix);
        data->error_latitude = data->set_latitude - data->latitude;
    } while (data->error_latitude < data->pre_error_latitude);
    data->pre_error_latitude = data->error_latitude;

    do
    {
        move_front(controller_fix);
        data->error_latitude = data->set_latitude - data->latitude;
    } while (data->error_latitude < data->pre_error_latitude);
 }

void longitude_controller(GPSData *data, s_controller_fixedpoint *controller_fix){

    data->pre_error_longitude = data->set_longitude - data->longitude;

    do{
        move_right(controller_fix);
        data->error_longitude = data->set_longitude - data->longitude;
    } while (data->error_longitude < data->pre_error_longitude);
    data->pre_error_longitude = data->error_longitude;

    do{
        move_back(controller_fix);
        data->error_longitude = data->set_longitude - data->longitude;
    } while (data->error_longitude < data->pre_error_longitude);
    data->pre_error_longitude = data->error_longitude;

    do{
        move_left(controller_fix);
        data->error_longitude = data->set_longitude - data->longitude;
    } while (data->error_longitude < data->pre_error_longitude);
    data->pre_error_longitude = data->error_longitude;

    do{
        move_front(controller_fix);
        data->error_longitude = data->set_longitude - data->longitude;
    } while (data->error_longitude < data->pre_error_longitude);
 }


void SetHome(GPSData *data){

    data->hlatitude = data->latitude;
    data->hNS = data->NS;
    data->hlongitude = data->longitude;
    data->hEW = data->EW;
    data->haltitude = data->altitude;

    LATBbits.LATB3 = 1;
}

void BackHome(GPSData *data){

    data->set_latitude = data->latitude;
    data->set_NS = data->NS;
    data->set_longitude = data->longitude;
    data->set_EW = data->EW;
    data->set_altitude = data->altitude;

}*/
//END COMMENTED GPS CONTROLLER

/****************************************************************************
 * Function Name: getGPSsetpointRS232
 * Created by: Matt Woodard on 02.01.2016
 
 * Description: This function is used to take a latitude and longitude 
 * setpoint over RS232

 * Inputs:  *GPSData
 * Returns: None 
 * Calls:   None
 
 * Last Modified: 28.01.2016 (MW)
****************************************************************************/
void getGPSsetpointRS232(GPSData *RS232setpoint)
{
    double latitude_RS232 = 0;
    double longitude_RS232 = 0;
    
    char *pch;
    pch = strtok(GPSRx,",");
    latitude_RS232 = atof(pch);
    
    pch = strtok(NULL," ");
    longitude_RS232 = atof(pch);
    
    
    //LATBbits.LATB4 = 0;
    RS232setpoint->latitude_setpoint = latitude_RS232;
    RS232setpoint->longitude_setpoint = longitude_RS232;     
}

/****************************************************************************
 * Function Name: echoGPSsetpointRS232
 * Created by: Matt Woodard on 02.01.2016
 
 * Description: This function is used to display the latitude and longitude
 * setpoints over RS232 which will be used for the controller

 * Inputs:  *GPSData
 * Returns: None 
 * Calls:   None
 
 * Last Modified: 28.01.2016 (MW)
****************************************************************************/
void echoGPSsetpointRS232(GPSData *RS232setpoint)
{
    double latitude_RS232 = RS232setpoint->latitude_setpoint;
    double longitude_RS232 = RS232setpoint->longitude_setpoint;
    
    while(U3STAbits.TRMT == 0);
    
    sprintf(uart3TxBuff,"\r\nUser_Setpoint:     ");
    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1; 
    
    while(U3STAbits.TRMT == 0);
    
    sprintf(uart3TxBuff, "%04.4f,%05.4f", latitude_RS232,longitude_RS232);

    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1; 
}

/****************************************************************************
 * Function Name: echoGPSposition
 * Created by: Matt Woodard on 02.01.2016
 
 * Description: This function simply displays the current latitude and
 * longitude values read from the GPS

 * Inputs:  *GPSData
 * Returns: None 
 * Calls:   None
 
 * Last Modified: 28.01.2016 (MW)
****************************************************************************/
void echoGPSposition(GPSData *GPSposition)
{
    double latitude_GPS = GPSposition->latitude;
    double longitude_GPS = GPSposition->longitude;
    
    while(U3STAbits.TRMT == 0);
    
    sprintf(uart3TxBuff,"\r\nGPS_Result:        ");
    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1; 
    
    while(U3STAbits.TRMT == 0);
    
    sprintf(uart3TxBuff,"%04.4f,%05.4f", latitude_GPS,longitude_GPS);
    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1; 
}

/****************************************************************************
 * Function Name: echoGPSstringNMEA
 * Created by: Matt Woodard on 02.01.2016
 
 * Description: This function displays part of the NMEA string as read
 * from the GPS

 * Inputs:  *GPSData
 * Returns: None 
 * Calls:   None
 
 * Last Modified: 28.01.2016 (MW)
****************************************************************************/
void echoGPSstringNMEA(GPS_Commun NMEAstring)
{
    while(U3STAbits.TRMT == 0);
    
    //sprintf(uart3TxBuff,"\r\n%s,%s,%s,%s,%s,%s,%s",NMEAstring.header,NMEAstring.UTC_Time,
     //       NMEAstring.latitude,NMEAstring.NS_ind,NMEAstring.longitude,
     //       NMEAstring.EW_ind,NMEAstring.altitude);
    
    sprintf(uart3TxBuff,"\r\n%s,%s,%s,%s",NMEAstring.header,NMEAstring.UTC_Time,NMEAstring.latitude,NMEAstring.longitude);
    
    DMA1CONbits.CHEN = 1;                          
    DMA1REQbits.FORCE = 1;       
}

