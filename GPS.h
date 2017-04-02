/* 
 * File:   GPS.h
 * Author: Alejandro
 *
 * Created on 20. August 2015, 13:51
 */

#ifndef GPS_H
#define	GPS_H

#include "PID.h"

#define FCY 40000000ULL


#define BAUDRATE_GPS 38400
#define BRGVAL_GPS   ((FCY/BAUDRATE_GPS)/16)-1

// Size of the buffer for received data (in Byte)
//#define GPS_TxBufSize                           24
#define GPS_TxBufSize                           10
// Size of the buffer for received data (in Byte)
//#define GPS_RxBufSize                           77
#define GPS_RxBufSize                           256

#define defaultLat  50.906618
#define defaultLon  8.024381
#define defaultAlt  0

#define latDeviation        0.000003
#define lonDeviation        0.000003

//#endif	/* GPS_H */

typedef struct communicationGPS{// Struct to safe and use the sensor data from GPS
//GPS format
//$GPGGA,hhmmss.sss,ddmm.mmmm,a,dddmm.mmmm,a,x,xx,x.x,x.x,M,,,,xxxx*hh<CR><LF>
//$GPGGA,111636.932,2447.0949,N,12100.5223,E,1,11,0.8,118.2,M,,,,0000*02<CR><LF>
    
//$GPRMC,hhmmss.sss,A,dddmm.mmmm,a,dddmm.mmmm,a,x.x,x.x,ddmmyy,,,a*hh<CR><LF>
//$GPRMC,111636.932,A,2447.0949,N,12100.5223,E,000.0,000.0,030407,,,A*61<CR><LF>

                  
    //latitude format ddmm.mmmm
    //longitude format dddmm.mmmm || Byte array
    
    char header[7];
    char UTC_Time[11];  //  8-17
    //unsigned int latitude_[3]; // 19-22
    //unsigned int _latitude[3]; // 24-27
    char latitude[10]; //   19-27
    char NS_ind[2];       // 29
    //unsigned int longitude_[4];// 31-35
    //unsigned int _longitude[3];// 37-40
    char longitude[11];     //31-40
    char EW_ind[2];       // 42
    char GPS_quality_ind[2];        // 44
    char Nsatellites[3];         // 46-47
    char HDOP[5];      // 49-51
    //unsigned int altitude_[4]; // 53-55
    //unsigned int _altitude;    // 57
    char altitude[8];
    char groundSpeed[5];
    char groundCourse[5];
    
    char checksum[3];

 }GPS_Commun;

 extern GPS_Commun GPScommdata;
 extern GPS_Commun *ptr_GPScommdata;

extern char GPSRx[GPS_RxBufSize];
extern char uart1RxBuff[GPS_RxBufSize];       

extern unsigned long temp;
extern unsigned long check;
extern char checkSum[2];

extern int gpsRollCorrection;
extern int gpsPitchCorrection;


typedef struct GPSDataSensor{ // Struct to save and use the sensor data from GPS
    //actual values
    double latitude;
    double prev_latitude;
    double longitude;
    double prev_longitude;
    float altitude;
    
    float groundSpeed;
    float groundCourse;

    //home
    int home_f;
    double hlatitude;
    double hlongitude;
    double haltitude;

    double latitude_setpoint;
    
    double longitude_setpoint;
    double altitude_setpoint;
    double speed_setpoint;
    double direction_setpoint;
    
    float set_altitude;     //Not sure if this is used elswhere but can be deleted

}GPSData;

extern GPSData GPSdata;
extern GPSData *ptr_GPSdata;

void __attribute__((interrupt, no_auto_psv)) _DMA7Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _DMA8Interrupt(void);
void cfgDMA7_UART_Rx(void);
void cfgDMA8_UART_Tx(void);
void cfgUART1(void);
void initGPScommunication(void);
void storeGPScommdata (GPS_Commun *data);
void storeGPSdata (GPSData *data, GPS_Commun *comm);
void move_right(s_controller_fixedpoint *controller_fix);
void move_left(s_controller_fixedpoint *controller_fix);
void move_front(s_controller_fixedpoint *controller_fix);
void move_back(s_controller_fixedpoint *controller_fix);
int parseNMEA(void);
long int calcChecksum(int);
void setCoordinates(GPSData *dataStore);
//void latitude_controller(GPSData *data, s_controller_fixedpoint *controller_fix);
//void longitude_controller(GPSData *data, s_controller_fixedpoint *controller_fix);
//void SetHome(GPSData *data);
//void BackHome(GPSData *data);
float calcDistance();
float calcHeading();
void GPSnavigate();
void PitchRollCorrection(double, double);

void getGPSsetpointRS232(GPSData *RS232setpoint);
void echoGPSsetpointRS232(GPSData *RS232setpoint);
void echoGPSposition(GPSData *GPSposition);
void echoGPSstringNMEA(GPS_Commun NMEAstring);
int turnToHeading(s_controller_fixedpoint *ctrlr);
int followHeading(s_controller_fixedpoint *ctrlr);

int setHome(int GPSlockedStatus, GPSData *dataGPS);
int checkGPSlock();

#endif	/* GPS_H */
