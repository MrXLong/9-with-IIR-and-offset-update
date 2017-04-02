/* 
 * File:   defines.h
 * Author: Matt
 *
 * Created on 18. August 2016, 14:01
 */

#include <stdint.h>
#include <stddef.h>

#ifndef DEFINES_H
#define	DEFINES_H

#define Bound(_x, _min, _max) { if (_x > _max) _x = _max; else if (_x < _min) _x = _min; }

#define M_PI 3.14159265358979323846

#define CHANNEL_UNUSED          0   // udb_pwIn[0], udb_pwOut[0], etc. are not used, but allow lazy code everywhere else  :)
#define CHANNEL_1               1
#define CHANNEL_2               2
#define CHANNEL_3               3
#define CHANNEL_4               4
#define CHANNEL_5               5
#define CHANNEL_6               6
#define CHANNEL_7               7
#define CHANNEL_8               8
#define CHANNEL_9               9
#define CHANNEL_10              10
#define CHANNEL_11              11
#define CHANNEL_12              12
#define CHANNEL_13              13
#define CHANNEL_14              14
#define CHANNEL_15              15
#define CHANNEL_16              16

typedef uint8_t boolean;
#define true                    1
#define false                   0

// UDB Types
struct bb   { uint8_t B0; uint8_t B1; };
struct bbbb { uint8_t B0; uint8_t B1; uint8_t B2; uint8_t B3; };
struct ww   { int16_t W0; int16_t W1; };
struct wwww { int16_t W0; int16_t W1; int16_t W2; int16_t W3; };
struct LL   { int32_t L0; int32_t L1; };

union intbb      { int16_t BB;  struct bb _; };
union uintbb     { uint16_t BB; struct bb _; };
union longbbbb   { int32_t WW;  struct ww _; struct bbbb __; };
union longww     { int32_t WW;  struct ww _; };
union longlongLL { int64_t LL;  struct LL _; struct wwww __; };

// UDB Constants
#define RMAX 16384 // 0b0100000000000000    // 1.0 in 2.14 fractional format

////////////////////////////////////////////////////////////////////////////////
// Raw Accelerometer and Gyroscope(rate) Values
//extern struct ADchannel udb_xaccel, udb_yaccel, udb_zaccel;// x, y, and z accelerometer channels
//extern struct ADchannel udb_xrate,  udb_yrate,  udb_zrate; // x, y, and z gyro channels
//extern struct ADchannel udb_vref;                          // reference voltage  

extern union intbb dcm_declination_angle;
extern int16_t udb_pwTrim[];  


// Negate VALUE if NEEDS_REVERSING is true
#define REVERSE_IF_NEEDED(NEEDS_REVERSING, VALUE) ((NEEDS_REVERSING) ? (-(VALUE)) : (VALUE))


// ALTITUDEHOLD_STABILIZED and ALTITUDEHOLD_WAYPOINT options
#define AH_NONE             0
#define AH_PITCH_ONLY       1
#define AH_FULL             3


// AIRFRAME_TYPE options
#define AIRFRAME_NONE       0
#define QUAD                1    // RST Quadrokopter
#define BOXCOPTER           2    // RST Boxcopter

// FAILSAFE_TYPE options
#define FAILSAFE_RTL                1
#define FAILSAFE_MAIN_FLIGHTPLAN    2

// FLIGHT_PLAN_TYPE options
#define FP_NONE                     0
#define FP_WAYPOINTS                1
#define FP_LOGO                     2

// TELEMETRY_OUTPUT_FORMAT options
#define SERIAL_NONE         0    // No serial data is sent
#define SERIAL_DEBUG        1    // UAV Dev Board debug info
#define SERIAL_ARDUSTATION  2    // Compatible with ArduStation
#define SERIAL_UDB          3    // Pete's efficient UAV Dev Board format
#define SERIAL_OSD_REMZIBI  4    // Output data formatted to use as input to a Remzibi OSD (only works with GPS_UBX)
#define SERIAL_OSD_IF       5    // Output data formatted to use as input to a IF OSD (only works with GPS_UBX)
#define SERIAL_MAGNETOMETER 6    // Debugging the magnetometer
#define SERIAL_UDB_EXTRA    7    // Extra Telemetry beyond that provided by SERIAL_UDB for higher bandwidth connections
#define SERIAL_CAM_TRACK    8    // Output Location in a format usable by a 2nd UDB to target its camera at this plane
#define SERIAL_MAVLINK      9    // The Micro Air Vehicle Link protocol from the PixHawk Project


#include "gain_variables.h"

// GNU compiler specific macros for specifically marking variables as unused
// If not using GNU, then these macros make no alteration to the code
#ifdef __GNUC__
#  define UNUSED(x) UNUSED_ ## x __attribute__((__unused__))
#else
#  define UNUSED(x) UNUSED_ ## x
#endif

#ifdef __GNUC__
#  define UNUSED_FUNCTION(x) __attribute__((__unused__)) UNUSED_ ## x
#else
#  define UNUSED_FUNCTION(x) UNUSED_ ## x
#endif


#endif	/* DEFINES_H */

