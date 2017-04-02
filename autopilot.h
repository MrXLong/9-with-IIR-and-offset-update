/* 
 * File:   autopilot.h
 * Author: Matt
 *
 * Created on 22. September 2016, 17:10
 */

#ifndef AUTOPILOT_H
#define	AUTOPILOT_H

#include "defines.h"

void setPitchAngle(float desired_angle);
void setRollAngle(float desired_angle);
void setYawAngle(float desired_angle);

void setThrottle(unsigned int desired_throttle);

void setPitchRate(float desired_rate);
void setRollRate(float desired_rate);
void setYawRate(float desired_rate);

float rad_to_degree(float radians);
float deg_to_radian(float radians);

void guidance_vert_init(void);
void simple_takeoff(unsigned int height_cm);
void run_hover_loop(unsigned int height_cm);
void guidance_vert_run();

void autopilot_periodic(void);

void land_now(void);

extern boolean takeoff_requested;
extern boolean land_requested;
#endif	/* AUTOPILOT_H */

