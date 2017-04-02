/* 
 * File:   Filter.h
 * Author: LJ
 * Comments: This filter include different filters for the sensors.
 * Revision history: 
 */
#ifndef _FILTER_H_
#define _FILTER_H_

#include "matrix_calculator.h"
#define IIR_count 4

typedef struct Kalman_parameters
{
	float Kalman_dt;
	float Kalman_Q; // process noise covariance
	float Kalman_R; // meas. noise covariance
}Kalman_parameters;

//elements and matrixs for the MPL3115A2
extern matrix_with_dimension Kalman_P;
extern matrix_with_dimension Kalman_init_P; // The initial P = B * Q * B' would be used later in the recursion
extern matrix_with_dimension Kalman_X;
extern matrix_with_dimension Kalman_for_MPL3115A2_B;
extern matrix_with_dimension Kalman_for_MPL3115A2_A;
extern matrix_with_dimension Kalman_for_MPL3115A2_C;
extern matrix_with_dimension Kalman_for_MPL3115A2_I;
extern matrix_with_dimension Kalman_gain_MPL3115A2;
extern Kalman_parameters MPL3115A2_Kalman_Parameter;

extern int height_after_kalman;

typedef struct IIR_Coeff
{
    float IIR_Coeff_Set[IIR_count];
}IIR_Coeff;

typedef struct IIR_Filter_Elements
{
    float IIR_Eingang_float[IIR_count];
}IIR_Filter_Elements;

void Array_for_the_filter(IIR_Filter_Elements* I_F_E, float variable);
float IIR_Filter(IIR_Filter_Elements I_F_E, IIR_Coeff I_F_C);
void IIR_Coeff_Initial(IIR_Coeff* I_F_C ,float C[IIR_count]);
void Median_filter( float* a,  int length);

Kalman_parameters Kalman_parameters_init(float dt, float Q, float R);
void Kalman_MPL3115A2_init();
void Kalman_MPL3115A2_altitude();
void IMU_Z_ProzessAcc_filering();

#endif
