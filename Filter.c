/*
 * File:   Filter.c
 * Author: LJ
 *
 * Created on February 3, 2017, 10:13 AM
 */


#include "Filter.h"
#include "matrix_calculator.h"
#include "MPL3115A2_Barometer.h"
#include "UM7.h"
#include "UART1_K6.h"
#include <stdlib.h>

//elements and matrixs for the MPL3115A2
matrix_with_dimension Kalman_P;
matrix_with_dimension Kalman_init_P; // The initial P = B * Q * B' would be used later in the recursion
matrix_with_dimension Kalman_X;
matrix_with_dimension Kalman_for_MPL3115A2_B;
matrix_with_dimension Kalman_for_MPL3115A2_A;
matrix_with_dimension Kalman_for_MPL3115A2_C;
matrix_with_dimension Kalman_for_MPL3115A2_I;
matrix_with_dimension Kalman_gain_MPL3115A2;
Kalman_parameters MPL3115A2_Kalman_Parameter;

int height_after_kalman;


/*
 This IIR filter function return void, means better set the parameters as global variables. 
 */
float IIR_Filter( IIR_Filter_Elements I_F_E, IIR_Coeff I_F_C)
{
    float temp;
    int i = 0;
    
    for(i = 0; i < IIR_count ; i++)
    {
        temp = I_F_E.IIR_Eingang_float[i] * I_F_C.IIR_Coeff_Set[i] + temp;     
    }
    return temp;
}

/*
 This function return void, helps to initial the coefficient for the IIR Filter . 
 */
void IIR_Coeff_Initial(IIR_Coeff* I_F_C ,float C[IIR_count])
{
    int i = 0;
    for(i = 0; i < IIR_count - 1; i++)
    {
        I_F_C->IIR_Coeff_Set[i] = C[i];
    }
}

/*
 This function helps to refresh the array which would be used in the filter. It helps to save the new data in and delete the first data (FIFO). 
 */
void Array_for_the_filter(IIR_Filter_Elements* I_F_E, float variable)
{
    int i;
    for(i = 0; i < IIR_count - 1; i++)
    {
        I_F_E->IIR_Eingang_float[i] = I_F_E->IIR_Eingang_float[i+1]; 
    }
    I_F_E->IIR_Eingang_float[IIR_count - 1] = variable;
    
}

 
//This function helps to throw away the extreme data and get an average from the rest.
void Median_filter( float* a, int length)
{
    int i,j;
    float t;
    for(j=0;j<length;j++)
    { 
        for(i=0;i<length-1-j;i++)
        {
            if(*(a + i) < *(a + i + 1)) 
            {
                t=*(a + i);
                *(a + i) = *(a + i + 1);
                *(a + i + 1) = t;
            }
        }
    }
}


//Kalman Filter
Kalman_parameters Kalman_parameters_init(float dt, float Q, float R)
{
    Kalman_parameters K_Parameter;
    K_Parameter.Kalman_Q = Q;
    K_Parameter.Kalman_R = R;
    K_Parameter.Kalman_dt = dt;
    return K_Parameter;
}

void Kalman_MPL3115A2_init()
{
    matrix_with_dimension temp1, temp2;
    
    MPL3115A2_Kalman_Parameter = Kalman_parameters_init(0.15, 2, 30); 
    //Kalman_for_MPL3115A2_B.matrix = {{(MPL3115A2_Kalman_Parameter.Kalman_dt^2)/2, 0, 0}, {MPL3115A2_Kalman_Parameter.Kalman_dt, 0, 0}, {0, 0, 0}};
    Kalman_for_MPL3115A2_B.matrix[0][0] = (MPL3115A2_Kalman_Parameter.Kalman_dt * MPL3115A2_Kalman_Parameter.Kalman_dt)/2;
    Kalman_for_MPL3115A2_B.matrix[1][0] = MPL3115A2_Kalman_Parameter.Kalman_dt;    
    Kalman_for_MPL3115A2_B.row = 2;
    Kalman_for_MPL3115A2_B.column = 1;
    
    //Kalman_for_MPL3115A2_A.matrix = {{1, MPL3115A2_Kalman_Parameter.Kalman_dt},{0,1}};
    Kalman_for_MPL3115A2_A.matrix[0][0] = 1;
    Kalman_for_MPL3115A2_A.matrix[0][1] = MPL3115A2_Kalman_Parameter.Kalman_dt;
    //Kalman_for_MPL3115A2_B.matrix[1][0] = 
    Kalman_for_MPL3115A2_A.matrix[1][1] = 1;
    Kalman_for_MPL3115A2_A.row = 2;
    Kalman_for_MPL3115A2_A.column = 2;
    
    //Kalman_for_MPL3115A2_C.matrix = {1,0};
    Kalman_for_MPL3115A2_C.matrix[0][0] = 1;
    Kalman_for_MPL3115A2_C.row = 1;
    Kalman_for_MPL3115A2_C.column = 2;
    
    //Kalman_X.matrix = {0,0};
    Kalman_X.row = 2;
    Kalman_X.column = 1;
    
    //Kalman_gain_MPL3115A2.matrix = {0,0};
    Kalman_gain_MPL3115A2.row = 2;
    Kalman_gain_MPL3115A2.column = 1;
    
    Kalman_P.column = 2;
    Kalman_P.row = 2;
    temp1 = Matrix_scalar( Kalman_for_MPL3115A2_B, MPL3115A2_Kalman_Parameter.Kalman_Q); 
    temp2 = Matrix_transpose(Kalman_for_MPL3115A2_B);
    Kalman_P = Matrix_multiplication(temp1,  temp2);
    Kalman_init_P = Kalman_P;
    
    //Kalman_for_MPL3115A2_I.matrix = {{1,0},{0,1}};
    //Kalman_for_MPL3115A2_I.row = 2;
    //Kalman_for_MPL3115A2_I.column = 2;
    Kalman_for_MPL3115A2_I = Eye_creater(2);
}

void IMU_Z_ProzessAcc_filering()
{
    if(Kalman_X.matrix[1][0] < 0)
    {
        if(UM7_Sensors.processedAcc_Z < 0)
        {
            if(abs(UM7_Sensors.processedAcc_Z) < 8){UM7_Sensors.processedAcc_Z = 0;} 
        }
    }
    if(Kalman_X.matrix[1][0] > 0)
    {
        if(UM7_Sensors.processedAcc_Z > 0)
        {
            if(abs(UM7_Sensors.processedAcc_Z) < 8){UM7_Sensors.processedAcc_Z = 0;} 
        }
    }
}

void Kalman_MPL3115A2_altitude()
{

    //Measurement update
    float i;
    matrix_with_dimension temp1,temp2;
    
    // Kalman_gain_MPL3115A2 = P * C'/(C*P*C' + R)
    temp1 = Matrix_transpose(Kalman_for_MPL3115A2_C);
    temp1 = Matrix_multiplication(Kalman_P, temp1);
    temp2 = AVA(Kalman_for_MPL3115A2_C, Kalman_P);
    i = temp2.matrix[0][0] + MPL3115A2_Kalman_Parameter.Kalman_R;
    i = 1/i;
    Kalman_gain_MPL3115A2 = Matrix_scalar(temp1,i);
    
    // state vector X_estimate = X_predict + Kalman_gain_MPL3115A2 * (measurement - C*X_predict)   here we use the same X as to save some space
    temp1 = Matrix_multiplication(Kalman_for_MPL3115A2_C, Kalman_X);
    i = MPL_Barometer_Dist_cm - temp1.matrix[0][0];
    temp1 = Matrix_scalar(Kalman_gain_MPL3115A2, i); 
    Kalman_X = Matrix_plus_minus(Kalman_X, temp1,1);
    
    // covariance update  P = (I - Kalman_gain_MPL3115A2 * C) * P
    temp1 = Matrix_multiplication(Kalman_gain_MPL3115A2, Kalman_for_MPL3115A2_C);
    temp1 = Matrix_plus_minus(Kalman_for_MPL3115A2_I, temp1,0);
    Kalman_P = Matrix_multiplication(temp1, Kalman_P);
    
    height_after_kalman = (int)(temp1.matrix[0][0] + 0.5);
//    i = (int)(Kalman_X.matrix[0][0] + 0.5);
//    output_with_K6(i, 5, 1, 1);
    
    // time update
    // x = A * X + B* U 
    // P = A*P*A' + B * Q *B'
//    IMU_Z_ProzessAcc_filering();
    IMU_Z_ProzessAcc_filering();
    temp1 = Matrix_multiplication(Kalman_for_MPL3115A2_A, Kalman_X);
    temp2 = Matrix_scalar(Kalman_for_MPL3115A2_B, UM7_Sensors.processedAcc_Z); // wait the right input of ACC-z
    Kalman_X = Matrix_plus_minus(temp1, temp2, 1);
    
    temp1 = AVA(Kalman_for_MPL3115A2_A, Kalman_P);
    Kalman_P = Matrix_plus_minus(temp1, Kalman_init_P, 1);
    
    height_after_kalman = Kalman_X.matrix[0][0];
}