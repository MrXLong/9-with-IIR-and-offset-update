/*
 * File:   matrix_caculator.c
 * Author: LJ
 *
 * Created on February 17, 2017, 9:38 AM
 */

#include "matrix_calculator.h"
#include <stdint.h> 

matrix_with_dimension Matrix_multiplication( matrix_with_dimension matrix1, matrix_with_dimension matrix2)
{
    int i, j, k;
    float sum = 0;
    matrix_with_dimension multiply_result;
    multiply_result.row = matrix1.row;
    multiply_result.column = matrix2.column;
    for(i = 0; i < multiply_result.row; i++){
        for(j = 0; j < multiply_result.column; j++){
            for(k = 0; k < multiply_result.row; k++){
                sum = matrix1.matrix[i][k] * matrix2.matrix[k][j] + sum;
            }
        multiply_result.matrix[i][j] = sum;
        sum = 0;     
        }
        
    }
    
    return multiply_result; 
}

matrix_with_dimension Matrix_scalar(matrix_with_dimension matrix1, float scalar1)
{
    int i, j;
    matrix_with_dimension Matrix_scalar_result;
    Matrix_scalar_result.row = matrix1.row;
    Matrix_scalar_result.column = matrix1.column;
    
    for(i = 0; i < matrix1.row; i++){
        for(j = 0; j < matrix1.column; j++){
            Matrix_scalar_result.matrix[i][j] = matrix1.matrix[i][j] * scalar1;
        }
    }
    return Matrix_scalar_result;
}// gai

matrix_with_dimension Matrix_plus_minus(matrix_with_dimension matrix1, matrix_with_dimension matrix2, int operation_here)
{
    int i, j;
    matrix_with_dimension Matrix_plus_result;
    Matrix_plus_result.column = matrix1.column;
    Matrix_plus_result.row = matrix1.row;
    
    for(i = 0; i < matrix1.row; i++){
        for(j = 0; j < matrix1.column; j++){
            if(operation_here == 1){
                Matrix_plus_result.matrix[i][j] = matrix1.matrix[i][j] + matrix2.matrix[i][j];}
            else
                Matrix_plus_result.matrix[i][j] = matrix1.matrix[i][j] - matrix2.matrix[i][j];
        }
    }
    return Matrix_plus_result;
}

matrix_with_dimension Matrix_transpose(matrix_with_dimension matrix1)
{
    int i, j;
    matrix_with_dimension Transpose_matrix;
    Transpose_matrix.column = matrix1.row;
    Transpose_matrix.row = matrix1.column;
    
    for(i = 0; i < matrix1.row; i++){
        for(j = 0; j < matrix1.column; j++){
            Transpose_matrix.matrix[j][i] = matrix1.matrix[i][j];
        }
    }
    return Transpose_matrix;
}

matrix_with_dimension AVA(matrix_with_dimension matrix1, matrix_with_dimension matrix2)
{
    matrix_with_dimension temp1, temp2;
    temp1 = Matrix_transpose(matrix1);
    temp2 = Matrix_multiplication(matrix1,matrix2);
    temp1 = Matrix_multiplication(temp2, temp1);
    return temp1;
}

matrix_with_dimension Eye_creater(int i)
{
    matrix_with_dimension temp1;
    temp1.column = i;
    temp1.row = i;
    
    int k,j;
    for(k = 0; k < i; k++){
        for(j = 0; j < i; j++){
            temp1.matrix[k][j] = 0;
        }
    }
    for(k = 0; k < i; k++){
        temp1.matrix[k][k] = 1;
    }
    
    return temp1;
}

// this block would initial some matrix which would be used in the main function to check if the functions in this file go well
