/* 
 * File:   matrix_calculator.h
 * Author: LJ
 * Comments: 
 * Revision history: 
 */
#ifndef _MATRIX_CALCULATORH_
#define _MATRIX_CALCULATORH_

typedef struct matrix_with_dimension
{
    int row;
    int column;
    float matrix[3][3]; //= {{0,0,0}, {0,0,0}, {0,0,0}}; 
}matrix_with_dimension;

matrix_with_dimension Matrix_multiplication( matrix_with_dimension matrix1, matrix_with_dimension matrix2);
matrix_with_dimension Matrix_scalar(matrix_with_dimension matrix1, float scalar1);
matrix_with_dimension Matrix_plus_minus(matrix_with_dimension matrix1, matrix_with_dimension matrix2, int operation_here);
matrix_with_dimension Matrix_transpose(matrix_with_dimension matrix1);
matrix_with_dimension AVA(matrix_with_dimension matrix1, matrix_with_dimension matrix2);
matrix_with_dimension Eye_creater(int i);
 

#endif