#ifndef UTILS_MATH_H
#define UTILS_MATH_H

/**
 * Inverts a 3x3 matrix.
 *
 * @param A Input 3x3 matrix
 * @param B Output inverse - 3x3 matrix
 **/
void matinv_3x3(float A[3][3], float A_inv[3][3]);

/**
 * Multiplies two 3x3 matrices. R = A * B.
 *
 * @param A Input 3x3 matrix
 * @param B Input 3x3 matrix
 * @param R Resulting matrix
 **/
void matmul_3x3(float A[3][3], float B[3][3], float R[3][3]);

/**
 * Multiplies two 3x3 matrices, but transposes the first one. R = A.T * B.
 *
 * @param A Input 3x3 matrix
 * @param B Input 3x3 matrix
 * @param R Resulting matrix
 **/
void transpose_and_mult_3x3(float A[3][3], float B[3][3], float R[3][3]);

/**
 * Multiplies a 3x3 matrix by a 3x1 array, but transposes the matrix. R = A.T *
 *B.
 *
 * @param A Input 3x3 matrix
 * @param B Input 3x1 array
 * @param R Resulting array
 **/
void transpose_and_mult_3x3x1(float A[3][3], float B[3], float R[3]);

/**
 * Multiplies two 2x2 matrices. R = A * B.
 *
 * @param A Input 2x2 matrix
 * @param B Input 2x2 matrix
 * @param R Resulting matrix
 **/
void matmul_2x2(float A[2][2], float B[2][2], float R[2][2]);

/**
 * Multiplies a 2x2 matrix by a 2x1 array. R = A * B.
 *
 * @param A Input 2x2 matrix
 * @param B Input 2x1 array
 * @param R Resulting array
 **/
void matmul_2x2x1(float A[2][2], float B[2], float R[2]);

#endif