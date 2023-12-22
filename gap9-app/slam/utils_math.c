#include "utils_math.h"
#include <stdio.h>
#include <string.h>

void matinv_3x3(float A[3][3], float A_inv[3][3]) {
    float det = 0.0f;
    int i, j;
    for (i = 0; i < 3; i++)
        det = det + (A[0][i] * (A[1][(i + 1) % 3] * A[2][(i + 2) % 3] -
                                A[1][(i + 2) % 3] * A[2][(i + 1) % 3]));

    float A_copy[3][3];
    memcpy(A_copy, A, 9 * sizeof(float));

    for (i = 0; i < 3; i++)
        for (j = 0; j < 3; j++)
            A_inv[j][i] = ((A_copy[(i + 1) % 3][(j + 1) % 3] *
                            A_copy[(i + 2) % 3][(j + 2) % 3]) -
                           (A_copy[(i + 1) % 3][(j + 2) % 3] *
                            A_copy[(i + 2) % 3][(j + 1) % 3])) /
                          det;
}

void matmul_3x3(float A[3][3], float B[3][3], float R[3][3]) {
    float A_copy[3][3];
    float B_copy[3][3];
    memcpy(A_copy, A, 9 * sizeof(float));
    memcpy(B_copy, B, 9 * sizeof(float));

    for (int16_t i = 0; i < 3; i++) {
        for (int16_t j = 0; j < 3; j++) {
            R[i][j] = 0;
            for (int16_t k = 0; k < 3; k++)
                R[i][j] += A_copy[i][k] * B_copy[k][j];
        }
    }
}

void transpose_and_mult_3x3(float A[3][3], float B[3][3], float R[3][3]) {
    float A_copy[3][3];
    float B_copy[3][3];
    memcpy(A_copy, A, 9 * sizeof(float));
    memcpy(B_copy, B, 9 * sizeof(float));

    for (int16_t i = 0; i < 3; i++) {
        for (int16_t j = 0; j < 3; j++) {
            R[i][j] = 0;
            for (int16_t k = 0; k < 3; k++)
                R[i][j] += A_copy[k][i] * B_copy[k][j];
        }
    }
}

void transpose_and_mult_3x3x1(float A[3][3], float B[3], float R[3]) {
    float A_copy[3][3];
    float B_copy[3];
    memcpy(A_copy, A, 9 * sizeof(float));
    memcpy(B_copy, B, 3 * sizeof(float));

    for (int16_t i = 0; i < 3; i++) {
        R[i] = 0;
        for (int16_t k = 0; k < 3; k++) R[i] += A_copy[k][i] * B_copy[k];
    }
}

void matmul_2x2(float A[2][2], float B[2][2], float R[2][2]) {
    float A_copy[2][2];
    float B_copy[2][2];
    memcpy(A_copy, A, 4 * sizeof(float));
    memcpy(B_copy, B, 4 * sizeof(float));

    for (int16_t i = 0; i < 2; i++) {
        for (int16_t j = 0; j < 2; j++) {
            R[i][j] = 0;
            for (int16_t k = 0; k < 2; k++)
                R[i][j] += A_copy[i][k] * B_copy[k][j];
        }
    }
}

void matmul_2x2x1(float A[2][2], float B[2], float R[2]) {
    float A_copy[2][2];
    float B_copy[2];
    memcpy(A_copy, A, 4 * sizeof(float));
    memcpy(B_copy, B, 2 * sizeof(float));

    for (int16_t i = 0; i < 2; i++) {
        R[i] = 0;
        for (int16_t k = 0; k < 2; k++) R[i] += A_copy[i][k] * B_copy[k];
    }
}