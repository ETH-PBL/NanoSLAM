/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pmsis.h"
#include "sparse-matrix.h"
#include <math.h>
#include "assert.h"

extern csr sp_mat1;

void move_array_safe(char *dest, const char *src, size_t n) {
    while (n > 0) {
        n--;
        dest[n] = src[n];
    }
}

void insert_element(csr *mat, int16_t N, int16_t i, int16_t j, float value) {
    if (fabs(value) < NZ_THRESHOLD) return;
    ASSERT(mat->nz_cnt + 1 <= MAX_SIZE_SP_MAT, "Sparse matrix is full");

    // If we have a dense matrix with elemetns of coordinates (i, j), we call as
    // order the value i*N + j.

    // First element added in the matrix
    if (mat->max_line == 0) {
        mat->max_order = -1;
        mat->nz_cnt = -1;
    }

    // If the element to be added has a smaller order than the maximum order in
    // the matrix
    if (i * N + j <= mat->max_order) {
        int32_t row_start = mat->row_ptr[i];   // 1
        int32_t row_end = mat->row_ptr[i + 1]; // 2
        int32_t k_last = row_start;
        for (int32_t k = row_start; k < row_end; k++) {
            k_last = row_end;
            if (mat->col_idx[k] == j) { // Element is already at that position,
                                        // thus only update the existing one
                mat->data[k] = value;
                mat->col_idx[k] = j;
                return;
            }
            if (mat->col_idx[k] > j) {
                k_last = k;
                break;
            }
        }

        // Shift the data array and the column index array one position to the
        // right and make space for the new element
        move_array_safe((char *)&(mat->data[k_last + 1]),
                        (char *)&(mat->data[k_last]),
                        (mat->nz_cnt - k_last + 1) * sizeof(float));
        move_array_safe((char *)&(mat->col_idx[k_last + 1]),
                        (char *)&(mat->col_idx[k_last]),
                        (mat->nz_cnt - k_last + 1) * sizeof(int16_t));

        mat->data[k_last] = value;
        mat->col_idx[k_last] = j;
        for (int16_t l = i + 1; l <= mat->max_line; l++) {
            mat->row_ptr[l]++;
        }
        mat->nz_cnt++;
        return;
    }

    // The element to be added has a higher order than any previously added
    // element so it is added "to the end of the csr arrays"
    mat->nz_cnt++;
    mat->data[mat->nz_cnt] = value;
    mat->col_idx[mat->nz_cnt] = j;
    for (int16_t k = mat->max_line; k <= i; k++) {
        mat->row_ptr[k + 1] = mat->row_ptr[k];
    }

    mat->row_ptr[i + 1]++;
    mat->max_line = i + 1;

    mat->max_order = i * N + j;
    return;
}

void add_to_element(csr *mat, int16_t N, int16_t i, int16_t j, float value) {
    int32_t row_start = mat->row_ptr[i];
    int32_t row_end = mat->row_ptr[i + 1];

    for (int32_t k = row_start; k < row_end; k++) {
        if (mat->col_idx[k] == j) {
            mat->data[k] += value;
            return;
        }
    }
    insert_element(mat, N, i, j, value);
}

int16_t already_added(csr *mat, int16_t N, int16_t i, int16_t j) {
    int32_t row_start = mat->row_ptr[i];
    int32_t row_end = mat->row_ptr[i + 1];

    for (int32_t k = row_start; k < row_end; k++) {
        if (mat->col_idx[k] > j) {
            return 0;
        }
        if (mat->col_idx[k] == j) {
            return 1;
        }
    }
    return 0;
}

float get_element(csr *mat, int16_t N, int16_t i, int16_t j) {
    int32_t row_start = mat->row_ptr[i];
    int32_t row_end = mat->row_ptr[i + 1];

    for (int32_t k = row_start; k < row_end; k++) {
        if (mat->col_idx[k] == j) {
            return mat->data[k];
        }
        if (mat->col_idx[k] > j) {
            break;
        }
    }

    return 0.0f;
}

float get_element_sym(csr *mat, int16_t N, int16_t i, int16_t j) {
    if (i < j) {
        int16_t aux = i;
        i = j;
        j = aux;
    }

    int32_t row_start = mat->row_ptr[i];
    int32_t row_end = mat->row_ptr[i + 1];

    for (int32_t k = row_start; k < row_end; k++) {
        if (mat->col_idx[k] == j) {
            return mat->data[k];
        }
        if (mat->col_idx[k] > j) {
            break;
        }
    }

    return 0.0f;
}

void transpose_lower_triangular_in_place(csr *mat, int16_t N) {
    csr *mat_tr = &sp_mat1;
    init_sp_mat(mat_tr);

    for (int32_t i = 0; i < N; i++)
        for (int32_t k = mat->row_ptr[i]; k < mat->row_ptr[i + 1]; k++)
            insert_element(mat_tr, N, mat->col_idx[k], i, mat->data[k]);

    memcpy(mat, mat_tr, sizeof(csr));
}

void transpose_lower_triangular(csr *mat, csr *mat_tr, int16_t N) {
    for (int16_t i = 0; i < N; i++)
        for (int16_t k = mat->row_ptr[i]; k < mat->row_ptr[i + 1]; k++) {
            insert_element(mat_tr, N, mat->col_idx[k], i, mat->data[k]);
        }
}

void init_sp_mat(csr *mat) {
    mat->nz_cnt = 0;
    mat->max_line = 0;
    mat->max_order = 0;
    memset(mat->row_ptr, 0, (MAX_SIZE + 1) * sizeof(int16_t));
}
