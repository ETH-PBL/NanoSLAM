/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SPARSE_MAT_H
#define SPARSE_MAT_H

#include "config_size.h"

#define NZ_THRESHOLD 0.00001f // any element below this value is set to 0

typedef struct {
    float data[MAX_SIZE_SP_MAT];      // contains the values of the matrix
    int16_t col_idx[MAX_SIZE_SP_MAT]; // the corresponding column index for each
                                      // value
    int16_t row_ptr[MAX_SIZE + 1];    // row_ptr[i] indicates where the i-th row
                                      // starts in data and col_idx
    int16_t max_line;  // the index of the most bottom written line plus 1
    int32_t nz_cnt;    // number of non-zero entries minus 1
    int32_t max_order; // the maximum i*N + j among all elements
} csr;

/**
 * @brief Insert elements into the sparse csr matrix.
 *
 * @param mat Pointer to the csr matrix.
 * @param N Dimension of the equivalent dense matrix.
 * @param i Line index of the element.
 * @param j Column index of the element.
 * @param value The value of the element.
 */
void insert_element(csr *mat, int16_t N, int16_t i, int16_t j, float value);

/**
 * @brief Add a value to an existing element in the sparse matrix.
 *
 * @param mat Pointer to the csr matrix.
 * @param N Dimension of the equivalent dense matrix.
 * @param i Line index of the element.
 * @param j Column index of the element.
 * @param value The value to be added to the element.
 */
void add_to_element(csr *mat, int16_t N, int16_t i, int16_t j, float value);

/**
 * @brief Check if an element has already been added to the position (i,j).
 *
 * @param mat Pointer to the csr matrix.
 * @param N Dimension of the equivalent dense matrix.
 * @param i Line index of the element.
 * @param j Column index of the element.
 * @return 1 if an element has been found, 0 otherwise.
 */
int16_t already_added(csr *mat, int16_t N, int16_t i, int16_t j);

/**
 * @brief Fetch the value of the element from the position (i,j).
 *
 * @param mat Pointer to the csr matrix.
 * @param N Dimension of the equivalent dense matrix.
 * @param i Line index of the element.
 * @param j Column index of the element.
 * @return The value of the element.
 */
float get_element(csr *mat, int16_t N, int16_t i, int16_t j);

/**
 * @brief Fetch the value of the element at the position (i,j) from csr matrix
 * that corresponds to a symmetric dense matrix. Therefore, if i<j, the function
 * will swap them.
 *
 * @param mat Pointer to the csr matrix.
 * @param N Dimension of the equivalent dense matrix.
 * @param i Line index of the element.
 * @param j Column index of the element.
 * @return The value of the element.
 */
float get_element_sym(csr *mat, int16_t N, int16_t i, int16_t j);

/**
 * @brief Transpose csr matrix in place.
 *
 * @param mat Pointer to the csr matrix.
 * @param N Dimension of the equivalent dense matrix.
 */
void transpose_lower_triangular_in_place(csr *mat, int16_t N);

/**
 * @brief Transpose csr matrix.
 *
 * @param mat Pointer to the input csr matrix.
 * @param mat_tr Pointer to the output csr matrix where the transposed will be
 placed.

 * @param N Dimension of the equivalent dense matrix.
 */
void transpose_lower_triangular(csr *mat, csr *mat_tr, int16_t N);

/**
 * @brief Initialize a csr sparse matrix, all values will be cleaned.
 *
 * @param mat Pointer to the csr matrix.
 */
void init_sp_mat(csr *mat);

#endif