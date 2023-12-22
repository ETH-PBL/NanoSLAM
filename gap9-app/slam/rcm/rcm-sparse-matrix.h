/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef RCM_SPARSE_H
#define RCM_SPARSE_H

#include "sparse-matrix.h"

/**
 * Computes a permutation that reduces the bandwidth of a matrix.
 *
 * @param mat Pointer to the sparse matrix
 * @param perm Output array where the permutation is put
 * @param N Size of the equivalent dense matrix
 **/
void rcm(csr *mat, int16_t *perm, int16_t N);

/**
 * Provides the inverse of a permutation.
 *
 * @param perm Input permutation
 * @param perm_inv Output inverse permutation
 **/
void permutation_inverse(int16_t *perm, int16_t *perm_inv, int N);

/**
 * Applies a permutation to an array.
 *
 * @param array Input array
 * @param perm Permutation array
 * @param N Array size
 **/
void permute_array(float *array, int16_t *perm, int16_t N);

/**
 * Computes a permutation that reduces the bandwidth of a matrix.
 * It exploits the sparsity of the input matrix and it runs faster than rcm()
 *
 * @param mat Pointer to the sparse matrix
 * @param perm Output array where the permutation is put
 * @param N Size of the equivalent dense matrix
 **/
void rcm_fast(csr *mat, int16_t *perm, int16_t N);

#endif