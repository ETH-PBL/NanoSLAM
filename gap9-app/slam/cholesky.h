/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef CHOLESKY_H
#define CHOLESKY_H

#include "sparse-matrix.h"

/**
 * Computes the Cholesky decomposition using the Fabric Controller only. The
 *operation is performed in place.
 *
 * @param H Pointer to the input csr matrix
 * @param N Size of the equivalent dense matrix
 * @param perm Permutation array
 **/
void cholesky_sparse_fc_only(csr *H, int16_t N, int16_t *perm);

/**
 * Computes the Cholesky decomposition using the Cluster. The operation is
 *performed in place.
 *
 * @param H Pointer to the input csr matrix
 * @param N Size of the equivalent dense matrix
 * @param perm Permutation array
 **/
void cholesky_sparse_parallel(csr *H, int16_t N, int16_t *perm);

#endif