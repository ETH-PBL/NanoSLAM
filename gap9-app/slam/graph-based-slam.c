/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cholesky.h"
#include "config_size.h"
#include "graph-based-slam.h"
#include "rcm-sparse-matrix.h"
#include "sparse-matrix.h"
#include "utils_math.h"
#include "pmsis.h"
#include "assert.h"

extern csr sp_mat0;
extern csr sp_mat1;

PI_L2 float array0[MAX_SIZE];
PI_L2 float array1[MAX_SIZE];
PI_L2 int16_t array2[MAX_SIZE];
PI_L2 int16_t array3[MAX_SIZE];

PI_L2 measurement measurements[MAX_POSES - 1];

void v2t(float pose[3], float T[3][3]) {
    float c = cosf(pose[2]);
    float s = sinf(pose[2]);
    T[0][0] = c;
    T[0][1] = -s;
    T[0][2] = pose[0];
    T[1][0] = s;
    T[1][1] = c;
    T[1][2] = pose[1];
    T[2][0] = 0;
    T[2][1] = 0;
    T[2][2] = 1;
}

void t2v(float T[3][3], float v[3]) {
    float x = T[0][2];
    float y = T[1][2];
    float theta = atan2f(T[1][0], T[0][0]);
    v[0] = x;
    v[1] = y;
    v[2] = theta;
}

void linearize_pose_pose_constraint(float *x1, float *x2, float *z,
                                    float A[3][3], float B[3][3], float *e) {
    // Compute e
    float Zij[3][3];
    float Xi[3][3];
    float Xj[3][3];

    v2t(z, Zij);
    v2t(x1, Xi);
    v2t(x2, Xj);

    matinv_3x3(Zij, Zij);
    matinv_3x3(Xi, Xi);
    matmul_3x3(Zij, Xi, Xi);
    matmul_3x3(Xi, Xj, Xi);

    t2v(Xi, e);

    // compute A & B
    float s = sinf(x1[2]);
    float c = cosf(x1[2]);

    float s2 = sinf(z[2]);
    float c2 = cosf(z[2]);

    float RiT[2][2] = {{c, s}, {-s, c}};
    float RijT[2][2] = {{c2, s2}, {-s2, c2}};
    float dRiT_dTheta[2][2] = {{-s, c}, {-c, -s}};
    float delta_t[2] = {x2[0] - x1[0], x2[1] - x1[1]};

    float aux0[2][2];
    float aux1[2];

    matmul_2x2(RijT, dRiT_dTheta, aux0);
    matmul_2x2x1(aux0, delta_t, aux1);
    matmul_2x2(RijT, RiT, aux0);

    A[0][0] = -aux0[0][0];
    A[0][1] = -aux0[0][1];
    A[0][2] = aux1[0];
    A[1][0] = -aux0[1][0];
    A[1][1] = -aux0[1][1];
    A[1][2] = aux1[1];
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = -1;

    B[0][0] = aux0[0][0];
    B[0][1] = aux0[0][1];
    B[0][2] = 0;
    B[1][0] = aux0[1][0];
    B[1][1] = aux0[1][1];
    B[1][2] = 0;
    B[2][0] = 0;
    B[2][1] = 0;
    B[2][2] = 1;
}

void build_H_and_b(float *x, csr *H, float *b, measurement *measurements,
                   constraint *constraints, int16_t nr_of_poses,
                   int16_t nr_of_constr) {
    float z[3];
    float A[3][3], B[3][3], e[3];

    int16_t dim = 3 * nr_of_poses;

    insert_element(H, dim, 0, 0, 1000);
    insert_element(H, dim, 1, 1, 1000);
    insert_element(H, dim, 2, 0, 0);
    insert_element(H, dim, 2, 1, 0);
    insert_element(H, dim, 2, 2, 1000);

    for (int16_t i = 1; i < nr_of_poses; i++) {
        float xi[3] = {x[3 * i - 3], x[3 * i - 2], x[3 * i - 1]};
        float xj[3] = {x[3 * i], x[3 * i + 1], x[3 * i + 2]};
        memcpy(z, measurements[i - 1].z, sizeof(z));

        linearize_pose_pose_constraint(xi, xj, z, A, B, e);
        float bi[3], bj[3];
        float Hii[3][3], Hjj[3][3], Hij[3][3];
        transpose_and_mult_3x3x1(A, e, bi);
        transpose_and_mult_3x3x1(B, e, bj);
        transpose_and_mult_3x3(A, A, Hii);
        transpose_and_mult_3x3(A, B, Hij);
        transpose_and_mult_3x3(B, B, Hjj);

        add_to_element(H, dim, 3 * (i - 1) + 0, 3 * (i - 1) + 0,
                       Hii[0][0]); // Hii
        add_to_element(H, dim, 3 * (i - 1) + 1, 3 * (i - 1) + 1,
                       Hii[1][1]); // Hii
        insert_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 0,
                       Hii[2][0]); // Hii
        insert_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 1,
                       Hii[2][1]); // Hii
        add_to_element(H, dim, 3 * (i - 1) + 2, 3 * (i - 1) + 2,
                       Hii[2][2]); // Hii

        insert_element(H, dim, 3 * i + 0, 3 * (i - 1) + 0, -1); // Hij

        insert_element(H, dim, 3 * i + 0, 3 * i + 0, Hjj[0][0]); // Hjj

        insert_element(H, dim, 3 * i + 1, 3 * (i - 1) + 1, -1); // Hij

        insert_element(H, dim, 3 * i + 1, 3 * i + 1, Hjj[1][1]); // Hjj

        insert_element(H, dim, 3 * i + 0, 3 * (i - 1) + 2, Hij[2][0]); // Hij
        insert_element(H, dim, 3 * i + 1, 3 * (i - 1) + 2, Hij[2][1]); // Hij
        insert_element(H, dim, 3 * i + 2, 3 * (i - 1) + 2, -1);        // Hij

        // insert_element(H, dim, 3*i + 2, 3*i + 0, 0); // Hjj
        // insert_element(H, dim, 3*i + 2, 3*i + 1, 0); // Hjj
        insert_element(H, dim, 3 * i + 2, 3 * i + 2, Hjj[2][2]); // Hjj

        for (int16_t i0 = 0; i0 < 3; i0++) {
            b[3 * (i - 1) + i0] += -bi[i0];
            b[3 * i + i0] += -bj[i0];
        }
    }

    for (int16_t i = 0; i < nr_of_constr; i++) {
        int fromIdx = constraints[i].fromIdx;
        int toIdx = constraints[i].toIdx;
        float info = 20.0f;
        memcpy(z, constraints[i].z, sizeof(z));

        float xi[3] = {x[3 * fromIdx], x[3 * fromIdx + 1], x[3 * fromIdx + 2]};
        float xj[3] = {x[3 * toIdx], x[3 * toIdx + 1], x[3 * toIdx + 2]};

        linearize_pose_pose_constraint(xi, xj, z, A, B, e);
        float bi[3], bj[3];
        float Hii[3][3], Hjj[3][3], Hij[3][3];
        transpose_and_mult_3x3x1(A, e, bi);
        transpose_and_mult_3x3x1(B, e, bj);
        transpose_and_mult_3x3(A, A, Hii);
        transpose_and_mult_3x3(A, B, Hij);
        transpose_and_mult_3x3(B, B, Hjj);

        for (int16_t i0 = 0; i0 < 3; i0++) {
            add_to_element(H, dim, 3 * fromIdx + i0, 3 * fromIdx + i0,
                           info * Hii[i0][i0]); // Hii
            add_to_element(H, dim, 3 * toIdx + i0, 3 * toIdx + i0,
                           info * Hjj[i0][i0]); // Hjj
            insert_element(H, dim, 3 * fromIdx + i0, 3 * toIdx + i0,
                           info * (-1.0f)); // Hij
        }
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * fromIdx + 0,
                       info * Hii[2][0]); // Hii
        add_to_element(H, dim, 3 * fromIdx + 2, 3 * fromIdx + 1,
                       info * Hii[2][1]); // Hii

        insert_element(H, dim, 3 * fromIdx + 2, 3 * toIdx + 0,
                       info * Hij[2][0]); // Hij
        insert_element(H, dim, 3 * fromIdx + 2, 3 * toIdx + 1,
                       info * Hij[2][1]); // Hij

        for (int16_t i0 = 0; i0 < 3; i0++) {
            b[3 * fromIdx + i0] += -bi[i0] * info;
            b[3 * toIdx + i0] += -bj[i0] * info;
        }
    }
}

void forward_substitution(csr *L, float *b, int N) {
    float *sol = array1;
    memset(sol, 0, N * sizeof(float));
    sol[0] = b[0] / get_element(L, N, 0, 0);
    for (int i = 1; i < N; i++) {
        float sum = 0;
        for (int32_t k = L->row_ptr[i]; k < L->row_ptr[i + 1]; k++)
            if (i > L->col_idx[k]) sum += L->data[k] * sol[L->col_idx[k]];
        sol[i] = (1.0f / get_element(L, N, i, i)) * (b[i] - sum);
    }

    memcpy(b, sol, N * sizeof(float));
}

void backward_substitution(csr *L, float *b, int N) {
    float *sol = array1;
    memset(sol, 0, N * sizeof(float));
    sol[N - 1] = b[N - 1] / get_element(L, N, N - 1, N - 1);
    for (int i = N - 2; i >= 0; i--) {
        float sum = 0;
        for (int32_t k = L->row_ptr[i]; k < L->row_ptr[i + 1]; k++)
            if (i < L->col_idx[k]) sum += L->data[k] * sol[L->col_idx[k]];
        sol[i] = (1.0f / get_element(L, N, i, i)) * (b[i] - sum);
    }

    memcpy(b, sol, N * sizeof(float));
}

void solve_for_x(csr *H, float *b, int16_t *perm, int16_t size) {
    if (H->nz_cnt + 1 > MAX_SIZE_SP_MAT) {
        printf("Input SpMat element overflow\n");
        while (1) pi_time_wait_us(1000);
    }

    // Cholesky Decomposition
    int32_t t0 = pi_time_get_us();
    cholesky_sparse_parallel(H, size, perm);
    // cholesky_sparse_fc_only(H, size, perm);

    if (H->nz_cnt + 1 > MAX_SIZE_SP_MAT) {
        printf("SpMat element overflow\n");
        while (1) pi_time_wait_us(1000);
    }

    // Permute array
    permute_array(b, perm, size);

    /* L * L.T * x = b;   we note L.T * x = y
       Solves L * y = b; */
    forward_substitution(H, b, size);
    float *y = b;

    // Solves L.T * x = y
    transpose_lower_triangular_in_place(H, size);
    backward_substitution(H, y, size);
    float *x = y;

    // Inverse permute the solution
    int16_t *permInv = array3;
    permutation_inverse(perm, permInv, size);
    permute_array(x, permInv, size);
}

void measurement_from_poses(float *x_from, float *x_to, float *z) {
    for (int16_t k = 0; k < 3; k++) z[k] = x_to[k] - x_from[k];

    float s = sinf(-x_from[2]);
    float c = cosf(-x_from[2]);
    float R[2][2] = {{c, -s}, {s, c}};
    matmul_2x2x1(R, z, z);
}

void measurements_from_poses(float *x, measurement *measurements,
                             int nr_of_poses) {
    float z[3];
    for (int16_t i = 1; i < nr_of_poses; i++) {
        float xi[3] = {x[3 * i - 3], x[3 * i - 2], x[3 * i - 1]};
        float xj[3] = {x[3 * i], x[3 * i + 1], x[3 * i + 2]};
        measurement_from_poses(xi, xj, z);

        measurements[i - 1].z[0] = z[0];
        measurements[i - 1].z[1] = z[1];
        measurements[i - 1].z[2] = z[2];
    }
}

int16_t abs_int(int16_t x) {
    if (x < 0)
        return x * (-1);
    else
        return x;
}

int16_t bandwidth_test(csr *mat, int N, int16_t *perm) {
    int16_t perm_inv[N];
    for (int16_t i = 0; i < N; i++)
        for (int16_t j = 0; j < N; j++)
            if (perm[j] == i) {
                perm_inv[i] = j;
                break;
            }

    // Bandwidth test
    int16_t bw_len = 0;
    for (int16_t i = 0; i < N; i++) {
        for (int32_t k = mat->row_ptr[i]; k < mat->row_ptr[i + 1]; k++) {
            int j = mat->col_idx[k];
            int indx = perm_inv[i];
            int indy = perm_inv[j];
            if (abs_int(indx - indy) > bw_len) {
                bw_len = abs_int(indx - indy);
            }
        }
    }
    return bw_len;
}

void ls_slam(float *x, constraint *constraints, int16_t nr_of_poses,
             int16_t nr_of_constr) {
    csr *H = &sp_mat0;
    memset(array0, 0, MAX_SIZE * sizeof(float));
    memset(array1, 0, MAX_SIZE * sizeof(float));

    float *b = array0;
    int16_t *perm = array2;

    // Compute the graph edge measurements
    measurements_from_poses(x, measurements, nr_of_poses);
    for (int16_t k = 0; k < 3; k++) {
        // Empty the sparse matrix
        init_sp_mat(H);
        memset(b, 0, 3 * nr_of_poses * sizeof(float));

        // Compute the input of the linear equation system
        build_H_and_b(x, H, b, measurements, constraints, nr_of_poses,
                      nr_of_constr);

        // Get the permutation that minimizes the bandwidth of H
        rcm_fast(H, perm, 3 * nr_of_poses);

        // Solve the linear equation system
        solve_for_x(H, b, perm, 3 * nr_of_poses);

        // Update the poses
        for (int16_t i = 0; i < 3 * nr_of_poses; i++) x[i] += b[i];
    }
}