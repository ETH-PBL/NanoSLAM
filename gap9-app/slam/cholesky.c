/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "../dma/dma_transfers.h"
#include "config_size.h"
#include "pmsis.h"
#include "sparse-matrix.h"
#include <math.h>
#include <string.h>

extern struct pi_device cluster_dev;
static struct pi_cluster_task cl_task;
static pi_task_t task_block_illegal;

extern csr L_L1;
extern csr H_L1;
extern csr sp_mat1;

PI_CL_L1 int16_t perm_l1[MAX_SIZE];
PI_CL_L1 float L_col_elem[NUM_CORES][MAX_SIZE / NUM_CORES + 2];
PI_CL_L1 int16_t L_col_ind[NUM_CORES][MAX_SIZE / NUM_CORES + 2];
PI_CL_L1 float sum_global;

typedef struct {
    int16_t N;
    int16_t j;
    float value;
} cluster_job;

typedef struct {
    csr *mat;
    int16_t *perm;
    int16_t size;
} fabric_job;

static float sparse_dot_product(csr *mat, int16_t row_a, int16_t row_b,
                                int16_t max_col) {
    float sum = 0;
    int16_t i = row_a;
    int16_t j = row_b;

    int16_t start = mat->row_ptr[i];
    for (int32_t k = mat->row_ptr[j]; k < mat->row_ptr[j + 1]; k++) {
        int16_t col_idx_j = mat->col_idx[k];
        if (col_idx_j > max_col) break;
        for (int32_t o = start; o < mat->row_ptr[i + 1]; o++) {
            int16_t col_idx_i = mat->col_idx[o];
            if (col_idx_j == col_idx_i) {
                sum += mat->data[k] * mat->data[o];
                start = o;
                break;
            }
            if (col_idx_i > col_idx_j) {
                start = o;
                break;
            }
        }
    }
    return sum;
}

void cholesky_sparse_fc_only(csr *H, int16_t N, int16_t *perm) {
    int16_t i, j;
    int32_t k;
    csr *L = &sp_mat1;
    init_sp_mat(L);

    for (j = 0; j < N; j++) {
        float sum = 0;
        for (k = 0; k < j; k++) sum += powf(get_element(L, N, j, k), 2);

        float element = sqrtf(get_element_sym(H, N, perm[j], perm[j]) - sum);
        insert_element(L, N, j, j, element);

        for (i = j + 1; i < N; i++) {
            sum = sparse_dot_product(L, j, i, j - 1);

            float Hij = get_element_sym(H, N, perm[i], perm[j]) - sum;
            if (Hij == 0.0f)
                element = 0.0f;
            else
                element = (1.0f / get_element(L, N, j, j)) * Hij;

            insert_element(L, N, i, j, element);
        }
    }
    // printf("Size of L is %d\n", L->nz_cnt);
    memcpy(H, L, sizeof(csr));
}

void cluster_cholesky_inner_loop(void *args) {
    int16_t core_id = (int16_t)(pi_core_id());

    cluster_job *job0 = (cluster_job *)args;
    int16_t j = job0->j;
    int16_t N = job0->N;
    float sum = 0;
    float Ljj_inv = job0->value;

    int16_t total_it = N - j - 1;
    int16_t start = core_id * total_it / NUM_CORES;
    int16_t stop = (core_id + 1) * total_it / NUM_CORES;

    int16_t cnt = 0;

    for (int16_t i = j + 1 + start; i < j + 1 + stop; i++) {
        sum = sparse_dot_product(&L_L1, j, i, j - 1);

        float Hij = get_element_sym(&H_L1, N, perm_l1[i], perm_l1[j]) - sum;
        if (Hij != 0.0f) {
            float element = Ljj_inv * Hij;
            L_col_elem[core_id][cnt] = element;
            L_col_ind[core_id][cnt] = i;
            cnt++;
        }
    }
    pi_cl_team_barrier();

    pi_cl_team_critical_enter();
    for (int16_t k = 0; k < cnt; k++)
        insert_element(&L_L1, N, L_col_ind[core_id][k], j,
                       L_col_elem[core_id][k]);
    pi_cl_team_critical_exit();

    pi_cl_team_barrier();
}

void cluster_cholesky_sum(void *args) {
    int32_t core_id = (int32_t)(pi_core_id());
    cluster_job *job0 = (cluster_job *)args;
    int16_t j = job0->j;
    int32_t N = job0->N;

    int32_t start = core_id * j / NUM_CORES;
    int32_t stop = (core_id + 1) * j / NUM_CORES;
    float sum_local = 0.0f;
    for (int32_t k = start; k < stop; k++)
        sum_local += powf(get_element(&L_L1, N, j, k), 2);

    pi_cl_team_critical_enter();
    sum_global += sum_local;
    pi_cl_team_critical_exit();
    pi_cl_team_barrier();
}

void master_core_delegate_cholesky(void *args) {
    fabric_job *job_fc = (fabric_job *)args;
    int32_t N = job_fc->size;

    init_sp_mat(&L_L1);
    cl_memcpy_from_l2(&H_L1, job_fc->mat, sizeof(csr));
    cl_memcpy_from_l2(&perm_l1, job_fc->perm, N * sizeof(int16_t));

    cluster_job job_cl = {
        .N = N,
        .j = 0,
    };

    for (int16_t j = 0; j < N; j++) {
        sum_global = 0.0f;
        job_cl.j = j;
        pi_cl_team_fork(NUM_CORES, cluster_cholesky_sum, &job_cl);
        float sum = sum_global;

        float element =
            sqrtf(get_element_sym(&H_L1, N, perm_l1[j], perm_l1[j]) - sum);
        insert_element(&L_L1, N, j, j, element);

        float Ljj_inv = (1.0f / get_element(&L_L1, N, j, j));
        job_cl.j = j;
        job_cl.value = Ljj_inv;

        pi_cl_team_fork(NUM_CORES, cluster_cholesky_inner_loop, &job_cl);
    }
    cl_memcpy_to_l2(job_fc->mat, &L_L1, sizeof(csr));
}

void cholesky_sparse_parallel(csr *H, int16_t N, int16_t *perm) {
    fabric_job job_fc = {
        .mat = H,
        .perm = perm,
        .size = N,
    };
    pi_task_block(&task_block_illegal);
    pi_cluster_send_task_to_cl_async(
        &cluster_dev,
        pi_cluster_task(&cl_task, master_core_delegate_cholesky, &job_fc),
        &task_block_illegal);
    pi_task_wait_on(&task_block_illegal);
}