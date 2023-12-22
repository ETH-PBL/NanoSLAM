/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "graph-based-slam.h"
#include "pmsis.h"
#include "rcm-sparse-matrix.h"

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

#include "les-test-data/H1.h"
#include "les-test-data/H2.h"
#include "les-test-data/H3.h"

#include "les-test-data/b1.h"
#include "les-test-data/b2.h"
#include "les-test-data/b3.h"

#include "les-test-data/x1_gt.h"
#include "les-test-data/x2_gt.h"
#include "les-test-data/x3_gt.h"

#define SIZE 150
PI_L2 csr sp_mat;
PI_L2 int16_t perm[SIZE];

void evaluate_les(float H[SIZE][SIZE], float *b, float *x_gt) {
    // Create the sparse matrix from the dense input matrix
    memset(&sp_mat, 0, sizeof(csr));
    for (int16_t i = 0; i < SIZE; i++)
        for (int16_t j = 0; j < SIZE; j++)
            insert_element(&sp_mat, SIZE, i, j, H[i][j]);

    // Compute the RCM permutation
    rcm(&sp_mat, perm, SIZE);

    // Solve the LES
    solve_for_x(&sp_mat, b, perm, SIZE);

    float max_error = -1.0f;
    for (int16_t i = 0; i < SIZE; i++) {
        float error = b[i] - x_gt[i];
        if (error < 0.0f) error *= -1.0f;
        if (max_error < error) max_error = error;
    }
    if (max_error < 0.002) {
        printf("  Passed! Max error is %.5f\n", max_error);
    } else {
        printf("  Failed! Max error is %.5f\n", max_error);
    }
}

void unit_test_lin_system() {
    printf("Experiment 1\n");
    evaluate_les(H1, b1, x1_gt);
    printf("Experiment 2\n");
    evaluate_les(H2, b2, x2_gt);
    printf("Experiment 3\n");
    evaluate_les(H3, b3, x3_gt);
}

/* Program Entry. */
int main_func(void) {
    pi_freq_set(PI_FREQ_DOMAIN_FC, FREQU_FC);
    pi_freq_set(PI_FREQ_DOMAIN_CL, FREQU_CL);
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);

    printf("FC frequency : %ld\nCL frequency : %ld\n", cur_fc_freq,
           cur_cl_freq);

    uint32_t errors = 0;

    /* Init cluster configuration structure. */
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0; /* Set cluster ID. */

    /* Configure & open cluster. */
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev)) {
        printf("Cluster open failed !\n");
        pmsis_exit(-1);
    }

    unit_test_lin_system();
    return errors;
}

/* Program Entry. */
int main(void) { return pmsis_kickoff((void *)main_func); }
