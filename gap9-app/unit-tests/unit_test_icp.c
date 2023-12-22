/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "icp.h"
#include "pmsis.h"

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

#define SIZE_ICP 640
static PI_L2 icp_points_t p, q;

void unit_test_icp() {
    p.num = SIZE_ICP;
    q.num = SIZE_ICP;

    for (int16_t i = 0; i < SIZE_ICP; i++) {
        p.items[i].x = i / 100.0f + 0.2;
        q.items[i].x = (i + 0) / 100.0f;

        p.items[i].y = i / 100.0f + 0;
        q.items[i].y = (i + 0) / 100.0f;
    }

    // Execute ICP
    float icp_res[3];
    icp_job_t job = {0};

    icp(&p, &q, 25, 0.3f, icp_res, &job);

    if (!job.has_nan) {
        printf("ICP Succeded! \n");
        printf("Execution time [ms]: %d \n", job.time_ms);
        printf("ICP result: Tr: %.3f %.3f  Rot:%.3f \n", icp_res[0], icp_res[1],
               icp_res[2]);
    } else {
        printf("ICP Failed! \n");
    }
    printf("Expected ICP result: Tr: %.3f %.3f  Rot:%.3f \n", 0.101, -0.099,
           -0.000);
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

    unit_test_icp();

    return errors;
}

/* Program Entry. */
int main(void) { return pmsis_kickoff((void *)main_func); }
