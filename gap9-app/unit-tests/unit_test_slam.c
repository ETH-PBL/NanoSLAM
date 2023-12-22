/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "../icp/icp.h"
#include "pmsis.h"
#include "point_cloud.h"
#include "rcm-sparse-matrix.h"
#include "slam-test-data/pos_gt.h"
#include "slam-test-data/pos_test.h"
#include "slam-test-data/scans_test.h"
#include "slam-test-data/tof_test.h"
#include "spi_cmds_decode.h"
#include "spi_driver.h"

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

PI_L2 icp_points_t ref_scan, cur_scan;

#define SIZE_POSES 2000
#define SIZE_SCANS 4

void unit_test_slam() {
    pcl_pose_t pose0 = {0};
    float icp_res[3];

    printf("Hierarchical SLAM optimization\n");

    // Add poses to the graph
    for (int16_t i = 0; i < SIZE_POSES; i++) {
        pose0.id = i;
        memcpy(pose0.pos, pos_test[i], 3 * sizeof(float));
        memcpy(pose0.tof, tof_test[i], 32 * sizeof(int16_t));
        pcl_add_augmented_pose(pose0);
    }

    // Compute and add loop closure constraints
    for (int16_t i = 0; i < SIZE_SCANS; i++) {
        int16_t ref_pose = scans_test[i][1];
        int16_t comp_pose = scans_test[i][0];
        if (ref_pose >= 0) {
            icp_job_t icp_job = {0};
            // Extract first scan
            pcl_extract_scan(comp_pose, comp_pose + 20, &cur_scan, 1400);
            // Extract second scan
            pcl_extract_scan(ref_pose, ref_pose + 20, &ref_scan, 1400);
            // Run ICP
            icp(&ref_scan, &cur_scan, 25, 0.3f, icp_res, &icp_job);
            if (!icp_job.has_nan) {
                // Add LC edge
                printf("ICP added LC edge from %d to %d Tr: %.3f %.3f  "
                       "Rot:%.3f \n",
                       comp_pose, ref_pose, icp_res[0], icp_res[1], icp_res[2]);
                pcl_add_constraint(comp_pose, ref_pose, icp_res);
            }
        }
    }

    // Optimize graph
    slam_job_t slam_job = {0};
    pcl_optimize(&slam_job);
    pcl_to_poses(pos_test, 0, SIZE_POSES);
    printf("SLAM execution time [ms]: %d \n", slam_job.time_ms);
    printf("Main graph: %d \n", slam_job.main_graph_size);
    printf("Largest subgraph: %d \n", slam_job.max_subgraph_size);

    // Evaluate results
    float xy_error = 0.0f;
    float yaw_error = 0.0f;
    for (int16_t i = 0; i < SIZE_POSES; i++) {
        float error0 = fabs(pos_test[i][0] - pos_gt[i][0]);
        float error1 = fabs(pos_test[i][1] - pos_gt[i][1]);
        float error2 = fabs(pos_test[i][2] - pos_gt[i][2]);
        if (error0 + error1 > xy_error) xy_error = error0 + error1;
        if (error2 > yaw_error) yaw_error = error2;
    }
    if (xy_error < 0.001f && yaw_error < 0.001f) {
        printf("Unit test passed!");
    } else {
        printf("Unit test failed!");
    }
}

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

    unit_test_slam();

    return errors;
}

/* Program Entry. */
int main(void) { return pmsis_kickoff((void *)main_func); }
