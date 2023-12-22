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

#define FREQU_FC 400 * 1000 * 1000
#define FREQU_CL 400 * 1000 * 1000

struct pi_device cluster_dev;
struct pi_cluster_conf cl_conf;

#include "gpio_config.h"
#include "icp/icp.h"
#include "pmsis.h"
#include "point_cloud.h"
#include "rcm-sparse-matrix.h"
#include "spi_cmds_decode.h"
#include "spi_driver.h"

static uint8_t *spi_buf, *spi_buf_t;
PI_L2 icp_points_t ref_scan, cur_scan;

int get_spi_command(uint8_t *spi_cmd, uint8_t len) {
    if (spi_cmd[0] == 'S' && spi_cmd[len - 1] == 'E')
        return spi_cmd[1];
    else
        return -1;
}

void slam_loop() {
    gpio_init();
    spi_buf = pi_l2_malloc(SPIBUF_SIZE);
    spi_buf_t = pi_l2_malloc(3 * SPIBUF_SIZE + 1);
    spi_slave_driver_init();

    pcl_pose_t pose0 = {0};
    int16_t scan_info[2];
    int16_t pose_request[2];
    float icp_res[3];
    slam_job_t slam_job = {0};
    icp_job_t icp_job = {0};

    while (1) {
        // Wait new SPI packet
        spi_rcv_blocking(spi_buf, SPI_PKT_SIZE);
        pi_time_wait_us(100);
        int cmd = get_spi_command(spi_buf, SPI_PKT_SIZE);
        switch (cmd) {
        case POSE_CMD: // Add augmented pose: pose + 4 x ToF
            if (spi_decode_pose(spi_buf, &pose0) == SPI_OK) {
                pcl_add_augmented_pose(pose0);
            }
            break;

        case SCAN_CMD: // New Loop closure
            if (spi_decode_scan_info(spi_buf, scan_info) == SPI_OK) {
                int16_t ref_pose = scan_info[1];
                int16_t comp_pose = scan_info[0];
                if ((ref_pose >= 0) && (comp_pose >= 0)) {
                    memset(&icp_job, 0, sizeof(icp_job_t));
                    memset(icp_res, 0, 3 * sizeof(float));
                    set_cf_gpio(1); // GAP9 busy
                    if (comp_pose + 20 < pcl_get_pose_count()) {
                        // Extract first scan
                        pcl_extract_scan(comp_pose, comp_pose + 20, &cur_scan,
                                         1800);
                        // Extract second scan
                        pcl_extract_scan(ref_pose, ref_pose + 20, &ref_scan,
                                         1800);
                        // Run ICP between two scans
                        icp(&ref_scan, &cur_scan, 24, 0.3f, icp_res, &icp_job);
                        if ((icp_job.has_nan == 0) && (icp_job.error < 0.01)) {
                            // If ICP scan-matching is succesful, add new edge
                            pcl_add_constraint(comp_pose, ref_pose, icp_res);
                        }
                    }
                    set_cf_gpio(0); // GAP9 free

                    // Send feedback to the STM32
                    memcpy(spi_buf_t, &icp_job, sizeof(icp_job_t));
                    memcpy(spi_buf_t + sizeof(icp_job_t), icp_res,
                           3 * sizeof(float));
                    spi_send_blocking(spi_buf_t,
                                      sizeof(icp_job_t) + 3 * sizeof(float));
                }
            }
            break;

        case OPTIMIZE_CMD:
            set_cf_gpio(1);          // GAP9 busy
            pcl_optimize(&slam_job); // Optimize graph
            set_cf_gpio(0);          // GAP9 free

            // Send feedback to the STM3
            memcpy(spi_buf_t, &slam_job, sizeof(slam_job_t));
            spi_send_blocking(spi_buf_t, sizeof(slam_job_t));
            break;

        case POSE_REQ_CMD: // Send poses back to STM32
            if (spi_decode_pose_req(spi_buf, pose_request) == SPI_OK) {
                int16_t from_id = pose_request[0];
                int16_t nr_of_poses = pose_request[1];
                if (nr_of_poses > 100) break;
                pcl_to_poses((float *)(spi_buf_t), from_id, nr_of_poses);

                spi_send_blocking(spi_buf_t, 3 * 4 * nr_of_poses);
            }
            break;
        }
    }
}

int main_func(void) {
    pi_freq_set(PI_FREQ_DOMAIN_FC, FREQU_FC);
    pi_freq_set(PI_FREQ_DOMAIN_CL, FREQU_CL);
    int32_t cur_fc_freq = pi_freq_get(PI_FREQ_DOMAIN_FC);
    int32_t cur_cl_freq = pi_freq_get(PI_FREQ_DOMAIN_CL);

    printf("FC frequency : %ld\nCL frequency : %ld\n", cur_fc_freq,
           cur_cl_freq);

    /* Init cluster configuration structure. */
    pi_cluster_conf_init(&cl_conf);
    cl_conf.id = 0;

    /* Configure & open cluster. */
    pi_open_from_conf(&cluster_dev, &cl_conf);
    if (pi_cluster_open(&cluster_dev)) {
        printf("Cluster open failed !\n");
        pmsis_exit(-1);
    }

    slam_loop();

    printf("Program exits!\n");
    return 0;
}

int main(void) { return pmsis_kickoff((void *)main_func); }
