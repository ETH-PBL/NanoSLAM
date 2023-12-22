/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SPI_COMM_H
#define SPI_COMM_H

#include "scan.h"

typedef struct {
    float error;
    int16_t time_ms;
    int16_t has_nan;
    float result[3];
} icp_job_t;

typedef struct {
    int16_t number_of_poses;
    int16_t main_graph_size;
    int16_t max_subgraph_size;
    int16_t time_ms;
    int16_t has_nan;
} slam_job_t;

void spi_send_pose(full_pose_t *pose);
void spi_send_scan_info(int16_t from_id, int16_t to_id, icp_job_t *job);
void spi_rcv_poses(float poses[][3], int16_t from_id, int16_t nr_of_poses);
void spi_send_opt_cmd(slam_job_t *job);
int16_t spi_pose_count();
void spi_sanity_check(int16_t *state);

#endif