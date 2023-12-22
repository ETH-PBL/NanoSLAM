/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "point_cloud.h"
#include "../icp/icp.h"
#include "../slam/config_size.h"
#include "graph-based-slam.h"
#include "pmsis.h"
#include "utils_array.h"
#include <stdio.h>
#include <stdlib.h>

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

#define TOF_FREQ_HZ (10)
#define MAX_FLIGH_TIME_S (300)

#define MAX_CONSTRAINTS (50)

PI_L2 pcl_pose_t p_cloud[TOF_FREQ_HZ * MAX_FLIGH_TIME_S];
static int32_t pose_counter = 0;

PI_L2 float x[MAX_SIZE];
PI_L2 int16_t pose_map[MAX_SIZE];
PI_L2 float x_subgraph[(MAX_SIZE / 3)];
PI_L2 constraint constraints[MAX_CONSTRAINTS];
static int32_t constr_counter = 0;

void pcl_add_augmented_pose(pcl_pose_t pose) {
    p_cloud[pose_counter] = pose;
    pose_counter++;
}

int16_t pcl_get_pose_count() { return pose_counter; }

void pcl_extract_scan(int16_t start_id, int16_t end_id, icp_points_t *points,
                      int16_t range_limit) {

    const float offsets[4] = {0.025f, -0.025f, 0.02f, -0.02f};
    const float step = -((float)M_PI_4) / 8;

    points->num = 0;

    for (int16_t pose_id = start_id; pose_id <= end_id; pose_id++) {

        float frame_x = p_cloud[pose_id].pos[0];
        float frame_y = p_cloud[pose_id].pos[1];
        float frame_yaw = p_cloud[pose_id].pos[2];

        for (int dir = 0; dir < 4; dir++) {

            float dir_yaw = frame_yaw;
            if (dir == DIR_BACK)
                dir_yaw += (float)M_PI;
            else if (dir == DIR_LEFT)
                dir_yaw += (float)M_PI_2;
            else if (dir == DIR_RIGHT)
                dir_yaw -= (float)M_PI_2;
            float angle = -4 * step + step / 2;
            for (int col = 0; col < 8; angle += step, col++) {

                // Check if measurement is valid
                int16_t measurement = p_cloud[pose_id].tof[dir][col];
                if (measurement < 0 || measurement > range_limit) continue;

                // Apply rotation and add point
                float dist_x = (float)measurement / 1000.0f;
                float dist_y = tanf(angle) * dist_x;
                dist_x += offsets[dir];
                icp_point_t p = {
                    .x = frame_x + dist_x * cosf(dir_yaw) -
                         dist_y * sinf(dir_yaw),
                    .y = frame_y + dist_x * sinf(dir_yaw) +
                         dist_y * cosf(dir_yaw),
                };
                points->items[points->num++] = p;
            }
        }
    }
}

static constraint comp_constr(float *icp_res, float *ref_pose, float *cur_pose,
                              int16_t ref_pose_idx, int16_t cur_pose_idx) {

    // Prepare SLAM constraint
    float c = cosf(icp_res[2]);
    float s = sinf(icp_res[2]);
    float x = cur_pose[0] * c - cur_pose[1] * s + icp_res[0];
    float y = cur_pose[0] * s + cur_pose[1] * c + icp_res[1];
    float corrected_pose[3] = {x, y, cur_pose[2] + icp_res[2]};
    constraint constraint = {
        .fromIdx = cur_pose_idx,
        .toIdx = ref_pose_idx,
    };
    measurement_from_poses(corrected_pose, ref_pose, constraint.z);

    return constraint;
}

void pcl_add_constraint(int16_t from_node, int16_t to_node, float *icp) {
    constraint c = comp_constr(icp, p_cloud[to_node].pos,
                               p_cloud[from_node].pos, to_node, from_node);
    constraints[constr_counter] = c;
    constr_counter++;
}

static inline float calc_dist(float *p1, float *p2) {
    float dif_x = p1[0] - p2[0];
    float dif_y = p1[1] - p2[1];
    return sqrtf(dif_x * dif_x + dif_y * dif_y);
}

static inline float abs_val(float val) {
    float val_ret = (val > 0) ? val : -val;
    return val_ret;
}

// Creates a sparse graph out of the complete graph. This graph is firstly
// optimized, and then the subgraphs.
static void pcl_reduce(float *x, int16_t *pose_nr, int16_t *scan_poses,
                       int16_t scan_pose_nr) {
    int16_t pose_idx = 0;
    int16_t scan_idx = 0;
    float last_pose[3];
    pose_map[0] = 0;
    memcpy(x, p_cloud[0].pos, 3 * sizeof(float));
    memcpy(last_pose, p_cloud[0].pos, 3 * sizeof(float));
    if (scan_poses[scan_idx] == 0) scan_idx++;
    for (int16_t i = 1; i < pose_counter; i++) {
        if (i == scan_poses[scan_idx] && (scan_idx < scan_pose_nr)) {
            scan_idx++;
            pose_idx++;
            memcpy(&x[3 * pose_idx], p_cloud[i].pos, 3 * sizeof(float));
            memcpy(last_pose, p_cloud[i].pos, 3 * sizeof(float));
            pose_map[pose_idx] = i;
        } else {
            float dist = calc_dist(last_pose, p_cloud[i].pos);
            if (dist > 0.3f ||
                (abs_val(p_cloud[i].pos[2] - last_pose[2]) > 0.4)) {
                pose_idx++;
                memcpy(&x[3 * pose_idx], p_cloud[i].pos, 3 * sizeof(float));
                memcpy(last_pose, p_cloud[i].pos, 3 * sizeof(float));
                pose_map[pose_idx] = i;
            }
        }
    }
    *pose_nr = pose_idx + 1;
}

void pcl_to_poses(float *x, int16_t from_id, int16_t nr_of_poses) {
    if (nr_of_poses <= 0) return;
    for (int16_t i = from_id; i < from_id + nr_of_poses; i++) {
        memcpy(&x[3 * (i - from_id)], p_cloud[i].pos, 3 * sizeof(float));
    }
}

static void pcl_update(float *x, int16_t from_id, int16_t nr_of_poses) {
    if (nr_of_poses <= 0) return;
    for (int16_t i = from_id; i < from_id + nr_of_poses; i++) {
        memcpy(p_cloud[i].pos, &x[3 * (i - from_id)], 3 * sizeof(float));
    }
}

void pcl_optimize(slam_job_t *job) {
    memset(job, 0, sizeof(slam_job_t));

    uint32_t t_start = pi_time_get_us();
    if (constr_counter == 0) return;

    int16_t scan_poses_list[2 * constr_counter];
    for (int16_t i = 0; i < constr_counter; i++) {
        scan_poses_list[2 * i] = constraints[i].toIdx;
        scan_poses_list[2 * i + 1] = constraints[i].fromIdx;
    }

    int16_t scan_pose_nr;
    delete_duplicates_and_sort(scan_poses_list, 2 * constr_counter,
                               &scan_pose_nr);

    int16_t pose_nr;
    pcl_reduce(x, &pose_nr, scan_poses_list, scan_pose_nr);

    for (int i = 0; i < scan_pose_nr; i++) {
        for (int j = 0; j < pose_nr; j++) {
            if (pose_map[j] == constraints[i].toIdx) {
                constraints[i].toIdx = j;
            }
            if (pose_map[j] == constraints[i].fromIdx) {
                constraints[i].fromIdx = j;
                break;
            }
        }
    }

    job->main_graph_size = pose_nr;
    ls_slam(x, constraints, pose_nr, constr_counter);

    for (int i = 0; i < pose_nr - 1; i++) {
        // Build subgraph
        int16_t from_pose_id = pose_map[i];
        int16_t subgraph_size = pose_map[i + 1] - pose_map[i] + 1;
        if (subgraph_size == 2) {
            memcpy(p_cloud[pose_map[i]].pos, &x[3 * i], 3 * sizeof(float));
            continue;
        }
        pcl_to_poses(x_subgraph, from_pose_id, subgraph_size);

        // Offset
        float offset[3];
        offset[0] = x_subgraph[0] - x[3 * i + 0];
        offset[1] = x_subgraph[1] - x[3 * i + 1];
        offset[2] = x_subgraph[2] - x[3 * i + 2];

        for (int j = 0; j < subgraph_size; j++) {
            x_subgraph[3 * j + 0] -= offset[0];
            x_subgraph[3 * j + 1] -= offset[1];
            x_subgraph[3 * j + 2] -= offset[2];
        }

        // Create constraints
        constraint c_sub = {0};
        c_sub.fromIdx = subgraph_size - 1;
        c_sub.toIdx = 0;
        measurement_from_poses(&x[3 * (i + 1)], x_subgraph, c_sub.z);
        if (subgraph_size > job->max_subgraph_size)
            job->max_subgraph_size = subgraph_size;
        ls_slam(x_subgraph, &c_sub, subgraph_size, 1);
        pcl_update(x_subgraph, from_pose_id, subgraph_size - 1);
    }

    int16_t from_pose_id = pose_map[pose_nr - 1];
    int16_t subgraph_size = pose_counter - from_pose_id;
    pcl_to_poses(x_subgraph, from_pose_id, subgraph_size);
    float offset[3];
    offset[0] = x_subgraph[0] - x[3 * (pose_nr - 1) + 0];
    offset[1] = x_subgraph[1] - x[3 * (pose_nr - 1) + 1];
    offset[2] = x_subgraph[2] - x[3 * (pose_nr - 1) + 2];

    for (int j = 0; j < subgraph_size; j++) {
        x_subgraph[3 * j + 0] -= offset[0];
        x_subgraph[3 * j + 1] -= offset[1];
        x_subgraph[3 * j + 2] -= offset[2];
    }
    pcl_update(x_subgraph, from_pose_id, subgraph_size);

    constr_counter = 0;

    for (int16_t i = 0; i < pose_counter; i++) {
        if (isnan(p_cloud[i].pos[0]) || isnan(p_cloud[i].pos[1]) ||
            isnan(p_cloud[i].pos[2])) {
            job->has_nan = 1;
            return;
        }
    }

    job->number_of_poses = pose_counter;

    uint32_t t_stop = pi_time_get_us();
    job->time_ms = (uint16_t)(t_stop - t_start) / 1000;
}
