/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "scan_log.h"
#include "FreeRTOS.h"
#include "deck.h"
#include "estimator_kalman.h"
#include "log.h"
#include "spi_comm.h"
#include "task.h"
#include <debug.h>
#include <math.h>
#include <string.h>

#define MAX_SCANS 50
static scan_info_t scan_log[MAX_SCANS];
static int16_t scan_cnt = 0;
static scan_info_t last_scan = {.pose_id = 0, .x = 100, .y = 100, .yaw = 0};

int16_t add_scan_to_log(float x, float y, float yaw, int16_t pose_id) {
    if (scan_cnt >= MAX_SCANS) return -1;

    scan_log[scan_cnt].pose_id = pose_id;
    scan_log[scan_cnt].x = x;
    scan_log[scan_cnt].y = y;
    scan_log[scan_cnt].yaw = yaw;
    scan_cnt++;

    last_scan.x = x;
    last_scan.y = y;

    return (scan_cnt - 1);
}

uint8_t check_for_nearby_scans(float range, scan_info_t *scan_log0) {
    float x = logGetFloat(logGetVarId("stateEstimate", "x"));
    float y = logGetFloat(logGetVarId("stateEstimate", "y"));

    // DEBUG_PRINT("Go\n");
    for (int16_t i = 0; i < scan_cnt; i++) {
        float scan_x = scan_log[i].x;
        float scan_y = scan_log[i].y;
        float dist =
            sqrtf((x - scan_x) * (x - scan_x) + (y - scan_y) * (y - scan_y));

        if (dist < range) {
            memcpy(scan_log0, &scan_log[i], sizeof(scan_info_t));
            return 1;
        }
    }
    return 0;
}

uint8_t last_scan_further_than(float dist_th) {
    float dist;
    float x = logGetFloat(logGetVarId("stateEstimate", "x"));
    float y = logGetFloat(logGetVarId("stateEstimate", "y"));

    dist = sqrtf((x - last_scan.x) * (x - last_scan.x) +
                 (y - last_scan.y) * (y - last_scan.y));

    if (dist > dist_th)
        return 1;
    else
        return 0;
}

void update_optimized_poses() {
    DEBUG_PRINT("Total scans %d\n", scan_cnt);
    for (int16_t i = 0; i < scan_cnt; i++) {
        float poses[3];
        DEBUG_PRINT("Requesting %d\n", scan_log[i].pose_id);
        spi_rcv_poses(poses, scan_log[i].pose_id, 1);
        float scan_x_old = scan_log[i].x;
        float scan_y_old = scan_log[i].y;
        scan_log[i].x = poses[0];
        scan_log[i].y = poses[1];
        DEBUG_PRINT("Updated pose %d: %.2f %.2f from %.2f %.2f\n",
                    scan_log[i].pose_id, poses[0], poses[1], scan_x_old,
                    scan_y_old);
        vTaskDelay(M2T(1));
    }
}

void correct_position() {
    float x_cur = logGetFloat(logGetVarId("stateEstimate", "x"));
    float y_cur = logGetFloat(logGetVarId("stateEstimate", "y"));
    float z_cur = logGetFloat(logGetVarId("stateEstimate", "z"));
    float yaw_cur = logGetFloat(logGetVarId("stateEstimate", "yaw"));
    yaw_cur = yaw_cur / 180.0f * (float)M_PI;

    int16_t pose_id = scan_log[scan_cnt - 1].pose_id;
    float pose_opt[3];
    spi_rcv_poses(pose_opt, pose_id, 1);

    float dx = pose_opt[0] - scan_log[scan_cnt - 1].x;
    float dy = pose_opt[1] - scan_log[scan_cnt - 1].y;
    float dyaw = pose_opt[2] - scan_log[scan_cnt - 1].yaw;

    DEBUG_PRINT("Corrected from (%.2f, %.2f, %.2f)\n", x_cur, y_cur, yaw_cur);
    DEBUG_PRINT("Corrected to (%.2f, %.2f, %.2f)\n", x_cur + dx, y_cur + dy,
                yaw_cur + dyaw);
    vTaskDelay(5);
    estimatorKalmanInitToPose(x_cur + dx, y_cur + dy, z_cur, yaw_cur + dyaw);
}