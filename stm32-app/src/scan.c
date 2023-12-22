/*
 * Authors: The code is authored by Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#include "scan.h"
#include "FreeRTOS.h"
#include "crtp_commander_high_level.h"
#include "log.h"
#include "task.h"
#include <math.h>
#include <stdlib.h>

#include "tof_matrix.h"

#define DEBUG_MODULE __FILE__

#include "debug.h"

static int16_t pose_cnt = 0;

int compare_int16(const void *a, const void *b) {
    return (*(int16_t *)a - *(int16_t *)b);
}

void scan_extract_points(const int16_t tof_data[4][8],
                         scan_frame_t *scan_frame) {
    memset(scan_frame, 0, sizeof(scan_frame_t));
    const float step = -((float)M_PI_4) / 8;

    scan_frame->num = 0;

    for (int dir = 0; dir < 4; dir++) {
        int valid_pixels = 0;
        float dir_yaw = 0.0f;
        if (dir == DIR_BACK)
            dir_yaw += (float)M_PI;
        else if (dir == DIR_LEFT)
            dir_yaw += (float)M_PI_2;
        else if (dir == DIR_RIGHT)
            dir_yaw -= (float)M_PI_2;
        float angle = -4 * step + step / 2;
        for (int col = 0; col < 8; angle += step, col++) {
            // Check if measurement is valid
            int16_t measurement = tof_data[dir][col];
            if (measurement < 0) continue;
            if (measurement > 800.0f) continue;

            // Apply rotation and add point
            float dist_x = (float)measurement / 1000.0f;
            float dist_y = tanf(angle) * dist_x;

            scan_frame->x[scan_frame->num] =
                dist_x * cosf(dir_yaw) - dist_y * sinf(dir_yaw);
            scan_frame->y[scan_frame->num] =
                dist_x * sinf(dir_yaw) + dist_y * cosf(dir_yaw);
            scan_frame->num++;
            valid_pixels++;
        }
        if (valid_pixels < 3) scan_frame->num -= valid_pixels;
    }
}

uint8_t get_full_pose(int16_t tof_data_buf[4][8][8],
                      full_pose_t *complete_pose) {
    memset(complete_pose, 0, sizeof(full_pose_t));
    // Read measurements from each sensor
    for (int16_t dir = 0; dir < 4; dir++) {
        extract_measurements(tof_data_buf[dir], complete_pose->tof[dir]);
    }

    complete_pose->pos[0] = logGetFloat(logGetVarId("stateEstimate", "x"));
    complete_pose->pos[1] = logGetFloat(logGetVarId("stateEstimate", "y"));
    complete_pose->pos[2] =
        logGetFloat(logGetVarId("stateEstimate", "yaw")) / 180.0f * (float)M_PI;
    complete_pose->timestamp = (int32_t)xTaskGetTickCount();
    complete_pose->id = pose_cnt;
    pose_cnt++;
    return 1;
}

void extract_measurements(const int16_t matrix[8][8], int16_t measurements[8]) {

    for (int col = 0; col < 8; col++) {

        // Extract center four pixels of each column discarding invalid pixels
        int16_t rows[4];
        uint8_t num_rows = 0;
        for (int i = 2; i < 6; i++) {
            if (matrix[i][col] >= 0) {
                rows[num_rows++] = matrix[i][col];
            }
        }
        if (num_rows == 0) {
            measurements[col] = -1;
            continue;
        }

        // Find median
        qsort(rows, num_rows, sizeof(int16_t), compare_int16);
        if (num_rows & 1) {
            measurements[col] = rows[num_rows >> 1];
        } else {
            measurements[col] =
                (rows[num_rows >> 1] + rows[(num_rows >> 1) - 1]) >> 1;
        }
    }
}

int16_t get_last_pose_id() { return pose_cnt - 1; }