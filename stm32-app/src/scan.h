/*
 * Authors: The code is authored by Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#ifndef SCAN_H
#define SCAN_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DIR_FRONT = 0,
    DIR_BACK = 1,
    DIR_LEFT = 2,
    DIR_RIGHT = 3,
} direction_t;

typedef struct {
    int16_t num;
    float x[32];
    float y[32];
} scan_frame_t;

typedef struct {
    int16_t id;
    int32_t timestamp;
    float pos[3];
    int16_t tof[4][8];
} full_pose_t;

#define SCAN_SIZE 15

int compare_int16(const void *a, const void *b);
uint8_t get_full_pose(int16_t tof_data_buf[4][8][8],
                      full_pose_t *complete_pose);
void scan_extract_points(const int16_t tof_data[4][8],
                         scan_frame_t *scan_frame);
void extract_measurements(const int16_t matrix[8][8], int16_t measurements[8]);
int16_t get_last_pose_id();

#endif // SCAN_H
