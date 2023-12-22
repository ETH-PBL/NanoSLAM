/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SCAN_LOG_H
#define SCAN_LOG_H

typedef enum {
    NEW = 0,
    EXISTING = 1,
} scan_type_t;

typedef struct {
    int16_t pose_id;
    float x;
    float y;
    float yaw;
} scan_info_t;

int16_t add_scan_to_log(float x, float y, float yaw, int16_t pose_id);
// int16_t get_last_scan_id();
uint8_t check_for_nearby_scans(float range, scan_info_t *scan_log0);
uint8_t last_scan_further_than(float dist_th);
void update_optimized_poses();
void correct_position();

#endif