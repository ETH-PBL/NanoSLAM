/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "scan.h"
#include <math.h>
#include <string.h>

#define THETA_LEN 18
#define RHO_LEN 32
#define MIN_POINTS_ON_LINE 5
#define MAX_LINES 30

static uint8_t get_index(float *array, uint8_t len, float val) {
    for (uint8_t i = 0; i < len; i++) {
        if (val >= array[i] && val < array[i + 1]) return i;
    }

    return 255;
}

uint8_t corner_detected_hough(scan_frame_t *scan_frame) {
    if (scan_frame->num < 15) return 0;

    float thetas[THETA_LEN];
    float th_cos[THETA_LEN];
    float th_sin[THETA_LEN];
    float rhos[RHO_LEN];

    float theta_step = 180.0f / (float)THETA_LEN;
    for (int16_t i = 0; i < THETA_LEN; i++) {
        thetas[i] = ((float)i * theta_step - 90.0f) / 180.0f * (float)M_PI;
        th_sin[i] = sinf(thetas[i]);
        th_cos[i] = cosf(thetas[i]);
    }

    float rho_step = 3.0f / (RHO_LEN - 1);
    for (int16_t i = 0; i < RHO_LEN; i++)
        rhos[i] = ((float)i * rho_step - 1.5f);

    uint8_t accumulator[THETA_LEN][RHO_LEN - 1];
    memset(accumulator, 0, sizeof(accumulator));

    for (int16_t i = 0; i < THETA_LEN; i++) {
        for (int16_t j = 0; j < scan_frame->num; j++) {
            float x = scan_frame->x[j];
            float y = scan_frame->y[j];
            float rho = y * th_cos[i] - x * th_sin[i];
            uint8_t rho_idx = get_index(rhos, RHO_LEN, rho);
            if (rho_idx != 255) accumulator[i][rho_idx] += 1;
        }
    }

    float lines_slope[MAX_LINES];
    int16_t lines_cnt = 0;
    for (int16_t i = 0; i < THETA_LEN; i++) {
        for (int16_t j = 0; j < RHO_LEN - 1; j++) {
            if (lines_cnt == MAX_LINES) return 0;
            if (accumulator[i][j] > MIN_POINTS_ON_LINE) {
                lines_slope[lines_cnt] = thetas[i];
                lines_cnt++;
            }
        }
    }

    float slope_bound_l = M_PI / 2 - M_PI / 6;
    float slope_bound_h = M_PI / 2 + M_PI / 6;
    for (int16_t i = 0; i < lines_cnt; i++) {
        for (int16_t j = 0; j < lines_cnt; j++) {
            float slope_diff = fabs(lines_slope[i] - lines_slope[j]);
            if (slope_diff < slope_bound_h && slope_diff > slope_bound_l)
                return 1;
        }
    }

    return 0;
}

uint8_t corner_detected_eigen(scan_frame_t *scan_frame) {
    if (scan_frame->num < 15) return 0;

    float x_mean = 0.0f;
    float y_mean = 0.0f;
    for (int16_t i = 0; i < scan_frame->num; i++) {
        x_mean += scan_frame->x[i];
        y_mean += scan_frame->y[i];
    }
    x_mean /= scan_frame->num;
    y_mean /= scan_frame->num;

    for (int16_t i = 0; i < scan_frame->num; i++) {
        scan_frame->x[i] -= x_mean;
        scan_frame->y[i] -= y_mean;
    }

    float C[2][2] = {{0, 0}, {0, 0}};
    for (int16_t i = 0; i < scan_frame->num; i++) {
        C[0][0] += scan_frame->x[i] * scan_frame->x[i];
        C[0][1] += scan_frame->x[i] * scan_frame->y[i];
        C[1][1] += scan_frame->y[i] * scan_frame->y[i];
    }
    C[0][0] /= scan_frame->num;
    C[1][1] /= scan_frame->num;
    C[0][1] /= scan_frame->num;
    C[1][0] = C[0][1];

    // Eigenvalue calculation
    float a = C[0][0];
    float b = C[0][1];
    float c = C[1][1];

    float delta = (a + c) * (a + c) - 4.0f * (a * c - b * b);
    float x1 = 0.5f * (a + c - sqrtf(delta));
    float x2 = 0.5f * (a + c + sqrtf(delta));

    if (x1 == 0 || x2 == 0) return 0;

    float ratio;
    if (x1 > x2)
        ratio = x2 / x1;
    else
        ratio = x1 / x2;

    if (ratio > 0.25f)
        return 1;
    else
        return 0;
}