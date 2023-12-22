/*
 * Authors: The code is based on the implementation of Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#ifndef SLAM_FIRMWARE_ICP_H
#define SLAM_FIRMWARE_ICP_H

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "pmsis.h"

#define SIZE_POINTS 700

typedef struct {
    float x;
    float y;
} icp_point_t;

typedef struct {
    size_t num;
    icp_point_t items[SIZE_POINTS];
} icp_points_t;

typedef struct {
    float error;
    int16_t time_ms;
    int16_t has_nan;
} icp_job_t;

static inline float point_norm(icp_point_t p) {
    return sqrtf(p.x * p.x + p.y * p.y);
}

static inline void point_normalize(icp_point_t *p) {
    float norm = point_norm(*p);
    p->x /= norm;
    p->y /= norm;
}

static inline float points_dist_squared(icp_point_t a, icp_point_t b) {
    float x = a.x - b.x;
    float y = a.y - b.y;
    return x * x + y * y;
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]);

void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, float *rotation, icp_job_t *job);

void compute_correspondences_parallel(const icp_points_t *P,
                                      const icp_points_t *Q, float error_bound,
                                      int16_t cor_P[], int16_t cor_Q[]);

#endif // SLAM_FIRMWARE_ICP_H
