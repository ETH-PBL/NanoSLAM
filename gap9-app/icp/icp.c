/*
 * Authors: The code is based on the implementation of Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 * This code represents a parallel version of the original implementation.
 */

#include <stdint.h>
#include <stdio.h>

#include "../dma/dma_transfers.h"
#include "assert.h"
#include "config_size.h"
#include "icp.h"

extern struct pi_device cluster_dev;
static struct pi_cluster_task cl_task;
static pi_task_t task_block_illegal;

#define H_ZERO                                                                 \
    { 1, 0, 0, 0, 1, 0, 0, 0, 1 }

PI_L2 int16_t cor_P[SIZE_POINTS];
PI_L2 int16_t cor_Q[SIZE_POINTS];

PI_L2 icp_points_t cloud0, cloud1, cloud2;

PI_CL_L1 int16_t cor_P_cl[SIZE_POINTS];
PI_CL_L1 int16_t cor_Q_cl[NUM_CORES][SIZE_POINTS];
PI_CL_L1 icp_points_t cloud0_cl, cloud1_cl;

static void matmul_2x2(const float a[4], const float b[4], float c[4]) {
    c[0] = a[0] * b[0] + a[1] * b[2];
    c[1] = a[0] * b[1] + a[1] * b[3];
    c[2] = a[2] * b[0] + a[3] * b[2];
    c[3] = a[2] * b[1] + a[3] * b[3];
}

static void matmul_3x3(const float a[9], const float b[9], float c[9]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            c[i * 3 + j] = 0;
            for (int k = 0; k < 3; k++) {
                c[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
            }
        }
    }
}

static icp_point_t matpointmul(const float r[4], const icp_point_t p) {
    icp_point_t out = {
        .x = r[0] * p.x + r[1] * p.y,
        .y = r[2] * p.x + r[3] * p.y,
    };
    return out;
}

static void compute_correspondences(const icp_points_t *P,
                                    const icp_points_t *Q, float error_bound,
                                    int16_t cor_P[], int16_t cor_Q[]) {
    ASSERT(P->num <= SIZE_POINTS, "Scan size exceeded!");
    ASSERT(Q->num <= SIZE_POINTS, "Scan size exceeded!");
    int16_t p_num = (int16_t)P->num;
    int16_t q_num = (int16_t)Q->num;
    memset(cor_Q, 0, Q->num * sizeof(int16_t));
    // memset(cor_P, 0, P->num * sizeof(int16_t));
    for (int16_t i = 0; i < p_num; ++i) {
        float min_dist = error_bound;
        int16_t chosen_idx = -1;
        icp_point_t cur_point = P->items[i];
        for (int16_t j = 0; j < q_num; ++j) {
            float dist = points_dist_squared(cur_point, Q->items[j]);
            if (dist < min_dist) {
                min_dist = dist;
                chosen_idx = j;
            }
        }
        cor_P[i] = chosen_idx;
        if (chosen_idx != -1) cor_Q[chosen_idx]++;
    }
}

/////////////////////////

typedef struct {
    icp_points_t *P;
    icp_points_t *Q;
    float error_bound;
    int16_t *cor_P;
    int16_t *cor_Q;
} fabric_job;

void cluster_icp_loop(void *args) {
    int32_t core_id = (int32_t)(pi_core_id());

    icp_points_t *P = (icp_points_t *)&cloud0_cl;
    icp_points_t *Q = (icp_points_t *)&cloud1_cl;

    fabric_job *job_fc = (fabric_job *)args;
    int16_t p_num = (int16_t)job_fc->P->num;
    int16_t q_num = (int16_t)job_fc->Q->num;
    float error_bound = job_fc->error_bound;

    int16_t start = core_id * p_num / NUM_CORES;
    int16_t stop = (core_id + 1) * p_num / NUM_CORES;

    for (int16_t i = start; i < stop; i++) {
        float min_dist = error_bound;
        int16_t chosen_idx = -1;
        icp_point_t cur_point = P->items[i];
        for (int16_t j = 0; j < q_num; ++j) {
            float dist = points_dist_squared(cur_point, Q->items[j]);
            if (dist < min_dist) {
                min_dist = dist;
                chosen_idx = j;
            }
        }

        cor_P_cl[i] = chosen_idx;
        if (chosen_idx != -1) cor_Q_cl[core_id][chosen_idx]++;
    }
    pi_cl_team_barrier();
}

void master_core_delegate_correspondences(void *args) {
    fabric_job *job_fc = (fabric_job *)args;

    // Cluster pointers
    icp_points_t *P = (icp_points_t *)&cloud0_cl;
    icp_points_t *Q = (icp_points_t *)&cloud1_cl;
    P->num = job_fc->P->num;
    Q->num = job_fc->Q->num;

    // DMA transfers - fetch data from l2
    cl_memcpy_from_l2(P->items, job_fc->P->items,
                      job_fc->P->num * sizeof(icp_point_t));
    cl_memcpy_from_l2(Q->items, job_fc->Q->items,
                      job_fc->Q->num * sizeof(icp_point_t));

    ASSERT(P->num <= SIZE_POINTS, "Scan size exceeded!");
    ASSERT(Q->num <= SIZE_POINTS, "Scan size exceeded!");
    int16_t p_num = (int16_t)P->num;
    int16_t q_num = (int16_t)Q->num;
    memset(cor_Q_cl, 0, sizeof(cor_Q_cl));
    // memset(cor_P_cl, 0, sizeof(cor_P_cl));

    pi_cl_team_fork(NUM_CORES, cluster_icp_loop, (void *)job_fc);

    for (int16_t j = 0; j < q_num; j++)
        for (int16_t k = 1; k < NUM_CORES; k++)
            cor_Q_cl[0][j] += cor_Q_cl[k][j];

    cl_memcpy_to_l2(cor_P, cor_P_cl, p_num * sizeof(int16_t));
    cl_memcpy_to_l2(cor_Q, cor_Q_cl[0], q_num * sizeof(int16_t));
}

void compute_correspondences_parallel(const icp_points_t *P,
                                      const icp_points_t *Q, float error_bound,
                                      int16_t cor_P[], int16_t cor_Q[]) {
    fabric_job job_fc = {
        .P = P,
        .Q = Q,
        .error_bound = error_bound,
        .cor_P = cor_P,
        .cor_Q = cor_Q,
    };
    pi_task_block(&task_block_illegal);
    pi_cluster_send_task_to_cl_async(
        &cluster_dev,
        pi_cluster_task(&cl_task, master_core_delegate_correspondences,
                        &job_fc),
        &task_block_illegal);
    pi_task_wait_on(&task_block_illegal);
}

static void center_data_P(const icp_points_t *points, const int16_t *cor,
                          icp_point_t *center, icp_points_t *centered) {
    float x = 0;
    float y = 0;
    for (int16_t i = 0; i < (int16_t)points->num; ++i) {
        if (cor[i] < 0) continue;
        x += points->items[i].x;
        y += points->items[i].y;
    }
    x /= (float)points->num;
    y /= (float)points->num;
    for (int16_t i = 0; i < (int16_t)points->num; i++) {
        if (cor[i] < 0) continue;
        centered->items[i].x = points->items[i].x - x;
        centered->items[i].y = points->items[i].y - y;
    }
    center->x = x;
    center->y = y;
}

static void center_data_Q(const icp_points_t *points, const int16_t *cor,
                          icp_point_t *center, icp_points_t *centered) {
    float x = 0;
    float y = 0;
    size_t num = 0;
    for (int16_t i = 0; i < (int16_t)points->num; i++) {
        if (!cor[i]) continue;
        float count = (float)cor[i];
        x += points->items[i].x * count;
        y += points->items[i].y * count;
        num += cor[i];
    }

    x /= (float)num;
    y /= (float)num;
    for (int16_t i = 0; i < (int16_t)points->num; i++) {
        if (!cor[i]) continue;
        centered->items[i].x = points->items[i].x - x;
        centered->items[i].y = points->items[i].y - y;
    }
    center->x = x;
    center->y = y;
}

static void compute_cross_variance(icp_points_t *P, icp_points_t *Q,
                                   const int16_t cor[], float cov[4]) {
    memset(cov, 0, sizeof(float[4]));
    for (size_t i = 0; i < P->num; ++i) {
        int16_t j = cor[i];
        if (j < 0) continue;
        icp_point_t p = P->items[i];
        icp_point_t q = Q->items[j];
        cov[0] += p.x * q.x;
        cov[1] += p.y * q.x;
        cov[2] += p.x * q.y;
        cov[3] += p.y * q.y;
    }
}

void svd(const float in[4], float u[4], float sigma[4], float v[4]) {

    float a = in[0], b = in[1], c = in[2], d = in[3];

    float e = 0.5f * (a + d);
    float f = 0.5f * (a - d);
    float g = 0.5f * (b + c);
    float h = 0.5f * (c - b);

    if (sigma != NULL) {

        float q = sqrtf(e * e + h * h);
        float r = sqrtf(f * f + g * g);

        float sx = q + r;
        float sy = q - r;

        sigma[0] = sx;
        sigma[1] = 0;
        sigma[2] = 0;
        sigma[3] = sy;
    }

    float a1 = atan2f(g, f);
    float a2 = atan2f(h, e);

    float theta = 0.5f * (a2 - a1);
    float phi = 0.5f * (a2 + a1);

    float theta_sin = sinf(theta);
    float theta_cos = cosf(theta);
    float phi_sin = sinf(phi);
    float phi_cos = cosf(phi);

    u[0] = theta_cos;
    u[1] = -theta_sin;
    u[2] = theta_sin;
    u[3] = theta_cos;

    v[0] = phi_cos;
    v[1] = -phi_sin;
    v[2] = phi_sin;
    v[3] = phi_cos;
}

/***
 * Performs the iterative closest point algorithm.
 * @param Q Original point cloud to match against.
 * @param P Moved point cloud that will be matched.
 * @param max_iterations Maximum number of iterations.
 */
void icp(const icp_points_t *Q, const icp_points_t *P, int max_iterations,
         float error_bound, float *result, icp_job_t *job) {

    memset(job, 0, sizeof(icp_job_t));
    uint32_t t_start = pi_time_get_us();

    if ((Q->num == 0) || (P->num == 0)) return;

    // Allocate additional memory
    icp_points_t *P_copy = (icp_points_t *)&cloud0;
    memcpy(P_copy, P, sizeof(icp_points_t));

    icp_points_t *P_centered = (icp_points_t *)&cloud1;
    icp_points_t *Q_centered = (icp_points_t *)&cloud2;
    memset(P_centered, 0, sizeof(icp_points_t));
    memset(Q_centered, 0, sizeof(icp_points_t));
    P_centered->num = P->num;
    Q_centered->num = Q->num;

    float t_a[9] = H_ZERO, t_b[9] = H_ZERO;
    float *next = t_a, *prev = t_b;
    for (int16_t iteration = 0; iteration < max_iterations; iteration++) {
        // Compute correspondences
        compute_correspondences_parallel(P_copy, Q, error_bound, cor_P, cor_Q);
        // compute_correspondences(P_copy, Q, error_bound, cor_P, cor_Q);

        // Center the data for P and Q (based on correspondences)
        icp_point_t center_of_P, center_of_Q;
        center_data_P(P_copy, cor_P, &center_of_P, P_centered);
        center_data_Q(Q, cor_Q, &center_of_Q, Q_centered);

        // Computer the cross variance on centered data
        float cov[4];
        compute_cross_variance(P_centered, Q_centered, cor_P, cov);

        // Execute SVD to obtain transformation
        float u[4], sigma[4], v[4], r[4];
        svd(cov, u, sigma, v);

        matmul_2x2(u, v, r);
        icp_point_t t = matpointmul(r, center_of_P);
        t.x = center_of_Q.x - t.x;
        t.y = center_of_Q.y - t.y;

        // Chain transformations using homogenous coordinates
        float transform[9] = {
            r[0], r[1], t.x, r[2], r[3], t.y, 0, 0, 1,
        };
        matmul_3x3(transform, prev, next);
        float *tmp = next;
        next = prev;
        prev = tmp;

        // Apply transformation to point cloud
        for (size_t i = 0; i < P_copy->num; ++i) {
            icp_point_t p = matpointmul(r, P_copy->items[i]);
            p.x += t.x;
            p.y += t.y;
            P_copy->items[i] = p;
        }

        // Compute ICP error
        float icp_error = 0.0f;
        int16_t error_count = 0;
        for (size_t i = 0; i < P_copy->num; ++i) {
            int16_t j = cor_P[i];
            if (j < 0) continue;

            icp_error += points_dist_squared(P_copy->items[i], Q->items[j]);
            error_count++;
        }
        job->error = icp_error / (float)error_count;
    }

    // Write transform result
    result[0] = prev[2];
    result[1] = prev[5];
    result[2] = atan2f(prev[3], prev[0]);

    uint32_t t_stop = pi_time_get_us();
    job->time_ms = (uint16_t)(t_stop - t_start) / 1000;

    if (isnan(result[0]) || isnan(result[1]) || isnan(result[2])) {
        job->has_nan = 1;
    }
}
