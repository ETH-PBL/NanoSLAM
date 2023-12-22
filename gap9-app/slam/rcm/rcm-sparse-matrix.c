/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "assert.h"
#include "circular-queue.h"
#include "heapsort.h"
#include "node.h"
#include "pmsis.h"
#include "rcm-sparse-matrix.h"
#include "sparse-matrix.h"

#define MAX_NEIGHBORS 100
PI_L2 node nodes[MAX_SIZE];
PI_L2 circular_queue_t q;
PI_L2 node nodes_line[MAX_NEIGHBORS];
extern float array1[MAX_SIZE];
extern csr sp_mat1;

static int16_t is_neighbour(csr *mat, int16_t N, int16_t i, int16_t j) {
    if (i == j) return 0;

    if (i < j) {
        int16_t aux = i;
        i = j;
        j = aux;
    }

    int16_t res = already_added(mat, N, i, j);

    return res;
}

static void count_all_neighbors(csr *mat, node *nodes, int16_t N) {
    for (int16_t i = 0; i < N; i++) {
        nodes[i].index = i;
        for (int32_t k = mat->row_ptr[i]; k < mat->row_ptr[i + 1]; k++)
            if (mat->col_idx[k] < i) {
                nodes[i].neighbors++;
                nodes[mat->col_idx[k]].neighbors++;
            }
    }
}

void rcm(csr *mat, int16_t *perm, int16_t N) {
    memset(nodes, 0, N * sizeof(node));
    count_all_neighbors(mat, nodes, N);

    // quickSortNodes(nodes, N);
    heapsort_nodes(nodes, N);

    // Init queue
    init_queue(&q);

    int16_t perm_index = 0;
    int16_t nodes_index = 0;

    for (int16_t i = 0; i < N; i++) {
        if (nodes[i].neighbors > 0)
            break;
        else {
            perm[perm_index++] = nodes[i].index;
            nodes[i].visited = 1;
        }
    }

    nodes_index = perm_index;

    node C;
    while (perm_index < N) {
        if (nodes[nodes_index].visited == 0) {
            push_to_queue(&q, nodes[nodes_index]);
            nodes[nodes_index].visited = 1;
        }
        nodes_index++;
        int32_t t0 = pi_time_get_us();
        while (!is_empty(&q)) {
            pop_from_queue(&q, &C);
            perm[perm_index++] = C.index;

            int16_t neighbour_counter = 0;
            for (int16_t i = nodes_index; i < N; i++) {
                if (is_neighbour(mat, N, C.index, nodes[i].index)) {
                    if (nodes[i].visited == 0) {
                        push_to_queue(&q, nodes[i]);
                        nodes[i].visited = 1;
                    }
                    neighbour_counter++;
                    if (neighbour_counter == C.neighbors) break;
                }
            }
        }
        int32_t t1 = pi_time_get_us();
        // printf("Rcm:  %d \n", t1 - t0);
    }

    for (int16_t i = 0; i < N / 2; i++) {
        int16_t temp = perm[i];
        perm[i] = perm[N - i - 1];
        perm[N - i - 1] = temp;
    }
}

static int16_t get_min_order_unvisited(node *nodes, int16_t N) {
    node min_node = {
        .index = 0,
        .neighbors = 10000,
        .visited = 0,
    };
    for (int16_t i = 0; i < N; i++)
        if (nodes[i].neighbors < min_node.neighbors && nodes[i].visited == 0)
            min_node = nodes[i];

    return min_node.index;
}

void rcm_fast(csr *mat, int16_t *perm, int16_t N) {
    csr *mat_tr = &sp_mat1;
    // memset(mat_tr, 0, sizeof(csr));
    init_sp_mat(mat_tr);
    transpose_lower_triangular(mat, mat_tr, N);

    memset(nodes, 0, N * sizeof(node));
    memset(nodes_line, 0, sizeof(nodes_line));

    count_all_neighbors(mat, nodes, N);

    // Init queue
    init_queue(&q);
    int16_t perm_index = 0;

    node C;
    while (perm_index < N) {
        int16_t node_idx = get_min_order_unvisited(nodes, N);
        push_to_queue(&q, nodes[node_idx]);
        nodes[node_idx].visited = 1;

        while (!is_empty(&q)) {
            pop_from_queue(&q, &C);
            perm[perm_index++] = C.index;

            int16_t neighbors_of_C = 0;
            for (int16_t i = mat->row_ptr[C.index];
                 i < mat->row_ptr[C.index + 1]; i++) {
                int16_t idx = mat->col_idx[i];
                if (idx == C.index) continue;

                nodes_line[neighbors_of_C].index = idx;
                nodes_line[neighbors_of_C].neighbors = nodes[idx].neighbors;
                neighbors_of_C++;
            }

            for (int16_t i = mat_tr->row_ptr[C.index];
                 i < mat_tr->row_ptr[C.index + 1]; i++) {
                int16_t idx = mat_tr->col_idx[i];
                if (idx == C.index) continue;
                nodes_line[neighbors_of_C].index = idx;
                nodes_line[neighbors_of_C].neighbors = nodes[idx].neighbors;
                neighbors_of_C++;
            }

            heapsort_nodes(nodes_line, neighbors_of_C);
            // printf("Node %d adds...\n", C.index);
            for (int16_t i = 0; i < neighbors_of_C; i++) {
                // printf("    Neighbr %d\n", nodes_line[i].index);
                node_idx = nodes_line[i].index;
                if (nodes[node_idx].visited == 0) {
                    push_to_queue(&q, nodes[node_idx]);
                    nodes[node_idx].visited = 1;
                }
            }
        }
    }

    for (int16_t i = 0; i < N / 2; i++) {
        int16_t temp = perm[i];
        perm[i] = perm[N - i - 1];
        perm[N - i - 1] = temp;
    }

    int32_t checksum = 0;
    for (int16_t i = 0; i < N; i++) checksum += perm[i];

    ASSERT(checksum == (N * (N - 1) / 2), "RCM CHECKSUM ERROR! \n");
}

void permutation_inverse(int16_t *perm, int16_t *perm_inv, int N) {
    for (int16_t i = 0; i < N; i++) perm_inv[perm[i]] = i;
}

void permute_array(float *array, int16_t *perm, int16_t N) {
    for (int16_t i = 0; i < N; i++) array1[i] = array[perm[i]];

    memcpy(array, array1, N * sizeof(float));
}
