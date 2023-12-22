/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef GRAPH_BASED_SLAM_H
#define GRAPH_BASED_SLAM_H

#include "pmsis.h"
#include "sparse-matrix.h"

// #include "run_on_pc.h"

typedef struct {
    int16_t fromIdx;
    int16_t toIdx;
    float z[3];
} constraint;

typedef struct {
    float z[3];
} measurement;

/**
 * Calculates the jacobians of A, B and the error.
 *
 * @param x1 First pose
 * @param x2 Second pose
 * @param z "Measured" constrained between the poses
 * @param A Jacobian w.r.t. x1
 * @param B Jacobian w.r.t. x2
 * @param e pointer to the pose error vector
 **/
void linearize_pose_pose_constraint(float *x1, float *x2, float *z,
                                    float A[3][3], float B[3][3], float *e);

/**
 * Calculate the H matrix and the b array involved in the system H * x = -b.
 Complete demonstration found in the paper: Grisetti, Giorgio, et al. "A
 tutorial on graph-based SLAM." IEEE Intelligent Transportation Systems
 Magazine 2.4 (2010): 31-43.
 *
 * Calculates the cumulative contribution of the jacobians
 * @param x Array that contains the flattened poses of size 3 * nr_of_poses
 * @param H Hessian matrix of the linear equation system
 * @param b Column vector of the linear equation system
 * @param measurements Pointer to the odometry measurements
 * @param constraints Pointer to the loop closure constraint
 * @param nr_of_poses Number of poses
 * @param nr_of_constr Number of loop closure constraints
 **/
void build_H_and_b(float *x, csr *H, float *b, measurement *measurements,
                   constraint *constraints, int16_t nr_of_poses,
                   int16_t nr_of_constr);

/**
 * Solve the equation H * x = -b. This function solves the linearized equation
 *system H * x = -b, which is the most complex part of a graph-based SLAM
 *iteration.
 *
 * @param H Hessian matrix of the linear equation system
 * @param b Column vector of the linear equation system
 * @param perm Permutation array containing the reordering information
 * @param size Dimension of the equation system: size = 3*nr_of_poses
 **/
void solve_for_x(csr *H, float *b, int16_t *perm, int16_t size);

/**
 * Computes a measurement out of two poses
 *
 * @param x_from First pose
 * @param x_to Second pose
 * @param z Output measurement
 **/
void measurement_from_poses(float *x_from, float *x_to, float *z);

/**
 * Computes the measurement given a series of consecutive poses
 *
 * @param x Array of dimension (3*nr_of_poses)
 * @param measurements Aoutput array of measurements
 * @param nr_of_poses Number of poses
 **/
void measurements_from_poses(float *x, measurement *measurements,
                             int nr_of_poses);

/**
 * Forward substitution. This function solves an equation system of the shape L
 ** y = b, given that L is a lower triangular matrix.
 *
 * @param L Pointer to a lower triangular Cholesky matrix (L * L.T = H)
 * @param b array of dimension (N x 1).
 * @param N dimension of the equation system.
 **/
void forward_substitution(csr *L, float *b, int N);

/**
 * Backward substitution. This function solves an equation system of the shape
 *L.T * x = y, given that L is a lower triangular matrix.
 *
 * @param L Pointer to an upper triangular Cholesky matrix
 * @param b array of dimension (N x 1).
 * @param N dimension of the equation system.
 **/
void backward_substitution(csr *L, float *b, int N);

/**
 * Run least squares SLAM. This function optimizes a graph given its initial
 *poses and the loop closure constraints.
 *
 * @param x Array containing the poses
 * @param constraints Pointer to the loop closure constraints
 * @param nr_of_poses Number of poses in the graph
 * @return nr_of_constr Number of loop closure constraints
 **/
void ls_slam(float *x, constraint *constraints, int16_t nr_of_poses,
             int16_t nr_of_constr);

/**
 * Auxiliary function for computing the bandwidth of a matrix.
 *
 * @param mat Pointer to the sparse matrix
 * @param N Size of the equivalent dense matrix
 * @param perm Permutation array
 **/
int16_t bandwidth_test(csr *mat, int N, int16_t *perm);

#endif
