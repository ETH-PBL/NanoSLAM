/*
 * Authors: The code is authored by Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#ifndef EXPLORATION_H
#define EXPLORATION_H

#include "FreeRTOS.h"
#include <stddef.h>

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))

#define EXP_HEIGHT 0.6f            // m
#define EXP_ACCELERATION 0.25f     // m/s/s
#define EXP_LAT_VELOCITY 0.5f      // m/s/m
#define EXP_WALL_DIST 0.5f         // m
#define EXP_SLOW_DOWN_DIST 0.75f   // m
#define EXP_WAYPOINT_INTERVAL 1.0f // m
#define EXP_WAYPOINT_TOL 0.75f     // m

void explore_iter(int16_t tof_data_buf[4][8][8], float velocity);
void stop_drone();

#endif // EXPLORATION_H
