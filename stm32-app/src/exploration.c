/*
 * Authors: The code is authored by Carl Friess in the work
 * Friess, Carl, et al. "Fully Onboard SLAM for Distributed Mapping with a Swarm
 * of Nano-Drones." arXiv preprint arXiv:2309.03678 (2023).
 * https://arxiv.org/pdf/2309.03678.pdf
 */

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "debug.h"
#include "estimator_kalman.h"
#include "exploration.h"
#include "log.h"
#include "scan.h"
#include "task.h"
#include "tof_matrix.h"

static float get_dist(int16_t tof_raw[8][8]) {

    // Select center pixels
    size_t num = 0;
    int16_t d[4];
    for (int i = 3; i <= 4; ++i) {
        for (int j = 3; j <= 4; ++j) {
            if (tof_raw[i][j] >= 0) {
                d[num++] = tof_raw[i][j];
            }
        }
    }
    if (num == 0) {
        return INFINITY;
    }

    // Find median distance
    qsort(d, num, sizeof(int16_t), compare_int16);
    float median;
    if (num & 1) {
        median = d[num >> 1];
    } else {
        median = (float)(d[num >> 1] + d[(num >> 1) - 1]);
        median *= 0.5f;
    }

    return median / 1000.0f;
}

static void pri_vel(float d, float target, float *out) {
    if (d <= EXP_SLOW_DOWN_DIST) {
        target = (d / EXP_SLOW_DOWN_DIST + 0.1f) * target;
    }
    float limit_p = *out + EXP_ACCELERATION * 0.067f;
    float limit_n = *out - EXP_ACCELERATION * 0.067f;
    *out = MAX(limit_n, MIN(limit_p, target));
}

static float sec_vel(float d1, float d2) {

    // Stabilise horizontally (in the center or fixed distance from the wall)
    float width = d1 + d2;
    if (width < 3 * EXP_WALL_DIST) {
        return (d1 - 0.5f * width) * EXP_LAT_VELOCITY;
    } else if (d2 < 2 * EXP_WALL_DIST) {
        return (EXP_WALL_DIST - d2) * EXP_LAT_VELOCITY;
    } else if (d1 < 2 * EXP_WALL_DIST) {
        return (d1 - EXP_WALL_DIST) * EXP_LAT_VELOCITY;
    } else {
        return 0;
    }
}

static direction_t dir_priority[4] = {
    DIR_LEFT,
    DIR_BACK,
    DIR_RIGHT,
    DIR_FRONT,
};
static direction_t cur_dir = DIR_FRONT;
static setpoint_t setpoint = {
    .mode =
        {
            .x = modeVelocity,
            .y = modeVelocity,
            .z = modeAbs,
            .yaw = modeAbs,
        },
    .position =
        {
            .x = 0,
            .y = 0,
            .z = EXP_HEIGHT,
        },
    .attitude =
        {
            .yaw = 0,
        },
};

void explore_iter(int16_t tof_data_buf[4][8][8], float velocity) {
    // Fetch distances from ToF sensors
    float dists[4];
    for (int i = 0; i < 4; i++) {
        dists[i] = get_dist(tof_data_buf[i]);
    }
    dists[DIR_FRONT] += 0.02f;
    dists[DIR_BACK] += 0.02f;
    dists[DIR_LEFT] += 0.025f;
    dists[DIR_RIGHT] += 0.025f;

    // Calculate possible distances to next pose
    float dist_to_corner = dists[cur_dir] - EXP_WALL_DIST;

    // Check if we are getting too close to a corner
    if (dist_to_corner <= 0) {

        // Check for dead end
        switch (cur_dir) {
        case DIR_FRONT:
        case DIR_BACK:
            if (dists[DIR_LEFT] < 2 * EXP_WALL_DIST &&
                dists[DIR_RIGHT] < 2 * EXP_WALL_DIST) {
                return;
            }
            break;
        case DIR_RIGHT:
        case DIR_LEFT:
            if (dists[DIR_FRONT] < 2 * EXP_WALL_DIST &&
                dists[DIR_BACK] < 2 * EXP_WALL_DIST) {
                return;
            }
            break;
        }

        // Find a new direction to fly in
        for (int i = 0; i < 4; ++i) {
            direction_t next_dir = dir_priority[i];
            switch (next_dir) {
            case DIR_FRONT:
            case DIR_BACK:
                if (cur_dir == DIR_FRONT || cur_dir == DIR_BACK) {
                    continue;
                }
                break;
            case DIR_LEFT:
            case DIR_RIGHT:
                if (cur_dir == DIR_LEFT || cur_dir == DIR_RIGHT) {
                    continue;
                }
                break;
            }
            if (dists[next_dir] >= 2 * EXP_WALL_DIST) {
                cur_dir = next_dir;
                break;
            }
        }

        // Reset velocity
        setpoint.velocity.x = setpoint.velocity.y = 0;
    }

    // Calculate the next setpoint
    switch (cur_dir) {
    case DIR_FRONT:
        pri_vel(dist_to_corner, velocity, &setpoint.velocity.x);
        setpoint.velocity.y = sec_vel(dists[DIR_LEFT], dists[DIR_RIGHT]);
        break;
    case DIR_BACK:
        pri_vel(dist_to_corner, -velocity, &setpoint.velocity.x);
        setpoint.velocity.y = sec_vel(dists[DIR_LEFT], dists[DIR_RIGHT]);
        break;
    case DIR_LEFT:
        pri_vel(dist_to_corner, velocity, &setpoint.velocity.y);
        setpoint.velocity.x = sec_vel(dists[DIR_FRONT], dists[DIR_BACK]);
        break;
    case DIR_RIGHT:
        pri_vel(dist_to_corner, -velocity, &setpoint.velocity.y);
        setpoint.velocity.x = sec_vel(dists[DIR_FRONT], dists[DIR_BACK]);
        break;
    }
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
}

void stop_drone() {
    setpoint.velocity.y = 0;

    // Stop the drone
    setpoint.velocity.x = setpoint.velocity.y = 0;
    commanderSetSetpoint(&setpoint, COMMANDER_PRIORITY_EXTRX);
    vTaskDelay(M2T(1000));
}