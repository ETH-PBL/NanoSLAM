/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "trajectory.h"
#include "FreeRTOS.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "debug.h"
#include "estimator_kalman.h"
#include "exploration.h"
#include "log.h"
#include "param.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include <string.h>

extern SemaphoreHandle_t tofSemaphore1;
extern QueueHandle_t commandQueue;
extern int16_t tof_data_buf[4][8][8];

#define SCAN_NUM_FRAMES 15
#define SCAN_TIME_MS 2000

static float go_to_x;
static float go_to_y;
void fly_task(void *parameters) {
    uint8_t cmd_value;
    // TickType_t prevWakeTime = xTaskGetTickCount();
    while (1) {
        cmd_err_code_e res =
            pop_from_command_queue(&cmd_value, M2T(70)); // TODO maybe time 0
        if (res == CMD_SUCCESS && cmd_value == EXIT_CMD) {
            stop_drone();
            while (1) vTaskDelay(10);
        }

        if (res == CMD_SUCCESS && cmd_value == SPIN_CMD) {
            spin_drone();
        }

        if (res == CMD_SUCCESS && cmd_value == STOP_CMD) {
            stop_drone();
        }

        if (res == CMD_SUCCESS && cmd_value == STOP_SPIN_CMD) {
            stop_drone();
            spin_drone();
        }

        if (res == CMD_SUCCESS && cmd_value == STOP_GOTO_CMD) {
            stop_drone();

            float z = logGetFloat(logGetVarId("stateEstimate", "z"));
            float yaw = logGetFloat(logGetVarId("stateEstimate", "yaw")) /
                        180.0f * (float)M_PI;
            crtpCommanderHighLevelGoTo(go_to_x, go_to_y, z, yaw, 2.0f, false);
            vTaskDelay(M2T(2000));
        }

        if (xSemaphoreTake(tofSemaphore1, portMAX_DELAY)) {
            explore_iter(tof_data_buf, 0.4f);
        }
        // vTaskDelayUntil(&prevWakeTime, M2T(70));
    }
}

cmd_err_code_e pop_from_command_queue(uint8_t *cmd, TickType_t timeout_ms) {
    if (commandQueue != NULL) {
        if (xQueueReceive(commandQueue, cmd, (TickType_t)timeout_ms) ==
            pdTRUE) {
            return CMD_SUCCESS;
        } else
            return CMD_TO;
    } else
        return CMD_ERR;
}

cmd_err_code_e push_to_command_queue(uint8_t cmd) {
    UBaseType_t available_space;
    if (commandQueue != NULL) {
        available_space = uxQueueSpacesAvailable(commandQueue);
        if (available_space == 0) xQueueReset(commandQueue);

        if (xQueueSendToBack(commandQueue, (void *)(&cmd), (TickType_t)0) ==
            pdTRUE) {
            return CMD_SUCCESS;
        }
    }
    return CMD_ERR;
}

static uint8_t spinning_in_progress = 0;

void spin_drone() {
    spinning_in_progress = 1;
    DEBUG_PRINT("%d: Scan spinning\n", xTaskGetTickCount());
    // Get current pose and rotate the drone in place
    float x = logGetFloat(logGetVarId("stateEstimate", "x"));
    float y = logGetFloat(logGetVarId("stateEstimate", "y"));
    float z = logGetFloat(logGetVarId("stateEstimate", "z"));
    float yaw =
        logGetFloat(logGetVarId("stateEstimate", "yaw")) / 180.0f * (float)M_PI;

    crtpCommanderHighLevelGoTo(x, y, z, yaw + M_PI_4, SCAN_TIME_MS / 1000.0f,
                               false);

    // Continue rotating
    vTaskDelay(M2T(SCAN_TIME_MS));
    DEBUG_PRINT("%d: Scan spinning completed\n", xTaskGetTickCount());

    // Reverse rotation
    crtpCommanderHighLevelGoTo(x, y, z, yaw, SCAN_TIME_MS / 1000.0f, false);
    vTaskDelay(M2T(SCAN_TIME_MS));
    spinning_in_progress = 0;
    vTaskDelay(100);
}

uint8_t is_spinning() { return spinning_in_progress; }

void go_to(float x, float y) {
    go_to_x = x;
    go_to_y = y;
    push_to_command_queue(STOP_GOTO_CMD);
    vTaskDelay(M2T(3000));
}
