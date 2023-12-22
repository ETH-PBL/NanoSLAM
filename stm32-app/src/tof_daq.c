/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "FreeRTOS.h"
#include "debug.h"
#include "pm.h"
#include "semphr.h"
#include "tof_matrix.h"

extern SemaphoreHandle_t tofSemaphore0;
extern SemaphoreHandle_t tofSemaphore1;
int16_t tof_data_buf[4][8][8];
static uint8_t tof_init = 0;

void tof_task(void *parameters) {
    tof_matrix_init();
    tof_init = 1;
    // vTaskDelay(100);
    TickType_t prevWakeTime = xTaskGetTickCount();
    while (1) {
        int16_t matrix[8][8];
        if (pmIsBatteryLow()) DEBUG_PRINT("Battery Low!\n");

        for (int16_t i = 0; i < 4; i++) {
            tof_err_code_e e = get_tof_matrix(i, matrix);
            int16_t err_cnt = 0;
            while (e != TOF_SUCCESS) {
                err_cnt++;
                // DEBUG_PRINT("TOF Read Failed! [%d] with %d\n", i, e);
                vTaskDelay(5);
                e = get_tof_matrix(i, matrix);
                if (err_cnt == 10) {
                    xSemaphoreTake(tofSemaphore0, 0);
                    xSemaphoreTake(tofSemaphore1, 0);
                }
            }
            memcpy(&tof_data_buf[i], matrix, 8 * 8 * sizeof(int16_t));
            vTaskDelay(1);
        }

        xSemaphoreGive(tofSemaphore0);
        xSemaphoreGive(tofSemaphore1);

        vTaskDelayUntil(&prevWakeTime, M2T(68));
    }
}

uint8_t tof_is_init() { return tof_init; }
