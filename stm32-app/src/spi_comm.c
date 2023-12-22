/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "spi_comm.h"
#include "FreeRTOS.h"
#include "debug.h"
#include "deck.h"
#include "scan.h"
#include "task.h"

#define MAX_PKT_SIZE (97)

typedef enum {
    POSE_CMD = 0,
    SCAN_CMD = 1,
    OPTIMIZE_CMD = 2,
    POSE_REQ_CMD = 3,
    CHECK_CMD = 4,
} spi_cmd_t;

static uint8_t to_send[2 * MAX_PKT_SIZE];
static uint8_t to_recv[601];

void spi_dummy_clk_cycles() {
    float spi_dummy_rev, spi_dummy_tr;
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    spiExchange(1, &spi_dummy_tr, &spi_dummy_rev);
    spiEndTransaction();
}

void spi_send_pose(full_pose_t *pose) {
    to_send[0] = 'S';
    to_send[1] = POSE_CMD;
    int16_t buf_idx = 2;
    memcpy(&to_send[buf_idx], pose, sizeof(full_pose_t));

    uint32_t checksum = 0;
    for (int16_t i = 0; i < sizeof(full_pose_t); i++) {
        checksum += *((uint8_t *)pose + i);
    }
    buf_idx += sizeof(full_pose_t);
    memcpy(&to_send[buf_idx], &checksum, sizeof(uint32_t));

    buf_idx += sizeof(uint32_t);
    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();
}

void spi_send_scan_info(int16_t from_id, int16_t to_id, icp_job_t *job) {
    memset(job, 0, sizeof(icp_job_t));
    to_send[0] = 'S';
    to_send[1] = SCAN_CMD;
    memcpy(&to_send[2], &from_id, sizeof(int16_t));
    memcpy(&to_send[4], &to_id, sizeof(int16_t));

    uint32_t checksum = 0;
    for (int16_t i = 2; i < 6; i++) {
        checksum += to_send[i];
    }
    memcpy(&to_send[6], &checksum, sizeof(uint32_t));

    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    vTaskDelay(M2T(5));
    DEBUG_PRINT("SPI sent scan info!\n");
    while (digitalRead(DECK_GPIO_IO2)) {
        vTaskDelay(M2T(2));
        DEBUG_PRINT("GAP9 Busy!\n");
    }

    vTaskDelay(M2T(3));

    // Wait feedback
    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(sizeof(float) + 2 * sizeof(int16_t) + 3 * sizeof(float),
                to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    job->error = *(float *)(to_recv + 0);
    job->time_ms = *(int16_t *)(to_recv + 4);
    job->has_nan = *(int16_t *)(to_recv + 6);
    memcpy(job->result, to_recv + 8, 3 * sizeof(float));
}

void spi_send_opt_cmd(slam_job_t *job) {
    memset(job, 0, sizeof(slam_job_t));
    to_send[0] = 'S';
    to_send[1] = OPTIMIZE_CMD;
    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    vTaskDelay(M2T(5));
    while (digitalRead(DECK_GPIO_IO2)) vTaskDelay(M2T(2));
    vTaskDelay(M2T(3));

    // Wait feedback
    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(sizeof(slam_job_t), to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    memcpy(job, to_recv, sizeof(slam_job_t));
}

void spi_send_pose_req(int16_t from_id, int16_t nr_of_poses) {
    to_send[0] = 'S';
    to_send[1] = POSE_REQ_CMD;
    memcpy(&to_send[2], &from_id, sizeof(int16_t));
    memcpy(&to_send[4], &nr_of_poses, sizeof(int16_t));

    uint32_t checksum = 0;
    for (int16_t i = 2; i < 6; i++) {
        checksum += to_send[i];
    }
    memcpy(&to_send[6], &checksum, sizeof(uint32_t));

    to_send[MAX_PKT_SIZE - 1] = 'E';

    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(MAX_PKT_SIZE, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();
}

void spi_rcv_poses(float poses[][3], int16_t from_id, int16_t nr_of_poses) {
    spi_send_pose_req(from_id, nr_of_poses);

    vTaskDelay(M2T(2));
    spi_dummy_clk_cycles();
    spiBeginTransaction(SPI_BAUDRATE_2MHZ);
    digitalWrite(DECK_GPIO_IO1, 0);
    spiExchange(3 * 4 * nr_of_poses, to_send, to_recv);
    digitalWrite(DECK_GPIO_IO1, 1);
    spiEndTransaction();

    for (int16_t i = 0; i < nr_of_poses; i++) {
        poses[i][0] = *(float *)(to_recv + 12 * i + 0);
        poses[i][1] = *(float *)(to_recv + 12 * i + 4);
        poses[i][2] = *(float *)(to_recv + 12 * i + 8);
    }
}
