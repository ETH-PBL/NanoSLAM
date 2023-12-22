/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SPI_CMDS_H
#define SPI_CMDS_H

#include "pmsis.h"
#include "point_cloud.h"

#define SPIBUF_SIZE (200)
#define SPI_PKT_SIZE (97)

typedef enum {
    POSE_CMD = 0,
    SCAN_CMD = 1,
    OPTIMIZE_CMD = 2,
    POSE_REQ_CMD = 3,
    CHECK_CMD = 4,
} spi_cmd_t;

typedef enum spi_transfer {
    SPI_OK = 0,
    SPI_WRONG_CMD = -1,
    SPI_ERR = -2,
    SPI_CHECKSUM_ERR = -3
} spi_transfer_t;

/**
 * @brief Transforms an SPI packet into the corresponding augmented pose.
 *
 * @param buffer The SPI packet as array
 * @param pose Pointer to where to store the pose
 * @return Status of the operation
 */
spi_transfer_t spi_decode_pose(uint8_t *buffer, pcl_pose_t *pose);

/**
 * @brief Transforms an SPI packet into the corresponding scan information.
 *
 * @param buffer The SPI packet as array
 * @param scan_info Two-entries array containing the ids of the two poses for
 * which a loop closure constraint should be derived
 * @return Status of the operation
 */
spi_transfer_t spi_decode_scan_info(uint8_t *buffer, int16_t *scan_info);

/**
 * @brief Requests a batch of poses (x, y, yaw) from the STM32.
 *
 * @param buffer The SPI packet as array
 * @param pose_request Pose request information: how many poses to fetch and
 * from which id
 * @return Status of the operation
 */
spi_transfer_t spi_decode_pose_req(uint8_t *buffer, int16_t *pose_request);

#endif