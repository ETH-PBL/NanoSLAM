/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

/**
 * @brief Initializes the SPI slave communication.
 *
 */
void spi_slave_driver_init();

/**
 * @brief Blocks until "len" bytes are received .
 *
 * @param data Pointer to where to store the received bytes
 * @param len Number of bytes to receive
 */
void spi_rcv_blocking(void *data, size_t len);

/**
 * @brief Blocks until "len" bytes are received .
 *
 * @param data Pointer to where to store the received bytes
 * @param len Number of bytes to receive
 */
void spi_send_blocking(void *data, size_t len);

#endif