/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "pmsis.h"
#include "spi_config.h"

static pi_device_t spi_slave;
static struct pi_spi_conf spi_slave_conf;
static pi_task_t send_task, receive_task, end_task;

static void spi_slave_pad_init() {
    pi_pad_set_function(SPI_SLAVE_PAD_SCK, SPI_SLAVE_PAD_FUNC);
    pi_pad_set_function(SPI_SLAVE_PAD_CS0, SPI_SLAVE_PAD_FUNC);
    pi_pad_set_function(SPI_SLAVE_PAD_SDO, SPI_SLAVE_PAD_FUNC);
    pi_pad_set_function(SPI_SLAVE_PAD_SDI, SPI_SLAVE_PAD_FUNC);
}

static void spi_slave_init(pi_device_t *spi_slave,
                           struct pi_spi_conf *spi_slave_conf) {
    pi_assert(spi_slave);
    pi_assert(spi_slave_conf);

    pi_spi_conf_init(spi_slave_conf);
    spi_slave_conf->wordsize = SPI_SLAVE_WORDSIZE;
    spi_slave_conf->big_endian = SPI_SLAVE_ENDIANESS;
    spi_slave_conf->max_baudrate = SPI_SLAVE_BAUDRATE;
    spi_slave_conf->polarity = SPI_SLAVE_POLARITY;
    spi_slave_conf->phase = SPI_SLAVE_PHASE;
    spi_slave_conf->itf = SPI_SLAVE_ITF;
    spi_slave_conf->cs = SPI_SLAVE_CS;
    spi_slave_conf->dummy_clk_cycle = SPI_MASTER_DUMMY_CYCLE;
    spi_slave_conf->dummy_clk_cycle_mode = SPI_MASTER_DUMMY_CYCLE_MODE;
    spi_slave_conf->is_slave = SPI_SLAVE_IS_SLAVE;
    pi_open_from_conf(spi_slave, spi_slave_conf);
}

void spi_slave_driver_init() {
    spi_slave_pad_init();
    spi_slave_init(&spi_slave, &spi_slave_conf);
    if (pi_spi_open(&spi_slave)) {
        printf("ERROR: Failed to open SPI peripheral\n");
    }
}

void spi_send_blocking(void *data, size_t len) {
    pi_spi_send(&spi_slave, data, 8 * len, SPI_NO_OPTION);
}

void spi_rcv_blocking(void *data, size_t len) {
    pi_spi_receive(&spi_slave, data, 8 * len, SPI_NO_OPTION);
}