/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef TOFDAQ_H
#define TOFDAQ_H

void tof_task(void *parameters);
uint8_t tof_is_init();
extern int16_t tof_data_buf[4][8][8];

#endif