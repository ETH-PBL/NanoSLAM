/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef DMA_TRANSFERS_H
#define DMA_TRANSFERS_H

#include "pmsis.h"

/**
 * Transfers a number of bytes from the L2 FC to L1 CL.
 *
 * @param l1_dest Destination address in L1
 * @param l2_src Source address in L2
 * @param size Number of bytes to copy
 **/
void cl_memcpy_from_l2(void *l1_dest, void *l2_src, int32_t size);

/**
 * Transfers a number of bytes from the L1 CL to L2 FC.
 *
 * @param l2_dest Destination address in L2
 * @param l1_src Source address in L1
 * @param size Number of bytes to copy
 **/
void cl_memcpy_to_l2(void *l2_dest, void *l1_src, int32_t size);

#endif