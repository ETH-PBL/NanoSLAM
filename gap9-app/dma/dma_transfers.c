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

void cl_memcpy_from_l2(void *l1_dest, void *l2_src, int32_t size) {
    pi_cl_dma_copy_t copy;
    copy.dir = PI_CL_DMA_DIR_EXT2LOC;
    copy.merge = 0;
    copy.size = size;
    copy.id = 0;
    copy.ext = (uint32_t)l2_src;
    copy.loc = (uint32_t)l1_dest;

    pi_cl_dma_memcpy(&copy);
    pi_cl_dma_wait(&copy);
}

void cl_memcpy_to_l2(void *l2_dest, void *l1_src, int32_t size) {
    pi_cl_dma_copy_t copy;
    copy.dir = PI_CL_DMA_DIR_LOC2EXT;
    copy.merge = 0;
    copy.size = size;
    copy.id = 0;
    copy.ext = (uint32_t)l2_dest;
    copy.loc = (uint32_t)l1_src;

    pi_cl_dma_memcpy(&copy);
    pi_cl_dma_wait(&copy);
}