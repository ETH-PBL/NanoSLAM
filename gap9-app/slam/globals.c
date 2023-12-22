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
#include "config_size.h"
#include "sparse-matrix.h"

PI_L2 csr sp_mat0;
PI_L2 csr sp_mat1;
PI_CL_L1 csr L_L1;
PI_CL_L1 csr H_L1;