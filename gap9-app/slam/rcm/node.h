/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef NODE_H
#define NODE_H

#include <stdlib.h>

typedef struct {
    int16_t index;
    int16_t neighbors;
    int8_t visited;
} node;

#endif