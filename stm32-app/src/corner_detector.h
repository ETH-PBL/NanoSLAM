/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef CORNER_DET_H
#define CORNER_DET_H

uint8_t corner_detected_hough(scan_frame_t *scan_frame);
uint8_t corner_detected_eigen(scan_frame_t *scan_frame);

#endif