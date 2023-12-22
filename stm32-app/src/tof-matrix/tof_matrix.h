/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef TOF_MATRIX_H
#define TOF_MATRIX_H

#include "vl53l5cx_api.h"

/**
 * Possible return codes returned by the ToF functions
 */
typedef enum TOF_err_code {
    TOF_SUCCESS = 0,     /**< Operation succeeded.*/
    TOF_NOT_READY = 1,   /**< ToF Data is not ready. Wait more.*/
    TOF_DATA_ERROR = 2,  /**< Sensor error: value, I2C access, etc.).*/
    TOF_CONFIG_ERROR = 3 /**< Sensor configuration error: I2C access, etc.).*/
} tof_err_code_e;

tof_err_code_e tof_matrix_init();
tof_err_code_e config_sensors(VL53L5CX_Configuration *p_dev,
                              uint16_t new_i2c_address);
tof_err_code_e get_tof_matrix(uint8_t sensor_id, int16_t matrix[][8]);
tof_err_code_e get_tof_row(uint8_t sensor_id, uint8_t rowIdx,
                           int16_t *row); // row 0 is the top one

#endif