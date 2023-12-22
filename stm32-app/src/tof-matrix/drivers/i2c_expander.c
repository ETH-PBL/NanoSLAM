/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "i2c_expander.h"

bool I2C_expander_set_pin(uint8_t pin_number, uint8_t pin_value) {
    bool status = true;
    uint8_t reg_value;

    // Read current value of the output register
    status = status && i2cdevReadByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS,
                                      OUTPUT_REG_ADDRESS, &reg_value);

    if (pin_value == 1)
        reg_value |= (1 << pin_number);
    else
        reg_value &= ~(1 << pin_number);

    // Update output register value
    status = status && i2cdevWriteByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS,
                                       OUTPUT_REG_ADDRESS, reg_value);

    return status;
}

bool I2C_expander_initialize() {
    bool status = true;

    status = status && i2cdevWriteByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS,
                                       CONFIG_REG_ADDRESS, 0x00);
    status = status && i2cdevWriteByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS,
                                       POL_INV_REG_ADDRESS, 0x00);
    status = status && i2cdevWriteByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS,
                                       OUTPUT_REG_ADDRESS, 0x01);

    return status;
}