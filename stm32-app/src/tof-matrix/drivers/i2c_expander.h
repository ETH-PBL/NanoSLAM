/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef I2C_EXPANDER
#define I2C_EXPANDER

#include "i2cdev.h"

// I2C ADDRESS
#define I2C_EXPANDER_I2C_ADDRESS ((uint8_t)0x20)

// Registers
#define INPUT_REG_ADDRESS ((uint8_t)0x00)
#define OUTPUT_REG_ADDRESS ((uint8_t)0x01)
#define POL_INV_REG_ADDRESS ((uint8_t)0x02)
#define CONFIG_REG_ADDRESS ((uint8_t)0x03)

// Pins
#define S0_EN ((uint8_t)0)
#define S1_EN ((uint8_t)1)
#define S2_EN ((uint8_t)2)
#define S3_EN ((uint8_t)3)
#define LED0 ((uint8_t)4)
#define LED1 ((uint8_t)5)

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)

bool I2C_expander_set_pin(uint8_t pin_number, uint8_t pin_value);
bool I2C_expander_initialize();

#endif