/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#define SPIN_CMD (0)
#define STOP_CMD (1)
#define STOP_SPIN_CMD (2)
#define STOP_GOTO_CMD (3)
#define EXIT_CMD (4)

typedef enum cmd_err_code {
    CMD_SUCCESS = 1,
    CMD_TO = -1,
    CMD_ERR = -2,
} cmd_err_code_e;

void fly_task(void *parameters);
cmd_err_code_e pop_from_command_queue(uint8_t *cmd, TickType_t timeout_ms);
cmd_err_code_e push_to_command_queue(uint8_t cmd);
void spin_drone();
void go_to(float x, float y);
uint8_t is_spinning();

#endif
