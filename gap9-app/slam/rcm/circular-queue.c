/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#include "circular-queue.h"
#include "pmsis.h"
#include <stdio.h>
#include <string.h>

int16_t init_queue(circular_queue_t *queue) {
    memset(queue, 0, sizeof(circular_queue_t));
    return 1;
}

int16_t is_full(circular_queue_t *queue) {
    if (queue->size == MAX_SIZE) {
        return 1;
    } else {
        return 0;
    }
}

int16_t is_empty(circular_queue_t *queue) {
    if (queue->size == 0) {
        return 1;
    } else {
        return 0;
    }
}

int16_t push_to_queue(circular_queue_t *queue, node element) {
    if (is_full(queue)) return 0;

    queue->nodes[queue->tail] = element;
    queue->tail = (queue->tail + 1) % MAX_SIZE;
    queue->size++;

    return 1;
}

int16_t pop_from_queue(circular_queue_t *queue, node *element) {
    if (is_empty(queue)) return 0;

    *element = queue->nodes[queue->head];
    queue->head = (queue->head + 1) % MAX_SIZE;
    queue->size--;

    return 1;
}
