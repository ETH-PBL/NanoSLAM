/*
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Vlad Niculescu
 */

#ifndef CIRCULAR_QUEUE_H
#define CIRCULAR_QUEUE_H

#include "config_size.h"
#include "node.h"

typedef struct {
    node nodes[MAX_SIZE];
    int16_t head;
    int16_t tail;
    int16_t size;
} circular_queue_t;

/**
 * Empties a given queue.
 *
 * @param queue Pointer to the queue
 * @return 1 if the operation was succesful
 **/
int16_t init_queue(circular_queue_t *queue);

/**
 * Evaluates if a queue is full.
 *
 * @param queue Pointer to the queue
 * @return 1 if full, 0 otherwise
 **/
int16_t is_full(circular_queue_t *queue);

/**
 * Evaluates if a queue is empty.
 *
 * @param queue Pointer to the queue
 * @return 1 if empty, 0 otherwise
 **/
int16_t is_empty(circular_queue_t *queue);

/**
 * Adds an element to the queue.
 *
 * @param queue Pointer to the queue
 * @param element The node to be added
 * @return 1 if operation was succesful
 **/
int16_t push_to_queue(circular_queue_t *queue, node element);

/**
 * Pops an element from the queue.
 *
 * @param queue Pointer to the queue
 * @param element Pointer to the node that will store the popped element
 * @return 1 if operation was succesful
 **/
int16_t pop_from_queue(circular_queue_t *queue, node *element);

#endif