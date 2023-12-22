#ifndef HEAP_SORT_H
#define HEAP_SORT_H

#include "node.h"

/**
 * Sorts a list of nodes by the number of neighbors.
 *
 * @param queue Pointer to the first node
 * @param N Number of nodes
 **/
void heapsort_nodes(node *nodes, int16_t N);

#endif