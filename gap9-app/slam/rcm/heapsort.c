#include "heapsort.h"
#include "node.h"
#include <stdio.h>
#include <stdlib.h>

static void swap(node *a, node *b) {
    node temp = *a;
    *a = *b;
    *b = temp;
}

static void heapify(node *nodes, int16_t N, int16_t i) {
    int16_t largest = i;
    int16_t left = 2 * i + 1;
    int16_t right = 2 * i + 2;

    if (left < N && nodes[left].neighbors > nodes[largest].neighbors)
        largest = left;

    if (right < N && nodes[right].neighbors > nodes[largest].neighbors)
        largest = right;

    if (largest != i) {
        swap(&nodes[i], &nodes[largest]);
        heapify(nodes, N, largest);
    }
}

void heapsort_nodes(node *nodes, int16_t N) {
    for (int16_t i = N / 2 - 1; i >= 0; i--) heapify(nodes, N, i);

    for (int16_t i = N - 1; i >= 0; i--) {
        swap(&nodes[0], &nodes[i]);
        heapify(nodes, i, 0);
    }
}
