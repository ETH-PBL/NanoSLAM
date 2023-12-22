#include "pmsis.h"

static void removeDuplicates(int16_t arr[], int16_t n, int16_t *n_out) {
    if (n == 0 || n == 1) {
        *n_out = n;
        return;
    }

    int16_t temp[n];
    int16_t j = 0;

    for (int16_t i = 0; i < n - 1; i++)
        if (arr[i] != arr[i + 1]) temp[j++] = arr[i];

    temp[j++] = arr[n - 1];

    for (int16_t i = 0; i < j; i++) arr[i] = temp[i];

    *n_out = j;
}

static void swap(int16_t *xp, int16_t *yp) {
    int16_t temp = *xp;
    *xp = *yp;
    *yp = temp;
}

static void bubbleSort(int16_t arr[], int16_t n) {
    int16_t i, j;
    for (i = 0; i < n - 1; i++)

        // Last i elements are already in place
        for (j = 0; j < n - i - 1; j++)
            if (arr[j] > arr[j + 1]) swap(&arr[j], &arr[j + 1]);
}

void delete_duplicates_and_sort(int16_t *array, int16_t size,
                                int16_t *new_size) {
    bubbleSort(array, size);
    removeDuplicates(array, size, new_size);
}
