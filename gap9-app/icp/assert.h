#ifndef ASSERT_H
#define ASSERT_H

#include <stdlib.h>
#include <stdio.h>

#define ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            printf("Assertion failed: %s\n", message); \
            while(1) pi_time_wait_us(1000); \
        } \
    } while (0)

#endif