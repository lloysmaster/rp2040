#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

typedef int32_t q16_16;

#define TO_Q16(x) ((q16_16)((int32_t)(x) * 65536))

static inline q16_16 q_div(q16_16 a, q16_16 b) {
    return (q16_16)(((int64_t)a * 65536) / b);
}

#endif // FIXED_POINT_H
