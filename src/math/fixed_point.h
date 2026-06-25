#ifndef FIXED_POINT_H
#define FIXED_POINT_H

#include <stdint.h>

typedef int32_t q16_16;

#define TO_Q16(x)   ((q16_16)((x) << 16))
#define FROM_Q16(x) ((x) >> 16)

/*
  @brief Convierte un valor entero raw a formato Q16_16 aplicando un factor de escala.
  @param raw El valor crudo (ej: lectura de sensor int16_t).
  @param scale El factor de escala (ej: sensibilidad del sensor).
  @return q16_16 El valor convertido en punto fijo.
 */

static inline q16_16 raw_to_q16(int16_t raw, int32_t scale) {
    return (q16_16)(raw * scale) << 16;
}

static inline q16_16 q_mul(q16_16 a, q16_16 b) {
    return (q16_16)(((int64_t)a * (int64_t)b) >> 16);
}

static inline q16_16 q_div(q16_16 a, q16_16 b) {
    return (q16_16)(((int64_t)a << 16) / b);
}

#endif // FIXED_POINT_H