typedef int32_t q16_16;

#define TO_Q16(x) ((q16_16)((x) << 16))
#define FROM_Q16(x) ((x) >> 16)


static inline q16_16 q_mul(q16_16 a, q16_16 b) {
    // Usar int64_t es obligatorio, el compilador lo optimiza bien en M0+
    return (q16_16)(( (int64_t)a * (int64_t)b ) >> 16);
}

static inline q16_16 q_div(q16_16 a, q16_16 b) {
    return (q16_16)(((int64_t)a << 16) / b);
}