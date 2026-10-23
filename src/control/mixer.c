#include "mixer.h"
#include "hal/esc/dshot.h"

void mixer_init(void) {
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void mixer_mix(const attitude_cmd_t *attitude, mixer_output_t *output) {
    if (attitude == 0 || output == 0) {
        return;
    }

    if (!attitude->enabled) {
        for (int i = 0; i < 4; ++i) {
            output->motor[i] = 0;
        }
        return;
    }

    const int32_t min_thr = (int32_t)DSHOT_MIN_THROTTLE;
    const int32_t max_thr = 2047;
    const int32_t avail   = max_thr - min_thr;

    int32_t throttle = attitude->throttle;
    int32_t roll     = attitude->roll_output;
    int32_t pitch    = attitude->pitch_output;
    int32_t yaw      = attitude->yaw_output;

    // Motores: [0]=M1 atrás-der, [1]=M2 adelante-der, [2]=M3 atrás-izq, [3]=M4 adelante-izq
    // --- 1) Roll + pitch (máxima prioridad) ---
    int32_t rp[4] = {
         roll + pitch,
         roll - pitch,
        -roll + pitch,
        -roll - pitch
    };

    int32_t rp_max = rp[0], rp_min = rp[0];
    for (int i = 1; i < 4; ++i) {
        if (rp[i] > rp_max) rp_max = rp[i];
        if (rp[i] < rp_min) rp_min = rp[i];
    }
    int32_t rp_range = rp_max - rp_min;

    // Si roll/pitch solos ya no caben, se escalan y yaw queda en 0
    if (rp_range > avail) {
        for (int i = 0; i < 4; ++i) {
            rp[i] = (rp[i] * avail) / rp_range;
        }
        rp_range = avail;
    }

    // --- 2) Yaw (prioridad baja): solo usa el margen que sobra ---
    // Cada motor suma ±yaw, así que el rango total crece como máximo 2*|yaw|.
    int32_t yaw_limit = (avail - rp_range) / 2;
    yaw = clamp_i32(yaw, -yaw_limit, yaw_limit);

    const int32_t yaw_a = MIXER_YAW_CW_DIAGONAL_M1_M4 ? yaw : -yaw;

    output->motor[0] = throttle + rp[0] + yaw_a;
    output->motor[1] = throttle + rp[1] - yaw_a;
    output->motor[2] = throttle + rp[2] - yaw_a;
    output->motor[3] = throttle + rp[3] + yaw_a;

    // --- 3) Desplazar todo para que entre en [min_thr, max_thr] ---
    // (el rango ya es <= avail, así que el shift nunca deforma roll/pitch)
    int32_t max_val = output->motor[0];
    int32_t min_val = output->motor[0];
    for (int i = 1; i < 4; ++i) {
        if (output->motor[i] > max_val) max_val = output->motor[i];
        if (output->motor[i] < min_val) min_val = output->motor[i];
    }

    int32_t offset = 0;
    if (max_val > max_thr) {
        offset = max_val - max_thr;
    } else if (min_val < min_thr) {
        offset = min_val - min_thr;
    }

    for (int i = 0; i < 4; ++i) {
        output->motor[i] = clamp_i32(output->motor[i] - offset, min_thr, max_thr);
    }
}