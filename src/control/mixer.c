#include "mixer.h"

void mixer_init(void) {
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

    int32_t throttle = attitude->throttle;
    int32_t roll = attitude->roll_output;
    int32_t pitch = attitude->pitch_output;
    int32_t yaw = attitude->yaw_output;

    // If throttle is zero, force all motor outputs to zero to avoid
    // sending tiny PID-induced commands while disarmed/min throttle.
    if (throttle == 0) {
        for (int i = 0; i < 4; ++i) {
            output->motor[i] = 0;
        }
        return;
    }

    // Distribución para el quad en X soldado como:
    //   motor[0]=M1 atrás-derecha   motor[1]=M2 adelante-derecha
    //   motor[2]=M3 atrás-izquierda motor[3]=M4 adelante-izquierda
    // roll+ sube el lado derecho, pitch+ sube los traseros (morro arriba) y
    // yaw+ acelera la diagonal que gira en horario, igual que los signos del
    // giroscopio que consume el PID.
    const int32_t yaw_a = MIXER_YAW_CW_DIAGONAL_M1_M4 ? yaw : -yaw;

    output->motor[0] = throttle + roll + pitch + yaw_a;
    output->motor[1] = throttle + roll - pitch - yaw_a;
    output->motor[2] = throttle - roll + pitch - yaw_a;
    output->motor[3] = throttle - roll - pitch + yaw_a;

    for (int i = 0; i < 4; ++i) {
        if (output->motor[i] < 0) {
            output->motor[i] = 0;
        } else if (output->motor[i] > 1000) {
            output->motor[i] = 1000;
        }
    }
}
