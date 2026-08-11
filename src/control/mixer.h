#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include "core/attitude.h"

// Sentido de giro de las hélices: 1 si la diagonal M1 (atrás-derecha) + M4
// (adelante-izquierda) gira en horario vista desde arriba, 0 si la horaria es
// la diagonal M2 + M3. Define el signo del yaw en el mixer.
#define MIXER_YAW_CW_DIAGONAL_M1_M4 1

typedef struct {
    int32_t motor[4];
} mixer_output_t;

void mixer_init(void);
void mixer_mix(const attitude_cmd_t *attitude, mixer_output_t *output);

#endif
