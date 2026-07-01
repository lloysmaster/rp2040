#ifndef MIXER_H
#define MIXER_H

#include <stdint.h>
#include "core/attitude.h"

typedef struct {
    int32_t motor[4];
} mixer_output_t;

void mixer_init(void);
void mixer_mix(const attitude_cmd_t *attitude, mixer_output_t *output);

#endif
