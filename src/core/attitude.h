#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>
#include "math/fixed_point.h"
#include "hal/rx/crossfire.h"
#include "core/flighMode.h"

typedef struct {
    int32_t roll_output;
    int32_t pitch_output;
    int32_t yaw_output;
    int32_t throttle;
    bool enabled;
} attitude_cmd_t;

void attitude_init(void);
void attitude_set_mode(flight_mode_t mode);
void attitude_update(const crsf_data_t *rc_data, const q16_16 gyro[3], attitude_cmd_t *output);

#endif
