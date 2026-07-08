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

    output->motor[0] = throttle + roll - pitch + yaw;
    output->motor[1] = throttle - roll - pitch - yaw;
    output->motor[2] = throttle + roll + pitch - yaw;
    output->motor[3] = throttle - roll + pitch + yaw;

    for (int i = 0; i < 4; ++i) {
        if (output->motor[i] < 0) {
            output->motor[i] = 0;
        } else if (output->motor[i] > 1000) {
            output->motor[i] = 1000;
        }
    }
}
