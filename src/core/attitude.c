#include <stddef.h>
#include "attitude.h"
#include "control/pid.h"
#include "control/mixer.h"

static pid_t roll_pid;
static pid_t pitch_pid;
static pid_t yaw_pid;
static bool attitude_ready = false;

static float q16_to_float(q16_16 value) {
    return (float)value / 65536.0f;
}

static float rc_to_rate(uint16_t channel) {
    int32_t centered = (int32_t)channel - 992;
    if (centered > 250) {
        centered = 250;
    } else if (centered < -250) {
        centered = -250;
    }

    if (centered > -20 && centered < 20) {
        centered = 0;
    }

    return (float)centered * 0.25f;
}

void attitude_init(void) {
    pid_init(&roll_pid, 0.25f, 0.01f, 0.002f, 500.0f, 2000.0f);
    pid_init(&pitch_pid, 0.25f, 0.01f, 0.002f, 500.0f, 2000.0f);
    pid_init(&yaw_pid, 0.18f, 0.005f, 0.001f, 300.0f, 1500.0f);
    attitude_ready = true;
}

void attitude_update(const crsf_data_t *rc_data, const q16_16 gyro[3], attitude_cmd_t *output) {
    if (!attitude_ready || rc_data == NULL || gyro == NULL || output == NULL) {
        return;
    }

    const float dt_s = 0.005f;
    const float roll_rate = q16_to_float(gyro[0]);
    const float pitch_rate = q16_to_float(gyro[1]);
    const float yaw_rate = q16_to_float(gyro[2]);

    const float desired_roll = rc_to_rate(rc_data->channels[0]);
    const float desired_pitch = -rc_to_rate(rc_data->channels[1]);
    const float desired_yaw = rc_to_rate(rc_data->channels[3]);

    output->roll_output = (int32_t)pid_update(&roll_pid, desired_roll, roll_rate, dt_s);
    output->pitch_output = (int32_t)pid_update(&pitch_pid, desired_pitch, pitch_rate, dt_s);
    output->yaw_output = (int32_t)pid_update(&yaw_pid, desired_yaw, yaw_rate, dt_s);
    output->throttle = (rc_data->channels[2] < 172) ? 0 : (int32_t)((rc_data->channels[2] - 172) * 1000 / 1639);
    output->enabled = rc_data->is_connected;
}
