#include <stddef.h>
#include "attitude.h"
#include "control/pid.h"
#include "hal/esc/dshot.h"

typedef struct {
    float prev_filtered[3];
} filter_state_t;

static pid_t roll_pid;
static pid_t pitch_pid;
static pid_t yaw_pid;
static bool attitude_ready = false;
static filter_state_t filter_state;

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

static void apply_notch_filter(const float raw[3], float filtered[3]) {
    const float alpha = 0.18f;
    for (int i = 0; i < 3; ++i) {
        filter_state.prev_filtered[i] = filter_state.prev_filtered[i] + alpha * (raw[i] - filter_state.prev_filtered[i]);
        filtered[i] = filter_state.prev_filtered[i];
    }
}

static void apply_sensor_filter(const q16_16 gyro[3], float filtered[3]) {
    float raw[3];
    for (int i = 0; i < 3; ++i) {
        raw[i] = q16_to_float(gyro[i]);
    }

    apply_notch_filter(raw, filtered);
}

void attitude_init(void) {
    pid_init(&roll_pid, 0.25f, 0.01f, 0.002f, 500.0f, 2000.0f);
    pid_init(&pitch_pid, 0.25f, 0.01f, 0.002f, 500.0f, 2000.0f);
    pid_init(&yaw_pid, 0.18f, 0.005f, 0.001f, 300.0f, 1500.0f);
    for (int i = 0; i < 3; ++i) {
        filter_state.prev_filtered[i] = 0.0f;
    }
    attitude_ready = true;
}

void attitude_update(const crsf_data_t *rc_data, const q16_16 gyro[3], attitude_cmd_t *output) {
    if (!attitude_ready || rc_data == NULL || gyro == NULL || output == NULL) {
        return;
    }

    if (!rc_data->is_connected) {
        output->roll_output = 0;
        output->pitch_output = 0;
        output->yaw_output = 0;
        output->throttle = 0;
        output->enabled = false;
        return;
    }

    const float dt_s = 0.005f;
    float filtered_rates[3];
    apply_sensor_filter(gyro, filtered_rates);
    const float roll_rate = filtered_rates[0];
    const float pitch_rate = filtered_rates[1];
    const float yaw_rate = filtered_rates[2];

    const float desired_roll = rc_to_rate(rc_data->channels[0]);
    const float desired_pitch = -rc_to_rate(rc_data->channels[1]);
    const float desired_yaw = rc_to_rate(rc_data->channels[3]);

    output->roll_output = (int32_t)pid_update(&roll_pid, desired_roll, roll_rate, dt_s);
    output->pitch_output = (int32_t)pid_update(&pitch_pid, desired_pitch, pitch_rate, dt_s);
    output->yaw_output = (int32_t)pid_update(&yaw_pid, desired_yaw, yaw_rate, dt_s);

    const uint16_t rc_throttle = rc_data->channels[2];
    const uint16_t throttle_min = 172u;
    const uint16_t throttle_max = 1811u;
    if (rc_throttle <= throttle_min) {
        output->throttle = (int32_t)DSHOT_MIN_THROTTLE;
    } else {
        output->throttle = (int32_t)((rc_throttle - throttle_min) * 1000u / (throttle_max - throttle_min));
        if (output->throttle < (int32_t)DSHOT_MIN_THROTTLE) {
            output->throttle = (int32_t)DSHOT_MIN_THROTTLE;
        }
    }
    output->enabled = rc_data->is_connected;
}
