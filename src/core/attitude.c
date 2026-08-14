#include <stddef.h>
#include <math.h>
#include "attitude.h"
#include "control/pid.h"
#include "control/mixer.h"
#include "hal/esc/dshot.h"
#include "config/rcMap.h"

typedef struct {
    float prev_filtered[3];
} filter_state_t;

static pid_t roll_pid;
static pid_t pitch_pid;
static pid_t yaw_pid;
static bool attitude_ready = false;
static flight_mode_t current_mode = FLIGHT_MODE_STABILIZED;
static filter_state_t filter_state;

// Ángulos estimados en radianes (referencia para la telemetría)
static float angle_roll = 0.0f;
static float angle_pitch = 0.0f;
static float angle_yaw = 0.0f;

#define DEG_TO_RAD 0.01745329252f
#define ATTITUDE_COMPLEMENTARY_ALPHA 0.98f
#define PI_F 3.14159265359f

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

    switch (current_mode) {
        case FLIGHT_MODE_KALMAN:
            for (int i = 0; i < 3; ++i) {
                filtered[i] = raw[i];
            }
            break;
        case FLIGHT_MODE_STABILIZED:
        case FLIGHT_MODE_ACRO:
        default:
            apply_notch_filter(raw, filtered);
            break;
    }
}

void attitude_init(void) {
    pid_init(&roll_pid, 0.25f, 0.01f, 0.002f, 500.0f, 2000.0f);
    pid_init(&pitch_pid, 0.25f, 0.01f, 0.002f, 500.0f, 2000.0f);
    pid_init(&yaw_pid, 0.18f, 0.005f, 0.001f, 300.0f, 1500.0f);
    for (int i = 0; i < 3; ++i) {
        filter_state.prev_filtered[i] = 0.0f;
    }
    angle_roll = 0.0f;
    angle_pitch = 0.0f;
    angle_yaw = 0.0f;
    attitude_ready = true;
}

void attitude_set_mode(flight_mode_t mode) {
    current_mode = mode;
}

flight_mode_t attitude_get_mode(void) {
    return current_mode;
}

void attitude_estimate(const q16_16 accel[3], const q16_16 gyro[3], float dt_s) {
    if (accel == NULL || gyro == NULL || dt_s <= 0.0f) {
        return;
    }

    const float ax = q16_to_float(accel[0]);
    const float ay = q16_to_float(accel[1]);
    const float az = q16_to_float(accel[2]);

    const float roll_rate = q16_to_float(gyro[0]) * DEG_TO_RAD;
    const float pitch_rate = q16_to_float(gyro[1]) * DEG_TO_RAD;
    const float yaw_rate = q16_to_float(gyro[2]) * DEG_TO_RAD;

    // Predicción por integración del giroscopio
    float roll = angle_roll + roll_rate * dt_s;
    float pitch = angle_pitch + pitch_rate * dt_s;
    angle_yaw += yaw_rate * dt_s;

    // Corrección con el acelerómetro solo si el módulo está cerca de 1 g
    const float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_magnitude > 0.7f && accel_magnitude < 1.3f) {
        const float roll_acc = atan2f(ay, az);
        const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
        roll = ATTITUDE_COMPLEMENTARY_ALPHA * roll + (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * roll_acc;
        pitch = ATTITUDE_COMPLEMENTARY_ALPHA * pitch + (1.0f - ATTITUDE_COMPLEMENTARY_ALPHA) * pitch_acc;
    }

    angle_roll = roll;
    angle_pitch = pitch;

    // El yaw se integra sin referencia absoluta: se normaliza a [-pi, pi]
    while (angle_yaw > PI_F) angle_yaw -= 2.0f * PI_F;
    while (angle_yaw < -PI_F) angle_yaw += 2.0f * PI_F;
}

void attitude_get_angles(float *roll_rad, float *pitch_rad, float *yaw_rad) {
    if (roll_rad != NULL) *roll_rad = angle_roll;
    if (pitch_rad != NULL) *pitch_rad = angle_pitch;
    if (yaw_rad != NULL) *yaw_rad = angle_yaw;
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

    const float desired_roll  = RC_SIGN_ROLL  * rc_to_rate(rc_data->channels[RC_CHANNEL_ROLL]);
    const float desired_pitch = RC_SIGN_PITCH * rc_to_rate(rc_data->channels[RC_CHANNEL_PITCH]);
    const float desired_yaw   = RC_SIGN_YAW   * rc_to_rate(rc_data->channels[RC_CHANNEL_YAW]);

    output->roll_output = (int32_t)pid_update(&roll_pid, desired_roll, roll_rate, dt_s);
    output->pitch_output = (int32_t)pid_update(&pitch_pid, desired_pitch, pitch_rate, dt_s);
    output->yaw_output = (int32_t)pid_update(&yaw_pid, desired_yaw, yaw_rate, dt_s);

    const uint16_t rc_throttle = rc_data->channels[RC_CHANNEL_THROTTLE];
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
