#include <stddef.h>
#include <math.h>
#include "attitude.h"
#include "control/pid.h"
#include "control/mixer.h"
#include "hal/esc/dshot.h"
#include "config/rcMap.h"
#include "config/gyro.h"
_Static_assert(RC_BASE_RATE_DPS * RC_RATE / (1.0f - RC_SUPER_RATE) < 0.85f * GYRO_FULL_SCALE_DPS,
               "Rate maximo demasiado cerca del fondo de escala del giro");

typedef struct {
    float prev_filtered[3];
} filter_state_t;

static pid_t roll_pid;
static pid_t pitch_pid;
static pid_t yaw_pid;
static bool attitude_ready = false;
static flight_mode_t current_mode = FLIGHT_MODE_STABILIZED;
static filter_state_t filter_state;

#define ATT_FF_ROLL   0.25f
#define ATT_FF_PITCH  0.25f
#define ATT_FF_YAW    0.25f

static pid_t * const axis_pid[3] = { &roll_pid, &pitch_pid, &yaw_pid };
static float ff_gain[3] = { ATT_FF_ROLL, ATT_FF_PITCH, ATT_FF_YAW };
// Tope de la contribución FF (~40 % del max_output de cada eje)
static const float ff_max[3]  = { 200.0f, 200.0f, 120.0f };

// Ángulos estimados en radianes (referencia para la telemetría)
static float angle_roll = 0.0f;
static float angle_pitch = 0.0f;
static float angle_yaw = 0.0f;


#define PID_KP_MAX 50.0f
#define PID_KI_MAX 5.0f
#define PID_KD_MAX 5.0f
#define PID_KFF_MAX 5.0f


// Constante de tiempo del filtro complementario. Equivale al alpha 0.98 que se
// usaba con el periodo fijo de 5 ms: tau = dt * alpha / (1 - alpha).
#define ATTITUDE_COMPLEMENTARY_TAU_S 0.245f
// Corte del filtro de velocidades angulares. Equivale al alpha 0.18 que se
// usaba a 200 Hz: fc = (1 - alpha) / (2 * pi * dt * alpha).
#define ATTITUDE_RATE_FILTER_CUTOFF_HZ 80.0f
#define PI_F 3.14159265359f

static float q16_to_float(q16_16 value) {
    return (float)value / 65536.0f;
}

void attitude_reset_pids(void) {
    pid_reset(&roll_pid);
    pid_reset(&pitch_pid);
    pid_reset(&yaw_pid);
    for (int i = 0; i < 3; ++i) {
        filter_state.prev_filtered[i] = 0.0f;
    }
}
uint8_t attitude_set_pid(uint8_t axis, float kp, float ki, float kd, float kff) {
    if (axis > 2) return PID_STATUS_BAD_AXIS;
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) || !isfinite(kff) ||
        kp < 0.0f || kp > PID_KP_MAX ||
        ki < 0.0f || ki > PID_KI_MAX ||
        kd < 0.0f || kd > PID_KD_MAX ||
        kff < 0.0f || kff > PID_KFF_MAX) {
        return PID_STATUS_BAD_RANGE;
    }
    ff_gain[axis] = kff;
    // Mismos límites que en attitude_init(); pid_init además resetea el integrador
    switch (axis) {
        case 0: pid_init(&roll_pid,  kp, ki, kd, 500.0f, 2000.0f); break;
        case 1: pid_init(&pitch_pid, kp, ki, kd, 500.0f, 2000.0f); break;
        case 2: pid_init(&yaw_pid,   kp, ki, kd, 300.0f, 1500.0f); break;
    }
        // pid_init puso kff en 0: se vuelve a aplicar el feedforward de ese eje
    pid_set_ff(axis_pid[axis], ff_gain[axis], ff_max[axis]);
    return PID_STATUS_OK;
}
static float rc_to_rate(uint16_t channel) {
    // 1. Clampear a límites físicos del receptor
    if (channel < CRSF_CHANNEL_MIN) channel = CRSF_CHANNEL_MIN;
    if (channel > CRSF_CHANNEL_MAX) channel = CRSF_CHANNEL_MAX;

    // 2. Normalizar de -1.0f a +1.0f centrado en CRSF_CHANNEL_MID
    float rcCommand = 0.0f;
    if (channel >= CRSF_CHANNEL_MID) {
        rcCommand = (float)(channel - CRSF_CHANNEL_MID) / (float)(CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID);
    } else {
        rcCommand = (float)(channel - CRSF_CHANNEL_MID) / (float)(CRSF_CHANNEL_MID - CRSF_CHANNEL_MIN);
    }

    // Guardrail por si hay jitter fuera de rango
    if (rcCommand > 1.0f) rcCommand = 1.0f;
    if (rcCommand < -1.0f) rcCommand = -1.0f;

    // Guardamos el signo para aplicarlo al final
    float sign = (rcCommand < 0.0f) ? -1.0f : 1.0f;
    float absRc = fabsf(rcCommand);

    // 3. Cálculo de Expo estilo Betaflight
    // Fórmula: rcCommand * (rcCommand^2 * expo + 1 - expo)
    float rcFactor = 1.0f - RC_EXPO + (RC_EXPO * absRc * absRc);
    float expoCommand = absRc * rcFactor;

    // 4. Cálculo de Super Rate (Agilidad extra en extremos)
    // Fórmula Betaflight rates: anglular_velocity = rcRate * expoCommand / (1 - (absRc * superRate))
    float superFactor = 1.0f - (absRc * RC_SUPER_RATE);
    if (superFactor < 0.01f) {
        superFactor = 0.01f; // Evitar división por cero
    }
    
    float finalRate = (RC_RATE * expoCommand) / superFactor;

    // 5. Mapeo final a rad/s usando tu MAX_RATE_RAD_S como referencia máxima
    return sign * finalRate * RC_BASE_RATE_RAD_S;
}
// Un alpha fijo ata el corte del filtro al periodo del bucle: al cambiar la
// frecuencia habria que retocarlo a mano. Se deriva de dt para que el corte en
// Hz sea el mismo a cualquier cadencia.
static float lowpass_alpha(float cutoff_hz, float dt_s) {
    const float rc = 1.0f / (2.0f * PI_F * cutoff_hz);
    return dt_s / (rc + dt_s);
}

static void apply_lowpass_filter(const float raw[3], float filtered[3], float alpha) {
    for (int i = 0; i < 3; ++i) {
        filter_state.prev_filtered[i] = filter_state.prev_filtered[i] + alpha * (raw[i] - filter_state.prev_filtered[i]);
        filtered[i] = filter_state.prev_filtered[i];
    }
}

static void apply_sensor_filter(const q16_16 gyro[3], float filtered[3], float dt_s) {
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
            apply_lowpass_filter(raw, filtered,
                               lowpass_alpha(ATTITUDE_RATE_FILTER_CUTOFF_HZ, dt_s));
            break;
    }
}

void attitude_init(void) {
    pid_init(&roll_pid, 38.0f, 0.11f, 0.008f, 500.0f, 2000.0f);
    pid_init(&pitch_pid, 38.0f, 0.11f, 0.008f, 500.0f, 2000.0f);
    pid_init(&yaw_pid, 38.0f, 0.1f, 0.0f, 300.0f, 1500.0f);
    for (int i = 0; i < 3; ++i) {
        pid_set_ff(axis_pid[i], ff_gain[i], ff_max[i]);
    }
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

    const float roll_rate  = q16_to_float(gyro[0]); // Ya está en rad/s
    const float pitch_rate = q16_to_float(gyro[1]); // Ya está en rad/s
    const float yaw_rate   = q16_to_float(gyro[2]); // Ya está en rad/s

    // Predicción por integración del giroscopio
    float roll = angle_roll + roll_rate * dt_s;
    float pitch = angle_pitch + pitch_rate * dt_s;
    angle_yaw += yaw_rate * dt_s;

    // Corrección con el acelerómetro solo si el módulo está cerca de 1 g
    const float accel_magnitude = sqrtf(ax * ax + ay * ay + az * az);
    if (accel_magnitude > 0.7f && accel_magnitude < 1.3f) {
        const float roll_acc = atan2f(ay, az);
        const float pitch_acc = atan2f(-ax, sqrtf(ay * ay + az * az));
        const float alpha = ATTITUDE_COMPLEMENTARY_TAU_S /
                            (ATTITUDE_COMPLEMENTARY_TAU_S + dt_s);
        roll = alpha * roll + (1.0f - alpha) * roll_acc;
        pitch = alpha * pitch + (1.0f - alpha) * pitch_acc;
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

void attitude_update(const crsf_data_t *rc_data, const q16_16 gyro[3],
                    attitude_cmd_t *output, float dt_s) {
    if (!attitude_ready || rc_data == NULL || gyro == NULL || output == NULL ||
        dt_s <= 0.0f) {
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

    float filtered_rates[3];
    apply_sensor_filter(gyro, filtered_rates, dt_s);
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
    // Mapeo directo al rango DShot (DSHOT_MIN_THROTTLE a 2047)
    output->throttle = (int32_t)(DSHOT_MIN_THROTTLE + 
        ((uint32_t)(rc_throttle - throttle_min) * (2047u - DSHOT_MIN_THROTTLE)) / 
        (throttle_max - throttle_min));
}
    output->enabled = rc_data->is_connected;
}
