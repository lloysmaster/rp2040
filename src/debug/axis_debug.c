#include "debug/axis_debug.h"

#include <stdio.h>

// Umbrales para considerar que algo se movió de verdad y no es ruido
#define RC_MOVE_THRESHOLD 40    // cuentas CRSF (rango 172..1811, centro 992)
#define GYRO_MOVE_THRESHOLD 15.0f // °/s

#define AXIS_DEBUG_REPORTED_CHANNELS 8

// Rol que el firmware asigna a cada canal (ver attitude_update y main).
static const char *channel_role(uint8_t index) {
    switch (index) {
        case 0: return "ROLL (alabeo)";
        case 1: return "PITCH (cabeceo)";
        case 2: return "THROTTLE (gas)";
        case 3: return "YAW (guiñada)";
        case 4: return "ARM (armado)";
        default: return "sin uso";
    }
}

static const char *channel_direction(uint8_t index, int32_t deviation) {
    const bool positive = deviation > 0;
    switch (index) {
        case 0: return positive ? "stick a la DERECHA" : "stick a la IZQUIERDA";
        case 1: return positive ? "stick ATRAS (morro arriba)" : "stick ADELANTE (morro abajo)";
        case 2: return positive ? "gas ARRIBA" : "gas ABAJO";
        case 3: return positive ? "giro a la DERECHA" : "giro a la IZQUIERDA";
        case 4: return positive ? "interruptor ARRIBA" : "interruptor ABAJO";
        default: return positive ? "hacia +" : "hacia -";
    }
}

static uint16_t rc_reference[CRSF_MAX_CHANNELS];
static bool rc_reference_valid = false;

// Picos de giroscopio acumulados entre impresiones (con signo)
static float gyro_peak[3];
static float accel_last[3];

static float q16_to_float(q16_16 value) {
    return (float)value / 65536.0f;
}

void axis_debug_init(void) {
    rc_reference_valid = false;
    for (int i = 0; i < CRSF_MAX_CHANNELS; ++i) {
        rc_reference[i] = 0;
    }
    for (int i = 0; i < 3; ++i) {
        gyro_peak[i] = 0.0f;
        accel_last[i] = 0.0f;
    }
}

void axis_debug_feed_imu(const q16_16 gyro[3], const q16_16 accel[3]) {
    if (gyro != NULL) {
        for (int i = 0; i < 3; ++i) {
            const float rate = q16_to_float(gyro[i]);
            const float magnitude = rate < 0.0f ? -rate : rate;
            const float peak = gyro_peak[i] < 0.0f ? -gyro_peak[i] : gyro_peak[i];
            if (magnitude > peak) {
                gyro_peak[i] = rate;
            }
        }
    }
    if (accel != NULL) {
        for (int i = 0; i < 3; ++i) {
            accel_last[i] = q16_to_float(accel[i]);
        }
    }
}

static void print_gyro_axes(void) {
    static const char *axis_name[3] = {"X -> ROLL", "Y -> PITCH", "Z -> YAW"};

    int dominant = -1;
    float dominant_magnitude = GYRO_MOVE_THRESHOLD;
    for (int i = 0; i < 3; ++i) {
        const float magnitude = gyro_peak[i] < 0.0f ? -gyro_peak[i] : gyro_peak[i];
        if (magnitude > dominant_magnitude) {
            dominant_magnitude = magnitude;
            dominant = i;
        }
    }

    printf("[EJES] Giro pico (deg/s): X(roll)=%+7.1f  Y(pitch)=%+7.1f  Z(yaw)=%+7.1f -> %s\n",
           gyro_peak[0], gyro_peak[1], gyro_peak[2],
           dominant < 0 ? "sin movimiento" : axis_name[dominant]);

    float roll_rad = 0.0f, pitch_rad = 0.0f, yaw_rad = 0.0f;
    attitude_get_angles(&roll_rad, &pitch_rad, &yaw_rad);
    printf("[EJES] Angulos (deg): roll=%+7.1f  pitch=%+7.1f  yaw=%+7.1f | Accel (g): x=%+5.2f y=%+5.2f z=%+5.2f\n",
           roll_rad * 57.2957795f, pitch_rad * 57.2957795f, yaw_rad * 57.2957795f,
           accel_last[0], accel_last[1], accel_last[2]);

    for (int i = 0; i < 3; ++i) {
        gyro_peak[i] = 0.0f;
    }
}

static void print_rc_channels(const crsf_data_t *rc_data) {
    printf("[RC] ");
    for (uint8_t i = 0; i < AXIS_DEBUG_REPORTED_CHANNELS; ++i) {
        printf("CH%u=%4u ", (unsigned)(i + 1), (unsigned)rc_data->channels[i]);
    }
    printf("\n");

    if (!rc_reference_valid) {
        printf("[RC] Esperando referencia de canales (mantener sticks quietos)\n");
        return;
    }

    int8_t moved = -1;
    int32_t moved_deviation = 0;
    uint8_t moved_count = 0;
    for (uint8_t i = 0; i < CRSF_MAX_CHANNELS; ++i) {
        const int32_t deviation = (int32_t)rc_data->channels[i] - (int32_t)rc_reference[i];
        const int32_t magnitude = deviation < 0 ? -deviation : deviation;
        if (magnitude < RC_MOVE_THRESHOLD) {
            continue;
        }
        ++moved_count;
        const int32_t best = moved_deviation < 0 ? -moved_deviation : moved_deviation;
        if (magnitude > best) {
            moved = (int8_t)i;
            moved_deviation = deviation;
        }
    }

    if (moved < 0) {
        printf("[RC] Sticks en reposo\n");
        return;
    }

    printf("[RC] Moviendo CH%u = %s | valor=%u ref=%u desvio=%+ld (%s)%s\n",
           (unsigned)(moved + 1),
           channel_role((uint8_t)moved),
           (unsigned)rc_data->channels[moved],
           (unsigned)rc_reference[moved],
           (long)moved_deviation,
           channel_direction((uint8_t)moved, moved_deviation),
           moved_count > 1 ? " | ATENCION: mas de un canal en movimiento" : "");
}

void axis_debug_print(const crsf_data_t *rc_data,
                      const attitude_cmd_t *cmd,
                      const mixer_output_t *motors,
                      bool armed) {
    if (rc_data == NULL || cmd == NULL || motors == NULL) {
        return;
    }

    if (rc_data->is_connected) {
        if (!rc_reference_valid) {
            for (uint8_t i = 0; i < CRSF_MAX_CHANNELS; ++i) {
                rc_reference[i] = rc_data->channels[i];
            }
            rc_reference_valid = true;
            printf("[RC] Referencia de canales tomada con los sticks en reposo\n");
        }
    } else {
        rc_reference_valid = false;
        printf("[RC] Receptor desconectado\n");
    }

    print_gyro_axes();
    print_rc_channels(rc_data);

    printf("[MIX] Armado=%d | PID roll=%+ld pitch=%+ld yaw=%+ld thr=%+ld | M1=%ld M2=%ld M3=%ld M4=%ld\n",
           armed ? 1 : 0,
           (long)cmd->roll_output, (long)cmd->pitch_output,
           (long)cmd->yaw_output, (long)cmd->throttle,
           (long)motors->motor[0], (long)motors->motor[1],
           (long)motors->motor[2], (long)motors->motor[3]);
}
