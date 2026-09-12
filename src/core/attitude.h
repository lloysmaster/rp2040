#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>
#include "math/fixed_point.h"
#include "hal/rx/crossfire.h"
#include "core/flighMode.h"


#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 992
#define CRSF_CHANNEL_MAX 1811

// Define la tasa máxima de rotación deseada cuando el stick esté al 100%
#define MAX_RATE_DPS     500.0f                 // Ejemplo: 500 °/s en fondo de escala
#define DEG_TO_RAD_F     0.01745329252f         // Identico al usado en mpu.c
#define MAX_RATE_RAD_S   (MAX_RATE_DPS * DEG_TO_RAD_F) // ~8.72 rad/s

typedef struct {
    int32_t roll_output;
    int32_t pitch_output;
    int32_t yaw_output;
    int32_t throttle;
    bool enabled;
} attitude_cmd_t;

void attitude_init(void);
void attitude_set_mode(flight_mode_t mode);
flight_mode_t attitude_get_mode(void);
// dt_s es el tiempo real transcurrido desde la muestra anterior: los PID y el
// filtro de velocidades lo necesitan para que sus ganancias y su frecuencia de
// corte no dependan de la cadencia del bucle.
void attitude_update(const crsf_data_t *rc_data, const q16_16 gyro[3],
                     attitude_cmd_t *output, float dt_s);

// Estimación de ángulos (filtro complementario giroscopio + acelerómetro).
// Necesaria para la telemetría CRSF de actitud.
void attitude_estimate(const q16_16 accel[3], const q16_16 gyro[3], float dt_s);
void attitude_get_angles(float *roll_rad, float *pitch_rad, float *yaw_rad);

#endif
