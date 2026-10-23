#include "pid.h"
#include <stddef.h>

// Corte del filtro sobre la derivada del setpoint. Los paquetes CRSF llegan
// a otra cadencia que el bucle, así que la derivada sale "escalonada" y hay
// que suavizarla.
#define PID_FF_FILTER_CUTOFF_HZ 40.0f
#define PID_TWO_PI              6.28318530718f

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_i_term) {
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->max_output = max_output;
    pid->max_i_term = max_i_term; // Limite directo sobre la contribución I
    pid->kff = 0.0f;
    pid->max_ff_term = 0.0f;
    pid->prev_setpoint = 0.0f;
    pid->ff_filtered = 0.0f;
    pid->ff_primed = false;
}
void pid_set_ff(pid_t *pid, float kff, float max_ff_term) {
    if (pid == NULL) return;
    pid->kff = kff;
    pid->max_ff_term = max_ff_term;
    pid->ff_filtered = 0.0f;
    pid->ff_primed = false;
}
void pid_reset(pid_t *pid) {
    if (pid == NULL) return;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->ff_filtered = 0.0f;
    pid->ff_primed = false;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt_s) {
    if (pid == NULL || dt_s <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // 1. Término Proporcional
    float p_term = pid->kp * error;

    // 2. Término Integral con anti-windup acotado a la salida
    pid->integral += error * dt_s;
    float i_term = pid->ki * pid->integral;

    if (i_term > pid->max_i_term) {
        i_term = pid->max_i_term;
        pid->integral = i_term / pid->ki; // Clampea el acumulador interno
    } else if (i_term < -pid->max_i_term) {
        i_term = -pid->max_i_term;
        pid->integral = i_term / pid->ki;
    }

    // 3. Término Derivativo sobre la medición (previene Derivative Kick)
    float d_measurement = (measurement - pid->prev_measurement) / dt_s;
    float d_term = -pid->kd * d_measurement;

    // 4. feed foward
    float ff_term = 0.0f;
    if (pid->kff > 0.0f) {
        if (!pid->ff_primed) {
            // Primera muestra tras reset: evita un pico en la derivada
            pid->prev_setpoint = setpoint;
            pid->ff_filtered = 0.0f;
            pid->ff_primed = true;
        }
        const float d_setpoint = (setpoint - pid->prev_setpoint) / dt_s;
        pid->prev_setpoint = setpoint;

        const float rc = 1.0f / (PID_TWO_PI * PID_FF_FILTER_CUTOFF_HZ);
        const float alpha = dt_s / (rc + dt_s);
        pid->ff_filtered += alpha * (d_setpoint - pid->ff_filtered);

        ff_term = pid->kff * pid->ff_filtered;
        if (ff_term > pid->max_ff_term) ff_term = pid->max_ff_term;
        else if (ff_term < -pid->max_ff_term) ff_term = -pid->max_ff_term;
    }

    // 5. Suma y Clampeo de Salida Total
    float output = p_term + i_term + d_term + ff_term;

    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }

    pid->prev_measurement = measurement;
    return output;
}