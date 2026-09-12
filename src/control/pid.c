#include "pid.h"
#include <stddef.h>
void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_i_term) {
    if (pid == NULL) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_measurement = 0.0f;
    pid->max_output = max_output;
    pid->max_i_term = max_i_term; // Limite directo sobre la contribución I
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

    // 4. Suma y Clampeo de Salida Total
    float output = p_term + i_term + d_term;

    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }

    pid->prev_measurement = measurement;
    return output;
}