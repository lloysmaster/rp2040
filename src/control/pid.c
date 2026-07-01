#include "pid.h"

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_integral) {
    if (pid == 0) {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->max_output = max_output;
    pid->max_integral = max_integral;
}

float pid_update(pid_t *pid, float setpoint, float measurement, float dt_s) {
    if (pid == 0) {
        return 0.0f;
    }

    float error = setpoint - measurement;
    pid->integral += error * dt_s;

    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    float derivative = (error - pid->prev_error) / (dt_s > 0.0f ? dt_s : 0.001f);
    float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);

    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }

    pid->prev_error = error;
    return output;
}
