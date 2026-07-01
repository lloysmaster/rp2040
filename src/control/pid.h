#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float max_output;
    float max_integral;
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_integral);
float pid_update(pid_t *pid, float setpoint, float measurement, float dt_s);

#endif
