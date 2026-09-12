#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_measurement;
    float max_output;
    float max_integral;
    float max_i_term; // Limite directo sobre la contribución I
} pid_t;

void pid_init(pid_t *pid, float kp, float ki, float kd, float max_output, float max_integral);
float pid_update(pid_t *pid, float setpoint, float measurement, float dt_s);

#endif
