// --- PARÁMETROS DE FILTRADO Y PID ---
#define FILTER_CUTOFF_HZ  80.0f
#define FILTER_DT        (1.0f / LOOP_FREQ_HZ)
#define FILTER_RC        (1.0f / (2.0f * 3.14159f * FILTER_CUTOFF_HZ))
#define FILTER_ALPHA     (FILTER_DT / (FILTER_RC + FILTER_DT))
#define PID_LIMIT_I      100.0f
// Puedes poner aquí tus Kp, Ki, Kd iniciales
#define ROLL_KP          0.0f
#define ROLL_KI          0.0f
#define ROLL_KD          0.0f

#define PITCH_KP          0.0f
#define PITCH_KI          0.0f
#define PITCH_KD          0.0f

#define YAW_KP          0.0f
#define YAW_KI          0.0f
#define YAW_KD          0.0f
