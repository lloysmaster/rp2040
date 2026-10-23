#ifndef CONFIG_RCMAP_H
#define CONFIG_RCMAP_H

// --- MAPEO DE LOS CANALES DEL CONTROL ---
#define RC_CHANNEL_ROLL     0
#define RC_CHANNEL_PITCH    1
#define RC_CHANNEL_THROTTLE 2
#define RC_CHANNEL_YAW      3
#define RC_CHANNEL_ARM      4

// Sentido de cada eje
#define RC_SIGN_ROLL  (-1)
#define RC_SIGN_PITCH (+1)
#define RC_SIGN_YAW   (+1)

// --- CONFIGURACIÓN DE RATES ESTILO BETAFLIGHT ---
// RC Rate: Multiplicador base (ej. 1.0 = 100%)
#define RC_RATE         1.00f
// Expo: Sensibilidad cerca del centro (0.0 = lineal, hasta 1.0)
#define RC_EXPO         0.20f
// Super Rate: Define cuánta agilidad extra tienes en los extremos de las palancas
#define RC_SUPER_RATE   0.70f

#endif // CONFIG_RCMAP_H