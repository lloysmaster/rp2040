#ifndef LOOPCONFIG_H
#define LOOPCONFIG_H

// --- PARAMETROS DE VUELO ---
// Cadencia del ejecutivo ciclico. El coste medido de un ciclo completo es de
// unos 250 us (140 us de calculo mas la lectura del MPU), asi que a 1 kHz el
// RP2040 queda en torno al 25-35 % de ocupacion. No conviene pasar de 1 kHz sin
// desactivar GYRO_DEBUG_ENABLED y compilar en Release: la tasa de salida del
// sensor tambien la fija MPU_SMPLRT_DIV/MPU_DLPF_CFG (1 kHz por defecto) y el
// bucle no puede ir mas rapido que el sensor.
#define LOOP_FREQ_HZ     1000
#define TARGET_LOOP_US   (1000000 / LOOP_FREQ_HZ)
// Paso de integracion nominal. Solo se usa como respaldo: el bucle mide el dt
// real entre muestras.
#define NOMINAL_LOOP_DT_S (1.0f / (float)LOOP_FREQ_HZ)

#endif
