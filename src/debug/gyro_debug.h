#ifndef GYRO_DEBUG_H
#define GYRO_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#include "math/fixed_point.h"

// Debug de giroscopio orientado a ajustar gyro_sensitivity_lsb_per_dps y a
// verificar la calibración de sesgo.
//
// Procedimiento para medir la sensibilidad:
//   1. Dron quieto: comprobar que la deriva mostrada es cercana a 0 °/s
//      (si no, recalibrar con 'z').
//   2. Elegir el ángulo de referencia con '1' (90°), '2' (180°) o '3' (360°).
//   3. Pulsar 'r', girar el dron ese ángulo exacto sobre un solo eje y volver
//      a pulsar 'r'.
//   4. El debug imprime el ángulo integrado y la sensibilidad sugerida; con 'a'
//      se aplica en caliente y con 'z' se recalibra para probarla.
//   5. Copiar el valor final en GYRO_SENSITIVITY_LSB_PER_DPS (config/gyro.h).

void gyro_debug_init(void);

// Alimenta el debug con cada muestra del IMU (giro en °/s en Q16.16).
void gyro_debug_feed(const q16_16 gyro[3], const q16_16 accel[3], float dt_s);

// Procesa las teclas recibidas por el puerto serie e imprime el estado
// periódicamente. Llamar en cada iteración del bucle principal. Con los
// motores armados se rechaza la calibración, que bloquea el bucle unos
// segundos.
void gyro_debug_service(bool armed);

#endif // GYRO_DEBUG_H
