#ifndef AXIS_DEBUG_H
#define AXIS_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

#include "control/mixer.h"
#include "core/attitude.h"
#include "hal/rx/crossfire.h"
#include "math/fixed_point.h"

// Depuración de ejes: sirve para comprobar que roll/pitch/yaw del giroscopio,
// del estimador y del mando coinciden entre sí y que ningún canal del control
// está intercambiado.
//
// Procedimiento:
//   1. Con el dron quieto y el mando encendido, esperar la línea de referencia
//      (se toma automáticamente al recibir el primer paquete CRSF).
//   2. Mover un solo stick por vez: el debug indica qué canal se movió, con qué
//      eje está mapeado y hacia dónde.
//   3. Inclinar el dron sobre un eje por vez: el debug indica qué eje del
//      giroscopio se movió y qué eje de control (roll/pitch/yaw) lo consume.

void axis_debug_init(void);

// Acumula los picos de giroscopio/acelerómetro entre impresiones. Llamar en
// cada muestra del IMU (200 Hz) para no perder movimientos rápidos.
void axis_debug_feed_imu(const q16_16 gyro[3], const q16_16 accel[3]);

// Imprime el estado de ejes y canales. Llamar de forma periódica (~5 Hz).
void axis_debug_print(const crsf_data_t *rc_data,
                      const attitude_cmd_t *cmd,
                      const mixer_output_t *motors,
                      bool armed);

#endif // AXIS_DEBUG_H
