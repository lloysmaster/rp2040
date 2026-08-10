#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>

// Planificador de telemetría CRSF: toma los datos de batería, GPS y actitud y
// los envía por el UART CRSF a distintas frecuencias sin bloquear el lazo.
void telemetry_init(void);

// Indica si el vehículo está armado (afecta la cadena de modo de vuelo)
void telemetry_set_armed(bool armed);

// Llamar en cada iteración del lazo principal
void telemetry_update(void);

#endif // TELEMETRY_H
