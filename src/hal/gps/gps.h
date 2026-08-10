#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t latitude_1e7;   // grados * 1e7 (formato CRSF)
    int32_t longitude_1e7;  // grados * 1e7 (formato CRSF)
    int16_t altitude_m;     // metros sobre el nivel del mar
    uint16_t speed_kmh_x10; // km/h * 10
    uint16_t heading_x100;  // grados * 100 (0..35999)
    uint8_t satellites;
    bool fix_valid;
    uint32_t last_fix_time; // time_us_32() de la última sentencia válida
} gps_data_t;

void gps_init(void);

// Procesa los bytes acumulados por DMA. Retorna true si se actualizó la posición.
bool gps_update(void);

const gps_data_t* gps_get_data(void);

#endif
