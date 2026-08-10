#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double latitude;
    double longitude;
    float altitude;
    bool fix_valid;
    uint32_t last_packet_time;
} gps_data_t;

void gps_init(void);
bool gps_update(void);
const gps_data_t* gps_get_data(void);

#endif