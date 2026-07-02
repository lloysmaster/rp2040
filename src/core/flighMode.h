#ifndef FLIGHTMODE_H
#define FLIGHTMODE_H

typedef enum {
    FLIGHT_MODE_RATE = 0,
    FLIGHT_MODE_STABILIZED = 1,
    FLIGHT_MODE_ACRO = 2,
    FLIGHT_MODE_FREESTYLE = 3,
    FLIGHT_MODE_KALMAN = 4
} flight_mode_t;

#endif
