#include "telemetry/telemetry.h"
#include "hal/rx/crossfire.h"
#include "hal/gps/gps.h"
#include "hal/battery/battery.h"
#include "core/attitude.h"
#include "pico/stdlib.h"

// Periodos de envío por tipo de trama (microsegundos)
#define TELEMETRY_ATTITUDE_PERIOD_US    100000u // 10 Hz
#define TELEMETRY_BATTERY_PERIOD_US     200000u // 5 Hz
#define TELEMETRY_GPS_PERIOD_US         500000u // 2 Hz
#define TELEMETRY_FLIGHT_MODE_PERIOD_US 1000000u // 1 Hz

static uint32_t last_attitude_us = 0;
static uint32_t last_battery_us = 0;
static uint32_t last_gps_us = 0;
static uint32_t last_mode_us = 0;
static bool is_armed = false;

static const char* flight_mode_string(void) {
    if (!is_armed) {
        return "DISARM";
    }

    switch (attitude_get_mode()) {
        case FLIGHT_MODE_RATE:       return "RATE";
        case FLIGHT_MODE_STABILIZED: return "STAB";
        case FLIGHT_MODE_ACRO:       return "ACRO";
        case FLIGHT_MODE_FREESTYLE:  return "FREE";
        case FLIGHT_MODE_KALMAN:     return "KALMAN";
        default:                     return "WAIT";
    }
}

void telemetry_init(void) {
    const uint32_t now = time_us_32();
    last_attitude_us = now;
    last_battery_us = now;
    last_gps_us = now;
    last_mode_us = now;
    is_armed = false;
}

void telemetry_set_armed(bool armed) {
    is_armed = armed;
}

void telemetry_update(void) {
    const uint32_t now = time_us_32();

    if ((now - last_attitude_us) >= TELEMETRY_ATTITUDE_PERIOD_US) {
        float roll, pitch, yaw;
        attitude_get_angles(&roll, &pitch, &yaw);
        if (crsf_send_attitude(pitch, roll, yaw)) {
            last_attitude_us = now;
        }
    } else if ((now - last_battery_us) >= TELEMETRY_BATTERY_PERIOD_US) {
        const battery_data_t *bat = battery_get_data();
        if (crsf_send_battery(bat->voltage_dv, bat->current_da, bat->capacity_mah, bat->remaining_pct)) {
            last_battery_us = now;
        }
    } else if ((now - last_gps_us) >= TELEMETRY_GPS_PERIOD_US) {
        const gps_data_t *gps = gps_get_data();
        bool sent;
        if (gps->fix_valid) {
            sent = crsf_send_gps(gps->latitude_1e7, gps->longitude_1e7, gps->speed_kmh_x10,
                                 gps->heading_x100, gps->altitude_m, gps->satellites);
        } else {
            // Sin fix se informa posición nula y 0 satélites para que el emisor lo note
            sent = crsf_send_gps(0, 0, 0, 0, 0, 0);
        }
        if (sent) {
            last_gps_us = now;
        }
    } else if ((now - last_mode_us) >= TELEMETRY_FLIGHT_MODE_PERIOD_US) {
        if (crsf_send_flight_mode(flight_mode_string())) {
            last_mode_us = now;
        }
    }

    crsf_tx_pump();
}
