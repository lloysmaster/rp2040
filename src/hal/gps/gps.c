#include "hal/gps/gps.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include <string.h>

#define GPS_UART uart1
#define GPS_BAUD 9600 // O 38400 / 115200 dependiendo de tu módulo (ej. M8N suele ir a 9600 o 115200)
#define GPS_TX_PIN 4
#define GPS_RX_PIN 5

#define GPS_FIX_TIMEOUT_US 3000000u
#define NMEA_MAX_LEN 100
#define NMEA_MAX_FIELDS 20

uint8_t __attribute__((aligned(256))) gps_rx_buffer[256];
static int gps_dma_chan;
static uint8_t gps_read_idx = 0;
static gps_data_t gps_state = {0};

static char nmea_line[NMEA_MAX_LEN];
static uint8_t nmea_len = 0;
static bool nmea_overflow = false;

void gps_init(void) {
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(GPS_UART, false, false);
    uart_set_fifo_enabled(GPS_UART, true);

    gps_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(gps_dma_chan);

    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, 8); // Buffer circular de 256 bytes
    channel_config_set_dreq(&c, DREQ_UART1_RX);

    dma_channel_configure(
        gps_dma_chan,
        &c,
        gps_rx_buffer,
        &uart_get_hw(GPS_UART)->dr,
        0xFFFFFFFF,
        true
    );
}

static uint8_t hex_value(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0xFF;
}

// Valida el checksum XOR de una sentencia NMEA ("$....*hh") y recorta el sufijo
static bool nmea_checksum_ok(char *line, uint8_t len) {
    if (len < 4 || line[0] != '$') {
        return false;
    }

    int star = -1;
    for (int i = len - 1; i > 0; --i) {
        if (line[i] == '*') {
            star = i;
            break;
        }
    }
    if (star < 0 || (star + 2) >= len) {
        return false;
    }

    const uint8_t hi = hex_value(line[star + 1]);
    const uint8_t lo = hex_value(line[star + 2]);
    if (hi == 0xFF || lo == 0xFF) {
        return false;
    }

    uint8_t crc = 0;
    for (int i = 1; i < star; ++i) {
        crc ^= (uint8_t)line[i];
    }

    line[star] = '\0'; // El payload termina antes del '*'
    return crc == (uint8_t)((hi << 4) | lo);
}

// Convierte una cadena decimal a entero escalado (ej. "12.34" con decimals=3 -> 12340)
static int32_t parse_scaled(const char *s, uint8_t decimals) {
    if (s == NULL || *s == '\0') {
        return 0;
    }

    bool negative = false;
    if (*s == '-') {
        negative = true;
        ++s;
    } else if (*s == '+') {
        ++s;
    }

    int32_t value = 0;
    while (*s >= '0' && *s <= '9') {
        value = value * 10 + (*s - '0');
        ++s;
    }

    uint8_t used = 0;
    if (*s == '.') {
        ++s;
        while (used < decimals && *s >= '0' && *s <= '9') {
            value = value * 10 + (*s - '0');
            ++s;
            ++used;
        }
    }
    while (used < decimals) {
        value *= 10;
        ++used;
    }

    return negative ? -value : value;
}

// Convierte "ddmm.mmmm" (o "dddmm.mmmm") + hemisferio a grados * 1e7
static int32_t nmea_coord_to_1e7(const char *coord, const char *hemisphere) {
    if (coord == NULL || *coord == '\0') {
        return 0;
    }

    // ddmm.mmmmm -> minutos * 1e5
    const int32_t raw = parse_scaled(coord, 5); // ddmm.mmmmm sin punto
    const int32_t degrees = raw / 10000000;     // ddmm.mmmmm => dd
    const int32_t minutes_1e5 = raw % 10000000; // mm.mmmmm * 1e5

    int64_t result = (int64_t)degrees * 10000000LL + ((int64_t)minutes_1e5 * 10000000LL) / 6000000LL;

    if (hemisphere != NULL && (*hemisphere == 'S' || *hemisphere == 'W')) {
        result = -result;
    }
    return (int32_t)result;
}

static void parse_gga(char **fields, uint8_t count) {
    if (count < 10) {
        return;
    }

    const int fix_quality = (int)parse_scaled(fields[6], 0);
    gps_state.satellites = (uint8_t)parse_scaled(fields[7], 0);

    if (fix_quality <= 0) {
        gps_state.fix_valid = false;
        return;
    }

    gps_state.latitude_1e7 = nmea_coord_to_1e7(fields[2], fields[3]);
    gps_state.longitude_1e7 = nmea_coord_to_1e7(fields[4], fields[5]);
    gps_state.altitude_m = (int16_t)(parse_scaled(fields[9], 0));
    gps_state.fix_valid = true;
    gps_state.last_fix_time = time_us_32();
}

static void parse_rmc(char **fields, uint8_t count) {
    if (count < 9) {
        return;
    }

    if (fields[2][0] != 'A') { // 'A' = datos válidos, 'V' = aviso (sin fix)
        gps_state.fix_valid = false;
        return;
    }

    gps_state.latitude_1e7 = nmea_coord_to_1e7(fields[3], fields[4]);
    gps_state.longitude_1e7 = nmea_coord_to_1e7(fields[5], fields[6]);

    // Velocidad en nudos -> km/h * 10 (1 nudo = 1.852 km/h)
    const int32_t knots_x10 = parse_scaled(fields[7], 1);
    gps_state.speed_kmh_x10 = (uint16_t)((knots_x10 * 1852) / 1000);

    const int32_t heading_x100 = parse_scaled(fields[8], 2);
    gps_state.heading_x100 = (uint16_t)(heading_x100 % 36000);

    gps_state.fix_valid = true;
    gps_state.last_fix_time = time_us_32();
}

static bool parse_nmea_sentence(char *line, uint8_t len) {
    if (!nmea_checksum_ok(line, len)) {
        return false;
    }

    char *fields[NMEA_MAX_FIELDS];
    uint8_t count = 0;
    fields[count++] = line;
    for (char *p = line; *p != '\0'; ++p) {
        if (*p == ',') {
            *p = '\0';
            if (count < NMEA_MAX_FIELDS) {
                fields[count++] = p + 1;
            }
        }
    }

    // fields[0] es "$GPGGA" / "$GNRMC" / etc: el talker varía según constelación
    const char *type = fields[0] + 3;
    if (strncmp(type, "GGA", 3) == 0) {
        parse_gga(fields, count);
        return true;
    }
    if (strncmp(type, "RMC", 3) == 0) {
        parse_rmc(fields, count);
        return true;
    }
    return false;
}

bool gps_update(void) {
    uint32_t current_dma_write_ptr = (uint32_t)dma_channel_hw_addr(gps_dma_chan)->write_addr;
    uint8_t write_idx = (uint8_t)(current_dma_write_ptr - (uint32_t)gps_rx_buffer);
    bool new_fix = false;

    while (gps_read_idx != write_idx) {
        const char ch = (char)gps_rx_buffer[gps_read_idx++];

        if (ch == '$') {
            nmea_len = 0;
            nmea_overflow = false;
        }

        if (ch == '\r' || ch == '\n') {
            if (nmea_len > 0 && !nmea_overflow) {
                nmea_line[nmea_len] = '\0';
                if (parse_nmea_sentence(nmea_line, nmea_len)) {
                    new_fix = true;
                }
            }
            nmea_len = 0;
            nmea_overflow = false;
            continue;
        }

        if (nmea_len < (NMEA_MAX_LEN - 1)) {
            nmea_line[nmea_len++] = ch;
        } else {
            nmea_overflow = true; // Sentencia demasiado larga: se descarta completa
        }
    }

    if (gps_state.fix_valid && (time_us_32() - gps_state.last_fix_time) > GPS_FIX_TIMEOUT_US) {
        gps_state.fix_valid = false;
    }

    return new_fix;
}

const gps_data_t* gps_get_data(void) {
    return &gps_state;
}
