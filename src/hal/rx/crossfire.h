#ifndef CROSSFIRE_H
#define CROSSFIRE_H

#include <stdint.h>
#include <stdbool.h>

#define CRSF_MAX_CHANNELS 16

// Direcciones y tipos de trama CRSF utilizados en la telemetría
#define CRSF_ADDR_FLIGHT_CONTROLLER 0xC8
#define CRSF_FRAME_GPS              0x02
#define CRSF_FRAME_BATTERY_SENSOR   0x08
#define CRSF_FRAME_LINK_STATISTICS  0x14
#define CRSF_FRAME_ATTITUDE         0x1E
#define CRSF_FRAME_FLIGHT_MODE      0x21

// Estructura para almacenar los datos del receptor
typedef struct {
    uint16_t channels[CRSF_MAX_CHANNELS];
    bool is_connected;
    uint32_t last_packet_time; // Para failsafe
} crsf_data_t;

// Contadores de diagnóstico del enlace serie (útiles para depurar el cableado)
typedef struct {
    uint32_t bytes_rx;      // bytes crudos leídos del UART por el DMA
    uint32_t frames_ok;     // tramas con CRC válido
    uint32_t frames_crc_err;// tramas descartadas por CRC
    uint32_t frames_len_err;// bytes de largo fuera de rango (desincronización)
    uint32_t uart_errors;   // flags acumulados del UART (overrun/break/paridad/framing)
    uint8_t last_bytes[8];  // últimos bytes recibidos, del más antiguo al más nuevo
    bool rx_pin_level;      // nivel actual del pin RX: en reposo debe ser alto
} crsf_debug_t;

// Inicializa el hardware (UART + DMA RX/TX) para CRSF
void crsf_init(void);

// Copia los contadores de diagnóstico acumulados desde el arranque
void crsf_get_debug(crsf_debug_t *out);

// Autotest por loopback interno del UART: no requiere cableado alguno.
// Retorna true si una trama de canales generada por el propio FC vuelve a
// entrar y se decodifica; false apunta a un problema de UART/DMA/parser.
// Deja los contadores de diagnóstico y el estado del receptor en cero.
bool crsf_self_test(void);

// Función principal que debe ser llamada frecuentemente para procesar el búfer DMA
// Retorna true si se decodificó un paquete válido nuevo
bool crsf_update(void);

// Retorna un puntero a los datos de los canales
const crsf_data_t* crsf_get_data(void);

// --- Telemetría (FC -> receptor/emisor) -------------------------------------
// Todas las funciones encolan la trama y retornan de inmediato: el envío por el
// UART lo realiza el DMA, por lo que no bloquean el lazo de control.
// Retornan false si la cola de transmisión está llena (la trama se descarta).

bool crsf_send_battery(uint16_t voltage_dv,      // décimas de voltio
                       uint16_t current_da,      // décimas de amperio
                       uint32_t capacity_mah,    // mAh consumidos (24 bits)
                       uint8_t remaining_pct);   // 0..100 %

bool crsf_send_gps(int32_t latitude_1e7,
                   int32_t longitude_1e7,
                   uint16_t groundspeed_kmh_x10,
                   uint16_t heading_deg_x100,
                   int16_t altitude_m,           // metros sobre el nivel del mar
                   uint8_t satellites);

bool crsf_send_attitude(float pitch_rad, float roll_rad, float yaw_rad);

bool crsf_send_flight_mode(const char *mode);

// Empuja la cola de transmisión hacia el UART. Llamar en cada iteración del lazo.
void crsf_tx_pump(void);

#endif // CROSSFIRE_H
