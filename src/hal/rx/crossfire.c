#include "crossfire.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include <string.h>

#define UART_ID uart0
#define BAUD_RATE 420000 
#define UART_TX_PIN 0
#define UART_RX_PIN 1

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_RC_CHANNELS_TYPE 0x16
#define CRSF_PAYLOAD_SIZE 22 // 22 bytes para 16 canales de 11 bits
#define CRSF_FAILSAFE_TIMEOUT_US 200000u

// Búfer circular alineado a 256 bytes (requerimiento del DMA de la RP2040 para wrap_ring)
uint8_t __attribute__((aligned(256))) crsf_rx_buffer[256];
static int dma_chan;
static uint8_t read_idx = 0;

// Cola circular de transmisión de telemetría (vaciada por DMA hacia el UART)
static uint8_t __attribute__((aligned(256))) crsf_tx_buffer[256];
static int dma_tx_chan;
static volatile uint8_t tx_head = 0; // siguiente posición a escribir
static uint8_t tx_tail = 0;          // siguiente posición a transmitir

typedef enum {
    CRSF_STATE_SYNC,
    CRSF_STATE_LEN,
    CRSF_STATE_TYPE,
    CRSF_STATE_PAYLOAD,
    CRSF_STATE_CRC
} crsf_state_t;

static crsf_state_t current_state = CRSF_STATE_SYNC;
static uint8_t payload_buffer[64];
static uint8_t payload_idx = 0;
static uint8_t expected_len = 0;

static crsf_data_t crsf_state;
static crsf_debug_t crsf_debug;
static uint8_t debug_byte_idx = 0;

static void debug_push_byte(uint8_t ch) {
    crsf_debug.bytes_rx++;
    crsf_debug.last_bytes[debug_byte_idx] = ch;
    debug_byte_idx = (uint8_t)((debug_byte_idx + 1) % sizeof(crsf_debug.last_bytes));
}

static void debug_reset(void) {
    crsf_debug = (crsf_debug_t){0};
    debug_byte_idx = 0;
}

void crsf_get_debug(crsf_debug_t *out) {
    if (out == NULL) {
        return;
    }
    *out = crsf_debug;
    out->rx_pin_level = gpio_get(UART_RX_PIN);
    // Reordenar el anillo para que quede del más antiguo al más nuevo
    for (uint8_t i = 0; i < sizeof(out->last_bytes); ++i) {
        out->last_bytes[i] = crsf_debug.last_bytes[(debug_byte_idx + i) % sizeof(crsf_debug.last_bytes)];
    }
}

// Cálculo estándar de CRC8 para protocolo CRSF (Polinomio 0xD5)
static uint8_t calc_crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Desempaqueta los 22 bytes en 16 canales de 11 bits
static void decode_rc_channels(uint8_t *payload) {
    crsf_state.channels[0]  = (payload[0]       | payload[1] << 8) & 0x07FF;
    crsf_state.channels[1]  = (payload[1] >> 3  | payload[2] << 5) & 0x07FF;
    crsf_state.channels[2]  = (payload[2] >> 6  | payload[3] << 2 | payload[4] << 10) & 0x07FF;
    crsf_state.channels[3]  = (payload[4] >> 1  | payload[5] << 7) & 0x07FF;
    crsf_state.channels[4]  = (payload[5] >> 4  | payload[6] << 4) & 0x07FF;
    crsf_state.channels[5]  = (payload[6] >> 7  | payload[7] << 1 | payload[8] << 9) & 0x07FF;
    crsf_state.channels[6]  = (payload[8] >> 2  | payload[9] << 6) & 0x07FF;
    crsf_state.channels[7]  = (payload[9] >> 5  | payload[10] << 3) & 0x07FF;
    crsf_state.channels[8]  = (payload[11]      | payload[12] << 8) & 0x07FF;
    crsf_state.channels[9]  = (payload[12] >> 3 | payload[13] << 5) & 0x07FF;
    crsf_state.channels[10] = (payload[13] >> 6 | payload[14] << 2 | payload[15] << 10) & 0x07FF;
    crsf_state.channels[11] = (payload[15] >> 1 | payload[16] << 7) & 0x07FF;
    crsf_state.channels[12] = (payload[16] >> 4 | payload[17] << 4) & 0x07FF;
    crsf_state.channels[13] = (payload[17] >> 7 | payload[18] << 1 | payload[19] << 9) & 0x07FF;
    crsf_state.channels[14] = (payload[19] >> 2 | payload[20] << 6) & 0x07FF;
    crsf_state.channels[15] = (payload[20] >> 5 | payload[21] << 3) & 0x07FF;
    
    crsf_state.is_connected = true;
    crsf_state.last_packet_time = time_us_32();
}

void crsf_init(void) {
    // Configuración UART
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true); // Necesario para evitar overflow rápido

    // Configuración DMA
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false); // Siempre leemos del mismo registro (UART_DR)
    channel_config_set_write_increment(&c, true); // Avanzamos en nuestro búfer
    channel_config_set_ring(&c, true, 8);         // Búfer circular de 2^8 = 256 bytes
    channel_config_set_dreq(&c, DREQ_UART0_RX);   // Disparar por UART RX

    dma_channel_configure(
        dma_chan,
        &c,
        crsf_rx_buffer,               // Destino
        &uart_get_hw(UART_ID)->dr,    // Origen
        0xFFFFFFFF,                   // Transferencias (infinito en la práctica)
        true                          // Iniciar ya
    );

    // Canal DMA de transmisión: lee del búfer circular y escribe en el UART
    dma_tx_chan = dma_claim_unused_channel(true);
    dma_channel_config tc = dma_channel_get_default_config(dma_tx_chan);

    channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
    channel_config_set_read_increment(&tc, true);
    channel_config_set_write_increment(&tc, false);
    channel_config_set_ring(&tc, false, 8);       // El puntero de lectura envuelve cada 256 bytes
    channel_config_set_dreq(&tc, DREQ_UART0_TX);

    dma_channel_configure(
        dma_tx_chan,
        &tc,
        &uart_get_hw(UART_ID)->dr,    // Destino
        crsf_tx_buffer,               // Origen (se reajusta en cada envío)
        0,
        false                         // No arrancar todavía
    );
}

bool crsf_self_test(void) {
    // Loopback interno del PL011: TX se conecta a RX dentro del chip, así que
    // esta prueba valida UART + DMA + parser sin depender de ningún cable.
    hw_set_bits(&uart_get_hw(UART_ID)->cr, UART_UARTCR_LBE_BITS);

    // Trama RC válida con los 16 canales en el centro (992 = 0x3E0)
    uint8_t frame[26];
    frame[0] = CRSF_SYNC_BYTE;
    frame[1] = CRSF_PAYLOAD_SIZE + 2;
    frame[2] = CRSF_RC_CHANNELS_TYPE;

    uint32_t bits = 0;
    uint8_t bits_available = 0;
    uint8_t idx = 3;
    for (uint8_t i = 0; i < CRSF_MAX_CHANNELS; ++i) {
        bits |= (uint32_t)992u << bits_available;
        bits_available += 11;
        while (bits_available >= 8) {
            frame[idx++] = (uint8_t)(bits & 0xFF);
            bits >>= 8;
            bits_available -= 8;
        }
    }
    frame[25] = calc_crc8(&frame[2], 23);

    uart_write_blocking(UART_ID, frame, sizeof(frame));
    sleep_ms(5); // Tiempo para que el DMA vuelque los bytes al búfer circular

    const bool ok = crsf_update();

    hw_clear_bits(&uart_get_hw(UART_ID)->cr, UART_UARTCR_LBE_BITS);

    // La prueba no debe dejar rastros en el estado del receptor
    current_state = CRSF_STATE_SYNC;
    payload_idx = 0;
    crsf_state.is_connected = false;
    for (uint8_t i = 0; i < CRSF_MAX_CHANNELS; ++i) {
        crsf_state.channels[i] = 0;
    }
    debug_reset();

    return ok;
}

// --- Telemetría: construcción y encolado de tramas --------------------------

// Posición realmente consumida: mientras el DMA transmite, sus bytes pendientes
// no pueden sobrescribirse, así que se consulta su puntero de lectura.
static uint8_t tx_consumed_idx(void) {
    if (dma_channel_is_busy(dma_tx_chan)) {
        const uint32_t read_addr = (uint32_t)dma_channel_hw_addr(dma_tx_chan)->read_addr;
        return (uint8_t)(read_addr - (uint32_t)crsf_tx_buffer);
    }
    return tx_tail;
}

static uint8_t tx_free_space(void) {
    // Se deja 1 byte libre para poder distinguir cola llena de cola vacía
    return (uint8_t)(255u - (uint8_t)(tx_head - tx_consumed_idx()));
}

// Encola una trama CRSF completa: [dirección][largo][tipo][payload][crc]
static bool crsf_enqueue_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len) {
    const uint8_t frame_len = (uint8_t)(payload_len + 4); // dir + len + tipo + payload + crc
    if (payload_len > 60 || tx_free_space() < frame_len) {
        return false; // Telemetría es best-effort: si no entra, se descarta
    }

    uint8_t crc_input[62];
    crc_input[0] = type;
    memcpy(&crc_input[1], payload, payload_len);
    const uint8_t crc = calc_crc8(crc_input, (uint8_t)(payload_len + 1));

    uint8_t head = tx_head;
    crsf_tx_buffer[head++] = CRSF_ADDR_FLIGHT_CONTROLLER;
    crsf_tx_buffer[head++] = (uint8_t)(payload_len + 2); // tipo + payload + crc
    crsf_tx_buffer[head++] = type;
    for (uint8_t i = 0; i < payload_len; ++i) {
        crsf_tx_buffer[head++] = payload[i];
    }
    crsf_tx_buffer[head++] = crc;
    tx_head = head;

    crsf_tx_pump();
    return true;
}

void crsf_tx_pump(void) {
    if (dma_channel_is_busy(dma_tx_chan)) {
        return;
    }

    const uint8_t pending = (uint8_t)(tx_head - tx_tail);
    if (pending == 0) {
        return;
    }

    dma_channel_set_read_addr(dma_tx_chan, &crsf_tx_buffer[tx_tail], false);
    dma_channel_set_trans_count(dma_tx_chan, pending, true);
    tx_tail = (uint8_t)(tx_tail + pending);
}

bool crsf_send_battery(uint16_t voltage_dv, uint16_t current_da, uint32_t capacity_mah, uint8_t remaining_pct) {
    uint8_t payload[8];
    payload[0] = (uint8_t)(voltage_dv >> 8);
    payload[1] = (uint8_t)(voltage_dv & 0xFF);
    payload[2] = (uint8_t)(current_da >> 8);
    payload[3] = (uint8_t)(current_da & 0xFF);
    payload[4] = (uint8_t)((capacity_mah >> 16) & 0xFF);
    payload[5] = (uint8_t)((capacity_mah >> 8) & 0xFF);
    payload[6] = (uint8_t)(capacity_mah & 0xFF);
    payload[7] = remaining_pct > 100u ? 100u : remaining_pct;
    return crsf_enqueue_frame(CRSF_FRAME_BATTERY_SENSOR, payload, sizeof(payload));
}

bool crsf_send_gps(int32_t latitude_1e7, int32_t longitude_1e7, uint16_t groundspeed_kmh_x10,
                   uint16_t heading_deg_x100, int16_t altitude_m, uint8_t satellites) {
    // El campo de altitud viaja con un offset de +1000 m (0 => -1000 m)
    int32_t altitude_field = (int32_t)altitude_m + 1000;
    if (altitude_field < 0) altitude_field = 0;
    if (altitude_field > 0xFFFF) altitude_field = 0xFFFF;

    uint8_t payload[15];
    payload[0]  = (uint8_t)((latitude_1e7 >> 24) & 0xFF);
    payload[1]  = (uint8_t)((latitude_1e7 >> 16) & 0xFF);
    payload[2]  = (uint8_t)((latitude_1e7 >> 8) & 0xFF);
    payload[3]  = (uint8_t)(latitude_1e7 & 0xFF);
    payload[4]  = (uint8_t)((longitude_1e7 >> 24) & 0xFF);
    payload[5]  = (uint8_t)((longitude_1e7 >> 16) & 0xFF);
    payload[6]  = (uint8_t)((longitude_1e7 >> 8) & 0xFF);
    payload[7]  = (uint8_t)(longitude_1e7 & 0xFF);
    payload[8]  = (uint8_t)(groundspeed_kmh_x10 >> 8);
    payload[9]  = (uint8_t)(groundspeed_kmh_x10 & 0xFF);
    payload[10] = (uint8_t)(heading_deg_x100 >> 8);
    payload[11] = (uint8_t)(heading_deg_x100 & 0xFF);
    payload[12] = (uint8_t)(((uint32_t)altitude_field >> 8) & 0xFF);
    payload[13] = (uint8_t)((uint32_t)altitude_field & 0xFF);
    payload[14] = satellites;
    return crsf_enqueue_frame(CRSF_FRAME_GPS, payload, sizeof(payload));
}

static int16_t rad_to_crsf(float radians) {
    float scaled = radians * 10000.0f;
    if (scaled > 32767.0f) scaled = 32767.0f;
    if (scaled < -32768.0f) scaled = -32768.0f;
    return (int16_t)scaled;
}

bool crsf_send_attitude(float pitch_rad, float roll_rad, float yaw_rad) {
    const int16_t pitch = rad_to_crsf(pitch_rad);
    const int16_t roll  = rad_to_crsf(roll_rad);
    const int16_t yaw   = rad_to_crsf(yaw_rad);

    uint8_t payload[6];
    payload[0] = (uint8_t)(((uint16_t)pitch >> 8) & 0xFF);
    payload[1] = (uint8_t)((uint16_t)pitch & 0xFF);
    payload[2] = (uint8_t)(((uint16_t)roll >> 8) & 0xFF);
    payload[3] = (uint8_t)((uint16_t)roll & 0xFF);
    payload[4] = (uint8_t)(((uint16_t)yaw >> 8) & 0xFF);
    payload[5] = (uint8_t)((uint16_t)yaw & 0xFF);
    return crsf_enqueue_frame(CRSF_FRAME_ATTITUDE, payload, sizeof(payload));
}

bool crsf_send_flight_mode(const char *mode) {
    if (mode == NULL) {
        return false;
    }

    uint8_t payload[16];
    uint8_t len = 0;
    while (len < (sizeof(payload) - 1) && mode[len] != '\0') {
        payload[len] = (uint8_t)mode[len];
        ++len;
    }
    payload[len++] = '\0'; // La cadena viaja terminada en nulo
    return crsf_enqueue_frame(CRSF_FRAME_FLIGHT_MODE, payload, len);
}

bool crsf_update(void) {
    bool new_data_ready = false;

    // Errores de línea (overrun, framing, paridad, break): se leen y limpian.
    // Un valor creciente delata baudios mal ajustados o ruido en el cable.
    const uint32_t rsr = uart_get_hw(UART_ID)->rsr & 0x0Fu;
    if (rsr != 0) {
        crsf_debug.uart_errors |= rsr;
        uart_get_hw(UART_ID)->rsr = 0;
    }

    if (crsf_state.is_connected && (time_us_32() - crsf_state.last_packet_time) > CRSF_FAILSAFE_TIMEOUT_US) {
        crsf_state.is_connected = false;
        for (int i = 0; i < CRSF_MAX_CHANNELS; ++i) {
            crsf_state.channels[i] = 0;
        }
        crsf_state.channels[2] = 0;
        crsf_state.channels[4] = 0;
    }
    
    // Calcular dónde está escribiendo el DMA ahora mismo (dirección relativa al inicio del búfer)
    uint32_t current_dma_write_ptr = (uint32_t)dma_channel_hw_addr(dma_chan)->write_addr;
    uint8_t write_idx = (uint8_t)(current_dma_write_ptr - (uint32_t)crsf_rx_buffer);

    // Procesar mientras haya datos nuevos en el ring buffer
    while (read_idx != write_idx) {
        uint8_t ch = crsf_rx_buffer[read_idx++]; // read_idx hace wrap automático gracias a ser uint8_t
        debug_push_byte(ch);
        
        switch (current_state) {
            case CRSF_STATE_SYNC:
                if (ch == CRSF_SYNC_BYTE) {
                    current_state = CRSF_STATE_LEN;
                }
                break;
                
            case CRSF_STATE_LEN:
                expected_len = ch;
                if (expected_len > 64 || expected_len < 2) {
                    crsf_debug.frames_len_err++;
                    current_state = CRSF_STATE_SYNC; // Longitud inválida, reiniciar
                } else {
                    payload_idx = 0;
                    current_state = CRSF_STATE_TYPE;
                }
                break;
                
            case CRSF_STATE_TYPE:
                payload_buffer[payload_idx++] = ch;
                current_state = CRSF_STATE_PAYLOAD;
                break;
                
            case CRSF_STATE_PAYLOAD:
                payload_buffer[payload_idx++] = ch;
                // expected_len incluye el byte de Type, los de Payload y 1 byte de CRC
                if (payload_idx == expected_len - 1) { 
                    current_state = CRSF_STATE_CRC;
                }
                break;
                
            case CRSF_STATE_CRC:
                {
                    uint8_t received_crc = ch;
                    // El CRC se calcula sobre el Type y el Payload
                    uint8_t calculated_crc = calc_crc8(payload_buffer, expected_len - 1);
                    
                    if (received_crc == calculated_crc) {
                        crsf_debug.frames_ok++;
                        // Si el tipo de paquete es de canales RC, decodificamos
                        if (payload_buffer[0] == CRSF_RC_CHANNELS_TYPE && (expected_len - 2) == CRSF_PAYLOAD_SIZE) {
                            decode_rc_channels(&payload_buffer[1]); // Pasar puntero saltando el byte Type
                            new_data_ready = true;
                        }
                    } else {
                        crsf_debug.frames_crc_err++;
                    }
                    current_state = CRSF_STATE_SYNC; // Reiniciar siempre al final de la trama
                }
                break;
        }
    }
    
    return new_data_ready;
}

const crsf_data_t* crsf_get_data(void) {
    return &crsf_state;
}