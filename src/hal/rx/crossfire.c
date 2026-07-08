#include "crossfire.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"

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
}

bool crsf_update(void) {
    bool new_data_ready = false;

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
        
        switch (current_state) {
            case CRSF_STATE_SYNC:
                if (ch == CRSF_SYNC_BYTE) {
                    current_state = CRSF_STATE_LEN;
                }
                break;
                
            case CRSF_STATE_LEN:
                expected_len = ch;
                if (expected_len > 64 || expected_len < 2) {
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
                        // Si el tipo de paquete es de canales RC, decodificamos
                        if (payload_buffer[0] == CRSF_RC_CHANNELS_TYPE && (expected_len - 2) == CRSF_PAYLOAD_SIZE) {
                            decode_rc_channels(&payload_buffer[1]); // Pasar puntero saltando el byte Type
                            new_data_ready = true;
                        }
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