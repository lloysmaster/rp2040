#include "hal/gps/gps.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/dma.h"
#include <string.h>
#include <stdio.h>

#define GPS_UART uart1
#define GPS_BAUD 9600 // O 38400 / 115200 dependiendo de tu módulo (ej. M8N suele ir a 9600 o 115200)
#define GPS_TX_PIN 4
#define GPS_RX_PIN 5

uint8_t __attribute__((aligned(256))) gps_rx_buffer[256];
static int gps_dma_chan;
static uint8_t gps_read_idx = 0;
static gps_data_t gps_state = {0};

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

bool gps_update(void) {
    uint32_t current_dma_write_ptr = (uint32_t)dma_channel_hw_addr(gps_dma_chan)->write_addr;
    uint8_t write_idx = (uint8_t)(current_dma_write_ptr - (uint32_t)gps_rx_buffer);
    bool new_fix = false;

    // Procesamiento básico de bytes (ejemplo de captura de trama NMEA simplificada)
    while (gps_read_idx != write_idx) {
        uint8_t ch = gps_rx_buffer[gps_read_idx++];
        
        // Aquí implementarías tu parser NMEA (ej. buscar '$GPRMC' o '$GNGGA' y extraer lat/lon)
        // Para fines prácticos del ejemplo, simulamos que cuando llega un '$', evaluamos la línea completa o acumulamos.
        if (ch == '\n') {
            // Parsear sentencia acumulada...
            // gps_state.fix_valid = true;
            // gps_state.last_packet_time = time_us_32();
            // new_fix = true;
        }
    }
    return new_fix;
}

const gps_data_t* gps_get_data(void) {
    return &gps_state;
}