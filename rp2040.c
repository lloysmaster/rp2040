#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hal/mpu/mpu.h"

// Asegúrate de que este pin coincida con tu conexión física
#define PIN_DRDY 14

typedef int32_t q16_16;
#define GYRO_SCALE_FACTOR 500

volatile bool data_ready = false;

// Callback de la interrupción
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == PIN_DRDY) {
        data_ready = true;
    }
}

static inline q16_16 raw_to_q16(int16_t raw) {
    return (q16_16)((int32_t)raw * GYRO_SCALE_FACTOR);
}

int main() {
    stdio_init_all();
    mpu_init_spi();

    // 1. Configurar Pin de interrupción
    gpio_init(PIN_DRDY);
    gpio_set_dir(PIN_DRDY, GPIO_IN);
    gpio_pull_up(PIN_DRDY);
    gpio_set_irq_enabled_with_callback(PIN_DRDY, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // 2. Configuración DMA
    int dma_tx = dma_claim_unused_channel(true);
    int dma_rx = dma_claim_unused_channel(true);

    dma_channel_config c_tx = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8);
    channel_config_set_dreq(&c_tx, spi_get_dreq(SPI_PORT, true));

    dma_channel_config c_rx = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8);
    channel_config_set_dreq(&c_rx, spi_get_dreq(SPI_PORT, false));
    channel_config_set_read_increment(&c_rx, false);
    channel_config_set_write_increment(&c_rx, true);

    uint8_t tx_buf[7] = {0x43 | 0x80, 0, 0, 0, 0, 0, 0}; 
    uint8_t rx_buf[7] = {0};

    printf("Esperando datos del MPU...\n");

    while (true) {
        // Solo entramos si el sensor disparó la interrupción
        if (data_ready) {
            data_ready = false; // Resetear bandera inmediatamente

            gpio_put(PIN_CS, 0);
            dma_channel_configure(dma_tx, &c_tx, &spi_get_hw(SPI_PORT)->dr, tx_buf, 7, true);
            dma_channel_configure(dma_rx, &c_rx, rx_buf, &spi_get_hw(SPI_PORT)->dr, 7, true);

            dma_channel_wait_for_finish_blocking(dma_rx);
            while (spi_is_busy(SPI_PORT));
            gpio_put(PIN_CS, 1);

            int16_t gx = (rx_buf[1] << 8) | rx_buf[2];
            int16_t gy = (rx_buf[3] << 8) | rx_buf[4];
            int16_t gz = (rx_buf[5] << 8) | rx_buf[6];

            printf("Gyro X: %d | Y: %d | Z: %d\n", (raw_to_q16(gx) >> 16), 
                                                 (raw_to_q16(gy) >> 16), 
                                                 (raw_to_q16(gz) >> 16));
        }
        // Aquí podrías añadir un __wfi() (wait for interrupt) para ahorrar energía
    }
}