#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hal/mpu/mpu.h"
#include "math/fixed_point.h"
#include <stdio.h>

volatile bool data_ready = false;

// Callback de la interrupción externa del GPIO
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 14) { // Pin DRDY
        data_ready = true;
    }
}

int main() {
    stdio_init_all();
    
    mpu_config_t cfg = {
        .spi = spi1, 
        .pin_cs = 13, 
        .pin_sck = 10,
        .pin_mosi = 11, 
        .pin_miso = 12, 
        .pin_drdy = 14
    };
    
    // Inicializa hardware base y canales DMA vinculados al periférico
    mpu_init(&cfg);
    mpu_enable_drdy();
    
    // Habilitar interrupción física en el flanco de subida del pin DRDY
    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    while (1) {
        if (data_ready) {
            data_ready = false;
            
            q16_16 accel_data[3];
            mpu_read_accel_fixed(accel_data); // Ejecutado internamente bajo DMA
            
            q16_16 gyro_data[3];
            mpu_read_gyro_fixed(gyro_data);   // Ejecutado internamente bajo DMA
            
            // Impresión formateada extrayendo parte entera y decimal del Q16.16
            printf("X: %ld.%04ld | Y: %ld.%04ld | Z: %ld.%04ld\n", 
                    accel_data[0] >> 16, (accel_data[0] & 0xFFFF) * 10000 / 65536,
                    accel_data[1] >> 16, (accel_data[1] & 0xFFFF) * 10000 / 65536,
                    accel_data[2] >> 16, (accel_data[2] & 0xFFFF) * 10000 / 65536);
                    
            printf("GX: %ld.%04ld | GY: %ld.%04ld | GZ: %ld.%04ld\n",
                    gyro_data[0] >> 16, (gyro_data[0] & 0xFFFF) * 10000 / 65536,
                    gyro_data[1] >> 16, (gyro_data[1] & 0xFFFF) * 10000 / 65536,
                    gyro_data[2] >> 16, (gyro_data[2] & 0xFFFF) * 10000 / 65536);
        }
    }
}