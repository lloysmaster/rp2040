#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Asegúrate de que las rutas a los headers coincidan con la estructura de tus carpetas
#include "hal/mpu/mpu.h"
#include "hal/rx/crossfire.h" 
#include "math/fixed_point.h"

volatile bool data_ready = false;

// Callback de la interrupción externa del GPIO (MPU DRDY)
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 14) { // Pin DRDY
        data_ready = true;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Pequeña espera para abrir la consola serial

    printf("Pico FC iniciada. Configurando DMA para MPU y CRSF...\n");

    // --- Inicialización del sensor MPU6500 ---
    mpu_config_t cfg = {
        .spi = spi1, 
        .pin_cs = 13, 
        .pin_sck = 10,
        .pin_mosi = 11, 
        .pin_miso = 12, 
        .pin_drdy = 14
    };
    
    mpu_init(&cfg);
    mpu_enable_drdy();
    
    // Habilitar interrupción física en el flanco de subida del pin DRDY
    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // --- Inicialización del receptor Crossfire ---
    crsf_init();

    printf("Sistema listo. Escuchando datos...\n");

    while (true) {
        // 1. Procesar datos del receptor Crossfire (No bloqueante)
        if (crsf_update()) {
            const crsf_data_t* rc_data = crsf_get_data();
            
            // CRSF entrega valores entre ~172 y ~1811 (centro en 992)
            printf("[CRSF] CH1: %d | CH2: %d | CH3: %d | CH4: %d\n", 
                   rc_data->channels[0], 
                   rc_data->channels[1], 
                   rc_data->channels[2], 
                   rc_data->channels[3]);
        }
        
        // 2. Procesar datos del MPU (Acelerómetro y Giroscopio)
        if (data_ready) {
            data_ready = false;
            
            q16_16 accel_data[3];
            mpu_read_accel_fixed(accel_data); // Ejecutado internamente bajo DMA
            
            q16_16 gyro_data[3];
            mpu_read_gyro_fixed(gyro_data);   // Ejecutado internamente bajo DMA
            
            // Impresión formateada extrayendo parte entera y decimal del Q16.16
            printf("[MPU] X: %ld.%04ld | Y: %ld.%04ld | Z: %ld.%04ld\n", 
                    accel_data[0] >> 16, (accel_data[0] & 0xFFFF) * 10000 / 65536,
                    accel_data[1] >> 16, (accel_data[1] & 0xFFFF) * 10000 / 65536,
                    accel_data[2] >> 16, (accel_data[2] & 0xFFFF) * 10000 / 65536);
                    
            printf("[MPU] GX: %ld.%04ld | GY: %ld.%04ld | GZ: %ld.%04ld\n",
                    gyro_data[0] >> 16, (gyro_data[0] & 0xFFFF) * 10000 / 65536,
                    gyro_data[1] >> 16, (gyro_data[1] & 0xFFFF) * 10000 / 65536,
                    gyro_data[2] >> 16, (gyro_data[2] & 0xFFFF) * 10000 / 65536);
        }
    }
    
    return 0;
}