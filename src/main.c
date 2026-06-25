#include <stdio.h>
#include "pico/stdlib.h"
#include "hal/mpu/mpu.h"
#include "math/fixed_point.h"

volatile bool data_ready = false;

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 14) { // Pin DRDY
        data_ready = true;
    }
}

int main() {
    stdio_init_all();
    
    mpu_config_t cfg = {
        .spi = spi1, .pin_cs = 13, .pin_sck = 10,
        .pin_mosi = 11, .pin_miso = 12, .pin_drdy = 14
    };
    
    mpu_init(&cfg);
    mpu_enable_drdy();
    
    // Habilitar interrupción en flanco de subida
    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    while (1) {
        // En tu main.c, dentro del while(1)
if (data_ready) {
    data_ready = false;
    q16_16 accel_data[3];
    mpu_read_accel_fixed(accel_data);
    q16_16 gyro_data[3];
    mpu_read_gyro_fixed(gyro_data);
    
    // Para visualizar: convertimos el valor Q16.16 a float (o entero)
    // Usaremos un casting simple para ver el valor en la terminal
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