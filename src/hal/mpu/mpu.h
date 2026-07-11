#ifndef MPU_H
#define MPU_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "math/fixed_point.h"

// Estructura de configuración
typedef struct {
    spi_inst_t *spi;
    uint8_t pin_cs;
    uint8_t pin_sck;
    uint8_t pin_mosi;
    uint8_t pin_miso;
    uint8_t pin_drdy;
    uint16_t gyro_sensitivity_lsb_per_dps;
} mpu_config_t;

/**
 * @brief Inicializa el hardware SPI, reserva los canales DMA y configura el MPU6500.
 * @param config Puntero a la estructura de configuración.
 */
void mpu_init(const mpu_config_t *config);

/**
 * @brief Lee los 3 ejes del giroscopio usando DMA y los convierte a Q16_16.
 * @param output Arreglo de 3 elementos q16_16.
 */
void mpu_read_gyro_fixed(q16_16 *output);

/**
 * @brief Configura la interrupción Data Ready en el sensor.
 */
void mpu_enable_drdy(void);

#endif // MPU_H
