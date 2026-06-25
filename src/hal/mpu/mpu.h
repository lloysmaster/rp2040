#ifndef MPU_H
#define MPU_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "math/fixed_point.h"

// Direcciones de registros clave (MPU6500)
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_INT_PIN_CFG   0x37
#define MPU_REG_INT_ENABLE    0x38
#define MPU_REG_PWR_MGMT_1    0x6B
#define MPU_REG_WHO_AM_I      0x75

// Estructura de configuración
typedef struct {
    spi_inst_t *spi;
    uint8_t pin_cs;
    uint8_t pin_sck;
    uint8_t pin_mosi;
    uint8_t pin_miso;
    uint8_t pin_drdy;
} mpu_config_t;

/**
 * @brief Inicializa el hardware SPI y configura el MPU6500.
 * @param config Puntero a la estructura de configuración.
 */
void mpu_init(mpu_config_t *config);

/**
 * @brief Escribe un byte en un registro específico.
 */
void mpu_write(uint8_t reg, uint8_t data);

/**
 * @brief Lee varios bytes desde un registro.
 */
void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief Lee los 3 ejes del acelerómetro y los convierte a Q16_16.
 * @param output Arreglo de 3 elementos q16_16.
 */
void mpu_read_accel_fixed(q16_16 *output);
void mpu_read_gyro_fixed(q16_16 *output);

/**
 * @brief Función para configurar la interrupción Data Ready en el sensor.
 */
void mpu_enable_drdy(void);

#endif // MPU_H