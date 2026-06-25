#ifndef MPU_H
#define MPU_H

#include "pico/stdlib.h"
#include "hardware/spi.h"

// Pines
#define SPI_PORT    spi1
#define PIN_MISO    12
#define PIN_CS      13
#define PIN_SCK     10
#define PIN_MOSI    11
#define PIN_DRDY    14

// Registros
#define MPU_WHO_AM_I    0x75
#define MPU_PWR_MGMT_1  0x6B
#define MPU_ACCEL_XOUT_H 0x3B

// Prototipos
void mpu_init_spi();
void mpu_write(uint8_t reg, uint8_t data);
void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len);

#endif
