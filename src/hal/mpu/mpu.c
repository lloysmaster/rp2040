#include "mpu.h"
#include "hardware/gpio.h"
#include "math/fixed_point.h"

static mpu_config_t *g_cfg;

void mpu_init(mpu_config_t *config) {
    g_cfg = config;

    // Configurar SPI
    spi_init(g_cfg->spi, 1000000); // 1MHz
    gpio_set_function(g_cfg->pin_sck, GPIO_FUNC_SPI);
    gpio_set_function(g_cfg->pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(g_cfg->pin_miso, GPIO_FUNC_SPI);

    // Configurar CS
    gpio_init(g_cfg->pin_cs);
    gpio_set_dir(g_cfg->pin_cs, GPIO_OUT);
    gpio_put(g_cfg->pin_cs, 1);

    // Configurar DRDY como entrada con pull-down
    gpio_init(g_cfg->pin_drdy);
    gpio_set_dir(g_cfg->pin_drdy, GPIO_IN);
    gpio_pull_down(g_cfg->pin_drdy);
}

void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg & 0x7F, data}; // MSB 0 para escritura
    gpio_put(g_cfg->pin_cs, 0);
    spi_write_blocking(g_cfg->spi, buf, 2);
    gpio_put(g_cfg->pin_cs, 1);
}

void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t addr = reg | 0x80; // MSB 1 para lectura
    gpio_put(g_cfg->pin_cs, 0);
    spi_write_blocking(g_cfg->spi, &addr, 1);
    spi_read_blocking(g_cfg->spi, 0, buf, len);
    gpio_put(g_cfg->pin_cs, 1);
}

// Ejemplo de uso de Fixed Point con datos del sensor
void mpu_read_accel_fixed(q16_16 *output) {
    uint8_t raw[6];
    mpu_read(0x3B, raw, 6); // Registros ACCEL_XOUT_H a ZOUT_L

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];

    // Suponiendo escala de 2g (16384 LSB/g)
    output[0] = q_div(TO_Q16(ax), TO_Q16(16384)); // Ajustar factor según sensibilidad
    output[1] = q_div(TO_Q16(ay), TO_Q16(16384));
    output[2] = q_div(TO_Q16(az), TO_Q16(16384));
}
void mpu_read_gyro_fixed(q16_16 *output) {
    uint8_t raw[6];
    // Los registros del giroscopio comienzan en 0x43
    mpu_read(0x43, raw, 6); 

    int16_t gx = (raw[0] << 8) | raw[1];
    int16_t gy = (raw[2] << 8) | raw[3];
    int16_t gz = (raw[4] << 8) | raw[5];

    // Ajusta el divisor según la escala que configures en el giroscopio
    // Por defecto suele ser 131 LSB/(°/s)
    output[0] = q_div(TO_Q16(gx), TO_Q16(131));
    output[1] = q_div(TO_Q16(gy), TO_Q16(131));
    output[2] = q_div(TO_Q16(gz), TO_Q16(131));
}
void mpu_enable_drdy(void) {
    // 1. Despertar el sensor (sacarlo de modo sleep)
    mpu_write(MPU_REG_PWR_MGMT_1, 0x01); // Reloj auto-selección
    
    // 2. Habilitar la interrupción Data Ready en el registro 0x38
    mpu_write(MPU_REG_INT_ENABLE, 0x01); 
}