#include "hal/mpu/mpu.h"

void mpu_init_spi() {
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    
    spi_init(SPI_PORT, 1000 * 1000);

    // Habilitar interrupción de datos listos (Data Ready)
mpu_write(0x38, 0x01); // BIT 0: RAW_RDY_EN (Data ready interrupt)
// Opcional: configurar pin INT como active high, push-pull
mpu_write(0x37, 0x00);
}

void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data}; 
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, buf, 2);
    gpio_put(PIN_CS, 1);
}

void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    reg |= 0x80; // Bit de lectura
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &reg, 1);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    gpio_put(PIN_CS, 1);
}