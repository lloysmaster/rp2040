#include "mpu.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "math/fixed_point.h"

static mpu_config_t *g_cfg;

// Canales DMA estáticos para el módulo MPU
static int dma_tx_chan = -1;
static int dma_rx_chan = -1;
static uint8_t dma_dummy_byte = 0x00;

static uint8_t mpu_gyro_fs_bits_from_sensitivity(uint16_t sensitivity_lsb_per_dps) {
    if (sensitivity_lsb_per_dps >= 131) {
        return 0x00; // ±250 °/s -> 131 LSB/(°/s)
    }
    if (sensitivity_lsb_per_dps >= 65) {
        return 0x08; // ±500 °/s -> 65.5 LSB/(°/s)
    }
    if (sensitivity_lsb_per_dps >= 32) {
        return 0x10; // ±1000 °/s -> 32.8 LSB/(°/s)
    }
    return 0x18; // ±2000 °/s -> 16.4 LSB/(°/s)
}

void mpu_init(mpu_config_t *config) {
    g_cfg = config;

    // Configurar SPI a 1MHz para inicialización segura
    spi_init(g_cfg->spi, 1000000); 
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

    // Solicitar canales DMA libres del sistema
    if (dma_tx_chan < 0) dma_tx_chan = dma_claim_unused_channel(true);
    if (dma_rx_chan < 0) dma_rx_chan = dma_claim_unused_channel(true);

    // Configurar la sensibilidad del giroscopio en el registro GYRO_CONFIG
    uint16_t sensitivity = (g_cfg->gyro_sensitivity_lsb_per_dps != 0)
        ? g_cfg->gyro_sensitivity_lsb_per_dps
        : 131;
    mpu_write(MPU_REG_GYRO_CONFIG, mpu_gyro_fs_bits_from_sensitivity(sensitivity));
}

void mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = { (uint8_t)(reg & 0x7F), data }; // MSB 0 para escritura
    gpio_put(g_cfg->pin_cs, 0);
    spi_write_blocking(g_cfg->spi, buf, 2);
    gpio_put(g_cfg->pin_cs, 1);
}

void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len) {
    uint8_t addr = reg | 0x80; // MSB 1 para lectura
    
    gpio_put(g_cfg->pin_cs, 0);
    
    // Enviamos la dirección del registro de forma bloqueante rápida (solo 1 byte)
    spi_write_blocking(g_cfg->spi, &addr, 1);

    // Determinar los DREQ (Data Request) correctos según el bloque SPI en uso
    uint dreq_rx = (spi_get_index(g_cfg->spi) == 0) ? DREQ_SPI0_RX : DREQ_SPI1_RX;
    uint dreq_tx = (spi_get_index(g_cfg->spi) == 0) ? DREQ_SPI0_TX : DREQ_SPI1_TX;

    // 1. Configuración DMA RX: Recibe datos desde el registro FIFO de SPI hacia nuestro buffer
    dma_channel_config c_rx = dma_channel_get_default_config(dma_rx_chan);
    channel_config_set_transfer_data_size(&c_rx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_rx, false);  // Dirección fija (registro SPI DR)
    channel_config_set_write_increment(&c_rx, true);   // Incrementar buffer de destino
    channel_config_set_dreq(&c_rx, dreq_rx);

    dma_channel_configure(
        dma_rx_chan,
        &c_rx,
        buf,                             // Destino
        &spi_get_hw(g_cfg->spi)->dr,     // Origen (Registro de datos SPI)
        len,                             // Cantidad de bytes
        false                            // No iniciar de inmediato
    );

    // 2. Configuración DMA TX: Envía bytes nulos para forzar el reloj de SPI (Full-Duplex)
    dma_channel_config c_tx = dma_channel_get_default_config(dma_tx_chan);
    channel_config_set_transfer_data_size(&c_tx, DMA_SIZE_8);
    channel_config_set_read_increment(&c_tx, false);  // Dirección fija (byte dummy constante)
    channel_config_set_write_increment(&c_tx, false); // Dirección fija (registro SPI DR)
    channel_config_set_dreq(&c_tx, dreq_tx);

    dma_channel_configure(
        dma_tx_chan,
        &c_tx,
        &spi_get_hw(g_cfg->spi)->dr,     // Destino
        &dma_dummy_byte,                 // Origen
        len,                             // Cantidad de bytes
        false                            // No iniciar de inmediato
    );

    // Disparar ambos canales en simultáneo usando una máscara de bits
    dma_start_channel_mask((1u << dma_rx_chan) | (1u << dma_tx_chan));

    // Esperar a que el canal RX termine de recolectar todos los datos solicitados
    dma_channel_wait_for_finish_blocking(dma_rx_chan);

    gpio_put(g_cfg->pin_cs, 1);
}

void mpu_read_accel_fixed(q16_16 *output) {
    uint8_t raw[6];
    mpu_read(MPU_REG_ACCEL_XOUT_H, raw, 6); // Lectura por DMA

    int16_t ax = (raw[0] << 8) | raw[1];
    int16_t ay = (raw[2] << 8) | raw[3];
    int16_t az = (raw[4] << 8) | raw[5];

    // Escala de 2g (16384 LSB/g)
    output[0] = q_div(TO_Q16(ax), TO_Q16(16384));
    output[1] = q_div(TO_Q16(ay), TO_Q16(16384));
    output[2] = q_div(TO_Q16(az), TO_Q16(16384));
}

void mpu_read_gyro_fixed(q16_16 *output) {
    uint8_t raw[6];
    // Los registros de velocidad angular comienzan en 0x43 (GYRO_XOUT_H)
    mpu_read(MPU_REG_GYRO_XOUT_H, raw, 6); // Lectura por DMA

    int16_t gx = (raw[0] << 8) | raw[1];
    int16_t gy = (raw[2] << 8) | raw[3];
    int16_t gz = (raw[4] << 8) | raw[5];

    uint16_t sensitivity = (g_cfg != NULL && g_cfg->gyro_sensitivity_lsb_per_dps != 0)
        ? g_cfg->gyro_sensitivity_lsb_per_dps
        : 131;

    output[0] = q_div(TO_Q16(gx), TO_Q16(sensitivity));
    output[1] = q_div(TO_Q16(gy), TO_Q16(sensitivity));
    output[2] = q_div(TO_Q16(gz), TO_Q16(sensitivity));
}

void mpu_enable_drdy(void) {
    // 1. Despertar el sensor (selección automática de reloj interno)
    mpu_write(MPU_REG_PWR_MGMT_1, 0x01);
    
    // 2. Habilitar la interrupción Data Ready en el sensor
    mpu_write(MPU_REG_INT_ENABLE, 0x01); 
}