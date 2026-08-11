#include "mpu.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "math/fixed_point.h"
#include "config/gyro.h"

static mpu_config_t *g_cfg;

// Canales DMA estáticos para el módulo MPU
static int dma_tx_chan = -1;
static int dma_rx_chan = -1;
static uint8_t dma_dummy_byte = 0x00;

static float g_gyro_sensitivity = GYRO_SENSITIVITY_LSB_PER_DPS;
static uint8_t g_gyro_fs_bits = 0x00;
static mpu_calibration_t g_cal;
static int16_t g_last_gyro_raw[3];

static q16_16 float_to_q16(float value) {
    return (q16_16)(value * 65536.0f);
}

static uint8_t mpu_gyro_fs_bits_from_sensitivity(float sensitivity_lsb_per_dps) {
    if (sensitivity_lsb_per_dps >= 98.0f) {
        return 0x00; // +-250 °/s -> 131 LSB/(°/s)
    }
    if (sensitivity_lsb_per_dps >= 49.0f) {
        return 0x08; // +-500 °/s -> 65.5 LSB/(°/s)
    }
    if (sensitivity_lsb_per_dps >= 24.0f) {
        return 0x10; // +-1000 °/s -> 32.8 LSB/(°/s)
    }
    return 0x18; // +-2000 °/s -> 16.4 LSB/(°/s)
}

static void mpu_decode_raw(const uint8_t raw[6], int16_t out[3]) {
    out[0] = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    out[1] = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    out[2] = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
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

    // Despertar el sensor antes de configurarlo (reloj interno automático)
    mpu_write(MPU_REG_PWR_MGMT_1, 0x01);
    sleep_ms(50);

    // Configurar la sensibilidad del giroscopio en el registro GYRO_CONFIG
    g_gyro_sensitivity = (g_cfg->gyro_sensitivity_lsb_per_dps > 0.0f)
        ? g_cfg->gyro_sensitivity_lsb_per_dps
        : GYRO_SENSITIVITY_LSB_PER_DPS;
    g_gyro_fs_bits = mpu_gyro_fs_bits_from_sensitivity(g_gyro_sensitivity);
    mpu_write(MPU_REG_GYRO_CONFIG, g_gyro_fs_bits);

    for (int i = 0; i < 3; ++i) {
        g_cal.gyro_bias_lsb[i] = 0.0f;
        g_cal.accel_bias_lsb[i] = 0.0f;
        g_cal.gyro_spread_lsb[i] = 0;
        g_last_gyro_raw[i] = 0;
    }
    g_cal.valid = false;
    g_cal.samples = 0;
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

void mpu_read_accel_raw(int16_t *output) {
    uint8_t raw[6];
    mpu_read(MPU_REG_ACCEL_XOUT_H, raw, 6); // Lectura por DMA
    mpu_decode_raw(raw, output);
}

void mpu_read_gyro_raw(int16_t *output) {
    uint8_t raw[6];
    // Los registros de velocidad angular comienzan en 0x43 (GYRO_XOUT_H)
    mpu_read(MPU_REG_GYRO_XOUT_H, raw, 6); // Lectura por DMA
    mpu_decode_raw(raw, output);
}

void mpu_get_last_gyro_raw(int16_t *output) {
    for (int i = 0; i < 3; ++i) {
        output[i] = g_last_gyro_raw[i];
    }
}

void mpu_read_accel_fixed(q16_16 *output) {
    int16_t raw[3];
    mpu_read_accel_raw(raw);

    for (int i = 0; i < 3; ++i) {
        const float corrected = (float)raw[i] - g_cal.accel_bias_lsb[i];
        output[i] = float_to_q16(corrected / ACCEL_SENSITIVITY_LSB_PER_G);
    }
}

void mpu_read_gyro_fixed(q16_16 *output) {
    int16_t raw[3];
    mpu_read_gyro_raw(raw);

    const float sensitivity = (g_gyro_sensitivity > 0.0f)
        ? g_gyro_sensitivity
        : GYRO_SENSITIVITY_LSB_PER_DPS;

    for (int i = 0; i < 3; ++i) {
        g_last_gyro_raw[i] = raw[i];
        const float corrected = (float)raw[i] - g_cal.gyro_bias_lsb[i];
        output[i] = float_to_q16(corrected / sensitivity);
    }
}

bool mpu_calibrate(uint16_t samples, uint16_t max_spread_lsb) {
    if (samples == 0) {
        return false;
    }

    int64_t gyro_sum[3] = {0, 0, 0};
    int64_t accel_sum[3] = {0, 0, 0};
    int16_t gyro_min[3] = {INT16_MAX, INT16_MAX, INT16_MAX};
    int16_t gyro_max[3] = {INT16_MIN, INT16_MIN, INT16_MIN};

    for (uint16_t s = 0; s < samples; ++s) {
        int16_t gyro[3];
        int16_t accel[3];
        mpu_read_gyro_raw(gyro);
        mpu_read_accel_raw(accel);

        for (int i = 0; i < 3; ++i) {
            gyro_sum[i] += gyro[i];
            accel_sum[i] += accel[i];
            if (gyro[i] < gyro_min[i]) gyro_min[i] = gyro[i];
            if (gyro[i] > gyro_max[i]) gyro_max[i] = gyro[i];
        }
        sleep_ms(2); // ~500 Hz de muestreo
    }

    int32_t spread[3];
    bool steady = true;
    for (int i = 0; i < 3; ++i) {
        spread[i] = (int32_t)gyro_max[i] - (int32_t)gyro_min[i];
        if (spread[i] > (int32_t)max_spread_lsb) {
            steady = false;
        }
    }

    for (int i = 0; i < 3; ++i) {
        g_cal.gyro_spread_lsb[i] = spread[i];
    }
    g_cal.samples = samples;

    if (!steady) {
        // El sensor se movió: no se aplican los nuevos sesgos.
        g_cal.valid = false;
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        g_cal.gyro_bias_lsb[i] = (float)gyro_sum[i] / (float)samples;
        g_cal.accel_bias_lsb[i] = (float)accel_sum[i] / (float)samples;
    }
    // El eje Z mide +1 g con el dron nivelado: ese componente no es sesgo.
    g_cal.accel_bias_lsb[2] -= ACCEL_SENSITIVITY_LSB_PER_G;
    g_cal.valid = true;
    return true;
}

const mpu_calibration_t *mpu_get_calibration(void) {
    return &g_cal;
}

bool mpu_set_gyro_sensitivity(float lsb_per_dps) {
    if (lsb_per_dps <= 0.0f) {
        return false;
    }

    g_gyro_sensitivity = lsb_per_dps;

    const uint8_t fs_bits = mpu_gyro_fs_bits_from_sensitivity(lsb_per_dps);
    if (fs_bits == g_gyro_fs_bits) {
        return false;
    }

    // Cambia el fondo de escala: el sesgo medido con la escala anterior ya no sirve.
    g_gyro_fs_bits = fs_bits;
    mpu_write(MPU_REG_GYRO_CONFIG, g_gyro_fs_bits);
    for (int i = 0; i < 3; ++i) {
        g_cal.gyro_bias_lsb[i] = 0.0f;
    }
    g_cal.valid = false;
    return true;
}

float mpu_get_gyro_sensitivity(void) {
    return g_gyro_sensitivity;
}

void mpu_enable_drdy(void) {
    // 1. Asegurar que el sensor está despierto (reloj interno automático)
    mpu_write(MPU_REG_PWR_MGMT_1, 0x01);
    
    // 2. Habilitar la interrupción Data Ready en el sensor
    mpu_write(MPU_REG_INT_ENABLE, 0x01); 
}
