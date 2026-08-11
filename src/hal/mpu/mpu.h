#ifndef MPU_H
#define MPU_H

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "math/fixed_point.h"

// Direcciones de registros clave (MPU6500)
#define MPU_REG_ACCEL_XOUT_H  0x3B
#define MPU_REG_GYRO_XOUT_H  0x43
#define MPU_REG_INT_PIN_CFG   0x37
#define MPU_REG_INT_ENABLE    0x38
#define MPU_REG_GYRO_CONFIG   0x1B
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
    // Fondo de escala del giroscopio en °/s: 250, 500, 1000 o 2000.
    uint16_t gyro_full_scale_dps;
    // LSB por °/s medidos para ese fondo de escala (nominal 131.0, 65.5, 32.8
    // o 16.4). Se ignora si no pertenece al fondo de escala configurado.
    float gyro_sensitivity_lsb_per_dps;
} mpu_config_t;

// Resultado de la última calibración de sensores en reposo.
typedef struct {
    bool valid;                 // true si la calibración terminó sin movimiento
    uint16_t samples;           // muestras promediadas
    float gyro_bias_lsb[3];     // sesgo del giroscopio en LSB
    float accel_bias_lsb[3];    // sesgo del acelerómetro en LSB (Z ya sin 1 g)
    int32_t gyro_spread_lsb[3]; // recorrido pico a pico visto al calibrar
} mpu_calibration_t;

/**
 * @brief Inicializa el hardware SPI, reserva los canales DMA y configura el MPU6500.
 * @param config Puntero a la estructura de configuración.
 */
void mpu_init(mpu_config_t *config);

/**
 * @brief Escribe un byte en un registro específico (Bloqueante estándar para configuración).
 */
void mpu_write(uint8_t reg, uint8_t data);

/**
 * @brief Lee varios bytes desde un registro utilizando canales DMA TX y RX en paralelo.
 */
void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len);

/**
 * @brief Lee los 3 ejes del acelerómetro usando DMA y los convierte a Q16_16 (en g).
 * @param output Arreglo de 3 elementos q16_16.
 */
void mpu_read_accel_fixed(q16_16 *output);

/**
 * @brief Lee los 3 ejes del giroscopio usando DMA y los convierte a Q16_16 (en °/s).
 * @param output Arreglo de 3 elementos q16_16.
 */
void mpu_read_gyro_fixed(q16_16 *output);

/**
 * @brief Lee los 3 ejes crudos del giroscopio, sin escalar ni descontar el sesgo.
 */
void mpu_read_gyro_raw(int16_t *output);

/**
 * @brief Lee los 3 ejes crudos del acelerómetro, sin escalar ni descontar el sesgo.
 */
void mpu_read_accel_raw(int16_t *output);

/**
 * @brief Devuelve la última lectura cruda del giroscopio hecha por mpu_read_gyro_fixed().
 */
void mpu_get_last_gyro_raw(int16_t *output);

/**
 * @brief Promedia el sensor en reposo para obtener el sesgo del giroscopio y del
 *        acelerómetro. El dron debe estar quieto y nivelado.
 * @param samples Cantidad de muestras a promediar.
 * @param max_spread_lsb Recorrido pico a pico del giroscopio tolerado; si se
 *        supera, se descarta el resultado y se conservan los sesgos anteriores.
 * @return true si la calibración se aplicó.
 */
bool mpu_calibrate(uint16_t samples, uint16_t max_spread_lsb);

/**
 * @brief Devuelve el resultado de la última calibración (nunca NULL).
 */
const mpu_calibration_t *mpu_get_calibration(void);

/**
 * @brief Ajusta en caliente los LSB por °/s usados al convertir el giroscopio.
 *        Solo acepta valores dentro de GYRO_SENSITIVITY_MAX_DEVIATION del
 *        nominal del fondo de escala actual: el fondo de escala no se toca, así
 *        que el sesgo calibrado sigue siendo válido.
 * @return true si el valor se aplicó.
 */
bool mpu_set_gyro_sensitivity(float lsb_per_dps);

/**
 * @brief LSB por °/s en uso.
 */
float mpu_get_gyro_sensitivity(void);

/**
 * @brief Cambia el fondo de escala del giroscopio (250, 500, 1000 o 2000 °/s),
 *        deja la sensibilidad en el nominal de esa escala e invalida el sesgo,
 *        que estaba expresado en LSB de la escala anterior.
 * @return true si la escala pedida es válida y se aplicó.
 */
bool mpu_set_gyro_full_scale(uint16_t full_scale_dps);

/**
 * @brief Fondo de escala del giroscopio en °/s.
 */
uint16_t mpu_get_gyro_full_scale(void);

/**
 * @brief Sensibilidad nominal en LSB/(°/s) del fondo de escala en uso.
 */
float mpu_get_nominal_gyro_sensitivity(void);

/**
 * @brief Configura la interrupción Data Ready en el sensor.
 */
void mpu_enable_drdy(void);

#endif // MPU_H
