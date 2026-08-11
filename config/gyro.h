#ifndef CONFIG_GYRO_H
#define CONFIG_GYRO_H

// --- REGISTROS MPU6500 ---
#define REG_GYRO_CONFIG  0x1B
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_XOUT_H  0x43
#define READ_BIT         0x80

// Sensibilidad del giroscopio en LSB/(°/s). Valores nominales del MPU6500:
//   +-250 °/s -> 131.0   +-500 °/s -> 65.5
//   +-1000 °/s -> 32.8   +-2000 °/s -> 16.4
// El valor real de cada sensor difiere algunos puntos porcentuales: usar el
// debug de giro (tecla 'r' del menú serie) para medirlo y ajustar este número.
#define GYRO_SENSITIVITY_LSB_PER_DPS 65.5f

// Sensibilidad del acelerómetro en LSB/g (escala +-2 g del MPU6500).
#define ACCEL_SENSITIVITY_LSB_PER_G 16384.0f

// Calibración de sesgo: a 500 Hz de muestreo, 1000 muestras son ~2 s.
#define GYRO_CALIBRATION_SAMPLES 1000

// Movimiento máximo tolerado durante la calibración (pico a pico, en LSB).
// Si se supera, la calibración se descarta porque el dron no estaba quieto.
#define GYRO_CALIBRATION_MAX_SPREAD_LSB 250

#endif // CONFIG_GYRO_H
