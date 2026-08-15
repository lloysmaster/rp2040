#ifndef CONFIG_GYRO_H
#define CONFIG_GYRO_H

// --- REGISTROS MPU6500 ---
#define REG_GYRO_CONFIG  0x1B
#define REG_PWR_MGMT_1   0x6B
#define REG_GYRO_XOUT_H  0x43
#define READ_BIT         0x80

// Fondo de escala del giroscopio en °/s: 250, 500, 1000 o 2000. Define el
// registro GYRO_CONFIG y la sensibilidad nominal correspondiente.
#define GYRO_FULL_SCALE_DPS 500

// Sensibilidad real del giroscopio en LSB/(°/s). Nominales del MPU6500:
//   +-250 °/s -> 131.0   +-500 °/s -> 65.5
//   +-1000 °/s -> 32.8   +-2000 °/s -> 16.4
// Cada sensor difiere algunos puntos porcentuales del nominal: medirlo con el
// debug de giro (tecla 'r') y copiar aquí el valor. Debe pertenecer al fondo de
// escala elegido arriba; si no, se ignora y se usa el nominal.
#define GYRO_SENSITIVITY_LSB_PER_DPS 65.5f

// Desvío máximo aceptado respecto de la sensibilidad nominal (fracción). Un
// error mayor no es de escala: es un giro mal hecho o el fondo de escala
// equivocado.
#define GYRO_SENSITIVITY_MAX_DEVIATION 0.25f

// Filtro digital interno (DLPF) del giroscopio: 1 = 184 Hz de banda y salida a
// 1 kHz. Con 0 el DLPF queda puenteado (250 Hz de banda, salida a 8 kHz) y el
// bucle vería aliasing de las vibraciones del motor.
#define MPU_DLPF_CFG 0x01

// Filtro digital interno del acelerómetro (ACCEL_CONFIG2): 1 = 184 Hz.
#define MPU_ACCEL_DLPF_CFG 0x01

// Divisor de la tasa de salida: 0 deja el 1 kHz que fija el DLPF. La cadencia
// del sensor debe ser al menos la del bucle de control.
#define MPU_SMPLRT_DIV 0x00

// Sensibilidad del acelerómetro en LSB/g (escala +-2 g del MPU6500).
#define ACCEL_SENSITIVITY_LSB_PER_G 16384.0f

// Calibración de sesgo: a 500 Hz de muestreo, 1000 muestras son ~2 s.
#define GYRO_CALIBRATION_SAMPLES 1000

// Movimiento máximo tolerado durante la calibración (pico a pico, en LSB).
// Si se supera, la calibración se descarta porque el dron no estaba quieto.
#define GYRO_CALIBRATION_MAX_SPREAD_LSB 250

#endif // CONFIG_GYRO_H
