#ifndef CONFIG_BATTERY_H
#define CONFIG_BATTERY_H

// --- Sensado de batería (telemetría CRSF BATTERY_SENSOR) ---
// Divisor resistivo de tensión conectado a ADC0 (GP26) y sensor de corriente
// opcional conectado a ADC1 (GP27). Ajustar según el hardware real.

#define BATTERY_VOLTAGE_ADC_PIN   26
#define BATTERY_VOLTAGE_ADC_INPUT 0

#define BATTERY_CURRENT_ADC_PIN   27
#define BATTERY_CURRENT_ADC_INPUT 1

// Poner en 0 si no hay sensor de corriente conectado (se reporta 0 A)
#define BATTERY_CURRENT_SENSOR_ENABLED 1

// Referencia y resolución del ADC de la RP2040
#define BATTERY_ADC_VREF_V 3.3f
#define BATTERY_ADC_MAX    4095.0f

// Relación del divisor: Vbat = Vadc * BATTERY_VOLTAGE_DIVIDER
// Ejemplo típico para 6S con R1=100k y R2=10k -> (100k + 10k) / 10k = 11.0
#define BATTERY_VOLTAGE_DIVIDER 11.0f

// Sensor de corriente: salida lineal (ej. ACS712 / sensor del PDB)
#define BATTERY_CURRENT_OFFSET_V   0.0f   // Tensión de salida con 0 A
#define BATTERY_CURRENT_SCALE_A_V  40.0f  // Amperios por voltio de salida

// Rango de tensión por celda usado para estimar el porcentaje restante
#define BATTERY_CELL_MIN_V 3.40f
#define BATTERY_CELL_MAX_V 4.20f

#endif // CONFIG_BATTERY_H
