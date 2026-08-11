#include "debug/gyro_debug.h"

#include <stdio.h>

#include "pico/stdlib.h"

#include "config/gyro.h"
#include "hal/mpu/mpu.h"

#define GYRO_DEBUG_PRINT_PERIOD_US 250000u
#define GYRO_DEBUG_SATURATION_LSB 32000
#define GYRO_DEBUG_SENSITIVITY_STEP 0.1f
// Por encima de este giro se considera que el dron se movio y la medicion de
// deriva vuelve a empezar, para que siempre sea la deriva del ultimo reposo.
#define GYRO_DEBUG_MOVEMENT_DPS 3.0f
// Un eje secundario mayor a esta fraccion del eje dominante significa que el
// giro no fue sobre un solo eje y la medicion no sirve para ajustar la escala.
#define GYRO_DEBUG_MAX_CROSS_AXIS 0.10f

static const char *axis_name[3] = {"X (roll)", "Y (pitch)", "Z (yaw)"};

static bool measuring;
static bool print_enabled = true;
static float reference_deg = 360.0f;
static float measured_deg[3];   // ángulo integrado durante la medición
static float drift_deg[3];      // ángulo integrado en reposo (deriva)
static float drift_seconds;
static float suggested_sensitivity;
static uint32_t saturated_samples;
static uint32_t last_print_us;
static bool drift_restarted;

static bool suggestion_valid;
static uint32_t samples_since_print;
static float last_rate_dps[3];
static float last_accel_g[3];
static int16_t last_raw[3];

static float q16_to_float(q16_16 value) {
    return (float)value / 65536.0f;
}

static float absf(float value) {
    return value < 0.0f ? -value : value;
}

static uint16_t next_full_scale(uint16_t full_scale_dps) {
    switch (full_scale_dps) {
        case 250:  return 500;
        case 500:  return 1000;
        case 1000: return 2000;
        default:   return 250;
    }
}

static void print_help(void) {
    printf("\n[GIRO] --- Ajuste de gyro_sensitivity_lsb_per_dps ---\n");
    printf("[GIRO]  h : esta ayuda\n");
    printf("[GIRO]  z : calibrar sesgo con el dron quieto y desarmado (~%d ms)\n",
           (int)(GYRO_CALIBRATION_SAMPLES * 2));
    printf("[GIRO]  1/2/3 : referencia de giro 90 / 180 / 360 grados\n");
    printf("[GIRO]  r : iniciar o detener la medicion de un giro de referencia\n");
    printf("[GIRO]  a : aplicar la sensibilidad sugerida por la ultima medicion\n");
    printf("[GIRO]  f : cambiar el fondo de escala (250/500/1000/2000 dps, obliga a recalibrar)\n");
    printf("[GIRO]  +/- : ajustar la sensibilidad en %+.1f LSB/(deg/s)\n",
           (double)GYRO_DEBUG_SENSITIVITY_STEP);
    printf("[GIRO]  d : reiniciar la medicion de deriva (tambien se reinicia sola al mover)\n");
    printf("[GIRO]  p : pausar o reanudar las impresiones periodicas\n\n");
}

static void reset_drift(void) {
    for (int i = 0; i < 3; ++i) {
        drift_deg[i] = 0.0f;
    }
    drift_seconds = 0.0f;
    saturated_samples = 0;
    drift_restarted = false;
}

static void print_calibration(void) {
    const mpu_calibration_t *cal = mpu_get_calibration();
    if (cal->samples == 0) {
        printf("[GIRO] Sesgo sin calibrar (pulsar 'z')\n");
        return;
    }
    printf("[GIRO] Sesgo %s (%u muestras): giro X=%+.1f Y=%+.1f Z=%+.1f LSB | "
           "accel X=%+.0f Y=%+.0f Z=%+.0f LSB | pico a pico X=%ld Y=%ld Z=%ld LSB\n",
           cal->valid ? "OK" : "DESCARTADO (hubo movimiento)",
           (unsigned)cal->samples,
           (double)cal->gyro_bias_lsb[0], (double)cal->gyro_bias_lsb[1], (double)cal->gyro_bias_lsb[2],
           (double)cal->accel_bias_lsb[0], (double)cal->accel_bias_lsb[1], (double)cal->accel_bias_lsb[2],
           (long)cal->gyro_spread_lsb[0], (long)cal->gyro_spread_lsb[1], (long)cal->gyro_spread_lsb[2]);
}

static void run_calibration(void) {
    printf("[GIRO] Calibrando sesgo: mantener el dron quieto y nivelado...\n");
    const bool ok = mpu_calibrate(GYRO_CALIBRATION_SAMPLES, GYRO_CALIBRATION_MAX_SPREAD_LSB);
    print_calibration();
    if (!ok) {
        printf("[GIRO] Calibracion rechazada: repetir sin mover el dron\n");
    }
    reset_drift();
}

static void start_measurement(void) {
    for (int i = 0; i < 3; ++i) {
        measured_deg[i] = 0.0f;
    }
    saturated_samples = 0;
    measuring = true;
    printf("[GIRO] Midiendo: girar el dron %.0f grados sobre UN solo eje y pulsar 'r'\n",
           (double)reference_deg);
}

static void finish_measurement(void) {
    measuring = false;
    suggestion_valid = false;

    int dominant = 0;
    for (int i = 1; i < 3; ++i) {
        if (absf(measured_deg[i]) > absf(measured_deg[dominant])) {
            dominant = i;
        }
    }

    const float measured = absf(measured_deg[dominant]);
    printf("[GIRO] Integrado: X=%+.1f Y=%+.1f Z=%+.1f grados\n",
           (double)measured_deg[0], (double)measured_deg[1], (double)measured_deg[2]);

    if (saturated_samples > 0) {
        printf("[GIRO] ATENCION: %lu muestras saturadas, el giro fue demasiado rapido "
               "para el fondo de escala actual\n", (unsigned long)saturated_samples);
    }

    if (measured < 5.0f) {
        printf("[GIRO] Medicion descartada: apenas se detecto giro\n");
        return;
    }

    const float sensitivity = mpu_get_gyro_sensitivity();
    // El angulo integrado escala inversamente con la sensibilidad usada, asi que
    // la sensibilidad correcta es la actual corregida por el error de medicion.
    const float candidate = sensitivity * measured / reference_deg;
    const float error_pct = 100.0f * (measured - reference_deg) / reference_deg;

    printf("[GIRO] Eje dominante %s: medido %.1f grados vs referencia %.0f (error %+.1f%%)\n",
           axis_name[dominant], (double)measured, (double)reference_deg, (double)error_pct);

    const mpu_calibration_t *cal = mpu_get_calibration();
    if (!cal->valid) {
        printf("[GIRO] Medicion no valida: el sesgo no esta calibrado, pulsar 'z' y repetir\n");
        return;
    }

    for (int i = 0; i < 3; ++i) {
        if (i == dominant) {
            continue;
        }
        if (absf(measured_deg[i]) > GYRO_DEBUG_MAX_CROSS_AXIS * measured) {
            printf("[GIRO] Medicion no valida: %s giro %+.1f grados (mas del %.0f%% del eje "
                   "dominante), repetir el giro sobre un solo eje\n",
                   axis_name[i], (double)measured_deg[i],
                   (double)(GYRO_DEBUG_MAX_CROSS_AXIS * 100.0f));
            return;
        }
    }

    const float nominal = mpu_get_nominal_gyro_sensitivity();
    const float deviation_pct = 100.0f * absf(candidate - nominal) / nominal;
    if (deviation_pct > 100.0f * GYRO_SENSITIVITY_MAX_DEVIATION) {
        printf("[GIRO] Sugerencia %.2f descartada: se aparta %.0f%% del nominal %.1f del fondo "
               "de escala %u dps. Un error asi no es de escala: revisar el giro o cambiar el "
               "fondo de escala con 'f'\n",
               (double)candidate, (double)deviation_pct, (double)nominal,
               (unsigned)mpu_get_gyro_full_scale());
        return;
    }

    suggested_sensitivity = candidate;
    suggestion_valid = true;
    printf("[GIRO] Sensibilidad sugerida %.2f LSB/(deg/s) (actual %.2f, nominal %.1f) -> "
           "pulsar 'a' para aplicarla\n",
           (double)suggested_sensitivity, (double)sensitivity, (double)nominal);
}

static void apply_suggestion(void) {
    if (!suggestion_valid) {
        printf("[GIRO] No hay una sugerencia valida: medir un giro con 'r'\n");
        return;
    }
    if (!mpu_set_gyro_sensitivity(suggested_sensitivity)) {
        printf("[GIRO] Sensibilidad %.2f rechazada para el fondo de escala %u dps\n",
               (double)suggested_sensitivity, (unsigned)mpu_get_gyro_full_scale());
        return;
    }
    printf("[GIRO] Sensibilidad aplicada: %.2f LSB/(deg/s) (+-%u dps)\n",
           (double)mpu_get_gyro_sensitivity(), (unsigned)mpu_get_gyro_full_scale());
    printf("[GIRO] Copiar este valor en GYRO_SENSITIVITY_LSB_PER_DPS (config/gyro.h)\n");
    reset_drift();
}

static void nudge_sensitivity(float delta) {
    const float target = mpu_get_gyro_sensitivity() + delta;
    if (!mpu_set_gyro_sensitivity(target)) {
        printf("[GIRO] %.2f fuera del %.0f%% permitido alrededor del nominal %.1f (+-%u dps)\n",
               (double)target, (double)(GYRO_SENSITIVITY_MAX_DEVIATION * 100.0f),
               (double)mpu_get_nominal_gyro_sensitivity(), (unsigned)mpu_get_gyro_full_scale());
        return;
    }
    printf("[GIRO] Sensibilidad = %.2f LSB/(deg/s) (+-%u dps)\n",
           (double)mpu_get_gyro_sensitivity(), (unsigned)mpu_get_gyro_full_scale());
    reset_drift();
}

static void cycle_full_scale(void) {
    const uint16_t target = next_full_scale(mpu_get_gyro_full_scale());
    mpu_set_gyro_full_scale(target);
    suggestion_valid = false;
    printf("[GIRO] Fondo de escala = +-%u dps, sensibilidad nominal %.1f LSB/(deg/s)\n",
           (unsigned)mpu_get_gyro_full_scale(), (double)mpu_get_gyro_sensitivity());
    printf("[GIRO] El sesgo quedo invalidado (estaba en LSB de la escala anterior): "
           "calibrar con 'z' antes de medir\n");
    reset_drift();
}

static void handle_command(int c, bool armed) {
    switch (c) {
        case 'h':
        case '?':
            print_help();
            break;
        case 'z':
            if (armed) {
                printf("[GIRO] Calibracion bloqueada con los motores armados\n");
            } else {
                run_calibration();
            }
            break;
        case '1':
            reference_deg = 90.0f;
            printf("[GIRO] Referencia = 90 grados\n");
            break;
        case '2':
            reference_deg = 180.0f;
            printf("[GIRO] Referencia = 180 grados\n");
            break;
        case '3':
            reference_deg = 360.0f;
            printf("[GIRO] Referencia = 360 grados\n");
            break;
        case 'r':
            if (measuring) {
                finish_measurement();
            } else {
                start_measurement();
            }
            break;
        case 'a':
            apply_suggestion();
            break;
        case 'f':
            if (armed) {
                printf("[GIRO] Cambio de fondo de escala bloqueado con los motores armados\n");
            } else {
                cycle_full_scale();
            }
            break;
        case '+':
            nudge_sensitivity(GYRO_DEBUG_SENSITIVITY_STEP);
            break;
        case '-':
            nudge_sensitivity(-GYRO_DEBUG_SENSITIVITY_STEP);
            break;
        case 'd':
            reset_drift();
            printf("[GIRO] Deriva reiniciada\n");
            break;
        case 'p':
            print_enabled = !print_enabled;
            printf("[GIRO] Impresiones %s\n", print_enabled ? "activas" : "pausadas");
            break;
        default:
            break;
    }
}

void gyro_debug_init(void) {
    measuring = false;
    print_enabled = true;
    reference_deg = 360.0f;
    suggested_sensitivity = 0.0f;
    suggestion_valid = false;
    samples_since_print = 0;
    last_print_us = time_us_32();
    for (int i = 0; i < 3; ++i) {
        measured_deg[i] = 0.0f;
        last_rate_dps[i] = 0.0f;
        last_accel_g[i] = 0.0f;
        last_raw[i] = 0;
    }
    reset_drift();
    print_help();
}

void gyro_debug_feed(const q16_16 gyro[3], const q16_16 accel[3], float dt_s) {
    if (gyro == NULL || dt_s <= 0.0f) {
        return;
    }

    mpu_get_last_gyro_raw(last_raw);
    ++samples_since_print;

    if (!measuring) {
        for (int i = 0; i < 3; ++i) {
            if (absf(q16_to_float(gyro[i])) > GYRO_DEBUG_MOVEMENT_DPS) {
                reset_drift();
                drift_restarted = true;
                break;
            }
        }
    }

    for (int i = 0; i < 3; ++i) {
        const float rate = q16_to_float(gyro[i]);
        last_rate_dps[i] = rate;
        if (measuring) {
            measured_deg[i] += rate * dt_s;
        } else {
            drift_deg[i] += rate * dt_s;
        }
        const int32_t magnitude = last_raw[i] < 0 ? -(int32_t)last_raw[i] : (int32_t)last_raw[i];
        if (magnitude >= GYRO_DEBUG_SATURATION_LSB) {
            ++saturated_samples;
        }
    }

    if (!measuring) {
        drift_seconds += dt_s;
    }

    if (accel != NULL) {
        for (int i = 0; i < 3; ++i) {
            last_accel_g[i] = q16_to_float(accel[i]);
        }
    }
}

void gyro_debug_service(bool armed) {
    int c = getchar_timeout_us(0);
    while (c != PICO_ERROR_TIMEOUT) {
        handle_command(c, armed);
        c = getchar_timeout_us(0);
    }

    const uint32_t now_us = time_us_32();
    const uint32_t elapsed_us = now_us - last_print_us;
    if (!print_enabled || elapsed_us < GYRO_DEBUG_PRINT_PERIOD_US) {
        return;
    }
    last_print_us = now_us;

    // Muestras por segundo reales: si no coinciden con el bucle nominal de
    // 200 Hz, todo lo integrado esta mal escalado.
    const float sample_hz = (float)samples_since_print * 1e6f / (float)elapsed_us;
    samples_since_print = 0;

    const float sensitivity = mpu_get_gyro_sensitivity();
    printf("[GIRO] sens=%.2f LSB/(deg/s) (+-%u dps) | %.0f Hz | crudo X=%6ld Y=%6ld Z=%6ld | "
           "dps X=%+7.1f Y=%+7.1f Z=%+7.1f | accel g X=%+5.2f Y=%+5.2f Z=%+5.2f\n",
           (double)sensitivity, (unsigned)mpu_get_gyro_full_scale(), (double)sample_hz,
           (long)last_raw[0], (long)last_raw[1], (long)last_raw[2],
           (double)last_rate_dps[0], (double)last_rate_dps[1], (double)last_rate_dps[2],
           (double)last_accel_g[0], (double)last_accel_g[1], (double)last_accel_g[2]);

    if (measuring) {
        printf("[GIRO] MIDIENDO (ref %.0f grados): integrado X=%+.1f Y=%+.1f Z=%+.1f | "
               "pulsar 'r' al terminar el giro\n",
               (double)reference_deg,
               (double)measured_deg[0], (double)measured_deg[1], (double)measured_deg[2]);
        return;
    }

    if (!mpu_get_calibration()->valid) {
        printf("[GIRO] SESGO SIN CALIBRAR: la deriva y las mediciones no son fiables, pulsar 'z'\n");
    }

    if (drift_restarted && drift_seconds < 1.0f) {
        printf("[GIRO] Movimiento detectado: deriva reiniciada\n");
        drift_restarted = false;
    }

    if (drift_seconds > 0.5f) {
        printf("[GIRO] Deriva en reposo, %.1f s: X=%+.2f Y=%+.2f Z=%+.2f grados "
               "(%+.2f %+.2f %+.2f deg/s medios)\n",
               (double)drift_seconds,
               (double)drift_deg[0], (double)drift_deg[1], (double)drift_deg[2],
               (double)(drift_deg[0] / drift_seconds),
               (double)(drift_deg[1] / drift_seconds),
               (double)(drift_deg[2] / drift_seconds));
    }
}
