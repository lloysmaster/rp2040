#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Asegúrate de que las rutas a los headers coincidan con la estructura de tus carpetas
#include "hal/mpu/mpu.h"

#include "hal/rx/crossfire.h" 
#include "math/fixed_point.h"
#include "core/attitude.h"
#include "control/mixer.h"
#include "hal/esc/dshot.h"
#include "hal/gps/gps.h"
#include "hal/battery/battery.h"
#include "telemetry/telemetry.h"
#include "debug/gyro_debug.h"
#include "config/pinout.h"
#include "config/gyro.h"


#define SAFE_ARMED_IDLE_THROTTLE DSHOT_MIN_THROTTLE

volatile bool data_ready = false;

// Callback de la interrupción externa del GPIO (MPU DRDY)
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 14) { // Pin DRDY
        data_ready = true;
    }
}

int main() {
    // --- Inicialización de stdio para Debug (USB/UART) ---
    stdio_init_all();
    sleep_ms(2000); // Pausa breve para permitir la conexión del monitor serie
    printf("[DEBUG] Sistema iniciando...\n");

    // --- Inicialización del sensor MPU6500 ---
    mpu_config_t cfg = {
        .spi = spi1, 
        .pin_cs = 13, 
        .pin_sck = 10,
        .pin_mosi = 11, 
        .pin_miso = 12, 
        .pin_drdy = 14,
        .gyro_sensitivity_lsb_per_dps = GYRO_SENSITIVITY_LSB_PER_DPS
    };
    
    mpu_init(&cfg);
    printf("[DEBUG] MPU6500 inicializado\n");
    mpu_enable_drdy();

    // Calibración de sesgo en reposo: el dron debe estar quieto y nivelado.
    printf("[DEBUG] Calibrando sensores: no mover el dron...\n");
    if (mpu_calibrate(GYRO_CALIBRATION_SAMPLES, GYRO_CALIBRATION_MAX_SPREAD_LSB)) {
        const mpu_calibration_t *cal = mpu_get_calibration();
        printf("[DEBUG] Calibracion OK: sesgo giro X=%+.1f Y=%+.1f Z=%+.1f LSB\n",
               (double)cal->gyro_bias_lsb[0], (double)cal->gyro_bias_lsb[1],
               (double)cal->gyro_bias_lsb[2]);
    } else {
        printf("[DEBUG] Calibracion rechazada (hubo movimiento): repetir con 'z' desde el debug\n");
    }
    
    // Habilitar interrupción física en el flanco de subida del pin DRDY
    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // --- Inicialización de periféricos ---
    crsf_init();
    gps_init();
    battery_init();
    telemetry_init();
    attitude_init();
    attitude_set_mode(FLIGHT_MODE_ACRO);
    mixer_init();
    gyro_debug_init();
    dshot_init(MOTOR_BASE_PIN);
    
    for (int i = 0; i < 4; ++i) {
        dshot_set_throttle(i, 0);
    }
    printf("[DEBUG] Periféricos y motores inicializados\n");

    uint32_t last_loop_us = time_us_32();
    static bool esc_armed = false;
    static attitude_cmd_t last_attitude_cmd = {0};
    static mixer_output_t last_motor_cmd = {0};
    uint16_t esc_throttle[4] = {0, 0, 0, 0};

    while (true) {
        // 1. Procesar datos del receptor Crossfire (No bloqueante)
        const crsf_data_t* rc_data = crsf_get_data();
        if (crsf_update()) {
            // CRSF entrega valores entre ~172 y ~1811 (centro en 992)
        }

        // 2. Procesar datos del MPU (Acelerómetro y Giroscopio)
        if (data_ready) {
            data_ready = false;

            q16_16 accel_data[3];
            mpu_read_accel_fixed(accel_data); // Ejecutado internamente bajo DMA

            q16_16 gyro_data[3];
            mpu_read_gyro_fixed(gyro_data);   // Ejecutado internamente bajo DMA

            gyro_debug_feed(gyro_data, accel_data, 0.005f);
            attitude_estimate(accel_data, gyro_data, 0.005f);
            attitude_update(rc_data, gyro_data, &last_attitude_cmd);
            mixer_mix(&last_attitude_cmd, &last_motor_cmd);
        }

        for (int i = 0; i < 4; ++i) {
            if (!esc_armed) {
                esc_throttle[i] = 0;
            } else {
                uint32_t val = (uint32_t)((last_motor_cmd.motor[i] * 2047u) / 1000u);
                if (val > 2047u) val = 2047u;
                if (val < SAFE_ARMED_IDLE_THROTTLE) val = SAFE_ARMED_IDLE_THROTTLE;
                esc_throttle[i] = (uint16_t)val;
            }
        }

        if (!rc_data->is_connected) {
            if (esc_armed) {
                if (dshot_disarm()) {
                    esc_armed = false;
                    printf("[DEBUG] RC Desconectado: Motores desarmados por seguridad\n");
                }
            }
            for (int i = 0; i < 4; ++i) {
                dshot_set_throttle(i, 0);
            }
        } else {
            bool arm_switch = (rc_data->channels[4] > 1500);
            bool throttle_low = (rc_data->channels[2] < 1050);

            if (!esc_armed) {
                if (arm_switch && throttle_low) {
                    if (dshot_arm()) {
                        esc_armed = true;
                        printf("[DEBUG] ¡Motores ARMADOS!\n");
                    }
                }
                for (int i = 0; i < 4; ++i) dshot_set_throttle(i, 0);
            } else {
                if (!arm_switch) {
                    if (dshot_disarm()) {
                        esc_armed = false;
                        printf("[DEBUG] Motores DESARMADOS por interruptor\n");
                    }
                    for (int i = 0; i < 4; ++i) dshot_set_throttle(i, 0);
                } else {
                    for (int i = 0; i < 4; ++i) {
                        uint16_t motor_cmd = esc_throttle[i];
                        if (esc_armed && motor_cmd < SAFE_ARMED_IDLE_THROTTLE) {
                            motor_cmd = SAFE_ARMED_IDLE_THROTTLE;
                        }
                        dshot_set_throttle(i, motor_cmd);
                    }
                }
            }
        }

        // --- Debug de giroscopio y calibración (menú por puerto serie) ---
        gyro_debug_service(esc_armed);

        // 3. Telemetría: GPS y batería se muestrean y se reenvían por CRSF al receptor
        gps_update();
        battery_update();
        telemetry_set_armed(esc_armed);
        telemetry_update();

        uint32_t now_us = time_us_32();
        uint32_t loop_dt_us = now_us - last_loop_us;
        last_loop_us = now_us;
        if (loop_dt_us < 5000) {
            sleep_us(5000 - loop_dt_us);
        }
    }
    
    return 0;
}