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
#include "config/pinout.h"

#define ARM_THRESHOLD 250
#define DISARM_THRESHOLD 180
#define SAFE_ARMED_IDLE_THROTTLE DSHOT_MIN_THROTTLE

volatile bool data_ready = false;

// Callback de la interrupción externa del GPIO (MPU DRDY)
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 14) { // Pin DRDY
        data_ready = true;
    }
}

int main() {
    



    // --- Inicialización del sensor MPU6500 ---
    mpu_config_t cfg = {
        .spi = spi1, 
        .pin_cs = 13, 
        .pin_sck = 10,
        .pin_mosi = 11, 
        .pin_miso = 12, 
        .pin_drdy = 14,
        .gyro_sensitivity_lsb_per_dps = 131
    };
    
    mpu_init(&cfg);
    mpu_enable_drdy();
    
    // Habilitar interrupción física en el flanco de subida del pin DRDY
    gpio_set_irq_enabled_with_callback(14, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // --- Inicialización del receptor Crossfire ---
    crsf_init();
    attitude_init();
    attitude_set_mode(FLIGHT_MODE_ACRO);
    mixer_init();
    dshot_init(MOTOR_BASE_PIN);
        for (int i = 0; i < 4; ++i) {
            dshot_set_throttle(i, 0);
        }

    

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
                        
                    }
                }
                for (int i = 0; i < 4; ++i) dshot_set_throttle(i, 0);
            } else {
                if (!arm_switch) {
                    if (dshot_disarm()) {
                        esc_armed = false;
                        
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

        

        

        uint32_t now_us = time_us_32();
        uint32_t loop_dt_us = now_us - last_loop_us;
        last_loop_us = now_us;
        if (loop_dt_us < 5000) {
            sleep_us(5000 - loop_dt_us);
        }
    }
    
    return 0;
}