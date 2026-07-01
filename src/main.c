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

volatile bool data_ready = false;

// Callback de la interrupción externa del GPIO (MPU DRDY)
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == 14) { // Pin DRDY
        data_ready = true;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000); // Pequeña espera para abrir la consola serial

    printf("Pico FC iniciada. Configurando DMA para MPU y CRSF...\n");

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
    mixer_init();
    dshot_init(MOTOR_BASE_PIN);
    dshot_set_throttle(0);

    printf("Sistema listo. Escuchando datos...\n");

    uint32_t last_loop_us = time_us_32();
    while (true) {
        // 1. Procesar datos del receptor Crossfire (No bloqueante)
        const crsf_data_t* rc_data = crsf_get_data();
        if (crsf_update()) {
            // CRSF entrega valores entre ~172 y ~1811 (centro en 992)
            printf("[CRSF] CH1: %d | CH2: %d | CH3: %d | CH4: %d\n", 
                   rc_data->channels[0], 
                   rc_data->channels[1], 
                   rc_data->channels[2], 
                   rc_data->channels[3]);
        }
        
        // 2. Procesar datos del MPU (Acelerómetro y Giroscopio)
        if (data_ready) {
            data_ready = false;
            
            q16_16 accel_data[3];
            mpu_read_accel_fixed(accel_data); // Ejecutado internamente bajo DMA
            
            q16_16 gyro_data[3];
            mpu_read_gyro_fixed(gyro_data);   // Ejecutado internamente bajo DMA

            if (rc_data->is_connected) {
                attitude_cmd_t attitude_cmd = {0};
                mixer_output_t motor_cmd = {0};

                attitude_update(rc_data, gyro_data, &attitude_cmd);
                mixer_mix(&attitude_cmd, &motor_cmd);

                static bool esc_armed = false;
                uint16_t esc_throttle = (uint16_t)((motor_cmd.motor[0] * 2047) / 1000);
                if (esc_throttle > 2047) {
                    esc_throttle = 2047;
                }

                bool arm_switch = (rc_data->channels[4] > 1500);
                bool throttle_low = (rc_data->channels[2] < 1050);

                if (!esc_armed) {
                    if (arm_switch && throttle_low) {
                        if (dshot_arm()) {
                            esc_armed = true;
                            printf("[ARM] Armed (module)\n");
                        }
                    }
                    dshot_set_throttle(0);
                } else {
                    if (!arm_switch) {
                        if (dshot_disarm()) {
                            esc_armed = false;
                            printf("[ARM] Disarmed (module)\n");
                        }
                        dshot_set_throttle(0);
                    } else {
                        dshot_set_throttle(esc_throttle);
                    }
                }

                printf("[DBG] RC_CH3:%d | arm:%d | esc_throttle:%u\n",
                       rc_data->channels[2],
                       esc_armed,
                       esc_throttle);

                printf("[ATT] R:%d P:%d Y:%d T:%d | M:%d,%d,%d,%d | DSHOT:%u\n",
                       attitude_cmd.roll_output,
                       attitude_cmd.pitch_output,
                       attitude_cmd.yaw_output,
                       attitude_cmd.throttle,
                       motor_cmd.motor[0],
                       motor_cmd.motor[1],
                       motor_cmd.motor[2],
                       motor_cmd.motor[3],
                       esc_throttle);
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