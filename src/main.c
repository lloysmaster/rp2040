#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "config/pinout.h"
#include "control/mixer.h"
#include "core/attitude.h"
#include "hal/esc/dshot.h"
#include "hal/mpu/mpu.h"
#include "hal/rx/crossfire.h"
#include "math/fixed_point.h"

static volatile bool data_ready = false;

// Callback de la interrupción externa del GPIO (MPU DRDY)
static void gpio_callback(uint gpio, uint32_t events) {
    (void)events;
    if (gpio == PIN_DRDY) {
        data_ready = true;
    }
}

int main(void) {
    // --- Inicialización del sensor MPU6500 ---
    const mpu_config_t cfg = {
        .spi = MPU_SPI_INST,
        .pin_cs = PIN_CS,
        .pin_sck = PIN_SCK,
        .pin_mosi = PIN_MOSI,
        .pin_miso = PIN_MISO,
        .pin_drdy = PIN_DRDY,
        .gyro_sensitivity_lsb_per_dps = 131
    };

    mpu_init(&cfg);
    mpu_enable_drdy();

    // Habilitar interrupción física en el flanco de subida del pin DRDY
    gpio_set_irq_enabled_with_callback(PIN_DRDY, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // --- Inicialización del receptor Crossfire ---
    crsf_init();
    attitude_init();
    if (!dshot_init(MOTOR_BASE_PIN)) {
        return 1;
    }
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
        crsf_update();
        const crsf_data_t* rc_data = crsf_get_data();

        // 2. Procesar datos del giroscopio
        if (data_ready) {
            data_ready = false;

            q16_16 gyro_data[3];
            mpu_read_gyro_fixed(gyro_data);

            attitude_update(rc_data, gyro_data, &last_attitude_cmd);
            mixer_mix(&last_attitude_cmd, &last_motor_cmd);
        }

        for (int i = 0; i < 4; ++i) {
            if (!esc_armed) {
                esc_throttle[i] = 0;
            } else {
                uint32_t val = (uint32_t)((last_motor_cmd.motor[i] * DSHOT_MAX_THROTTLE) / 1000u);
                if (val > DSHOT_MAX_THROTTLE) {
                    val = DSHOT_MAX_THROTTLE;
                }
                if (val < DSHOT_MIN_THROTTLE) {
                    val = DSHOT_MIN_THROTTLE;
                }
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
                for (int i = 0; i < 4; ++i) {
                    dshot_set_throttle(i, 0);
                }
            } else {
                if (!arm_switch) {
                    if (dshot_disarm()) {
                        esc_armed = false;
                    }
                    for (int i = 0; i < 4; ++i) {
                        dshot_set_throttle(i, 0);
                    }
                } else {
                    for (int i = 0; i < 4; ++i) {
                        dshot_set_throttle(i, esc_throttle[i]);
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
