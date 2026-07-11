#include "dshot.h"

#include <stdbool.h>
#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "dshot.pio.h"

static PIO s_pio = pio0;
static int s_sm[4] = { -1, -1, -1, -1 };
static bool s_initialized = false;

static bool dshot_send_raw_packet(uint8_t motor, uint16_t packet_value, bool telemetry);

static uint16_t dshot_build_frame(uint16_t value, bool telemetry)
{
    uint16_t packet = (uint16_t)((value & 0x07FF) << 1) | (telemetry ? 1u : 0u);
    uint16_t checksum = 0;
    uint16_t temp_packet = packet;

    for (int i = 0; i < 3; ++i) {
        checksum ^= (temp_packet & 0x0F);
        temp_packet >>= 4;
    }
    checksum &= 0x0F;

    return (uint16_t)((packet << 4) | checksum);
}

static bool dshot_send_frame_to_sm(int sm, uint16_t frame)
{
    if (!s_initialized || sm < 0) {
        return false;
    }

    pio_sm_clear_fifos(s_pio, sm);
    pio_sm_restart(s_pio, sm);
    pio_sm_put_blocking(s_pio, sm, (uint32_t)frame << 16);
    return true;
}

static bool dshot_send_stop_frames(uint32_t repeats, uint32_t interval_ms)
{
    if (!s_initialized) {
        return false;
    }

    for (uint32_t k = 0; k < repeats; ++k) {
        for (int i = 0; i < 4; ++i) {
            if (!dshot_send_frame_to_sm(s_sm[i], dshot_build_frame(0, false))) {
                return false;
            }
        }
        sleep_ms(interval_ms);
    }

    return true;
}

bool dshot_init(uint32_t base_pin)
{
    if (s_initialized) {
        return true;
    }

    s_pio = pio0;
    // Add program once
    uint offset = pio_add_program(s_pio, &dshot_program);

    // Acquire 4 state machines
    for (int i = 0; i < 4; ++i) {
        int claimed_sm = pio_claim_unused_sm(s_pio, true);
        if (claimed_sm < 0) {
            // Unclaim any previously claimed SMs on failure
            for (int j = 0; j < i; ++j) {
                pio_sm_set_enabled(s_pio, s_sm[j], false);
                pio_sm_unclaim(s_pio, (uint)s_sm[j]);
                s_sm[j] = -1;
            }
            return false;
        }
        s_sm[i] = claimed_sm;

        pio_gpio_init(s_pio, base_pin + i);
        pio_sm_set_consecutive_pindirs(s_pio, s_sm[i], base_pin + i, 1, true);

        pio_sm_config cfg = dshot_program_get_default_config(offset);
        sm_config_set_set_pins(&cfg, base_pin + i, 1);
        sm_config_set_out_shift(&cfg, false, false, 16);

        float div = (float)clock_get_hz(clk_sys) / 8100000.0f;
        sm_config_set_clkdiv(&cfg, div);

        pio_sm_init(s_pio, s_sm[i], offset, &cfg);
        pio_sm_set_enabled(s_pio, s_sm[i], true);
    }

    s_initialized = true;
    return dshot_send_stop_frames(100, 10);
}

bool dshot_set_throttle(uint8_t motor, uint16_t throttle)
{
    if (!s_initialized || motor >= 4) {
        return false;
    }

    if (throttle == 0) {
        return dshot_send_raw_packet(motor, 0, false);
    }

    if (throttle < DSHOT_MIN_THROTTLE) {
        throttle = DSHOT_MIN_THROTTLE;
    } else if (throttle > DSHOT_MAX_THROTTLE) {
        throttle = DSHOT_MAX_THROTTLE;
    }

    return dshot_send_raw_packet(motor, throttle, false);
}

static bool dshot_send_raw_packet(uint8_t motor, uint16_t packet_value, bool telemetry)
{
    if (!s_initialized || motor >= 4 || packet_value > 0x07FF) {
        return false;
    }

    uint16_t frame = dshot_build_frame(packet_value, telemetry);
    return dshot_send_frame_to_sm(s_sm[motor], frame);
}

bool dshot_arm(void)
{
    return dshot_send_stop_frames(5, 20);
}

bool dshot_disarm(void)
{
    return dshot_send_stop_frames(5, 20);
}
