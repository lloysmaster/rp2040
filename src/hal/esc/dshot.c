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
static uint32_t s_pin_base = 0;
static bool s_initialized = false;
static bool s_armed = false;

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

static void dshot_startup_sequence(void)
{
    for (int k = 0; k < 100; ++k) {
        for (int i = 0; i < 4; ++i) {
            dshot_send_frame_to_sm(s_sm[i], dshot_build_frame(0, false));
        }
        sleep_ms(10);
    }
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

    s_pin_base = base_pin;
    s_initialized = true;
    dshot_startup_sequence();
    return true;
}

void dshot_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    for (int i = 0; i < 4; ++i) {
        if (s_sm[i] >= 0) {
            pio_sm_set_enabled(s_pio, s_sm[i], false);
            pio_sm_unclaim(s_pio, (uint)s_sm[i]);
            s_sm[i] = -1;
        }
    }

    s_initialized = false;
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

bool dshot_send_raw_packet(uint8_t motor, uint16_t packet_value, bool telemetry)
{
    if (!s_initialized || motor >= 4 || packet_value > 0x07FF) {
        return false;
    }

    uint16_t frame = dshot_build_frame(packet_value, telemetry);
    return dshot_send_frame_to_sm(s_sm[motor], frame);
}

bool dshot_arm(void)
{
    if (!s_initialized) {
        return false;
    }

    for (int i = 0; i < 5; ++i) {
        for (int m = 0; m < 4; ++m) {
            dshot_send_frame_to_sm(s_sm[m], dshot_build_frame(0, false));
        }
        sleep_ms(20);
    }

    s_armed = true;
    return true;
}

bool dshot_disarm(void)
{
    if (!s_initialized) {
        return false;
    }

    for (int i = 0; i < 5; ++i) {
        for (int m = 0; m < 4; ++m) {
            dshot_send_frame_to_sm(s_sm[m], dshot_build_frame(0, false));
        }
        sleep_ms(20);
    }

    s_armed = false;
    return true;
}

bool dshot_is_armed(void)
{
    return s_initialized && s_armed;
}

bool dshot_send_command(uint16_t command)
{
    if (command > 47 || !s_initialized) {
        return false;
    }

    for (int m = 0; m < 4; ++m) {
        if (!dshot_send_raw_packet((uint8_t)m, command, true)) {
            return false;
        }
    }
    return true;
}
