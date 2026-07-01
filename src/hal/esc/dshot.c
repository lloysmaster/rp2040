#include "dshot.h"

#include <stdbool.h>
#include <stdint.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "dshot.pio.h"

static PIO s_pio = pio0;
static int s_sm = -1;
static uint32_t s_pin = 0;
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

static bool dshot_send_frame(uint16_t frame)
{
    if (!s_initialized) {
        return false;
    }

    pio_sm_clear_fifos(s_pio, s_sm);
    pio_sm_restart(s_pio, s_sm);
    pio_sm_put_blocking(s_pio, s_sm, (uint32_t)frame << 16);
    return true;
}

static void dshot_startup_sequence(void)
{
    for (int i = 0; i < 100; ++i) {
        dshot_send_frame(dshot_build_frame(0, false));
        sleep_ms(10);
    }
}

bool dshot_init(uint32_t pin)
{
    if (s_initialized) {
        return true;
    }

    s_pio = pio0;
    int claimed_sm = pio_claim_unused_sm(s_pio, true);
    if (claimed_sm < 0) {
        return false;
    }
    s_sm = claimed_sm;

    uint offset = pio_add_program(s_pio, &dshot_program);
    pio_sm_config cfg = dshot_program_get_default_config(offset);

    pio_gpio_init(s_pio, pin);
    pio_sm_set_consecutive_pindirs(s_pio, s_sm, pin, 1, true);
    sm_config_set_set_pins(&cfg, pin, 1);
    sm_config_set_out_shift(&cfg, false, false, 16);

    float div = (float)clock_get_hz(clk_sys) / 8100000.0f;
    sm_config_set_clkdiv(&cfg, div);

    pio_sm_init(s_pio, s_sm, offset, &cfg);
    pio_sm_set_enabled(s_pio, s_sm, true);

    s_pin = pin;
    s_initialized = true;
    dshot_startup_sequence();
    return true;
}

void dshot_deinit(void)
{
    if (!s_initialized) {
        return;
    }

    pio_sm_set_enabled(s_pio, s_sm, false);
    pio_sm_unclaim(s_pio, (uint)s_sm);
    s_sm = -1;
    s_initialized = false;
}

bool dshot_set_throttle(uint16_t throttle)
{
    if (!s_initialized) {
        return false;
    }

    if (throttle < DSHOT_MIN_THROTTLE) {
        throttle = DSHOT_MIN_THROTTLE;
    } else if (throttle > DSHOT_MAX_THROTTLE) {
        throttle = DSHOT_MAX_THROTTLE;
    }

    uint16_t frame = dshot_build_frame(throttle, false);
    return dshot_send_frame(frame);
}

bool dshot_arm(void)
{
    if (!s_initialized) {
        return false;
    }

    for (int i = 0; i < 5; ++i) {
        dshot_send_frame(dshot_build_frame(0, false));
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
        dshot_send_frame(dshot_build_frame(0, false));
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
    if (command > 47) {
        return false;
    }

    return dshot_send_frame(dshot_build_frame(command, true));
}
