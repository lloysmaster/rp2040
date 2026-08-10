#include "hal/battery/battery.h"
#include "config/battery.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define BATTERY_SAMPLE_PERIOD_US 20000u // 50 Hz
#define BATTERY_FILTER_ALPHA 0.1f

static battery_data_t battery_state = {0};
static float voltage_filtered_v = 0.0f;
static float current_filtered_a = 0.0f;
static float consumed_mah = 0.0f;
static uint32_t last_sample_us = 0;

static float read_adc_volts(uint8_t input) {
    adc_select_input(input);
    const uint16_t raw = adc_read();
    return ((float)raw * BATTERY_ADC_VREF_V) / BATTERY_ADC_MAX;
}

static float read_pack_voltage(void) {
    return read_adc_volts(BATTERY_VOLTAGE_ADC_INPUT) * BATTERY_VOLTAGE_DIVIDER;
}

static float read_pack_current(void) {
#if BATTERY_CURRENT_SENSOR_ENABLED
    const float volts = read_adc_volts(BATTERY_CURRENT_ADC_INPUT) - BATTERY_CURRENT_OFFSET_V;
    const float amps = volts * BATTERY_CURRENT_SCALE_A_V;
    return amps > 0.0f ? amps : 0.0f;
#else
    return 0.0f;
#endif
}

void battery_init(void) {
    adc_init();
    adc_gpio_init(BATTERY_VOLTAGE_ADC_PIN);
#if BATTERY_CURRENT_SENSOR_ENABLED
    adc_gpio_init(BATTERY_CURRENT_ADC_PIN);
#endif

    // Promedio inicial para estabilizar la lectura y detectar el número de celdas
    float sum = 0.0f;
    for (int i = 0; i < 16; ++i) {
        sum += read_pack_voltage();
    }
    voltage_filtered_v = sum / 16.0f;

    uint8_t cells = (uint8_t)((voltage_filtered_v / BATTERY_CELL_MAX_V) + 0.99f);
    if (cells < 1) cells = 1;
    if (cells > 8) cells = 8;
    battery_state.cell_count = cells;

    last_sample_us = time_us_32();
    battery_update();
}

void battery_update(void) {
    const uint32_t now = time_us_32();
    const uint32_t dt_us = now - last_sample_us;
    if (dt_us < BATTERY_SAMPLE_PERIOD_US) {
        return;
    }
    last_sample_us = now;

    const float voltage = read_pack_voltage();
    const float current = read_pack_current();

    voltage_filtered_v += BATTERY_FILTER_ALPHA * (voltage - voltage_filtered_v);
    current_filtered_a += BATTERY_FILTER_ALPHA * (current - current_filtered_a);

    // Integración del consumo: A * h * 1000 = mAh
    consumed_mah += current_filtered_a * ((float)dt_us / 3600000000.0f) * 1000.0f;

    const float cell_v = voltage_filtered_v / (float)battery_state.cell_count;
    float remaining = (cell_v - BATTERY_CELL_MIN_V) / (BATTERY_CELL_MAX_V - BATTERY_CELL_MIN_V) * 100.0f;
    if (remaining < 0.0f) remaining = 0.0f;
    if (remaining > 100.0f) remaining = 100.0f;

    battery_state.voltage_dv = (uint16_t)(voltage_filtered_v * 10.0f + 0.5f);
    battery_state.current_da = (uint16_t)(current_filtered_a * 10.0f + 0.5f);
    battery_state.capacity_mah = (uint32_t)consumed_mah;
    battery_state.remaining_pct = (uint8_t)remaining;
}

const battery_data_t* battery_get_data(void) {
    return &battery_state;
}
