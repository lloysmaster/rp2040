#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint16_t voltage_dv;   // décimas de voltio (formato CRSF)
    uint16_t current_da;   // décimas de amperio (formato CRSF)
    uint32_t capacity_mah; // mAh consumidos desde el arranque
    uint8_t remaining_pct; // 0..100 %
    uint8_t cell_count;    // celdas detectadas al encender
} battery_data_t;

// Configura el ADC para medir tensión (y corriente si está habilitada)
void battery_init(void);

// Muestrea el ADC e integra el consumo. Llamar periódicamente desde el lazo.
void battery_update(void);

const battery_data_t* battery_get_data(void);

#endif // BATTERY_H
