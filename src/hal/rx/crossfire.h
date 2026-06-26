#ifndef CROSSFIRE_H
#define CROSSFIRE_H

#include <stdint.h>
#include <stdbool.h>

#define CRSF_MAX_CHANNELS 16

// Estructura para almacenar los datos del receptor
typedef struct {
    uint16_t channels[CRSF_MAX_CHANNELS];
    bool is_connected;
    uint32_t last_packet_time; // Para failsafe
} crsf_data_t;

// Inicializa el hardware (UART + DMA) para CRSF
void crsf_init(void);

// Función principal que debe ser llamada frecuentemente para procesar el búfer DMA
// Retorna true si se decodificó un paquete válido nuevo
bool crsf_update(void);

// Retorna un puntero a los datos de los canales
const crsf_data_t* crsf_get_data(void);

#endif // CROSSFIRE_H