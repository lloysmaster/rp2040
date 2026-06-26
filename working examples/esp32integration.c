#include <stdio.h>
#include "pico/stdlib.h"
#include "hal/rx/crossfire.h"

int main() {
    stdio_init_all();
    sleep_ms(2000); 

    printf("Pico FC iniciada. Configurando DMA y escuchando...\n");
    
    crsf_init();

    while (true) {
        // Ejecutamos crsf_update() de forma no bloqueante.
        // Si hay una trama completa y validada, retorna true.
        if (crsf_update()) {
            const crsf_data_t* rc_data = crsf_get_data();
            
            // CRSF entrega valores entre ~172 y ~1811 (centro en 992)
            printf("CH1: %d | CH2: %d | CH3: %d | CH4: %d\n", 
                   rc_data->channels[0], 
                   rc_data->channels[1], 
                   rc_data->channels[2], 
                   rc_data->channels[3]);
        }
        
        // Aquí podés poner tu lazo de control (ej. lectura MPU6500 SPI, cálculo PID, etc.)
        // sin preocuparte por bloquear la recepción de la UART.
    }
    return 0;
}