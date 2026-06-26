#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#define UART_ID uart0
#define BAUD_RATE 420000 
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Buffer para guardar la trama sin bloquear el procesador
uint8_t buffer[26];
int idx = 0;

int main() {
    stdio_init_all();
    uart_init(UART_ID, BAUD_RATE);
    
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_fifo_enabled(UART_ID, true);

    sleep_ms(2000); 
    printf("Pico FC iniciada. Escuchando CRSF a %d baudios...\n", BAUD_RATE);

    while (true) {
        if (uart_is_readable(UART_ID)) {
            uint8_t ch = uart_getc(UART_ID);
            
            // Máquina de estados básica para sincronizar la trama
            if (idx == 0) {
                if (ch == 0xC8) { // Si es el byte Sync de CRSF, empezamos a guardar
                    buffer[idx++] = ch;
                }
            } else {
                buffer[idx++] = ch; // Guardamos el resto de los bytes
                
                if (idx == 26) {
                    // Ya tenemos los 26 bytes. Los imprimimos todos de una sola vez.
                    printf("Trama CRSF: ");
                    for(int i = 0; i < 26; i++) {
                        printf("%02X ", buffer[i]);
                    }
                    printf("\n");
                    
                    idx = 0; // Reiniciamos el índice para la próxima trama
                }
            }
        }
    }
    return 0;
}