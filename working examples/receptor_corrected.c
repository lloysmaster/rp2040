#include <esp_now.h>
#include <WiFi.h>

#define CRSF_BAUDRATE 420000

// Canales (Centro = 992)
uint16_t channels[16] = {992, 992, 992, 992, 992, 992, 992, 992, 
                         992, 992, 992, 992, 992, 992, 992, 992}; 

// Estructura para recibir datos vía ESP-NOW
typedef struct struct_message {
    uint16_t ch[16];
} struct_message;

struct_message incomingData;

// Callback: Se ejecuta cuando llega un paquete ESP-NOW
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incoming_data, int len) {
    memcpy(&incomingData, incoming_data, sizeof(incomingData));
    
    // Actualizamos nuestro array local de canales
    for(int i = 0; i < 16; i++) {
        channels[i] = incomingData.ch[i];
    }
    
    // Debug: mostrar throttle (canal 2)
    Serial.print("Throttle: ");
    Serial.println(channels[2]);
}

// Función estándar CRC8 para CRSF (Polinomio 0xD5)
uint8_t crsf_crc8(uint8_t *ptr, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= ptr[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc <<= 1;
        }
    }
    return crc;
}

void enviarTramaCRSF() {
    uint8_t frame[26];
    
    frame[0] = 0xC8; // Sync
    frame[1] = 24;   // Length 
    frame[2] = 0x16; // Type: RC Channels Packed
    
    // Empaquetado de 16 canales de 11 bits cada uno
    uint32_t bits = 0;
    uint8_t bits_available = 0;
    uint8_t byte_idx = 3;

    for (int i = 0; i < 16; i++) {
        bits |= (channels[i] & 0x07FF) << bits_available;
        bits_available += 11;
        
        while (bits_available >= 8) {
            frame[byte_idx++] = bits & 0xFF;
            bits >>= 8;
            bits_available -= 8;
        }
    }

    frame[25] = crsf_crc8(&frame[1], 24);  // ✅ CORREGIDO: CRC desde frame[1], no frame[2]
    Serial2.write(frame, 26);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(CRSF_BAUDRATE, SERIAL_8N1, 16, 17);
    
    Serial.print("MAC del Receptor: ");
    Serial.println(WiFi.macAddress());
    
    // Configurar ESP-NOW
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error inicializando ESP-NOW");
        return;
    }
    
    // Registrar callback para recibir paquetes broadcast
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("Receptor ESP-NOW inicializado (escuchando broadcast). Enviando CRSF a 100Hz...");
}

unsigned long lastSendTime = 0;

void loop() {
    // Enviar la trama CRSF a 100Hz (cada 10ms)
    if (millis() - lastSendTime >= 10) {
        enviarTramaCRSF();
        lastSendTime = millis();
    }
}
