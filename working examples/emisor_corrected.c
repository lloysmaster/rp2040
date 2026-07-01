#include <esp_now.h>
#include <WiFi.h>

const int potPin = 34;

// ✅ Broadcast - Funciona sin necesidad de conocer la MAC del receptor
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 

uint8_t crc8(const uint8_t *ptr, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= *ptr++;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
      else crc <<= 1;
    }
  }
  return crc;
}

void setup() {
  Serial.begin(115200);
  
  // Debug: Mostrar MAC del emisor
  Serial.print("MAC del Emisor: ");
  Serial.println(WiFi.macAddress());
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  Serial.println("ESP-NOW inicializado correctamente");
}

void loop() {
  uint16_t channels[16];
  
  // ✅ CAMBIO: Potenciómetro en canal 2 (THROTTLE)
  channels[0] = 992;  // Roll - Centro
  channels[1] = 992;  // Pitch - Centro
  channels[2] = map(analogRead(potPin), 0, 4095, 172, 1811);  // Throttle - Del potenciómetro
  channels[3] = 992;  // Yaw - Centro
  
  for(int i = 4; i < 16; i++) {
    channels[i] = 992;  // Resto - Centro
  }

  uint8_t packet[26];
  packet[0] = 0xC8;       // Sync
  packet[1] = 24;         // Largo
  packet[2] = 0x16;       // Tipo RC_CHANNELS_PACKED

  uint8_t *p = &packet[3];
  p[0] = (uint8_t)(channels[0] & 0xFF);
  p[1] = (uint8_t)((channels[0] >> 8) | (channels[1] << 3));
  p[2] = (uint8_t)((channels[1] >> 5) | (channels[2] << 6));
  p[3] = (uint8_t)(channels[2] >> 2);
  p[4] = (uint8_t)((channels[2] >> 10) | (channels[3] << 1));
  p[5] = (uint8_t)((channels[3] >> 7) | (channels[4] << 4));
  p[6] = (uint8_t)((channels[4] >> 4) | (channels[5] << 7));
  p[7] = (uint8_t)(channels[5] >> 1);
  p[8] = (uint8_t)((channels[5] >> 9) | (channels[6] << 2));
  p[9] = (uint8_t)((channels[6] >> 6) | (channels[7] << 5));
  p[10] = (uint8_t)(channels[7] >> 3);
  p[11] = (uint8_t)((channels[8] & 0xFF));
  p[12] = (uint8_t)((channels[8] >> 8) | (channels[9] << 3));
  p[13] = (uint8_t)((channels[9] >> 5) | (channels[10] << 6));
  p[14] = (uint8_t)(channels[10] >> 2);
  p[15] = (uint8_t)((channels[10] >> 10) | (channels[11] << 1));
  p[16] = (uint8_t)((channels[11] >> 7) | (channels[12] << 4));
  p[17] = (uint8_t)((channels[12] >> 4) | (channels[13] << 7));
  p[18] = (uint8_t)(channels[13] >> 1);
  p[19] = (uint8_t)((channels[13] >> 9) | (channels[14] << 2));
  p[20] = (uint8_t)((channels[14] >> 6) | (channels[15] << 5));
  p[21] = (uint8_t)(channels[15] >> 3);
  
  packet[25] = crc8(&packet[1], 24);

  // Enviar por ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, packet, 26);
  
  if (result == ESP_OK) {
    Serial.println("Trama enviada correctamente");
  } else {
    Serial.print("Error enviando: ");
    Serial.println(result);
  }

  delay(20); 
}
