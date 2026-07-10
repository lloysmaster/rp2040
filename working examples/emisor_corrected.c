#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

const int joyXPin = 34;       // VRx del joystick principal (Roll)
const int joyYPin = 35;       // VRy del joystick principal (Pitch)

const int joy2XPin = 32;      // VRx del segundo stick (Yaw)
const int joy2YPin = 33;      // VRy del segundo stick (Throttle)

const int armSwitchPinLeft = 25;   // Lado izquierdo del switch de armado (CH4 -> 1000)
const int armSwitchPinRight = 26;  // Lado derecho del switch de armado (CH4 -> 2000)

// Mapeo de canales RC para este firmware:
// CH0 -> Roll  (stick principal eje X)
// CH1 -> Pitch (stick principal eje Y)
// CH2 -> Throttle (segundo stick eje Y)
// CH3 -> Yaw    (segundo stick eje X)
// CH4 -> Arm/Disarm (switch con centro a GND y lados a GPIO)

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

uint16_t map_rc_channel(int raw) {
  return (uint16_t)map(raw, 0, 4095, 172, 1811);
}

uint16_t read_arm_switch_channel() {
  bool left = (digitalRead(armSwitchPinLeft) == LOW);
  bool right = (digitalRead(armSwitchPinRight) == LOW);

  if (left) return 1000;
  if (right) return 2000;
  return 1500;
}

void setup() {
  Serial.begin(115200);
  pinMode(armSwitchPinLeft, INPUT_PULLUP);
  pinMode(armSwitchPinRight, INPUT_PULLUP);

  // Debug: Mostrar MAC del emisor
  Serial.print("MAC del Emisor: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("No se pudo registrar el peer de ESP-NOW");
    return;
  }

  Serial.println("ESP-NOW inicializado correctamente");
}

void loop() {
  uint16_t channels[16] = {0};

  int rawX = analogRead(joyXPin);
  int rawY = analogRead(joyYPin);
  int raw2X = analogRead(joy2XPin);
  int raw2Y = analogRead(joy2YPin);
  uint16_t armSwitchValue = read_arm_switch_channel();

  // Stick principal: Roll/Pitch
  channels[0] = map_rc_channel(rawX);     // CH0 Roll
  channels[1] = map_rc_channel(rawY);     // CH1 Pitch

  // Segundo stick: Throttle/Yaw
  channels[2] = map_rc_channel(raw2Y);  // CH2 Throttle (invertido para que subir suba el throttle)
  channels[3] = map_rc_channel(raw2X);         // CH3 Yaw

  // Switch de armado
  channels[4] = armSwitchValue;           // CH4 Arm/Disarm

  for (int i = 5; i < 16; i++) {
    channels[i] = 992;  // Resto - Centro
  }

  Serial.print("CH0:");
  Serial.print(channels[0]);
  Serial.print(" CH1:");
  Serial.print(channels[1]);
  Serial.print(" CH2:");
  Serial.print(channels[2]);
  Serial.print(" CH3:");
  Serial.print(channels[3]);
  Serial.print(" CH4:");
  Serial.println(channels[4]);
  Serial.print("Raw X:");
  Serial.print(rawX);
  Serial.print(" Raw Y:");
  Serial.print(rawY);
  Serial.print(" Raw 2Y:");
  Serial.print(raw2Y);
  Serial.print(" Raw 2X:");
  Serial.println(raw2X);

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

  packet[25] = crc8(&packet[2], packet[1] - 1);

  // Enviar por ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, packet, sizeof(packet));

  if (result == ESP_OK) {
    Serial.println("Trama enviada correctamente");
  } else {
    Serial.print("Error enviando: ");
    Serial.println(result);
  }

  delay(20);
}
