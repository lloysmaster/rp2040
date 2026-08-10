#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CRSF_BAUDRATE 420000

// ===== Paquete de canales RC recibido por ESP-NOW (desde el emisor) =====
typedef struct __attribute__((packed)) {
  uint8_t type;        // 0x01 = canales RC
  uint16_t ch[16];
} rc_packet_t;

// ===== Paquete de telemetria enviado por ESP-NOW (hacia el emisor) =====
typedef struct __attribute__((packed)) {
  uint8_t type;         // 0x02 = telemetria
  float batteryVoltage;
  float batteryCurrent;
  uint32_t batteryCapacity;
  uint8_t batteryRemaining;
  float gpsLat;
  float gpsLon;
  float gpsSpeed;
  float gpsHeading;
  int16_t gpsAltitude;
  uint8_t gpsSatellites;
  float attitudePitch;
  float attitudeRoll;
  float attitudeYaw;
  uint8_t linkRSSI1;
  uint8_t linkRSSI2;
  uint8_t linkQuality;
  int8_t  linkSNR;
  uint8_t linkTXPower;
  char flightMode[16];
} telemetry_packet_t;

uint16_t channels[16] = {992, 992, 992, 992, 992, 992, 992, 992,
                          992, 992, 992, 992, 992, 992, 992, 992};

telemetry_packet_t telemetry = {0x02};

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------- ESP-NOW: recepcion de canales RC desde el emisor ----------
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incoming_data, int len) {
  if (len < 1) return;
  if (incoming_data[0] == 0x01 && len == sizeof(rc_packet_t)) {
    rc_packet_t pkt;
    memcpy(&pkt, incoming_data, sizeof(pkt));
    for (int i = 0; i < 16; i++) channels[i] = pkt.ch[i];

    Serial.print("Throttle: ");
    Serial.println(channels[2]);
  }
}

// ---------- CRC8 CRSF (poly 0xD5) ----------
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) {
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

// ---------- Empaquetado y envio de canales CRSF hacia el FC ----------
void enviarTramaCRSF() {
  uint8_t frame[26];
  frame[0] = 0xC8; // Sync
  frame[1] = 24;   // Largo
  frame[2] = 0x16; // Tipo: RC_CHANNELS_PACKED

  uint32_t bits = 0;
  uint8_t bits_available = 0;
  uint8_t byte_idx = 3;
  for (int i = 0; i < 16; i++) {
    bits |= (uint32_t)(channels[i] & 0x07FF) << bits_available;
    bits_available += 11;
    while (bits_available >= 8) {
      frame[byte_idx++] = bits & 0xFF;
      bits >>= 8;
      bits_available -= 8;
    }
  }
  frame[25] = crsf_crc8(&frame[1], 24);
  Serial2.write(frame, 26);
}

// ---------- Lectura y parseo de telemetria CRSF entrante (desde el FC) ----------
#define CRSF_MAX_FRAME_LEN 64
uint8_t crsfBuf[CRSF_MAX_FRAME_LEN];
uint8_t crsfIdx = 0;
uint8_t crsfLen = 0;
enum { CRSF_WAIT_SYNC, CRSF_WAIT_LEN, CRSF_WAIT_DATA } crsfState = CRSF_WAIT_SYNC;

void parseCRSFFrame(uint8_t *frame, uint8_t frameLen) {
  // frame[0] = tipo ; frame[1..frameLen-2] = payload ; frame[frameLen-1] = crc
  uint8_t type = frame[0];
  uint8_t *payload = &frame[1];
  uint8_t payloadLen = frameLen - 2;

  uint8_t crcCalc = crsf_crc8(frame, frameLen - 1);
  if (crcCalc != frame[frameLen - 1]) return; // trama corrupta, descartar

  switch (type) {
    case 0x08: // BATTERY_SENSOR
      if (payloadLen >= 8) {
        telemetry.batteryVoltage   = ((payload[0] << 8) | payload[1]) / 10.0f;
        telemetry.batteryCurrent   = ((payload[2] << 8) | payload[3]) / 10.0f;
        telemetry.batteryCapacity  = ((uint32_t)payload[4] << 16) | (payload[5] << 8) | payload[6];
        telemetry.batteryRemaining = payload[7];
      }
      break;

    case 0x02: // GPS
      if (payloadLen >= 15) {
        int32_t lat = ((int32_t)payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3];
        int32_t lon = ((int32_t)payload[4] << 24) | (payload[5] << 16) | (payload[6] << 8) | payload[7];
        telemetry.gpsLat       = lat / 1e7f;
        telemetry.gpsLon       = lon / 1e7f;
        telemetry.gpsSpeed     = ((payload[8] << 8) | payload[9]) / 10.0f;     // km/h
        telemetry.gpsHeading   = ((payload[10] << 8) | payload[11]) / 100.0f; // grados
        telemetry.gpsAltitude  = (int16_t)(((payload[12] << 8) | payload[13]) - 1000); // metros
        telemetry.gpsSatellites = payload[14];
      }
      break;

    case 0x1E: // ATTITUDE
      if (payloadLen >= 6) {
        int16_t pitch = (payload[0] << 8) | payload[1];
        int16_t roll  = (payload[2] << 8) | payload[3];
        int16_t yaw   = (payload[4] << 8) | payload[5];
        telemetry.attitudePitch = pitch / 10000.0f; // radianes
        telemetry.attitudeRoll  = roll  / 10000.0f;
        telemetry.attitudeYaw   = yaw   / 10000.0f;
      }
      break;

    case 0x14: // LINK_STATISTICS
      if (payloadLen >= 10) {
        telemetry.linkRSSI1   = payload[0];
        telemetry.linkRSSI2   = payload[1];
        telemetry.linkQuality = payload[2];
        telemetry.linkSNR     = (int8_t)payload[3];
        telemetry.linkTXPower = payload[6];
      }
      break;

    case 0x21: // FLIGHT_MODE (string ASCII terminada en \0)
      {
        uint8_t n = payloadLen < sizeof(telemetry.flightMode) - 1
                      ? payloadLen
                      : sizeof(telemetry.flightMode) - 1;
        memcpy(telemetry.flightMode, payload, n);
        telemetry.flightMode[n] = '\0';
      }
      break;

    default:
      break; // otros tipos de trama se ignoran (heartbeat, device info, etc)
  }
}

void leerTelemetriaCRSF() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();

    switch (crsfState) {
      case CRSF_WAIT_SYNC:
        // 0xC8 = direccion FC, 0xEA = direccion Radio/TX (depende del firmware)
        if (b == 0xC8 || b == 0xEA) {
          crsfIdx = 0;
          crsfState = CRSF_WAIT_LEN;
        }
        break;

      case CRSF_WAIT_LEN:
        crsfLen = b; // largo = tipo + payload + crc
        if (crsfLen < 2 || crsfLen > CRSF_MAX_FRAME_LEN - 1) {
          crsfState = CRSF_WAIT_SYNC; // largo invalido, reiniciar
        } else {
          crsfIdx = 0;
          crsfState = CRSF_WAIT_DATA;
        }
        break;

      case CRSF_WAIT_DATA:
        crsfBuf[crsfIdx++] = b;
        if (crsfIdx >= crsfLen) {
          parseCRSFFrame(crsfBuf, crsfLen);
          crsfState = CRSF_WAIT_SYNC;
        }
        break;
    }
  }
}

// ---------- Envio periodico de telemetria al emisor por ESP-NOW ----------
unsigned long lastTelemetrySend = 0;
void enviarTelemetriaESPNOW() {
  if (millis() - lastTelemetrySend >= 50) { // ~20Hz, de sobra para telemetria
    esp_now_send(broadcastAddress, (uint8_t*)&telemetry, sizeof(telemetry));
    lastTelemetrySend = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(CRSF_BAUDRATE, SERIAL_8N1, 16, 17);

  Serial.print("MAC del Receptor: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); // debe coincidir con el emisor

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("No se pudo registrar el peer de ESP-NOW");
    return;
  }

  Serial.println("Receptor listo: RC por ESP-NOW -> CRSF al FC, y telemetria CRSF -> ESP-NOW al emisor.");
}

unsigned long lastSendTime = 0;
void loop() {
  leerTelemetriaCRSF();       // 1. Leer telemetria que llega del FC por Serial2
  enviarTelemetriaESPNOW();   // 2. Reenviarla al emisor cuando corresponda

  if (millis() - lastSendTime >= 10) { // 3. Enviar canales RC al FC a 100Hz
    enviarTramaCRSF();
    lastSendTime = millis();
  }
}
