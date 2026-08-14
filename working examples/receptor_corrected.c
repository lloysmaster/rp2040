#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>

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

// ===== Calidad del enlace ESP-NOW, medida en el receptor =====
// El FC no genera tramas LINK_STATISTICS: el enlace de radio real es el
// ESP-NOW entre emisor y receptor, por lo que estas metricas se calculan aqui.
#define RC_PACKET_PERIOD_MS   20   // el emisor envia canales a 50Hz
#define LINK_WINDOW_MS        1000 // ventana de medicion de LQ
#define LINK_EXPECTED_PACKETS (LINK_WINDOW_MS / RC_PACKET_PERIOD_MS)
#define LINK_TIMEOUT_MS       500  // sin paquetes RC => enlace caido

volatile int8_t   uplinkRSSI = 0;          // dBm del ultimo paquete RC recibido
volatile int8_t   uplinkNoiseFloor = -96;  // dBm de piso de ruido informado por el PHY
volatile uint16_t uplinkPacketsInWindow = 0;
volatile uint32_t lastRcRxMillis = 0;
volatile uint16_t telemetrySentInWindow = 0;
volatile uint16_t telemetryAckedInWindow = 0;

uint32_t lastLinkWindowMillis = 0;
uint8_t  uplinkQuality = 0;   // % de paquetes RC recibidos respecto de los esperados
uint8_t  downlinkQuality = 0; // % de tramas de telemetria aceptadas por el radio
uint8_t  txPowerDbm = 0;
bool     linkUp = false;

// ---------- ESP-NOW: recepcion de canales RC desde el emisor ----------
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incoming_data, int len) {
  if (len < 1) return;
  if (incoming_data[0] == 0x01 && len == sizeof(rc_packet_t)) {
    rc_packet_t pkt;
    memcpy(&pkt, incoming_data, sizeof(pkt));
    for (int i = 0; i < 16; i++) channels[i] = pkt.ch[i];

    if (info != NULL && info->rx_ctrl != NULL) {
      uplinkRSSI = info->rx_ctrl->rssi;
      uplinkNoiseFloor = info->rx_ctrl->noise_floor;
    }
    uplinkPacketsInWindow++;
    lastRcRxMillis = millis();

    Serial.print("Throttle: ");
    Serial.println(channels[2]);
  }
}

// ---------- ESP-NOW: resultado del envio de telemetria (calidad de bajada) ----------
// Con direccion de broadcast el radio no espera ACK, por lo que este porcentaje
// refleja que la capa de radio acepto la trama, no que el emisor la recibio.
// La firma del callback cambio en IDF 5.4 (core ESP32 3.2+): antes recibia la MAC
// del destino, ahora una estructura con la informacion de la transmision.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 4, 0)
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
  telemetrySentInWindow++;
  if (status == ESP_NOW_SEND_SUCCESS) telemetryAckedInWindow++;
}

static uint8_t percent(uint32_t part, uint32_t total) {
  if (total == 0) return 0;
  uint32_t pct = (part * 100) / total;
  return pct > 100 ? 100 : (uint8_t)pct;
}

// Potencia de transmision del radio WiFi en dBm (el driver la reporta en 0.25dBm)
static uint8_t leerPotenciaTxDbm() {
  int8_t power_quarter_dbm = 0;
  if (esp_wifi_get_max_tx_power(&power_quarter_dbm) != ESP_OK) return 0;
  int power_dbm = power_quarter_dbm / 4;
  return power_dbm < 0 ? 0 : (uint8_t)power_dbm;
}

// ---------- Calculo de las metricas de calidad de senal ----------
void actualizarCalidadEnlace() {
  uint32_t now = millis();

  if (now - lastLinkWindowMillis >= LINK_WINDOW_MS) {
    uint16_t rcRx = uplinkPacketsInWindow;
    uint16_t telSent = telemetrySentInWindow;
    uint16_t telAcked = telemetryAckedInWindow;
    uplinkPacketsInWindow = 0;
    telemetrySentInWindow = 0;
    telemetryAckedInWindow = 0;
    lastLinkWindowMillis = now;

    uplinkQuality = percent(rcRx, LINK_EXPECTED_PACKETS);
    downlinkQuality = percent(telAcked, telSent);
    txPowerDbm = leerPotenciaTxDbm();
  }

  linkUp = (lastRcRxMillis != 0) && (now - lastRcRxMillis < LINK_TIMEOUT_MS);

  if (!linkUp) {
    uplinkQuality = 0;
    telemetry.linkRSSI1 = 0;
    telemetry.linkRSSI2 = 0;
    telemetry.linkQuality = 0;
    telemetry.linkSNR = 0;
    telemetry.linkTXPower = txPowerDbm;
    return;
  }

  int8_t rssi = uplinkRSSI;
  int8_t noise = uplinkNoiseFloor;
  int snr = (int)rssi - (int)noise;
  if (snr < -128) snr = -128;
  if (snr > 127) snr = 127;

  telemetry.linkRSSI1   = rssi < 0 ? (uint8_t)(-rssi) : 0; // convencion CRSF: dBm en positivo
  telemetry.linkRSSI2   = 0;                               // el ESP32 expone una sola antena
  telemetry.linkQuality = uplinkQuality;
  telemetry.linkSNR     = (int8_t)snr;
  telemetry.linkTXPower = txPowerDbm;
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
  // El CRC cubre tipo + payload (no el sync ni el byte de largo)
  frame[25] = crsf_crc8(&frame[2], 23);
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

    // 0x14 LINK_STATISTICS no se parsea: el enlace de radio es el ESP-NOW de este
    // receptor, asi que las metricas de calidad se miden localmente y se envian al FC.

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

// ---------- Envio de LINK_STATISTICS al FC (calidad del enlace RC) ----------
void enviarLinkStatsCRSF() {
  uint8_t frame[14];
  frame[0] = 0xC8; // Sync
  frame[1] = 12;   // Largo: tipo + 10 bytes de payload + crc
  frame[2] = 0x14; // Tipo: LINK_STATISTICS

  frame[3]  = telemetry.linkRSSI1;   // RSSI de subida antena 1 (dBm positivo)
  frame[4]  = telemetry.linkRSSI2;   // RSSI de subida antena 2
  frame[5]  = telemetry.linkQuality; // LQ de subida en %
  frame[6]  = (uint8_t)telemetry.linkSNR;
  frame[7]  = 0;                     // antena activa (unica)
  frame[8]  = 0;                     // modo RF (no aplica en ESP-NOW)
  frame[9]  = telemetry.linkTXPower; // potencia de transmision en dBm
  frame[10] = telemetry.linkRSSI1;   // RSSI de bajada: mismo enlace fisico
  frame[11] = downlinkQuality;       // LQ de bajada (telemetria confirmada)
  frame[12] = (uint8_t)telemetry.linkSNR;

  frame[13] = crsf_crc8(&frame[2], 11);
  Serial2.write(frame, 14);
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
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 1;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("No se pudo registrar el peer de ESP-NOW");
    return;
  }

  txPowerDbm = leerPotenciaTxDbm();
  lastLinkWindowMillis = millis();

  Serial.println("Receptor listo: RC por ESP-NOW -> CRSF al FC, y telemetria CRSF -> ESP-NOW al emisor.");
}

unsigned long lastSendTime = 0;
unsigned long lastLinkStatsSend = 0;
void loop() {
  leerTelemetriaCRSF();       // 1. Leer telemetria que llega del FC por Serial2
  actualizarCalidadEnlace();  // 2. Medir RSSI/LQ/SNR del enlace ESP-NOW
  enviarTelemetriaESPNOW();   // 3. Reenviar la telemetria al emisor cuando corresponda

  if (millis() - lastSendTime >= 10) { // 4. Enviar canales RC al FC a 100Hz
    enviarTramaCRSF();
    lastSendTime = millis();
  }

  if (millis() - lastLinkStatsSend >= 100) { // 5. Informar la calidad del enlace al FC a 10Hz
    enviarLinkStatsCRSF();
    lastLinkStatsSend = millis();
  }
}
