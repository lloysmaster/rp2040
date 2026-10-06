#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "BluetoothSerial.h"

// Verificar si Bluetooth está habilitado en el ESP32
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Run make menuconfig to and enable it
#endif

BluetoothSerial SerialBT;

// --- Pines ---
const int joyXPin = 34;   // CH0: Roll
const int joyYPin = 35;   // CH1: Pitch
const int joy2XPin = 32;  // CH3: Yaw
const int joy2YPin = 33;  // CH2: Throttle

// Pin único para el switch de armado (derecho)
const int armSwitchPinRight = 26;

// =========================================================================
// CALIBRACIÓN DE POTENCIÓMETROS
// =========================================================================
const int CH0_MIN = 445;   
const int CH0_MAX = 3265;  

const int CH1_MIN = 3750;  
const int CH1_MAX = 805;   

const int CH2_MIN = 3205;  
const int CH2_MAX = 440;   

const int CH3_MIN = 705;   
const int CH3_MAX = 3935;
// =========================================================================

String currentLatitude = "-34.603722";
String currentLongitude = "-58.381592";
float pidKp = 1.0;
float pidKi = 0.0;
float pidKd = 0.0;

// Bandera para indicar que cambió el PID y debe enviarse por ESP-NOW
bool pidChanged = false;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ===== Paquete de canales RC enviado por ESP-NOW =====
typedef struct __attribute__((packed)) {
  uint8_t type;        // 0x01 = canales RC
  uint16_t ch[16];
} rc_packet_t;

// ===== Paquete de configuración PID enviado por ESP-NOW =====
typedef struct __attribute__((packed)) {
  uint8_t type;        // 0x03 = configuración PID
  float kp;
  float ki;
  float kd;
} pid_packet_t;

// ===== Paquete de telemetría recibido por ESP-NOW =====
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

telemetry_packet_t telemetry = {0x02};
volatile bool telemetryValid = false;
unsigned long lastTelemetryRxMillis = 0;

// ---------- ESP-NOW: recepción de telemetría ----------
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incoming_data, int len) {
  if (len < 1) return;
  if (incoming_data[0] == 0x02 && len == sizeof(telemetry_packet_t)) {
    memcpy(&telemetry, incoming_data, sizeof(telemetry));
    telemetryValid = true;
    lastTelemetryRxMillis = millis();
  }
}

uint16_t map_rc_channel(int raw, int minRaw, int maxRaw) {
  if (minRaw < maxRaw) {
    raw = constrain(raw, minRaw, maxRaw);
  } else {
    raw = constrain(raw, maxRaw, minRaw);
  }
  return (uint16_t)map(raw, minRaw, maxRaw, 172, 1811);
}

// Lectura de armado con resistencia interna PULLUP
uint16_t read_arm_switch_channel() {
  if (digitalRead(armSwitchPinRight) == LOW) {
    return 1811; // Armado (CRSF Max)
  } else {
    return 992;  // Desarmado (CRSF Min)
  }
}

void handleBluetoothCommands() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();

    if (command == "get_home") {
      SerialBT.print("home:");
      SerialBT.print(currentLatitude);
      SerialBT.print(",");
      SerialBT.println(currentLongitude);
    }
    else if (command == "get_position") {
      SerialBT.print("pos:");
      SerialBT.print(currentLatitude);
      SerialBT.print(",");
      SerialBT.println(currentLongitude);
    }
    else if (command == "get_pid") {
      SerialBT.print("pid:");
      SerialBT.print(pidKp);
      SerialBT.print(",");
      SerialBT.print(pidKi);
      SerialBT.print(",");
      SerialBT.println(pidKd);
    }
    else if (command.startsWith("set_pid:")) {
      int firstComma = command.indexOf(',', 8);
      int secondComma = command.indexOf(',', firstComma + 1);
      
      if (firstComma != -1 && secondComma != -1) {
        pidKp = command.substring(8, firstComma).toFloat();
        pidKi = command.substring(firstComma + 1, secondComma).toFloat();
        pidKd = command.substring(secondComma + 1).toFloat();
        pidChanged = true; // Activamos la bandera para enviarlo por ESP-NOW
      }
      
      SerialBT.print("pid:");
      SerialBT.print(pidKp);
      SerialBT.print(",");
      SerialBT.print(pidKi);
      SerialBT.print(",");
      SerialBT.println(pidKd);
      
      Serial.print("LLEGÓ DE APP -> [");
      Serial.print(command);
      Serial.println("]");
    }
    else if (command.startsWith("set_home:")) {
      int commaIndex = command.indexOf(',', 9);
      if (commaIndex != -1) {
        currentLatitude = command.substring(9, commaIndex);
        currentLongitude = command.substring(commaIndex + 1);
      }
      SerialBT.print("home:");
      SerialBT.print(currentLatitude);
      SerialBT.print(",");
      SerialBT.println(currentLongitude);
    }
    else if (command == "get_telemetry") {
      if (!telemetryValid) {
        SerialBT.println("telemetry:sin_datos");
      } else {
        SerialBT.print("telemetry:");
        SerialBT.print(telemetry.batteryVoltage);  SerialBT.print(",");
        SerialBT.print(telemetry.batteryCurrent);  SerialBT.print(",");
        SerialBT.print(telemetry.batteryRemaining); SerialBT.print(",");
        SerialBT.print(telemetry.gpsLat, 7);  SerialBT.print(",");
        SerialBT.print(telemetry.gpsLon, 7);  SerialBT.print(",");
        SerialBT.print(telemetry.gpsSatellites); SerialBT.print(",");
        SerialBT.print(telemetry.linkQuality); SerialBT.print(",");
        SerialBT.print(telemetry.linkRSSI1); SerialBT.print(",");
        SerialBT.println(telemetry.flightMode);
      }
    }
    else if (command == "get_battery") {
      SerialBT.print("battery:");
      SerialBT.print(telemetry.batteryVoltage); SerialBT.print("V,");
      SerialBT.print(telemetry.batteryCurrent); SerialBT.print("A,");
      SerialBT.print(telemetry.batteryRemaining); SerialBT.println("%");
    }
    else if (command == "get_gps") {
      SerialBT.print("gps:");
      SerialBT.print(telemetry.gpsLat, 7); SerialBT.print(",");
      SerialBT.print(telemetry.gpsLon, 7); SerialBT.print(",sats:");
      SerialBT.println(telemetry.gpsSatellites);
    }
    else if (command == "get_link") {
      SerialBT.print("link:LQ=");
      SerialBT.print(telemetry.linkQuality); SerialBT.print("%,RSSI1=");
      SerialBT.print(telemetry.linkRSSI1); SerialBT.print(",RSSI2=");
      SerialBT.print(telemetry.linkRSSI2); SerialBT.print(",SNR=");
      SerialBT.println(telemetry.linkSNR);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(armSwitchPinRight, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  SerialBT.begin("PicoDrone-Control");
  Serial.println("Bluetooth iniciado. Dispositivo visible como 'PicoDrone-Control'");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

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

  Serial.println("ESP-NOW inicializado correctamente");
}

unsigned long lastSendTime = 0;

void loop() {
  handleBluetoothCommands();

  // Si la aplicación cambió los PID, los enviamos inmediatamente por ESP-NOW
  if (pidChanged) {
    pid_packet_t pidPacket;
    pidPacket.type = 0x03;
    pidPacket.kp = pidKp;
    pidPacket.ki = pidKi;
    pidPacket.kd = pidKd;
    
    esp_now_send(broadcastAddress, (uint8_t*)&pidPacket, sizeof(pidPacket));
    pidChanged = false;
    Serial.println("Paquete PID enviado por ESP-NOW al receptor.");
  }

  // Envío periódico de canales RC cada 20 ms
  if (millis() - lastSendTime >= 20) {
    rc_packet_t packet;
    packet.type = 0x01;

    int rawX  = analogRead(joyXPin);
    int rawY  = analogRead(joyYPin);
    int raw2X = analogRead(joy2XPin);
    int raw2Y = analogRead(joy2YPin);

    packet.ch[0] = map_rc_channel(rawX,  CH0_MIN, CH0_MAX);   // Roll
    packet.ch[1] = map_rc_channel(rawY,  CH1_MIN, CH1_MAX);   // Pitch
    packet.ch[2] = map_rc_channel(raw2Y, CH2_MIN, CH2_MAX);   // Throttle
    packet.ch[3] = map_rc_channel(raw2X, CH3_MIN, CH3_MAX);   // Yaw
    packet.ch[4] = read_arm_switch_channel();                 // Arm Switch

    for (int i = 5; i < 16; i++) {
      packet.ch[i] = 992;
    }

    esp_now_send(broadcastAddress, (uint8_t*)&packet, sizeof(packet));
    lastSendTime = millis();
  }
}