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

const int armSwitchPinRight = 26;

// =========================================================================
// CALIBRACIÓN DE EXTREMOS (Tope físico)
// =========================================================================
const int CH0_MIN = 445;   
const int CH0_MAX = 3265;  

const int CH1_MIN = 805;  
const int CH1_MAX = 3750;   

const int CH2_MIN = 3205;  
const int CH2_MAX = 440;   

const int CH3_MIN = 705;   
const int CH3_MAX = 3935;

// Centros dinámicos
int CH0_CENTER = 2048;
int CH1_CENTER = 2048;
int CH3_CENTER = 2048;
// =========================================================================

// Variables para el Filtro Exponencial (EMA)
float filteredCh[4] = {0, 0, 0, 0};
const float alpha = 0.3f; 

String currentLatitude = "-34.603722";
String currentLongitude = "-58.381592";

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

typedef struct __attribute__((packed)) {
  uint8_t type;         // 0x01 = canales RC
  uint16_t ch[16];
} rc_packet_t;

typedef struct __attribute__((packed)) {
  uint8_t type;   // 0x03
  uint8_t seq;
  uint8_t axis;   // 0 roll, 1 pitch, 2 yaw
  float kp, ki, kd;
} pid_packet_t;

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
  uint8_t pidAckSeq;     
  uint8_t pidAckAxis;
  uint8_t pidAckStatus;  
} telemetry_packet_t;

telemetry_packet_t telemetry = {0x02};
volatile bool telemetryValid = false;
unsigned long lastTelemetryRxMillis = 0;

struct PidJob { uint8_t axis; float kp, ki, kd; };
PidJob  pidQueue[4];
uint8_t pidQHead = 0, pidQCount = 0;
uint8_t pidSeq = 0;
bool    pidInFlight = false;
uint8_t pidTries = 0;
uint32_t pidLastTx = 0;
float   pidCache[3][3] = {{2.5f,0.01f,0.002f},{2.5f,0.01f,0.002f},{0.18f,0.005f,0.001f}};

#define PID_MAX_TRIES 10
#define PID_RETRY_MS  100

void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incoming_data, int len) {
  if (len < 1) return;
  if (incoming_data[0] == 0x02 && len == sizeof(telemetry_packet_t)) {
    memcpy(&telemetry, incoming_data, sizeof(telemetry));
    telemetryValid = true;
    lastTelemetryRxMillis = millis();
  }
}

uint16_t map_rc_channel_advanced(int raw, int minRaw, int maxRaw, int centerRaw) {
  const int deadzone = 18; 
  if (abs(raw - centerRaw) <= deadzone) {
    raw = centerRaw;
  }

  int out = 992; 
  if (raw < centerRaw) {
    out = map(raw, minRaw < centerRaw ? minRaw : maxRaw, centerRaw, 172, 992);
  } else if (raw > centerRaw) {
    out = map(raw, centerRaw, maxRaw > centerRaw ? maxRaw : minRaw, 992, 1811);
  }
  return (uint16_t)constrain(out, 172, 1811);
}

uint16_t map_throttle_channel(int raw, int minRaw, int maxRaw) {
  if (minRaw < maxRaw) {
    raw = constrain(raw, minRaw, maxRaw);
  } else {
    raw = constrain(raw, maxRaw, minRaw);
  }
  return (uint16_t)map(raw, minRaw, maxRaw, 172, 1811);
}

uint16_t read_arm_switch_channel() {
  if (digitalRead(armSwitchPinRight) == LOW) {
    return 1811; 
  } else {
    return 992;  
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
    else if (command.startsWith("set_pid:")) {
      int c1 = command.indexOf(',', 8);
      int c2 = (c1 != -1) ? command.indexOf(',', c1 + 1) : -1;
      int c3 = (c2 != -1) ? command.indexOf(',', c2 + 1) : -1;
      if (c3 == -1) {
        SerialBT.println("pid_err:formato");
      } else {
        int axis = command.substring(8, c1).toInt();
        if (axis < 0 || axis > 2) {
          SerialBT.println("pid_err:eje");
        } else if (pidQCount >= 4) {
          SerialBT.println("pid_err:ocupado");
        } else {
          PidJob &j = pidQueue[(pidQHead + pidQCount) % 4];
          j.axis = axis;
          j.kp = command.substring(c1 + 1, c2).toFloat();
          j.ki = command.substring(c2 + 1, c3).toFloat();
          j.kd = command.substring(c3 + 1).toFloat();
          pidQCount++;
          SerialBT.print("pid_queued:"); SerialBT.println(axis);
        }
      }
    }
    else if (command.startsWith("get_pid:")) {
      int a = command.substring(8).toInt();
      if (a >= 0 && a <= 2) {
        SerialBT.print("pid:"); SerialBT.print(a); SerialBT.print(",");
        SerialBT.print(pidCache[a][0], 4); SerialBT.print(",");
        SerialBT.print(pidCache[a][1], 4); SerialBT.print(",");
        SerialBT.println(pidCache[a][2], 4);
      }
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

void servicePid() {
  if (pidQCount == 0) return;
  PidJob &j = pidQueue[pidQHead];

  if (!pidInFlight) {
    uint8_t lastAck = telemetry.pidAckSeq;
    do { pidSeq++; } while (pidSeq == 0 || pidSeq == lastAck);
    pidInFlight = true;
    pidTries = 0;
  }

  if (telemetry.pidAckSeq == pidSeq) {
    uint8_t st = telemetry.pidAckStatus;
    if (st == 0) {
      pidCache[j.axis][0] = j.kp; pidCache[j.axis][1] = j.ki; pidCache[j.axis][2] = j.kd;
      SerialBT.print("pid_ok:"); SerialBT.print(j.axis); SerialBT.print(",");
      SerialBT.print(j.kp, 4); SerialBT.print(","); SerialBT.print(j.ki, 4);
      SerialBT.print(","); SerialBT.println(j.kd, 4);
    } else {
      SerialBT.print("pid_err:"); SerialBT.print(j.axis);
      SerialBT.println(st == 1 ? ",armado" : st == 2 ? ",rango" : ",eje");
    }
    pidQHead = (pidQHead + 1) % 4; pidQCount--; pidInFlight = false;
    return;
  }

  if (pidTries >= PID_MAX_TRIES) {
    SerialBT.print("pid_err:"); SerialBT.print(j.axis); SerialBT.println(",timeout");
    pidQHead = (pidQHead + 1) % 4; pidQCount--; pidInFlight = false;
    return;
  }

  if (pidTries == 0 || millis() - pidLastTx >= PID_RETRY_MS) {
    pid_packet_t p = {0x03, pidSeq, j.axis, j.kp, j.ki, j.kd};
    esp_now_send(broadcastAddress, (uint8_t*)&p, sizeof(p));
    pidTries++;
    pidLastTx = millis();
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(armSwitchPinRight, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  // 1. Inicializar WiFi y ESP-NOW primero
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
  esp_now_add_peer(&peerInfo);

  // 2. Calibración rápida de centros al iniciar
  long sumX = 0, sumY = 0, sum2X = 0;
  int muestras = 30; // Reducido ligeramente para mayor agilidad

  for (int i = 0; i < muestras; i++) {
    sumX  += analogRead(joyXPin);   
    sumY  += analogRead(joyYPin);   
    sum2X += analogRead(joy2XPin);  
    delay(3); 
  }

  CH0_CENTER = sumX / muestras;
  CH1_CENTER = sumY / muestras;
  CH3_CENTER = sum2X / muestras;

  filteredCh[0] = CH0_CENTER;
  filteredCh[1] = CH1_CENTER;
  filteredCh[2] = analogRead(joy2YPin); 
  filteredCh[3] = CH3_CENTER;

  // 3. Inicializar Bluetooth al final del setup para evitar bloqueos de Bluedroid
  SerialBT.begin("PicoDrone-Control");
  Serial.println("Bluetooth iniciado correctamente como 'PicoDrone-Control'");
}

unsigned long lastSendTime = 0;

void loop() {
  handleBluetoothCommands();
  servicePid();

  if (millis() - lastSendTime >= 20) {
    rc_packet_t packet;
    packet.type = 0x01;

    int rawX  = analogRead(joyXPin);
    int rawY  = analogRead(joyYPin);
    int raw2Y = analogRead(joy2YPin); 
    int raw2X = analogRead(joy2XPin); 

    filteredCh[0] = (alpha * rawX)  + ((1.0f - alpha) * filteredCh[0]);
    filteredCh[1] = (alpha * rawY)  + ((1.0f - alpha) * filteredCh[1]);
    filteredCh[2] = (alpha * raw2Y) + ((1.0f - alpha) * filteredCh[2]);
    filteredCh[3] = (alpha * raw2X) + ((1.0f - alpha) * filteredCh[3]);

    packet.ch[0] = map_rc_channel_advanced((int)filteredCh[0], CH0_MIN, CH0_MAX, CH0_CENTER);   
    packet.ch[1] = map_rc_channel_advanced((int)filteredCh[1], CH1_MIN, CH1_MAX, CH1_CENTER);   
    packet.ch[2] = map_throttle_channel((int)filteredCh[2], CH2_MIN, CH2_MAX);                  
    packet.ch[3] = map_rc_channel_advanced((int)filteredCh[3], CH3_MIN, CH3_MAX, CH3_CENTER);   
    packet.ch[4] = read_arm_switch_channel();                                                 

    for (int i = 5; i < 16; i++) {
      packet.ch[i] = 992;
    }

    esp_now_send(broadcastAddress, (uint8_t*)&packet, sizeof(packet));
    lastSendTime = millis();
  }
}