#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

const char* ssid = "Remobility";
const char* password = "12345678";
const char* nodeName = "FOOT";

IPAddress mainESP(192, 168, 4, 1);
const int UDP_PORT = 4210;

WiFiUDP udp;
Adafruit_BNO08x bno08x(-1);
sh2_SensorValue_t sensorValue;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void startSensor() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    Serial.println("Cannot start rotation vector");
  }
}

void setup() {
  Serial.begin(115200);
  delay(700);
  Wire.begin();

  if (!bno08x.begin_I2C()) {
    Serial.println("BNO085 not found");
    while (1) delay(100);
  }

  startSensor();
  connectWiFi();
  Serial.println("FOOT ready");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (bno08x.wasReset()) startSensor();
  if (!bno08x.getSensorEvent(&sensorValue)) return;
  if (sensorValue.sensorId != SH2_GAME_ROTATION_VECTOR) return;

  float x = sensorValue.un.gameRotationVector.i;
  float y = sensorValue.un.gameRotationVector.j;
  float z = sensorValue.un.gameRotationVector.k;
  float w = sensorValue.un.gameRotationVector.real;

  char packet[130];
  snprintf(packet, sizeof(packet), "%s,%lu,%.7f,%.7f,%.7f,%.7f", nodeName, millis(), w, x, y, z);

  udp.beginPacket(mainESP, UDP_PORT);
  udp.write((const uint8_t*)packet, strlen(packet));
  udp.endPacket();

  delay(20);
}