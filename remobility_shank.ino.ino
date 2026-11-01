#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_BNO08x.h>
#include <ArduinoJson.h>
#include <math.h>

#define SDA_PIN 6
#define SCL_PIN 7
#define SAMPLE_US 20000

const char* ssid = "Remobility";
const char* password = "12345678";

IPAddress s3IP(192, 168, 4, 1);
const uint16_t s3Port = 4210;

Adafruit_BNO08x bno(-1);
sh2_SensorValue_t sensor;
WiFiUDP udp;

struct Quaternion {
  float w, x, y, z;
};

uint32_t sequence = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!bno.begin_I2C()) {
    Serial.println("BNO085 initialization failed");
    while (true) delay(1000);
  }

  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, SAMPLE_US)) {
    Serial.println("Failed to enable rotation report");
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to Remobility");

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("Connected! IP: ");
  Serial.println(WiFi.localIP());

  Serial.println("SHANK ready");
}

void loop() {
  if (bno.wasReset()) {
    Serial.println("BNO085 reset detected");
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, SAMPLE_US);
  }

  if (bno.getSensorEvent(&sensor) &&
      sensor.sensorId == SH2_GAME_ROTATION_VECTOR) {

    Quaternion q = {
      sensor.un.gameRotationVector.real,
      sensor.un.gameRotationVector.i,
      sensor.un.gameRotationVector.j,
      sensor.un.gameRotationVector.k
    };

    float n = sqrtf(
      q.w*q.w + q.x*q.x +
      q.y*q.y + q.z*q.z
    );

    if (!isfinite(n) || n < 0.000001f) return;

    JsonDocument doc;

    doc["device"] = "SHANK";
    doc["time"] = millis();
    doc["sequence"] = sequence++;

    JsonArray raw = doc["q_raw"].to<JsonArray>();

    raw.add(q.w / n);
    raw.add(q.x / n);
    raw.add(q.y / n);
    raw.add(q.z / n);

    if (WiFi.status() == WL_CONNECTED) {
      udp.beginPacket(s3IP, s3Port);
      serializeJson(doc, udp);
      udp.endPacket();
    }
  }
}
