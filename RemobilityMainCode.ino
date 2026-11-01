#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

const char* BACKEND_URL = "http://192.168.4.2:8003/api/sensors/data";

const int NUM_MOTORS = 6;
const int NUM_PHASES = 7;
const int TOTAL_CYCLES = 5;

const float SPOOL_CIRCUMFERENCE = 3.14;
const float STEPS_PER_REV = 512.0;
const int STEP_DELAY_US = 2000;

const float EXTRA_PULL_CM = 3.5;
const float EXTRA_PULL_CM_M0 = 0.5;
const float EXTRA_PULL_CM_M1 = 0.5;
const float EXTRA_PULL_CM_M3 = 1.5;

const char* ssid = "Remobility";
const char* password = "12345678";
const int UDP_PORT = 4210;

WebServer server(80);
WiFiUDP udp;

bool running = false;
bool stopRequested = false;
bool returning = false;
bool emergencyStopped = false;

int currentCycle = 0;
int currentPhase = 0;

int motors[NUM_MOTORS][4] = {
  {4, 5, 6, 7},
  {15, 16, 17, 18},
  {8, 9, 10, 11},
  {12, 13, 1, 2},
  {38, 39, 40, 41},
  {42, 47, 48, 21}
};

const int sequence[4][4] = {
  {1, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1}
};

float vectorLength[NUM_PHASES][NUM_MOTORS] = {
  {7.0, 4.5, 6.0, 9.0, 6.0, 6.0},
  {2.0, 7.0, 12.0, 11.0, 12.0, 3.25},
  {7.0, 1.8, 8.0, 8.0, 10.0, 6.0},
  {13.0, 7.5, 2.8, 5.0, 8.0, 9.0},
  {17.0, 11.0, 6.0, 0.2, 5.0, 14.0},
  {11.5, 11.0, 9.0, 8.0, 0.8, 7.5},
  {5.5, 8.5, 11.0, 12.0, 9.0, 0.2}
};

struct MotorInfo {
  float previousVector;
  float targetVector;
  float delta;
  int direction;
  float commandCm;
  long targetSteps;
  long currentSteps;
  float extraPullCm;
};

struct Quaternion {
  float w, x, y, z;
};

MotorInfo motorInfo[NUM_MOTORS];

Quaternion qShank = {1, 0, 0, 0};
Quaternion qFoot = {1, 0, 0, 0};
Quaternion qRelative = {1, 0, 0, 0};
Quaternion qZero = {1, 0, 0, 0};
Quaternion qMotion = {1, 0, 0, 0};

bool shankValid = false;
bool footValid = false;
bool calibrated = false;

unsigned long lastShank = 0;
unsigned long lastFoot = 0;

float angleA = 0;
float angleB = 0;
float angleC = 0;

float minA = 999, maxA = -999;
float minB = 999, maxB = -999;

// ============================================================
// BACKEND QUEUE
// ============================================================

struct SensorPacket {
  char sensor[8];
  float w, x, y, z;
  uint32_t timestamp_ms;
  int cycle;
  int phase;
};

QueueHandle_t sensorQueue = nullptr;

volatile uint32_t queueDrops = 0;
volatile uint32_t httpSuccess = 0;
volatile uint32_t httpFailure = 0;

// ============================================================
// SEND DATA TO FASTAPI
// ============================================================

bool sendToBackend(const SensorPacket &p) {
  HTTPClient http;

  http.begin(BACKEND_URL);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(300);

  JsonDocument doc;

  doc["session_id"] = "remobility_001";
  doc["sensor"] = p.sensor;
  doc["timestamp_ms"] = p.timestamp_ms;

  doc["qw"] = p.w;
  doc["qx"] = p.x;
  doc["qy"] = p.y;
  doc["qz"] = p.z;

  doc["cycle"] = p.cycle;
  doc["phase"] = p.phase;

  String payload;

  serializeJson(doc, payload);

  int response = http.POST(payload);

  http.end();

  return response >= 200 && response < 300;
}

void backendTask(void *parameter) {
  SensorPacket p;

  for (;;) {
    if (xQueueReceive(sensorQueue, &p, portMAX_DELAY) == pdTRUE) {
      bool ok = sendToBackend(p);

      if (ok) httpSuccess++;
      else httpFailure++;

      static uint32_t sent = 0;

      if (++sent % 50 == 0) {
        Serial.printf(
          "[FastAPI] last=%s %s, ok=%lu, fail=%lu, dropped=%lu, queued=%u\n",
          p.sensor,
          ok ? "OK" : "FAILED",
          (unsigned long)httpSuccess,
          (unsigned long)httpFailure,
          (unsigned long)queueDrops,
          (unsigned int)uxQueueMessagesWaiting(sensorQueue)
        );
      }
    }
  }
}

// ============================================================
// QUATERNION
// ============================================================

Quaternion normalizeQ(Quaternion q) {
  float n = sqrt(
    q.w * q.w +
    q.x * q.x +
    q.y * q.y +
    q.z * q.z
  );

  if (n < 0.00001) return {1, 0, 0, 0};

  return {
    q.w / n,
    q.x / n,
    q.y / n,
    q.z / n
  };
}

Quaternion inverseQ(Quaternion q) {
  q = normalizeQ(q);

  return {
    q.w,
    -q.x,
    -q.y,
    -q.z
  };
}

Quaternion multiplyQ(Quaternion a, Quaternion b) {
  Quaternion r;

  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;

  return normalizeQ(r);
}

void quaternionToEuler(
  Quaternion q,
  float &roll,
  float &pitch,
  float &yaw
) {
  q = normalizeQ(q);

  float sinr = 2.0 * (q.w*q.x + q.y*q.z);
  float cosr = 1.0 - 2.0 * (q.x*q.x + q.y*q.y);

  roll = atan2(sinr, cosr);

  float sinp = 2.0 * (q.w*q.y - q.z*q.x);

  if (fabs(sinp) >= 1)
    pitch = copysign(PI / 2, sinp);
  else
    pitch = asin(sinp);

  float siny = 2.0 * (q.w*q.z + q.x*q.y);
  float cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z);

  yaw = atan2(siny, cosy);

  roll *= 180.0 / PI;
  pitch *= 180.0 / PI;
  yaw *= 180.0 / PI;
}

// ============================================================
// SENSOR STATUS
// ============================================================

bool shankOnline() {
  return shankValid && millis() - lastShank < 500;
}

bool footOnline() {
  return footValid && millis() - lastFoot < 500;
}

// ============================================================
// MOTION CALCULATION
// ============================================================

void updateMotion() {
  if (!shankOnline() || !footOnline()) return;

  qRelative = multiplyQ(inverseQ(qShank), qFoot);

  if (!calibrated) return;

  qMotion = multiplyQ(inverseQ(qZero), qRelative);

  quaternionToEuler(
    qMotion,
    angleA,
    angleB,
    angleC
  );

  if (running) {
    if (angleA < minA) minA = angleA;
    if (angleA > maxA) maxA = angleA;

    if (angleB < minB) minB = angleB;
    if (angleB > maxB) maxB = angleB;
  }
}

// ============================================================
// SENSOR CALIBRATION
// ============================================================

void calibrateSensors() {
  if (!shankOnline() || !footOnline()) return;

  qRelative = multiplyQ(inverseQ(qShank), qFoot);

  qZero = qRelative;
  qMotion = {1, 0, 0, 0};

  angleA = 0;
  angleB = 0;
  angleC = 0;

  minA = 999;
  maxA = -999;

  minB = 999;
  maxB = -999;

  calibrated = true;
}

// ============================================================
// RECEIVE IMU
// ============================================================

void receiveIMU() {
  int packetSize = udp.parsePacket();

  while (packetSize > 0) {
    JsonDocument doc;

    if (deserializeJson(doc, udp) == DeserializationError::Ok) {
      const char* device = doc["device"] | "";
      JsonArray q = doc["q_raw"].as<JsonArray>();

      if (!q.isNull() && q.size() == 4) {
        float w = q[0].as<float>();
        float x = q[1].as<float>();
        float y = q[2].as<float>();
        float z = q[3].as<float>();

        if (
          isfinite(w) &&
          isfinite(x) &&
          isfinite(y) &&
          isfinite(z)
        ) {
          Quaternion value = {w, x, y, z};

          if (strcmp(device, "FOOT") == 0) {
            qFoot = value;
            footValid = true;
            lastFoot = millis();
          }
          else if (strcmp(device, "SHANK") == 0) {
            qShank = value;
            shankValid = true;
            lastShank = millis();
          }

          if (
            sensorQueue != nullptr &&
            (
              strcmp(device, "FOOT") == 0 ||
              strcmp(device, "SHANK") == 0
            )
          ) {
            SensorPacket p = {};

            strncpy(
              p.sensor,
              device,
              sizeof(p.sensor) - 1
            );

            p.w = w;
            p.x = x;
            p.y = y;
            p.z = z;

            p.timestamp_ms = millis();
            p.cycle = running ? currentCycle + 1 : 0;
            p.phase = currentPhase;

            if (
              xQueueSend(sensorQueue, &p, 0) != pdTRUE
            ) {
              queueDrops++;
            }
          }
        }
      }
    }

    packetSize = udp.parsePacket();
  }
}

// ============================================================
// MOTOR FUNCTIONS
// ============================================================

long cmToSteps(float cm) {
  return round(
    (cm / SPOOL_CIRCUMFERENCE) * STEPS_PER_REV
  );
}

void stepMotor(int pins[], int stepIndex) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(
      pins[i],
      sequence[stepIndex][i]
    );
  }
}

void stopMotor(int pins[]) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(pins[i], LOW);
  }
}

void stopAllMotors() {
  for (int m = 0; m < NUM_MOTORS; m++) {
    stopMotor(motors[m]);
  }
}

// ============================================================
// PREPARE PHASE
// ============================================================

void preparePhase(int oldPhase, int newPhase) {
  int mainMotor = newPhase - 1;

  for (int m = 0; m < NUM_MOTORS; m++) {
    motorInfo[m].extraPullCm = 0;

    motorInfo[m].previousVector =
      vectorLength[oldPhase][m];

    motorInfo[m].targetVector =
      vectorLength[newPhase][m];

    motorInfo[m].delta =
      motorInfo[m].targetVector -
      motorInfo[m].previousVector;

    if (motorInfo[m].delta > 0) {
      motorInfo[m].direction = 1;
      motorInfo[m].commandCm = motorInfo[m].delta;
    }
    else if (motorInfo[m].delta < 0) {
      motorInfo[m].direction = -1;

      motorInfo[m].commandCm =
        fabsf(motorInfo[m].delta);

      if (m == mainMotor) {
        motorInfo[m].extraPullCm = EXTRA_PULL_CM;

        if (m == 0)
          motorInfo[m].extraPullCm += EXTRA_PULL_CM_M0;
        else if (m == 1)
          motorInfo[m].extraPullCm += EXTRA_PULL_CM_M1;
        else if (m == 3)
          motorInfo[m].extraPullCm += EXTRA_PULL_CM_M3;

        motorInfo[m].commandCm +=
          motorInfo[m].extraPullCm;
      }
    }
    else {
      motorInfo[m].direction = 0;
      motorInfo[m].commandCm = 0;
    }

    motorInfo[m].targetSteps =
      cmToSteps(motorInfo[m].commandCm);

    motorInfo[m].currentSteps = 0;
  }
}

void serviceSystem() {
  server.handleClient();
  receiveIMU();
  updateMotion();
}

// ============================================================
// RUN PHASE
// ============================================================

void runPhase(int mainMotor) {
  while (
    motorInfo[mainMotor].currentSteps <
    motorInfo[mainMotor].targetSteps
  ) {
    serviceSystem();

    if (emergencyStopped) {
      stopAllMotors();
      return;
    }

    bool active[NUM_MOTORS];

    for (int m = 0; m < NUM_MOTORS; m++) {
      active[m] =
        motorInfo[m].currentSteps <
        motorInfo[m].targetSteps;
    }

    for (int state = 0; state < 4; state++) {
      for (int m = 0; m < NUM_MOTORS; m++) {
        if (!active[m]) continue;

        if (motorInfo[m].direction == -1)
          stepMotor(motors[m], state);
        else if (motorInfo[m].direction == 1)
          stepMotor(motors[m], 3 - state);
      }

      delayMicroseconds(STEP_DELAY_US);
    }

    for (int m = 0; m < NUM_MOTORS; m++) {
      if (active[m])
        motorInfo[m].currentSteps++;
    }
  }
}

// ============================================================
// RETURN TO P0
// ============================================================

void prepareReturn(int returnFromPhase) {
  for (int m = 0; m < NUM_MOTORS; m++) {
    motorInfo[m].extraPullCm = 0;

    motorInfo[m].previousVector =
      vectorLength[returnFromPhase][m];

    motorInfo[m].targetVector =
      vectorLength[0][m];

    motorInfo[m].delta =
      motorInfo[m].targetVector -
      motorInfo[m].previousVector;

    if (motorInfo[m].delta > 0) {
      motorInfo[m].direction = 1;
      motorInfo[m].commandCm = motorInfo[m].delta;
    }
    else if (motorInfo[m].delta < 0) {
      motorInfo[m].direction = -1;

      motorInfo[m].commandCm =
        fabsf(motorInfo[m].delta);
    }
    else {
      motorInfo[m].direction = 0;
      motorInfo[m].commandCm = 0;
    }

    motorInfo[m].targetSteps =
      cmToSteps(motorInfo[m].commandCm);

    motorInfo[m].currentSteps = 0;
  }
}

void runReturn() {
  while (true) {
    serviceSystem();

    if (emergencyStopped) {
      stopAllMotors();
      return;
    }

    bool active[NUM_MOTORS];
    bool anyActive = false;

    for (int m = 0; m < NUM_MOTORS; m++) {
      active[m] =
        motorInfo[m].currentSteps <
        motorInfo[m].targetSteps;

      if (active[m])
        anyActive = true;
    }

    if (!anyActive) return;

    for (int state = 0; state < 4; state++) {
      for (int m = 0; m < NUM_MOTORS; m++) {
        if (!active[m]) continue;

        if (motorInfo[m].direction == -1)
          stepMotor(motors[m], state);
        else if (motorInfo[m].direction == 1)
          stepMotor(motors[m], 3 - state);
      }

      delayMicroseconds(STEP_DELAY_US);
    }

    for (int m = 0; m < NUM_MOTORS; m++) {
      if (active[m])
        motorInfo[m].currentSteps++;
    }
  }
}

// ============================================================
// SYSTEM STATUS
// ============================================================

String getStatus() {
  String status = "SẴN SÀNG";
  String phase = "P0";

  if (running && returning) {
    status = "ĐANG TRỞ VỀ";
    phase = "P" + String(currentPhase) + " → P0";
  }
  else if (running) {
    status = stopRequested
      ? "ĐANG DỪNG"
      : "ĐANG HOẠT ĐỘNG";

    phase = "P" + String(currentPhase);
  }

  if (emergencyStopped) {
    status = "ĐÃ DỪNG KHẨN CẤP";
    phase = "KHÔNG XÁC ĐỊNH P0";
  }

  // VẪN GIỮ NGUYÊN PHÉP TÍNH ROM
  // CHỈ KHÔNG HIỂN THỊ TRÊN DASHBOARD
  float romA = maxA > minA ? maxA - minA : 0;
  float romB = maxB > minB ? maxB - minB : 0;

  String output = "{";

  output += "\"status\":\"" + status + "\",";
  output += "\"cycle\":" +
    String(running ? currentCycle + 1 : 0) + ",";

  output += "\"phase\":\"" + phase + "\",";

  output += "\"shank\":" +
    String(shankOnline() ? "true" : "false") + ",";

  output += "\"foot\":" +
    String(footOnline() ? "true" : "false") + ",";

  output += "\"calibrated\":" +
    String(calibrated ? "true" : "false") + ",";

  output += "\"a\":" + String(angleA, 2) + ",";
  output += "\"b\":" + String(angleB, 2) + ",";
  output += "\"c\":" + String(angleC, 2) + ",";

  output += "\"romA\":" + String(romA, 2) + ",";
  output += "\"romB\":" + String(romB, 2) + ",";

  output += "\"qw\":" + String(qMotion.w, 6) + ",";
  output += "\"qx\":" + String(qMotion.x, 6) + ",";
  output += "\"qy\":" + String(qMotion.y, 6) + ",";
  output += "\"qz\":" + String(qMotion.z, 6) + ",";

  output += "\"emergencyStopped\":" +
    String(emergencyStopped ? "true" : "false") + ",";

  output += "\"httpOk\":" +
    String((unsigned long)httpSuccess) + ",";

  output += "\"httpFail\":" +
    String((unsigned long)httpFailure) + ",";

  output += "\"queueDrops\":" +
    String((unsigned long)queueDrops);

  output += "}";

  return output;
}

// ============================================================
// HTML DASHBOARD
// CHỈ CHỈNH GIAO DIỆN:
// - ẨN GÓC B
// - XÓA THẺ ROM A/B
// - GIỮ NGUYÊN MÔ HÌNH 3D VÀ QUỸ ĐẠO
// ============================================================

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Remobility</title>

<style>
*{box-sizing:border-box}

:root{
  --navy:#062c40;
  --blue:#087ba9;
  --cyan:#1bb5d8;
  --black:#080d11;
}

body{
  margin:0;
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Arial,sans-serif;
  background:#e8ebed;
  color:#17242b;
}

header{
  height:96px;
  padding:0 5%;
  display:flex;
  align-items:center;
  justify-content:space-between;
  background:#f7f8f8;
  border-bottom:1px solid #ccd3d7;
}

.brand h1{
  margin:0;
  color:var(--navy);
  font-size:30px;
}

.brand p{
  margin:4px 0 0;
  color:#74818a;
  font-size:13px;
}

.online{
  font-size:12px;
  font-weight:700;
  letter-spacing:.8px;
}

.dot{
  display:inline-block;
  width:9px;
  height:9px;
  margin-right:8px;
  border-radius:50%;
  background:var(--cyan);
  box-shadow:0 0 10px var(--cyan);
}

main{
  width:min(1280px,94%);
  margin:28px auto;
}

.motion{
  display:grid;
  grid-template-columns:2fr 1fr;
  overflow:hidden;
  border-radius:18px;
  background:var(--black);
  box-shadow:0 14px 35px rgba(0,0,0,.14);
}

.viewport{
  position:relative;
  height:500px;
  overflow:hidden;
  background:radial-gradient(
    circle at 50% 42%,
    #172a34 0,
    #080d11 62%
  );
}

.viewport-title{
  position:absolute;
  z-index:2;
  left:24px;
  top:22px;
  color:#9dafb9;
  font-size:11px;
  letter-spacing:1.7px;
  font-weight:700;
}

.viewport-info{
  position:absolute;
  z-index:2;
  left:24px;
  bottom:20px;
  color:#667d88;
  font-size:10px;
  line-height:1.7;
}

#footCanvas{
  position:absolute;
  inset:0;
  width:100%;
  height:100%;
}

.telemetry{
  padding:30px;
  color:white;
  background:linear-gradient(145deg,#101820,#081015);
}

.telemetry h2{
  margin:0 0 24px;
  color:#80939e;
  font-size:12px;
  letter-spacing:2px;
}

.metric{
  padding:16px 0;
  border-bottom:1px solid #22313a;
}

.metric-name{
  color:#7f929d;
  font-size:11px;
  letter-spacing:1px;
}

.metric-value{
  margin-top:5px;
  font-size:27px;
  font-weight:650;
}

.sensor{
  display:flex;
  justify-content:space-between;
  margin-top:18px;
  color:#9aabb4;
  font-size:11px;
}

.good{color:#39d6a3}
.bad{color:#ff7777}

/* CHỈ CÒN MỘT THẺ QUỸ ĐẠO */
.cards{
  display:grid;
  grid-template-columns:1fr;
  gap:18px;
  margin-top:18px;
}

.card{
  min-height:165px;
  padding:22px;
  border:1px solid #ccd3d7;
  border-radius:14px;
  background:#f8f9f9;
}

.card-title{
  color:#77868e;
  font-size:11px;
  font-weight:700;
  letter-spacing:1.5px;
}

.trajectory{
  height:115px;
  margin-top:10px;
  border-left:1px solid #ccd5da;
  border-bottom:1px solid #ccd5da;
}

#trajectory{
  width:100%;
  height:100%;
}

.session{
  display:flex;
  align-items:center;
  justify-content:space-between;
  gap:20px;
  margin-top:18px;
  padding:20px 24px;
  border:1px solid #ccd3d7;
  border-radius:14px;
  background:#f8f9f9;
}

.state{
  color:var(--navy);
  font-weight:700;
}

.phase{
  color:var(--blue);
  font-weight:700;
}

button{
  height:46px;
  padding:0 24px;
  border-radius:8px;
  font-weight:700;
  cursor:pointer;
}

.calibrate{
  border:1px solid #aeb9bf;
  background:white;
  color:#3b4b54;
}

.start{
  border:0;
  background:linear-gradient(100deg,var(--navy),var(--blue));
  color:white;
}

.stop{
  border:1px solid #b9c1c6;
  background:#e4e8ea;
  color:#38464e;
}

.emergency{
  border:0;
  background:#b71c1c;
  color:white;
}

.reset{
  border:1px solid #b71c1c;
  background:white;
  color:#b71c1c;
}

button:disabled{
  opacity:.5;
  cursor:not-allowed;
}

@media(max-width:800px){
  .motion{grid-template-columns:1fr}
  .viewport{height:390px}
  .cards{grid-template-columns:1fr}
  .session{
    flex-direction:column;
    align-items:stretch;
  }
}
</style>
</head>

<body>

<header>
  <div class="brand">
    <h1>Remobility</h1>
    <p>Hệ thống phân tích chuyển động và hỗ trợ phục hồi cổ chân</p>
  </div>

  <div class="online">
    <span class="dot"></span>HỆ THỐNG CỤC BỘ
  </div>
</header>

<main>

<section class="motion">

  <div class="viewport">
    <div class="viewport-title">
      CHUYỂN ĐỘNG CỔ CHÂN 3D THEO THỜI GIAN THỰC
    </div>

    <canvas id="footCanvas"></canvas>

    <div class="viewport-info">
      CẲNG CHÂN → BÀN CHÂN<br>
      ĐỊNH HƯỚNG TƯƠNG ĐỐI
    </div>
  </div>

  <div class="telemetry">
    <h2>DỮ LIỆU CHUYỂN ĐỘNG</h2>

    <!-- GIỮ GÓC A -->
    <div class="metric">
      <div class="metric-name">GÓC A</div>
      <div class="metric-value">
        <span id="angleA">0.0</span>°
      </div>
    </div>

    <!-- GIỮ GÓC XOAY C -->
    <div class="metric">
      <div class="metric-name">GÓC XOAY</div>
      <div class="metric-value">
        <span id="angleC">0.0</span>°
      </div>
    </div>

    <!-- GÓC B ĐÃ ĐƯỢC ẨN KHỎI GIAO DIỆN -->

    <div class="sensor">
      <span>CẢM BIẾN CẲNG CHÂN</span>
      <span id="shankStatus">ĐANG CHỜ</span>
    </div>

    <div class="sensor">
      <span>CẢM BIẾN BÀN CHÂN</span>
      <span id="footStatus">ĐANG CHỜ</span>
    </div>
  </div>

</section>

<!-- CHỈ GIỮ THẺ QUỸ ĐẠO -->
<section class="cards">
  <div class="card">
    <div class="card-title">QUỸ ĐẠO CHUYỂN ĐỘNG</div>

    <div class="trajectory">
      <canvas id="trajectory"></canvas>
    </div>
  </div>
</section>

<section class="session">

  <div>
    <div class="state" id="systemStatus">
      SẴN SÀNG
    </div>

    <div style="margin-top:5px;font-size:13px;color:#7a878e">
      <span class="phase" id="phase">P0</span>
      &nbsp;&nbsp;
      CHU KỲ <span id="cycle">0</span>/5
    </div>
  </div>

  <div>
    <button class="calibrate" onclick="calibrate()">
      HIỆU CHUẨN
    </button>

    <button class="start" id="startButton" onclick="startSystem()">
      BẮT ĐẦU
    </button>

    <button class="stop" onclick="stopSystem()">
      DỪNG PHA
    </button>

    <button class="emergency" onclick="emergencyStop()">
      DỪNG KHẨN CẤP
    </button>

    <button
      class="reset"
      id="resetButton"
      onclick="resetEmergency()"
      style="display:none"
    >
      ĐẶT LẠI DỪNG KHẨN CẤP
    </button>
  </div>

</section>

</main>

<script>

// ============================================================
// 3D MODEL — GIỮ NGUYÊN
// ============================================================

let trajectory = [];
let motionA = 0, motionB = 0, motionC = 0;

const footCanvas = document.getElementById("footCanvas");
const footCtx = footCanvas.getContext("2d");

const footVertices = [
  [-0.50,-0.55,-0.36],
  [0.50,-0.55,-0.36],
  [0.55,-0.55,0.32],
  [-0.55,-0.55,0.32],

  [-0.72,0,-0.44],
  [0.72,0,-0.44],
  [0.78,0,0.38],
  [-0.78,0,0.38],

  [-0.88,0.85,-0.43],
  [0.88,0.85,-0.43],
  [0.96,0.85,0.36],
  [-0.96,0.85,0.36],

  [-0.94,1.75,-0.37],
  [0.94,1.75,-0.37],
  [0.98,1.75,0.30],
  [-0.98,1.75,0.30],

  [-0.90,2.55,-0.31],
  [0.90,2.55,-0.31],
  [0.92,2.55,0.25],
  [-0.92,2.55,0.25],

  [-0.78,3.20,-0.24],
  [0.78,3.20,-0.24],
  [0.80,3.20,0.18],
  [-0.80,3.20,0.18],

  [-0.55,3.68,-0.17],
  [0.55,3.68,-0.17],
  [0.57,3.68,0.12],
  [-0.57,3.68,0.12]
];

const footFaces = [
  [0,1,5,4],
  [1,2,6,5],
  [2,3,7,6],
  [3,0,4,7],

  [4,5,9,8],
  [5,6,10,9],
  [6,7,11,10],
  [7,4,8,11],

  [8,9,13,12],
  [9,10,14,13],
  [10,11,15,14],
  [11,8,12,15],

  [12,13,17,16],
  [13,14,18,17],
  [14,15,19,18],
  [15,12,16,19],

  [16,17,21,20],
  [17,18,22,21],
  [18,19,23,22],
  [19,16,20,23],

  [20,21,25,24],
  [21,22,26,25],
  [22,23,27,26],
  [23,20,24,27],
  [24,25,26,27]
];

function resizeFootCanvas() {
  const r = footCanvas.getBoundingClientRect();
  const d = window.devicePixelRatio || 1;

  footCanvas.width = r.width * d;
  footCanvas.height = r.height * d;

  footCtx.setTransform(d, 0, 0, d, 0, 0);
}

function rotatePoint(p, ax, ay, az) {
  let x = p[0], y = p[1], z = p[2];

  let c = Math.cos(ax), s = Math.sin(ax);

  let ny = y*c - z*s;
  let nz = y*s + z*c;

  y = ny;
  z = nz;

  c = Math.cos(ay);
  s = Math.sin(ay);

  let nx = x*c + z*s;

  nz = -x*s + z*c;

  x = nx;
  z = nz;

  c = Math.cos(az);
  s = Math.sin(az);

  nx = x*c - y*s;
  ny = x*s + y*c;

  return [nx, ny, z];
}

function projectPoint(p, cx, cy, scale) {
  const perspective = 6 / (6 + p[2]);

  return [
    cx + p[0] * scale * perspective,
    cy - p[1] * scale * perspective
  ];
}

function drawFoot() {
  resizeFootCanvas();

  const r = footCanvas.getBoundingClientRect();
  const w = r.width;
  const h = r.height;

  footCtx.clearRect(0, 0, w, h);

  // GIỮ NGUYÊN CẢ 3 GÓC CHO MÔ HÌNH 3D
  const ax = -motionB * Math.PI / 180;
  const ay = motionC * Math.PI / 180;
  const az = motionA * Math.PI / 180;

  const neutralX = -20 * Math.PI / 180;
  const neutralZ = -8 * Math.PI / 180;

  let transformed = [];

  for (let i = 0; i < footVertices.length; i++) {
    let p = rotatePoint(
      footVertices[i],
      neutralX,
      0,
      neutralZ
    );

    p = rotatePoint(p, ax, ay, az);

    transformed.push(p);
  }

  const cx = w * 0.50;
  const cy = h * 0.73;

  const scale = Math.min(w, h) * 0.105;

  const projected = transformed.map(
    p => projectPoint(p, cx, cy, scale)
  );

  let sorted = footFaces.map(face => {
    let depth = 0;

    for (let i = 0; i < face.length; i++) {
      depth += transformed[face[i]][2];
    }

    return {
      face: face,
      depth: depth / face.length
    };
  });

  sorted.sort((a, b) => b.depth - a.depth);

  for (let i = 0; i < sorted.length; i++) {
    const face = sorted[i].face;

    footCtx.beginPath();

    for (let j = 0; j < face.length; j++) {
      const p = projected[face[j]];

      if (j === 0)
        footCtx.moveTo(p[0], p[1]);
      else
        footCtx.lineTo(p[0], p[1]);
    }

    footCtx.closePath();

    const light = Math.max(
      24,
      Math.min(46, 34 - sorted[i].depth * 5)
    );

    footCtx.fillStyle =
      "hsl(192,68%," + light + "%)";

    footCtx.fill();

    footCtx.strokeStyle = "rgba(88,218,243,.75)";
    footCtx.lineWidth = 1;
    footCtx.stroke();
  }

  footCtx.beginPath();
  footCtx.arc(cx, cy, 5, 0, Math.PI * 2);

  footCtx.fillStyle = "#5ad9f3";
  footCtx.fill();

  footCtx.font = "10px Arial";
  footCtx.fillStyle = "#718b96";
  footCtx.fillText("CỔ CHÂN", cx + 11, cy + 4);
}

// ============================================================
// TRAJECTORY — GIỮ NGUYÊN
// ============================================================

function drawTrajectory() {
  const canvas = document.getElementById("trajectory");
  const r = canvas.getBoundingClientRect();
  const d = window.devicePixelRatio || 1;

  canvas.width = r.width * d;
  canvas.height = r.height * d;

  const ctx = canvas.getContext("2d");

  ctx.setTransform(d, 0, 0, d, 0, 0);

  const w = r.width;
  const h = r.height;

  ctx.clearRect(0, 0, w, h);

  ctx.strokeStyle = "#dce3e6";
  ctx.lineWidth = 1;

  ctx.beginPath();

  ctx.moveTo(w/2, 0);
  ctx.lineTo(w/2, h);

  ctx.moveTo(0, h/2);
  ctx.lineTo(w, h/2);

  ctx.stroke();

  if (trajectory.length < 2) return;

  ctx.beginPath();

  for (let i = 0; i < trajectory.length; i++) {
    const x = w/2 + trajectory[i][0] * 3;
    const y = h/2 - trajectory[i][1] * 3;

    if (i === 0)
      ctx.moveTo(x, y);
    else
      ctx.lineTo(x, y);
  }

  ctx.strokeStyle = "#087ba9";
  ctx.lineWidth = 2;
  ctx.stroke();
}

// ============================================================
// BUTTON CONTROLS — GIỮ NGUYÊN
// ============================================================

function startSystem() {
  trajectory = [];
  fetch("/start");
}

function stopSystem() {
  fetch("/stop");
}

function calibrate() {
  trajectory = [];
  fetch("/calibrate");
}

function emergencyStop() {
  fetch("/emergency-stop", {
    method: "POST"
  }).catch(() => alert(
    "Không gửi được lệnh. Hãy ngắt nguồn động cơ bằng công tắc vật lý!"
  ));
}

function resetEmergency() {
  if (!confirm(
    "Chỉ đặt lại sau khi đã kiểm tra cơ cấu và xác nhận an toàn. Tiếp tục?"
  )) return;

  fetch("/reset-emergency", {
    method: "POST"
  })
  .then(r => {
    if (!r.ok)
      alert("Chưa thể đặt lại.");
  })
  .catch(() => alert("Mất kết nối."));
}

function sensorState(id, online) {
  const el = document.getElementById(id);

  el.textContent = online
    ? "KẾT NỐI"
    : "MẤT KẾT NỐI";

  el.className = online
    ? "good"
    : "bad";
}

// ============================================================
// DASHBOARD UPDATE
// CHỈ BỎ CẬP NHẬT PHẦN TỬ HTML CỦA GÓC B VÀ ROM
// ============================================================

async function update() {
  try {
    const response = await fetch("/status", {
      cache: "no-store"
    });

    const data = await response.json();

    document.getElementById("systemStatus").textContent =
      data.status;

    document.getElementById("phase").textContent =
      data.phase;

    document.getElementById("cycle").textContent =
      data.cycle;

    sensorState("shankStatus", data.shank);
    sensorState("footStatus", data.foot);

    // CHỈ HIỂN THỊ GÓC A VÀ GÓC XOAY C
    document.getElementById("angleA").textContent =
      Number(data.a).toFixed(1);

    document.getElementById("angleC").textContent =
      Number(data.c).toFixed(1);

    // KHÔNG CẬP NHẬT angleB, romA, romB TRÊN HTML

    document.getElementById("startButton").disabled =
      Boolean(data.emergencyStopped);

    document.getElementById("resetButton").style.display =
      data.emergencyStopped ? "inline-block" : "none";

    // GIỮ NGUYÊN 3 GÓC CHO THUẬT TOÁN HIỂN THỊ 3D
    motionA = Number(data.a);
    motionB = Number(data.b);
    motionC = Number(data.c);

    drawFoot();

    // GIỮ NGUYÊN BIỂU ĐỒ QUỸ ĐẠO A/B
    if (
      data.calibrated &&
      data.shank &&
      data.foot
    ) {
      trajectory.push([data.a, data.b]);

      if (trajectory.length > 600)
        trajectory.shift();

      drawTrajectory();
    }

  } catch (error) {
    document.getElementById("systemStatus").textContent =
      "MẤT KẾT NỐI";
  }
}

window.addEventListener("resize", () => {
  drawFoot();
  drawTrajectory();
});

drawFoot();
drawTrajectory();
update();

setInterval(update, 100);

</script>
</body>
</html>
)rawliteral";

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  for (int m = 0; m < NUM_MOTORS; m++) {
    for (int i = 0; i < 4; i++) {
      pinMode(motors[m][i], OUTPUT);
      digitalWrite(motors[m][i], LOW);
    }
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  udp.begin(UDP_PORT);

  sensorQueue = xQueueCreate(
    100,
    sizeof(SensorPacket)
  );

  if (sensorQueue == nullptr) {
    Serial.println(
      "[FastAPI] ERROR: cannot allocate queue"
    );
  }
  else {
    BaseType_t taskCreated =
      xTaskCreatePinnedToCore(
        backendTask,
        "FastAPI",
        8192,
        nullptr,
        1,
        nullptr,
        0
      );

    if (taskCreated != pdPASS) {
      Serial.println(
        "[FastAPI] ERROR: cannot create task"
      );

      vQueueDelete(sensorQueue);
      sensorQueue = nullptr;
    }
  }

  Serial.println("Remobility started");
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    server.send(
      200,
      "text/html; charset=utf-8",
      index_html
    );
  });

  server.on("/start", []() {
    if (emergencyStopped) {
      server.send(
        409,
        "text/plain",
        "DUNG KHAN CAP - CAN DAT LAI"
      );
      return;
    }

    if (!running) {
      running = true;
      stopRequested = false;
      returning = false;

      currentCycle = 0;
      currentPhase = 0;

      minA = 999;
      maxA = -999;
      minB = 999;
      maxB = -999;
    }

    server.send(200, "text/plain", "OK");
  });

  server.on("/stop", []() {
    if (running)
      stopRequested = true;

    server.send(200, "text/plain", "OK");
  });

  server.on("/emergency-stop", HTTP_POST, []() {
    emergencyStopped = true;

    stopAllMotors();

    server.send(
      200,
      "text/plain",
      "DA DUNG KHAN CAP"
    );
  });

  server.on("/reset-emergency", HTTP_POST, []() {
    if (running) {
      server.send(
        409,
        "text/plain",
        "HE THONG CHUA DUNG"
      );
      return;
    }

    emergencyStopped = false;

    server.send(
      200,
      "text/plain",
      "DA DAT LAI - KIEM TRA VI TRI TRUOC KHI BAT DAU"
    );
  });

  server.on("/calibrate", []() {
    calibrateSensors();

    if (calibrated)
      server.send(
        200,
        "text/plain",
        "CALIBRATED"
      );
    else
      server.send(
        409,
        "text/plain",
        "IMU NOT READY"
      );
  });

  server.on("/status", []() {
    server.sendHeader(
      "Cache-Control",
      "no-store"
    );

    server.send(
      200,
      "application/json; charset=utf-8",
      getStatus()
    );
  });

  server.begin();
}

// ============================================================
// MAIN LOOP — THUẬT TOÁN
// ============================================================

void loop() {
  serviceSystem();

  if (!running) return;

  if (emergencyStopped) {
    stopAllMotors();
    running = false;
    return;
  }

  for (
    currentCycle = 0;
    currentCycle < TOTAL_CYCLES && !stopRequested;
    currentCycle++
  ) {
    currentPhase = 0;

    for (
      int newPhase = 1;
      newPhase < NUM_PHASES;
      newPhase++
    ) {
      if (emergencyStopped) break;

      int oldPhase = newPhase - 1;
      int mainMotor = newPhase - 1;

      preparePhase(oldPhase, newPhase);
      runPhase(mainMotor);

      if (emergencyStopped) break;

      currentPhase = newPhase;

      if (stopRequested) break;
    }

    if (emergencyStopped) break;

    returning = true;

    prepareReturn(currentPhase);
    runReturn();

    if (emergencyStopped) break;

    returning = false;
    currentPhase = 0;

    if (stopRequested) break;
  }

  stopAllMotors();

  running = false;
  stopRequested = false;
  returning = false;

  if (!emergencyStopped)
    currentPhase = 0;
}
