#include <Arduino.h>
#include <Wire.h>
#include <SCServo.h>
#include "MPU6050_6Axis_MotionApps20.h"

// Pins: Seeed XIAO ESP32-S3
#define LIDAR_RXD 3
#define LIDAR_TXD 4
#define IMU_SDA 5
#define IMU_SCL 6
#define SERVO_RXD 44
#define SERVO_TXD 43

#define MPU6050_ADDR 0x68
#define SERVO_ID_TILT 1
#define SERVO_CENTER 2048
// +1: normal direction, -1: inverted direction
#define SERVO_DIRECTION -1

#define LIDAR_BAUDRATE 230400
#define SERVO_BAUDRATE 1000000
#define USB_BAUDRATE 921600

#define PROTO_SYNC1 0xFE
#define PROTO_SYNC2 0xEF

#define MSG_IMU_DATA 0x02
#define MSG_SCAN_SLICE 0x04
#define MSG_SCAN_STATUS 0x05
#define CMD_SERVO_ANGLE 0x10
#define CMD_SCAN_START 0x12
#define CMD_SCAN_STOP 0x13

#define TILT_MIN_DEFAULT -45.0f
#define TILT_MAX_DEFAULT 45.0f
#define TILT_STEP_DEFAULT 2.0f
#define TILT_SAFE_LIMIT 50.0f

#define SETTLE_MS_MIN 150
#define SETTLE_MS_MAX 1300
// Production fast-detection profile: capture immediately after the tilt command.
// This intentionally includes points measured while the servo is moving.
#define SCAN_SETTLE_DISABLED 1
#define IMU_READ_MS 10
#define IMU_SEND_MS 50
#define MAX_SCAN_POINTS 720
#define MAX_PROTO_PAYLOAD 4096
#define CAPTURE_TIMEOUT_MS 110
#define CAPTURE_MIN_POINTS 25
#define SWEEP_PREPARE_MS 120
#define SWEEP_FINALIZE_MS 500
#define SWEEP_LIDAR_REV_HZ 6.0f
#define SWEEP_SPEED_MIN_DPS 4.0f
#define SWEEP_SPEED_MAX_DPS 80.0f
// Human-readable USB debug mode.
// 1: Print IMU + servo ReadPos as text and suppress binary TX frames.
// 0: Normal binary protocol behavior.
#define USB_TEXT_DEBUG_MODE 0

HardwareSerial LidarSerial(1);
SMS_STS servo;
MPU6050 mpu(MPU6050_ADDR, &Wire);

struct LidarPoint {
  uint16_t angle_x100;
  uint16_t dist_mm;
  uint8_t quality;
};

static LidarPoint scanPoints[MAX_SCAN_POINTS];
static uint16_t scanCount = 0;
static volatile bool scanReady = false;

static float imuPitch = 0.0f;
static float imuRoll = 0.0f;
static float imuYaw = 0.0f;
static uint32_t lastImuReadMs = 0;
static uint32_t lastImuSendMs = 0;
static uint8_t servoTxId = SERVO_ID_TILT;
static int8_t servoReadId = -1;
static uint32_t lastServoDebugMs = 0;
static uint32_t lastServoProbeMs = 0;
static bool dmpReady = false;
static uint16_t dmpPacketSize = 0;
static uint8_t dmpFifoBuffer[64];
static Quaternion dmpQ;
static VectorFloat dmpGravity;
static float dmpYpr[3];

enum Scan3DState {
  S3D_IDLE,
  S3D_PREPARE,
  S3D_SWEEPING,
  S3D_FINALIZING,
  S3D_COMPLETE
};

static Scan3DState scanState = S3D_IDLE;
static float tiltMin = TILT_MIN_DEFAULT;
static float tiltMax = TILT_MAX_DEFAULT;
static float tiltStep = TILT_STEP_DEFAULT;
static float currentTilt = 0.0f;
static uint16_t stepIndex = 0;
static uint16_t totalSteps = 0;
static uint32_t stateStartedMs = 0;
static uint32_t settleMsCurrent = SETTLE_MS_MIN;
static float captureTiltStart = 0.0f;
static float captureTiltEnd = 0.0f;
static float sweepSpeedDps = 0.0f;
static int8_t sweepDirection = 1;
static uint32_t lastSweepUpdateMs = 0;
static uint16_t slicesSent = 0;

static uint8_t rxFrame[128];
static uint16_t rxIndex = 0;
static uint16_t rxLen = 0;
static uint8_t rxType = 0;
static uint8_t rxState = 0;

static uint8_t lidarBuf[2300];
static uint16_t lidarIndex = 0;
static uint16_t lidarExpected = 0;
static uint8_t lidarState = 0;
static bool lidarMotorOn = false;
static int8_t probeServoId();

static uint8_t checksum(uint8_t type, uint16_t len, const uint8_t *payload) {
  uint8_t cs = type ^ (uint8_t)(len & 0xFF) ^ (uint8_t)(len >> 8);
  for (uint16_t i = 0; i < len; i++) {
    cs ^= payload[i];
  }
  return cs;
}

static void writeU16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static void writeF32(uint8_t *p, float v) {
  memcpy(p, &v, sizeof(float));
}

static float readF32(const uint8_t *p) {
  float v;
  memcpy(&v, p, sizeof(float));
  return v;
}

static void sendFrame(uint8_t type, const uint8_t *payload, uint16_t len) {
#if USB_TEXT_DEBUG_MODE
  (void)type;
  (void)payload;
  (void)len;
  return;
#else
  Serial.write(PROTO_SYNC1);
  Serial.write(PROTO_SYNC2);
  Serial.write(type);
  Serial.write((uint8_t)(len & 0xFF));
  Serial.write((uint8_t)(len >> 8));
  if (len > 0 && payload != nullptr) {
    Serial.write(payload, len);
  }
  Serial.write(checksum(type, len, payload));
#endif
}

static void sendStatus(uint8_t state, uint16_t step, uint16_t total) {
  uint8_t payload[5];
  payload[0] = state;
  writeU16(payload + 1, step);
  writeU16(payload + 3, total);
  sendFrame(MSG_SCAN_STATUS, payload, sizeof(payload));
}

static int angleToServoPos(float angleDeg) {
  float limited = constrain(angleDeg, -TILT_SAFE_LIMIT, TILT_SAFE_LIMIT);
  int pos = SERVO_CENTER + (int)(SERVO_DIRECTION * limited * 4096.0f / 360.0f);
  return constrain(pos, 0, 4095);
}

static void setTiltAngle(float angleDeg) {
  currentTilt = constrain(angleDeg, -TILT_SAFE_LIMIT, TILT_SAFE_LIMIT);
  int pos = angleToServoPos(currentTilt);
  servo.WritePosEx(servoTxId, pos, 1800, 50);
}

static float servoPosToAngle(int pos) {
  return ((float)(pos - SERVO_CENTER)) * 360.0f / (4096.0f * (float)SERVO_DIRECTION);
}

static bool readServoTilt(float *angleDegOut) {
  uint32_t nowMs = millis();
  if (servoReadId < 0 && (nowMs - lastServoProbeMs >= 2000)) {
    lastServoProbeMs = nowMs;
    servoReadId = probeServoId();
  }

  if (servoReadId < 0) {
    return false;
  }

  int pos = servo.ReadPos(servoReadId);
  if (pos < 0) {
    servoReadId = -1;
    return false;
  }

  *angleDegOut = constrain(servoPosToAngle(pos), -TILT_SAFE_LIMIT, TILT_SAFE_LIMIT);
  return true;
}

static void initServo() {
  delay(100);
  // Prefer the expected ID first.
  if (servo.Ping(SERVO_ID_TILT) != -1) {
    servoTxId = SERVO_ID_TILT;
    servoReadId = SERVO_ID_TILT;
    servo.EnableTorque(servoTxId, 1);
    return;
  }

  // Try to detect a servo ID on a typical small range.
  for (uint8_t id = 0; id <= 20; id++) {
    if (servo.Ping(id) != -1) {
      servoTxId = id;
      servoReadId = id;
      servo.EnableTorque(servoTxId, 1);
      return;
    }
    delay(10);
  }

  // Fallback: broadcast command works even when ID is unknown.
  servoTxId = 0xFE;
  servoReadId = -1;
}

static int8_t probeServoId() {
  if (servoTxId != 0xFE && servo.Ping(servoTxId) != -1) {
    return (int8_t)servoTxId;
  }
  for (uint8_t id = 0; id <= 20; id++) {
    if (servo.Ping(id) != -1) {
      return (int8_t)id;
    }
    delay(2);
  }
  return (int8_t)-1;
}

static void debugServoImuStatus(uint32_t nowMs) {
#if USB_TEXT_DEBUG_MODE
  if (nowMs - lastServoDebugMs < 250) {
    return;
  }
  lastServoDebugMs = nowMs;

  if (servoReadId < 0 && (nowMs - lastServoProbeMs >= 2000)) {
    lastServoProbeMs = nowMs;
    servoReadId = probeServoId();
  }

  int pos = -1;
  if (servoReadId >= 0) {
    pos = servo.ReadPos(servoReadId);
    if (pos < 0) {
      servoReadId = -1;
    }
  }

  Serial.print("[dbg] tx_id=");
  Serial.print((int)servoTxId);
  Serial.print(" read_id=");
  Serial.print((int)servoReadId);
  Serial.print(" read_pos=");
  Serial.print(pos);
  Serial.print(" tilt=");
  Serial.print(currentTilt, 2);
  Serial.print(" imu_pitch=");
  Serial.print(imuPitch, 2);
  Serial.print(" imu_roll=");
  Serial.print(imuRoll, 2);
  Serial.print(" imu_yaw=");
  Serial.println(imuYaw, 2);
#else
  (void)nowMs;
#endif
}

static void initImu() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(400000);
  delay(100);

  mpu.initialize();
  uint8_t devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpPacketSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  } else {
    dmpReady = false;
  }
}

static void updateImu() {
  if (!dmpReady) {
    return;
  }

  if (!mpu.dmpGetCurrentFIFOPacket(dmpFifoBuffer)) {
    return;
  }
  mpu.dmpGetQuaternion(&dmpQ, dmpFifoBuffer);
  mpu.dmpGetGravity(&dmpGravity, &dmpQ);
  mpu.dmpGetYawPitchRoll(dmpYpr, &dmpQ, &dmpGravity);

  imuYaw = dmpYpr[0] * 180.0f / PI;
  imuPitch = dmpYpr[1] * 180.0f / PI;
  imuRoll = dmpYpr[2] * 180.0f / PI;
}

static void sendImu() {
  uint8_t payload[12];
  writeF32(payload + 0, imuPitch);
  writeF32(payload + 4, imuRoll);
  writeF32(payload + 8, imuYaw);
  sendFrame(MSG_IMU_DATA, payload, sizeof(payload));
}

static void resetLidarScan() {
  scanCount = 0;
  scanReady = false;
}

static float rawAngleDeg(uint16_t raw) {
  return (float)(raw >> 1) / 64.0f;
}

static void processLidarPacket(const uint8_t *pkt, uint16_t len) {
  if (scanReady) {
    // Drop incoming packets until host side consumes the completed slice.
    return;
  }
  if (len < 10) {
    return;
  }
  uint8_t ct = pkt[2];
  uint8_t lsn = pkt[3];
  if (lsn == 0 || len < (uint16_t)(10 + lsn * 3)) {
    return;
  }

  bool newScan = (ct & 0x01) != 0;
  if (newScan && scanCount > 0) {
    captureTiltEnd = currentTilt;
    if (!readServoTilt(&captureTiltEnd)) {
      captureTiltEnd = currentTilt;
    }
    scanReady = true;
    return;
  }

  float start = rawAngleDeg((uint16_t)pkt[4] | ((uint16_t)pkt[5] << 8));
  float end = rawAngleDeg((uint16_t)pkt[6] | ((uint16_t)pkt[7] << 8));
  float span = end - start;
  if (span < 0.0f) {
    span += 360.0f;
  }

  for (uint8_t i = 0; i < lsn && scanCount < MAX_SCAN_POINTS; i++) {
    if (scanCount == 0 && i == 0) {
      captureTiltStart = currentTilt;
      if (!readServoTilt(&captureTiltStart)) {
        captureTiltStart = currentTilt;
      }
      captureTiltEnd = captureTiltStart;
    }
    const uint8_t *s = pkt + 10 + i * 3;
    float angle = start + (lsn > 1 ? span * i / (float)(lsn - 1) : 0.0f);
    while (angle >= 360.0f) {
      angle -= 360.0f;
    }
    scanPoints[scanCount].angle_x100 = (uint16_t)(angle * 100.0f + 0.5f);
    scanPoints[scanCount].dist_mm = (uint16_t)s[1] | ((uint16_t)s[2] << 8);
    scanPoints[scanCount].quality = s[0];
    scanCount++;
  }

  if (scanCount > 0) {
    captureTiltEnd = currentTilt;
    if (!readServoTilt(&captureTiltEnd)) {
      captureTiltEnd = currentTilt;
    }
  }
}

static void lidarStartMotor() {
  if (lidarMotorOn) {
    return;
  }
  const uint8_t cmd[] = {0xA5, 0x60};
  LidarSerial.write(cmd, sizeof(cmd));
  lidarMotorOn = true;
}

static void lidarStopMotor() {
  if (!lidarMotorOn) {
    return;
  }
  const uint8_t cmd[] = {0xA5, 0x65};
  LidarSerial.write(cmd, sizeof(cmd));
  lidarMotorOn = false;
}

static void pollLidar() {
  // Avoid starving other tasks when LiDAR stream is continuous.
  int budget = 512;
  while (budget-- > 0 && LidarSerial.available()) {
    uint8_t b = (uint8_t)LidarSerial.read();
    switch (lidarState) {
      case 0:
        if (b == 0xAA) {
          lidarBuf[0] = b;
          lidarIndex = 1;
          lidarState = 1;
        }
        break;
      case 1:
        if (b == 0x55) {
          lidarBuf[lidarIndex++] = b;
          lidarState = 2;
        } else {
          lidarState = 0;
        }
        break;
      case 2:
        lidarBuf[lidarIndex++] = b;
        if (lidarIndex == 4) {
          lidarExpected = 10 + (uint16_t)lidarBuf[3] * 3;
          if (lidarExpected > sizeof(lidarBuf)) {
            lidarState = 0;
          }
        }
        if (lidarIndex >= 4 && lidarIndex >= lidarExpected) {
          processLidarPacket(lidarBuf, lidarIndex);
          lidarState = 0;
        }
        break;
    }
  }
}

static void sendScanSlice() {
  uint16_t count = scanCount;
  uint16_t len = 22 + count * 5;
  if (len > MAX_PROTO_PAYLOAD) {
    count = (MAX_PROTO_PAYLOAD - 22) / 5;
    len = 22 + count * 5;
  }

  static uint8_t payload[MAX_PROTO_PAYLOAD];
  writeF32(payload + 0, currentTilt);
  writeF32(payload + 4, captureTiltStart);
  writeF32(payload + 8, captureTiltEnd);
  writeF32(payload + 12, imuPitch);
  writeF32(payload + 16, imuRoll);
  writeU16(payload + 20, count);
  for (uint16_t i = 0; i < count; i++) {
    uint8_t *p = payload + 22 + i * 5;
    writeU16(p + 0, scanPoints[i].angle_x100);
    writeU16(p + 2, scanPoints[i].dist_mm);
    p[4] = scanPoints[i].quality;
  }
  sendFrame(MSG_SCAN_SLICE, payload, len);
}

static uint32_t computeSettleMs(float deltaDeg) {
#if SCAN_SETTLE_DISABLED
  (void)deltaDeg;
  return 0;
#else
  float a = fabsf(deltaDeg);
  // Faster profile:
  // keep ~1.25s class at 45deg while reducing small-step overhead.
  uint32_t ms = (uint32_t)(110.0f + 25.0f * a + 0.5f);
  if (ms < SETTLE_MS_MIN) {
    ms = SETTLE_MS_MIN;
  }
  if (ms > SETTLE_MS_MAX) {
    ms = SETTLE_MS_MAX;
  }
  return ms;
#endif
}

static uint16_t computeTotalSteps(float minA, float maxA, float stepA) {
  if (stepA <= 0.0f || maxA < minA) {
    return 0;
  }
  return (uint16_t)(floorf((maxA - minA) / stepA + 0.5f) + 1);
}

static void updateSweepCommand() {
  uint32_t nowMs = millis();
  uint32_t dtMs = nowMs - lastSweepUpdateMs;
  if (dtMs == 0) {
    return;
  }
  lastSweepUpdateMs = nowMs;

  float dt = (float)dtMs / 1000.0f;
  float next = currentTilt + (float)sweepDirection * sweepSpeedDps * dt;
  if (next > tiltMax) {
    next = tiltMax;
  }
  if (next < tiltMin) {
    next = tiltMin;
  }
  currentTilt = next;
  setTiltAngle(currentTilt);
}

static void startScan(float minA, float maxA, float stepA) {
  tiltMin = constrain(minA, -TILT_SAFE_LIMIT, TILT_SAFE_LIMIT);
  tiltMax = constrain(maxA, -TILT_SAFE_LIMIT, TILT_SAFE_LIMIT);
  tiltStep = stepA > 0.0f ? stepA : TILT_STEP_DEFAULT;
  if (tiltMax < tiltMin) {
    float t = tiltMin;
    tiltMin = tiltMax;
    tiltMax = t;
  }
  totalSteps = computeTotalSteps(tiltMin, tiltMax, tiltStep);
  if (totalSteps == 0) {
    return;
  }
  float spacing = stepA > 0.0f ? stepA : TILT_STEP_DEFAULT;
  sweepSpeedDps = constrain(fabsf(spacing) * SWEEP_LIDAR_REV_HZ, SWEEP_SPEED_MIN_DPS, SWEEP_SPEED_MAX_DPS);
  lidarStartMotor();
  stepIndex = 0;
  slicesSent = 0;
  sweepDirection = 1;
  currentTilt = tiltMin;
  setTiltAngle(currentTilt);
  resetLidarScan();
  captureTiltStart = currentTilt;
  captureTiltEnd = currentTilt;
  scanState = S3D_PREPARE;
  stateStartedMs = millis();
  lastSweepUpdateMs = stateStartedMs;
  sendStatus(0, stepIndex, totalSteps);
}

static void stopScan() {
  scanState = S3D_IDLE;
  lidarStopMotor();
  resetLidarScan();
  setTiltAngle(0.0f);
  sendStatus(2, stepIndex, totalSteps);
}

static void updateScanState() {
  switch (scanState) {
    case S3D_IDLE:
      break;
    case S3D_PREPARE:
      if (millis() - stateStartedMs >= SWEEP_PREPARE_MS) {
        resetLidarScan();
        lastSweepUpdateMs = millis();
        scanState = S3D_SWEEPING;
      }
      break;
    case S3D_SWEEPING:
      updateSweepCommand();
      if (scanReady) {
        sendScanSlice();
        resetLidarScan();
        slicesSent++;
        uint16_t progress = slicesSent;
        if (progress > totalSteps) {
          progress = totalSteps;
        }
        sendStatus(1, progress, totalSteps);
      }
      // Capture both directions:
      //  1) up-sweep: tiltMin -> tiltMax
      //  2) down-sweep: tiltMax -> tiltMin
      if (sweepDirection > 0 && currentTilt >= (tiltMax - 0.01f)) {
        sweepDirection = -1;
      } else if (sweepDirection < 0 && currentTilt <= (tiltMin + 0.01f)) {
        stateStartedMs = millis();
        scanState = S3D_FINALIZING;
      }
      break;
    case S3D_FINALIZING:
      setTiltAngle(tiltMin);
      if (scanReady) {
        sendScanSlice();
        resetLidarScan();
        slicesSent++;
        uint16_t progress = slicesSent;
        if (progress > totalSteps) {
          progress = totalSteps;
        }
        sendStatus(1, progress, totalSteps);
      }
      if (millis() - stateStartedMs >= SWEEP_FINALIZE_MS) {
        scanState = S3D_COMPLETE;
      }
      break;
    case S3D_COMPLETE:
      setTiltAngle(0.0f);
      lidarStopMotor();
      sendStatus(2, totalSteps, totalSteps);
      scanState = S3D_IDLE;
      break;
  }
}

static void handleCommand(uint8_t type, const uint8_t *payload, uint16_t len) {
  if (type == CMD_SERVO_ANGLE && len >= 4) {
    setTiltAngle(readF32(payload));
  } else if (type == CMD_SCAN_START) {
    if (len >= 12) {
      startScan(readF32(payload), readF32(payload + 4), readF32(payload + 8));
    } else {
      startScan(TILT_MIN_DEFAULT, TILT_MAX_DEFAULT, TILT_STEP_DEFAULT);
    }
  } else if (type == CMD_SCAN_STOP) {
    stopScan();
  }
}

static void pollUsbCommands() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    switch (rxState) {
      case 0:
        rxState = (b == PROTO_SYNC1) ? 1 : 0;
        break;
      case 1:
        rxState = (b == PROTO_SYNC2) ? 2 : 0;
        break;
      case 2:
        rxType = b;
        rxState = 3;
        break;
      case 3:
        rxLen = b;
        rxState = 4;
        break;
      case 4:
        rxLen |= ((uint16_t)b << 8);
        rxIndex = 0;
        if (rxLen > sizeof(rxFrame)) {
          rxState = 0;
        } else {
          rxState = rxLen == 0 ? 6 : 5;
        }
        break;
      case 5:
        rxFrame[rxIndex++] = b;
        if (rxIndex >= rxLen) {
          rxState = 6;
        }
        break;
      case 6: {
        uint8_t cs = checksum(rxType, rxLen, rxFrame);
        if (cs == b) {
          handleCommand(rxType, rxFrame, rxLen);
        }
        rxState = 0;
        break;
      }
    }
  }
}

void setup() {
  Serial.begin(USB_BAUDRATE);
  Serial0.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_RXD, SERVO_TXD);
  LidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RXD, LIDAR_TXD);
  servo.pSerial = &Serial0;
  initServo();
  // If no servo response, retry once with swapped UART pins.
  if (servoReadId < 0) {
    Serial0.end();
    delay(50);
    Serial0.begin(SERVO_BAUDRATE, SERIAL_8N1, SERVO_TXD, SERVO_RXD);
    delay(50);
    initServo();
  }
#if USB_TEXT_DEBUG_MODE
  Serial.println("[dbg] USB_TEXT_DEBUG_MODE=1 (binary TX disabled)");
  Serial.print("[dbg] init tx_id=");
  Serial.println((int)servoTxId);
#endif
  initImu();

  lidarStopMotor();
  setTiltAngle(0.0f);
}

void loop() {
  pollLidar();
  pollUsbCommands();

  uint32_t now = millis();
  if (now - lastImuReadMs >= IMU_READ_MS) {
    lastImuReadMs = now;
    updateImu();
  }
  if (now - lastImuSendMs >= IMU_SEND_MS) {
    lastImuSendMs = now;
    sendImu();
  }
  debugServoImuStatus(now);
  updateScanState();
}
