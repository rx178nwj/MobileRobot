/**
 * PR2040_DC_MOTOR_DRIVER Firmware
 *
 * Hardware : PR2040_BASESYSTEM REV1.5.0  (N.W Works)
 * MCU      : RP2040
 * Arduino  : arduino-pico (Earle Philhower)
 *            https://github.com/earlephilhower/arduino-pico
 *
 * ---- Pin Assignment (from schematic) ----------------------------------------
 *
 * Motor (TB67H451 H-bridge x4)
 *   Motor1  IN1=GPIO8   IN2=GPIO9
 *   Motor2  IN1=GPIO10  IN2=GPIO11
 *   Motor3  IN1=GPIO14  IN2=GPIO15
 *   Motor4  IN1=GPIO16  IN2=GPIO17
 *
 * Encoder (Quadrature x4)
 *   Enc1  A=GPIO18  B=GPIO19
 *   Enc2  A=GPIO20  B=GPIO21
 *   Enc3  A=GPIO22  B=GPIO23
 *   Enc4  A=GPIO24  B=GPIO25
 *
 * ADC Internal (RP2040, current sensing via INA21x + LPF)
 *   CH0=GPIO26  CH1=GPIO27  CH2=GPIO28  CH3=GPIO29
 *
 * Controller I2C (I2C1 slave, addr=0x60)
 *   SDA=GPIO2  SCL=GPIO3
 *
 * IMU MPU-6881 (I2C0)
 *   INT=GPIO0  SDA=GPIO12  SCL=GPIO13
 *
 * External ADC MCP3208/MAX11125 (SPI0)
 *   MISO=GPIO4  CS=GPIO5  SCK=GPIO6  MOSI=GPIO7
 *
 * ---- Control Modes ----------------------------------------------------------
 *
 * Each wheel has three control modes:
 *   0  DIRECT   : duty set directly, no feedback (-1000..+1000)
 *   1  VELOCITY : velocity PID, target in counts/sec
 *   2  POSITION : cascade control (outer P -> velocity PI -> duty)
 *
 * Default PID gains (velocity inner loop):
 *   Kp=1.0  Ki=0.05  Kd=0.0  output: -1000..+1000
 * Default position outer loop:
 *   posKp=0.5  maxVelCps=2000 counts/sec
 *
 * ---- I2C Slave Register Map (Wire1, addr=I2C_SLAVE_ADDR) -------------------
 *
 * Write (controller -> board):
 *   0x01 REG_MOTOR_ALL      [ 8 B]  int16 x4  duty -1000..+1000 (little-endian)
 *   0x02 REG_MOTOR_SINGLE   [ 3 B]  uint8 index(0-3) + int16 duty
 *   0x03 REG_STOP_ALL       [ 0 B]  coast all motors
 *   0x04 REG_BRAKE_ALL      [ 0 B]  active brake all motors
 *   0x05 REG_RESET_ENC      [ 0 B]  reset encoder counts
 *   0x06 REG_SET_MODE_ALL   [ 4 B]  uint8 x4  (0=DIRECT,1=VEL,2=POS)
 *   0x07 REG_SET_MODE_SINGLE[ 2 B]  uint8 index + uint8 mode
 *   0x08 REG_SET_VEL_ALL    [16 B]  float x4  velocity targets (cps)
 *   0x09 REG_SET_VEL_SINGLE [ 5 B]  uint8 index + float target
 *   0x0A REG_SET_POS_ALL    [16 B]  int32 x4  position targets (counts)
 *   0x0B REG_SET_POS_SINGLE [ 5 B]  uint8 index + int32 target
 *   0x0C REG_SET_VEL_PID    [13 B]  uint8 index + float kp, ki, kd
 *   0x0D REG_SET_POS_GAINS  [ 9 B]  uint8 index + float posKp, maxVelCps
 *
 * Read (controller reads after writing register address):
 *   0x10 REG_ENC0  [4 B] int32  encoder 0 count
 *   0x11 REG_ENC1  [4 B] int32  encoder 1 count
 *   0x12 REG_ENC2  [4 B] int32  encoder 2 count
 *   0x13 REG_ENC3  [4 B] int32  encoder 3 count
 *   0x14 REG_VEL0  [4 B] int32  wheel 0 velocity (counts/sec)
 *   0x15 REG_VEL1  [4 B] int32  wheel 1 velocity
 *   0x16 REG_VEL2  [4 B] int32  wheel 2 velocity
 *   0x17 REG_VEL3  [4 B] int32  wheel 3 velocity
 *   0x20 REG_ADC0  [2 B] int16  ADC ch0 raw (0..4095)
 *   0x21 REG_ADC1  [2 B] int16  ADC ch1 raw
 *   0x22 REG_ADC2  [2 B] int16  ADC ch2 raw
 *   0x23 REG_ADC3  [2 B] int16  ADC ch3 raw
 *   0x30 REG_STATUS [44 B] enc(int32x4)+vel(int32x4)+adc(int16x4)+ts(uint32)
 *   0xFF REG_DEVICE_ID [2 B] 0x20 0x40
 *
 * ---- USB Serial Protocol ----------------------------------------------------
 *
 * USB CDC Serial (115200 baud)
 * Packet format: [0xAA][CMD][LEN][DATA x LEN][CHECKSUM]
 * CHECKSUM = XOR(CMD, LEN, DATA[0..LEN-1])
 *
 * Commands (Host -> Board):
 *   0x01  CMD_SET_MOTORS       data: int16 x4 (duty -1000..+1000, little-endian)
 *   0x02  CMD_STOP_ALL         data: none (coast)
 *   0x03  CMD_BRAKE_ALL        data: none (active brake)
 *   0x04  CMD_SET_MOTOR_SINGLE data: uint8 index(0-3), int16 duty
 *   0x10  CMD_REQUEST_STATUS   data: none (triggers one status packet)
 *   0x11  CMD_RESET_ENCODERS   data: none
 *   0x12  CMD_REQUEST_EXT_ADC  data: none → RESP_EXT_ADC int16 x8 (16 B)
 *   0x20  CMD_SET_MODE_ALL     data: uint8 x4  (0=DIRECT,1=VEL,2=POS)
 *   0x21  CMD_SET_MODE_SINGLE  data: uint8 index, uint8 mode
 *   0x22  CMD_SET_VEL_ALL      data: float x4  velocity targets (cps)
 *   0x23  CMD_SET_VEL_SINGLE   data: uint8 index, float target
 *   0x24  CMD_SET_POS_ALL      data: int32 x4  position targets (counts)
 *   0x25  CMD_SET_POS_SINGLE   data: uint8 index, int32 target
 *   0x26  CMD_SET_VEL_PID      data: uint8 index, float kp, ki, kd
 *   0x27  CMD_SET_POS_GAINS    data: uint8 index, float posKp, maxVelCps
 *
 * Responses (Board -> Host):
 *   0x80  RESP_ACK     data: [cmd]
 *   0x81  RESP_NAK     data: [cmd]
 *   0x91  RESP_STATUS  [44 B] enc(int32x4)+vel(int32x4)+adc(int16x4)+ts(uint32)
 *   0x92  RESP_EXT_ADC [16 B] int16 x8 MCP3208 ch0..7
 *
 * Status is also sent automatically at STATUS_INTERVAL_MS (10 ms = 100 Hz).
 */

#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include <MadgwickAHRS.h>
#include "Config.h"
#include "WheelController.h"
#include "MCP3208.h"
#include "SensorConverter.h"

// ---------------------------------------------------------------------------
// Instances
// ---------------------------------------------------------------------------
static MCP3208 extAdc(SPI0_CSN_PIN, SPI);

static WheelController wheels[4] = {
    WheelController(MOTOR1_IN1, MOTOR1_IN2, ENC1_A, ENC1_B),
    WheelController(MOTOR2_IN1, MOTOR2_IN2, ENC2_A, ENC2_B),
    WheelController(MOTOR3_IN1, MOTOR3_IN2, ENC3_A, ENC3_B),
    WheelController(MOTOR4_IN1, MOTOR4_IN2, ENC4_A, ENC4_B),
};

// ---------------------------------------------------------------------------
// USB Serial receive state machine
// ---------------------------------------------------------------------------
static uint8_t  rxBuf[MAX_PACKET_DATA + 4];
static uint8_t  rxIdx      = 0;
static bool     rxInPacket = false;
static uint32_t rxStartMs  = 0;
static constexpr uint32_t RX_TIMEOUT_MS = 50;

// ---------------------------------------------------------------------------
// I2C1 slave state
// ---------------------------------------------------------------------------
// Read buffer: filled when controller sets a register address.
// onRequest() sends from this buffer.
static uint8_t  i2cReadBuf[64];
static uint8_t  i2cReadLen = 0;

// Write buffer: raw bytes received from controller in onReceive().
// Processed in loop() to avoid heavy work inside the ISR callback.
static uint8_t  i2cWriteBuf[32];
static uint8_t  i2cWriteLen  = 0;
static volatile bool i2cWritePending = false;

// ---------------------------------------------------------------------------
// External ADC buffer (updated periodically in loop, served from cache)
// ---------------------------------------------------------------------------
static int16_t  extAdcBuf[8] = {0};  // MCP3208 ch0..7, last sampled values

// ---------------------------------------------------------------------------
// IMU — MPU-6881 direct Wire + Madgwick AHRS filter
// ---------------------------------------------------------------------------
static Madgwick  madgwick;
static bool      imuOk        = false;
static uint8_t   imuAddr      = MPU6881_I2C_ADDR;  // auto-detected address
static float     imuAx = 0, imuAy = 0, imuAz = 0;  // [g]
static float     imuGx = 0, imuGy = 0, imuGz = 0;  // [dps]
static float     imuGyroBias[3]  = {0};  // gyro bias [dps]
static float     imuAccelBias[3] = {0};  // accel bias [g] (gravity removed)

// Wheel calibration scale (applied to CPS_TO_MMPS at runtime)
static float     wheelCalScale   = 1.0f;

// ---------------------------------------------------------------------------
// IMU calibration flash persistence (EEPROM emulation)
// ---------------------------------------------------------------------------
struct ImuCalData {
    uint32_t magic;        // must equal IMU_CAL_MAGIC to be valid
    float    gyroBias[3];  // [dps]
    float    accelBias[3]; // [g]
};
constexpr uint32_t IMU_CAL_MAGIC       = 0xCA1BEEF0UL;
constexpr int      IMU_CAL_EEPROM_ADDR = 0;   // ImuCalData starts here (28 bytes)

// Wheel calibration scale (stored after IMU cal, aligned to 32)
struct WheelCalData {
    uint32_t magic;  // must equal WHEEL_CAL_MAGIC
    float    scale;  // runtime multiplier for CPS_TO_MMPS (1.0 = no correction)
};
constexpr uint32_t WHEEL_CAL_MAGIC       = 0xCA1B10C1UL;  // bumped: gear ratio updated to 54.78
constexpr int      WHEEL_CAL_EEPROM_ADDR = 32; // after ImuCalData
constexpr int      IMU_CAL_EEPROM_SIZE   = 64; // total EEPROM bytes reserved

// ---------------------------------------------------------------------------
// Timing
// ---------------------------------------------------------------------------
static uint32_t lastStatusMs   = 0;
static uint32_t lastControlMs  = 0;
static uint32_t lastExtAdcMs   = 0;
static uint32_t lastImuMs      = 0;
static uint32_t lastImuRetryMs = 0;
static uint32_t lastLedMs      = 0;

static constexpr uint32_t EXT_ADC_INTERVAL_MS  = 10;    // 100 Hz
static constexpr uint32_t IMU_RETRY_INTERVAL_MS = 2000; // retry init if not connected

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void     processPacket(uint8_t cmd, const uint8_t* data, uint8_t len);
static void     sendStatus();
static void     sendAck(uint8_t cmd);
static void     sendNak(uint8_t cmd);
static uint8_t  calcChecksum(uint8_t cmd, uint8_t len, const uint8_t* data);
static void     emergencyStop();
static void     sendExtAdc();
static void     setBodyVelocity(float linear_mmps, float omega_degps);
static bool     initIMU();
static void     readIMU();
static void     sendIMU();
static void     calibrateIMU();
static void     saveCalibration();
static bool     loadCalibration();
static void     saveWheelCal();
static bool     loadWheelCal();
static void     buildI2CReadBuffer(uint8_t reg);
static void     processI2CWrite(const uint8_t* data, uint8_t len);
static void     onI2CReceive(int numBytes);
static void     onI2CRequest();

// ---------------------------------------------------------------------------
// setup
// ---------------------------------------------------------------------------
void setup() {
    // USB Serial
    Serial.begin(SERIAL_BAUD);

    // PWM for all motor pins: 20 kHz, range 0..1000
    analogWriteFreq(PWM_FREQ_HZ);
    analogWriteRange(PWM_DUTY_MAX);

    // Wheel controllers (Motor + Encoder)
    for (int i = 0; i < 4; i++) {
        wheels[i].begin(i);
    }

    // Status LEDs
    pinMode(LED_RED_PIN,    OUTPUT);
    pinMode(LED_GREEN_PIN,  OUTPUT);
    pinMode(LED_YELLOW_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN,    LOW);
    digitalWrite(LED_GREEN_PIN,  LOW);
    digitalWrite(LED_YELLOW_PIN, LOW);

    // Internal ADC resolution: 12 bit
    analogReadResolution(12);

    // SPI0 for MCP3208 external ADC
    SPI.setRX(SPI0_RX_PIN);
    SPI.setSCK(SPI0_SCK_PIN);
    SPI.setTX(SPI0_TX_PIN);
    extAdc.begin();  // also sets CS pin high

    // IMU interrupt pin
    pinMode(IMU_INT_PIN, INPUT);

    // I2C0 for MPU-6881 (master)
    Wire.setSDA(I2C0_SDA_PIN);
    Wire.setSCL(I2C0_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);

    // Load IMU calibration from flash (if previously saved)
    EEPROM.begin(IMU_CAL_EEPROM_SIZE);
    bool calLoaded = loadCalibration();

    // MPU-6881 direct initialization
    imuOk = initIMU();
    if (imuOk) {
        madgwick.begin(1000.0f / IMU_INTERVAL_MS);
        readIMU();  // prime the filter
    }
    if (calLoaded) {
        Serial.println("IMU calibration loaded from flash.");
    }
    if (loadWheelCal()) {
        Serial.print("Wheel cal loaded: scale=");
        Serial.println(wheelCalScale, 6);
    }

    // I2C1 for controller communication (slave, GPIO2=SDA, GPIO3=SCL)
    Wire1.setSDA(I2C1_SDA_PIN);
    Wire1.setSCL(I2C1_SCL_PIN);
    Wire1.begin(I2C_SLAVE_ADDR);
    Wire1.onReceive(onI2CReceive);
    Wire1.onRequest(onI2CRequest);

    uint32_t now = millis();
    lastControlMs = now;
    lastStatusMs  = now;
    lastExtAdcMs  = now;

    // Initial ADC sample so buffer is valid immediately
    extAdc.readAll(extAdcBuf);

    Serial.println("PR2040_MotorDriver Ready");
}

// ---------------------------------------------------------------------------
// loop
// ---------------------------------------------------------------------------
void loop() {
    uint32_t now = millis();

    // --- Control loop (CONTROL_INTERVAL_MS = 5 ms) ---
    if (now - lastControlMs >= CONTROL_INTERVAL_MS) {
        uint32_t dt = now - lastControlMs;
        lastControlMs = now;
        for (int i = 0; i < 4; i++) {
            wheels[i].update(dt);
        }
    }

    // --- External ADC periodic sampling (EXT_ADC_INTERVAL_MS = 10 ms) ---
    if (now - lastExtAdcMs >= EXT_ADC_INTERVAL_MS) {
        lastExtAdcMs = now;
        extAdc.readAll(extAdcBuf);
    }

    // --- IMU periodic sampling (IMU_INTERVAL_MS = 10 ms = 100 Hz) ---
    if (now - lastImuMs >= IMU_INTERVAL_MS) {
        lastImuMs = now;
        readIMU();
    }

    // --- IMU auto-retry if not yet connected ---
    if (!imuOk && (now - lastImuRetryMs >= IMU_RETRY_INTERVAL_MS)) {
        lastImuRetryMs = now;
        imuOk = initIMU();
        if (imuOk) {
            madgwick.begin(1000.0f / IMU_INTERVAL_MS);
            readIMU();
            Serial.println("IMU connected OK");
        }
    }

    // --- Parse incoming serial bytes ---
    while (Serial.available()) {
        uint8_t b = (uint8_t)Serial.read();

        if (!rxInPacket) {
            if (b == PACKET_HEADER) {
                rxInPacket = true;
                rxIdx      = 0;
                rxBuf[rxIdx++] = b;
                rxStartMs  = millis();
            }
        } else {
            // Timeout check
            if (millis() - rxStartMs > RX_TIMEOUT_MS) {
                rxInPacket = false;
                rxIdx      = 0;
                continue;
            }

            rxBuf[rxIdx++] = b;

            // Overflow guard
            if (rxIdx >= sizeof(rxBuf)) {
                rxInPacket = false;
                rxIdx      = 0;
                continue;
            }

            // Once we have HEADER + CMD + LEN, we know expected total length
            if (rxIdx >= 3) {
                uint8_t len = rxBuf[2];
                if (len > MAX_PACKET_DATA) {
                    rxInPacket = false;
                    rxIdx      = 0;
                    continue;
                }
                uint8_t expectedTotal = 1 + 1 + 1 + len + 1;
                if (rxIdx >= expectedTotal) {
                    uint8_t cmd         = rxBuf[1];
                    const uint8_t* data = &rxBuf[3];
                    uint8_t rxCksum     = rxBuf[3 + len];
                    uint8_t calcCksum   = calcChecksum(cmd, len, data);

                    if (rxCksum == calcCksum) {
                        processPacket(cmd, data, len);
                    } else {
                        sendNak(cmd);
                    }

                    rxInPacket = false;
                    rxIdx      = 0;
                }
            }
        }
    }

    // --- Process pending I2C write command (deferred from ISR) ---
    if (i2cWritePending) {
        i2cWritePending = false;
        processI2CWrite(i2cWriteBuf, i2cWriteLen);
    }

    // --- Periodic status (USB Serial) ---
    now = millis();
    if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
        lastStatusMs = now;
        sendStatus();
    }

    // --- Yellow LED heartbeat (500ms toggle = 1Hz blink) ---
    if (now - lastLedMs >= 500) {
        lastLedMs = now;
        digitalWrite(LED_YELLOW_PIN, !digitalRead(LED_YELLOW_PIN));
    }

    // --- Green LED: ON while any motor driver is supplying current ---
    // Active when: VELOCITY/POSITION mode (PID running), or DIRECT with wheel spinning
    {
        bool anyActive = false;
        for (int i = 0; i < 4; i++) {
            ControlMode m = wheels[i].getMode();
            if (m == ControlMode::VELOCITY || m == ControlMode::POSITION) {
                anyActive = true;
                break;
            }
            if (abs(wheels[i].getVelocityInt()) > 20) {
                anyActive = true;
                break;
            }
        }
        digitalWrite(LED_GREEN_PIN, anyActive ? HIGH : LOW);
    }
}

// ---------------------------------------------------------------------------
// processPacket
// ---------------------------------------------------------------------------
static void processPacket(uint8_t cmd, const uint8_t* data, uint8_t len) {
    switch (cmd) {

    // --- DIRECT duty control ---

    case CMD_SET_MOTORS: {
        if (len != 8) { sendNak(cmd); return; }
        for (int i = 0; i < 4; i++) {
            int16_t duty;
            memcpy(&duty, &data[i * 2], 2);
            wheels[i].setDuty(duty);
        }
        sendAck(cmd);
        break;
    }

    case CMD_SET_MOTOR_SINGLE: {
        if (len != 3) { sendNak(cmd); return; }
        uint8_t idx = data[0];
        if (idx >= 4) { sendNak(cmd); return; }
        int16_t duty;
        memcpy(&duty, &data[1], 2);
        wheels[idx].setDuty(duty);
        sendAck(cmd);
        break;
    }

    case CMD_STOP_ALL: {
        emergencyStop();
        sendAck(cmd);
        break;
    }

    case CMD_BRAKE_ALL: {
        for (int i = 0; i < 4; i++) wheels[i].brake();
        sendAck(cmd);
        break;
    }

    // --- Status / encoders ---

    case CMD_REQUEST_STATUS: {
        sendStatus();
        break;
    }

    case CMD_RESET_ENCODERS: {
        for (int i = 0; i < 4; i++) wheels[i].resetEncoder();
        sendAck(cmd);
        break;
    }

    case CMD_REQUEST_EXT_ADC: {
        sendExtAdc();
        break;
    }

    // --- Body (differential drive) velocity control ---

    case CMD_SET_BODY_VEL: {
        if (len != 8) { sendNak(cmd); return; }
        float linear_mmps, omega_degps;
        memcpy(&linear_mmps, &data[0], 4);
        memcpy(&omega_degps, &data[4], 4);
        setBodyVelocity(linear_mmps, omega_degps);
        sendAck(cmd);
        break;
    }

    case CMD_BODY_STOP: {
        wheels[BODY_LEFT_IDX].stop();
        wheels[BODY_RIGHT_IDX].stop();
        sendAck(cmd);
        break;
    }

    case CMD_SET_WHEEL_SCALE: {
        if (len != 4) { sendNak(cmd); return; }
        float scale;
        memcpy(&scale, &data[0], 4);
        if (scale < 0.5f || scale > 2.0f) { sendNak(cmd); return; }  // sanity check
        wheelCalScale = scale;
        saveWheelCal();
        Serial.print("Wheel cal scale saved: "); Serial.println(scale, 6);
        sendAck(cmd);
        break;
    }

    case CMD_REQUEST_IMU: {
        if (!imuOk) { sendNak(cmd); return; }
        sendIMU();
        break;
    }

    case CMD_CALIBRATE_IMU: {
        calibrateIMU();  // blocking ~5s — keep IMU level and still
        sendAck(cmd);
        break;
    }

    // --- Mode control ---

    case CMD_SET_MODE_ALL: {
        if (len != 4) { sendNak(cmd); return; }
        for (int i = 0; i < 4; i++) {
            wheels[i].setMode((ControlMode)data[i]);
        }
        sendAck(cmd);
        break;
    }

    case CMD_SET_MODE_SINGLE: {
        if (len != 2) { sendNak(cmd); return; }
        uint8_t idx = data[0];
        if (idx >= 4) { sendNak(cmd); return; }
        wheels[idx].setMode((ControlMode)data[1]);
        sendAck(cmd);
        break;
    }

    // --- Velocity targets ---

    case CMD_SET_VEL_ALL: {
        if (len != 16) { sendNak(cmd); return; }
        for (int i = 0; i < 4; i++) {
            float target;
            memcpy(&target, &data[i * 4], 4);
            wheels[i].setVelocityTarget(target);
        }
        sendAck(cmd);
        break;
    }

    case CMD_SET_VEL_SINGLE: {
        if (len != 5) { sendNak(cmd); return; }
        uint8_t idx = data[0];
        if (idx >= 4) { sendNak(cmd); return; }
        float target;
        memcpy(&target, &data[1], 4);
        wheels[idx].setVelocityTarget(target);
        sendAck(cmd);
        break;
    }

    // --- Position targets ---

    case CMD_SET_POS_ALL: {
        if (len != 16) { sendNak(cmd); return; }
        for (int i = 0; i < 4; i++) {
            int32_t target;
            memcpy(&target, &data[i * 4], 4);
            wheels[i].setPositionTarget(target);
        }
        sendAck(cmd);
        break;
    }

    case CMD_SET_POS_SINGLE: {
        if (len != 5) { sendNak(cmd); return; }
        uint8_t idx = data[0];
        if (idx >= 4) { sendNak(cmd); return; }
        int32_t target;
        memcpy(&target, &data[1], 4);
        wheels[idx].setPositionTarget(target);
        sendAck(cmd);
        break;
    }

    // --- PID / gains configuration ---

    case CMD_SET_VEL_PID: {
        if (len != 13) { sendNak(cmd); return; }
        uint8_t idx = data[0];
        if (idx >= 4) { sendNak(cmd); return; }
        float kp, ki, kd;
        memcpy(&kp, &data[1], 4);
        memcpy(&ki, &data[5], 4);
        memcpy(&kd, &data[9], 4);
        wheels[idx].setVelocityPID(kp, ki, kd);
        sendAck(cmd);
        break;
    }

    case CMD_SET_POS_GAINS: {
        if (len != 9) { sendNak(cmd); return; }
        uint8_t idx = data[0];
        if (idx >= 4) { sendNak(cmd); return; }
        float posKp, maxVelCps;
        memcpy(&posKp,      &data[1], 4);
        memcpy(&maxVelCps,  &data[5], 4);
        wheels[idx].setPositionGains(posKp, maxVelCps);
        sendAck(cmd);
        break;
    }

    default:
        sendNak(cmd);
        break;
    }
}

// ---------------------------------------------------------------------------
// sendStatus
// Response data layout (STATUS_DATA_LEN = 44 bytes):
//   [ 0..15] int32 x4   : encoder counts (little-endian)
//   [16..31] int32 x4   : velocities in counts/sec (little-endian)
//   [32..39] int16 x4   : ADC raw 0..4095 (little-endian)
//   [40..43] uint32     : timestamp (millis, little-endian)
// ---------------------------------------------------------------------------
static void sendStatus() {
    uint8_t data[STATUS_DATA_LEN];
    uint8_t off = 0;

    // Encoder counts
    for (int i = 0; i < 4; i++) {
        int32_t cnt = wheels[i].getPosition();
        memcpy(&data[off], &cnt, 4);
        off += 4;
    }

    // Velocities (int32 counts/sec)
    for (int i = 0; i < 4; i++) {
        int32_t vel = wheels[i].getVelocityInt();
        memcpy(&data[off], &vel, 4);
        off += 4;
    }

    // ADC current sensing (12-bit, 0..4095)
    for (int i = 0; i < 4; i++) {
        int16_t adc = (int16_t)analogRead(ADC_CURRENT_CH[i]);
        memcpy(&data[off], &adc, 2);
        off += 2;
    }

    // Timestamp
    uint32_t ts = millis();
    memcpy(&data[off], &ts, 4);
    off += 4;

    uint8_t cksum = calcChecksum(RESP_STATUS, STATUS_DATA_LEN, data);
    Serial.write(PACKET_HEADER);
    Serial.write(RESP_STATUS);
    Serial.write(STATUS_DATA_LEN);
    Serial.write(data, STATUS_DATA_LEN);
    Serial.write(cksum);
}

// ---------------------------------------------------------------------------
// sendAck / sendNak
// ---------------------------------------------------------------------------
static void sendAck(uint8_t cmd) {
    uint8_t cs = calcChecksum(RESP_ACK, 1, &cmd);
    Serial.write(PACKET_HEADER);
    Serial.write(RESP_ACK);
    Serial.write((uint8_t)1);
    Serial.write(cmd);
    Serial.write(cs);
}

static void sendNak(uint8_t cmd) {
    uint8_t cs = calcChecksum(RESP_NAK, 1, &cmd);
    Serial.write(PACKET_HEADER);
    Serial.write(RESP_NAK);
    Serial.write((uint8_t)1);
    Serial.write(cmd);
    Serial.write(cs);
}

// ---------------------------------------------------------------------------
// calcChecksum: XOR of cmd, len, and all data bytes
// ---------------------------------------------------------------------------
static uint8_t calcChecksum(uint8_t cmd, uint8_t len, const uint8_t* data) {
    uint8_t cs = cmd ^ len;
    for (uint8_t i = 0; i < len; i++) {
        cs ^= data[i];
    }
    return cs;
}

// ---------------------------------------------------------------------------
// emergencyStop: coast all motors immediately, switch to DIRECT mode
// ---------------------------------------------------------------------------
static void emergencyStop() {
    for (int i = 0; i < 4; i++) {
        wheels[i].stop();
    }
}

// ---------------------------------------------------------------------------
// setBodyVelocity
// Differential drive kinematics.
//   V_L [mm/s] = linear - ω_rad × (WHEEL_BASE_MM / 2)
//   V_R [mm/s] = linear + ω_rad × (WHEEL_BASE_MM / 2)
// BODY_LEFT/RIGHT_DIR: +1 or -1 to match physical motor mounting orientation.
// Automatically sets both drive wheels to VELOCITY mode.
// ---------------------------------------------------------------------------
static void setBodyVelocity(float linear_mmps, float omega_degps) {
    const float omega_radps = omega_degps * (3.14159265f / 180.0f);
    const float half_base   = WHEEL_BASE_MM * 0.5f;

    float v_left_mmps  = linear_mmps - omega_radps * half_base;
    float v_right_mmps = linear_mmps + omega_radps * half_base;

    // Convert mm/s → counts/s, apply mounting direction and wheel cal scale
    const float effective_cps = CPS_TO_MMPS * wheelCalScale;
    float cps_left  = BODY_LEFT_DIR  * v_left_mmps  / effective_cps;
    float cps_right = BODY_RIGHT_DIR * v_right_mmps / effective_cps;

    wheels[BODY_LEFT_IDX].setVelocityTarget(cps_left);
    wheels[BODY_RIGHT_IDX].setVelocityTarget(cps_right);
}

// ---------------------------------------------------------------------------
// sendExtAdc
// Response data layout (16 bytes): int16 x8 MCP3208 ch0..7 (little-endian)
// ---------------------------------------------------------------------------
static void sendExtAdc() {
    constexpr uint8_t DATA_LEN = 16;
    uint8_t data[DATA_LEN];
    for (int i = 0; i < 8; i++) {
        memcpy(&data[i * 2], &extAdcBuf[i], 2);
    }
    uint8_t cksum = calcChecksum(RESP_EXT_ADC, DATA_LEN, data);
    Serial.write(PACKET_HEADER);
    Serial.write(RESP_EXT_ADC);
    Serial.write(DATA_LEN);
    Serial.write(data, DATA_LEN);
    Serial.write(cksum);
}

// ---------------------------------------------------------------------------
// initIMU
// Direct Wire initialization of MPU-6881.
// Returns true if WHO_AM_I passes and configuration succeeds.
// ---------------------------------------------------------------------------
static bool initIMU() {
    // --- I2C bus scan: find IMU at 0x68 or 0x69 ---
    bool found = false;
    for (uint8_t addr : {(uint8_t)0x68, (uint8_t)0x69}) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        Serial.print("I2C scan 0x");
        Serial.print(addr, HEX);
        Serial.print(": ");
        Serial.println(err == 0 ? "ACK" : "NAK");
        if (err == 0) { imuAddr = addr; found = true; break; }
    }
    if (!found) {
        Serial.println("IMU: no device found on I2C bus (0x68, 0x69)");
        return false;
    }

    // --- WHO_AM_I check (accept known MPU-60xx family values) ---
    Wire.beginTransmission(imuAddr);
    Wire.write(MPU_REG_WHO_AM_I);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom((int)imuAddr, 1);
    if (!Wire.available()) return false;
    uint8_t whoami = Wire.read();
    Serial.print("IMU WHO_AM_I=0x"); Serial.println(whoami, HEX);
    // Accept: 0x70=MPU-6500/6881, 0x19=MPU-6886, 0x71, 0x74=ICM-20689, 0x68, 0x69
    const uint8_t known[] = {0x68, 0x69, 0x19, 0x70, 0x71, 0x74, 0x75};
    bool ok = false;
    for (uint8_t v : known) { if (whoami == v) { ok = true; break; } }
    if (!ok) {
        Serial.print("IMU WHO_AM_I=0x");
        Serial.print(whoami, HEX);
        Serial.println(" - unknown chip, attempting init anyway");
        // Fall through: try to initialize anyway
    }

    auto wrReg = [](uint8_t reg, uint8_t val) {
        Wire.beginTransmission(imuAddr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    };

    // Reset
    wrReg(MPU_REG_PWR_MGMT_1, 0x80);
    delay(100);
    // Wake up, PLL clock source
    wrReg(MPU_REG_PWR_MGMT_1, 0x01);
    delay(50);
    // DLPF: bandwidth ~42 Hz (config=3)
    wrReg(MPU_REG_CONFIG, 0x03);
    // Sample rate = 1 kHz / (1 + SMPLRT_DIV)
    //   SMPLRT_DIV=9 → 100 Hz (matches IMU_INTERVAL_MS=10)
    wrReg(MPU_REG_SMPLRT_DIV, 9);
    // Gyro full-scale: ±250°/s (bits[4:3]=00)
    wrReg(MPU_REG_GYRO_CONFIG, 0x00);
    // Accel full-scale: ±2g (bits[4:3]=00)
    wrReg(MPU_REG_ACCEL_CONFIG, 0x00);
    delay(10);
    return true;
}

// ---------------------------------------------------------------------------
// readIMU
// Called at IMU_INTERVAL_MS (100 Hz).
// Reads 14 bytes (accel + temp + gyro) via Wire, applies bias, feeds Madgwick.
// ---------------------------------------------------------------------------
static void readIMU() {
    if (!imuOk) return;

    Wire.beginTransmission(imuAddr);
    Wire.write(MPU_REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return;
    if (Wire.requestFrom((int)imuAddr, 14) < 14) return;

    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read();  // skip temperature
    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    // Convert: ±2g → 16384 LSB/g,  ±250°/s → 131 LSB/(°/s)
    imuAx = rawAx * MPU_ACCEL_SCALE - imuAccelBias[0];
    imuAy = rawAy * MPU_ACCEL_SCALE - imuAccelBias[1];
    imuAz = rawAz * MPU_ACCEL_SCALE - imuAccelBias[2];
    imuGx = rawGx * MPU_GYRO_SCALE  - imuGyroBias[0];
    imuGy = rawGy * MPU_GYRO_SCALE  - imuGyroBias[1];
    imuGz = rawGz * MPU_GYRO_SCALE  - imuGyroBias[2];

    madgwick.updateIMU(imuGx, imuGy, imuGz, imuAx, imuAy, imuAz);
}

// ---------------------------------------------------------------------------
// sendIMU
// RESP_IMU data layout (IMU_DATA_LEN = 40 bytes):
//   [0..11]  float x3  accel X,Y,Z  [g]
//   [12..23] float x3  gyro  X,Y,Z  [dps]
//   [24..27] float     roll   [°]
//   [28..31] float     pitch  [°]
//   [32..35] float     yaw    [°]
//   [36..39] uint32    timestamp (ms)
// ---------------------------------------------------------------------------
static void sendIMU() {
    uint8_t data[IMU_DATA_LEN];
    uint8_t off = 0;

    float ax = imuAx, ay = imuAy, az = imuAz;
    float gx = imuGx, gy = imuGy, gz = imuGz;
    float roll  = madgwick.getRoll();
    float pitch = madgwick.getPitch();
    float yaw   = madgwick.getYaw();

    memcpy(&data[off], &ax, 4);    off += 4;
    memcpy(&data[off], &ay, 4);    off += 4;
    memcpy(&data[off], &az, 4);    off += 4;
    memcpy(&data[off], &gx, 4);    off += 4;
    memcpy(&data[off], &gy, 4);    off += 4;
    memcpy(&data[off], &gz, 4);    off += 4;
    memcpy(&data[off], &roll,  4); off += 4;
    memcpy(&data[off], &pitch, 4); off += 4;
    memcpy(&data[off], &yaw,   4); off += 4;
    uint32_t ts = millis();
    memcpy(&data[off], &ts, 4);    off += 4;

    uint8_t cksum = calcChecksum(RESP_IMU, IMU_DATA_LEN, data);
    Serial.write(PACKET_HEADER);
    Serial.write(RESP_IMU);
    Serial.write(IMU_DATA_LEN);
    Serial.write(data, IMU_DATA_LEN);
    Serial.write(cksum);
}

// ---------------------------------------------------------------------------
// calibrateIMU
// Blocking ~2s. Keep IMU level and perfectly still during calibration.
// Collects 200 samples, computes gyro bias and accel offset (gravity removed).
// ---------------------------------------------------------------------------
static void calibrateIMU() {
    if (!imuOk) return;
    emergencyStop();

    constexpr int N = 200;
    double sumGx = 0, sumGy = 0, sumGz = 0;
    double sumAx = 0, sumAy = 0, sumAz = 0;

    for (int i = 0; i < N; i++) {
        Wire.beginTransmission(imuAddr);
        Wire.write(MPU_REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((int)imuAddr, 14);
        if (Wire.available() >= 14) {
            int16_t ax = (Wire.read() << 8) | Wire.read();
            int16_t ay = (Wire.read() << 8) | Wire.read();
            int16_t az = (Wire.read() << 8) | Wire.read();
            Wire.read(); Wire.read();  // skip temp
            int16_t gx = (Wire.read() << 8) | Wire.read();
            int16_t gy = (Wire.read() << 8) | Wire.read();
            int16_t gz = (Wire.read() << 8) | Wire.read();
            sumAx += ax * MPU_ACCEL_SCALE;
            sumAy += ay * MPU_ACCEL_SCALE;
            sumAz += az * MPU_ACCEL_SCALE;
            sumGx += gx * MPU_GYRO_SCALE;
            sumGy += gy * MPU_GYRO_SCALE;
            sumGz += gz * MPU_GYRO_SCALE;
        }
        delay(10);  // 100 Hz
    }

    imuGyroBias[0] = (float)(sumGx / N);
    imuGyroBias[1] = (float)(sumGy / N);
    imuGyroBias[2] = (float)(sumGz / N);
    // Accel bias: remove gravity from whichever axis is ~1g
    float avgAx = (float)(sumAx / N);
    float avgAy = (float)(sumAy / N);
    float avgAz = (float)(sumAz / N);
    imuAccelBias[0] = avgAx;
    imuAccelBias[1] = avgAy;
    // Z-axis: subtract gravity (1g or -1g depending on orientation)
    imuAccelBias[2] = avgAz - (avgAz > 0 ? 1.0f : -1.0f);

    madgwick.begin(1000.0f / IMU_INTERVAL_MS);  // reset filter
    saveCalibration();
    Serial.println("IMU calibration saved to flash.");
}

// ---------------------------------------------------------------------------
// saveCalibration / loadCalibration
// Persist imuGyroBias and imuAccelBias to RP2040 flash via EEPROM emulation.
// ---------------------------------------------------------------------------
static void saveCalibration() {
    ImuCalData cal;
    cal.magic = IMU_CAL_MAGIC;
    memcpy(cal.gyroBias,  imuGyroBias,  sizeof(imuGyroBias));
    memcpy(cal.accelBias, imuAccelBias, sizeof(imuAccelBias));
    EEPROM.put(IMU_CAL_EEPROM_ADDR, cal);
    EEPROM.commit();
}

static bool loadCalibration() {
    ImuCalData cal;
    EEPROM.get(IMU_CAL_EEPROM_ADDR, cal);
    if (cal.magic != IMU_CAL_MAGIC) {
        Serial.println("IMU: no saved calibration (first use or flash erased).");
        return false;
    }
    memcpy(imuGyroBias,  cal.gyroBias,  sizeof(imuGyroBias));
    memcpy(imuAccelBias, cal.accelBias, sizeof(imuAccelBias));
    Serial.print("IMU cal loaded: gyroBias=[");
    Serial.print(imuGyroBias[0], 4); Serial.print(", ");
    Serial.print(imuGyroBias[1], 4); Serial.print(", ");
    Serial.print(imuGyroBias[2], 4); Serial.println("] dps");
    return true;
}

// ---------------------------------------------------------------------------
// saveWheelCal / loadWheelCal
// ---------------------------------------------------------------------------
static void saveWheelCal() {
    WheelCalData cal;
    cal.magic = WHEEL_CAL_MAGIC;
    cal.scale = wheelCalScale;
    EEPROM.put(WHEEL_CAL_EEPROM_ADDR, cal);
    EEPROM.commit();
}

static bool loadWheelCal() {
    WheelCalData cal;
    EEPROM.get(WHEEL_CAL_EEPROM_ADDR, cal);
    if (cal.magic != WHEEL_CAL_MAGIC) return false;
    if (cal.scale < 0.5f || cal.scale > 2.0f) return false;  // sanity check
    wheelCalScale = cal.scale;
    return true;
}

// ---------------------------------------------------------------------------
// buildI2CReadBuffer
// Called when the controller sets a register address (read preparation).
// Fills i2cReadBuf / i2cReadLen with the data for that register.
// ---------------------------------------------------------------------------
static void buildI2CReadBuffer(uint8_t reg) {
    i2cReadLen = 0;

    if (reg >= REG_ENC0 && reg <= REG_ENC0 + 3) {
        int32_t cnt = wheels[reg - REG_ENC0].getPosition();
        memcpy(i2cReadBuf, &cnt, 4);
        i2cReadLen = 4;

    } else if (reg >= REG_VEL0 && reg <= REG_VEL0 + 3) {
        int32_t vel = wheels[reg - REG_VEL0].getVelocityInt();
        memcpy(i2cReadBuf, &vel, 4);
        i2cReadLen = 4;

    } else if (reg >= REG_ADC0 && reg <= REG_ADC0 + 3) {
        int16_t adc = (int16_t)analogRead(ADC_CURRENT_CH[reg - REG_ADC0]);
        memcpy(i2cReadBuf, &adc, 2);
        i2cReadLen = 2;

    } else if (reg == REG_STATUS) {
        // Full status: enc(int32x4)+vel(int32x4)+adc(int16x4)+timestamp(uint32) = 44 B
        uint8_t off = 0;
        for (int i = 0; i < 4; i++) {
            int32_t cnt = wheels[i].getPosition();
            memcpy(&i2cReadBuf[off], &cnt, 4);
            off += 4;
        }
        for (int i = 0; i < 4; i++) {
            int32_t vel = wheels[i].getVelocityInt();
            memcpy(&i2cReadBuf[off], &vel, 4);
            off += 4;
        }
        for (int i = 0; i < 4; i++) {
            int16_t adc = (int16_t)analogRead(ADC_CURRENT_CH[i]);
            memcpy(&i2cReadBuf[off], &adc, 2);
            off += 2;
        }
        uint32_t ts = millis();
        memcpy(&i2cReadBuf[off], &ts, 4);
        off += 4;
        i2cReadLen = off;  // 44

    } else if (reg >= REG_EXT_ADC0 && reg <= REG_EXT_ADC0 + 7) {
        // Single MCP3208 channel from buffer (int16, 2 bytes)
        memcpy(i2cReadBuf, &extAdcBuf[reg - REG_EXT_ADC0], 2);
        i2cReadLen = 2;

    } else if (reg == REG_EXT_ADC_ALL) {
        // All 8 MCP3208 channels from buffer (int16 x8, 16 bytes)
        for (int i = 0; i < 8; i++) {
            memcpy(&i2cReadBuf[i * 2], &extAdcBuf[i], 2);
        }
        i2cReadLen = 16;

    } else if (reg >= REG_CURR0 && reg <= REG_CURR0 + 3) {
        // Motor current [A] converted from MCP3208 ch0-3 buffer
        float curr = SensorConverter::toCurrentA(extAdcBuf[reg - REG_CURR0]);
        memcpy(i2cReadBuf, &curr, 4);
        i2cReadLen = 4;

    } else if (reg == REG_VBATT) {
        // Battery voltage [V] converted from MCP3208 ch4 buffer
        float vbatt = SensorConverter::toBatteryV(extAdcBuf[4]);
        memcpy(i2cReadBuf, &vbatt, 4);
        i2cReadLen = 4;

    } else if (reg == REG_IMU) {
        // IMU data: float x9 + uint32 = 40 B (same layout as RESP_IMU)
        uint8_t off = 0;
        float vals[9] = {
            imuAx, imuAy, imuAz,
            imuGx, imuGy, imuGz,
            madgwick.getRoll(), madgwick.getPitch(), madgwick.getYaw()
        };
        for (int i = 0; i < 9; i++) {
            memcpy(&i2cReadBuf[off], &vals[i], 4); off += 4;
        }
        uint32_t ts = millis();
        memcpy(&i2cReadBuf[off], &ts, 4); off += 4;
        i2cReadLen = off;  // 40

    } else if (reg == REG_DEVICE_ID) {
        i2cReadBuf[0] = 0x20;
        i2cReadBuf[1] = 0x40;
        i2cReadLen = 2;
    }
}

// ---------------------------------------------------------------------------
// processI2CWrite
// Executes a command received via I2C write.
// data[0] = register address, data[1..] = payload
// ---------------------------------------------------------------------------
static void processI2CWrite(const uint8_t* data, uint8_t len) {
    if (len == 0) return;
    uint8_t reg = data[0];

    switch (reg) {

    case REG_MOTOR_ALL:
        if (len == 9) {
            for (int i = 0; i < 4; i++) {
                int16_t duty;
                memcpy(&duty, &data[1 + i * 2], 2);
                wheels[i].setDuty(duty);
            }
        }
        break;

    case REG_MOTOR_SINGLE:
        if (len == 4) {
            uint8_t idx = data[1];
            if (idx < 4) {
                int16_t duty;
                memcpy(&duty, &data[2], 2);
                wheels[idx].setDuty(duty);
            }
        }
        break;

    case REG_STOP_ALL:
        emergencyStop();
        break;

    case REG_BRAKE_ALL:
        for (int i = 0; i < 4; i++) wheels[i].brake();
        break;

    case REG_RESET_ENC:
        for (int i = 0; i < 4; i++) wheels[i].resetEncoder();
        break;

    case REG_SET_MODE_ALL:
        if (len == 5) {
            for (int i = 0; i < 4; i++) {
                wheels[i].setMode((ControlMode)data[1 + i]);
            }
        }
        break;

    case REG_SET_MODE_SINGLE:
        if (len == 3) {
            uint8_t idx = data[1];
            if (idx < 4) wheels[idx].setMode((ControlMode)data[2]);
        }
        break;

    case REG_SET_VEL_ALL:
        if (len == 17) {
            for (int i = 0; i < 4; i++) {
                float target;
                memcpy(&target, &data[1 + i * 4], 4);
                wheels[i].setVelocityTarget(target);
            }
        }
        break;

    case REG_SET_VEL_SINGLE:
        if (len == 6) {
            uint8_t idx = data[1];
            if (idx < 4) {
                float target;
                memcpy(&target, &data[2], 4);
                wheels[idx].setVelocityTarget(target);
            }
        }
        break;

    case REG_SET_POS_ALL:
        if (len == 17) {
            for (int i = 0; i < 4; i++) {
                int32_t target;
                memcpy(&target, &data[1 + i * 4], 4);
                wheels[i].setPositionTarget(target);
            }
        }
        break;

    case REG_SET_POS_SINGLE:
        if (len == 6) {
            uint8_t idx = data[1];
            if (idx < 4) {
                int32_t target;
                memcpy(&target, &data[2], 4);
                wheels[idx].setPositionTarget(target);
            }
        }
        break;

    case REG_SET_VEL_PID:
        if (len == 14) {
            uint8_t idx = data[1];
            if (idx < 4) {
                float kp, ki, kd;
                memcpy(&kp, &data[2],  4);
                memcpy(&ki, &data[6],  4);
                memcpy(&kd, &data[10], 4);
                wheels[idx].setVelocityPID(kp, ki, kd);
            }
        }
        break;

    case REG_SET_POS_GAINS:
        if (len == 10) {
            uint8_t idx = data[1];
            if (idx < 4) {
                float posKp, maxVelCps;
                memcpy(&posKp,     &data[2], 4);
                memcpy(&maxVelCps, &data[6], 4);
                wheels[idx].setPositionGains(posKp, maxVelCps);
            }
        }
        break;

    default:
        // Read-only register: just prepare the read buffer
        buildI2CReadBuffer(reg);
        break;
    }
}

// ---------------------------------------------------------------------------
// onI2CReceive  (called in ISR context by Wire1)
// Keep this short: buffer the bytes and set a flag for loop() to process.
// ---------------------------------------------------------------------------
static void onI2CReceive(int numBytes) {
    uint8_t len = 0;
    while (Wire1.available() && len < sizeof(i2cWriteBuf)) {
        i2cWriteBuf[len++] = Wire1.read();
    }
    // Drain any excess
    while (Wire1.available()) Wire1.read();

    // Single byte = register-address for a subsequent read
    if (len == 1) {
        buildI2CReadBuffer(i2cWriteBuf[0]);
        return;
    }

    // Write command: defer processing to loop()
    i2cWriteLen     = len;
    i2cWritePending = true;
}

// ---------------------------------------------------------------------------
// onI2CRequest  (called in ISR context by Wire1)
// Send the pre-built read buffer.
// ---------------------------------------------------------------------------
static void onI2CRequest() {
    if (i2cReadLen > 0) {
        Wire1.write(i2cReadBuf, i2cReadLen);
    }
}
