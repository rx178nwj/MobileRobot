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
 *   0x80  RESP_ACK    data: [cmd]
 *   0x81  RESP_NAK    data: [cmd]
 *   0x91  RESP_STATUS [44 B] enc(int32x4)+vel(int32x4)+adc(int16x4)+ts(uint32)
 *
 * Status is also sent automatically at STATUS_INTERVAL_MS (10 ms = 100 Hz).
 */

#include <Wire.h>
#include "Config.h"
#include "WheelController.h"

// ---------------------------------------------------------------------------
// Instances
// ---------------------------------------------------------------------------
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
// Timing
// ---------------------------------------------------------------------------
static uint32_t lastStatusMs  = 0;
static uint32_t lastControlMs = 0;

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
static void     processPacket(uint8_t cmd, const uint8_t* data, uint8_t len);
static void     sendStatus();
static void     sendAck(uint8_t cmd);
static void     sendNak(uint8_t cmd);
static uint8_t  calcChecksum(uint8_t cmd, uint8_t len, const uint8_t* data);
static void     emergencyStop();
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

    // Internal ADC resolution: 12 bit
    analogReadResolution(12);

    // IMU interrupt pin
    pinMode(IMU_INT_PIN, INPUT);

    // I2C0 for MPU-6881 (master)
    Wire.setSDA(I2C0_SDA_PIN);
    Wire.setSCL(I2C0_SCL_PIN);
    Wire.begin();

    // Brief IMU initialization (wake MPU-6881 from sleep)
    Wire.beginTransmission(MPU6881_I2C_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1
    Wire.write(0x01);  // CLKSEL = PLL with X gyro
    Wire.endTransmission();

    // I2C1 for controller communication (slave, GPIO2=SDA, GPIO3=SCL)
    Wire1.setSDA(I2C1_SDA_PIN);
    Wire1.setSCL(I2C1_SCL_PIN);
    Wire1.begin(I2C_SLAVE_ADDR);
    Wire1.onReceive(onI2CReceive);
    Wire1.onRequest(onI2CRequest);

    uint32_t now = millis();
    lastControlMs = now;
    lastStatusMs  = now;

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
