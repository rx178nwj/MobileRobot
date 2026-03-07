#pragma once

// =============================================================================
// PR2040_DC_MOTOR_DRIVER REV1.5.0 - Pin Configuration
// MCU: RP2040
// =============================================================================

// --- Motor Control (TB67H451 H-bridge x4) ------------------------------------
// Direction/PWM via IN1, IN2
constexpr uint8_t MOTOR1_IN1 = 8;
constexpr uint8_t MOTOR1_IN2 = 9;
constexpr uint8_t MOTOR2_IN1 = 10;
constexpr uint8_t MOTOR2_IN2 = 11;
constexpr uint8_t MOTOR3_IN1 = 14;
constexpr uint8_t MOTOR3_IN2 = 15;
constexpr uint8_t MOTOR4_IN1 = 16;
constexpr uint8_t MOTOR4_IN2 = 17;

constexpr uint8_t MOTOR_IN1[4] = { MOTOR1_IN1, MOTOR2_IN1, MOTOR3_IN1, MOTOR4_IN1 };
constexpr uint8_t MOTOR_IN2[4] = { MOTOR1_IN2, MOTOR2_IN2, MOTOR3_IN2, MOTOR4_IN2 };

// --- Encoder Input (Quadrature x4) -------------------------------------------
constexpr uint8_t ENC1_A = 18;
constexpr uint8_t ENC1_B = 19;
constexpr uint8_t ENC2_A = 20;
constexpr uint8_t ENC2_B = 21;
constexpr uint8_t ENC3_A = 22;
constexpr uint8_t ENC3_B = 23;
constexpr uint8_t ENC4_A = 24;
constexpr uint8_t ENC4_B = 25;

constexpr uint8_t ENC_A[4] = { ENC1_A, ENC2_A, ENC3_A, ENC4_A };
constexpr uint8_t ENC_B[4] = { ENC1_B, ENC2_B, ENC3_B, ENC4_B };

// --- Internal ADC (RP2040 ADC0-3) --------------------------------------------
// Connected to INA21x current sense amplifier outputs (via LPF)
constexpr uint8_t ADC_CURRENT_CH[4] = { 26, 27, 28, 29 };  // ADC0-3

// --- I2C1: Controller communication (slave) ----------------------------------
// GPIO2 = SDA, GPIO3 = SCL  (I2C1 peripheral)
constexpr uint8_t I2C1_SDA_PIN    = 2;
constexpr uint8_t I2C1_SCL_PIN    = 3;
constexpr uint8_t I2C_SLAVE_ADDR  = 0x60;  // 7-bit slave address

// I2C Register map
// Write registers (controller writes to this board):
//   REG_MOTOR_ALL      [8 B]  int16 x4  motor duties -1000..+1000 (little-endian)
//   REG_MOTOR_SINGLE   [3 B]  uint8 index(0-3) + int16 duty
//   REG_STOP_ALL       [0 B]  coast all motors
//   REG_BRAKE_ALL      [0 B]  active brake all motors
//   REG_RESET_ENC      [0 B]  reset all encoder counts
//   REG_SET_MODE_ALL   [4 B]  uint8 x4  control mode (0=DIRECT,1=VEL,2=POS)
//   REG_SET_MODE_SINGLE[2 B]  uint8 index + uint8 mode
//   REG_SET_VEL_ALL    [16 B] float x4  velocity targets (counts/sec)
//   REG_SET_VEL_SINGLE [5 B]  uint8 index + float target
//   REG_SET_POS_ALL    [16 B] int32 x4  position targets (counts)
//   REG_SET_POS_SINGLE [5 B]  uint8 index + int32 target
//   REG_SET_VEL_PID    [13 B] uint8 index + float kp, ki, kd
//   REG_SET_POS_GAINS  [9 B]  uint8 index + float posKp, maxVelCps
//
// Read registers (controller reads from this board):
//   REG_ENC0..3      [4 B each]  int32 encoder count (little-endian)
//   REG_VEL0..3      [4 B each]  int32 velocity (counts/sec, little-endian)
//   REG_ADC0..3      [2 B each]  int16 ADC raw 0..4095 (little-endian)
//   REG_STATUS       [44 B]  enc(int32x4)+vel(int32x4)+adc(int16x4)+timestamp(uint32)
//   REG_DEVICE_ID    [2 B]   0x20, 0x40

constexpr uint8_t REG_MOTOR_ALL       = 0x01;
constexpr uint8_t REG_MOTOR_SINGLE    = 0x02;
constexpr uint8_t REG_STOP_ALL        = 0x03;
constexpr uint8_t REG_BRAKE_ALL       = 0x04;
constexpr uint8_t REG_RESET_ENC       = 0x05;
constexpr uint8_t REG_SET_MODE_ALL    = 0x06;
constexpr uint8_t REG_SET_MODE_SINGLE = 0x07;
constexpr uint8_t REG_SET_VEL_ALL     = 0x08;
constexpr uint8_t REG_SET_VEL_SINGLE  = 0x09;
constexpr uint8_t REG_SET_POS_ALL     = 0x0A;
constexpr uint8_t REG_SET_POS_SINGLE  = 0x0B;
constexpr uint8_t REG_SET_VEL_PID     = 0x0C;
constexpr uint8_t REG_SET_POS_GAINS   = 0x0D;
constexpr uint8_t REG_ENC0            = 0x10;  // 0x10-0x13
constexpr uint8_t REG_VEL0            = 0x14;  // 0x14-0x17
constexpr uint8_t REG_ADC0            = 0x20;  // 0x20-0x23
constexpr uint8_t REG_STATUS          = 0x30;
constexpr uint8_t REG_DEVICE_ID       = 0xFF;

// --- IMU: MPU-6881 (I2C0) ----------------------------------------------------
constexpr uint8_t IMU_INT_PIN  = 0;
constexpr uint8_t I2C0_SDA_PIN = 12;
constexpr uint8_t I2C0_SCL_PIN = 13;
constexpr uint8_t MPU6881_I2C_ADDR = 0x68;

// --- External SPI ADC: MCP3208/MAX11125 (SPI0) -------------------------------
constexpr uint8_t SPI0_RX_PIN  = 4;   // MISO
constexpr uint8_t SPI0_CSN_PIN = 5;   // Chip Select
constexpr uint8_t SPI0_SCK_PIN = 6;   // Clock
constexpr uint8_t SPI0_TX_PIN  = 7;   // MOSI

// --- PWM Settings ------------------------------------------------------------
constexpr uint32_t PWM_FREQ_HZ = 20000;   // 20 kHz
constexpr int32_t  PWM_DUTY_MAX = 1000;   // duty: -1000 to +1000

// --- Control Loop ------------------------------------------------------------
constexpr uint32_t CONTROL_INTERVAL_MS = 5;  // 200 Hz control loop

// --- Serial Communication ---------------------------------------------------
constexpr uint32_t SERIAL_BAUD = 115200;

// Status packet transmission interval (ms)
constexpr uint32_t STATUS_INTERVAL_MS = 10;  // 100 Hz

// --- Binary Protocol ---------------------------------------------------------
// Packet format: [HEADER][CMD][LEN][DATA x LEN][CHECKSUM]
// CHECKSUM = XOR(CMD, LEN, DATA[0..LEN-1])

constexpr uint8_t PACKET_HEADER = 0xAA;
constexpr uint8_t MAX_PACKET_DATA = 64;

// Commands (Host -> RP2040)
constexpr uint8_t CMD_SET_MOTORS       = 0x01; // data: int16 x4 (duty -1000..+1000)
constexpr uint8_t CMD_STOP_ALL         = 0x02; // data: none  (coast)
constexpr uint8_t CMD_BRAKE_ALL        = 0x03; // data: none  (active brake)
constexpr uint8_t CMD_SET_MOTOR_SINGLE = 0x04; // data: uint8 index, int16 duty
constexpr uint8_t CMD_REQUEST_STATUS   = 0x10; // data: none
constexpr uint8_t CMD_RESET_ENCODERS   = 0x11; // data: none
// Wheel controller commands
constexpr uint8_t CMD_SET_MODE_ALL     = 0x20; // data: uint8 x4  (0=DIRECT,1=VEL,2=POS)
constexpr uint8_t CMD_SET_MODE_SINGLE  = 0x21; // data: uint8 index, uint8 mode
constexpr uint8_t CMD_SET_VEL_ALL      = 0x22; // data: float x4  velocity targets (cps)
constexpr uint8_t CMD_SET_VEL_SINGLE   = 0x23; // data: uint8 index, float target
constexpr uint8_t CMD_SET_POS_ALL      = 0x24; // data: int32 x4  position targets (counts)
constexpr uint8_t CMD_SET_POS_SINGLE   = 0x25; // data: uint8 index, int32 target
constexpr uint8_t CMD_SET_VEL_PID      = 0x26; // data: uint8 index, float kp, ki, kd
constexpr uint8_t CMD_SET_POS_GAINS    = 0x27; // data: uint8 index, float posKp, maxVelCps

// Responses (RP2040 -> Host)
constexpr uint8_t RESP_ACK    = 0x80; // data: echo cmd byte
constexpr uint8_t RESP_NAK    = 0x81; // data: echo cmd byte
constexpr uint8_t RESP_STATUS = 0x91; // see sendStatus() for layout

// Status packet data length (44 bytes):
//   [0..15]  int32 x4  encoder counts
//   [16..31] int32 x4  velocities (counts/sec, truncated to int)
//   [32..39] int16 x4  ADC raw 0..4095
//   [40..43] uint32    timestamp (millis)
constexpr uint8_t STATUS_DATA_LEN = 44;
