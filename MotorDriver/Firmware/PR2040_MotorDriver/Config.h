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

// --- Status LEDs -------------------------------------------------------------
constexpr uint8_t LED_RED_PIN    = 29;  // RED    LED (GPIO29)
constexpr uint8_t LED_GREEN_PIN  = 28;  // GREEN  LED (GPIO28)
constexpr uint8_t LED_YELLOW_PIN = 27;  // YELLOW LED (GPIO27) — heartbeat

// --- Internal ADC (RP2040 ADC0-3) --------------------------------------------
// Connected to INA21x current sense amplifier outputs (via LPF)
// Note: GPIO27-29 are shared with LEDs above; ADC reads will be affected
// while LEDs are driven HIGH.
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

// MPU-6881 internal register addresses
constexpr uint8_t MPU_REG_SMPLRT_DIV   = 0x19; // Sample rate divider
constexpr uint8_t MPU_REG_CONFIG       = 0x1A; // DLPF config
constexpr uint8_t MPU_REG_GYRO_CONFIG  = 0x1B; // Gyro full-scale
constexpr uint8_t MPU_REG_ACCEL_CONFIG = 0x1C; // Accel full-scale
constexpr uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B; // Accel X high byte (14 bytes: accel+temp+gyro)
constexpr uint8_t MPU_REG_PWR_MGMT_1   = 0x6B; // Power management
constexpr uint8_t MPU_REG_WHO_AM_I     = 0x75; // Device ID (should return 0x70)

// MPU-6881 sensitivity (at default full-scale)
//   Accel: ±2g   → 16384 LSB/g
//   Gyro:  ±250° → 131.0 LSB/(°/s)
//   Temp:  T[°C] = raw / 340.0 + 36.53
constexpr float MPU_ACCEL_SCALE = 1.0f / 16384.0f; // [g/count]
constexpr float MPU_GYRO_SCALE  = 1.0f / 131.0f;   // [dps/count]

// IMU sampling interval
constexpr uint32_t IMU_INTERVAL_MS = 10; // 100 Hz

// --- External SPI ADC: MCP3208/MAX11125 (SPI0) -------------------------------
constexpr uint8_t SPI0_RX_PIN  = 4;   // MISO
constexpr uint8_t SPI0_CSN_PIN = 5;   // Chip Select
constexpr uint8_t SPI0_SCK_PIN = 6;   // Clock
constexpr uint8_t SPI0_TX_PIN  = 7;   // MOSI

// --- Differential Drive (2-wheel) Configuration ------------------------------
// Motor index for each side (0-3 = M1-M4).
// DIR: +1.0 = positive duty/CPS moves wheel forward,
//      -1.0 = inverted (mirror-mounted motor)
constexpr uint8_t BODY_LEFT_IDX  = 0;     // M1 = left wheel
constexpr uint8_t BODY_RIGHT_IDX = 1;     // M2 = right wheel
constexpr float   BODY_LEFT_DIR  = -1.0f; // flip sign if left wheel is reversed
constexpr float   BODY_RIGHT_DIR =  1.0f; // flip sign if right wheel is reversed
// Center-to-center distance between left and right wheel contact patches [mm]
constexpr float   WHEEL_BASE_MM  = 191.5f;

// Body velocity USB Serial commands
constexpr uint8_t CMD_SET_BODY_VEL    = 0x30; // float linear[mm/s] + float omega[deg/s] (8 B)
constexpr uint8_t CMD_BODY_STOP       = 0x31; // no data — coast both drive wheels
constexpr uint8_t CMD_SET_WHEEL_SCALE = 0x33; // float scale (4 B) — save wheel cal to flash

// --- Wheel Physical Parameters -----------------------------------------------
// Wheel outer diameter [mm]
constexpr float WHEEL_DIAMETER_MM      = 67.5f;
constexpr float WHEEL_CIRCUMFERENCE_MM = 3.14159265f * WHEEL_DIAMETER_MM;  // ≈ 212.06 mm

// --- Encoder Physical Parameters ---------------------------------------------
// Motor: JGA25-370 DC geared motor, 12V, 620 RPM (no-load)
// Encoder: 11 PPR (pulses per revolution) on the motor shaft, quadrature (A+B)
// 4x quadrature decode → 44 counts per motor shaft revolution
// Gear ratio ≈ 18.8 (motor ~11600 RPM / 620 RPM output)
// Effective counts per wheel revolution = 11 × 4 × 18.8 ≈ 827.2
//
// Calibration: rotate wheel exactly 1 turn, read encoder count.
// Set ENCODER_GEAR_RATIO = measured_count / (ENCODER_PPR * 4).
constexpr float ENCODER_PPR        = 11.0f;   // pulses/motor-rev (1 channel)
constexpr float ENCODER_GEAR_RATIO = 54.78f;  // gearbox reduction ratio (measured via wheelcal: 51.45/0.939244)
constexpr float ENCODER_CPR        = ENCODER_PPR * 4.0f * ENCODER_GEAR_RATIO;  // ≈ 2410.3

// Conversion factors
constexpr float CPS_TO_MMPS  = WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR;  // (counts/sec) → (mm/sec)
constexpr float COUNTS_TO_MM = WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR;  // counts → mm

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
constexpr uint8_t CMD_REQUEST_EXT_ADC  = 0x12; // data: none → RESP_EXT_ADC (16 B)
constexpr uint8_t CMD_REQUEST_IMU      = 0x13; // data: none → RESP_IMU (40 B)
constexpr uint8_t CMD_CALIBRATE_IMU    = 0x14; // data: none → blocks ~5s, then ACK
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
constexpr uint8_t RESP_ACK     = 0x80; // data: echo cmd byte
constexpr uint8_t RESP_NAK     = 0x81; // data: echo cmd byte
constexpr uint8_t RESP_STATUS  = 0x91; // see sendStatus() for layout
constexpr uint8_t RESP_EXT_ADC = 0x92; // data: int16 x8 (16 B) MCP3208 ch0..7
constexpr uint8_t RESP_IMU     = 0x93; // data: float x9 + uint32 (40 B)

// IMU packet data length (40 bytes):
//   [0..11]  float x3  accel X,Y,Z  [g]   (calibrated, FastIMU)
//   [12..23] float x3  gyro  X,Y,Z  [dps] (calibrated, FastIMU)
//   [24..27] float     roll   [°]   (Madgwick filter output)
//   [28..31] float     pitch  [°]   (Madgwick filter output)
//   [32..35] float     yaw    [°]   (Madgwick filter output)
//   [36..39] uint32    timestamp (ms)
constexpr uint8_t IMU_DATA_LEN = 40;

// I2C register for IMU data
constexpr uint8_t REG_IMU = 0x61;  // 18 B: same layout as RESP_IMU

// Status packet data length (44 bytes):
//   [0..15]  int32 x4  encoder counts
//   [16..31] int32 x4  velocities (counts/sec, truncated to int)
//   [32..39] int16 x4  internal ADC raw 0..4095 (current sense)
//   [40..43] uint32    timestamp (millis)
constexpr uint8_t STATUS_DATA_LEN = 44;

// External ADC (MCP3208) — I2C read registers (raw)
//   REG_EXT_ADC0..7  [2 B each]  int16 MCP3208 ch0..7 (0..4095, little-endian)
//   REG_EXT_ADC_ALL  [16 B]      int16 x8 all channels
constexpr uint8_t REG_EXT_ADC0    = 0x40;  // 0x40-0x47
constexpr uint8_t REG_EXT_ADC_ALL = 0x48;  // all 8 channels at once (16 B)

// Physical value registers — I2C read (converted from MCP3208)
//   REG_CURR0..3  [4 B each]  float motor current [A] (little-endian)
//   REG_VBATT     [4 B]       float battery voltage [V] (little-endian)
constexpr uint8_t REG_CURR0  = 0x50;  // 0x50-0x53  float [A]
constexpr uint8_t REG_VBATT  = 0x54;  // float [V]
