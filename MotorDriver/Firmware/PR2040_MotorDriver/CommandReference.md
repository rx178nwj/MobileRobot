# PR2040_MotorDriver コマンドリファレンス

**ハードウェア**: PR2040_BASESYSTEM REV1.5.0
**対応インターフェース**: USB Serial (CDC) / I2C スレーブ

---

## 共通事項

### 制御モード

| 値 | モード | 説明 |
|---|---|---|
| 0 | DIRECT | デューティ直接指定（-1000〜+1000）。フィードバックなし |
| 1 | VELOCITY | 速度PID制御。目標値: counts/sec (float) |
| 2 | POSITION | 位置カスケード制御。目標値: エンコーダカウント (int32) |

### ホイールインデックス

| index | 対象 |
|---|---|
| 0 | Motor1 / Encoder1 |
| 1 | Motor2 / Encoder2 |
| 2 | Motor3 / Encoder3 |
| 3 | Motor4 / Encoder4 |

### 数値フォーマット

- すべての多バイト値: **リトルエンディアン**
- `int16` : 2 bytes, signed
- `int32` : 4 bytes, signed
- `uint8` : 1 byte, unsigned
- `float` : 4 bytes, IEEE 754 single precision

---

## USB Serial コマンド

### パケットフォーマット

```
送信 (Host → Board):
  [0xAA] [CMD] [LEN] [DATA × LEN] [CHECKSUM]

受信 (Board → Host):
  [0xAA] [TYPE] [LEN] [DATA × LEN] [CHECKSUM]

CHECKSUM = XOR(CMD/TYPE, LEN, DATA[0], ..., DATA[LEN-1])
ボーレート: 115200 bps (USB CDC)
```

---

### コマンド一覧 (Host → Board)

#### モータ直接制御

| CMD | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| CMD_SET_MOTORS | `0x01` | 8 | `int16[4]` duty[0..3] | 全モータ デューティ設定 |
| CMD_SET_MOTOR_SINGLE | `0x04` | 3 | `uint8` index + `int16` duty | 1軸 デューティ設定 |
| CMD_STOP_ALL | `0x02` | 0 | なし | 全モータ コースト停止 + DIRECT モードへ |
| CMD_BRAKE_ALL | `0x03` | 0 | なし | 全モータ アクティブブレーキ + DIRECT モードへ |

duty 範囲: -1000 (全力逆転) 〜 +1000 (全力正転)、0 = コースト

#### 状態取得 / エンコーダ

| CMD | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| CMD_REQUEST_STATUS | `0x10` | 0 | なし | ステータスパケット1回送信 → `RESP_STATUS` |
| CMD_RESET_ENCODERS | `0x11` | 0 | なし | 全エンコーダカウントリセット |
| CMD_REQUEST_EXT_ADC | `0x12` | 0 | なし | MCP3208 全チャンネル読み出し → `RESP_EXT_ADC` |

#### 制御モード設定

| CMD | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| CMD_SET_MODE_ALL | `0x20` | 4 | `uint8[4]` mode[0..3] | 全軸 制御モード設定 |
| CMD_SET_MODE_SINGLE | `0x21` | 2 | `uint8` index + `uint8` mode | 1軸 制御モード設定 |

#### 速度制御 (VELOCITY モード)

| CMD | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| CMD_SET_VEL_ALL | `0x22` | 16 | `float[4]` target[0..3] | 全軸 速度目標設定 [counts/sec] |
| CMD_SET_VEL_SINGLE | `0x23` | 5 | `uint8` index + `float` target | 1軸 速度目標設定 [counts/sec] |

#### 位置制御 (POSITION モード)

| CMD | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| CMD_SET_POS_ALL | `0x24` | 16 | `int32[4]` target[0..3] | 全軸 位置目標設定 [counts] |
| CMD_SET_POS_SINGLE | `0x25` | 5 | `uint8` index + `int32` target | 1軸 位置目標設定 [counts] |

#### PID / ゲイン設定

| CMD | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| CMD_SET_VEL_PID | `0x26` | 13 | `uint8` index + `float` kp + `float` ki + `float` kd | 速度PIDゲイン設定 |
| CMD_SET_POS_GAINS | `0x27` | 9 | `uint8` index + `float` posKp + `float` maxVelCps | 位置制御ゲイン設定 |

デフォルト速度PID: Kp=1.0, Ki=0.05, Kd=0.0
デフォルト位置ゲイン: posKp=0.5, maxVelCps=2000

---

### レスポンス一覧 (Board → Host)

| TYPE | 値 | LEN | データ構造 | 説明 |
|---|---|---|---|---|
| RESP_ACK | `0x80` | 1 | `uint8` echo_cmd | コマンド受理 |
| RESP_NAK | `0x81` | 1 | `uint8` echo_cmd | コマンドエラー（長さ不正・インデックス範囲外など） |
| RESP_STATUS | `0x91` | 44 | 下記参照 | ステータスパケット（100 Hz 自動送信） |
| RESP_EXT_ADC | `0x92` | 16 | `int16[8]` ch0..7 | MCP3208 全チャンネル raw値 (0..4095) |

#### RESP_STATUS レイアウト (44 bytes)

| オフセット | 型 | 内容 |
|---|---|---|
| 0〜15 | `int32[4]` | エンコーダカウント [counts] |
| 16〜31 | `int32[4]` | 速度 [counts/sec] |
| 32〜39 | `int16[4]` | 内部ADC raw (電流センス) 0..4095 |
| 40〜43 | `uint32` | タイムスタンプ [ms] (millis()) |

> ステータスは `CMD_REQUEST_STATUS` によるオンデマンド送信に加え、**100 Hz (10ms間隔) で自動送信**されます。

---

### USB Serial 使用例 (Python)

```python
import struct
import serial

ser = serial.Serial('COM3', 115200, timeout=1)

def send_cmd(cmd, data=b''):
    length = len(data)
    cs = cmd ^ length
    for b in data:
        cs ^= b
    packet = bytes([0xAA, cmd, length]) + data + bytes([cs])
    ser.write(packet)

# ── モータ直接制御 ──────────────────────────────────────────

# 全モータを duty=500 に設定（正転 50%）
send_cmd(0x01, struct.pack('<hhhh', 500, 500, 500, 500))

# Motor0 のみ duty=-300（逆転 30%）
send_cmd(0x04, bytes([0]) + struct.pack('<h', -300))

# 全停止（コースト）
send_cmd(0x02)

# 全ブレーキ
send_cmd(0x03)

# ── 速度制御 ────────────────────────────────────────────────

# 全軸を速度制御モードに設定
send_cmd(0x20, bytes([1, 1, 1, 1]))

# 全軸の速度目標を 500 counts/sec に設定
send_cmd(0x22, struct.pack('<ffff', 500.0, 500.0, 500.0, 500.0))

# Wheel1 の速度目標を -200 counts/sec に設定
send_cmd(0x23, bytes([1]) + struct.pack('<f', -200.0))

# ── 位置制御 ────────────────────────────────────────────────

# 全軸を位置制御モードに設定
send_cmd(0x20, bytes([2, 2, 2, 2]))

# 全軸の位置目標を 1000 counts に設定
send_cmd(0x24, struct.pack('<iiii', 1000, 1000, 1000, 1000))

# Wheel0 の位置目標を 2000 counts に設定
send_cmd(0x25, bytes([0]) + struct.pack('<i', 2000))

# ── PID チューニング ────────────────────────────────────────

# Wheel0 の速度PIDゲインを設定（Kp=2.0, Ki=0.1, Kd=0.0）
send_cmd(0x26, bytes([0]) + struct.pack('<fff', 2.0, 0.1, 0.0))

# Wheel0 の位置ゲインを設定（posKp=0.8, maxVelCps=1500）
send_cmd(0x27, bytes([0]) + struct.pack('<ff', 0.8, 1500.0))

# ── エンコーダ / ステータス ─────────────────────────────────

# エンコーダリセット
send_cmd(0x11)

# ステータス1回取得（RESP_STATUS が返る）
send_cmd(0x10)

# MCP3208 外部ADC 読み出し（RESP_EXT_ADC が返る）
send_cmd(0x12)

# ── ステータス受信 ───────────────────────────────────────────

def read_status(raw):
    """RESP_STATUS の 44 bytes を解析する"""
    enc  = struct.unpack_from('<iiii', raw, 0)
    vel  = struct.unpack_from('<iiii', raw, 16)
    adc  = struct.unpack_from('<hhhh', raw, 32)
    ts   = struct.unpack_from('<I',    raw, 40)[0]
    return {'enc': enc, 'vel': vel, 'adc': adc, 'ts_ms': ts}
```

---

## I2C コマンド

**スレーブアドレス**: `0x60` (7bit)
**ピン**: SDA=GPIO2 / SCL=GPIO3

### I2C 操作方式

```
書き込み (コントローラ → ボード):
  START → [0x60 W] → [REG_ADDR] → [DATA bytes...] → STOP

読み出し (コントローラ ← ボード):
  START → [0x60 W] → [REG_ADDR] → STOP
  START → [0x60 R] → [DATA bytes...] → STOP
```

---

### 書き込みレジスタ一覧

#### モータ直接制御

| レジスタ | アドレス | ペイロード長 | データ構造 | 説明 |
|---|---|---|---|---|
| REG_MOTOR_ALL | `0x01` | 8 | `int16[4]` duty[0..3] | 全モータ デューティ設定 |
| REG_MOTOR_SINGLE | `0x02` | 3 | `uint8` index + `int16` duty | 1軸 デューティ設定 |
| REG_STOP_ALL | `0x03` | 0 | なし | 全モータ コースト停止 |
| REG_BRAKE_ALL | `0x04` | 0 | なし | 全モータ アクティブブレーキ |
| REG_RESET_ENC | `0x05` | 0 | なし | 全エンコーダカウントリセット |

#### 制御モード設定

| レジスタ | アドレス | ペイロード長 | データ構造 | 説明 |
|---|---|---|---|---|
| REG_SET_MODE_ALL | `0x06` | 4 | `uint8[4]` mode[0..3] | 全軸 制御モード設定 |
| REG_SET_MODE_SINGLE | `0x07` | 2 | `uint8` index + `uint8` mode | 1軸 制御モード設定 |

#### 速度制御 (VELOCITY モード)

| レジスタ | アドレス | ペイロード長 | データ構造 | 説明 |
|---|---|---|---|---|
| REG_SET_VEL_ALL | `0x08` | 16 | `float[4]` target[0..3] | 全軸 速度目標 [counts/sec] |
| REG_SET_VEL_SINGLE | `0x09` | 5 | `uint8` index + `float` target | 1軸 速度目標 [counts/sec] |

#### 位置制御 (POSITION モード)

| レジスタ | アドレス | ペイロード長 | データ構造 | 説明 |
|---|---|---|---|---|
| REG_SET_POS_ALL | `0x0A` | 16 | `int32[4]` target[0..3] | 全軸 位置目標 [counts] |
| REG_SET_POS_SINGLE | `0x0B` | 5 | `uint8` index + `int32` target | 1軸 位置目標 [counts] |

#### PID / ゲイン設定

| レジスタ | アドレス | ペイロード長 | データ構造 | 説明 |
|---|---|---|---|---|
| REG_SET_VEL_PID | `0x0C` | 13 | `uint8` index + `float` kp + `float` ki + `float` kd | 速度PIDゲイン設定 |
| REG_SET_POS_GAINS | `0x0D` | 9 | `uint8` index + `float` posKp + `float` maxVelCps | 位置制御ゲイン設定 |

---

### 読み出しレジスタ一覧

#### エンコーダ / 速度

| レジスタ | アドレス | 返却長 | 型 | 内容 |
|---|---|---|---|---|
| REG_ENC0 | `0x10` | 4 | `int32` | Encoder0 カウント |
| REG_ENC1 | `0x11` | 4 | `int32` | Encoder1 カウント |
| REG_ENC2 | `0x12` | 4 | `int32` | Encoder2 カウント |
| REG_ENC3 | `0x13` | 4 | `int32` | Encoder3 カウント |
| REG_VEL0 | `0x14` | 4 | `int32` | Wheel0 速度 [counts/sec] |
| REG_VEL1 | `0x15` | 4 | `int32` | Wheel1 速度 [counts/sec] |
| REG_VEL2 | `0x16` | 4 | `int32` | Wheel2 速度 [counts/sec] |
| REG_VEL3 | `0x17` | 4 | `int32` | Wheel3 速度 [counts/sec] |

#### 内部ADC (電流センス)

| レジスタ | アドレス | 返却長 | 型 | 内容 |
|---|---|---|---|---|
| REG_ADC0 | `0x20` | 2 | `int16` | Motor1 電流センス raw (0..4095) |
| REG_ADC1 | `0x21` | 2 | `int16` | Motor2 電流センス raw |
| REG_ADC2 | `0x22` | 2 | `int16` | Motor3 電流センス raw |
| REG_ADC3 | `0x23` | 2 | `int16` | Motor4 電流センス raw |

#### 全ステータス

| レジスタ | アドレス | 返却長 | 内容 |
|---|---|---|---|
| REG_STATUS | `0x30` | 44 | enc(int32×4) + vel(int32×4) + adc(int16×4) + ts(uint32) |

#### 外部ADC (MCP3208)

| レジスタ | アドレス | 返却長 | 型 | 内容 |
|---|---|---|---|---|
| REG_EXT_ADC0 | `0x40` | 2 | `int16` | MCP3208 ch0 raw (0..4095) |
| REG_EXT_ADC1 | `0x41` | 2 | `int16` | MCP3208 ch1 raw |
| REG_EXT_ADC2 | `0x42` | 2 | `int16` | MCP3208 ch2 raw |
| REG_EXT_ADC3 | `0x43` | 2 | `int16` | MCP3208 ch3 raw |
| REG_EXT_ADC4 | `0x44` | 2 | `int16` | MCP3208 ch4 raw |
| REG_EXT_ADC5 | `0x45` | 2 | `int16` | MCP3208 ch5 raw |
| REG_EXT_ADC6 | `0x46` | 2 | `int16` | MCP3208 ch6 raw |
| REG_EXT_ADC7 | `0x47` | 2 | `int16` | MCP3208 ch7 raw |
| REG_EXT_ADC_ALL | `0x48` | 16 | `int16[8]` | MCP3208 全チャンネル一括 |

#### 物理値 (変換済み)

| レジスタ | アドレス | 返却長 | 型 | 内容 |
|---|---|---|---|---|
| REG_CURR0 | `0x50` | 4 | `float` | Motor1 電流 [A]（INA213変換済み） |
| REG_CURR1 | `0x51` | 4 | `float` | Motor2 電流 [A] |
| REG_CURR2 | `0x52` | 4 | `float` | Motor3 電流 [A] |
| REG_CURR3 | `0x53` | 4 | `float` | Motor4 電流 [A] |
| REG_VBATT | `0x54` | 4 | `float` | バッテリ電圧 [V]（分圧×11変換済み） |

#### デバイス識別

| レジスタ | アドレス | 返却長 | 内容 |
|---|---|---|---|
| REG_DEVICE_ID | `0xFF` | 2 | `0x20 0x40` |

---

### I2C 使用例 (Python / smbus2)

```python
import struct
from smbus2 import SMBus, i2c_msg

ADDR = 0x60

with SMBus(1) as bus:

    # ── モータ直接制御 ──────────────────────────────────────

    # 全モータ duty=500 設定
    data = list(struct.pack('<hhhh', 500, 500, 500, 500))
    bus.write_i2c_block_data(ADDR, 0x01, data)

    # Motor0 のみ duty=-300
    data = list(struct.pack('<Bh', 0, -300))
    bus.write_i2c_block_data(ADDR, 0x02, data)

    # 全停止
    bus.write_byte(ADDR, 0x03)

    # ── 速度制御 ────────────────────────────────────────────

    # 全軸を VELOCITY モードに設定
    bus.write_i2c_block_data(ADDR, 0x06, [1, 1, 1, 1])

    # 全軸の速度目標 500.0 counts/sec
    data = list(struct.pack('<ffff', 500.0, 500.0, 500.0, 500.0))
    bus.write_i2c_block_data(ADDR, 0x08, data)

    # Wheel1 の速度目標 -200.0 counts/sec
    data = list(struct.pack('<Bf', 1, -200.0))
    bus.write_i2c_block_data(ADDR, 0x09, data)

    # ── 位置制御 ────────────────────────────────────────────

    # 全軸を POSITION モードに設定
    bus.write_i2c_block_data(ADDR, 0x06, [2, 2, 2, 2])

    # 全軸の位置目標 1000 counts
    data = list(struct.pack('<iiii', 1000, 1000, 1000, 1000))
    bus.write_i2c_block_data(ADDR, 0x0A, data)

    # Wheel0 の位置目標 2000 counts
    data = list(struct.pack('<Bi', 0, 2000))
    bus.write_i2c_block_data(ADDR, 0x0B, data)

    # ── 読み出し ────────────────────────────────────────────

    # Encoder0 カウント読み出し
    raw = bus.read_i2c_block_data(ADDR, 0x10, 4)
    enc0 = struct.unpack('<i', bytes(raw))[0]
    print(f'Encoder0: {enc0} counts')

    # Wheel0 速度読み出し
    raw = bus.read_i2c_block_data(ADDR, 0x14, 4)
    vel0 = struct.unpack('<i', bytes(raw))[0]
    print(f'Wheel0 velocity: {vel0} counts/sec')

    # 全ステータス読み出し (44 bytes)
    raw = bus.read_i2c_block_data(ADDR, 0x30, 44)
    enc  = struct.unpack_from('<iiii', bytes(raw), 0)
    vel  = struct.unpack_from('<iiii', bytes(raw), 16)
    adc  = struct.unpack_from('<hhhh', bytes(raw), 32)
    ts   = struct.unpack_from('<I',    bytes(raw), 40)[0]
    print(f'enc={enc}, vel={vel}, adc={adc}, ts={ts}ms')

    # Motor1 電流 [A] 読み出し
    raw = bus.read_i2c_block_data(ADDR, 0x50, 4)
    curr0 = struct.unpack('<f', bytes(raw))[0]
    print(f'Motor1 current: {curr0:.3f} A')

    # バッテリ電圧 [V] 読み出し
    raw = bus.read_i2c_block_data(ADDR, 0x54, 4)
    vbatt = struct.unpack('<f', bytes(raw))[0]
    print(f'Battery: {vbatt:.2f} V')

    # デバイスID確認
    raw = bus.read_i2c_block_data(ADDR, 0xFF, 2)
    print(f'Device ID: {raw[0]:#04x} {raw[1]:#04x}')  # 0x20 0x40
```

---

## 物理値変換パラメータ

| パラメータ | 値 | 説明 |
|---|---|---|
| ホイール外径 | 67.5 mm | |
| ホイール円周 | ≈ 212.06 mm | π × 67.5 |
| エンコーダ PPR | 11 pulse/rev | モータ軸、1チャンネル |
| ギア比 | 18.8 | JGA25-370 620RPM@12V |
| エンコーダ CPR | ≈ 827.2 counts/rev | 出力軸（4倍デコード後） |
| counts/sec → mm/sec | × 0.2564 | `CPS_TO_MMPS` |
| counts → mm | × 0.2564 | `COUNTS_TO_MM` |

**速度換算例**:
- 1000 counts/sec ≈ 256 mm/sec ≈ 0.256 m/sec
- 1 m/sec ≈ 3900 counts/sec

**キャリブレーション**: ホイールを正確に1回転させてエンコーダカウントを実測し、
`Config.h` の `ENCODER_GEAR_RATIO = 実測カウント / (11 × 4)` を調整してください。
