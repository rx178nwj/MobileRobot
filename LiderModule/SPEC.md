# 3D LiDAR Scanner System — 要件定義・実装仕様書

## 1. プロジェクト概要

### 1.1 目的

2D LiDAR（YDLIDAR T-mini Pro）をサーボモーター（FEETECH STS3215）で上下に傾斜（チルト）させることで、低コストに3Dポイントクラウドを取得するシステムを構築する。

### 1.2 システム構成

```
┌─────────────────────────────────────────────────────────┐
│  Raspberry Pi (Host)                                     │
│  - 3Dポイントクラウド再構成                                │
│  - リアルタイム可視化 (Open3D / matplotlib)                │
│  - PLY/CSV ファイル保存                                   │
│  - スキャン制御コマンド送信                                │
└────────────────────┬────────────────────────────────────┘
                     │ USB CDC (/dev/ttyACM0)
                     │ Binary protocol
┌────────────────────┴────────────────────────────────────┐
│  Seeed バスサーボドライバーボード for XIAO                  │
│  ┌──────────────────────────────┐                        │
│  │  XIAO ESP32-C3 (差し込み装着)  │                        │
│  │  - LiDARデータ受信・解析       │                        │
│  │  - IMUデータ取得・フィルタ      │                        │
│  │  - サーボ制御                  │                        │
│  │  - 3Dスキャン状態機械          │                        │
│  │  - USB CDCバイナリ通信         │                        │
│  └──────────────────────────────┘                        │
│  DC IN (7.4V) → サーボ電源                                │
│  サーボコネクタ → STS3215                                  │
└─────────────────────────────────────────────────────────┘
        │ UART1         │ I2C
        ▼               ▼
  YDLIDAR T-mini Pro  MPU6050 IMU
  (5V別電源)          (3.3V XIAO給電)
```

### 1.3 動作概要

1. RPi から3Dスキャン開始コマンドを送信
2. ESP32-C3 がサーボを -45° → +45° まで段階的にチルト
3. 各チルト角度で LiDAR の360°スキャン1回転分を取得
4. チルト角度 + IMU姿勢 + LiDARスキャンデータを1スライスとしてRPiに送信
5. RPi側で座標変換し、3Dポイントクラウドを構築
6. スキャン完了後、PLY/CSV保存 or リアルタイム表示

---

## 2. ハードウェア仕様

### 2.1 コンポーネント一覧

| コンポーネント | 型番 | 数量 | 主な仕様 |
|---|---|---|---|
| マイコン | Seeed XIAO ESP32-C3 | 1 | RISC-V 160MHz, 400KB RAM, USB CDC, WiFi/BLE |
| サーボドライバー | Seeed バスサーボドライバーボード for XIAO | 1 | GH1.25コネクタ、UART半二重変換、DC IN 5.5×2.1mm |
| サーボモーター | FEETECH STS3215 (7.4V版) | 1 | 7.4V, 19.5kg·cm stall, 12bit磁気エンコーダ, 4096ステップ/360°, TTLシリアルバス 1Mbps |
| LiDAR | YDLIDAR T-mini Pro | 1 | 360° ToF, 4000pts/s, 0.02-12m, UART 230400bps, 3.3Vレベル, GH1.25-4P |
| IMU | MPU6050 (ブレイクアウトボード) | 1 | 6DoF, I2C 400kHz, addr 0x68, 3.3V動作 |
| ホスト | Raspberry Pi 4/5 | 1 | USB接続、Python 3.9+ |

### 2.2 電源設計

| 電源 | 電圧 | 電流容量 | 供給先 | 接続 |
|---|---|---|---|---|
| 7.4V (2S LiPo or ACアダプタ) | 7.4V | 3A以上 | STS3215 サーボ | ドライバーボード DC IN (5.5×2.1mm) |
| 5V (DC-DCコンバータ or USB) | 5V | 1.5A以上 | LiDAR T-mini Pro, XIAO ESP32-C3 | LiDAR VCC, XIAO USB-C |
| 3.3V (XIAO内蔵レギュレータ) | 3.3V | — | MPU6050 | XIAO 3.3Vピン |

> **注意**: 7.4V→5V DC-DC (例: MP1584) を使えば1電源に統一可能。GNDは全デバイスで共通化必須。

### 2.3 ピンアサイン（XIAO ESP32-C3）

| XIAOピン | GPIO | 機能 | 接続先 | プロトコル | 備考 |
|---|---|---|---|---|---|
| D6 | GPIO20 | UART0 TX | サーボドライバーボード内部配線 | TTL 1Mbps | ボードに差し込むだけ |
| D7 | GPIO21 | UART0 RX | サーボドライバーボード内部配線 | TTL 1Mbps | ボードに差し込むだけ |
| D2 | GPIO4 | UART1 RX | T-mini Pro Pin2 (TX) | 230400bps | 外部配線必要 |
| D3 | GPIO5 | UART1 TX | T-mini Pro Pin4 (RX) | 230400bps | 外部配線必要 |
| D4 | GPIO6 | I2C SDA | MPU6050 SDA | 400kHz | 外部配線必要 |
| D5 | GPIO7 | I2C SCL | MPU6050 SCL | 400kHz | 外部配線必要 |
| USB-C | GPIO18/19 | USB CDC | Raspberry Pi | Binary protocol | XIAOのUSBポート |
| D0, D1, D8, D9, D10 | — | 未使用 | — | — | 拡張用予備 |

### 2.4 LiDAR接続 (GH1.25-4P)

| T-mini Pro Pin | 信号 | 接続先 |
|---|---|---|
| Pin1: VCC | 5V | 外部5V電源 (+) |
| Pin2: TX | UART出力 (3.3V) | XIAO D2 (GPIO4, RX1) |
| Pin3: GND | 0V | 共通GND |
| Pin4: RX | UART入力 (3.3V) | XIAO D3 (GPIO5, TX1) |

> レベル変換不要。T-mini ProのUART信号は3.3Vレベル。

### 2.5 MPU6050接続

| MPU6050 Pin | 接続先 |
|---|---|
| VCC | XIAO 3.3V |
| GND | 共通GND |
| SDA | XIAO D4 (GPIO6) |
| SCL | XIAO D5 (GPIO7) |
| AD0 | GND (I2Cアドレス = 0x68) |

### 2.6 サーボドライバーボード設定

- **UART接続モード**を使用（USB接続モードではない）
- ボード前面の2ピンジャンパーキャップは**短絡しない**（デフォルト状態のまま）
- ボード裏面のはんだジャンパーは変更不要
- XIAOをボードのヘッダーに直接差し込んで使用
- STS3215はサーボコネクタにデイジーチェーン接続

### 2.7 機械構成

- STS3215の出力軸にLiDAR T-mini Proをマウント
- **チルト軸**: 水平 = 0°, 上向き = +, 下向き = -
- **チルト範囲**: -45° 〜 +45° (合計90°)
- STS3215のID = 1
- STS3215のセンター位置 = 2048 (12bit中点、水平0°に対応)

---

## 3. 通信プロトコル仕様

### 3.1 フレーム形式

ESP32-C3 ↔ Raspberry Pi 間のUSB CDCバイナリプロトコル。

```
[SYNC1][SYNC2][TYPE][LEN_L][LEN_H][PAYLOAD...][CHECKSUM]
  0xFE   0xEF  1byte  ---- 2byte ----  LEN bytes   XOR
```

| フィールド | サイズ | 説明 |
|---|---|---|
| SYNC1 | 1 byte | 0xFE (固定) |
| SYNC2 | 1 byte | 0xEF (固定) |
| TYPE | 1 byte | パケットタイプ |
| LEN_L | 1 byte | ペイロード長の下位バイト |
| LEN_H | 1 byte | ペイロード長の上位バイト |
| PAYLOAD | LEN bytes | データ本体 |
| CHECKSUM | 1 byte | TYPE ^ LEN_L ^ LEN_H ^ (全PAYLOADバイトのXOR) |

### 3.2 パケットタイプ

#### ESP32 → RPi (データ送信)

| Type | 名称 | 送信頻度 | Payload構造 |
|---|---|---|---|
| 0x02 | IMU data | 50Hz | `[pitch_f32][roll_f32][yaw_f32]` (12 bytes) |
| 0x04 | 3D scan slice | 各チルトステップ完了時 | 下記参照 |
| 0x05 | 3D scan status | 状態変化時 | `[state_u8][step_u16][total_u16]` (5 bytes) |

#### RPi → ESP32 (コマンド送信)

| Type | 名称 | Payload構造 |
|---|---|---|
| 0x10 | Servo angle (マニュアル) | `[angle_deg_f32]` (4 bytes) |
| 0x12 | 3D scan start | `[tilt_min_f32][tilt_max_f32][tilt_step_f32]` (12 bytes) またはペイロード0でデフォルト値使用 |
| 0x13 | 3D scan stop | ペイロードなし (LEN=0) |

### 3.3 3D scan slice (0x04) 詳細

```
Offset  Size  Field
0       4     tilt_deg      (float32) サーボのチルト角度 [度]
4       4     imu_pitch     (float32) IMU pitch [度]
8       4     imu_roll      (float32) IMU roll [度]
12      2     count         (uint16)  ポイント数 N
14      5×N   points        各ポイント:
                              [0-1] angle_x100 (uint16) LiDAR角度×100
                              [2-3] dist_mm    (uint16) 距離 [mm]
                              [4]   quality    (uint8)  信号品質
```

### 3.4 3D scan status (0x05) 詳細

| state値 | 意味 |
|---|---|
| 0 | スキャン開始 |
| 1 | 進行中 (step/total で進捗表示) |
| 2 | スキャン完了 |

### 3.5 USB CDC設定

- ボーレート: 921600 (USB CDCは実質ボーレート無関係だが設定必要)
- Arduino IDE設定: **USB CDC On Boot = Enabled**
- RPi側デバイス: `/dev/ttyACM0`
- DTR/RTS: 無効化 (ESP32リセット防止)

---

## 4. ESP32-C3 ファームウェア仕様

### 4.1 開発環境

- **IDE**: Arduino IDE 2.x
- **ボード**: Seeed Studio XIAO ESP32-C3
- **必須ライブラリ**:
  - `SCServo` — Feetech STS/SCS サーボ制御 (https://github.com/workloads/scservo)
  - `Wire` — I2C (Arduino標準)
- **ボード設定**:
  - USB CDC On Boot: **Enabled**
  - Upload Speed: 921600

### 4.2 シリアルポート割り当て

```c
// USB CDC → Raspberry Pi (バイナリプロトコル)
Serial    // USBSerial (CDC)

// UART0 → STS3215 サーボ (ドライバーボード内部配線)
Serial0   // HardwareSerial(0), pins D6/D7, 1Mbps

// UART1 → YDLIDAR T-mini Pro (外部配線)
HardwareSerial LidarSerial(1);  // pins D2(RX)/D3(TX), 230400bps
```

### 4.3 定数定義

```c
// ピン
#define LIDAR_RXD     4    // D2 = GPIO4
#define LIDAR_TXD     5    // D3 = GPIO5
#define IMU_SDA       6    // D4 = GPIO6
#define IMU_SCL       7    // D5 = GPIO7

// デバイス
#define MPU6050_ADDR      0x68
#define SERVO_ID_TILT     1
#define SERVO_CENTER      2048   // 水平0° = 12bit中点

// 通信速度
#define LIDAR_BAUDRATE    230400
#define SERVO_BAUDRATE    1000000
#define USB_BAUDRATE      921600

// プロトコル
#define PROTO_SYNC1       0xFE
#define PROTO_SYNC2       0xEF

// 3Dスキャンデフォルト値
#define TILT_MIN_DEFAULT    -45.0f  // 度
#define TILT_MAX_DEFAULT     45.0f
#define TILT_STEP_DEFAULT     2.0f
#define TILT_SAFE_LIMIT      50.0f  // サーボ保護上限

// タイミング
#define SETTLE_MS            120    // サーボ安定待ち [ms]
#define IMU_READ_MS           10    // IMU読み取り周期 (100Hz)
#define IMU_SEND_MS           50    // IMU送信周期 (50Hz)
```

### 4.4 LiDARパケットパーサー

T-mini Proのデータフォーマット:

```
[0xAA][0x55][CT][LSN][FSA_L][FSA_H][LSA_L][LSA_H][CS_L][CS_H][Si...]
```

- ヘッダ: 0xAA 0x55
- CT: パケットタイプ (bit0=1 で新スキャン開始)
- LSN: サンプル数
- FSA: 開始角度 (上位15bit / 64 = 度)
- LSA: 終了角度
- Si: 各ポイント3バイト (quality, dist_L, dist_H)

**パーサー状態機械**:
```
WAIT_H1 → WAIT_H2 → READ_CT → READ_LSN → READ_DATA → (処理) → WAIT_H1
```

**1スキャン完了検出**: CT の bit0 が 1 になった時、前回蓄積分を1スキャンとして確定。

### 4.5 MPU6050 初期化・読み取り

- ジャイロ: ±250°/s (感度 131 LSB/°/s)
- 加速度: ±2g (感度 16384 LSB/g)
- DLPF: 44Hz (レジスタ 0x1A = 0x03)
- 相補フィルタ: α=0.96 (ジャイロ重み96%、加速度4%)

```
pitch = α × (pitch + gyro_y × dt) + (1-α) × atan2(ax, sqrt(ay²+az²))
roll  = α × (roll  + gyro_x × dt) + (1-α) × atan2(ay, sqrt(ax²+az²))
yaw  += gyro_z × dt   // ドリフトあり、参考値
```

### 4.6 3Dスキャン状態機械

```
S3D_IDLE ──(CMD_3DSCAN_START)──→ S3D_MOVING
                                    │
                                    ▼
                              setTiltAngle(current)
                                    │
                                    ▼
                              S3D_SETTLING ──(SETTLE_MS経過)──→ S3D_CAPTURING
                                                                    │
                                                                    ▼
                                                              (scanComplete==true)
                                                                    │
                                                                    ▼
                                                              S3D_SENDING
                                                                    │
                                                          send3DSlice() + send3DStatus()
                                                                    │
                                                              stepIndex++
                                                                    │
                                                        ┌───(範囲内)───┐
                                                        ▼             ▼
                                                   S3D_MOVING    S3D_COMPLETE
                                                                    │
                                                              setTiltAngle(0°)
                                                              send3DStatus(2,...)
                                                                    │
                                                                    ▼
                                                               S3D_IDLE
```

### 4.7 サーボ角度変換

```c
// 角度(度) → STS3215ポジション (0-4095)
int16_t pos = SERVO_CENTER + (int16_t)(angleDeg * 4096.0f / 360.0f);
pos = constrain(pos, 0, 4095);

// 分解能: 360° / 4096 ≈ 0.088°/step
```

### 4.8 loop() の処理優先度

```
1. LiDAR UART受信 (while available, ノンブロッキング) ← 最優先
2. RPi USBコマンド受信 (while available)
3. IMU読み取り (10ms周期)
4. IMUデータ送信 (50ms周期)
5. 3Dスキャン状態機械更新
```

> ESP32-C3はシングルコア (RISC-V) のため、すべてノンブロッキング処理が必須。

---

## 5. Raspberry Pi ソフトウェア仕様

### 5.1 開発環境

- **言語**: Python 3.9+
- **必須パッケージ**: `pyserial`, `numpy`
- **推奨パッケージ**: `open3d` (3D可視化), `matplotlib` (フォールバック)

```bash
pip install pyserial numpy open3d
```

### 5.2 CLIインターフェース

```bash
# デフォルトスキャン (-45° ~ +45°, 2°刻み)
python3 rpi_tilt_3d.py --port /dev/ttyACM0

# パラメータ指定 + ファイル保存
python3 rpi_tilt_3d.py --tilt-min -45 --tilt-max 45 --step 1.0 --save scan.ply

# CSV保存 (CloudCompare等で開ける)
python3 rpi_tilt_3d.py --step 2 --save scan.csv --no-viz

# マニュアルチルト制御 (テスト用)
python3 rpi_tilt_3d.py --manual-tilt 30.0
```

| 引数 | 型 | デフォルト | 説明 |
|---|---|---|---|
| `--port` | str | `/dev/ttyACM0` | シリアルポート |
| `--baud` | int | 921600 | ボーレート |
| `--tilt-min` | float | -45.0 | チルト開始角度 [度] |
| `--tilt-max` | float | 45.0 | チルト終了角度 [度] |
| `--step` | float | 2.0 | チルトステップ [度] |
| `--save` | str | None | 保存ファイル名 (.ply or .csv) |
| `--no-viz` | flag | False | 可視化無効 |

### 5.3 座標変換

チルト角 α、LiDARスキャン角 θ、距離 d から3D座標 (X, Y, Z) を算出する。

#### 座標系定義

```
X: LiDAR前方 (tilt=0 時の0°方向)
Y: LiDAR左方
Z: 鉛直上方

チルト角α: 正 = 上向き, 負 = 下向き
LiDAR角θ: 0° = X方向, 反時計回り正
```

#### 変換式

```python
# Step 1: LiDAR 2D極座標 → ローカル3D
x_l = d * cos(θ)
y_l = d * sin(θ)

# Step 2: Y軸周りにチルト回転
X = x_l * cos(α)
Y = y_l
Z = -x_l * sin(α)

# Step 3: IMU補正 (プラットフォームの傾き)
R_pitch = Ry(imu_pitch)
R_roll  = Rx(imu_roll)
[X, Y, Z] = R_roll @ R_pitch @ [X, Y, Z]
```

#### NumPy実装

```python
def transform_slice_to_3d(angles_deg, dists_mm, tilt_deg,
                          imu_pitch_deg=0, imu_roll_deg=0,
                          min_dist_mm=20, max_dist_mm=12000):
    mask = (dists_mm > min_dist_mm) & (dists_mm < max_dist_mm)
    angles = np.radians(angles_deg[mask])
    dists = dists_mm[mask].astype(np.float64) / 1000.0  # → meters

    x_l = dists * np.cos(angles)
    y_l = dists * np.sin(angles)

    a = math.radians(tilt_deg)
    X = x_l * math.cos(a)
    Y = y_l
    Z = -x_l * math.sin(a)

    pts = np.stack([X, Y, Z], axis=1)

    # IMU correction (optional)
    if abs(imu_pitch_deg) > 0.1 or abs(imu_roll_deg) > 0.1:
        R = Rx(roll) @ Ry(pitch)
        pts = (R @ pts.T).T

    return pts  # Nx3 [meters]
```

### 5.4 ポイントクラウドカラーリング

高さ (Z値) に基づくグラデーション:

```
Z低 (床)  → 青  [0, 0, 1]
Z中 (壁)  → 緑  [0, 1, 0]
Z高 (天井) → 赤  [1, 0, 0]
```

### 5.5 出力ファイル形式

#### PLY (推奨)

Open3D, CloudCompare, MeshLab で開ける。

```
ply
format ascii 1.0
element vertex N
property float x
property float y
property float z
property uchar red
property uchar green
property uchar blue
end_header
1.2345 -0.5678 0.9012 255 128 0
...
```

#### CSV

```
x,y,z
1.2345,-0.5678,0.9012
...
```

### 5.6 可視化

- **Open3D** (推奨): リアルタイムに逐次更新。スキャン進行中にポイントが増えていく様子が見える。
- **matplotlib** (フォールバック): スキャン完了後に静的3D散布図。30,000点以上はランダムダウンサンプリング。

---

## 6. スキャンパフォーマンス

### 6.1 想定値

| パラメータ | 値 |
|---|---|
| LiDAR回転速度 | 6Hz (デフォルト) |
| 1スキャンあたりのポイント数 | ~667 (4000pts/s ÷ 6Hz) |
| サーボ安定待ち | 120ms |
| 1スライスの所要時間 | ~167ms (スキャン) + 120ms (安定) ≈ 287ms |

| ステップ幅 | スライス数 | 合計ポイント (概算) | 所要時間 (概算) |
|---|---|---|---|
| 2° | 46 | ~31K | ~13s |
| 1° | 91 | ~61K | ~26s |
| 0.5° | 181 | ~121K | ~52s |

### 6.2 USB CDC帯域

- 1スライス: 14 (header) + 667×5 (points) = ~3.35 KB
- 46スライス合計: ~154 KB
- USB CDC実効帯域 (12Mbps): 十分余裕あり

### 6.3 メモリ使用量 (ESP32-C3)

- LiDARスキャンバッファ: 700 × 5 bytes = 3.5 KB
- LiDARパケットバッファ: 256 bytes
- コマンドバッファ: 64 bytes
- ESP32-C3 RAM: 400 KB → 十分余裕あり

---

## 7. ファイル構成

```
project/
├── firmware/
│   └── lidar_tilt_3d/
│       └── lidar_tilt_3d.ino          # ESP32-C3 ファームウェア (Arduino)
├── host/
│   ├── rpi_tilt_3d.py                 # RPi 3Dスキャン + 可視化 (メイン)
│   ├── requirements.txt               # Python依存パッケージ
│   └── README.md                      # RPi側セットアップ手順
├── docs/
│   ├── SPEC.md                        # この仕様書
│   └── wiring.md                      # 配線図・写真
└── README.md                          # プロジェクト概要
```

### 7.1 requirements.txt

```
pyserial>=3.5
numpy>=1.21
open3d>=0.17
matplotlib>=3.5
```

---

## 8. 実装上の注意事項

### 8.1 ESP32-C3 固有の制約

- **シングルコア RISC-V**: すべてノンブロッキング処理。delay()はsetup()以外で使用禁止。
- **UART は2系統のみ**: UART0=サーボ、UART1=LiDAR で全て使い切り。デバッグ出力はUSB CDC経由。
- **Arduino IDE設定**: `USB CDC On Boot` を `Enabled` にしないと `Serial` がUSB CDCにマッピングされない。
- **Serial0 と Serial の違い**: `USB CDC On Boot = Enabled` の場合、`Serial` = USB CDC、`Serial0` = UART0 (HardwareSerial)。

### 8.2 サーボ制御の注意

- STS3215 のUARTは**半二重**。ReadPos/ReadLoad等のフィードバック読み取りは3Dスキャン中は行わない（スライス間に限定）。
- サーボ位置はエンコーダで保証されるが、移動後の振動が収まるまで120ms以上待つ。
- `st.WritePosEx(id, position, time, speed, acc)` — time=0 で即時移動。

### 8.3 LiDARデータの注意

- T-mini Pro は電源投入後、モーターが安定するまで数秒かかる。`0xA5 0x60` でスキャン開始コマンド送信。
- スキャンデータに距離0のポイントが含まれる場合がある → `dist_mm > 20` でフィルタ。
- 周囲光が強い環境ではquality値が低下。quality でフィルタする場合は閾値10程度。

### 8.4 RPi側の注意

- `Serial` 接続時に DTR/RTS を `False` にしないと ESP32-C3 がリセットされる。
- USB CDC のバッファオーバーフロー対策: `ser.read(512)` でチャンクリード。
- Open3D の `Visualizer` はメインスレッドで実行する必要がある。データ受信はバックグラウンドスレッド。

### 8.5 座標変換の注意

- すべての角度は**度**で送受信し、三角関数の直前にラジアンに変換する。
- 距離は**mm**で送受信し、座標変換後に**m**に変換する。
- IMU補正は微小角度（0.1°未満）では省略してよい。

---

## 9. テスト手順

### 9.1 ハードウェアテスト

1. **サーボ単体テスト**: Arduino IDEのシリアルモニタから `j`/`k` でサーボ動作確認 (SCServoサンプル)
2. **LiDAR単体テスト**: UART1からデータ受信できることを確認 (パケットヘッダ 0xAA 0x55 の検出)
3. **IMU単体テスト**: I2Cスキャンで0x68検出、pitch/roll値がシリアルモニタに出力されることを確認
4. **USB CDC テスト**: RPiに接続し `/dev/ttyACM0` が認識されること、`pyserial` で通信できること

### 9.2 統合テスト

1. **マニュアルチルト**: RPi から CMD_SERVO_ANGLE で±45°まで動作することを確認
2. **2Dスキャン確認**: tilt=0° 固定で1スライス分のデータがRPiに届くこと確認
3. **3Dスキャン (粗い)**: `--step 10` で9スライスの高速テスト
4. **3Dスキャン (標準)**: `--step 2` で46スライスの標準テスト、PLY保存 → CloudCompare で開いて確認

### 9.3 性能確認

- チェックサムエラー率が0.1%以下であること
- スライス間のチルト角度が等間隔であること (サーボフィードバックで確認)
- IMU pitch/roll が静止時 ±0.5°以内であること
- ポイントクラウドが空間的に整合していること (壁が直線、床が平面に見えるか)

---

## 10. 将来の拡張

- **ROS2対応**: `sensor_msgs/PointCloud2` パブリッシャーノードの追加
- **SLAM統合**: RPi上で `rtabmap` 等を使ったリアルタイムマッピング
- **WiFi送信**: ESP32-C3のWiFiを使い、USB接続不要でデータ送信
- **可変ステップ**: チルト端部 (±40°〜±45°) のステップを細かくして点密度を均一化
- **連続スキャン**: 往復チルト（-45→+45→-45→...）で継続的にポイントクラウドを更新
