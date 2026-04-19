# Seeed Bus Servo Driver Board × XIAO ESP32C3 × Feetech STS3215 セットアップガイド

> **対象ハードウェア**
> - Seeed Studio Bus Servo Driver Board（XIAO用）
> - Seeed Studio XIAO ESP32C3
> - Feetech シリアルバスサーボ STS3215（7.4V / 16〜27.4 kg·cm / 1:191 / 12bit 磁気エンコーダー）

---

## 1. サーボ STS3215 基本スペック

| 項目 | 値 |
|---|---|
| 型番 | STS3215 C044（1:191） |
| 動作電圧 | 6〜8.4V（推奨 7.4V） |
| ストールトルク | 27.4 kg·cm @7.4V |
| 定格トルク | 9 kg·cm |
| ギア比 | 1:191（金属ギア） |
| エンコーダー | 12bit 磁気エンコーダー |
| ポジション範囲 | 0〜4095 steps（360°） |
| 通信プロトコル | TTL シリアルバス（STSシリーズ） |
| 通信ボーレート | 1,000,000 bps |
| 定格電流 | 650 mA @7.4V |
| 外形寸法 | 45.2 × 24.7 × 35 mm |
| 重量 | 55 g |

---

## 2. システム構成図

```
[7.4V 電源]
    │
    ├─── [Bus Servo Driver Board] ──── [STS3215 サーボ #1]
    │            │                 └── [STS3215 サーボ #2] (デイジーチェーン)
    │            │
    │      [XIAO ESP32C3]
    │            │
    │     USB-C (書き込み・デバッグ用)
    │
    └─── GND 共通 ──── [XIAO GND]
```

> **重要：** サーボ電源（7.4V）と XIAO の電源は**別系統**にし、GND は必ず共通化すること。

---

## 3. ハードウェアセットアップ

### 3-1. ジャンパー設定

XIAO ESP32C3 で直接制御する場合は **UART モード**（デフォルト）を使用します。

| モード | 用途 | 設定 |
|---|---|---|
| UART モード（推奨） | XIAO などの MCU と直接接続 | ジャンパーをUART側に設定 |
| USB モード | PC の USB-UART 経由で接続 | ジャンパーをUSB側に設定 |

### 3-2. 物理接続

XIAO ESP32C3 をドライバーボードのヘッダーに**直接差し込む**だけで UART 接続が完了します（デュポンワイヤー不要）。

| ドライバーボード | XIAO ESP32C3 |
|---|---|
| RX | TX（D7 / GPIO21） |
| TX | RX（D6 / GPIO20） |

### 3-3. 電源接続

ドライバーボードの **2P 3.5mm スクリュー端子**に 7.4V 電源を接続します。

- 推奨電源：7.4V LiPo バッテリー（2セル）または 7.4V DCアダプター
- 電流容量の目安：サーボ1台あたり最低 **1〜2 A** の余裕を確保
  - 複数台同時動作・起動時の突入電流を考慮すること

---

## 4. Arduino IDE セットアップ

### 4-1. ESP32 ボードライブラリの追加

1. `File > Preferences` を開く
2. **Additional Boards Manager URLs** に以下を追加：

```
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
```

3. `Tools > Board > Boards Manager` で `esp32` を検索してインストール
4. ボード選択：`Tools > Board > ESP32 Arduino > **XIAO_ESP32C3**`

### 4-2. SCServo ライブラリのインストール

`Tools > Manage Libraries` で `SCServo` を検索してインストール。

または GitHub からダウンロードして以下に配置：

```
Documents/Arduino/libraries/SCServo/
```

---

## 5. ライブラリのクラス選択（重要）

STS3215 は **STSシリーズ（SMS/STSプロトコル）** を使用します。

| クラス | 対象プロトコル | STS3215 |
|---|---|---|
| `SMS_STS` | STS / SMS シリーズ | ✅ **こちらを使用** |
| `SCSCL` | SCS シリーズ | ❌ 使用しない |

---

## 6. スケッチ（Arduino コード）

### 6-1. 基本動作（位置制御）

```cpp
#include <SCServo.h>

SMS_STS sms_sts;

// ESP32C3 では Serial0 を使用
#define COMSerial Serial0

// XIAO ESP32C3 の UART ピン
#define S_RXD 20  // D6
#define S_TXD 21  // D7

// サーボパラメータ
#define SERVO_ID    1       // サーボ ID（出荷時デフォルト: 1）
#define POS_CENTER  2048    // 中央位置（12bit: 0〜4095）
#define POS_MIN     100     // 最小位置
#define POS_MAX     3995    // 最大位置
#define SPEED       1500    // 移動速度（steps/s）
#define ACC         50      // 加速度

void setup() {
  Serial.begin(115200);  // デバッグ用（USB-CDC）

  COMSerial.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &COMSerial;
  delay(1000);

  Serial.println("STS3215 Ready");
}

void loop() {
  // 中央位置へ移動
  sms_sts.WritePosEx(SERVO_ID, POS_CENTER, SPEED, ACC);
  Serial.println("Center");
  delay(2000);

  // 最小位置へ移動
  sms_sts.WritePosEx(SERVO_ID, POS_MIN, SPEED, ACC);
  Serial.println("Min");
  delay(2000);

  // 最大位置へ移動
  sms_sts.WritePosEx(SERVO_ID, POS_MAX, SPEED, ACC);
  Serial.println("Max");
  delay(2000);
}
```

### 6-2. 複数サーボの同期制御

```cpp
#include <SCServo.h>

SMS_STS sms_sts;

#define COMSerial Serial0
#define S_RXD 20
#define S_TXD 21

// サーボ数と各サーボのパラメータ
#define SERVO_NUM 2

u8  ID[]    = {1, 2};
s16 Pos[]   = {2048, 2048};
u16 Speed[] = {1500, 1500};
u8  ACC[]   = {50, 50};

void setup() {
  Serial.begin(115200);
  COMSerial.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &COMSerial;
  delay(1000);
}

void loop() {
  // 全サーボを中央へ
  for (int i = 0; i < SERVO_NUM; i++) Pos[i] = 2048;
  sms_sts.SyncWritePosEx(ID, SERVO_NUM, Pos, Speed, ACC);
  delay(2000);

  // 全サーボを最小位置へ
  for (int i = 0; i < SERVO_NUM; i++) Pos[i] = 200;
  sms_sts.SyncWritePosEx(ID, SERVO_NUM, Pos, Speed, ACC);
  delay(2000);
}
```

### 6-3. フィードバック取得（位置・速度・負荷・電圧）

```cpp
#include <SCServo.h>

SMS_STS sms_sts;

#define COMSerial Serial0
#define S_RXD 20
#define S_TXD 21
#define SERVO_ID 1

void setup() {
  Serial.begin(115200);
  COMSerial.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &COMSerial;
  delay(1000);
}

void loop() {
  int pos  = sms_sts.ReadPos(SERVO_ID);      // 現在位置 (0〜4095)
  int spd  = sms_sts.ReadSpeed(SERVO_ID);    // 現在速度 (steps/s)
  int load = sms_sts.ReadLoad(SERVO_ID);     // 負荷（トルク相当）
  int volt = sms_sts.ReadVoltage(SERVO_ID);  // 電圧（単位: 0.1V）
  int temp = sms_sts.ReadTemper(SERVO_ID);   // 温度（℃）

  Serial.print("Pos: ");  Serial.print(pos);
  Serial.print("  Spd: "); Serial.print(spd);
  Serial.print("  Load: "); Serial.print(load);
  Serial.print("  Volt: "); Serial.print(volt * 0.1, 1); Serial.print("V");
  Serial.print("  Temp: "); Serial.print(temp); Serial.println("℃");

  delay(500);
}
```

---

## 7. `WritePosEx` パラメータ詳細

```cpp
sms_sts.WritePosEx(u8 ID, s16 Position, u16 Speed, u8 ACC);
```

| 引数 | 型 | 説明 | 推奨範囲 |
|---|---|---|---|
| `ID` | u8 | サーボの ID | 1〜253 |
| `Position` | s16 | 目標位置（steps） | 0〜4095（12bit全範囲） |
| `Speed` | u16 | 移動速度（steps/s） | 0〜3400（0=最大速度） |
| `ACC` | u8 | 加速度 | 0〜254（0=最大加速度） |

**ポジションと角度の換算：**

```
角度(°) = position × (360 / 4096)
position = 角度(°) × (4096 / 360)

例：
  0°    → position = 0
  90°   → position = 1024
  180°  → position = 2048（中央）
  270°  → position = 3072
  360°  → position = 4095
```

---

## 8. サーボ ID の設定

複数台使用する場合、**1台ずつ**接続して Feetech の設定ツールで ID を割り当てます。

- ツール：**FT SCServo Debug**（Feetech 公式、Windows用）
- 出荷時のデフォルト ID：`1`
- 同一バス上に同じ ID が存在すると通信が衝突するため、必ず重複しないように設定すること

---

## 9. 注意事項チェックリスト

| チェック | 内容 |
|---|---|
| ☐ 電源電圧 | サーボ電源は 7.4V（6〜8.4V 範囲内）であること |
| ☐ 電源分離 | XIAO の電源とサーボ電源は別系統で供給すること |
| ☐ GND 共通 | サーボ電源の GND と XIAO の GND を必ず接続すること |
| ☐ ジャンパー | UART モードに設定されていること（デフォルト） |
| ☐ クラス選択 | `SMS_STS` クラスを使用していること（STSシリーズ用） |
| ☐ ボーレート | `1,000,000` bps で初期化していること |
| ☐ シリアルポート | `Serial0` を使用していること（ESP32C3） |
| ☐ サーボ ID | 接続する全サーボの ID が重複していないこと |
| ☐ 通電前確認 | 配線が正しく接続されてから電源を投入すること |

---

## 10. トラブルシューティング

| 症状 | 原因 | 対処 |
|---|---|---|
| サーボが動かない | ID が一致していない | FT SCServo Debug で ID を確認 |
| サーボが動かない | ジャンパーが USB モードになっている | UART モードに変更 |
| 通信エラーが出る | GND が共通化されていない | サーボ電源 GND と XIAO GND を接続 |
| サーボが振動・暴走する | 電源電流が不足している | 電源容量を増やす（最低1〜2A/台） |
| 読み取り値が -1 になる | 通信タイムアウト | ボーレートと配線（RX/TX）を確認 |
| 書き込みできない | ESP32C3 がブートモードになっていない | BOOTボタンを押しながらUSB接続 |

---

## 11. 参考リンク

- [Seeed Wiki - Bus Servo Driver Board（日本語）](https://wiki.seeedstudio.com/ja/bus_servo_driver_board/)
- [Seeed Wiki - Bus Servo Driver Board（英語）](https://wiki.seeedstudio.com/bus_servo_driver_board/)
- [Feetech 公式 - STS3215 製品ページ](https://www.feetechrc.com/74v-19-kgcm-plastic-case-metal-tooth-magnetic-code-double-axis-ttl-series-steering-gear.html)
- [SCServo ライブラリ（GitHub）](https://github.com/workloads/scservo)
- [Seeed Wiki - XIAO ESP32C3 Getting Started（日本語）](https://wiki.seeedstudio.com/ja/XIAO_ESP32C3_Getting_Started/)
