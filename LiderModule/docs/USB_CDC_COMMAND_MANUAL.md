# LiderModule USB CDC Command Manual

この文書は、Raspberry Pi から USB CDC (`/dev/ttyACM0`) 経由で LiderModule を制御するための通信仕様です。ClaudeCode / Codex / 任意のホストプログラムが、この内容だけを読んで通信制御を実装できることを目的とします。

対象ファームウェア:

- `LiderModule/firmware/lidar_tilt_3d/lidar_tilt_3d.ino`
- USB CDC binary protocol
- 本番プロファイル: サーボ安定待ちなし (`SCAN_SETTLE_DISABLED=1`)

## 1. Serial 設定

| 項目 | 値 |
|---|---|
| デバイス | `/dev/ttyACM0` |
| baudrate | `921600` |
| data bits | 8 |
| parity | none |
| stop bits | 1 |
| flow control | none |
| DTR / RTS | `False` 推奨 |

Raspberry Pi 側では `pyserial` を使う場合、ESP32-C3 の不要なリセットを避けるため、open 後に `dtr` と `rts` を `False` にしてください。

```python
import serial

ser = serial.Serial("/dev/ttyACM0", 921600, timeout=0.05, write_timeout=1)
ser.dtr = False
ser.rts = False
```

## 2. フレーム形式

全コマンド/メッセージは同じフレーム形式です。数値はリトルエンディアンです。

```text
[SYNC1][SYNC2][TYPE][LEN_L][LEN_H][PAYLOAD...][CHECKSUM]
 0xFE   0xEF  1byte  1byte   1byte     LEN bytes   1byte
```

| フィールド | サイズ | 内容 |
|---|---:|---|
| `SYNC1` | 1 | 固定 `0xFE` |
| `SYNC2` | 1 | 固定 `0xEF` |
| `TYPE` | 1 | コマンド/メッセージ種別 |
| `LEN_L` | 1 | payload 長の下位 byte |
| `LEN_H` | 1 | payload 長の上位 byte |
| `PAYLOAD` | `LEN` | 種別ごとのデータ |
| `CHECKSUM` | 1 | XOR checksum |

Checksum:

```text
CHECKSUM = TYPE ^ LEN_L ^ LEN_H ^ payload[0] ^ payload[1] ^ ... ^ payload[LEN-1]
```

payload が空の場合:

```text
CHECKSUM = TYPE ^ 0x00 ^ 0x00
```

## 3. 型

| 型 | サイズ | endian | Python struct |
|---|---:|---|---|
| `u8` | 1 | - | `B` |
| `u16` | 2 | little | `<H` |
| `f32` | 4 | little | `<f` |

角度は degree 単位です。

## 4. Raspberry Pi -> ESP32 コマンド

### 4.1 `0x10` Servo Angle

サーボを指定角度へ移動します。LiDAR計測は開始しません。

| 項目 | 値 |
|---|---|
| Type | `0x10` |
| Payload | `[angle_deg_f32]` |
| Payload length | 4 |

Payload:

```text
offset  size  type  name
0       4     f32   angle_deg
```

例: 0度へ戻す

```python
send_frame(ser, 0x10, struct.pack("<f", 0.0))
```

角度範囲:

- ファーム側で `-50.0` から `+50.0` degree に制限されます。
- 運用上の検出範囲は下方 `0` から `-45` degree、上方 `0` から `+45` degree です。

### 4.2 `0x12` 3D Scan Start

サーボを指定範囲で動かし、各チルト角でLiDARスライスを送信します。

| 項目 | 値 |
|---|---|
| Type | `0x12` |
| Payload | `[tilt_min_f32][tilt_max_f32][tilt_step_f32]` |
| Payload length | 12 |

Payload:

```text
offset  size  type  name
0       4     f32   tilt_min
4       4     f32   tilt_max
8       4     f32   tilt_step
```

ファーム側の動作:

- `tilt_min` / `tilt_max` は `-50.0` から `+50.0` degree に制限されます。
- `tilt_max < tilt_min` の場合、ファーム側で入れ替えます。
- `tilt_step <= 0` の場合、デフォルト値を使います。
- 本番プロファイルではサーボ安定待ちは行わず、移動中のLiDAR点も含めて取得します。
- スキャン開始時にLiDARモーターを起動し、完了/停止時に停止します。

単発スライス:

```python
# 指定角度で1スライス取得する実用パターン
angle = -30.0
send_frame(ser, 0x12, struct.pack("<fff", angle, angle, 1.0))
```

線形スキャン:

```python
# -45, -30, -15, 0 degree の4ステップ
send_frame(ser, 0x12, struct.pack("<fff", -45.0, 0.0, 15.0))
```

Payload length 0 の場合:

```text
tilt_min=-45.0, tilt_max=45.0, tilt_step=2.0
```

### 4.3 `0x13` 3D Scan Stop

実行中のスキャンを停止し、サーボを0度へ戻します。

| 項目 | 値 |
|---|---|
| Type | `0x13` |
| Payload | none |
| Payload length | 0 |

```python
send_frame(ser, 0x13)
```

## 5. ESP32 -> Raspberry Pi メッセージ

### 5.1 `0x02` IMU Data

IMU姿勢データです。約50Hzで送信されます。

| 項目 | 値 |
|---|---|
| Type | `0x02` |
| Payload | `[pitch_f32][roll_f32][yaw_f32]` |
| Payload length | 12 |

Payload:

```text
offset  size  type  name
0       4     f32   pitch_deg
4       4     f32   roll_deg
8       4     f32   yaw_deg
```

備考:

- MPU6050 DMP由来の値です。
- yaw は磁気北ではなく相対角です。

### 5.2 `0x04` 3D Scan Slice

1チルト角で取得したLiDAR点群スライスです。

| 項目 | 値 |
|---|---|
| Type | `0x04` |
| Payload | header + points |
| Payload length | `14 + count * 5` |

Payload header:

```text
offset  size  type  name
0       4     f32   tilt_deg
4       4     f32   imu_pitch_deg
8       4     f32   imu_roll_deg
12      2     u16   count
```

Point format, repeated `count` times:

```text
offset from points start  size  type  name
0                         2     u16   angle_x100
2                         2     u16   dist_mm
4                         1     u8    quality
```

Point conversion:

```python
angle_deg = angle_x100 / 100.0
distance_m = dist_mm / 1000.0
```

Filtering recommendation:

- `20 <= dist_mm <= 12000`
- `quality >= threshold` only when needed. Default threshold can be `0`.
- 本番高速検出ではサーボ移動中の点が混ざります。障害物の存在判定では複数スライス/複数周で確率的に扱ってください。

### 5.3 `0x05` 3D Scan Status

スキャン状態通知です。

| 項目 | 値 |
|---|---|
| Type | `0x05` |
| Payload | `[state_u8][step_u16][total_u16]` |
| Payload length | 5 |

Payload:

```text
offset  size  type  name
0       1     u8    state
1       2     u16   step
3       2     u16   total
```

State:

| state | 意味 |
|---:|---|
| 0 | scan started |
| 1 | scan running / slice sent |
| 2 | scan complete |

## 6. 下方/上方パターン制御

現在の運用はCSVパターンをホスト側で読み、各ステップを単発スライスとして送信します。

CSV:

```text
LiderModule/計測パターン.csv
```

下方モードの24ステップ:

```python
DOWN_ANGLES = [
    0.0, -15.0, -30.0, -45.0,
    -42.5, -27.5, -12.5, 0.0,
    -5.0, -20.0, -35.0, -45.0,
    -37.5, -22.5, -7.5, 0.0,
    -10.0, -25.0, -40.0, -45.0,
    -32.5, -17.5, -2.5, 0.0,
]
```

上方モードの24ステップ:

```python
UP_ANGLES = [
    0.0, 15.0, 30.0, 45.0,
    42.5, 27.5, 12.5, 0.0,
    5.0, 20.0, 35.0, 45.0,
    37.5, 22.5, 7.5, 0.0,
    10.0, 25.0, 40.0, 45.0,
    32.5, 17.5, 2.5, 0.0,
]
```

推奨制御:

1. serial open
2. `0x13` stop を送る
3. `0x10` 0度 を送る
4. パターン配列を先頭から処理
5. 各角度で `0x12` に `[angle, angle, 1.0]` を入れて送信
6. `0x04` slice と `0x05` complete を待つ
7. 未受信なら次ステップへ進む、または同角度を再試行する
8. 周回終了後、必要なら次周を続ける
9. 終了時に `0x13` stop と `0x10` 0度 を送る

## 7. Python 最小実装

以下は、フレーム送受信と下方パターン1周の最小例です。

```python
import queue
import struct
import threading
import time
from dataclasses import dataclass

import serial

SYNC1 = 0xFE
SYNC2 = 0xEF

MSG_IMU_DATA = 0x02
MSG_SCAN_SLICE = 0x04
MSG_SCAN_STATUS = 0x05
CMD_SERVO_ANGLE = 0x10
CMD_SCAN_START = 0x12
CMD_SCAN_STOP = 0x13


@dataclass
class ScanSlice:
    tilt_deg: float
    imu_pitch_deg: float
    imu_roll_deg: float
    points: list[tuple[float, int, int]]  # angle_deg, dist_mm, quality


@dataclass
class ScanStatus:
    state: int
    step: int
    total: int


def checksum(msg_type: int, payload: bytes) -> int:
    length = len(payload)
    cs = msg_type ^ (length & 0xFF) ^ ((length >> 8) & 0xFF)
    for b in payload:
        cs ^= b
    return cs & 0xFF


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    return bytes([SYNC1, SYNC2, msg_type, length & 0xFF, length >> 8]) + payload + bytes([checksum(msg_type, payload)])


def send_frame(ser, msg_type: int, payload: bytes = b"") -> None:
    ser.write(encode_frame(msg_type, payload))
    ser.flush()


def decode_payload(msg_type: int, payload: bytes):
    if msg_type == MSG_SCAN_STATUS:
        state, step, total = struct.unpack("<BHH", payload)
        return ScanStatus(state, step, total)

    if msg_type == MSG_SCAN_SLICE:
        tilt, imu_pitch, imu_roll, count = struct.unpack_from("<fffH", payload, 0)
        points = []
        raw = payload[14:]
        for i in range(count):
            angle_x100, dist_mm, quality = struct.unpack_from("<HHB", raw, i * 5)
            points.append((angle_x100 / 100.0, dist_mm, quality))
        return ScanSlice(tilt, imu_pitch, imu_roll, points)

    return None


def reader_loop(ser, out: queue.Queue, stop: threading.Event) -> None:
    state = 0
    msg_type = 0
    length = 0
    payload = bytearray()

    while not stop.is_set():
        chunk = ser.read(512)
        for b in chunk:
            if state == 0:
                state = 1 if b == SYNC1 else 0
            elif state == 1:
                state = 2 if b == SYNC2 else 0
            elif state == 2:
                msg_type = b
                state = 3
            elif state == 3:
                length = b
                state = 4
            elif state == 4:
                length |= b << 8
                payload = bytearray()
                state = 6 if length == 0 else 5
            elif state == 5:
                payload.append(b)
                if len(payload) >= length:
                    state = 6
            elif state == 6:
                if checksum(msg_type, payload) == b:
                    decoded = decode_payload(msg_type, bytes(payload))
                    if decoded is not None:
                        out.put(decoded)
                state = 0


def get_one_slice(ser, events: queue.Queue, angle: float, timeout_s: float = 4.0) -> ScanSlice | None:
    send_frame(ser, CMD_SCAN_START, struct.pack("<fff", angle, angle, 1.0))
    deadline = time.monotonic() + timeout_s
    got_slice = None
    complete = False

    while time.monotonic() < deadline and not (got_slice and complete):
        try:
            event = events.get(timeout=0.1)
        except queue.Empty:
            continue

        if isinstance(event, ScanSlice):
            got_slice = event
        elif isinstance(event, ScanStatus):
            complete = event.state == 2

    return got_slice


def main() -> None:
    down_angles = [
        0.0, -15.0, -30.0, -45.0, -42.5, -27.5, -12.5, 0.0,
        -5.0, -20.0, -35.0, -45.0, -37.5, -22.5, -7.5, 0.0,
        -10.0, -25.0, -40.0, -45.0, -32.5, -17.5, -2.5, 0.0,
    ]

    ser = serial.Serial("/dev/ttyACM0", 921600, timeout=0.05, write_timeout=1)
    ser.dtr = False
    ser.rts = False

    events: queue.Queue = queue.Queue()
    stop = threading.Event()
    thread = threading.Thread(target=reader_loop, args=(ser, events, stop), daemon=True)
    thread.start()

    try:
        send_frame(ser, CMD_SCAN_STOP)
        time.sleep(0.2)
        send_frame(ser, CMD_SERVO_ANGLE, struct.pack("<f", 0.0))
        time.sleep(0.2)

        for step, angle in enumerate(down_angles, start=1):
            scan_slice = get_one_slice(ser, events, angle)
            if scan_slice is None:
                print(f"step={step:02d} angle={angle:6.1f} no slice")
                continue

            valid = [
                p for p in scan_slice.points
                if 20 <= p[1] <= 12000
            ]
            print(
                f"step={step:02d} angle={angle:6.1f} "
                f"tilt={scan_slice.tilt_deg:6.1f} raw={len(scan_slice.points)} valid={len(valid)}"
            )
    finally:
        send_frame(ser, CMD_SCAN_STOP)
        time.sleep(0.2)
        send_frame(ser, CMD_SERVO_ANGLE, struct.pack("<f", 0.0))
        stop.set()
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()
```

## 8. 実装時の注意

- USBテキストデバッグモード (`USB_TEXT_DEBUG_MODE=1`) のファームでは、バイナリ応答が抑止されます。通常運用では `0` にしてください。
- `0x12` を連続送信する場合、前回スキャンの `0x05 state=2` を待ってから次を送るのが安全です。
- 本番高速検出では、スライスごとの点数が一定ではありません。`count=40` 程度のときも、`count=200` 以上のときもあります。
- 未受信が単発で発生しても、次ステップで復帰することがあります。障害物検出では複数周の確率的な集計を推奨します。
- 終了時は必ず `0x13` stop と `0x10` 0度を送って、LiDAR停止とサーボ中立復帰を行ってください。
- `0x04` の点群はLiDARローカル極座標です。3D変換が必要な場合は、`tilt_deg` とIMU `pitch/roll` を使って座標変換してください。

## 9. 動作確認済みコマンド

下方モード24ステップを10回、合計240ステップ実行した実測:

```text
ok=237
fail=3
raw_total=31673
accepted_total=25799
elapsed=103.13s
```

取得率:

```text
237 / 240 = 98.75%
```
