# LiderModule

LiderModule is a low-cost 3D LiDAR scanner module for a mobile robot. It tilts a 2D YDLIDAR T-mini Pro with a FEETECH STS3215 servo, reads MPU6050 attitude data on the ESP32-C3, and sends scan slices to a Raspberry Pi over USB CDC.

## Implementation Status

Implemented:

- ESP32-C3 Arduino firmware in `firmware/lidar_tilt_3d/lidar_tilt_3d.ino`
- Raspberry Pi host CLI in `host/rpi_tilt_3d.py`
- Binary USB CDC framing compatible with `SPEC.md`
- IMU packets, 3D scan status packets, 3D scan slice packets
- Manual tilt command and automatic tilt scan command
- Point-cloud transform, CSV export, PLY export, and optional Open3D/matplotlib visualization

Needs hardware validation:

- Exact YDLIDAR T-mini Pro packet distance scaling and checksum behavior
- SCServo library API compatibility for the installed Arduino library version
- Servo direction and center position on the assembled mechanism
- Final IMU sign convention after physical mounting

The requirements are implementable, but those items depend on the actual hardware, library version, and mounting orientation.

## Layout

```text
LiderModule/
  SPEC.md
  README.md
  firmware/
    lidar_tilt_3d/
      lidar_tilt_3d.ino
  host/
    rpi_tilt_3d.py
    requirements.txt
    test_protocol.py
  docs/
    wiring.md
```

## Firmware Setup

1. Install Arduino IDE 2.x.
2. Select `Seeed Studio XIAO ESP32-C3`.
3. Enable `USB CDC On Boot`.
4. Install the `SCServo` library.
5. Open `firmware/lidar_tilt_3d/lidar_tilt_3d.ino`.
6. Upload to the XIAO ESP32-C3.

Important serial assignments:

- `Serial`: USB CDC to Raspberry Pi
- `Serial0`: STS3215 servo bus, 1 Mbps, GPIO21 RX / GPIO20 TX
- `LidarSerial`: YDLIDAR, 230400 bps, GPIO4 RX / GPIO5 TX

## Host Setup

```bash
cd LiderModule/host
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

Default scan:

```bash
python3 rpi_tilt_3d.py --port /dev/ttyACM0
```

Save a PLY point cloud:

```bash
python3 rpi_tilt_3d.py --tilt-min -45 --tilt-max 45 --step 2 --save scan.ply
```

Save CSV without visualization:

```bash
python3 rpi_tilt_3d.py --step 2 --save scan.csv --no-viz
```

Manual tilt test:

```bash
python3 rpi_tilt_3d.py --manual-tilt 30.0 --no-viz
```

## Protocol Summary

Frames use:

```text
[0xFE][0xEF][TYPE][LEN_L][LEN_H][PAYLOAD...][CHECKSUM]
```

The checksum is XOR of `TYPE`, `LEN_L`, `LEN_H`, and all payload bytes.

Device to host:

- `0x02`: IMU data, `<float pitch><float roll><float yaw>`
- `0x04`: scan slice, `<float tilt><float imu_pitch><float imu_roll><uint16 count><points...>`
- `0x05`: scan status, `<uint8 state><uint16 step><uint16 total>`

Host to device:

- `0x10`: manual servo angle, `<float angle_deg>`
- `0x12`: scan start, `<float tilt_min><float tilt_max><float step>`
- `0x13`: scan stop, empty payload

## Validation

Host-side syntax and protocol tests:

```bash
cd LiderModule/host
python -m py_compile rpi_tilt_3d.py test_protocol.py
python -m unittest test_protocol.py
```

Firmware validation must be completed in Arduino IDE or `arduino-cli` with the target board package and `SCServo` installed.
