"""
PR2040_MotorDriver - Status Monitor
Continuously reads RESP_STATUS and MCP3208 external ADC packets.

Usage:
    python monitor.py [COM_PORT]
    python monitor.py          # auto-detect via VID_2E8A

Protocol:
    Packet: [0xAA][CMD][LEN][DATA x LEN][CHECKSUM]
    CHECKSUM = XOR(CMD, LEN, DATA[0..LEN-1])

RESP_STATUS (0x91), 44 bytes:
    [0..15]  int32 x4  encoder counts
    [16..31] int32 x4  velocities (counts/sec)
    [32..39] int16 x4  internal ADC raw (unused for current)
    [40..43] uint32    timestamp (ms)

RESP_EXT_ADC (0x92), 16 bytes:
    int16 x8  MCP3208 ch0..7
    ch0-3: Motor1-4 current (INA213, gain=50, Rshunt=10mΩ)
    ch4:   Battery voltage (divider 100kΩ/10kΩ, ratio=11)
"""

import sys
import struct
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# --------------------------------------------------------------------------- #
# Protocol constants (must match Config.h)
# --------------------------------------------------------------------------- #
PACKET_HEADER       = 0xAA
CMD_REQUEST_EXT_ADC = 0x12
CMD_REQUEST_IMU     = 0x13
RESP_STATUS         = 0x91
RESP_EXT_ADC        = 0x92
RESP_IMU            = 0x93
RESP_NAK            = 0x81
STATUS_DATA_LEN     = 44
EXT_ADC_DATA_LEN    = 16   # int16 x8
IMU_DATA_LEN        = 40   # float x9 + uint32

# --------------------------------------------------------------------------- #
# Physical conversion factors (must match SensorConverter.h)
# --------------------------------------------------------------------------- #
# Wheel / encoder
WHEEL_DIAMETER_MM      = 67.5
WHEEL_CIRCUMFERENCE_MM = 3.14159265 * WHEEL_DIAMETER_MM  # ≈ 212.06 mm
ENCODER_PPR            = 11.0
ENCODER_GEAR_RATIO     = 18.8
ENCODER_CPR            = ENCODER_PPR * 4.0 * ENCODER_GEAR_RATIO  # ≈ 827.2
CPS_TO_MMPS            = WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR    # ≈ 0.2564 mm/count

# MCP3208 — INA213 current sense (ch0-3)
#   REF pin: 3.3V divided by 20kΩ/10kΩ → V_ref = 1.1 V  (0A offset)
#   I [A] = (count - CURRENT_OFFSET_COUNTS) × CURRENT_SCALE
#   CURRENT_OFFSET_COUNTS = 1.1 × 4096 / 3.3 ≈ 1365.33  (ADC count at 0 A)
#   CURRENT_SCALE = 3.3 / (4096 × 50 × 0.010) ≈ 1.611e-3 A/count
ADC_VREF               = 3.3
ADC_COUNTS             = 4096
INA213_GAIN            = 50.0
SHUNT_OHMS             = 0.010
INA213_REF_R_UPPER     = 20000.0   # 20 kΩ
INA213_REF_R_LOWER     = 10000.0   # 10 kΩ
INA213_REF_VOLTAGE     = ADC_VREF * INA213_REF_R_LOWER / (INA213_REF_R_UPPER + INA213_REF_R_LOWER)  # 1.1 V
CURRENT_OFFSET_COUNTS  = INA213_REF_VOLTAGE * ADC_COUNTS / ADC_VREF   # ≈ 1365.33
CURRENT_SCALE          = ADC_VREF / (ADC_COUNTS * INA213_GAIN * SHUNT_OHMS)  # ≈ 1.611e-3 A/count

# Differential drive configuration (must match Config.h)
WHEEL_BASE_MM   = 205.0  # center-to-center wheel spacing [mm]
BODY_LEFT_IDX   = 0      # M1 = left wheel (0-based index into enc/vel arrays)
BODY_RIGHT_IDX  = 1      # M2 = right wheel
BODY_LEFT_DIR   =  1.0   # +1 or -1: mounting direction
BODY_RIGHT_DIR  = -1.0

# MCP3208 — Battery voltage (ch4)
#   V [V] = count × Vref × (1 + Rupper/Rlower) / ADC_counts
#         = count × 3.3 × 11 / 4096  ≈ count × 8.862e-3
VDIV_RATIO      = 11.0   # (100kΩ + 10kΩ) / 10kΩ
VOLTAGE_SCALE   = ADC_VREF * VDIV_RATIO / ADC_COUNTS                   # ≈ 8.862e-3 V/count

# --------------------------------------------------------------------------- #
# COM port auto-detection
# --------------------------------------------------------------------------- #
RP2040_VID = 0x2E8A

def find_rp2040_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == RP2040_VID:
            return port.device
    return None

# --------------------------------------------------------------------------- #
# Packet builder / parser
# --------------------------------------------------------------------------- #
def make_packet(cmd, data: bytes = b'') -> bytes:
    length = len(data)
    cs = cmd ^ length
    for b in data:
        cs ^= b
    return bytes([PACKET_HEADER, cmd, length]) + data + bytes([cs])

def parse_status(data):
    enc = struct.unpack_from('<4i', data, 0)
    vel = struct.unpack_from('<4i', data, 16)
    ts  = struct.unpack_from('<I',  data, 40)[0]
    return {'enc': enc, 'vel': vel, 'ts': ts}

def parse_ext_adc(data):
    """Return list of 8 int16 MCP3208 values."""
    return list(struct.unpack_from('<8h', data, 0))

class PacketReader:
    """State-machine based binary packet reader."""
    _S_HEADER, _S_CMD, _S_LEN, _S_DATA, _S_CS = range(5)

    def __init__(self):
        self._state = self._S_HEADER
        self._cmd = self._len = 0
        self._data = bytearray()
        self._packets = []

    def feed(self, raw_bytes):
        for b in raw_bytes:
            self._step(b)
        out = self._packets[:]
        self._packets.clear()
        return out

    def _step(self, b):
        if self._state == self._S_HEADER:
            if b == PACKET_HEADER:
                self._state = self._S_CMD
        elif self._state == self._S_CMD:
            self._cmd, self._state = b, self._S_LEN
        elif self._state == self._S_LEN:
            self._len  = b
            self._data = bytearray()
            self._state = self._S_CS if b == 0 else self._S_DATA
        elif self._state == self._S_DATA:
            self._data.append(b)
            if len(self._data) == self._len:
                self._state = self._S_CS
        elif self._state == self._S_CS:
            cs = self._cmd ^ self._len
            for x in self._data:
                cs ^= x
            if b == cs:
                self._packets.append((self._cmd, bytes(self._data)))
            self._state = self._S_HEADER

# --------------------------------------------------------------------------- #
# Display
# --------------------------------------------------------------------------- #
CLEAR = "\033[2J\033[H"
BOLD  = "\033[1m"
RST   = "\033[0m"

def display(status, ext_adc, imu_data, imu_ok, pkt_count, fps):
    enc  = status['enc']
    vel  = status['vel']
    ts   = status['ts']
    mmps = [v * CPS_TO_MMPS for v in vel]

    # Motor current from MCP3208 ch0-3 (with 0A offset correction)
    curr = [(ext_adc[i] - CURRENT_OFFSET_COUNTS) * CURRENT_SCALE for i in range(4)]
    # Battery voltage from MCP3208 ch4
    vbat = ext_adc[4] * VOLTAGE_SCALE

    print(CLEAR, end='')
    print(f"{BOLD}PR2040_MotorDriver  Status Monitor{RST}   "
          f"ts={ts/1000:.2f}s  pkts={pkt_count}  {fps:.1f} Hz")
    print(f"Battery: {vbat:.2f} V   (MCP3208 ch4={ext_adc[4]})")

    # Body velocity (estimated from encoders)
    v_left  = vel[BODY_LEFT_IDX]  * CPS_TO_MMPS * BODY_LEFT_DIR
    v_right = vel[BODY_RIGHT_IDX] * CPS_TO_MMPS * BODY_RIGHT_DIR
    body_v   = (v_left + v_right) * 0.5
    body_w   = (v_right - v_left) / WHEEL_BASE_MM * (180.0 / 3.14159265)
    print(f"Body  Linear={body_v:+7.1f} mm/s   Angular={body_w:+7.2f} °/s"
          f"   (L={v_left:+6.1f}  R={v_right:+6.1f} mm/s)")

    # IMU attitude
    if imu_data:
        az, roll, pitch, yaw = imu_data[2], imu_data[6], imu_data[7], imu_data[8]
        print(f"IMU  Roll={roll:+7.2f}°  Pitch={pitch:+7.2f}°  Yaw={yaw:+7.2f}°"
              f"   Accel Z={az:+5.3f}g")
    elif imu_ok is False:
        print("IMU  [NOT CONNECTED — retrying every 2s]")
    else:
        print("IMU  [waiting...]")

    print("─" * 68)
    print(f"  {'':3}  {'ENC [counts]':>13} {'VEL [cps]':>10} {'VEL [mm/s]':>11}"
          f"  {'MCP3208':>7} {'CURR [A]':>9}")
    print("─" * 68)
    for i in range(4):
        print(f"  M{i+1}   {enc[i]:>13d} {vel[i]:>10d} {mmps[i]:>11.1f}"
              f"  {ext_adc[i]:>7d} {curr[i]:>9.3f}")
    print("─" * 68)
    print("Ctrl+C to exit")

# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main():
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    else:
        port = find_rp2040_port()
        if port is None:
            print("RP2040 not found. Specify COM port as argument: python monitor.py COM30")
            sys.exit(1)
        print(f"Auto-detected RP2040 on {port}")

    print(f"Opening {port} at 115200 baud...")
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=0.05)
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Make sure no other app (e.g. VS Code Serial Monitor) is using the port.")
        sys.exit(1)

    reader    = PacketReader()
    pkt_count = 0
    last_stat = None
    ext_adc   = [0] * 8          # MCP3208 ch0..7 (last received)
    imu_data  = None              # (ax,ay,az,gx,gy,gz,roll,pitch,yaw) last received
    imu_ok    = None              # True/False/None(unknown)
    fps_count = 0
    fps_t0    = time.monotonic()
    fps       = 0.0
    last_ext_req = 0.0            # time of last CMD_REQUEST_EXT_ADC
    last_imu_req = 0.0            # time of last CMD_REQUEST_IMU

    print("Waiting for data...")
    try:
        while True:
            now = time.monotonic()

            # Request MCP3208 data at ~10 Hz
            if now - last_ext_req >= 0.1:
                ser.write(make_packet(CMD_REQUEST_EXT_ADC))
                last_ext_req = now

            # Request IMU data at ~10 Hz
            if now - last_imu_req >= 0.1:
                ser.write(make_packet(CMD_REQUEST_IMU))
                last_imu_req = now

            raw = ser.read(256)
            if raw:
                for cmd, data in reader.feed(raw):
                    if cmd == RESP_STATUS and len(data) == STATUS_DATA_LEN:
                        last_stat  = parse_status(data)
                        pkt_count += 1
                        fps_count += 1
                    elif cmd == RESP_EXT_ADC and len(data) == EXT_ADC_DATA_LEN:
                        ext_adc = parse_ext_adc(data)
                    elif cmd == RESP_IMU and len(data) == IMU_DATA_LEN:
                        vals = struct.unpack_from('<9f', data, 0)
                        imu_data = vals  # (ax,ay,az,gx,gy,gz,roll,pitch,yaw)
                        imu_ok = True
                    elif cmd == RESP_NAK and len(data) == 1 and data[0] == CMD_REQUEST_IMU:
                        imu_ok = False
                        imu_data = None

            if now - fps_t0 >= 1.0:
                fps       = fps_count / (now - fps_t0)
                fps_count = 0
                fps_t0    = now

            if last_stat:
                display(last_stat, ext_adc, imu_data, imu_ok, pkt_count, fps)

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
