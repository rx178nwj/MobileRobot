"""
PR2040_MotorDriver - Interactive Commander
Send commands to the RP2040 motor driver via USB Serial.

Usage:
    python commander.py [COM_PORT]
    python commander.py          # auto-detect via VID_2E8A

--- Differential Drive (body velocity) ---
    drive <mm/s> <deg/s>  - Set linear [mm/s] + angular [deg/s] (+ = CCW/left turn)
    fwd   <mm/s>          - Drive forward (omega=0)
    bwd   <mm/s>          - Drive backward (omega=0)
    spin  <deg/s>         - Spin in place (+ = CCW / left, - = CW / right)
    bstop                 - Coast both drive wheels

--- Individual motor control ---
    stop              - Coast all motors
    brake             - Active brake all motors
    reset             - Reset all encoders
    status            - Request and print one status packet
    ext               - Request and print external ADC (MCP3208)
    m <i> <duty>      - Set motor i (1-4) duty -1000..+1000
    mall <d1> <d2> <d3> <d4>  - Set all motors
    mode <i> <m>      - Set motor i mode: 0=DIRECT, 1=VEL, 2=POS
    modeall <m0..m3>  - Set all motors mode
    vel <i> <cps>     - Set motor i velocity target (counts/sec)
    velall <c1..c4>   - Set all velocity targets
    velmm <i> <mmps>  - Set motor i velocity target (mm/sec)
    pos <i> <counts>  - Set motor i position target (counts)
    posmm <i> <mm>    - Set motor i position target (mm)
    pid <i> <kp> <ki> <kd>    - Set velocity PID gains for motor i
    posgain <i> <kp> <maxcps> - Set position gains for motor i

--- IMU ---
    imu               - Request and print IMU data (accel/gyro/roll/pitch/yaw)
    calibrate         - Calibrate IMU (keep board level & still, ~2s)

--- Other ---
    mon               - Start continuous monitor (Ctrl+C to stop)
    q / quit / exit   - Quit
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
PACKET_HEADER = 0xAA

CMD_SET_MOTORS       = 0x01
CMD_STOP_ALL         = 0x02
CMD_BRAKE_ALL        = 0x03
CMD_SET_MOTOR_SINGLE = 0x04
CMD_REQUEST_STATUS   = 0x10
CMD_RESET_ENCODERS   = 0x11
CMD_REQUEST_EXT_ADC  = 0x12
CMD_REQUEST_IMU      = 0x13
CMD_CALIBRATE_IMU    = 0x14
CMD_SET_MODE_ALL     = 0x20
CMD_SET_MODE_SINGLE  = 0x21
CMD_SET_VEL_ALL      = 0x22
CMD_SET_VEL_SINGLE   = 0x23
CMD_SET_POS_ALL      = 0x24
CMD_SET_POS_SINGLE   = 0x25
CMD_SET_VEL_PID      = 0x26
CMD_SET_POS_GAINS    = 0x27
CMD_SET_BODY_VEL     = 0x30
CMD_BODY_STOP        = 0x31

RESP_STATUS     = 0x91
RESP_EXT_ADC    = 0x92
RESP_IMU        = 0x93
RESP_ACK        = 0x80
RESP_NAK        = 0x81
STATUS_DATA_LEN = 44
IMU_DATA_LEN    = 40  # float x9 + uint32

# Physical conversion
WHEEL_DIAMETER_MM      = 67.5
WHEEL_CIRCUMFERENCE_MM = 3.14159265 * WHEEL_DIAMETER_MM
ENCODER_PPR            = 11.0
ENCODER_GEAR_RATIO     = 18.8
ENCODER_CPR            = ENCODER_PPR * 4.0 * ENCODER_GEAR_RATIO
CPS_TO_MMPS            = WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR

# Differential drive configuration (must match Config.h)
WHEEL_BASE_MM   = 205.0   # center-to-center wheel spacing [mm]
BODY_LEFT_IDX   = 0       # M1 = left wheel (0-based)
BODY_RIGHT_IDX  = 1       # M2 = right wheel (0-based)
BODY_LEFT_DIR   =  1.0    # +1 or -1: mounting direction of left wheel
BODY_RIGHT_DIR  = -1.0    # +1 or -1: mounting direction of right wheel

# MCP3208 — INA213 current sense (ch0-3)
#   REF pin: 3.3V divided by 20kΩ/10kΩ → V_ref = 1.1 V  (0A offset)
#   I [A] = (count - CURRENT_OFFSET_COUNTS) × CURRENT_SCALE
INA213_REF_VOLTAGE    = 3.3 * 10000.0 / (20000.0 + 10000.0)  # 1.1 V
CURRENT_OFFSET_COUNTS = INA213_REF_VOLTAGE * 4096 / 3.3       # ≈ 1365.33 count
CURRENT_SCALE         = 3.3 / (4096 * 50.0 * 0.010)           # ≈ 1.611e-3 A/count
# MCP3208 — Battery voltage (ch4): V[V] = count × 3.3 × 11 / 4096
VOLTAGE_SCALE = 3.3 * 11.0 / 4096                             # ≈ 8.862e-3 V/count

RP2040_VID = 0x2E8A

# --------------------------------------------------------------------------- #
# Packet builder
# --------------------------------------------------------------------------- #
def make_packet(cmd, data: bytes = b'') -> bytes:
    length = len(data)
    cs = cmd ^ length
    for b in data:
        cs ^= b
    return bytes([PACKET_HEADER, cmd, length]) + data + bytes([cs])

# --------------------------------------------------------------------------- #
# Packet reader (state machine)
# --------------------------------------------------------------------------- #
class PacketReader:
    STATE_HEADER, STATE_CMD, STATE_LEN, STATE_DATA, STATE_CS = range(5)

    def __init__(self):
        self._state = self.STATE_HEADER
        self._cmd = self._len = 0
        self._data = bytearray()
        self._packets = []

    def feed(self, raw):
        for b in raw:
            self._step(b)
        out = self._packets[:]
        self._packets.clear()
        return out

    def _step(self, b):
        if self._state == self.STATE_HEADER:
            if b == PACKET_HEADER:
                self._state = self.STATE_CMD
        elif self._state == self.STATE_CMD:
            self._cmd, self._state = b, self.STATE_LEN
        elif self._state == self.STATE_LEN:
            self._len  = b
            self._data = bytearray()
            self._state = self.STATE_CS if b == 0 else self.STATE_DATA
        elif self._state == self.STATE_DATA:
            self._data.append(b)
            if len(self._data) == self._len:
                self._state = self.STATE_CS
        elif self._state == self.STATE_CS:
            cs = self._cmd ^ self._len
            for x in self._data:
                cs ^= x
            if b == cs:
                self._packets.append((self._cmd, bytes(self._data)))
            self._state = self.STATE_HEADER

# --------------------------------------------------------------------------- #
# COM port auto-detection
# --------------------------------------------------------------------------- #
def find_rp2040_port():
    for port in serial.tools.list_ports.comports():
        if port.vid == RP2040_VID:
            return port.device
    return None

# --------------------------------------------------------------------------- #
# Status display
# --------------------------------------------------------------------------- #
def print_status(data):
    """Print RESP_STATUS. Current values require a separate EXT_ADC request."""
    if len(data) < STATUS_DATA_LEN:
        print(f"[NAK] Short status packet ({len(data)} bytes)")
        return
    enc = struct.unpack_from('<4i', data, 0)
    vel = struct.unpack_from('<4i', data, 16)
    ts  = struct.unpack_from('<I',  data, 40)[0]
    print(f"\n── Status  ts={ts/1000:.3f}s ─────────────────────────────────")
    print(f"  {'':3} {'ENC [cnt]':>12} {'VEL [cps]':>10} {'VEL [mm/s]':>11}")
    for i in range(4):
        mmps = vel[i] * CPS_TO_MMPS
        print(f"  M{i+1}  {enc[i]:>12d} {vel[i]:>10d} {mmps:>11.1f}")
    print("  (use 'ext' for motor current and battery voltage)")
    print()

def print_ext_adc(data):
    if len(data) < 16:
        print(f"[NAK] Short EXT_ADC packet ({len(data)} bytes)")
        return
    ch = struct.unpack_from('<8h', data, 0)
    print(f"\n── MCP3208 External ADC ────────────────────────────────────")
    for i in range(4):
        curr = (ch[i] - CURRENT_OFFSET_COUNTS) * CURRENT_SCALE
        print(f"  CH{i} (Motor{i+1} current): {ch[i]:5d}  {curr:.3f} A")
    vbat = ch[4] * VOLTAGE_SCALE
    print(f"  CH4 (Battery voltage):   {ch[4]:5d}  {vbat:.2f} V")
    for i in range(5, 8):
        print(f"  CH{i} (unused):           {ch[i]:5d}")
    print()

def print_imu(data):
    if len(data) < IMU_DATA_LEN:
        print(f"[NAK] Short IMU packet ({len(data)} bytes)")
        return
    # float x9: accelX,Y,Z [g], gyroX,Y,Z [dps], roll,pitch,yaw [°]
    ax, ay, az, gx, gy, gz, roll, pitch, yaw = struct.unpack_from('<9f', data, 0)
    ts = struct.unpack_from('<I', data, 36)[0]
    print(f"\n── MPU-6881 IMU  ts={ts/1000:.3f}s ─────────────────────────────")
    print(f"  Accel  X={ax:+7.4f} g     Y={ay:+7.4f} g     Z={az:+7.4f} g")
    print(f"  Gyro   X={gx:+8.3f} °/s  Y={gy:+8.3f} °/s  Z={gz:+8.3f} °/s")
    print(f"  Roll={roll:+7.2f}°  Pitch={pitch:+7.2f}°  Yaw={yaw:+7.2f}°")
    print()

# --------------------------------------------------------------------------- #
# Command dispatch
# --------------------------------------------------------------------------- #
def send_recv(ser, reader, pkt, wait_response=True, timeout=0.5, expected=None):
    """Send packet and wait for response. If expected is set, discard other packets."""
    ser.write(pkt)
    if not wait_response:
        return None
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        raw = ser.read(256)
        if raw:
            for p in reader.feed(raw):
                if expected is None or p[0] == expected:
                    return p
        time.sleep(0.01)
    return None

def parse_index(token, name="index"):
    idx = int(token)
    if idx < 1 or idx > 4:
        raise ValueError(f"{name} must be 1..4")
    return idx - 1  # convert to 0-based

def handle_command(line, ser, reader):
    parts = line.strip().split()
    if not parts:
        return True
    cmd = parts[0].lower()

    try:
        if cmd in ('q', 'quit', 'exit'):
            return False

        # --- Differential drive body velocity ---

        elif cmd == 'drive':
            if len(parts) < 3:
                print("Usage: drive <linear mm/s> <omega deg/s>  (+omega = CCW / left turn)")
                return True
            linear = float(parts[1])
            omega  = float(parts[2])
            data = struct.pack('<ff', linear, omega)
            ser.write(make_packet(CMD_SET_BODY_VEL, data))
            print(f"[OK] Body vel: linear={linear:.1f} mm/s  omega={omega:+.1f} °/s")

        elif cmd == 'fwd':
            speed = float(parts[1]) if len(parts) >= 2 else 100.0
            data = struct.pack('<ff', speed, 0.0)
            ser.write(make_packet(CMD_SET_BODY_VEL, data))
            print(f"[OK] Forward {speed:.1f} mm/s")

        elif cmd == 'bwd':
            speed = float(parts[1]) if len(parts) >= 2 else 100.0
            data = struct.pack('<ff', -speed, 0.0)
            ser.write(make_packet(CMD_SET_BODY_VEL, data))
            print(f"[OK] Backward {speed:.1f} mm/s")

        elif cmd == 'spin':
            if len(parts) < 2:
                print("Usage: spin <deg/s>  (+ = CCW/left,  - = CW/right)")
                return True
            omega = float(parts[1])
            data = struct.pack('<ff', 0.0, omega)
            ser.write(make_packet(CMD_SET_BODY_VEL, data))
            dir_str = "CCW (left)" if omega >= 0 else "CW (right)"
            print(f"[OK] Spin {omega:+.1f} °/s  {dir_str}")

        elif cmd == 'bstop':
            ser.write(make_packet(CMD_BODY_STOP))
            print("[OK] Drive wheels stopped")

        # --- Individual motor control ---

        elif cmd == 'stop':
            ser.write(make_packet(CMD_STOP_ALL))
            print("[OK] Coast all motors")

        elif cmd == 'brake':
            ser.write(make_packet(CMD_BRAKE_ALL))
            print("[OK] Brake all motors")

        elif cmd == 'reset':
            ser.write(make_packet(CMD_RESET_ENCODERS))
            print("[OK] Encoders reset")

        elif cmd == 'status':
            pkt = make_packet(CMD_REQUEST_STATUS)
            resp = send_recv(ser, reader, pkt, expected=RESP_STATUS)
            if resp:
                print_status(resp[1])
            else:
                print("[TIMEOUT] No status response")

        elif cmd == 'ext':
            pkt = make_packet(CMD_REQUEST_EXT_ADC)
            resp = send_recv(ser, reader, pkt, expected=RESP_EXT_ADC)
            if resp:
                print_ext_adc(resp[1])
            else:
                print("[TIMEOUT] No EXT_ADC response")

        elif cmd == 'imu':
            pkt = make_packet(CMD_REQUEST_IMU)
            resp = send_recv(ser, reader, pkt, expected=RESP_IMU, timeout=1.0)
            if resp:
                print_imu(resp[1])
            elif resp is None:
                print("[TIMEOUT] No IMU response (IMU not connected?)")

        elif cmd == 'calibrate':
            print("Keep IMU level and still. Calibrating (~5s)...")
            pkt = make_packet(CMD_CALIBRATE_IMU)
            resp = send_recv(ser, reader, pkt, timeout=10.0, expected=RESP_ACK)
            if resp:
                print("[OK] IMU calibration complete. Bias saved to flash. Madgwick filter reset.")
            else:
                print("[TIMEOUT/NAK] Calibration may have failed.")

        elif cmd == 'm':
            if len(parts) < 3:
                print("Usage: m <index 1-4> <duty -1000..1000>")
                return True
            idx  = parse_index(parts[1])
            duty = max(-1000, min(1000, int(parts[2])))
            data = struct.pack('<Bh', idx, duty)
            ser.write(make_packet(CMD_SET_MOTOR_SINGLE, data))
            print(f"[OK] Motor {idx+1} duty={duty}")

        elif cmd == 'mall':
            if len(parts) < 5:
                print("Usage: mall <d1> <d2> <d3> <d4>")
                return True
            duties = [max(-1000, min(1000, int(x))) for x in parts[1:5]]
            data = struct.pack('<4h', *duties)
            ser.write(make_packet(CMD_SET_MOTORS, data))
            print(f"[OK] All motors duty={duties}")

        elif cmd == 'mode':
            if len(parts) < 3:
                print("Usage: mode <index 1-4> <mode: 0=DIRECT 1=VEL 2=POS>")
                return True
            idx  = parse_index(parts[1])
            mode = int(parts[2])
            data = struct.pack('BB', idx, mode)
            ser.write(make_packet(CMD_SET_MODE_SINGLE, data))
            names = {0: 'DIRECT', 1: 'VEL', 2: 'POS'}
            print(f"[OK] Motor {idx+1} mode={names.get(mode, mode)}")

        elif cmd == 'modeall':
            if len(parts) < 5:
                print("Usage: modeall <m0> <m1> <m2> <m3>")
                return True
            modes = [int(x) for x in parts[1:5]]
            data  = struct.pack('4B', *modes)
            ser.write(make_packet(CMD_SET_MODE_ALL, data))
            print(f"[OK] All modes={modes}")

        elif cmd == 'vel':
            if len(parts) < 3:
                print("Usage: vel <index 1-4> <counts/sec>")
                return True
            idx = parse_index(parts[1])
            cps = float(parts[2])
            data = struct.pack('<Bf', idx, cps)
            ser.write(make_packet(CMD_SET_VEL_SINGLE, data))
            print(f"[OK] Motor {idx+1} vel={cps:.1f} cps ({cps * CPS_TO_MMPS:.1f} mm/s)")

        elif cmd == 'velall':
            if len(parts) < 5:
                print("Usage: velall <c1> <c2> <c3> <c4>  (counts/sec)")
                return True
            vals = [float(x) for x in parts[1:5]]
            data = struct.pack('<4f', *vals)
            ser.write(make_packet(CMD_SET_VEL_ALL, data))
            print(f"[OK] All vel={vals} cps")

        elif cmd == 'velmm':
            if len(parts) < 3:
                print("Usage: velmm <index 1-4> <mm/sec>")
                return True
            idx  = parse_index(parts[1])
            mmps = float(parts[2])
            cps  = mmps / CPS_TO_MMPS
            data = struct.pack('<Bf', idx, cps)
            ser.write(make_packet(CMD_SET_VEL_SINGLE, data))
            print(f"[OK] Motor {idx+1} vel={mmps:.1f} mm/s ({cps:.1f} cps)")

        elif cmd == 'pos':
            if len(parts) < 3:
                print("Usage: pos <index 1-4> <counts>")
                return True
            idx    = parse_index(parts[1])
            counts = int(parts[2])
            data   = struct.pack('<Bi', idx, counts)
            ser.write(make_packet(CMD_SET_POS_SINGLE, data))
            print(f"[OK] Motor {idx+1} pos={counts} counts")

        elif cmd == 'posmm':
            if len(parts) < 3:
                print("Usage: posmm <index 1-4> <mm>")
                return True
            idx    = parse_index(parts[1])
            mm     = float(parts[2])
            counts = int(mm / (WHEEL_CIRCUMFERENCE_MM / ENCODER_CPR))
            data   = struct.pack('<Bi', idx, counts)
            ser.write(make_packet(CMD_SET_POS_SINGLE, data))
            print(f"[OK] Motor {idx+1} pos={mm:.1f} mm ({counts} counts)")

        elif cmd == 'pid':
            if len(parts) < 5:
                print("Usage: pid <index 1-4> <kp> <ki> <kd>")
                return True
            idx = parse_index(parts[1])
            kp, ki, kd = float(parts[2]), float(parts[3]), float(parts[4])
            data = struct.pack('<Bfff', idx, kp, ki, kd)
            ser.write(make_packet(CMD_SET_VEL_PID, data))
            print(f"[OK] Motor {idx+1} PID kp={kp} ki={ki} kd={kd}")

        elif cmd == 'posgain':
            if len(parts) < 4:
                print("Usage: posgain <index 1-4> <posKp> <maxVelCps>")
                return True
            idx    = parse_index(parts[1])
            posKp  = float(parts[2])
            maxCps = float(parts[3])
            data   = struct.pack('<Bff', idx, posKp, maxCps)
            ser.write(make_packet(CMD_SET_POS_GAINS, data))
            print(f"[OK] Motor {idx+1} posKp={posKp} maxCps={maxCps}")

        elif cmd == 'mon':
            continuous_monitor(ser, reader)

        else:
            print(f"Unknown command: '{cmd}'. Type 'help' for list.")
            print(__doc__)

    except (ValueError, IndexError) as e:
        print(f"[Error] {e}")

    return True

# --------------------------------------------------------------------------- #
# Continuous monitor mode
# --------------------------------------------------------------------------- #
def continuous_monitor(ser, reader):
    CLEAR = "\033[2J\033[H"
    print("Continuous monitor — Ctrl+C to stop")
    last_status  = None
    ext_adc      = [0] * 8
    pkt_count    = 0
    fps_count    = 0
    fps          = 0.0
    t0           = time.monotonic()
    last_ext_req = 0.0

    try:
        while True:
            now = time.monotonic()

            # Request MCP3208 data at ~10 Hz
            if now - last_ext_req >= 0.1:
                ser.write(make_packet(CMD_REQUEST_EXT_ADC))
                last_ext_req = now

            raw = ser.read(256)
            if raw:
                for cmd, data in reader.feed(raw):
                    if cmd == RESP_STATUS and len(data) == STATUS_DATA_LEN:
                        last_status = data
                        pkt_count  += 1
                        fps_count  += 1
                    elif cmd == RESP_EXT_ADC and len(data) == 16:
                        ext_adc = list(struct.unpack_from('<8h', data, 0))

            if now - t0 >= 1.0:
                fps       = fps_count / (now - t0)
                fps_count = 0
                t0        = now

            if last_status:
                enc  = struct.unpack_from('<4i', last_status, 0)
                vel  = struct.unpack_from('<4i', last_status, 16)
                ts   = struct.unpack_from('<I',  last_status, 40)[0]
                vbat = ext_adc[4] * VOLTAGE_SCALE
                print(CLEAR, end='')
                print(f"Continuous Monitor   ts={ts/1000:.2f}s  pkts={pkt_count}  {fps:.1f} Hz")
                print(f"Battery: {vbat:.2f} V")
                print("─" * 68)
                print(f"  {'':3}  {'ENC [cnt]':>12} {'VEL [cps]':>10} {'mm/s':>8}"
                      f"  {'MCP3208':>7} {'CURR [A]':>9}")
                print("─" * 68)
                for i in range(4):
                    curr = (ext_adc[i] - CURRENT_OFFSET_COUNTS) * CURRENT_SCALE
                    print(f"  M{i+1}   {enc[i]:>12d} {vel[i]:>10d} "
                          f"{vel[i]*CPS_TO_MMPS:>8.1f}"
                          f"  {ext_adc[i]:>7d} {curr:>9.3f}")
                print("─" * 68)
                print("Ctrl+C to return to prompt")

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nReturning to commander prompt.")

# --------------------------------------------------------------------------- #
# Main
# --------------------------------------------------------------------------- #
def main():
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    else:
        port = find_rp2040_port()
        if port is None:
            print("RP2040 not found. Specify port: python commander.py COM30")
            sys.exit(1)
        print(f"Auto-detected RP2040 on {port}")

    print(f"Opening {port} at 115200 baud...")
    try:
        ser = serial.Serial(port, baudrate=115200, timeout=0.05)
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Make sure no other app (e.g. VS Code Serial Monitor) is using the port.")
        sys.exit(1)

    reader = PacketReader()
    print("PR2040_MotorDriver Commander ready.")
    print("Type a command or 'help' / 'q' to quit.\n")

    try:
        while True:
            try:
                line = input("cmd> ")
            except EOFError:
                break
            if line.strip().lower() in ('help', '?', 'h'):
                print(__doc__)
                continue
            if not handle_command(line, ser, reader):
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("Stopping all motors before exit...")
        ser.write(make_packet(CMD_STOP_ALL))
        time.sleep(0.1)
        ser.close()
        print("Disconnected.")

if __name__ == '__main__':
    main()
