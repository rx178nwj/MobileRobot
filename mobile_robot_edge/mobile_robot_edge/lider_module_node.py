#!/usr/bin/env python3
"""LiderModule ROS2 node for SLAM 2.5D."""

import math
import queue
import struct
import threading
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import serial

from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField

# ── Protocol constants ────────────────────────────────────────────────────────
SYNC1 = 0xFE
SYNC2 = 0xEF

MSG_IMU_DATA    = 0x02
MSG_SCAN_SLICE  = 0x04
MSG_SCAN_STATUS = 0x05
CMD_SERVO_ANGLE = 0x10
CMD_SCAN_START  = 0x12
CMD_SCAN_STOP   = 0x13

# 24-step downward scan pattern from USB_CDC_COMMAND_MANUAL.md §6
DOWN_ANGLES: List[float] = [
    0.0,   -15.0, -30.0, -45.0,
    -42.5, -27.5, -12.5,  0.0,
    -5.0,  -20.0, -35.0, -45.0,
    -37.5, -22.5,  -7.5,  0.0,
    -10.0, -25.0, -40.0, -45.0,
    -32.5, -17.5,  -2.5,  0.0,
]


# ── Data classes ──────────────────────────────────────────────────────────────
@dataclass
class ImuData:
    pitch: float
    roll: float
    yaw: float


@dataclass
class ScanSlice:
    tilt_deg: float
    tilt_start_deg: float
    tilt_end_deg: float
    imu_pitch: float
    imu_roll: float
    points: List[Tuple[float, int, int]]  # (angle_deg, dist_mm, quality)


@dataclass
class ScanStatus:
    state: int  # 0=started, 1=running, 2=complete
    step: int
    total: int


# ── Protocol helpers ──────────────────────────────────────────────────────────
def _checksum(msg_type: int, payload: bytes) -> int:
    n = len(payload)
    cs = msg_type ^ (n & 0xFF) ^ ((n >> 8) & 0xFF)
    for b in payload:
        cs ^= b
    return cs & 0xFF


def _encode_frame(msg_type: int, payload: bytes = b'') -> bytes:
    n = len(payload)
    return (
        bytes([SYNC1, SYNC2, msg_type, n & 0xFF, n >> 8])
        + payload
        + bytes([_checksum(msg_type, payload)])
    )


def _decode_payload(msg_type: int, payload: bytes):
    if msg_type == MSG_IMU_DATA and len(payload) >= 12:
        pitch, roll, yaw = struct.unpack_from('<fff', payload, 0)
        return ImuData(pitch, roll, yaw)

    if msg_type == MSG_SCAN_STATUS and len(payload) >= 5:
        state, step, total = struct.unpack('<BHH', payload)
        return ScanStatus(state, step, total)

    if msg_type == MSG_SCAN_SLICE and len(payload) >= 14:
        use_extended = False
        if len(payload) >= 22:
            ext_count = struct.unpack_from('<H', payload, 20)[0]
            use_extended = (22 + ext_count * 5) == len(payload)

        if use_extended:
            tilt_deg, tilt_start_deg, tilt_end_deg, imu_pitch, imu_roll, count = struct.unpack_from('<fffffH', payload, 0)
            header_size = 22
        else:
            tilt_deg, imu_pitch, imu_roll, count = struct.unpack_from('<fffH', payload, 0)
            tilt_start_deg = tilt_deg
            tilt_end_deg = tilt_deg
            header_size = 14
        raw = payload[header_size:]
        points: List[Tuple[float, int, int]] = []
        for i in range(count):
            if i * 5 + 5 > len(raw):
                break
            ax100, dist_mm, quality = struct.unpack_from('<HHB', raw, i * 5)
            points.append((ax100 / 100.0, dist_mm, quality))
        return ScanSlice(
            tilt_deg=tilt_deg,
            tilt_start_deg=tilt_start_deg,
            tilt_end_deg=tilt_end_deg,
            imu_pitch=imu_pitch,
            imu_roll=imu_roll,
            points=points,
        )

    return None


def _reader_loop(
    ser: serial.Serial,
    out: queue.Queue,
    stop: threading.Event,
) -> None:
    """Background thread: parse binary frames from serial and enqueue decoded objects."""
    state = 0
    msg_type = 0
    length = 0
    payload = bytearray()

    while not stop.is_set():
        try:
            chunk = ser.read(512)
        except Exception:
            break
        for b in chunk:
            if state == 0:
                state = 1 if b == SYNC1 else 0
            elif state == 1:
                if b == SYNC2:
                    state = 2
                elif b == SYNC1:
                    state = 1
                else:
                    state = 0
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
                if _checksum(msg_type, bytes(payload)) == b:
                    decoded = _decode_payload(msg_type, bytes(payload))
                    if decoded is not None:
                        out.put(decoded)
                state = 0


# ── ROS2 Node ─────────────────────────────────────────────────────────────────
class LiderModuleNode(Node):

    def __init__(self):
        super().__init__('lider_module_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('port',               '/dev/lider_module')
        self.declare_parameter('scan_frame_id',      'laser_frame')
        self.declare_parameter('imu_frame_id',       'imu_link')
        self.declare_parameter('min_range_m',        0.02)
        self.declare_parameter('max_range_m',        12.0)
        self.declare_parameter('laser_scan_bins',    720)   # 0.5° angular resolution
        self.declare_parameter('slice_timeout_s',    4.0)
        self.declare_parameter('pointcloud_interval_s', 2.0)   # 0 to disable
        self.declare_parameter('tilt_axis_offset_m', 0.023)    # sensor center above tilt axis
        self.declare_parameter('min_quality', 0)

        self._port         = self.get_parameter('port').value
        self._scan_frame   = self.get_parameter('scan_frame_id').value
        self._imu_frame    = self.get_parameter('imu_frame_id').value
        self._min_range    = self.get_parameter('min_range_m').value
        self._max_range    = self.get_parameter('max_range_m').value
        self._n_bins       = self.get_parameter('laser_scan_bins').value
        self._slice_timeout = self.get_parameter('slice_timeout_s').value
        self._pc_interval  = self.get_parameter('pointcloud_interval_s').value
        self._tilt_axis_offset_m = self.get_parameter('tilt_axis_offset_m').value
        self._min_quality = int(self.get_parameter('min_quality').value)

        # ── QoS ──────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._scan_pub = self.create_publisher(LaserScan,   '/scan',               sensor_qos)
        self._pc2_pub  = self.create_publisher(PointCloud2, '/lider/pointcloud2',  10)
        self._imu_pub  = self.create_publisher(Imu,         '/lider/imu',          sensor_qos)

        # ── Serial connection ─────────────────────────────────────────────
        try:
            self._ser = serial.Serial(
                self._port, 921600, timeout=0.05, write_timeout=1)
            self._ser.dtr = False
            self._ser.rts = False
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {self._port}: {e}')
            raise

        # ── Receiver thread ───────────────────────────────────────────────
        self._rx_queue: queue.Queue = queue.Queue()
        self._stop_event = threading.Event()
        self._rx_thread = threading.Thread(
            target=_reader_loop,
            args=(self._ser, self._rx_queue, self._stop_event),
            daemon=True,
        )
        self._rx_thread.start()

        # ── Scan state machine ────────────────────────────────────────────
        # States: 'init_stop' | 'init_zero' | 'idle' | 'waiting_2d' | '3d_scan' | '3d_wait'
        self._state              = 'init_stop'
        self._init_time          = time.monotonic()
        self._pending_slice: Optional[ScanSlice] = None
        self._slice_deadline     = 0.0
        self._3d_step            = 0
        self._3d_slices: List[ScanSlice] = []
        self._last_3d_time       = time.monotonic()

        # ── Main loop at 100 Hz ───────────────────────────────────────────
        self._timer = self.create_timer(0.01, self._loop)

        self.get_logger().info(
            f'LiderModuleNode started — port={self._port} '
            f'pc_interval={self._pc_interval}s')

    # ── Serial write ──────────────────────────────────────────────────────────
    def _send(self, msg_type: int, payload: bytes = b'') -> None:
        try:
            self._ser.write(_encode_frame(msg_type, payload))
            self._ser.flush()
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def _request_slice(self, angle: float) -> None:
        self._send(CMD_SCAN_START, struct.pack('<fff', angle, angle, 1.0))
        self._pending_slice = None
        self._slice_deadline = time.monotonic() + self._slice_timeout

    # ── Main loop (timer callback, non-blocking) ──────────────────────────────
    def _loop(self) -> None:
        self._drain_rx()
        self._step_state_machine()

    def _drain_rx(self) -> None:
        while True:
            try:
                msg = self._rx_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_msg(msg)

    def _handle_msg(self, msg) -> None:
        now_ros = self.get_clock().now().to_msg()

        if isinstance(msg, ImuData):
            self._publish_imu(msg, now_ros)

        elif isinstance(msg, ScanSlice):
            self._pending_slice = msg

        elif isinstance(msg, ScanStatus):
            if msg.state == 2:  # complete
                self._on_scan_complete()

    def _on_scan_complete(self) -> None:
        if self._state == 'waiting_2d':
            if self._pending_slice is not None:
                self._scan_pub.publish(
                    self._slice_to_laser_scan(self._pending_slice))
                self._pending_slice = None
            self._state = 'idle'

        elif self._state == '3d_wait':
            if self._pending_slice is not None:
                self._3d_slices.append(self._pending_slice)
                self._pending_slice = None
            self._3d_step += 1
            self._state = '3d_scan'  # advance in next loop tick

    def _step_state_machine(self) -> None:
        now = time.monotonic()

        if self._state == 'init_stop':
            self._send(CMD_SCAN_STOP)
            self._init_time = now
            self._state = 'init_zero'

        elif self._state == 'init_zero':
            if now - self._init_time >= 0.2:
                self._send(CMD_SERVO_ANGLE, struct.pack('<f', 0.0))
                self._init_time = now
                self._state = 'idle'

        elif self._state == 'idle':
            if now - self._init_time < 0.2:
                return
            # Check if 3D scan is due
            if self._pc_interval > 0 and now - self._last_3d_time >= self._pc_interval:
                self._start_3d()
            else:
                self._request_slice(0.0)
                self._state = 'waiting_2d'

        elif self._state == 'waiting_2d':
            if now > self._slice_deadline:
                self.get_logger().warn('2D slice timeout, retrying')
                self._request_slice(0.0)

        elif self._state == '3d_scan':
            if self._3d_step >= len(DOWN_ANGLES):
                self._finish_3d()
            else:
                self._request_slice(DOWN_ANGLES[self._3d_step])
                self._state = '3d_wait'

        elif self._state == '3d_wait':
            if now > self._slice_deadline:
                self.get_logger().warn(
                    f'3D step {self._3d_step} timeout, skipping')
                self._3d_step += 1
                self._state = '3d_scan'

    def _start_3d(self) -> None:
        self._3d_slices = []
        self._3d_step = 0
        self.get_logger().info('3D scan start')
        self._state = '3d_scan'

    def _finish_3d(self) -> None:
        self._publish_pointcloud()
        self._last_3d_time = time.monotonic()
        self._init_time = time.monotonic()
        self._state = 'idle'

    # ── IMU publisher ─────────────────────────────────────────────────────────
    def _publish_imu(self, data: ImuData, stamp) -> None:
        msg = Imu()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._imu_frame
        # MPU6050 DMP provides orientation; yaw is relative, not absolute
        msg.orientation_covariance[0]          = -1.0  # unknown orientation quat
        msg.angular_velocity_covariance[0]     = -1.0  # not provided
        msg.linear_acceleration_covariance[0]  = -1.0  # not provided
        self._imu_pub.publish(msg)

    # ── ScanSlice → LaserScan ─────────────────────────────────────────────────
    def _slice_to_laser_scan(self, sl: ScanSlice) -> LaserScan:
        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._scan_frame
        msg.angle_min       = -math.pi
        msg.angle_max       =  math.pi
        msg.angle_increment = 2.0 * math.pi / self._n_bins
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / 6.0   # T-mini Pro default ~6 Hz rotation
        msg.range_min       = self._min_range
        msg.range_max       = self._max_range

        ranges      = [float('inf')] * self._n_bins
        intensities = [0.0] * self._n_bins
        min_mm = int(self._min_range * 1000)
        max_mm = int(self._max_range * 1000)

        for angle_deg, dist_mm, quality in sl.points:
            if quality < self._min_quality:
                continue
            if not (min_mm <= dist_mm <= max_mm):
                continue
            # T-mini Pro reports 0–360°; convert to −180…+180 for LaserScan
            a = angle_deg if angle_deg <= 180.0 else angle_deg - 360.0
            idx = int((a + 180.0) / 360.0 * self._n_bins)
            idx = max(0, min(self._n_bins - 1, idx))
            r = dist_mm / 1000.0
            if r < ranges[idx]:
                ranges[idx] = r
                intensities[idx] = float(quality)

        msg.ranges      = ranges
        msg.intensities = intensities
        return msg

    # ── 3D slices → PointCloud2 ───────────────────────────────────────────────
    def _publish_pointcloud(self) -> None:
        if not self._3d_slices:
            return

        parts = [self._slice_to_3d(sl) for sl in self._3d_slices]
        points = np.vstack([p for p in parts if len(p)]).astype(np.float32)
        if len(points) == 0:
            return

        msg = PointCloud2()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._scan_frame
        msg.height      = 1
        msg.width       = len(points)
        msg.is_dense    = False
        msg.is_bigendian = False
        msg.point_step  = 12   # 3 × float32
        msg.row_step    = msg.point_step * msg.width
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = points.tobytes()
        self._pc2_pub.publish(msg)
        self.get_logger().info(
            f'PointCloud2: {len(points)} pts from {len(self._3d_slices)} slices')

    def _slice_to_3d(self, sl: ScanSlice) -> np.ndarray:
        """Convert a ScanSlice to Nx3 float64 array [meters].

        Coordinate system (SPEC.md §5.3):
          X = forward (tilt=0, angle=0°)
          Y = left
          Z = up
        """
        valid_points = [(a, d) for a, d, q in sl.points if q >= self._min_quality and 20 <= d <= 12000]
        if not valid_points:
            return np.zeros((0, 3), dtype=np.float32)

        count = len(valid_points)
        angles = np.array([math.radians(v[0]) for v in valid_points], dtype=np.float64)
        dists = np.array([v[1] / 1000.0 for v in valid_points], dtype=np.float64)
        tilts = np.linspace(
            math.radians(sl.tilt_start_deg),
            math.radians(sl.tilt_end_deg),
            count,
            dtype=np.float64,
        )

        x_l = dists * np.cos(angles)
        y_l = dists * np.sin(angles)

        X = x_l * np.cos(tilts) + self._tilt_axis_offset_m * np.sin(tilts)
        Y = y_l
        Z = -x_l * np.sin(tilts) + self._tilt_axis_offset_m * np.cos(tilts)

        pts = np.stack([X, Y, Z], axis=1)

        # Apply IMU correction for platform tilt (only when non-negligible)
        if abs(sl.imu_pitch) > 0.1 or abs(sl.imu_roll) > 0.1:
            p = math.radians(sl.imu_pitch)
            r = math.radians(sl.imu_roll)
            Ry = np.array([
                [ math.cos(p), 0, math.sin(p)],
                [ 0,           1, 0           ],
                [-math.sin(p), 0, math.cos(p)],
            ])
            Rx = np.array([
                [1, 0,            0           ],
                [0,  math.cos(r), -math.sin(r)],
                [0,  math.sin(r),  math.cos(r)],
            ])
            pts = ((Rx @ Ry) @ pts.T).T

        return pts.astype(np.float32)

    # ── Shutdown ──────────────────────────────────────────────────────────────
    def destroy_node(self) -> None:
        self._stop_event.set()
        try:
            self._send(CMD_SCAN_STOP)
            time.sleep(0.1)
            self._send(CMD_SERVO_ANGLE, struct.pack('<f', 0.0))
        except Exception:
            pass
        try:
            self._ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiderModuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
