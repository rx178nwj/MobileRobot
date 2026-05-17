#!/usr/bin/env python3
"""
lidar_pointcloud_bridge — bridges ESP32-S3 tilting LiDAR to ROS 2 PointCloud2.

Sends CMD_3DSCAN_START in continuous mode, accumulates tilt slices until
STATE_SCAN_COMPLETE, converts to sensor_msgs/PointCloud2, and publishes to
/lidar/points.  Broadcasts dynamic TF lidar_base_link → lidar_tilt_link on
each slice.

Protocol reference: LiderModule/SPEC.md §3
"""

import math
import struct
import threading
import time
from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None  # type: ignore[assignment]

# ── Protocol constants (SPEC.md §3.2) ────────────────────────────────────────
_SYNC1 = 0xFE
_SYNC2 = 0xEF

_TYPE_SCAN_SLICE  = 0x04
_TYPE_SCAN_STATUS = 0x05
_TYPE_SCAN_START  = 0x12
_TYPE_SCAN_STOP   = 0x13

_STATE_SCAN_START    = 0
_STATE_SCAN_COMPLETE = 2

_SCAN_MODE_CONTINUOUS = 1

# ── Frame parser states ──────────────────────────────────────────────────────
_S_H1, _S_H2, _S_HDR, _S_PL, _S_CK = range(5)


# ── Frame builder ─────────────────────────────────────────────────────────────
def _build_frame(pkt_type: int, payload: bytes) -> bytes:
    ln_l = len(payload) & 0xFF
    ln_h = (len(payload) >> 8) & 0xFF
    ck   = pkt_type ^ ln_l ^ ln_h
    for b in payload:
        ck ^= b
    return bytes([_SYNC1, _SYNC2, pkt_type, ln_l, ln_h]) + payload + bytes([ck])


def _cmd_scan_start(tilt_min: float, tilt_max: float, tilt_step: float) -> bytes:
    return _build_frame(
        _TYPE_SCAN_START,
        struct.pack('<fffB', tilt_min, tilt_max, tilt_step, _SCAN_MODE_CONTINUOUS),
    )


def _cmd_scan_stop() -> bytes:
    return _build_frame(_TYPE_SCAN_STOP, b'')


# ── Payload parsers ───────────────────────────────────────────────────────────
def _parse_scan_slice(payload: bytes):
    """Returns (tilt_deg, imu_pitch, imu_roll, angles_deg, dists_mm, quals) or None."""
    if len(payload) < 14:
        return None
    tilt_deg, imu_pitch, imu_roll = struct.unpack_from('<fff', payload, 0)
    count = struct.unpack_from('<H', payload, 12)[0]
    if len(payload) < 14 + count * 5:
        return None
    angles = np.empty(count, dtype=np.float32)
    dists  = np.empty(count, dtype=np.float32)
    quals  = np.empty(count, dtype=np.uint8)
    for i in range(count):
        off = 14 + i * 5
        a100, d_mm, q = struct.unpack_from('<HHB', payload, off)
        angles[i] = a100 / 100.0
        dists[i]  = float(d_mm)
        quals[i]  = q
    return tilt_deg, imu_pitch, imu_roll, angles, dists, quals


def _parse_scan_status(payload: bytes):
    """Returns (state, step, total) or None."""
    if len(payload) < 5:
        return None
    state = payload[0]
    step, total = struct.unpack_from('<HH', payload, 1)
    return state, step, total


# ── Coordinate transform (SPEC.md §5.3) ────────────────────────────────────
def _slice_to_points(
    tilt_deg: float,
    imu_pitch: float,
    imu_roll: float,
    angles_deg: np.ndarray,
    dists_mm: np.ndarray,
    quals: np.ndarray,
    min_dist: int,
    max_dist: int,
    min_quality: int,
) -> np.ndarray:
    """Returns Nx4 float32 array [x, y, z, intensity] in lidar_base_link frame (metres).

    The tilt rotation Ry(alpha) is already applied, so the output coordinates are
    in the UN-TILTED frame (lidar_base_link), NOT in lidar_tilt_link.
    See SPEC.md §5.3 Step 2: X=x_l*cos(a), Z=-x_l*sin(a).
    """
    mask = (dists_mm > min_dist) & (dists_mm < max_dist) & (quals >= min_quality)
    if not np.any(mask):
        return np.empty((0, 4), dtype=np.float32)

    a     = math.radians(tilt_deg)
    theta = np.radians(angles_deg[mask].astype(np.float64))
    d     = dists_mm[mask].astype(np.float64) / 1000.0  # mm → m

    x_l = d * np.cos(theta)
    y_l = d * np.sin(theta)

    X = x_l * math.cos(a)
    Y = y_l
    Z = -x_l * math.sin(a)

    if abs(imu_pitch) > 0.1 or abs(imu_roll) > 0.1:
        cp, sp = math.cos(math.radians(imu_pitch)), math.sin(math.radians(imu_pitch))
        cr, sr = math.cos(math.radians(imu_roll)),  math.sin(math.radians(imu_roll))
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0],  [0, cr, -sr], [0, sr, cr]])
        pts = (Rx @ Ry @ np.stack([X, Y, Z])).T
        X, Y, Z = pts[:, 0], pts[:, 1], pts[:, 2]

    intensity = quals[mask].astype(np.float32)
    return np.column_stack([X, Y, Z, intensity]).astype(np.float32)


# ── PointCloud2 helper ────────────────────────────────────────────────────────
def _make_pointcloud2(points: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height          = 1
    msg.width           = len(points)
    msg.is_dense        = False
    msg.is_bigendian    = False
    msg.point_step      = 16  # 4 × float32
    msg.row_step        = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = points.tobytes()
    return msg


# ── ROS 2 Node ────────────────────────────────────────────────────────────────
class LidarPointcloudBridge(Node):

    def __init__(self):
        super().__init__('lidar_pointcloud_bridge')

        self.declare_parameter('port',        '/dev/ttyACM1')
        self.declare_parameter('baud',        921600)
        self.declare_parameter('tilt_min',    -30.0)
        self.declare_parameter('tilt_max',    0.0)
        self.declare_parameter('tilt_step',   3.0)
        self.declare_parameter('min_dist_mm', 20)
        self.declare_parameter('max_dist_mm', 10000)
        self.declare_parameter('min_quality', 10)
        # cloud_frame: PointCloud2 header frame_id.
        #   Must be lidar_base_link because _slice_to_points already un-tilts the
        #   coordinates via Ry(alpha).  Using lidar_tilt_link would cause RTAB-Map
        #   to double-apply the tilt rotation.
        self.declare_parameter('cloud_frame', 'lidar_base_link')
        # base_frame / tilt_frame: parent/child for the visualization TF broadcast.
        self.declare_parameter('base_frame',  'lidar_base_link')
        self.declare_parameter('tilt_frame',  'lidar_tilt_link')

        self._port        = self.get_parameter('port').value
        self._baud        = self.get_parameter('baud').value
        self._tilt_min    = self.get_parameter('tilt_min').value
        self._tilt_max    = self.get_parameter('tilt_max').value
        self._tilt_step   = self.get_parameter('tilt_step').value
        self._min_dist    = self.get_parameter('min_dist_mm').value
        self._max_dist    = self.get_parameter('max_dist_mm').value
        self._min_quality = self.get_parameter('min_quality').value
        self._cloud_frame = self.get_parameter('cloud_frame').value
        self._base_frame  = self.get_parameter('base_frame').value
        self._tilt_frame  = self.get_parameter('tilt_frame').value

        self._pub_cloud  = self.create_publisher(PointCloud2, '/lidar/points',      10)
        self._pub_status = self.create_publisher(String,      '/lidar/scan_status', 10)
        self._tf_bc      = TransformBroadcaster(self)

        self._ser: Optional[object] = None
        self._slices: List[np.ndarray] = []
        self._cksum_errors  = 0
        self._total_frames  = 0

        self.get_logger().info(
            f'LiDAR bridge — port={self._port} tilt=[{self._tilt_min},{self._tilt_max}]deg '
            f'step={self._tilt_step}deg  cloud_frame={self._cloud_frame}'
        )

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ── Connection / main loop ────────────────────────────────────────────────
    def _run(self):
        if serial is None:
            self.get_logger().error(
                'pyserial not installed — run: pip install pyserial'
            )
            return
        while rclpy.ok():
            try:
                self._connect_and_read()
            except Exception as exc:
                self.get_logger().warn(f'Serial error: {exc} — retrying in 5 s')
                self._close()
                time.sleep(5.0)

    def _connect_and_read(self):
        self.get_logger().info(f'Opening {self._port} …')
        self._ser = serial.Serial(
            self._port, self._baud,
            timeout=1.0,
            dsrdtr=False, rtscts=False,
        )
        self.get_logger().info('Connected — sending scan start')
        self._send(
            _cmd_scan_start(self._tilt_min, self._tilt_max, self._tilt_step)
        )
        self._read_loop()

    def _send(self, data: bytes):
        if self._ser and self._ser.is_open:
            self._ser.write(data)

    def _close(self):
        if self._ser:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

    def _read_loop(self):
        state        = _S_H1
        pkt_type     = 0
        hdr_buf      = bytearray()
        payload      = bytearray()
        expected_len = 0
        last_rx       = time.monotonic()
        last_scan_rx  = time.monotonic()  # last time a SCAN_SLICE or SCAN_STATUS arrived

        while rclpy.ok() and self._ser and self._ser.is_open:
            now = time.monotonic()
            raw = self._ser.read(512)
            if not raw:
                if now - last_rx > 30.0:
                    self.get_logger().warn('30 s without data — restarting scan')
                    self._send(_cmd_scan_stop())
                    time.sleep(0.3)
                    self._slices.clear()
                    self._send(
                        _cmd_scan_start(self._tilt_min, self._tilt_max, self._tilt_step)
                    )
                    last_rx = last_scan_rx = time.monotonic()
                continue
            last_rx = now
            # Scan-specific watchdog: IMU packets (0x02) keep last_rx fresh but don't
            # indicate scan activity.  If no scan frame arrives for 10 s after connect,
            # the firmware likely rejected CMD_SCAN_START (LiDAR not yet warmed up).
            if now - last_scan_rx > 10.0:
                self.get_logger().warn('10 s without scan frame — resending scan start')
                self._send(_cmd_scan_start(self._tilt_min, self._tilt_max, self._tilt_step))
                last_scan_rx = now

            for byte in raw:
                if state == _S_H1:
                    if byte == _SYNC1:
                        state = _S_H2
                elif state == _S_H2:
                    state = _S_HDR if byte == _SYNC2 else _S_H1
                    if state == _S_HDR:
                        hdr_buf = bytearray()
                elif state == _S_HDR:
                    hdr_buf.append(byte)
                    if len(hdr_buf) == 3:
                        pkt_type     = hdr_buf[0]
                        expected_len = hdr_buf[1] | (hdr_buf[2] << 8)
                        payload      = bytearray()
                        state        = _S_PL if expected_len > 0 else _S_CK
                elif state == _S_PL:
                    payload.append(byte)
                    if len(payload) == expected_len:
                        state = _S_CK
                elif state == _S_CK:
                    ck = pkt_type ^ hdr_buf[1] ^ hdr_buf[2]
                    for b in payload:
                        ck ^= b
                    self._total_frames += 1
                    if ck == byte:
                        if pkt_type in (_TYPE_SCAN_SLICE, _TYPE_SCAN_STATUS):
                            last_scan_rx = time.monotonic()
                        self._dispatch(pkt_type, bytes(payload))
                    else:
                        self._cksum_errors += 1
                        if self._total_frames >= 100:
                            rate = self._cksum_errors / self._total_frames
                            if rate > 0.01:
                                self.get_logger().warn(
                                    f'Checksum error rate {rate:.1%} '
                                    f'({self._cksum_errors}/{self._total_frames})'
                                )
                    state = _S_H1

    # ── Packet handlers ───────────────────────────────────────────────────────
    def _dispatch(self, pkt_type: int, payload: bytes):
        if pkt_type == _TYPE_SCAN_SLICE:
            parsed = _parse_scan_slice(payload)
            if parsed is None:
                return
            tilt_deg, imu_pitch, imu_roll, angles, dists, quals = parsed
            self._broadcast_tilt_tf(tilt_deg)
            pts = _slice_to_points(
                tilt_deg, imu_pitch, imu_roll, angles, dists, quals,
                self._min_dist, self._max_dist, self._min_quality,
            )
            if len(pts):
                self._slices.append(pts)

        elif pkt_type == _TYPE_SCAN_STATUS:
            parsed = _parse_scan_status(payload)
            if parsed is None:
                return
            state, _step, _total = parsed
            if state == _STATE_SCAN_START:
                self._slices.clear()
                self._publish_status('scanning')
            elif state == _STATE_SCAN_COMPLETE:
                self._publish_cloud()
                self._slices.clear()
                self._publish_status('idle')

    def _broadcast_tilt_tf(self, tilt_deg: float):
        """Broadcast lidar_base_link → lidar_tilt_link for real-time visualization."""
        a = math.radians(tilt_deg)
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self._base_frame   # lidar_base_link
        t.child_frame_id  = self._tilt_frame   # lidar_tilt_link
        t.transform.rotation.y = math.sin(a / 2.0)
        t.transform.rotation.w = math.cos(a / 2.0)
        self._tf_bc.sendTransform(t)

    def _publish_cloud(self):
        if not self._slices:
            self.get_logger().warn('Scan complete but no points accumulated')
            return
        combined = np.vstack(self._slices).astype(np.float32)
        # frame_id = cloud_frame (lidar_base_link): coordinates are already in the
        # un-tilted frame because _slice_to_points applies Ry(alpha) internally.
        msg = _make_pointcloud2(
            combined, self._cloud_frame, self.get_clock().now().to_msg()
        )
        self._pub_cloud.publish(msg)
        self.get_logger().info(
            f'Published /lidar/points: {len(combined)} pts '
            f'from {len(self._slices)} slices'
        )

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self._pub_status.publish(msg)

    def destroy_node(self):
        self._send(_cmd_scan_stop())
        self._close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarPointcloudBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
