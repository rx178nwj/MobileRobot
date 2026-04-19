#!/usr/bin/env python3
"""Raspberry Pi host tool for the LiderModule 3D LiDAR scanner."""

from __future__ import annotations

import argparse
import csv
import math
import queue
import struct
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

import numpy as np

try:
    import serial
except ImportError:  # pragma: no cover - handled in main
    serial = None


SYNC1 = 0xFE
SYNC2 = 0xEF

MSG_IMU_DATA = 0x02
MSG_SCAN_SLICE = 0x04
MSG_SCAN_STATUS = 0x05
CMD_SERVO_ANGLE = 0x10
CMD_SCAN_START = 0x12
CMD_SCAN_STOP = 0x13


@dataclass
class ImuData:
    pitch: float
    roll: float
    yaw: float


@dataclass
class ScanStatus:
    state: int
    step: int
    total: int


@dataclass
class ScanSlice:
    tilt_deg: float
    imu_pitch_deg: float
    imu_roll_deg: float
    angles_deg: np.ndarray
    dists_mm: np.ndarray
    qualities: np.ndarray


def frame_checksum(msg_type: int, payload: bytes) -> int:
    length = len(payload)
    checksum = msg_type ^ (length & 0xFF) ^ ((length >> 8) & 0xFF)
    for b in payload:
        checksum ^= b
    return checksum & 0xFF


def encode_frame(msg_type: int, payload: bytes = b"") -> bytes:
    length = len(payload)
    header = bytes([SYNC1, SYNC2, msg_type, length & 0xFF, (length >> 8) & 0xFF])
    return header + payload + bytes([frame_checksum(msg_type, payload)])


def decode_payload(msg_type: int, payload: bytes):
    if msg_type == MSG_IMU_DATA:
        if len(payload) != 12:
            raise ValueError(f"bad IMU payload length: {len(payload)}")
        return ImuData(*struct.unpack("<fff", payload))

    if msg_type == MSG_SCAN_STATUS:
        if len(payload) != 5:
            raise ValueError(f"bad status payload length: {len(payload)}")
        state, step, total = struct.unpack("<BHH", payload)
        return ScanStatus(state, step, total)

    if msg_type == MSG_SCAN_SLICE:
        if len(payload) < 14:
            raise ValueError(f"bad slice payload length: {len(payload)}")
        tilt, imu_pitch, imu_roll, count = struct.unpack_from("<fffH", payload, 0)
        expected = 14 + count * 5
        if len(payload) != expected:
            raise ValueError(f"bad slice payload length: {len(payload)} != {expected}")
        raw = payload[14:]
        angles = np.empty(count, dtype=np.float64)
        dists = np.empty(count, dtype=np.uint16)
        qualities = np.empty(count, dtype=np.uint8)
        for i in range(count):
            angle_x100, dist_mm, quality = struct.unpack_from("<HHB", raw, i * 5)
            angles[i] = angle_x100 / 100.0
            dists[i] = dist_mm
            qualities[i] = quality
        return ScanSlice(tilt, imu_pitch, imu_roll, angles, dists, qualities)

    return payload


class FrameReader(threading.Thread):
    def __init__(self, ser, out_queue: "queue.Queue[object]", stop_event: threading.Event):
        super().__init__(daemon=True)
        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event
        self.errors = 0

    def run(self) -> None:
        state = 0
        msg_type = 0
        length = 0
        payload = bytearray()

        while not self.stop_event.is_set():
            chunk = self.ser.read(512)
            if not chunk:
                continue
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
                    if length > 65535:
                        state = 0
                    else:
                        payload = bytearray()
                        state = 6 if length == 0 else 5
                elif state == 5:
                    payload.append(b)
                    if len(payload) >= length:
                        state = 6
                elif state == 6:
                    if frame_checksum(msg_type, payload) == b:
                        try:
                            self.out_queue.put(decode_payload(msg_type, bytes(payload)))
                        except ValueError as exc:
                            self.out_queue.put(exc)
                    else:
                        self.errors += 1
                    state = 0


def rx_matrix(roll_rad: float) -> np.ndarray:
    c = math.cos(roll_rad)
    s = math.sin(roll_rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def ry_matrix(pitch_rad: float) -> np.ndarray:
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def transform_slice_to_3d(
    angles_deg: np.ndarray,
    dists_mm: np.ndarray,
    tilt_deg: float,
    imu_pitch_deg: float = 0.0,
    imu_roll_deg: float = 0.0,
    min_dist_mm: int = 20,
    max_dist_mm: int = 12000,
    min_quality: int = 0,
    qualities: Optional[np.ndarray] = None,
) -> np.ndarray:
    mask = (dists_mm > min_dist_mm) & (dists_mm < max_dist_mm)
    if qualities is not None and min_quality > 0:
        mask &= qualities >= min_quality

    angles = np.radians(angles_deg[mask])
    dists = dists_mm[mask].astype(np.float64) / 1000.0

    x_l = dists * np.cos(angles)
    y_l = dists * np.sin(angles)

    tilt = math.radians(tilt_deg)
    x = x_l * math.cos(tilt)
    y = y_l
    z = -x_l * math.sin(tilt)
    pts = np.stack([x, y, z], axis=1)

    if abs(imu_pitch_deg) > 0.1 or abs(imu_roll_deg) > 0.1:
        rot = rx_matrix(math.radians(imu_roll_deg)) @ ry_matrix(math.radians(imu_pitch_deg))
        pts = (rot @ pts.T).T
    return pts


def colors_from_z(points: np.ndarray) -> np.ndarray:
    if len(points) == 0:
        return np.empty((0, 3), dtype=np.uint8)
    z = points[:, 2]
    span = float(z.max() - z.min())
    t = np.zeros_like(z) if span < 1e-9 else (z - z.min()) / span
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:, 0] = np.clip(255 * t, 0, 255).astype(np.uint8)
    colors[:, 1] = np.clip(255 * (1.0 - np.abs(t - 0.5) * 2.0), 0, 255).astype(np.uint8)
    colors[:, 2] = np.clip(255 * (1.0 - t), 0, 255).astype(np.uint8)
    return colors


def save_csv(path: Path, points: np.ndarray) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "z"])
        writer.writerows(points.tolist())


def save_ply(path: Path, points: np.ndarray) -> None:
    colors = colors_from_z(points)
    with path.open("w", encoding="ascii", newline="\n") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for point, color in zip(points, colors):
            f.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def save_points(path: str, points: np.ndarray) -> None:
    out = Path(path)
    if out.suffix.lower() == ".csv":
        save_csv(out, points)
    elif out.suffix.lower() == ".ply":
        save_ply(out, points)
    else:
        raise ValueError("--save must end with .ply or .csv")


class Open3DViewer:
    def __init__(self):
        import open3d as o3d

        self.o3d = o3d
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="LiderModule 3D Scan")
        self.cloud = o3d.geometry.PointCloud()
        self.added = False

    def update(self, points: np.ndarray) -> None:
        if len(points) == 0:
            return
        self.cloud.points = self.o3d.utility.Vector3dVector(points)
        self.cloud.colors = self.o3d.utility.Vector3dVector(colors_from_z(points) / 255.0)
        if not self.added:
            self.vis.add_geometry(self.cloud)
            self.added = True
        else:
            self.vis.update_geometry(self.cloud)
        self.vis.poll_events()
        self.vis.update_renderer()

    def close(self) -> None:
        self.vis.destroy_window()


def send_manual_tilt(ser, angle: float) -> None:
    ser.write(encode_frame(CMD_SERVO_ANGLE, struct.pack("<f", angle)))
    ser.flush()


def send_scan_start(ser, tilt_min: float, tilt_max: float, step: float) -> None:
    ser.write(encode_frame(CMD_SCAN_START, struct.pack("<fff", tilt_min, tilt_max, step)))
    ser.flush()


def send_scan_stop(ser) -> None:
    ser.write(encode_frame(CMD_SCAN_STOP))
    ser.flush()


def print_status(status: ScanStatus) -> None:
    names = {0: "started", 1: "running", 2: "complete"}
    name = names.get(status.state, f"state={status.state}")
    print(f"[scan] {name}: {status.step}/{status.total}", flush=True)


def collect_scan(args: argparse.Namespace) -> np.ndarray:
    if serial is None:
        raise RuntimeError("pyserial is not installed. Run: pip install -r requirements.txt")

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.dtr = False
    ser.rts = False
    events: "queue.Queue[object]" = queue.Queue()
    stop_event = threading.Event()
    reader = FrameReader(ser, events, stop_event)
    reader.start()

    points = np.empty((0, 3), dtype=np.float64)
    viewer = None
    if not args.no_viz:
        try:
            viewer = Open3DViewer()
        except Exception as exc:  # pragma: no cover - depends on desktop runtime
            print(f"[viz] Open3D disabled: {exc}", file=sys.stderr)

    try:
        if args.manual_tilt is not None:
            send_manual_tilt(ser, args.manual_tilt)
            print(f"[servo] tilt command sent: {args.manual_tilt:.2f} deg")
            time.sleep(0.2)
            return points

        send_scan_start(ser, args.tilt_min, args.tilt_max, args.step)
        complete = False
        last_message = time.monotonic()
        while not complete:
            try:
                event = events.get(timeout=0.2)
            except queue.Empty:
                if time.monotonic() - last_message > args.timeout:
                    raise TimeoutError("no scanner data received before timeout")
                if viewer is not None:
                    viewer.update(points)
                continue

            last_message = time.monotonic()
            if isinstance(event, ValueError):
                print(f"[warn] {event}", file=sys.stderr)
            elif isinstance(event, ImuData):
                if args.verbose:
                    print(
                        f"[imu] pitch={event.pitch:.2f} roll={event.roll:.2f} yaw={event.yaw:.2f}",
                        flush=True,
                    )
            elif isinstance(event, ScanStatus):
                print_status(event)
                complete = event.state == 2
            elif isinstance(event, ScanSlice):
                pts = transform_slice_to_3d(
                    event.angles_deg,
                    event.dists_mm,
                    event.tilt_deg,
                    event.imu_pitch_deg,
                    event.imu_roll_deg,
                    min_dist_mm=args.min_dist,
                    max_dist_mm=args.max_dist,
                    min_quality=args.min_quality,
                    qualities=event.qualities,
                )
                if len(pts):
                    points = np.vstack([points, pts])
                print(
                    f"[slice] tilt={event.tilt_deg:.2f} deg raw={len(event.dists_mm)} "
                    f"accepted={len(pts)} total={len(points)}",
                    flush=True,
                )
                if viewer is not None:
                    viewer.update(points)
        return points
    finally:
        try:
            send_scan_stop(ser)
        except Exception:
            pass
        stop_event.set()
        reader.join(timeout=1.0)
        if viewer is not None:
            viewer.close()
        ser.close()


def plot_matplotlib(points: np.ndarray) -> None:
    import matplotlib.pyplot as plt

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    if len(points) > 50000:
        idx = np.random.default_rng(0).choice(len(points), size=50000, replace=False)
        shown = points[idx]
    else:
        shown = points
    colors = colors_from_z(shown) / 255.0
    ax.scatter(shown[:, 0], shown[:, 1], shown[:, 2], c=colors, s=1)
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    plt.show()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="LiderModule 3D LiDAR scanner host")
    parser.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    parser.add_argument("--baud", type=int, default=921600, help="USB CDC baud rate")
    parser.add_argument("--tilt-min", type=float, default=-45.0, help="scan start tilt [deg]")
    parser.add_argument("--tilt-max", type=float, default=45.0, help="scan end tilt [deg]")
    parser.add_argument("--step", type=float, default=2.0, help="tilt step [deg]")
    parser.add_argument("--save", help="output .ply or .csv path")
    parser.add_argument("--no-viz", action="store_true", help="disable live Open3D visualization")
    parser.add_argument("--manual-tilt", type=float, help="send one tilt angle command and exit")
    parser.add_argument("--min-dist", type=int, default=20, help="minimum accepted distance [mm]")
    parser.add_argument("--max-dist", type=int, default=12000, help="maximum accepted distance [mm]")
    parser.add_argument("--min-quality", type=int, default=0, help="minimum accepted LiDAR quality")
    parser.add_argument("--timeout", type=float, default=10.0, help="receive timeout [s]")
    parser.add_argument("--verbose", action="store_true", help="print IMU packets")
    parser.add_argument("--matplotlib", action="store_true", help="show final matplotlib plot")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.step <= 0:
        print("--step must be positive", file=sys.stderr)
        return 2

    try:
        points = collect_scan(args)
        if args.save and len(points):
            save_points(args.save, points)
            print(f"[save] wrote {len(points)} points to {args.save}")
        if args.matplotlib and len(points):
            plot_matplotlib(points)
    except KeyboardInterrupt:
        print("\n[stop] interrupted", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"[error] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
