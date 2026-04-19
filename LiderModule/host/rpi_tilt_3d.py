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


def load_scan_pattern(csv_path: str, mode: str = "下方") -> list[float]:
    """Load scan angles from CSV pattern file by step number."""
    with open(csv_path, 'r', encoding='utf-8-sig') as f:
        reader = csv.reader(f)
        rows = list(reader)

    # Find the mode section
    start_row = None
    for i, row in enumerate(rows):
        if row and row[0].startswith(mode):
            start_row = i + 3  # Skip header rows (mode + pattern + angle header)
            break

    if start_row is None:
        raise ValueError(f"Mode '{mode}' not found in CSV")

    # Find end row (next mode or end)
    end_row = len(rows)
    for j in range(start_row, len(rows)):
        if rows[j] and (rows[j][0].startswith("上方") or rows[j][0].startswith("下方")):
            if j > start_row:
                end_row = j
                break

    # Collect angle for each step number in the selected mode section.
    seq_to_angle = {}
    for row in rows[start_row:end_row]:
        if not row or len(row) < 2:
            continue
        angle = None
        for angle_str in (row[0], row[1]):
            if not angle_str:
                continue
            try:
                angle = float(angle_str)
                break
            except ValueError:
                continue
        if angle is None:
            continue

        for col_idx, cell in enumerate(row[2:], 2):  # Start from column 2 (index 2)
            if cell.strip():
                try:
                    seq = int(cell)
                    seq_to_angle[seq] = angle
                except ValueError:
                    pass

    if not seq_to_angle:
        return []

    # Execute in ascending step order (1,2,3...).
    return [angle for _, angle in sorted(seq_to_angle.items())]


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


def wait_for_latest_imu(
    events: "queue.Queue[object]", timeout_s: float, verbose: bool = False
) -> ImuData:
    deadline = time.monotonic() + timeout_s
    last_imu: Optional[ImuData] = None
    while time.monotonic() < deadline:
        remain = max(0.01, deadline - time.monotonic())
        try:
            event = events.get(timeout=min(0.2, remain))
        except queue.Empty:
            continue
        if isinstance(event, ImuData):
            last_imu = event
            if verbose:
                print(
                    f"[imu] pitch={event.pitch:.2f} roll={event.roll:.2f} yaw={event.yaw:.2f}",
                    flush=True,
                )
            return event
        if isinstance(event, ValueError):
            print(f"[warn] {event}", file=sys.stderr)
    if last_imu is not None:
        return last_imu
    raise TimeoutError("no IMU packet received before timeout")


def run_imu_settle_bench(ser, events: "queue.Queue[object]", args: argparse.Namespace) -> None:
    targets: list[float] = []
    for _ in range(args.bench_cycles):
        targets.append(args.bench_max)
        targets.append(args.bench_min)

    if not targets:
        print("[bench] no move scheduled", flush=True)
        return

    # Warm-up: ensure IMU stream is flowing and capture a baseline.
    baseline = wait_for_latest_imu(events, timeout_s=max(1.0, args.timeout), verbose=args.verbose)
    print(
        f"[bench] start roll={baseline.roll:.2f} deg moves={len(targets)} "
        f"window={args.bench_window_ms}ms p2p<={args.bench_peak2peak:.2f}deg",
        flush=True,
    )

    settled_times: list[float] = []
    failed = 0
    window_s = max(0.05, args.bench_window_ms / 1000.0)

    for idx, target in enumerate(targets, start=1):
        before = wait_for_latest_imu(events, timeout_s=max(1.0, args.timeout), verbose=args.verbose)
        start_roll = before.roll
        send_manual_tilt(ser, target)

        t0 = time.monotonic()
        samples: list[tuple[float, float]] = []
        moved = False
        stable_start: Optional[float] = None
        settled: Optional[float] = None

        while True:
            elapsed = time.monotonic() - t0
            if elapsed > args.bench_timeout:
                break

            event = wait_for_latest_imu(events, timeout_s=0.4, verbose=args.verbose)
            t = time.monotonic() - t0
            samples.append((t, event.roll))

            if not moved and abs(event.roll - start_roll) >= args.bench_min_move:
                moved = True

            if not moved:
                continue

            window = [roll for ts, roll in samples if (t - ts) <= window_s]
            if len(window) < 3:
                continue

            peak2peak = max(window) - min(window)
            drift = abs(window[-1] - window[0])
            if peak2peak <= args.bench_peak2peak and drift <= args.bench_peak2peak * 0.5:
                if stable_start is None:
                    stable_start = t
                elif (t - stable_start) >= window_s:
                    settled = t
                    break
            else:
                stable_start = None

        if settled is None:
            failed += 1
            print(
                f"[bench] {idx}/{len(targets)} target={target:+.1f} deg timeout "
                f"(>{args.bench_timeout:.2f}s)",
                flush=True,
            )
        else:
            settled_times.append(settled)
            print(
                f"[bench] {idx}/{len(targets)} target={target:+.1f} deg settled={settled:.3f}s",
                flush=True,
            )

    # Return to neutral after benchmark.
    send_manual_tilt(ser, 0.0)

    if not settled_times:
        print("[bench] no successful settle measurement", flush=True)
        return

    arr = np.array(settled_times, dtype=np.float64)
    p90 = float(np.percentile(arr, 90))
    reco_ms = int(math.ceil(p90 * 1000.0 + 50.0))
    print(
        f"[bench] summary ok={len(settled_times)}/{len(targets)} fail={failed} "
        f"mean={arr.mean():.3f}s median={np.median(arr):.3f}s p90={p90:.3f}s max={arr.max():.3f}s",
        flush=True,
    )
    print(f"[bench] recommended_settle_ms={reco_ms}", flush=True)


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
    if not args.no_viz and not args.imu_bench:
        try:
            viewer = Open3DViewer()
        except Exception as exc:  # pragma: no cover - depends on desktop runtime
            print(f"[viz] Open3D disabled: {exc}", file=sys.stderr)

    try:
        if args.imu_bench:
            run_imu_settle_bench(ser, events, args)
            return points

        if args.manual_tilt is not None:
            send_manual_tilt(ser, args.manual_tilt)
            print(f"[servo] tilt command sent: {args.manual_tilt:.2f} deg")
            time.sleep(0.2)
            return points

        if args.linear_scan:
            # Original linear scan
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
        else:
            # Pattern scan
            pattern_angles = load_scan_pattern(args.pattern_csv, args.mode)
            print(f"[scan] loaded {len(pattern_angles)} angles from {args.pattern_csv} mode={args.mode}")

            for i, angle in enumerate(pattern_angles):
                print(f"[scan] step {i+1}/{len(pattern_angles)}: tilt={angle:.2f} deg")
                send_scan_start(ser, angle, angle, 1.0)

                timeout = time.monotonic() + max(2.0, args.timeout)
                slice_received = False
                scan_complete = False
                while time.monotonic() < timeout and not (slice_received and scan_complete):
                    try:
                        event = events.get(timeout=0.1)
                    except queue.Empty:
                        continue

                    if isinstance(event, ScanSlice):
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
                        slice_received = True
                    elif isinstance(event, ScanStatus):
                        if args.verbose:
                            print_status(event)
                        scan_complete = event.state == 2
                    elif isinstance(event, ValueError):
                        print(f"[warn] {event}", file=sys.stderr)
                    elif isinstance(event, ImuData):
                        if args.verbose:
                            print(
                                f"[imu] pitch={event.pitch:.2f} roll={event.roll:.2f} yaw={event.yaw:.2f}",
                                flush=True,
                            )

                if not slice_received:
                    print(f"[warn] no slice received for angle {angle:.2f} deg", file=sys.stderr)

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
    parser.add_argument("--pattern-csv", default="../計測パターン.csv", help="scan pattern CSV file")
    parser.add_argument("--mode", choices=["下方", "上方"], default="下方", help="detection mode")
    parser.add_argument("--tilt-min", type=float, default=-45.0, help="scan start tilt [deg] (linear mode)")
    parser.add_argument("--tilt-max", type=float, default=45.0, help="scan end tilt [deg] (linear mode)")
    parser.add_argument("--step", type=float, default=2.0, help="tilt step [deg] (linear mode)")
    parser.add_argument("--linear-scan", action="store_true", help="use linear scan instead of CSV pattern")
    parser.add_argument("--save", help="output .ply or .csv path")
    parser.add_argument("--no-viz", action="store_true", help="disable live Open3D visualization")
    parser.add_argument("--manual-tilt", type=float, help="send one tilt angle command and exit")
    parser.add_argument(
        "--imu-bench",
        action="store_true",
        help="run fast IMU-only settle benchmark (LiDAR scan disabled)",
    )
    parser.add_argument(
        "--bench-min", type=float, default=-45.0, help="bench swing min angle [deg]"
    )
    parser.add_argument(
        "--bench-max", type=float, default=45.0, help="bench swing max angle [deg]"
    )
    parser.add_argument(
        "--bench-cycles", type=int, default=4, help="bench cycles (max->min pairs)"
    )
    parser.add_argument(
        "--bench-timeout", type=float, default=3.0, help="timeout per move [s]"
    )
    parser.add_argument(
        "--bench-min-move",
        type=float,
        default=2.0,
        help="minimum IMU roll change to detect motion [deg]",
    )
    parser.add_argument(
        "--bench-window-ms",
        type=int,
        default=120,
        help="stability window length [ms]",
    )
    parser.add_argument(
        "--bench-peak2peak",
        type=float,
        default=1.2,
        help="stability threshold: roll peak-to-peak in window [deg]",
    )
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
    if args.bench_cycles < 1:
        print("--bench-cycles must be >= 1", file=sys.stderr)
        return 2
    if args.bench_window_ms < 50:
        print("--bench-window-ms must be >= 50", file=sys.stderr)
        return 2
    if args.bench_timeout <= 0:
        print("--bench-timeout must be positive", file=sys.stderr)
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
