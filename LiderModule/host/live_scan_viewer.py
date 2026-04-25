#!/usr/bin/env python3
"""Continuous live visualization tool for LiderModule scans."""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import numpy as np

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

from rpi_tilt_3d import (
    FrameReader,
    ImuData,
    ScanSlice,
    ScanStatus,
    send_scan_start,
    send_scan_stop,
    transform_slice_to_3d,
)


@dataclass
class LiveStats:
    cycle: int = 0
    slices: int = 0
    accepted_points: int = 0
    raw_points: int = 0
    imu_pitch: float = 0.0
    imu_roll: float = 0.0
    imu_yaw: float = 0.0
    last_status_step: int = 0
    last_status_total: int = 0
    recoveries: int = 0
    imu_ref_pitch: float = 0.0
    imu_ref_roll: float = 0.0
    imu_ref_locked: bool = False
    accepted_points_up: int = 0
    accepted_points_down: int = 0
    raw_points_up: int = 0
    raw_points_down: int = 0
    last_direction: str = "unknown"


class RollingPointBuffer:
    def __init__(self, max_points: int):
        self.max_points = max_points
        self.chunks: deque[np.ndarray] = deque()
        self.total = 0

    def append(self, pts: np.ndarray) -> None:
        if len(pts) == 0:
            return
        self.chunks.append(pts)
        self.total += len(pts)

        while self.total > self.max_points and self.chunks:
            overflow = self.total - self.max_points
            head = self.chunks[0]
            if len(head) <= overflow:
                self.chunks.popleft()
                self.total -= len(head)
            else:
                self.chunks[0] = head[overflow:]
                self.total -= overflow

    def stacked(self) -> np.ndarray:
        if not self.chunks:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(self.chunks)

    def clear(self) -> None:
        self.chunks.clear()
        self.total = 0


class MatplotlibLiveViewer:
    def __init__(self, title: str, equal_axis: bool):
        import matplotlib.pyplot as plt
        from matplotlib.widgets import Button

        self.plt = plt
        self.equal_axis = equal_axis
        self.fig = plt.figure(figsize=(16, 9))
        self.fig.suptitle(title, fontsize=14)
        self.ax3d = self.fig.add_subplot(2, 3, 1, projection="3d")
        self.ax_xy = self.fig.add_subplot(2, 3, 2)
        self.ax_xz = self.fig.add_subplot(2, 3, 3)
        self.ax_yz = self.fig.add_subplot(2, 3, 4)
        self.ax_info = self.fig.add_subplot(2, 3, (5, 6))
        self.ax_info.axis("off")
        self.fig.subplots_adjust(left=0.05, right=0.98, top=0.92, bottom=0.08, wspace=0.26, hspace=0.30)

        self.ax_btn_clear = self.fig.add_axes([0.86, 0.015, 0.11, 0.045])
        self.btn_clear = Button(self.ax_btn_clear, "Clear")
        self.plt.show(block=False)

    @staticmethod
    def _set_axes_equal(ax, pts: np.ndarray) -> None:
        if len(pts) == 0:
            return
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        center = (mins + maxs) / 2.0
        radius = float(np.max(maxs - mins)) / 2.0
        if radius <= 0.0:
            radius = 1.0
        ax.set_xlim(center[0] - radius, center[0] + radius)
        ax.set_ylim(center[1] - radius, center[1] + radius)
        ax.set_zlim(center[2] - radius, center[2] + radius)

    @staticmethod
    def _height_profile(ax, pts: np.ndarray, color: str, label: str) -> None:
        if len(pts) < 20:
            return
        z = pts[:, 2]
        y = pts[:, 1]
        if float(np.max(z) - np.min(z)) <= 1e-6:
            return
        bins = np.linspace(float(np.min(z)), float(np.max(z)), 40)
        z_centers = 0.5 * (bins[:-1] + bins[1:])
        y_mean = np.full_like(z_centers, np.nan, dtype=np.float64)
        for i in range(len(bins) - 1):
            m = (z >= bins[i]) & (z < bins[i + 1])
            if np.count_nonzero(m) >= 5:
                y_mean[i] = float(np.mean(y[m]))
        valid = ~np.isnan(y_mean)
        if np.any(valid):
            ax.plot(y_mean[valid], z_centers[valid], color=color, linewidth=1.4, label=label)

    def update(
        self,
        pts_up: np.ndarray,
        idx_up: np.ndarray,
        pts_down: np.ndarray,
        idx_down: np.ndarray,
        stats: LiveStats,
    ) -> None:
        self.ax3d.clear()
        self.ax_xy.clear()
        self.ax_xz.clear()
        self.ax_yz.clear()
        self.ax_info.clear()
        self.ax_info.axis("off")

        if len(pts_up):
            c_up = idx_up.astype(np.float64)
            self.ax3d.scatter(pts_up[:, 0], pts_up[:, 1], pts_up[:, 2], c=c_up, cmap="Blues", s=1, label="Up")
            self.ax_xy.scatter(pts_up[:, 0], pts_up[:, 1], c=c_up, cmap="Blues", s=1)
            self.ax_xz.scatter(pts_up[:, 0], pts_up[:, 2], c=c_up, cmap="Blues", s=1)
            self.ax_yz.scatter(pts_up[:, 1], pts_up[:, 2], c=c_up, cmap="Blues", s=1)

        if len(pts_down):
            c_down = idx_down.astype(np.float64)
            self.ax3d.scatter(
                pts_down[:, 0], pts_down[:, 1], pts_down[:, 2], c=c_down, cmap="Oranges", s=1, label="Down"
            )
            self.ax_xy.scatter(pts_down[:, 0], pts_down[:, 1], c=c_down, cmap="Oranges", s=1)
            self.ax_xz.scatter(pts_down[:, 0], pts_down[:, 2], c=c_down, cmap="Oranges", s=1)
            self.ax_yz.scatter(pts_down[:, 1], pts_down[:, 2], c=c_down, cmap="Oranges", s=1)

        self._height_profile(self.ax_yz, pts_up, color="blue", label="mean Y (up)")
        self._height_profile(self.ax_yz, pts_down, color="darkorange", label="mean Y (down)")
        if len(pts_up) or len(pts_down):
            self.ax_yz.legend(loc="best", fontsize=8)

        self.ax3d.set_title("3D Live Point Cloud (Up/Down Split)")
        self.ax3d.set_xlabel("X [m]")
        self.ax3d.set_ylabel("Y [m]")
        self.ax3d.set_zlabel("Z [m]")
        if len(pts_up) or len(pts_down):
            self.ax3d.legend(loc="best", fontsize=8)
        if self.equal_axis and (len(pts_up) or len(pts_down)):
            merged = np.vstack([x for x in (pts_up, pts_down) if len(x)])
            self._set_axes_equal(self.ax3d, merged)

        self.ax_xy.set_title("Top View (X-Y)")
        self.ax_xy.set_xlabel("X [m]")
        self.ax_xy.set_ylabel("Y [m]")
        self.ax_xy.grid(True, alpha=0.3)

        self.ax_xz.set_title("Side View (X-Z)")
        self.ax_xz.set_xlabel("X [m]")
        self.ax_xz.set_ylabel("Z [m]")
        self.ax_xz.grid(True, alpha=0.3)

        self.ax_yz.set_title("Height-wise Y Graph (Y-Z)")
        self.ax_yz.set_xlabel("Y [m]")
        self.ax_yz.set_ylabel("Z [m]")
        self.ax_yz.grid(True, alpha=0.3)

        info = (
            f"cycle            : {stats.cycle}\n"
            f"slices           : {stats.slices}\n"
            f"recoveries       : {stats.recoveries}\n"
            f"raw points       : {stats.raw_points}\n"
            f"accepted points  : {stats.accepted_points}\n"
            f"up/down accepted : {stats.accepted_points_up} / {stats.accepted_points_down}\n"
            f"up/down raw      : {stats.raw_points_up} / {stats.raw_points_down}\n"
            f"last direction   : {stats.last_direction}\n"
            f"status           : {stats.last_status_step}/{stats.last_status_total}\n"
            f"IMU pitch/roll/yaw: {stats.imu_pitch:.2f} / {stats.imu_roll:.2f} / {stats.imu_yaw:.2f}\n"
            f"IMU ref (0deg)   : {stats.imu_ref_pitch:.2f} / {stats.imu_ref_roll:.2f} "
            f"({'locked' if stats.imu_ref_locked else 'waiting'})\n"
            f"render points    : up={len(pts_up)} down={len(pts_down)}"
        )
        self.ax_info.text(0.0, 1.0, info, va="top", ha="left", family="monospace", fontsize=11)

        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def close(self) -> None:
        self.plt.close(self.fig)

    def set_clear_callback(self, cb: Callable[[], None]) -> None:
        def _on_click(_event) -> None:
            cb()

        self.btn_clear.on_clicked(_on_click)


def sample_for_render(points: np.ndarray, max_points: int, seed: int) -> tuple[np.ndarray, np.ndarray]:
    if len(points) <= max_points:
        idx = np.arange(len(points), dtype=np.int64)
        return points, idx
    rng = np.random.default_rng(seed)
    idx = np.sort(rng.choice(len(points), size=max_points, replace=False))
    return points[idx], idx


def run_live(args: argparse.Namespace) -> int:
    if serial is None:
        print("[error] pyserial is not installed", file=sys.stderr)
        return 2

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.dtr = False
    ser.rts = False
    events: "queue.Queue[object]" = queue.Queue()
    stop_event = threading.Event()
    reader = FrameReader(ser, events, stop_event)
    reader.start()

    stats = LiveStats(cycle=1)
    buffer_up = RollingPointBuffer(max_points=args.window_points)
    buffer_down = RollingPointBuffer(max_points=args.window_points)
    viewer = MatplotlibLiveViewer(title=args.title, equal_axis=args.equal_axis)
    last_refresh = 0.0
    running = True
    last_tilt_end: Optional[float] = None
    last_tilt_motion_ts = time.monotonic()
    last_slice_ts = time.monotonic()
    last_recovery_ts = 0.0
    start_ts = time.monotonic()
    imu_ref_pitch: Optional[float] = None
    imu_ref_roll: Optional[float] = None
    stale_hits = 0
    have_slice_once = False
    last_direction_sign = 0

    try:
        def _clear_points() -> None:
            buffer_up.clear()
            buffer_down.clear()
            print("[live] clear requested from viewer button", flush=True)

        viewer.set_clear_callback(_clear_points)

        send_scan_stop(ser)
        time.sleep(0.1)
        send_scan_start(ser, args.tilt_min, args.tilt_max, args.step)
        print(
            f"[live] started continuous scan on {args.port}  "
            f"range={args.tilt_min}..{args.tilt_max} step={args.step}",
            flush=True,
        )

        while running:
            try:
                event = events.get(timeout=0.1)
            except queue.Empty:
                event = None

            if isinstance(event, ScanSlice):
                last_slice_ts = time.monotonic()
                have_slice_once = True
                tilt_mid = 0.5 * (event.tilt_start_deg + event.tilt_end_deg)
                if imu_ref_pitch is None and abs(tilt_mid) <= args.imu_ref_tilt_threshold_deg:
                    imu_ref_pitch = event.imu_pitch_deg
                    imu_ref_roll = event.imu_roll_deg
                    stats.imu_ref_pitch = imu_ref_pitch
                    stats.imu_ref_roll = imu_ref_roll
                    stats.imu_ref_locked = True
                    print(
                        f"[imu] reference locked at tilt~0: pitch={imu_ref_pitch:.2f} roll={imu_ref_roll:.2f}",
                        flush=True,
                    )
                if last_tilt_end is None:
                    last_tilt_end = event.tilt_end_deg
                    last_tilt_motion_ts = last_slice_ts
                else:
                    if abs(event.tilt_end_deg - last_tilt_end) >= args.motion_delta_deg:
                        last_tilt_motion_ts = last_slice_ts
                        last_tilt_end = event.tilt_end_deg

                delta = event.tilt_end_deg - event.tilt_start_deg
                if delta > args.direction_deadband_deg:
                    direction_sign = 1
                elif delta < -args.direction_deadband_deg:
                    direction_sign = -1
                else:
                    direction_sign = last_direction_sign
                if direction_sign == 0:
                    direction_sign = 1
                last_direction_sign = direction_sign
                stats.last_direction = "up" if direction_sign > 0 else "down"

                tilt_series = np.linspace(
                    event.tilt_start_deg,
                    event.tilt_end_deg,
                    num=len(event.angles_deg),
                    dtype=np.float64,
                )
                pts = transform_slice_to_3d(
                    event.angles_deg,
                    event.dists_mm,
                    tilt_series,
                    tilt_axis_offset_m=args.tilt_axis_offset_mm / 1000.0,
                    imu_pitch_deg=(imu_ref_pitch if imu_ref_pitch is not None else 0.0),
                    imu_roll_deg=(imu_ref_roll if imu_ref_roll is not None else 0.0),
                    min_dist_mm=args.min_dist,
                    max_dist_mm=args.max_dist,
                    min_quality=args.min_quality,
                    qualities=event.qualities,
                )
                if direction_sign > 0:
                    buffer_up.append(pts)
                    stats.accepted_points_up += len(pts)
                    stats.raw_points_up += len(event.dists_mm)
                else:
                    buffer_down.append(pts)
                    stats.accepted_points_down += len(pts)
                    stats.raw_points_down += len(event.dists_mm)
                stats.slices += 1
                stats.raw_points += len(event.dists_mm)
                stats.accepted_points += len(pts)

            elif isinstance(event, ScanStatus):
                stats.last_status_step = event.step
                stats.last_status_total = event.total
                if event.state == 2:
                    stats.cycle += 1
                    send_scan_start(ser, args.tilt_min, args.tilt_max, args.step)

            elif isinstance(event, ImuData):
                stats.imu_pitch = event.pitch
                stats.imu_roll = event.roll
                stats.imu_yaw = event.yaw

            elif isinstance(event, ValueError):
                print(f"[warn] {event}", file=sys.stderr)

            now = time.monotonic()
            no_motion_sec = now - last_tilt_motion_ts
            no_slice_sec = now - last_slice_ts
            cooldown_ok = (now - last_recovery_ts) >= args.recovery_cooldown_sec
            stale_motion = no_motion_sec >= args.auto_recover_sec
            stale_slice = no_slice_sec >= args.no_slice_recover_sec
            motion_with_no_slice = stale_motion and (no_slice_sec >= args.motion_requires_no_slice_sec)
            startup_grace_ok = (now - start_ts) >= args.recovery_startup_grace_sec
            stall_detected = stale_slice or motion_with_no_slice
            if stall_detected:
                stale_hits += 1
            else:
                stale_hits = 0

            if (
                args.auto_recover
                and have_slice_once
                and startup_grace_ok
                and cooldown_ok
                and stale_hits >= args.recovery_confirmations
                and stall_detected
            ):
                reason = "no-slice" if stale_slice else "no-motion+no-slice"
                print(
                    f"[recovery] trigger ({reason}) "
                    f"motion={no_motion_sec:.2f}s slice={no_slice_sec:.2f}s hits={stale_hits}",
                    flush=True,
                )
                try:
                    send_scan_stop(ser)
                    time.sleep(0.08)
                    if args.clear_on_recover:
                        buffer_up.clear()
                        buffer_down.clear()
                        print("[recovery] cleared buffers", flush=True)
                    send_scan_start(ser, args.tilt_min, args.tilt_max, args.step)
                    stats.recoveries += 1
                    last_recovery_ts = now
                    last_tilt_end = None
                    last_tilt_motion_ts = now
                    last_slice_ts = now
                    stale_hits = 0
                except Exception as exc:
                    print(f"[warn] recovery failed: {exc}", file=sys.stderr)

            if now - last_refresh >= args.refresh_sec:
                last_refresh = now
                points_up = buffer_up.stacked()
                points_down = buffer_down.stacked()
                shown_up, idx_up = sample_for_render(points_up, max_points=args.render_points // 2, seed=args.seed)
                shown_down, idx_down = sample_for_render(
                    points_down, max_points=args.render_points // 2, seed=args.seed + 1
                )
                viewer.update(shown_up, idx_up, shown_down, idx_down, stats)
    except KeyboardInterrupt:
        print("\n[live] interrupted", file=sys.stderr)
    finally:
        try:
            send_scan_stop(ser)
        except Exception:
            pass
        stop_event.set()
        reader.join(timeout=1.0)
        viewer.close()
        ser.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="LiderModule continuous live viewer")
    p.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    p.add_argument("--baud", type=int, default=921600, help="serial baudrate")
    p.add_argument("--tilt-min", type=float, default=-45.0, help="scan min tilt [deg]")
    p.add_argument("--tilt-max", type=float, default=45.0, help="scan max tilt [deg]")
    p.add_argument("--step", type=float, default=2.0, help="scan step (speed hint) [deg]")
    p.add_argument(
        "--tilt-axis-offset-mm",
        type=float,
        default=23.0,
        help="sensor center offset above tilt rotation axis [mm]",
    )
    p.add_argument("--min-dist", type=int, default=20, help="minimum accepted distance [mm]")
    p.add_argument("--max-dist", type=int, default=12000, help="maximum accepted distance [mm]")
    p.add_argument("--min-quality", type=int, default=0, help="minimum accepted quality")
    p.add_argument("--window-points", type=int, default=300000, help="rolling point window")
    p.add_argument("--render-points", type=int, default=120000, help="max points to render")
    p.add_argument("--refresh-sec", type=float, default=0.25, help="viewer refresh interval")
    p.add_argument("--seed", type=int, default=0, help="sampling seed")
    p.add_argument("--equal-axis", action="store_true", help="equalize 3D axis scale")
    p.add_argument("--title", default="Lider Continuous Live Viewer", help="window title")
    p.add_argument(
        "--imu-ref-tilt-threshold-deg",
        type=float,
        default=1.0,
        help="lock IMU correction when |tilt| is within this threshold around 0 deg",
    )
    p.add_argument("--auto-recover", action="store_true", help="auto recover when scan appears stalled")
    p.add_argument(
        "--auto-recover-sec",
        type=float,
        default=3.5,
        help="consider motion stale when tilt end does not change for this duration [s]",
    )
    p.add_argument(
        "--no-slice-recover-sec",
        type=float,
        default=4.0,
        help="consider link stale when no slice arrives for this duration [s]",
    )
    p.add_argument(
        "--motion-delta-deg",
        type=float,
        default=0.2,
        help="minimum tilt end delta to treat as motion [deg]",
    )
    p.add_argument(
        "--motion-requires-no-slice-sec",
        type=float,
        default=1.0,
        help="apply no-motion recovery only when no-slice also exceeds this duration [s]",
    )
    p.add_argument(
        "--recovery-confirmations",
        type=int,
        default=3,
        help="required consecutive stale checks before triggering recovery",
    )
    p.add_argument(
        "--recovery-startup-grace-sec",
        type=float,
        default=4.0,
        help="disable recovery checks for this initial period after start [s]",
    )
    p.add_argument(
        "--recovery-cooldown-sec",
        type=float,
        default=6.0,
        help="minimum interval between recovery attempts [s]",
    )
    p.add_argument(
        "--clear-on-recover",
        dest="clear_on_recover",
        action="store_true",
        default=True,
        help="clear accumulated point buffers when recovery happens (default: enabled)",
    )
    p.add_argument(
        "--no-clear-on-recover",
        dest="clear_on_recover",
        action="store_false",
        help="keep accumulated points on recovery",
    )
    p.add_argument(
        "--direction-deadband-deg",
        type=float,
        default=0.05,
        help="tilt delta deadband for up/down direction classification [deg]",
    )
    return p


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.step <= 0:
        print("[error] --step must be positive", file=sys.stderr)
        return 2
    if args.refresh_sec <= 0.01:
        print("[error] --refresh-sec must be > 0.01", file=sys.stderr)
        return 2
    if args.render_points < 1000:
        print("[error] --render-points must be >= 1000", file=sys.stderr)
        return 2
    if args.auto_recover_sec <= 0.1 or args.no_slice_recover_sec <= 0.1:
        print("[error] recovery thresholds must be > 0.1", file=sys.stderr)
        return 2
    if args.recovery_confirmations < 1:
        print("[error] --recovery-confirmations must be >= 1", file=sys.stderr)
        return 2
    if args.recovery_startup_grace_sec < 0:
        print("[error] --recovery-startup-grace-sec must be >= 0", file=sys.stderr)
        return 2
    return run_live(args)


if __name__ == "__main__":
    raise SystemExit(main())
