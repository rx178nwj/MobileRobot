#!/usr/bin/env python3
"""Downward-focused continuous monitor for step/hole detection."""

from __future__ import annotations

import argparse
import queue
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Optional

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
class HazardResult:
    status: str
    reason: str
    ground_z_ref: float
    profile_x: np.ndarray
    profile_z: np.ndarray
    profile_count: np.ndarray
    step_events_x: np.ndarray
    hole_bins_x: np.ndarray


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


def compute_hazard(points: np.ndarray, args: argparse.Namespace) -> HazardResult:
    if len(points) < args.min_points_for_eval:
        return HazardResult(
            status="UNKNOWN",
            reason="insufficient points",
            ground_z_ref=0.0,
            profile_x=np.empty(0, dtype=np.float64),
            profile_z=np.empty(0, dtype=np.float64),
            profile_count=np.empty(0, dtype=np.int32),
            step_events_x=np.empty(0, dtype=np.float64),
            hole_bins_x=np.empty(0, dtype=np.float64),
        )

    # Ground reference from near-center ROI.
    roi_ref = points[
        (points[:, 0] >= args.ref_x_min)
        & (points[:, 0] <= args.ref_x_max)
        & (np.abs(points[:, 1]) <= args.ref_y_half)
    ]
    if len(roi_ref) < args.min_ref_points:
        return HazardResult(
            status="UNKNOWN",
            reason="not enough reference ROI points",
            ground_z_ref=0.0,
            profile_x=np.empty(0, dtype=np.float64),
            profile_z=np.empty(0, dtype=np.float64),
            profile_count=np.empty(0, dtype=np.int32),
            step_events_x=np.empty(0, dtype=np.float64),
            hole_bins_x=np.empty(0, dtype=np.float64),
        )
    ground_z_ref = float(np.percentile(roi_ref[:, 2], args.ref_percentile))

    lane = points[
        (points[:, 0] >= args.eval_x_min)
        & (points[:, 0] <= args.eval_x_max)
        & (np.abs(points[:, 1]) <= args.eval_y_half)
    ]
    if len(lane) < args.min_lane_points:
        return HazardResult(
            status="UNKNOWN",
            reason="not enough lane points",
            ground_z_ref=ground_z_ref,
            profile_x=np.empty(0, dtype=np.float64),
            profile_z=np.empty(0, dtype=np.float64),
            profile_count=np.empty(0, dtype=np.int32),
            step_events_x=np.empty(0, dtype=np.float64),
            hole_bins_x=np.empty(0, dtype=np.float64),
        )

    edges = np.arange(args.eval_x_min, args.eval_x_max + args.bin_size, args.bin_size)
    xc_list: list[float] = []
    z_list: list[float] = []
    c_list: list[int] = []
    for i in range(len(edges) - 1):
        x0 = edges[i]
        x1 = edges[i + 1]
        m = (lane[:, 0] >= x0) & (lane[:, 0] < x1)
        pts = lane[m]
        xc_list.append(0.5 * (x0 + x1))
        c_list.append(int(len(pts)))
        if len(pts) >= args.min_points_per_bin:
            z_list.append(float(np.percentile(pts[:, 2], args.bin_percentile)))
        else:
            z_list.append(np.nan)

    profile_x = np.asarray(xc_list, dtype=np.float64)
    profile_z = np.asarray(z_list, dtype=np.float64)
    profile_count = np.asarray(c_list, dtype=np.int32)

    valid = ~np.isnan(profile_z)
    if np.count_nonzero(valid) < 3:
        return HazardResult(
            status="UNKNOWN",
            reason="not enough valid bins",
            ground_z_ref=ground_z_ref,
            profile_x=profile_x,
            profile_z=profile_z,
            profile_count=profile_count,
            step_events_x=np.empty(0, dtype=np.float64),
            hole_bins_x=np.empty(0, dtype=np.float64),
        )

    # Step-down: sudden negative jump in local floor profile.
    zf = profile_z.copy()
    nan_idx = np.isnan(zf)
    if np.any(nan_idx):
        zf[nan_idx] = np.interp(profile_x[nan_idx], profile_x[~nan_idx], zf[~nan_idx])
    dz = np.diff(zf)
    step_mask = dz < -args.step_drop_m
    step_events_x = profile_x[1:][step_mask]

    # Hole: sparse bins with strong drop wrt reference floor.
    deep_mask = (profile_z < (ground_z_ref - args.hole_depth_m)) & valid
    sparse_mask = profile_count < args.hole_sparse_points
    hole_mask = deep_mask | (sparse_mask & (profile_x > args.hole_x_min))
    hole_bins_x = profile_x[hole_mask]

    status = "SAFE"
    reason = "flat floor profile"
    if len(hole_bins_x) >= args.min_hole_bins:
        status = "HOLE"
        reason = f"hole-like bins={len(hole_bins_x)}"
    elif len(step_events_x) >= args.min_step_events:
        status = "STEP_DOWN"
        reason = f"step drops={len(step_events_x)}"

    return HazardResult(
        status=status,
        reason=reason,
        ground_z_ref=ground_z_ref,
        profile_x=profile_x,
        profile_z=profile_z,
        profile_count=profile_count,
        step_events_x=step_events_x,
        hole_bins_x=hole_bins_x,
    )


def render_summary(points: np.ndarray, hz: HazardResult, args: argparse.Namespace) -> Optional[Path]:
    if not args.save_png:
        return None

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle("Downward Hazard Monitor Summary", fontsize=14)
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax_xz = fig.add_subplot(2, 2, 2)
    ax_prof = fig.add_subplot(2, 2, 3)
    ax_info = fig.add_subplot(2, 2, 4)
    ax_info.axis("off")

    if len(points):
        n = len(points)
        if n > args.render_points:
            rng = np.random.default_rng(args.seed)
            idx = np.sort(rng.choice(n, size=args.render_points, replace=False))
            shown = points[idx]
        else:
            shown = points
        c = shown[:, 2]
        ax3d.scatter(shown[:, 0], shown[:, 1], shown[:, 2], c=c, cmap="viridis", s=1)
        ax_xz.scatter(shown[:, 0], shown[:, 2], c=c, cmap="viridis", s=1)

    ax3d.set_title("3D Point Cloud")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")

    ax_xz.set_title("Forward Height View (X-Z)")
    ax_xz.set_xlabel("X [m]")
    ax_xz.set_ylabel("Z [m]")
    ax_xz.grid(True, alpha=0.3)

    ax_prof.set_title("Floor Profile + Hazards")
    ax_prof.set_xlabel("X [m]")
    ax_prof.set_ylabel("Z [m]")
    ax_prof.grid(True, alpha=0.3)
    if len(hz.profile_x):
        ax_prof.plot(hz.profile_x, hz.profile_z, color="tab:blue", marker="o", markersize=3, linewidth=1)
        ax_prof.axhline(hz.ground_z_ref, color="green", linestyle="--", linewidth=1, label="ground ref")
        ax_prof.axhline(
            hz.ground_z_ref - args.hole_depth_m,
            color="red",
            linestyle="--",
            linewidth=1,
            label="hole threshold",
        )
        if len(hz.step_events_x):
            ys = np.interp(hz.step_events_x, hz.profile_x, np.nan_to_num(hz.profile_z, nan=hz.ground_z_ref))
            ax_prof.scatter(hz.step_events_x, ys, color="orange", marker="x", s=44, label="step-down")
        if len(hz.hole_bins_x):
            ys = np.interp(hz.hole_bins_x, hz.profile_x, np.nan_to_num(hz.profile_z, nan=hz.ground_z_ref))
            ax_prof.scatter(hz.hole_bins_x, ys, color="red", marker="D", s=36, label="hole")
        ax_prof.legend(loc="best", fontsize=8)

    info = (
        f"status         : {hz.status}\n"
        f"reason         : {hz.reason}\n"
        f"points         : {len(points)}\n"
        f"ground_z_ref   : {hz.ground_z_ref:.3f} m\n"
        f"step events    : {len(hz.step_events_x)}\n"
        f"hole bins      : {len(hz.hole_bins_x)}\n"
        f"scan range     : {args.tilt_min:.1f}..{args.tilt_max:.1f} deg\n"
        f"scan step hint : {args.step:.1f} deg"
    )
    ax_info.text(0.0, 1.0, info, va="top", ha="left", family="monospace", fontsize=11)

    out = Path(args.save_png).expanduser()
    fig.tight_layout()
    fig.savefig(out, dpi=160)
    plt.close(fig)
    return out


def run_monitor(args: argparse.Namespace) -> int:
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

    buf = RollingPointBuffer(max_points=args.window_points)
    imu_ref_pitch: Optional[float] = None
    imu_ref_roll: Optional[float] = None
    last_status = (0, 0)
    slices = 0
    points_total = 0
    status_print_ts = 0.0

    try:
        send_scan_stop(ser)
        time.sleep(0.1)
        send_scan_start(ser, args.tilt_min, args.tilt_max, args.step)
        print(
            f"[monitor] started port={args.port} range={args.tilt_min}..{args.tilt_max} step={args.step}",
            flush=True,
        )

        t0 = time.monotonic()
        while (time.monotonic() - t0) < args.duration_sec:
            try:
                event = events.get(timeout=0.2)
            except queue.Empty:
                event = None

            if isinstance(event, ScanSlice):
                tilt_mid = 0.5 * (event.tilt_start_deg + event.tilt_end_deg)
                if imu_ref_pitch is None and abs(tilt_mid) <= args.imu_ref_tilt_threshold_deg:
                    imu_ref_pitch = event.imu_pitch_deg
                    imu_ref_roll = event.imu_roll_deg
                    print(
                        f"[imu] reference locked at tilt~0: pitch={imu_ref_pitch:.2f} roll={imu_ref_roll:.2f}",
                        flush=True,
                    )
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
                buf.append(pts)
                slices += 1
                points_total += len(pts)

            elif isinstance(event, ScanStatus):
                last_status = (event.step, event.total)
                if event.state == 2:
                    send_scan_start(ser, args.tilt_min, args.tilt_max, args.step)

            elif isinstance(event, ImuData):
                pass

            elif isinstance(event, ValueError):
                print(f"[warn] {event}", file=sys.stderr)

            now = time.monotonic()
            if now - status_print_ts >= args.print_interval_sec:
                status_print_ts = now
                pts = buf.stacked()
                hz = compute_hazard(pts, args)
                print(
                    f"[hazard] status={hz.status:<9} reason={hz.reason} "
                    f"points={len(pts)} slices={slices} status={last_status[0]}/{last_status[1]}",
                    flush=True,
                )

        points = buf.stacked()
        hz = compute_hazard(points, args)
        print(
            f"[result] status={hz.status} reason={hz.reason} "
            f"points={len(points)} slices={slices} accepted_total={points_total}",
            flush=True,
        )
        out = render_summary(points, hz, args)
        if out is not None:
            print(f"[save] summary image: {out}", flush=True)
    finally:
        try:
            send_scan_stop(ser)
        except Exception:
            pass
        stop_event.set()
        reader.join(timeout=1.0)
        ser.close()

    return 0


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Downward continuous hazard monitor (step/hole)")
    p.add_argument("--port", default="/dev/ttyACM0", help="serial port")
    p.add_argument("--baud", type=int, default=921600, help="serial baudrate")
    p.add_argument("--tilt-min", type=float, default=-45.0, help="scan min tilt [deg]")
    p.add_argument("--tilt-max", type=float, default=0.0, help="scan max tilt [deg]")
    p.add_argument("--step", type=float, default=2.0, help="scan step hint [deg]")
    p.add_argument("--duration-sec", type=float, default=20.0, help="monitor duration [s]")
    p.add_argument("--print-interval-sec", type=float, default=1.0, help="status print interval [s]")
    p.add_argument("--window-points", type=int, default=220000, help="rolling point buffer size")
    p.add_argument("--render-points", type=int, default=120000, help="max points to render in summary")
    p.add_argument("--seed", type=int, default=0, help="sampling seed")
    p.add_argument("--save-png", help="summary image output path")

    p.add_argument("--min-dist", type=int, default=20, help="minimum accepted distance [mm]")
    p.add_argument("--max-dist", type=int, default=12000, help="maximum accepted distance [mm]")
    p.add_argument("--min-quality", type=int, default=0, help="minimum accepted quality")
    p.add_argument("--tilt-axis-offset-mm", type=float, default=23.0, help="sensor center offset [mm]")
    p.add_argument(
        "--imu-ref-tilt-threshold-deg",
        type=float,
        default=1.0,
        help="lock IMU correction near |tilt|<=threshold around 0 deg",
    )

    p.add_argument("--ref-x-min", type=float, default=0.20, help="ground reference ROI x-min [m]")
    p.add_argument("--ref-x-max", type=float, default=1.20, help="ground reference ROI x-max [m]")
    p.add_argument("--ref-y-half", type=float, default=0.20, help="ground reference ROI |y| limit [m]")
    p.add_argument("--ref-percentile", type=float, default=20.0, help="ground reference z percentile")
    p.add_argument("--min-ref-points", type=int, default=120, help="minimum ref points")

    p.add_argument("--eval-x-min", type=float, default=0.25, help="hazard eval x-min [m]")
    p.add_argument("--eval-x-max", type=float, default=1.80, help="hazard eval x-max [m]")
    p.add_argument("--eval-y-half", type=float, default=0.22, help="hazard eval |y| limit [m]")
    p.add_argument("--bin-size", type=float, default=0.08, help="forward bin size [m]")
    p.add_argument("--bin-percentile", type=float, default=15.0, help="bin floor z percentile")
    p.add_argument("--min-points-per-bin", type=int, default=20, help="minimum points for valid bin")
    p.add_argument("--min-lane-points", type=int, default=280, help="minimum lane points")
    p.add_argument("--min-points-for-eval", type=int, default=1000, help="minimum total points for evaluation")

    p.add_argument("--step-drop-m", type=float, default=0.12, help="step-down z jump threshold [m]")
    p.add_argument("--min-step-events", type=int, default=1, help="minimum step events for STEP_DOWN")
    p.add_argument("--hole-depth-m", type=float, default=0.18, help="depth threshold below ground ref [m]")
    p.add_argument("--hole-sparse-points", type=int, default=10, help="sparse bin threshold")
    p.add_argument("--hole-x-min", type=float, default=0.35, help="hole sparse test valid x-min [m]")
    p.add_argument("--min-hole-bins", type=int, default=2, help="minimum bins for HOLE")
    return p


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.step <= 0:
        print("[error] --step must be positive", file=sys.stderr)
        return 2
    if args.duration_sec <= 0:
        print("[error] --duration-sec must be positive", file=sys.stderr)
        return 2
    if args.eval_x_max <= args.eval_x_min:
        print("[error] eval x-range must satisfy x_max > x_min", file=sys.stderr)
        return 2
    return run_monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
