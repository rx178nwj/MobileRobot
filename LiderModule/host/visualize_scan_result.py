#!/usr/bin/env python3
"""Visualize LiderModule scan result files (.csv / .ply)."""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path
from typing import Iterable, Optional

import numpy as np


def load_csv_points(path: Path) -> np.ndarray:
    rows: list[list[float]] = []
    with path.open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            raise ValueError("CSV is empty")
        for row in reader:
            if len(row) < 3:
                continue
            try:
                rows.append([float(row[0]), float(row[1]), float(row[2])])
            except ValueError:
                continue
    if not rows:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def load_ascii_ply_points(path: Path) -> np.ndarray:
    with path.open("r", encoding="utf-8") as f:
        lines = f.readlines()

    if not lines or lines[0].strip() != "ply":
        raise ValueError("Not a PLY file")
    if len(lines) < 3 or "format ascii" not in lines[1]:
        raise ValueError("Only ASCII PLY is supported")

    vertex_count: Optional[int] = None
    end_header_idx: Optional[int] = None
    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith("element vertex"):
            parts = s.split()
            if len(parts) >= 3:
                vertex_count = int(parts[2])
        if s == "end_header":
            end_header_idx = i
            break

    if end_header_idx is None:
        raise ValueError("PLY header missing end_header")
    if vertex_count is None:
        raise ValueError("PLY header missing vertex count")

    points: list[list[float]] = []
    for line in lines[end_header_idx + 1 : end_header_idx + 1 + vertex_count]:
        parts = line.strip().split()
        if len(parts) < 3:
            continue
        try:
            points.append([float(parts[0]), float(parts[1]), float(parts[2])])
        except ValueError:
            continue

    if not points:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(points, dtype=np.float64)


def load_points(path: Path) -> np.ndarray:
    suffix = path.suffix.lower()
    if suffix == ".csv":
        return load_csv_points(path)
    if suffix == ".ply":
        return load_ascii_ply_points(path)
    raise ValueError(f"Unsupported file type: {suffix} (use .csv or .ply)")


def sample_points(points: np.ndarray, max_points: int, seed: int) -> tuple[np.ndarray, np.ndarray]:
    n = len(points)
    if n <= max_points:
        idx = np.arange(n, dtype=np.int64)
        return points, idx
    rng = np.random.default_rng(seed)
    idx = np.sort(rng.choice(n, size=max_points, replace=False))
    return points[idx], idx


def summarize(points: np.ndarray) -> str:
    if len(points) == 0:
        return "points=0"
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    return (
        f"points={len(points)}  "
        f"x=[{mins[0]:.3f},{maxs[0]:.3f}]  "
        f"y=[{mins[1]:.3f},{maxs[1]:.3f}]  "
        f"z=[{mins[2]:.3f},{maxs[2]:.3f}]"
    )


def set_axes_equal(ax, pts: np.ndarray) -> None:
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


def show_matplotlib(points: np.ndarray, order_idx: np.ndarray, args: argparse.Namespace) -> None:
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle(args.title, fontsize=14)

    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    c = order_idx.astype(np.float64)
    sc = ax3d.scatter(points[:, 0], points[:, 1], points[:, 2], c=c, cmap="turbo", s=1)
    ax3d.set_title("3D Point Cloud (color = acquisition order)")
    ax3d.set_xlabel("X [m]")
    ax3d.set_ylabel("Y [m]")
    ax3d.set_zlabel("Z [m]")
    if args.equal_axis:
        set_axes_equal(ax3d, points)
    cb = fig.colorbar(sc, ax=ax3d, shrink=0.7)
    cb.set_label("Point index")

    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.scatter(points[:, 0], points[:, 1], c=c, cmap="turbo", s=1)
    ax_xy.set_title("Top View (X-Y)")
    ax_xy.set_xlabel("X [m]")
    ax_xy.set_ylabel("Y [m]")
    ax_xy.grid(True, alpha=0.3)

    ax_xz = fig.add_subplot(2, 2, 3)
    ax_xz.scatter(points[:, 0], points[:, 2], c=c, cmap="turbo", s=1)
    ax_xz.set_title("Side View (X-Z)")
    ax_xz.set_xlabel("X [m]")
    ax_xz.set_ylabel("Z [m]")
    ax_xz.grid(True, alpha=0.3)

    ax_stats = fig.add_subplot(2, 2, 4)
    ax_stats.axis("off")
    ax_stats.text(
        0.0,
        1.0,
        summarize(points),
        va="top",
        ha="left",
        family="monospace",
        fontsize=11,
    )

    fig.tight_layout()
    if args.save_png:
        fig.savefig(args.save_png, dpi=160)
        print(f"[save] matplotlib figure: {args.save_png}")
    if not args.no_show:
        plt.show()
    plt.close(fig)


def show_open3d(points: np.ndarray) -> None:
    import open3d as o3d

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    if len(points):
        idx = np.linspace(0.0, 1.0, len(points), dtype=np.float64)
        colors = np.zeros((len(points), 3), dtype=np.float64)
        colors[:, 0] = idx
        colors[:, 1] = 1.0 - np.abs(idx - 0.5) * 2.0
        colors[:, 2] = 1.0 - idx
        cloud.colors = o3d.utility.Vector3dVector(np.clip(colors, 0.0, 1.0))
    o3d.visualization.draw_geometries([cloud], window_name="Lider Scan Result")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visualize Lider scan result (.csv/.ply)")
    parser.add_argument("input", help="input scan file path (.csv or .ply)")
    parser.add_argument(
        "--mode",
        choices=["matplotlib", "open3d", "both"],
        default="matplotlib",
        help="visualization backend",
    )
    parser.add_argument("--max-points", type=int, default=120000, help="max points to render")
    parser.add_argument("--seed", type=int, default=0, help="sampling seed")
    parser.add_argument("--save-png", help="save matplotlib image path")
    parser.add_argument("--title", default="LiderModule Scan Visualization", help="plot title")
    parser.add_argument("--equal-axis", action="store_true", help="force equal 3D axis scale")
    parser.add_argument("--no-show", action="store_true", help="do not open matplotlib window")
    return parser


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    path = Path(args.input).expanduser()
    if not path.exists():
        print(f"[error] file not found: {path}", file=sys.stderr)
        return 2

    try:
        points = load_points(path)
    except Exception as exc:
        print(f"[error] failed to load points: {exc}", file=sys.stderr)
        return 1

    if len(points) == 0:
        print("[error] no points loaded", file=sys.stderr)
        return 1

    shown_points, order_idx = sample_points(points, max_points=args.max_points, seed=args.seed)
    print(f"[info] loaded: {path}")
    print(f"[info] summary: {summarize(points)}")
    if len(shown_points) != len(points):
        print(f"[info] sampled for rendering: {len(shown_points)} / {len(points)}")

    try:
        if args.mode in ("matplotlib", "both"):
            show_matplotlib(shown_points, order_idx, args)
        if args.mode in ("open3d", "both"):
            show_open3d(shown_points)
    except Exception as exc:
        print(f"[error] visualization failed: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
