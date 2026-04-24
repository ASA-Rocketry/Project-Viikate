#!/usr/bin/env python3
import argparse
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation

FLOAT_RE = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
ROCKET_LENGTH = 1.2
ROLL_MARKER_SIZE = 0.22
HISTORY_SECONDS = 30.0


@dataclass
class Telemetry:
    altitude: float = 0.0
    vertical_velocity: float = 0.0
    accel_z: float = 0.0
    rot_z: float = 0.0
    accel_magnitude: float = 0.0
    acc: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gyro: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    mag: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    heading: float = 0.0
    timestamp: float = 0.0


class TelemetryStore:
    def __init__(self, max_points: int = 1200) -> None:
        self.lock = threading.Lock()
        self.latest: Optional[Telemetry] = None
        self.t: Deque[float] = deque(maxlen=max_points)
        self.alt: Deque[float] = deque(maxlen=max_points)
        self.vv: Deque[float] = deque(maxlen=max_points)
        self.orientation: Deque[Tuple[float, float, float]] = deque(
            maxlen=max_points
        )

    def update(self, item: Telemetry) -> None:
        with self.lock:
            self.latest = item
            self.t.append(item.timestamp)
            self.alt.append(item.altitude)
            self.vv.append(item.vertical_velocity)
            self.orientation.append(item.orientation)

    def snapshot(
        self,
    ) -> Tuple[
        Optional[Telemetry],
        List[float],
        List[float],
        List[float],
        List[Tuple[float, float, float]],
    ]:
        with self.lock:
            return (
                self.latest,
                list(self.t),
                list(self.alt),
                list(self.vv),
                list(self.orientation),
            )


def extract_float(label: str, text: str, default: float = 0.0) -> float:
    pattern = rf"{re.escape(label)}:\s*({FLOAT_RE})"
    match = re.search(pattern, text)
    return float(match.group(1)) if match else default


def extract_triplet(
    label: str,
    text: str,
    default: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Tuple[float, float, float]:
    pattern = (
        rf"{re.escape(label)}:\s*({FLOAT_RE})\s*,\s*({FLOAT_RE})\s*,\s*"
        rf"({FLOAT_RE})"
    )
    match = re.search(pattern, text)
    if not match:
        return default

    return (
        float(match.group(1)),
        float(match.group(2)),
        float(match.group(3)),
    )


def parse_frame(frame_text: str, timestamp: float) -> Optional[Telemetry]:
    if "Altitude:" not in frame_text:
        return None

    return Telemetry(
        altitude=extract_float("Altitude", frame_text),
        vertical_velocity=extract_float("Vertical Velocity", frame_text),
        accel_z=extract_float("AccelZ", frame_text),
        rot_z=extract_float("RotZ", frame_text),
        accel_magnitude=extract_float("AccelMagnitude", frame_text),
        acc=extract_triplet("Acc (x, y, z)", frame_text),
        gyro=extract_triplet("Gyro Angular Rate (x, y, z)", frame_text),
        orientation=extract_triplet("Orientation (x, y, z)", frame_text),
        mag=extract_triplet("Mag", frame_text),
        heading=extract_float("Heading", frame_text),
        timestamp=timestamp,
    )


def is_separator(line: str) -> bool:
    return bool(line) and set(line) == {"-"}


def read_serial_frames(port: str, baud: int, store: TelemetryStore) -> None:
    try:
        ser = serial.Serial(port, baud, timeout=0.2)
    except serial.SerialException as exc:
        print(f"Could not open serial port {port}: {exc}")
        return

    print(f"Connected to {port} @ {baud} baud")
    start_time = time.time()
    lines: List[str] = []

    while True:
        try:
            raw = ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            if is_separator(line):
                frame_text = "\n".join(lines)
                lines.clear()

                telemetry = parse_frame(
                    frame_text,
                    time.time() - start_time,
                )
                if telemetry is not None:
                    store.update(telemetry)
            else:
                lines.append(line)

        except serial.SerialException as exc:
            print(f"Serial error: {exc}")
            break
        except Exception as exc:
            print(f"Parse error: {exc}")
            lines.clear()


def rotation_matrix_from_euler(
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
) -> np.ndarray:
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw = np.radians(yaw_deg)

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, cr, -sr],
            [0.0, sr, cr],
        ]
    )

    ry = np.array(
        [
            [cp, 0.0, sp],
            [0.0, 1.0, 0.0],
            [-sp, 0.0, cp],
        ]
    )

    rz = np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    return rz @ ry @ rx


def normalize(vec: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm


def set_dynamic_ylim(
    ax: plt.Axes,
    values: List[float],
    min_span: float = 1.0,
) -> None:
    if not values:
        return

    vmin = min(values)
    vmax = max(values)
    span = max(vmax - vmin, min_span)
    pad = span * 0.15
    center = 0.5 * (vmin + vmax)

    ax.set_ylim(center - span / 2.0 - pad, center + span / 2.0 + pad)


def trim_history(
    t_hist: List[float],
    alt_hist: List[float],
    vv_hist: List[float],
    orientation_hist: List[Tuple[float, float, float]],
) -> Tuple[
    List[float],
    List[float],
    List[float],
    List[Tuple[float, float, float]],
]:
    if not t_hist:
        return [], [], [], []

    t_end = t_hist[-1]
    t_start = max(0.0, t_end - HISTORY_SECONDS)

    start_index = next((i for i, t in enumerate(t_hist) if t >= t_start), 0)

    return (
        t_hist[start_index:],
        alt_hist[start_index:],
        vv_hist[start_index:],
        orientation_hist[start_index:],
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Real-time rocket serial visualizer"
    )
    parser.add_argument(
        "--port",
        required=True,
        help="Serial port, e.g. COM5 or /dev/ttyUSB0",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate",
    )
    args = parser.parse_args()

    store = TelemetryStore()

    reader_thread = threading.Thread(
        target=read_serial_frames,
        args=(args.port, args.baud, store),
        daemon=True,
    )
    reader_thread.start()

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1.0])

    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_vv = fig.add_subplot(gs[1, 1], sharex=ax_alt)

    line_alt, = ax_alt.plot([], [], color="tab:blue", lw=2)
    line_vv, = ax_vv.plot([], [], color="tab:orange", lw=2)

    ax_alt.set_title("Altitude")
    ax_alt.set_ylabel("m")
    ax_alt.grid(True, alpha=0.3)

    ax_vv.set_title("Vertical Velocity")
    ax_vv.set_xlabel("Time (s)")
    ax_vv.set_ylabel("m/s")
    ax_vv.grid(True, alpha=0.3)

    fig.suptitle("Rocket Telemetry Visualizer")

    def update(_frame_index: int):
        latest, t_hist, alt_hist, vv_hist, orientation_hist = store.snapshot()

        if latest is None:
            return line_alt, line_vv

        t_plot, alt_plot, vv_plot, orientation_plot = trim_history(
            t_hist,
            alt_hist,
            vv_hist,
            orientation_hist,
        )

        line_alt.set_data(t_plot, alt_plot)
        line_vv.set_data(t_plot, vv_plot)

        if t_plot:
            x_left = max(0.0, t_plot[-1] - HISTORY_SECONDS)
            x_right = max(HISTORY_SECONDS, t_plot[-1])
            ax_alt.set_xlim(x_left, x_right)

        set_dynamic_ylim(ax_alt, alt_plot, min_span=1.0)
        set_dynamic_ylim(ax_vv, vv_plot, min_span=1.0)

        ax3d.cla()

        alt_now = latest.altitude

        # Assumes Orientation (x, y, z) = roll, pitch, yaw in degrees.
        roll, pitch, yaw = latest.orientation

        rotation = rotation_matrix_from_euler(roll, pitch, yaw)
        body_x = normalize(rotation @ np.array([1.0, 0.0, 0.0]))
        body_y = normalize(rotation @ np.array([0.0, 1.0, 0.0]))
        body_z = normalize(rotation @ np.array([0.0, 0.0, 1.0]))

        base = np.array([0.0, 0.0, alt_now])
        tip = base + body_z * ROCKET_LENGTH
        roll_center = base + body_z * (ROCKET_LENGTH * 0.45)

        # Build path history.
        base_path = np.column_stack(
            (
                np.zeros(len(alt_plot)),
                np.zeros(len(alt_plot)),
                np.array(alt_plot),
            )
        )

        tip_points = []
        for hist_alt, hist_orientation in zip(alt_plot, orientation_plot):
            hist_roll, hist_pitch, hist_yaw = hist_orientation
            hist_rotation = rotation_matrix_from_euler(
                hist_roll,
                hist_pitch,
                hist_yaw,
            )
            hist_body_z = normalize(
                hist_rotation @ np.array([0.0, 0.0, 1.0])
            )
            hist_base = np.array([0.0, 0.0, hist_alt])
            hist_tip = hist_base + hist_body_z * ROCKET_LENGTH
            tip_points.append(hist_tip)

        if tip_points:
            tip_path = np.vstack(tip_points)
        else:
            tip_path = np.empty((0, 3))

        # Ground plane.
        ground_half = 1.5
        gx = np.array(
            [
                [-ground_half, ground_half],
                [-ground_half, ground_half],
            ]
        )
        gy = np.array(
            [
                [-ground_half, -ground_half],
                [ground_half, ground_half],
            ]
        )
        gz = np.zeros((2, 2))
        ax3d.plot_surface(gx, gy, gz, color="lightgreen", alpha=0.18)

        # Known path of rocket base from altitude only.
        if len(base_path) > 1:
            ax3d.plot(
                base_path[:, 0],
                base_path[:, 1],
                base_path[:, 2],
                "--",
                color="gray",
                lw=1.8,
                alpha=0.9,
            )

        # Nose trail to visualize where the rocket has pointed.
        if len(tip_path) > 1:
            ax3d.plot(
                tip_path[:, 0],
                tip_path[:, 1],
                tip_path[:, 2],
                color="purple",
                lw=2.0,
                alpha=0.9,
            )

        # Current rocket body.
        ax3d.plot(
            [base[0], tip[0]],
            [base[1], tip[1]],
            [base[2], tip[2]],
            color="crimson",
            lw=4,
        )
        ax3d.scatter([base[0]], [base[1]], [base[2]], color="black", s=20)
        ax3d.scatter([tip[0]], [tip[1]], [tip[2]], color="navy", s=35)

        # Roll markers so roll is visible around the rocket's own axis.
        x1 = roll_center - body_x * ROLL_MARKER_SIZE
        x2 = roll_center + body_x * ROLL_MARKER_SIZE
        y1 = roll_center - body_y * ROLL_MARKER_SIZE
        y2 = roll_center + body_y * ROLL_MARKER_SIZE

        ax3d.plot(
            [x1[0], x2[0]],
            [x1[1], x2[1]],
            [x1[2], x2[2]],
            color="darkorange",
            lw=3,
        )
        ax3d.plot(
            [y1[0], y2[0]],
            [y1[1], y2[1]],
            [y1[2], y2[2]],
            color="seagreen",
            lw=3,
        )

        alt_min = min(alt_plot) if alt_plot else alt_now
        alt_max = max(alt_plot) if alt_plot else alt_now

        z_low = min(0.0, alt_min - 5.0)
        z_high = max(10.0, alt_max + 5.0)

        xy_extent = max(2.0, ROCKET_LENGTH + 0.6)

        ax3d.set_xlim(-xy_extent, xy_extent)
        ax3d.set_ylim(-xy_extent, xy_extent)
        ax3d.set_zlim(z_low, z_high)
        ax3d.set_box_aspect((1, 1, 2))
        ax3d.view_init(elev=20, azim=45)

        ax3d.set_title("3D Rocket Attitude + Path")
        ax3d.set_xlabel("X")
        ax3d.set_ylabel("Y")
        ax3d.set_zlabel("Altitude")

        info = (
            f"Altitude:           {latest.altitude:8.2f}\n"
            f"Vertical Velocity:  {latest.vertical_velocity:8.2f}\n"
            f"Roll:               {roll:8.2f} deg\n"
            f"Pitch:              {pitch:8.2f} deg\n"
            f"Yaw:                {yaw:8.2f} deg\n"
            f"Heading:            {latest.heading:8.2f} deg\n"
            f"AccelMagnitude:     {latest.accel_magnitude:8.2f}\n"
            f"\n"
            f"Gray dashed: base path\n"
            f"Purple: nose trail\n"
            f"Orange/green: roll markers"
        )

        ax3d.text2D(
            0.03,
            0.97,
            info,
            transform=ax3d.transAxes,
            va="top",
            family="monospace",
            bbox={
                "facecolor": "white",
                "alpha": 0.75,
                "edgecolor": "none",
            },
        )

        return line_alt, line_vv

    ani = FuncAnimation(
        fig,
        update,
        interval=50,
        cache_frame_data=False,
    )

    plt.tight_layout()
    plt.show()

    _ = ani


if __name__ == "__main__":
    main()
