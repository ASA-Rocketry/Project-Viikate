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

    def update(self, item: Telemetry) -> None:
        with self.lock:
            self.latest = item
            self.t.append(item.timestamp)
            self.alt.append(item.altitude)
            self.vv.append(item.vertical_velocity)

    def snapshot(
        self,
    ) -> Tuple[Optional[Telemetry], List[float], List[float], List[float]]:
        with self.lock:
            return self.latest, list(self.t), list(self.alt), list(self.vv)


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
        latest, t_hist, alt_hist, vv_hist = store.snapshot()

        if latest is None:
            return line_alt, line_vv

        if t_hist:
            t_end = t_hist[-1]
            t_start = max(0.0, t_end - HISTORY_SECONDS)
            start_index = next(
                (i for i, t in enumerate(t_hist) if t >= t_start),
                0,
            )
            t_plot = t_hist[start_index:]
            alt_plot = alt_hist[start_index:]
            vv_plot = vv_hist[start_index:]
        else:
            t_plot = []
            alt_plot = []
            vv_plot = []

        line_alt.set_data(t_plot, alt_plot)
        line_vv.set_data(t_plot, vv_plot)

        if t_plot:
            ax_alt.set_xlim(t_plot[0], max(HISTORY_SECONDS, t_plot[-1]))

        set_dynamic_ylim(ax_alt, alt_plot, min_span=1.0)
        set_dynamic_ylim(ax_vv, vv_plot, min_span=1.0)

        ax3d.cla()

        alt_now = latest.altitude

        # Assumes Orientation (x, y, z) = roll, pitch, yaw in degrees.
        roll, pitch, yaw = latest.orientation

        direction = rotation_matrix_from_euler(
            roll,
            pitch,
            yaw,
        ) @ np.array([0.0, 0.0, 1.0])

        norm = np.linalg.norm(direction)
        if norm > 0:
            direction = direction / norm
        else:
            direction = np.array([0.0, 0.0, 1.0])

        base = np.array([0.0, 0.0, alt_now])
        tip = base + direction * ROCKET_LENGTH

        pad_half = 1.5
        gx = np.array([[-pad_half, pad_half], [-pad_half, pad_half]])
        gy = np.array([[-pad_half, -pad_half], [pad_half, pad_half]])
        gz = np.zeros((2, 2))

        ax3d.plot_surface(gx, gy, gz, color="lightgreen", alpha=0.18)
        ax3d.plot(
            [0.0, 0.0],
            [0.0, 0.0],
            [0.0, alt_now],
            "--",
            color="gray",
            lw=1,
        )

        ax3d.plot(
            [base[0], tip[0]],
            [base[1], tip[1]],
            [base[2], tip[2]],
            color="crimson",
            lw=4,
        )
        ax3d.scatter([base[0]], [base[1]], [base[2]], color="black", s=20)
        ax3d.scatter([tip[0]], [tip[1]], [tip[2]], color="navy", s=35)

        z_low = min(0.0, alt_now - 5.0)
        z_high = max(10.0, alt_now + 5.0)

        ax3d.set_xlim(-2.0, 2.0)
        ax3d.set_ylim(-2.0, 2.0)
        ax3d.set_zlim(z_low, z_high)
        ax3d.set_box_aspect((1, 1, 2))
        ax3d.view_init(elev=20, azim=45)

        ax3d.set_title("3D Rocket Attitude")
        ax3d.set_xlabel("X")
        ax3d.set_ylabel("Y")
        ax3d.set_zlabel("Altitude")

        info = (
            f"Altitude:           {latest.altitude:8.2f}\n"
            f"Vertical Velocity:  {latest.vertical_velocity:8.2f}\n"
            f"AccelMagnitude:     {latest.accel_magnitude:8.2f}\n"
            f"Acc (x,y,z):        {latest.acc[0]:8.2f},"
            f" {latest.acc[1]:8.2f}, {latest.acc[2]:8.2f}\n"
            f"Gyro (x,y,z):       {latest.gyro[0]:8.2f},"
            f" {latest.gyro[1]:8.2f}, {latest.gyro[2]:8.2f}\n"
            f"Orientation:        {roll:8.2f}, {pitch:8.2f},"
            f" {yaw:8.2f}\n"
            f"Heading:            {latest.heading:8.2f}"
        )

        ax3d.text2D(
            0.03,
            0.97,
            info,
            transform=ax3d.transAxes,
            va="top",
            family="monospace",
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

    # Prevent unused variable warning and keep animation alive.
    _ = ani


if __name__ == "__main__":
    main()
