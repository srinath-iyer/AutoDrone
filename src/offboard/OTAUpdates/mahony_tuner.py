#!/usr/bin/env python3
"""
Live Mahony filter tuner for ESP32 logs.

Reads MPU6050 lines from serial or a log file and runs a local Mahony filter copy
with interactive sliders for Kp/Ki and robustness gates.

Expected input line examples:
  00:46:56.812 > MPU6050_READING:ax,ay,az,gx,gy,gz,temp
  00:46:56.812 > POSE_READING:x,y,z,roll,pitch,yaw
"""

from __future__ import annotations

import argparse
import math
import queue
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider

try:
    import serial
except Exception:
    serial = None


MPU_RE = re.compile(
    r"MPU6050_READING:\s*"
    r"([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),"
    r"([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)"
)
POSE_RE = re.compile(
    r"POSE_READING:\s*"
    r"([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),"
    r"([-+]?\d*\.?\d+),([-+]?\d*\.?\d+),([-+]?\d*\.?\d+)"
)

DEFAULT_LOG_PATH = Path(__file__).with_name("log.txt")
TIMESTAMP_RE = re.compile(r"(\d{2}):(\d{2}):(\d{2})\.(\d{3})")


@dataclass
class MahonyParams:
    kp: float = 2.2
    ki: float = 0.05
    int_lim: float = 0.35
    gyro_clip_dps: float = 700.0
    accel_min_g: float = 0.75
    accel_max_g: float = 1.25
    max_angle_rate_dps: float = 800.0


@dataclass
class MpuSample:
    t_rel: float
    dt: float
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


class MahonyFilter:
    def __init__(self) -> None:
        self.qw = 1.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.iex = 0.0
        self.iey = 0.0
        self.iez = 0.0
        self.last_roll = 0.0
        self.last_pitch = 0.0
        self.last_yaw = 0.0
        self.has_last = False

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def wrap180(deg: float) -> float:
        while deg > 180.0:
            deg -= 360.0
        while deg <= -180.0:
            deg += 360.0
        return deg

    @staticmethod
    def delta_angle(cur: float, prev: float) -> float:
        return MahonyFilter.wrap180(cur - prev)

    def normalize_quat(self) -> None:
        if not all(math.isfinite(v) for v in (self.qw, self.qx, self.qy, self.qz)):
            self.qw, self.qx, self.qy, self.qz = 1.0, 0.0, 0.0, 0.0
            return

        self.qw = self.clamp(self.qw, -4.0, 4.0)
        self.qx = self.clamp(self.qx, -4.0, 4.0)
        self.qy = self.clamp(self.qy, -4.0, 4.0)
        self.qz = self.clamp(self.qz, -4.0, 4.0)

        n = math.sqrt(self.qw * self.qw + self.qx * self.qx + self.qy * self.qy + self.qz * self.qz)
        if (not math.isfinite(n)) or n < 1e-6 or n > 8.0:
            self.qw, self.qx, self.qy, self.qz = 1.0, 0.0, 0.0, 0.0
            return

        inv_n = 1.0 / n
        self.qw *= inv_n
        self.qx *= inv_n
        self.qy *= inv_n
        self.qz *= inv_n

    def update(
        self,
        ax: float,
        ay: float,
        az: float,
        gx_dps: float,
        gy_dps: float,
        gz_dps: float,
        dt: float,
        p: MahonyParams,
    ) -> tuple[float, float, float]:
        gx_dps = self.clamp(gx_dps, -p.gyro_clip_dps, p.gyro_clip_dps)
        gy_dps = self.clamp(gy_dps, -p.gyro_clip_dps, p.gyro_clip_dps)
        gz_dps = self.clamp(gz_dps, -p.gyro_clip_dps, p.gyro_clip_dps)

        anorm = math.sqrt(ax * ax + ay * ay + az * az)
        if anorm > 1e-5:
            g_ratio = anorm / 9.81
            if p.accel_min_g <= g_ratio <= p.accel_max_g:
                ax /= anorm
                ay /= anorm
                az /= anorm

                vx = 2.0 * (self.qx * self.qz - self.qw * self.qy)
                vy = 2.0 * (self.qw * self.qx + self.qy * self.qz)
                vz = self.qw * self.qw - self.qx * self.qx - self.qy * self.qy + self.qz * self.qz

                ex = ay * vz - az * vy
                ey = az * vx - ax * vz
                ez = ax * vy - ay * vx

                self.iex = self.clamp(self.iex + ex * dt, -p.int_lim, p.int_lim)
                self.iey = self.clamp(self.iey + ey * dt, -p.int_lim, p.int_lim)
                self.iez = self.clamp(self.iez + ez * dt, -p.int_lim, p.int_lim)

                gx_dps += p.kp * ex + p.ki * self.iex
                gy_dps += p.kp * ey + p.ki * self.iey
                gz_dps += p.kp * ez + p.ki * self.iez
            else:
                self.iex *= 0.995
                self.iey *= 0.995
                self.iez *= 0.995

        gx = math.radians(gx_dps)
        gy = math.radians(gy_dps)
        gz = math.radians(gz_dps)
        half_dt = 0.5 * dt

        dq_w = (-self.qx * gx - self.qy * gy - self.qz * gz) * half_dt
        dq_x = (self.qw * gx + self.qy * gz - self.qz * gy) * half_dt
        dq_y = (self.qw * gy - self.qx * gz + self.qz * gx) * half_dt
        dq_z = (self.qw * gz + self.qx * gy - self.qy * gx) * half_dt

        self.qw += dq_w
        self.qx += dq_x
        self.qy += dq_y
        self.qz += dq_z
        self.normalize_quat()

        roll = math.degrees(math.atan2(2.0 * (self.qw * self.qx + self.qy * self.qz), 1.0 - 2.0 * (self.qx * self.qx + self.qy * self.qy)))

        sinp = 2.0 * (self.qw * self.qy - self.qz * self.qx)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(90.0, sinp)
        else:
            pitch = math.degrees(math.asin(sinp))

        yaw = math.degrees(math.atan2(2.0 * (self.qw * self.qz + self.qx * self.qy), 1.0 - 2.0 * (self.qy * self.qy + self.qz * self.qz)))

        roll = self.wrap180(roll)
        pitch = self.wrap180(pitch)
        yaw = self.wrap180(yaw)

        if self.has_last:
            max_step = max(0.3, p.max_angle_rate_dps * dt)
            roll = self.wrap180(self.last_roll + self.clamp(self.delta_angle(roll, self.last_roll), -max_step, max_step))
            pitch = self.wrap180(self.last_pitch + self.clamp(self.delta_angle(pitch, self.last_pitch), -max_step, max_step))
            yaw = self.wrap180(self.last_yaw + self.clamp(self.delta_angle(yaw, self.last_yaw), -max_step, max_step))

        self.last_roll = roll
        self.last_pitch = pitch
        self.last_yaw = yaw
        self.has_last = True

        return roll, pitch, yaw


def serial_line_source(port: str, baud: int, q: queue.Queue[str], stop_event: threading.Event) -> None:
    if serial is None:
        raise RuntimeError("pyserial is not installed. Install with: pip install pyserial")

    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as ser:
        while not stop_event.is_set():
            raw = ser.readline()
            if not raw:
                continue
            try:
                q.put_nowait(raw.decode("utf-8", errors="replace").strip())
            except queue.Full:
                pass


def file_line_source(path: Path, q: queue.Queue[str], stop_event: threading.Event, loop: bool) -> None:
    while not stop_event.is_set():
        with path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if stop_event.is_set():
                    return
                text = line.strip()
                if text:
                    try:
                        q.put_nowait(text)
                    except queue.Full:
                        pass
                time.sleep(0.002)
        if not loop:
            return


def resolve_log_path(log_path: Path | None) -> Path | None:
    if log_path is not None:
        return log_path
    if DEFAULT_LOG_PATH.exists():
        return DEFAULT_LOG_PATH
    return None


def extract_log_timestamp(line: str, fallback_time: float, start_timestamp: float | None) -> tuple[float, float | None]:
    match = TIMESTAMP_RE.search(line)
    if not match:
        return fallback_time, start_timestamp

    hours, minutes, seconds, millis = (int(part) for part in match.groups())
    absolute_seconds = hours * 3600.0 + minutes * 60.0 + seconds + millis / 1000.0
    if start_timestamp is None:
        start_timestamp = absolute_seconds
    return absolute_seconds - start_timestamp, start_timestamp


def compute_axis_limits(*series_groups: tuple[deque[float], deque[float]]) -> tuple[float, float] | None:
    values: list[float] = []
    for _times, series in series_groups:
        values.extend(v for v in series if math.isfinite(v))

    if not values:
        return None

    lo = min(values)
    hi = max(values)
    pad = max(5.0, (hi - lo) * 0.25)
    if lo == hi:
        pad = max(5.0, abs(lo) * 0.25, 1.0)
    return lo - pad, hi + pad


def recompute_mahony_series(
    samples: deque[MpuSample],
    params: MahonyParams,
    t_vals: deque[float],
    roll_vals: deque[float],
    pitch_vals: deque[float],
    yaw_vals: deque[float],
) -> None:
    filt = MahonyFilter()
    t_vals.clear()
    roll_vals.clear()
    pitch_vals.clear()
    yaw_vals.clear()

    for sample in samples:
        roll, pitch, yaw = filt.update(
            sample.ax,
            sample.ay,
            sample.az,
            sample.gx,
            sample.gy,
            sample.gz,
            sample.dt,
            params,
        )
        t_vals.append(sample.t_rel)
        roll_vals.append(roll)
        pitch_vals.append(pitch)
        yaw_vals.append(yaw)


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Real-time Mahony tuner using MPU6050_READING logs")
    p.add_argument("--port", help="Serial port, e.g. COM7")
    p.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    p.add_argument(
        "--log",
        type=Path,
        help="Replay from a monitor log file. Defaults to log.txt next to this script when present.",
    )
    p.add_argument("--loop", action="store_true", help="Loop log replay forever")
    p.add_argument("--window", type=int, default=600, help="Samples to keep in the visible window")
    return p


def main() -> None:
    args = build_parser().parse_args()
    args.log = resolve_log_path(args.log)

    if not args.port and not args.log:
        raise SystemExit(
            "Provide either --port COMx for live serial or --log <path> for replay. "
            f"No default log file found at {DEFAULT_LOG_PATH}."
        )

    params = MahonyParams()

    line_q: queue.Queue[str] = queue.Queue(maxsize=4096)
    stop_event = threading.Event()

    if args.port:
        reader = threading.Thread(target=serial_line_source, args=(args.port, args.baud, line_q, stop_event), daemon=True)
    else:
        reader = threading.Thread(target=file_line_source, args=(args.log, line_q, stop_event, args.loop), daemon=True)
    reader.start()

    mpu_samples: deque[MpuSample] = deque(maxlen=args.window)
    t_vals: deque[float] = deque(maxlen=args.window)
    roll_vals: deque[float] = deque(maxlen=args.window)
    pitch_vals: deque[float] = deque(maxlen=args.window)
    yaw_vals: deque[float] = deque(maxlen=args.window)

    pose_roll_vals: deque[float] = deque(maxlen=args.window)
    pose_pitch_vals: deque[float] = deque(maxlen=args.window)
    pose_yaw_vals: deque[float] = deque(maxlen=args.window)
    pose_t_vals: deque[float] = deque(maxlen=args.window)

    start_t = time.perf_counter()
    start_timestamp: float | None = None
    last_mpu_t_rel: float | None = None
    params_dirty = False

    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    plt.subplots_adjust(bottom=0.32, hspace=0.35)

    roll_line, = axes[0].plot([], [], "r-", linewidth=1.6, label="Mahony Roll")
    pitch_line, = axes[1].plot([], [], "r-", linewidth=1.6, label="Mahony Pitch")
    yaw_line, = axes[2].plot([], [], "r-", linewidth=1.6, label="Mahony Yaw")

    pose_roll_line, = axes[0].plot([], [], "k--", linewidth=1.0, alpha=0.8, label="ESP Pose Roll")
    pose_pitch_line, = axes[1].plot([], [], "k--", linewidth=1.0, alpha=0.8, label="ESP Pose Pitch")
    pose_yaw_line, = axes[2].plot([], [], "k--", linewidth=1.0, alpha=0.8, label="ESP Pose Yaw")

    axes[0].set_title("ROLL")
    axes[1].set_title("PITCH")
    axes[2].set_title("YAW")
    for ax in axes:
        ax.set_ylabel("deg")
        ax.grid(True, alpha=0.25)
        ax.legend(loc="upper right")
    axes[2].set_xlabel("time (s)")

    waiting_text = fig.text(
        0.5,
        0.5,
        "Waiting for 'MPU6050_READING' or 'POSE_READING'...",
        ha="center",
        va="center",
        fontsize=14,
        alpha=0.7,
    )
    stream_status_text = fig.text(0.5, 0.965, "", ha="center", va="top", fontsize=10, alpha=0.85)

    slider_axes = {
        "kp": fig.add_axes([0.10, 0.24, 0.35, 0.03]),
        "ki": fig.add_axes([0.55, 0.24, 0.35, 0.03]),
        "accel_min": fig.add_axes([0.10, 0.19, 0.35, 0.03]),
        "accel_max": fig.add_axes([0.55, 0.19, 0.35, 0.03]),
        "gyro_clip": fig.add_axes([0.10, 0.14, 0.35, 0.03]),
        "max_rate": fig.add_axes([0.55, 0.14, 0.35, 0.03]),
    }

    s_kp = Slider(slider_axes["kp"], "Kp", 0.1, 10.0, valinit=params.kp, valstep=0.05)
    s_ki = Slider(slider_axes["ki"], "Ki", 0.0, 1.0, valinit=params.ki, valstep=0.005)
    s_accel_min = Slider(slider_axes["accel_min"], "Accel min g", 0.4, 1.1, valinit=params.accel_min_g, valstep=0.01)
    s_accel_max = Slider(slider_axes["accel_max"], "Accel max g", 1.0, 1.8, valinit=params.accel_max_g, valstep=0.01)
    s_gyro_clip = Slider(slider_axes["gyro_clip"], "Gyro clip dps", 80.0, 1200.0, valinit=params.gyro_clip_dps, valstep=5.0)
    s_max_rate = Slider(slider_axes["max_rate"], "Max angle rate dps", 90.0, 1500.0, valinit=params.max_angle_rate_dps, valstep=10.0)

    def refresh_params(_: float) -> None:
        nonlocal params_dirty
        params.kp = float(s_kp.val)
        params.ki = float(s_ki.val)
        params.accel_min_g = float(s_accel_min.val)
        params.accel_max_g = float(s_accel_max.val)
        params.gyro_clip_dps = float(s_gyro_clip.val)
        params.max_angle_rate_dps = float(s_max_rate.val)
        params_dirty = True

    for slider in (s_kp, s_ki, s_accel_min, s_accel_max, s_gyro_clip, s_max_rate):
        slider.on_changed(refresh_params)

    def update(_: int):
        nonlocal start_timestamp, last_mpu_t_rel, params_dirty
        consumed_any = False
        consumed_mpu = False

        while True:
            try:
                line = line_q.get_nowait()
            except queue.Empty:
                break

            fallback_t = time.perf_counter() - start_t
            t_rel, start_timestamp = extract_log_timestamp(line, fallback_t, start_timestamp)

            m = MPU_RE.search(line)
            if m:
                ax, ay, az, gx, gy, gz, _temp = (float(v) for v in m.groups())
                if last_mpu_t_rel is None:
                    dt = 0.01
                else:
                    dt = min(max(t_rel - last_mpu_t_rel, 0.001), 0.05)
                last_mpu_t_rel = t_rel

                mpu_samples.append(MpuSample(t_rel=t_rel, dt=dt, ax=ax, ay=ay, az=az, gx=gx, gy=gy, gz=gz))
                consumed_any = True
                consumed_mpu = True
                continue

            p = POSE_RE.search(line)
            if p:
                _x, _y, _z, roll, pitch, yaw = (float(v) for v in p.groups())
                pose_t_vals.append(t_rel)
                pose_roll_vals.append(roll)
                pose_pitch_vals.append(pitch)
                pose_yaw_vals.append(yaw)
                consumed_any = True

        if consumed_any:
            waiting_text.set_visible(False)

        if consumed_mpu or params_dirty:
            recompute_mahony_series(mpu_samples, params, t_vals, roll_vals, pitch_vals, yaw_vals)
            params_dirty = False

        if t_vals:
            roll_line.set_data(t_vals, roll_vals)
            pitch_line.set_data(t_vals, pitch_vals)
            yaw_line.set_data(t_vals, yaw_vals)
        else:
            roll_line.set_data([], [])
            pitch_line.set_data([], [])
            yaw_line.set_data([], [])

        if pose_t_vals:
            pose_roll_line.set_data(pose_t_vals, pose_roll_vals)
            pose_pitch_line.set_data(pose_t_vals, pose_pitch_vals)
            pose_yaw_line.set_data(pose_t_vals, pose_yaw_vals)
        else:
            pose_roll_line.set_data([], [])
            pose_pitch_line.set_data([], [])
            pose_yaw_line.set_data([], [])

        time_candidates = []
        if t_vals:
            time_candidates.extend((t_vals[0], t_vals[-1]))
        if pose_t_vals:
            time_candidates.extend((pose_t_vals[0], pose_t_vals[-1]))

        if time_candidates:
            x_min = max(0.0, min(time_candidates))
            x_max = max(time_candidates) + 0.001
            for ax in axes:
                ax.set_xlim(x_min, x_max)

        axis_series = (
            (axes[0], (t_vals, roll_vals), (pose_t_vals, pose_roll_vals)),
            (axes[1], (t_vals, pitch_vals), (pose_t_vals, pose_pitch_vals)),
            (axes[2], (t_vals, yaw_vals), (pose_t_vals, pose_yaw_vals)),
        )
        for ax, mahony_series, pose_series in axis_series:
            limits = compute_axis_limits(mahony_series, pose_series)
            if limits is not None:
                ax.set_ylim(*limits)

        if pose_t_vals and not t_vals:
            stream_status_text.set_text("POSE_READING detected, but no MPU6050_READING samples are present in this log. Mahony output cannot be computed.")
            stream_status_text.set_color("darkred")
        elif t_vals:
            stream_status_text.set_text(f"Mahony replay active on {len(t_vals)} buffered MPU samples. Slider changes recompute the red trace.")
            stream_status_text.set_color("darkgreen")
        else:
            stream_status_text.set_text("")

        title_src = f"Serial {args.port}@{args.baud}" if args.port else f"Replay {args.log}"
        fig.suptitle(
            f"Mahony Tuner ({title_src}) | Kp={params.kp:.2f} Ki={params.ki:.3f} "
            f"accel=[{params.accel_min_g:.2f},{params.accel_max_g:.2f}]g",
            fontsize=11,
        )

        return roll_line, pitch_line, yaw_line, pose_roll_line, pose_pitch_line, pose_yaw_line, waiting_text, stream_status_text

    anim = FuncAnimation(fig, update, interval=40, blit=False)

    try:
        plt.show()
    finally:
        stop_event.set()
        reader.join(timeout=1.0)
        del anim


if __name__ == "__main__":
    main()