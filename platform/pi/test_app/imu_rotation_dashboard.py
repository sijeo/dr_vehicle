#!/usr/bin/env python3
"""IMU rotation dashboard: 3-D attitude cubes + live plots + CSV recording.

Receives the v2 UDP frames sent by imu_streamer (ISM330BX FIFO + SFLP) and shows

  * a vehicle-shaped box rotated by the on-chip SFLP game rotation vector,
  * a second box rotated by the host Mahony AHRS (independent reference),
  * roll / pitch / yaw, bias-corrected gyro and gravity-free level-frame
    acceleration over the last seconds,
  * calibration / quality / FIFO health.

Every received sample is written to CSV (toggle with 'r'); a recorded CSV can be
replayed through the same view with --replay.

Frames: vehicle FLU (x forward, y left, z up), world z up with arbitrary yaw
origin (no magnetometer). Angles follow REP-103: roll + = right side down,
pitch + = nose down, yaw + = counter-clockwise.

Keys:  r = start/stop recording   z = zero yaw   t = tare full attitude
       c = clear tare             q = quit

Usage:
  python imu_rotation_dashboard.py                       # listen on 0.0.0.0:5005
  python imu_rotation_dashboard.py --csv run1.csv        # choose the CSV path
  python imu_rotation_dashboard.py --replay run1.csv     # replay a recording
On the Pi:  ./imu_streamer --host <this PC's IP> [--gyro-fs 500]
"""
from __future__ import annotations

import argparse
import csv
import math
import queue
import socket
import struct
import sys
import threading
import time
import zlib
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------------------------------------------------------------- wire format
MAGIC_V1, MAGIC_V2 = 0x494D5531, 0x494D5532
V1 = struct.Struct("!IHHIQ6i4I43fI")             # 236 bytes, imu_wire_encode()
V2 = struct.Struct("!IHHIQ6i4I43f4I29fI")        # 368 bytes, imu_wire_encode_v2()
RAD2DEG = 180.0 / math.pi

CAL_STATES = {0: "UNINITIALIZED", 1: "LOADING", 2: "SETTLING", 3: "COLLECTING",
              4: "RESTARTING", 5: "VALID", 6: "FAILED"}
CORE_FLAGS = {1: "STATIONARY", 4: "BUMP", 5: "SATURATION", 6: "SAMPLE_GAP",
              9: "RATE_BAD", 11: "WARMUP", 12: "GOOD", 13: "MARGINAL", 14: "BAD"}
FF_SFLP, FF_FIFO_STREAM, FF_FIFO_OVR, FF_DROPPED = 1 << 0, 1 << 3, 1 << 4, 1 << 5
FF_AHRS, FF_CONV_DECIDED, FF_CONV_CONJ, FF_LEGACY = 1 << 8, 1 << 9, 1 << 10, 1 << 12

XYZ = ("x", "y", "z")
COLUMNS = (
    ["host_time_s", "seq", "imu_time_ns", "cal_state", "flags", "fusion_flags",
     "sflp_seq", "driver_seq", "driver_dropped"]
    + [f"raw_a{a}" for a in XYZ] + [f"raw_g{a}" for a in XYZ]
    + [f"acc_sensor_{a}_mps2" for a in XYZ] + [f"gyro_sensor_raw_{a}_dps" for a in XYZ]
    + [f"acc_unf_{a}_mps2" for a in XYZ] + [f"gyro_unf_{a}_dps" for a in XYZ]
    + [f"acc_filt_{a}_mps2" for a in XYZ] + [f"gyro_filt_{a}_dps" for a in XYZ]
    + [f"acc_std_{a}" for a in XYZ] + [f"gyro_std_{a}_dps" for a in XYZ]
    + [f"bias_{a}_counts" for a in XYZ]
    + ["acc_norm_filt_mps2", "gyro_norm_filt_dps", "cal_progress", "sample_rate_hz",
       "bump_duty_percent", "cutoff_hz", "quality_score", "stationary_score"]
    + ["q_sflp_w", "q_sflp_x", "q_sflp_y", "q_sflp_z", "roll_sflp_deg", "pitch_sflp_deg", "yaw_sflp_deg"]
    + ["q_ahrs_w", "q_ahrs_x", "q_ahrs_y", "q_ahrs_z", "roll_ahrs_deg", "pitch_ahrs_deg", "yaw_ahrs_deg"]
    + [f"gravity_sflp_{a}_mps2" for a in XYZ] + [f"gbias_sflp_{a}_dps" for a in XYZ]
    + [f"lin_acc_{a}_mps2" for a in XYZ]
    + ["lin_lvl_fwd_mps2", "lin_lvl_left_mps2", "lin_lvl_up_mps2"]
    + ["temp_c", "g_ref_mps2", "tilt_consistency_deg"]
)
INT_COLUMNS = {"seq", "imu_time_ns", "cal_state", "flags", "fusion_flags", "sflp_seq",
               "driver_seq", "driver_dropped"} | {f"raw_a{a}" for a in XYZ} | {f"raw_g{a}" for a in XYZ}
HEX_COLUMNS = {"flags", "fusion_flags"}


def parse_frame(data: bytes) -> dict:
    """Decode one UDP frame into a flat dict keyed like COLUMNS."""
    if len(data) == V2.size:
        st, magic, nfloat = V2, MAGIC_V2, 2
    elif len(data) == V1.size:
        st, magic, nfloat = V1, MAGIC_V1, 1
    else:
        raise ValueError(f"unexpected frame size {len(data)}")
    if struct.unpack_from("!I", data, len(data) - 4)[0] != (zlib.crc32(data[:-4]) & 0xFFFFFFFF):
        raise ValueError("CRC mismatch")
    v = st.unpack(data)
    if v[0] != magic or v[2] != st.size:
        raise ValueError("protocol mismatch")
    f = v[15:58]
    d = {"host_time_s": time.time(), "seq": v[3], "imu_time_ns": v[4],
         "cal_state": v[11], "flags": v[12]}
    for i, a in enumerate(XYZ):
        d[f"raw_a{a}"], d[f"raw_g{a}"] = v[5 + i], v[8 + i]
        d[f"acc_sensor_{a}_mps2"] = f[0 + i]
        d[f"gyro_sensor_raw_{a}_dps"] = f[3 + i] * RAD2DEG
        d[f"acc_unf_{a}_mps2"] = f[6 + i]
        d[f"gyro_unf_{a}_dps"] = f[9 + i] * RAD2DEG
        d[f"acc_filt_{a}_mps2"] = f[12 + i]
        d[f"gyro_filt_{a}_dps"] = f[15 + i] * RAD2DEG
        d[f"acc_std_{a}"] = f[21 + i]
        d[f"gyro_std_{a}_dps"] = f[27 + i] * RAD2DEG
        d[f"bias_{a}_counts"] = f[30 + i]
    d.update(acc_norm_filt_mps2=f[34], gyro_norm_filt_dps=f[36] * RAD2DEG, cal_progress=f[37],
             sample_rate_hz=f[38], bump_duty_percent=f[39], cutoff_hz=f[40],
             quality_score=f[41], stationary_score=f[42])
    if nfloat == 2:
        d["fusion_flags"], d["sflp_seq"], d["driver_seq"], d["driver_dropped"] = v[58:62]
        names = (["q_sflp_w", "q_sflp_x", "q_sflp_y", "q_sflp_z", "roll_sflp_deg", "pitch_sflp_deg", "yaw_sflp_deg",
                  "q_ahrs_w", "q_ahrs_x", "q_ahrs_y", "q_ahrs_z", "roll_ahrs_deg", "pitch_ahrs_deg", "yaw_ahrs_deg"]
                 + [f"gravity_sflp_{a}_mps2" for a in XYZ] + [f"gbias_sflp_{a}_dps" for a in XYZ]
                 + [f"lin_acc_{a}_mps2" for a in XYZ]
                 + ["lin_lvl_fwd_mps2", "lin_lvl_left_mps2", "lin_lvl_up_mps2",
                    "temp_c", "g_ref_mps2", "tilt_consistency_deg"])
        d.update(zip(names, v[62:91]))
    else:   # v1 streamer: no fusion block
        d.update({c: math.nan for c in COLUMNS if c not in d})
        d.update(fusion_flags=FF_LEGACY, sflp_seq=0, driver_seq=v[3], driver_dropped=0,
                 q_sflp_w=1.0, q_sflp_x=0.0, q_sflp_y=0.0, q_sflp_z=0.0,
                 q_ahrs_w=1.0, q_ahrs_x=0.0, q_ahrs_y=0.0, q_ahrs_z=0.0)
    return d


# ---------------------------------------------------------------- quaternions
def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def q_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def q_norm(q):
    n = math.sqrt(sum(c * c for c in q))
    return (1.0, 0.0, 0.0, 0.0) if not n or math.isnan(n) else tuple(c / n for c in q)


def q_yaw_only(q):
    w, x, y, z = q
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def q_to_euler_deg(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG


def q_rotate(q, v):
    """v_world = q v q* (q maps vehicle -> world)."""
    w, x, y, z = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


# ---------------------------------------------------------------- data sources
class Recorder:
    def __init__(self, path: Optional[Path]) -> None:
        self.path = path
        self._fh = None
        self._writer = None
        self._last_flush = 0.0
        self.rows = 0
        self.lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self._writer is not None

    def start(self) -> None:
        with self.lock:
            if self._writer or self.path is None:
                return
            self._fh = open(self.path, "a", newline="")
            self._writer = csv.writer(self._fh)
            if self._fh.tell() == 0:
                self._writer.writerow(COLUMNS)
            print(f"[dashboard] recording -> {self.path}", flush=True)

    def stop(self) -> None:
        with self.lock:
            if self._fh:
                self._fh.close()
                print(f"[dashboard] recording stopped ({self.rows} rows in {self.path})", flush=True)
            self._fh = self._writer = None

    def write(self, d: dict) -> None:
        with self.lock:
            if not self._writer:
                return
            self._writer.writerow([f"0x{d[c]:08x}" if c in HEX_COLUMNS else d[c] for c in COLUMNS])
            self.rows += 1
            now = time.monotonic()
            if now - self._last_flush > 1.0:
                self._fh.flush()
                self._last_flush = now


class Receiver(threading.Thread):
    def __init__(self, bind: str, port: int, out: "queue.Queue[dict]", rec: Recorder) -> None:
        super().__init__(daemon=True)
        self.bind, self.port, self.out, self.rec = bind, port, out, rec
        self.stop_event = threading.Event()
        self.packets = self.bad = self.seq_gaps = 0
        self._last_seq: Optional[int] = None

    def run(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            if sys.platform == "win32":     # stop other processes silently sharing the port
                try:
                    sock.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
                except (OSError, AttributeError):
                    pass
            else:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
            sock.bind((self.bind, self.port))
            sock.settimeout(0.2)
            print(f"[dashboard] listening on {self.bind}:{self.port}", flush=True)
            while not self.stop_event.is_set():
                try:
                    data, _ = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                try:
                    d = parse_frame(data)
                except ValueError:
                    self.bad += 1
                    continue
                self.packets += 1
                if self._last_seq is not None and d["seq"] != (self._last_seq + 1) & 0xFFFFFFFF:
                    self.seq_gaps += (d["seq"] - self._last_seq - 1) & 0xFFFFFFFF
                self._last_seq = d["seq"]
                self.rec.write(d)
                self.out.put(d)


class Replayer(threading.Thread):
    def __init__(self, path: Path, out: "queue.Queue[dict]", speed: float) -> None:
        super().__init__(daemon=True)
        self.path, self.out, self.speed = path, out, speed
        self.stop_event = threading.Event()
        self.packets = self.bad = self.seq_gaps = 0

    def run(self) -> None:
        with open(self.path, newline="") as fh:
            t0_imu = t0_wall = None
            for row in csv.DictReader(fh):
                if self.stop_event.is_set():
                    return
                d = {}
                for k, s in row.items():
                    if k in INT_COLUMNS:
                        d[k] = int(s, 0) if s else 0
                    else:
                        d[k] = float(s) if s not in ("", None) else math.nan
                t = d["imu_time_ns"] * 1e-9
                if t0_imu is None:
                    t0_imu, t0_wall = t, time.monotonic()
                lag = (t - t0_imu) / self.speed - (time.monotonic() - t0_wall)
                if lag > 0:
                    time.sleep(lag)
                self.packets += 1
                self.out.put(d)
        print("[dashboard] replay finished", flush=True)


# ---------------------------------------------------------------- view
# Validated categorical slots 1-3 (dataviz reference palette, light mode).
C_X, C_Y, C_Z = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"

BOX = (2.0, 1.2, 0.5)     # vehicle-shaped: long along x (forward)


def box_faces(q):
    lx, ly, lz = (s / 2 for s in BOX)
    corners = [(sx * lx, sy * ly, sz * lz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    c = [q_rotate(q, p) for p in corners]
    # index = 4*ix + 2*iy + iz with i* = 0 for -, 1 for +
    return [
        [c[4], c[6], c[7], c[5]],   # +x front
        [c[0], c[1], c[3], c[2]],   # -x back
        [c[2], c[3], c[7], c[6]],   # +y left
        [c[0], c[4], c[5], c[1]],   # -y right
        [c[1], c[5], c[7], c[3]],   # +z top
        [c[0], c[2], c[6], c[4]],   # -z bottom
    ]


FACE_COLORS = [C_X, "#d9d8d3", C_Y, "#d9d8d3", C_Z, "#bfbeb8"]


class CubeView:
    def __init__(self, ax, title: str) -> None:
        self.ax = ax
        ax.set_title(title, color=INK, fontsize=11, loc="left", pad=18)
        lim = 1.3
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim); ax.set_zlim(-lim, lim)
        ax.set_box_aspect((1, 1, 1))
        ax.set_xlabel("world X", color=INK2, fontsize=8)
        ax.set_ylabel("world Y", color=INK2, fontsize=8)
        ax.set_zlabel("up", color=INK2, fontsize=8)
        ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
        ax.view_init(elev=22, azim=-55)
        self.poly = Poly3DCollection(box_faces((1, 0, 0, 0)), facecolors=FACE_COLORS,
                                     edgecolors="#ffffff", linewidths=1.5, alpha=0.92)
        ax.add_collection3d(self.poly)
        self.axes_lines, self.axes_text = [], []
        for vec, col, lab in (((1.4, 0, 0), C_X, "F"), ((0, 1.0, 0), C_Y, "L"), ((0, 0, 0.9), C_Z, "U")):
            (ln,) = ax.plot([0, vec[0]], [0, vec[1]], [0, vec[2]], color=col, lw=2)
            self.axes_lines.append((ln, vec))
            self.axes_text.append((ax.text(*vec, lab, color=INK, fontsize=9, weight="bold"), vec))
        self.label = ax.text2D(0.0, 0.97, "", transform=ax.transAxes, color=INK, fontsize=10,
                               family="monospace")

    def update(self, q, euler, valid: bool) -> None:
        self.poly.set_verts(box_faces(q))
        self.poly.set_alpha(0.92 if valid else 0.25)
        for ln, vec in self.axes_lines:
            p = q_rotate(q, vec)
            ln.set_data_3d([0, p[0]], [0, p[1]], [0, p[2]])
        for txt, vec in self.axes_text:
            txt.set_position_3d(q_rotate(q, tuple(1.12 * c for c in vec)))
        if valid:
            self.label.set_text(f"roll {euler[0]:+7.2f}°  pitch {euler[1]:+7.2f}°  yaw {euler[2]:+7.2f}°")
        else:
            self.label.set_text("no data")


class Dashboard:
    def __init__(self, src, rec: Recorder, window_s: float) -> None:
        self.src, self.rec = src, rec
        self.q: "queue.Queue[dict]" = src.out
        self.window_s = window_s
        self.hist = {k: deque(maxlen=int(400 * window_s)) for k in (
            "t", "r_s", "p_s", "y_s", "r_a", "p_a", "y_a", "gx", "gy", "gz", "ax", "ay", "az")}
        self.last: Optional[dict] = None
        self.tare = (1.0, 0.0, 0.0, 0.0)
        self.rate_mark = (time.monotonic(), 0)
        self.rx_rate = 0.0

        for k in list(plt.rcParams):
            if k.startswith("keymap."):
                plt.rcParams[k] = [b for b in plt.rcParams[k] if b not in ("r", "z", "t", "c")]
        self.fig = plt.figure(figsize=(15, 9), facecolor=SURFACE)
        self.fig.canvas.manager.set_window_title("IMU rotation dashboard (ISM330BX SFLP)")
        gs = self.fig.add_gridspec(3, 3, width_ratios=(1.0, 1.0, 1.25), height_ratios=(1, 1, 0.55),
                                   left=0.03, right=0.98, top=0.95, bottom=0.05, wspace=0.18, hspace=0.32)
        self.cube_s = CubeView(self.fig.add_subplot(gs[0:2, 0], projection="3d"), "On-chip SFLP (game rotation)")
        self.cube_a = CubeView(self.fig.add_subplot(gs[0:2, 1], projection="3d"), "Host Mahony AHRS")
        right = gs[0:3, 2].subgridspec(3, 1, hspace=0.45)
        self.ax_e = self.fig.add_subplot(right[0])
        self.ax_g = self.fig.add_subplot(right[1], sharex=self.ax_e)
        self.ax_a = self.fig.add_subplot(right[2], sharex=self.ax_e)
        self.lines = {}
        for ax, title, unit in ((self.ax_e, "Attitude", "deg"),
                                (self.ax_g, "Gyro (bias-corrected)", "deg/s"),
                                (self.ax_a, "Linear accel, level frame", "m/s²")):
            ax.set_facecolor(SURFACE)
            ax.set_title(title, color=INK, fontsize=10, loc="left")
            ax.set_ylabel(unit, color=INK2, fontsize=9)
            ax.grid(True, color=GRID, lw=0.8)
            ax.tick_params(labelsize=8, colors=INK2)
            for s in ax.spines.values():
                s.set_color(GRID)
        for key, col, ls, lab in (("r_s", C_X, "-", "roll SFLP"), ("p_s", C_Y, "-", "pitch SFLP"), ("y_s", C_Z, "-", "yaw SFLP"),
                                  ("r_a", C_X, "--", "roll AHRS"), ("p_a", C_Y, "--", "pitch AHRS"),
                                  ("y_a", C_Z, "--", "yaw AHRS")):
            (self.lines[key],) = self.ax_e.plot([], [], color=col, ls=ls, lw=2 if ls == "-" else 1.2, label=lab)
        for key, col, lab in (("gx", C_X, "x roll rate"), ("gy", C_Y, "y pitch rate"), ("gz", C_Z, "z yaw rate")):
            (self.lines[key],) = self.ax_g.plot([], [], color=col, lw=1.5, label=lab)
        for key, col, lab in (("ax", C_X, "forward"), ("ay", C_Y, "left"), ("az", C_Z, "up")):
            (self.lines[key],) = self.ax_a.plot([], [], color=col, lw=1.5, label=lab)
        for ax in (self.ax_e, self.ax_g, self.ax_a):
            ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), fontsize=8, ncol=3, frameon=False,
                      labelcolor=INK2, borderaxespad=0.1)
        self.ax_a.set_xlabel("time [s]", color=INK2, fontsize=9)

        self.ax_s = self.fig.add_subplot(gs[2, 0:2])
        self.ax_s.axis("off")
        self.status = self.ax_s.text(0.0, 1.0, "waiting for packets…", va="top", ha="left",
                                     family="monospace", fontsize=9, color=INK)
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    # -- interaction
    def on_key(self, ev) -> None:
        if ev.key == "r":
            self.rec.stop() if self.rec.active else self.rec.start()
        elif ev.key == "z" and self.last:
            self.tare = q_yaw_only(self._q("sflp", self.last))
        elif ev.key == "t" and self.last:
            self.tare = self._q("sflp", self.last)
        elif ev.key == "c":
            self.tare = (1.0, 0.0, 0.0, 0.0)

    @staticmethod
    def _q(which: str, d: dict):
        return q_norm(tuple(d[f"q_{which}_{c}"] for c in "wxyz"))

    def _display_q(self, which: str, d: dict):
        return q_norm(q_mul(q_conj(self.tare), self._q(which, d)))

    # -- animation
    def update(self, _frame):
        n = 0
        while True:
            try:
                d = self.q.get_nowait()
            except queue.Empty:
                break
            n += 1
            self.last = d
            h = self.hist
            h["t"].append(d["imu_time_ns"] * 1e-9)
            es = q_to_euler_deg(self._display_q("sflp", d))
            ea = q_to_euler_deg(self._display_q("ahrs", d))
            for k, v in zip(("r_s", "p_s", "y_s", "r_a", "p_a", "y_a"), es + ea):
                h[k].append(v)
            for a in XYZ:
                h["g" + a].append(d[f"gyro_unf_{a}_dps"])
            h["ax"].append(d["lin_lvl_fwd_mps2"])
            h["ay"].append(d["lin_lvl_left_mps2"])
            h["az"].append(d["lin_lvl_up_mps2"])

        now = time.monotonic()
        if now - self.rate_mark[0] >= 1.0:
            self.rx_rate = (self.src.packets - self.rate_mark[1]) / (now - self.rate_mark[0])
            self.rate_mark = (now, self.src.packets)

        d = self.last
        if d is None:
            return []
        ff = int(d["fusion_flags"])
        qs, qa = self._display_q("sflp", d), self._display_q("ahrs", d)
        self.cube_s.update(qs, q_to_euler_deg(qs), bool(ff & FF_SFLP))
        self.cube_a.update(qa, q_to_euler_deg(qa), bool(ff & FF_AHRS))

        t = list(self.hist["t"])
        if t:
            t_end = t[-1]
            i0 = next((i for i, v in enumerate(t) if v >= t_end - self.window_s), 0)
            step = max(1, (len(t) - i0) // 1200)        # keep drawing cheap
            tx = [v - t_end for v in t[i0::step]]
            for k, ln in self.lines.items():
                ys = list(self.hist[k])[i0::step]
                if k in ("y_s", "y_a"):     # break the line at the ±180° wrap
                    ys = [math.nan if j and abs(v - ys[j - 1]) > 180 else v for j, v in enumerate(ys)]
                ln.set_data(tx, ys)
            for ax in (self.ax_e, self.ax_g, self.ax_a):
                ax.set_xlim(-self.window_s, 0)
                ax.relim()
                ax.autoscale_view(scalex=False)

        flags = int(d["flags"])
        core = " ".join(n for b, n in CORE_FLAGS.items() if flags & (1 << b)) or "-"
        conv = ("conjugated" if ff & FF_CONV_CONJ else "direct") if ff & FF_CONV_DECIDED else "pending (tilt the IMU)"
        path = "HW FIFO" if ff & FF_FIFO_STREAM else ("legacy driver" if ff & FF_LEGACY else "register poll")
        health = []
        if ff & FF_FIFO_OVR:
            health.append("FIFO OVERRUN")
        if ff & FF_DROPPED:
            health.append("DROPPED")
        tare = "none" if self.tare == (1.0, 0.0, 0.0, 0.0) else "active"
        lines = [
            f"cal {CAL_STATES.get(int(d['cal_state']), '?'):<11} {100 * d['cal_progress']:5.1f}%   "
            f"quality {d['quality_score']:5.1f}   stationary {d['stationary_score']:5.1f}   flags {core}",
            f"rate {d['sample_rate_hz']:6.1f} Hz (rx {self.rx_rate:6.1f}/s)   read path {path}   "
            f"driver dropped {int(d['driver_dropped'])}   UDP gaps {self.src.seq_gaps}   bad {self.src.bad}   "
            f"{' '.join(health)}",
            f"|a| {d['acc_norm_filt_mps2']:6.3f} m/s²   g_ref {d['g_ref_mps2']:6.3f}   "
            f"tilt consistency {d['tilt_consistency_deg']:5.2f}°   SFLP convention {conv}",
            f"SFLP gyro bias [dps] {d['gbias_sflp_x_dps']:+7.3f} {d['gbias_sflp_y_dps']:+7.3f} "
            f"{d['gbias_sflp_z_dps']:+7.3f}   temp {d['temp_c']:5.1f} °C   tare {tare}",
            f"recording {'ON  -> ' + self.rec.path.name + f' ({self.rec.rows} rows)' if self.rec.active else 'off'}"
            f"      keys: r record  z zero-yaw  t tare  c clear  q quit",
        ]
        self.status.set_text("\n".join(lines))
        return []

    def run(self) -> None:
        self._anim = FuncAnimation(self.fig, self.update, interval=50, cache_frame_data=False)
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--csv", type=Path, default=None,
                    help="CSV output path (default imu_rotation_YYYYmmdd_HHMMSS.csv)")
    ap.add_argument("--no-record", action="store_true", help="start with recording off (toggle with 'r')")
    ap.add_argument("--replay", type=Path, default=None, help="replay a CSV written by this dashboard")
    ap.add_argument("--speed", type=float, default=1.0, help="replay speed factor")
    ap.add_argument("--window", type=float, default=10.0, help="plot window [s]")
    args = ap.parse_args()

    out: "queue.Queue[dict]" = queue.Queue(maxsize=20000)
    if args.replay:
        rec = Recorder(None)
        src = Replayer(args.replay, out, args.speed)
    else:
        path = args.csv or Path(f"imu_rotation_{datetime.now():%Y%m%d_%H%M%S}.csv")
        rec = Recorder(path)
        if not args.no_record:
            rec.start()
        src = Receiver(args.bind, args.port, out, rec)
    src.start()
    try:
        Dashboard(src, rec, args.window).run()
    finally:
        src.stop_event.set()
        rec.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
