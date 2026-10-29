#!/usr/bin/env python3
"""IMU rotation dashboard: guided mount calibration, 3-D attitude cubes, live plots, CSV.

Receives the v2/v3 UDP frames sent by imu_streamer (ISM330BX FIFO + SFLP) and shows

  * a vehicle-shaped box rotated by the host Mahony AHRS (primary attitude),
  * a second box rotated by the on-chip SFLP game rotation vector, with a live
    consistency check against gravity (SFLP is diagnostic until it passes),
  * roll / pitch / yaw, bias-corrected gyro and gravity-free level-frame
    acceleration over the last seconds,
  * calibration / quality / FIFO health.

Mount calibration wizard (starts automatically, 'm' to rerun): prompts you to
  1. hold the IMU (or park the vehicle) level and still,
  2. turn it 180 deg about vertical and hold still (S skips: single position),
  3. show the forward direction: tip the nose DOWN and hold (bench), or press A
     and accelerate straight forward (vehicle).
It computes the sensor->vehicle rotation, sends it to the streamer, which
applies it live and stores it in imu_mount.txt on the Pi for the next run.

Every received sample is written to CSV (toggle with 'r'); a recorded CSV can be
replayed through the same view with --replay.

Frames: vehicle FLU (x forward, y left, z up), world z up with arbitrary yaw
origin (no magnetometer). Angles follow REP-103: roll + = right side down,
pitch + = nose down, yaw + = counter-clockwise.

Keys:  m = mount calibration wizard     SPACE = capture step   S = skip step
       A = capture forward by driving   ENTER = apply result   ESC = cancel wizard
       r = start/stop recording         z = zero yaw   t = tare   c = clear tare
       q = quit

Usage:
  python imu_rotation_dashboard.py                       # listen on 0.0.0.0:5005
  python imu_rotation_dashboard.py --csv run1.csv        # choose the CSV path
  python imu_rotation_dashboard.py --no-wizard           # skip the start-up calibration
  python imu_rotation_dashboard.py --replay run1.csv     # replay a recording
On the Pi:  ./imu_streamer --host <this PC's IP> --gyro-fs 500
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
V2 = struct.Struct("!IHHIQ6i4I43f4I29fI")        # 368 bytes, imu_wire_encode_v2() version 2
V3 = struct.Struct("!IHHIQ6i4I43f4I30fI")        # 372 bytes, version 3 (+ sflp_consistency_deg)
RAD2DEG = 180.0 / math.pi

CAL_STATES = {0: "UNINITIALIZED", 1: "LOADING", 2: "SETTLING", 3: "COLLECTING",
              4: "RESTARTING", 5: "VALID", 6: "FAILED"}
CORE_FLAGS = {1: "STATIONARY", 4: "BUMP", 5: "SATURATION", 6: "SAMPLE_GAP",
              9: "RATE_BAD", 11: "WARMUP", 12: "GOOD", 13: "MARGINAL", 14: "BAD"}
FF_SFLP, FF_FIFO_STREAM, FF_FIFO_OVR, FF_DROPPED = 1 << 0, 1 << 3, 1 << 4, 1 << 5
FF_AHRS, FF_LEGACY, FF_LIN_SFLP, FF_SFLP_SUSPECT = 1 << 8, 1 << 12, 1 << 13, 1 << 14

XYZ = ("x", "y", "z")
FUSION_NAMES = (["q_sflp_w", "q_sflp_x", "q_sflp_y", "q_sflp_z", "roll_sflp_deg", "pitch_sflp_deg", "yaw_sflp_deg",
                 "q_ahrs_w", "q_ahrs_x", "q_ahrs_y", "q_ahrs_z", "roll_ahrs_deg", "pitch_ahrs_deg", "yaw_ahrs_deg"]
                + [f"gravity_sflp_{a}_mps2" for a in XYZ] + [f"gbias_sflp_{a}_dps" for a in XYZ]
                + [f"lin_acc_{a}_mps2" for a in XYZ]
                + ["lin_lvl_fwd_mps2", "lin_lvl_left_mps2", "lin_lvl_up_mps2",
                   "temp_c", "g_ref_mps2", "tilt_consistency_deg", "sflp_consistency_deg"])
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
    + FUSION_NAMES
)
INT_COLUMNS = {"seq", "imu_time_ns", "cal_state", "flags", "fusion_flags", "sflp_seq",
               "driver_seq", "driver_dropped"} | {f"raw_a{a}" for a in XYZ} | {f"raw_g{a}" for a in XYZ}
HEX_COLUMNS = {"flags", "fusion_flags"}


def parse_frame(data: bytes) -> dict:
    """Decode one UDP frame into a flat dict keyed like COLUMNS."""
    sizes = {V3.size: (V3, MAGIC_V2, 30), V2.size: (V2, MAGIC_V2, 29), V1.size: (V1, MAGIC_V1, 0)}
    if len(data) not in sizes:
        raise ValueError(f"unexpected frame size {len(data)}")
    st, magic, nfus = sizes[len(data)]
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
    if nfus:
        d["fusion_flags"], d["sflp_seq"], d["driver_seq"], d["driver_dropped"] = v[58:62]
        d.update(zip(FUSION_NAMES, v[62:62 + nfus]))
    else:   # v1 streamer: no fusion block
        d.update(fusion_flags=FF_LEGACY, sflp_seq=0, driver_seq=v[3], driver_dropped=0,
                 q_sflp_w=1.0, q_sflp_x=0.0, q_sflp_y=0.0, q_sflp_z=0.0,
                 q_ahrs_w=1.0, q_ahrs_x=0.0, q_ahrs_y=0.0, q_ahrs_z=0.0)
    for c in COLUMNS:
        d.setdefault(c, math.nan)
    return d


# ---------------------------------------------------------------- vector / quaternion math
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


def v_norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 1e-9 else (0.0, 0.0, 0.0)


def v_dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def v_cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def v_angle_deg(a, b):
    return math.degrees(math.acos(max(-1.0, min(1.0, v_dot(v_norm(a), v_norm(b))))))


def mat_to_euler_deg(R):
    """ZYX (REP-103) angles of a rotation matrix."""
    return (math.degrees(math.atan2(R[2][1], R[2][2])),
            math.degrees(math.asin(max(-1.0, min(1.0, -R[2][0])))),
            math.degrees(math.atan2(R[1][0], R[0][0])))


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
    """UDP frames in; IMUACK replies in; IMUCTL commands out to the streamer."""

    def __init__(self, bind: str, port: int, out: "queue.Queue[dict]", rec: Recorder) -> None:
        super().__init__(daemon=True)
        self.bind, self.port, self.out, self.rec = bind, port, out, rec
        self.stop_event = threading.Event()
        self.packets = self.bad = self.seq_gaps = 0
        self._last_seq: Optional[int] = None
        self.sock: Optional[socket.socket] = None
        self.peer = None                      # streamer address, learned from its frames
        self.acks: "queue.Queue[str]" = queue.Queue()

    def send_ctl(self, text: str) -> bool:
        if self.sock is None or self.peer is None:
            return False
        self.sock.sendto(("IMUCTL " + text).encode(), self.peer)
        return True

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
            self.sock = sock
            print(f"[dashboard] listening on {self.bind}:{self.port}", flush=True)
            while not self.stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(2048)
                except socket.timeout:
                    continue
                except OSError:            # Windows: ICMP port-unreachable after a send
                    continue
                if data.startswith(b"IMUACK"):
                    self.acks.put(data.decode(errors="replace"))
                    continue
                try:
                    d = parse_frame(data)
                except ValueError:
                    self.bad += 1
                    continue
                self.peer = addr
                self.packets += 1
                if self._last_seq is not None and d["seq"] != (self._last_seq + 1) & 0xFFFFFFFF:
                    self.seq_gaps += (d["seq"] - self._last_seq - 1) & 0xFFFFFFFF
                self._last_seq = d["seq"]
                self.rec.write(d)
                self.out.put(d)
            self.sock = None


class Replayer(threading.Thread):
    def __init__(self, path: Path, out: "queue.Queue[dict]", speed: float) -> None:
        super().__init__(daemon=True)
        self.path, self.out, self.speed = path, out, speed
        self.stop_event = threading.Event()
        self.packets = self.bad = self.seq_gaps = 0
        self.acks: "queue.Queue[str]" = queue.Queue()

    def send_ctl(self, text: str) -> bool:
        return False

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
                for c in COLUMNS:               # older recordings lack newer columns
                    d.setdefault(c, math.nan)
                t = d["imu_time_ns"] * 1e-9
                if t0_imu is None:
                    t0_imu, t0_wall = t, time.monotonic()
                lag = (t - t0_imu) / self.speed - (time.monotonic() - t0_wall)
                if lag > 0:
                    time.sleep(lag)
                self.packets += 1
                self.out.put(d)
        print("[dashboard] replay finished", flush=True)


# ---------------------------------------------------------------- mount calibration wizard
class MountWizard:
    """Guided sensor->vehicle mount calibration (IMU_FIELD_CALIBRATION.md §3-4, simplified).

    Works in the streamer's *current* vehicle frame (acc_unf_*), so the result is a
    correction rotation D sent as IMUCTL MOUNT_DELTA; the streamer applies
    R_bv <- D * R_bv. Because D is built from measured directions, it also fixes a
    wrong --mount preset (board upside down, x pointing sideways, ...).
    """
    STILL_S, DRIVE_S = 3.0, 1.5
    STILL_GYRO_DPS, STILL_ACC_STD = 1.5, 0.08
    MIN_NOSE_TILT_DEG, MIN_DRIVE_ACC = 10.0, 1.0

    def __init__(self, src) -> None:
        self.src = src
        self.state = "idle"
        self.msg = ""
        self.warn = ""
        self.result = None
        self.captures = {}
        self.need_n = 1
        self.done_at = self.sent_at = 0.0
        self.retries = 0
        self.yaw_zeroed = False
        self._reset_capture()

    # -- public
    @property
    def active(self) -> bool:
        return self.state != "idle"

    def start(self) -> None:
        self.captures = {}
        self.result = None
        self.warn = ""
        self._goto("level")

    def cancel(self, why: str = "cancelled") -> None:
        self.state = "idle"
        print(f"[wizard] {why}", flush=True)

    def key(self, k: str) -> None:
        if k == "escape":
            self.cancel()
        elif self.state in ("level", "turn", "nose") and k == " ":
            self._begin_capture("still")
        elif self.state == "nose" and k == "a":
            self._begin_capture("drive")
        elif self.state == "turn" and k == "s":
            self._goto("nose")
        elif self.state == "review" and k == "enter":
            self._send()

    def feed(self, d: dict) -> None:
        if self.capturing is None:
            return
        acc = (d["acc_unf_x_mps2"], d["acc_unf_y_mps2"], d["acc_unf_z_mps2"])
        if any(math.isnan(c) for c in acc):
            return
        t = d["imu_time_ns"] * 1e-9
        if self.capturing == "still":
            still = (d["gyro_norm_filt_dps"] < self.STILL_GYRO_DPS and
                     max(d["acc_std_x"], d["acc_std_y"], d["acc_std_z"]) < self.STILL_ACC_STD)
            if not still:
                if self.n:
                    self.warn = "movement detected - hold still, capture restarted"
                self._reset_capture(keep_mode=True)
                return
            self._accumulate(acc, t)
            if t - self.t0 >= self.STILL_S:
                self._finish(tuple(s / self.n for s in self.sum))
        else:                                    # drive: horizontal accel w.r.t. the measured up
            up = self.captures["up"]
            h = tuple(a - v_dot(acc, up) * u for a, u in zip(acc, up))
            if math.sqrt(v_dot(h, h)) >= self.MIN_DRIVE_ACC and d["gyro_norm_filt_dps"] < 5.0:
                self._accumulate(h, t)
            rate = d["sample_rate_hz"] if 20.0 < d["sample_rate_hz"] < 5000.0 else 240.0
            self.need_n = int(self.DRIVE_S * rate)
            if self.n >= self.need_n:
                self._finish(tuple(s / self.n for s in self.sum))

    def poll_ack(self) -> None:
        while True:
            try:
                a = self.src.acks.get_nowait()
            except queue.Empty:
                break
            print(f"[wizard] {a}", flush=True)
            if self.state == "sending" and a.startswith("IMUACK MOUNT_DELTA"):
                if " OK" in a:
                    self.state = "done"
                    self.done_at = time.monotonic()
                    self.msg = "Applied on the Pi and saved to imu_mount.txt."
                else:
                    self.state = "review"
                    self.warn = "streamer rejected the result: " + a
        if self.state == "sending" and time.monotonic() - self.sent_at > 1.0:
            if self.retries < 3:
                self._send(retry=True)
            else:
                self.state = "review"
                self.warn = "no reply from imu_streamer (old binary? firewall?) - ENTER to retry"

    def progress(self) -> float:
        if self.capturing is None or not self.n:
            return 0.0
        if self.capturing == "drive":
            return min(1.0, self.n / max(1, self.need_n))
        return min(1.0, (self.t_last - self.t0) / self.STILL_S)

    # -- internals
    def _reset_capture(self, keep_mode: bool = False) -> None:
        if not keep_mode:
            self.capturing = None
        self.n = 0
        self.sum = [0.0, 0.0, 0.0]
        self.t0 = self.t_last = 0.0

    def _begin_capture(self, mode: str) -> None:
        self._reset_capture()
        if mode == "drive":
            self._set_up()
            self.need_n = int(self.DRIVE_S * 240)
        self.capturing = mode
        self.warn = ""

    def _set_up(self) -> None:
        """Vehicle 'up' in the current frame: mean of the two headings (slope cancels)."""
        f1, f2 = self.captures["f1"], self.captures.get("f2")
        u1 = v_norm(f1)
        self.captures["up"] = v_norm(tuple(a + b for a, b in zip(u1, v_norm(f2)))) if f2 else u1
        self.captures["g"] = math.sqrt(v_dot(f1, f1))

    def _accumulate(self, v, t) -> None:
        if not self.n:
            self.t0 = t
        self.n += 1
        self.t_last = t
        for i in range(3):
            self.sum[i] += v[i]

    def _goto(self, state: str) -> None:
        self.state = state
        self._reset_capture()
        self.msg = {
            "level": "STEP 1/3  LEVEL: place the IMU flat (vehicle: park on level ground), forward end "
                     "away from you.\nKeep it completely still and press SPACE.",
            "turn": "STEP 2/3  TURN 180 deg: rotate it half a turn about vertical on the same spot "
                    "(vehicle: turn around,\npark on the same marks). Keep still and press SPACE.   "
                    "S = skip (single position)",
            "nose": "STEP 3/3  FORWARD: without turning it, tip the FRONT end DOWN by 20-40 deg, hold still, press SPACE.\n"
                    "Vehicle: press A, then accelerate firmly in a straight line for ~3 s.",
        }[state]

    def _finish(self, mean) -> None:
        mode = self.capturing
        self._reset_capture()
        if self.state == "level":
            self.captures["f1"] = mean
            tilt = v_angle_deg(mean, (0, 0, 1))
            if tilt > 30:
                self.warn = (f"gravity is {tilt:.0f} deg from +z: the mount preset is wrong; "
                             "the wizard will correct it")
            self._goto("turn")
        elif self.state == "turn":
            self.captures["f2"] = mean
            self._goto("nose")
        elif self.state == "nose":
            f1, f2 = self.captures["f1"], self.captures.get("f2")
            self._set_up()
            up = self.captures["up"]
            if mode == "drive":
                h = mean                                         # already horizontal
                fwd = v_norm(h)
            else:
                # Use the change from the last still capture on the same spot and
                # heading, so the surface slope does not leak into the yaw.
                ref = v_norm(f2 if f2 else f1)
                dv = tuple(a - b for a, b in zip(v_norm(mean), ref))
                h = tuple(a - v_dot(dv, up) * u for a, u in zip(dv, up))
                if math.degrees(math.asin(min(1.0, math.sqrt(v_dot(h, h))))) < self.MIN_NOSE_TILT_DEG:
                    self.warn = f"tilt too small (< {self.MIN_NOSE_TILT_DEG:.0f} deg) - tip the nose further down"
                    self._goto("nose")
                    return
                fwd = v_norm(tuple(-c for c in h))              # nose down: gravity reaction moves backwards
            x_new = fwd
            z_new = up
            y_new = v_cross(z_new, x_new)
            D = [list(x_new), list(y_new), list(z_new)]          # rows: new axes in the current frame
            slope = 0.5 * v_angle_deg(f1, f2) if f2 else math.nan
            self.result = dict(D=D, euler=mat_to_euler_deg([[D[c][r] for c in range(3)] for r in range(3)]),
                               slope=slope, g=self.captures["g"], two_pos=bool(f2))
            self.state = "review"
            self.msg = ""

    def _send(self, retry: bool = False) -> None:
        if not self.result:
            return
        D = self.result["D"]
        text = "MOUNT_DELTA " + " ".join(f"{D[r][c]:.6f}" for r in range(3) for c in range(3))
        if not self.src.send_ctl(text):
            self.warn = "cannot send: no frames received from the streamer yet"
            return
        self.retries = self.retries + 1 if retry else 0
        self.sent_at = time.monotonic()
        self.state = "sending"

    def overlay_text(self) -> str:
        if self.state == "idle":
            return ""
        lines = ["MOUNT CALIBRATION   (ESC = cancel)", ""]
        if self.state in ("level", "turn", "nose"):
            lines.append(self.msg)
            if self.capturing:
                p = self.progress()
                what = "hold still" if self.capturing == "still" else "keep accelerating straight"
                lines.append(f"capturing [{'#' * int(30 * p):<30}] {100 * p:3.0f}%   {what}")
        elif self.state == "review":
            r = self.result
            e = r["euler"]
            lines.append(f"Correction to apply: roll {e[0]:+6.2f} deg  pitch {e[1]:+6.2f} deg  yaw {e[2]:+6.2f} deg"
                         f"   |g| = {r['g']:.3f} m/s²")
            lines.append(f"surface slope: {r['slope']:.2f} deg" if r["two_pos"]
                         else "single position: surface slope is included in roll/pitch")
            if max(abs(e[0]), abs(e[1]), abs(e[2])) > 30:
                lines.append("Large correction: the sensor axes differ from the vehicle axes - it will be compensated.")
            lines.append("ENTER = apply on the Pi and save    ESC = discard    m = start again")
        elif self.state == "sending":
            lines.append("sending to imu_streamer ...")
        elif self.state == "done":
            lines.append(self.msg)
            lines.append("Yaw zeroed. Rotate the IMU: the boxes should now follow it.")
        if self.warn:
            lines += ["", "! " + self.warn]
        return "\n".join(lines)


# ---------------------------------------------------------------- view
# Validated categorical slots 1-3 (dataviz reference palette, light mode).
C_X, C_Y, C_Z = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, GRID, SURFACE = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
WARN_INK = "#a3401b"

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
        self.title = ax.set_title(title, color=INK, fontsize=11, loc="left", pad=18)
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

    def update(self, q, euler, valid: bool, dim: bool = False) -> None:
        self.poly.set_verts(box_faces(q))
        self.poly.set_alpha(0.25 if (not valid or dim) else 0.92)
        for ln, vec in self.axes_lines:
            p = q_rotate(q, vec)
            ln.set_data_3d([0, p[0]], [0, p[1]], [0, p[2]])
        for txt, vec in self.axes_text:
            txt.set_position_3d(q_rotate(q, tuple(1.12 * c for c in vec)))
        if valid:
            self.label.set_text(f"roll {euler[0]:+7.2f}°  pitch {euler[1]:+7.2f}°  yaw {euler[2]:+7.2f}°")
        else:
            self.label.set_text("no data")


IDENT = (1.0, 0.0, 0.0, 0.0)


class Dashboard:
    PLOT_EVERY = 3            # redraw the time series every 3rd frame (cubes every frame)
    MAX_POINTS = 600          # per line, after decimation

    def __init__(self, src, rec: Recorder, window_s: float, wizard: bool) -> None:
        self.src, self.rec = src, rec
        self.q: "queue.Queue[dict]" = src.out
        self.window_s = window_s
        self.hist = {k: deque(maxlen=int(500 * window_s)) for k in (
            "t", "r_s", "p_s", "y_s", "r_a", "p_a", "y_a", "gx", "gy", "gz", "ax", "ay", "az")}
        self.last: Optional[dict] = None
        self.tare = {"sflp": IDENT, "ahrs": IDENT}
        self.rate_mark = (time.monotonic(), 0)
        self.rx_rate = 0.0
        self.frame = 0
        self.last_autoscale = 0.0
        self.wiz = MountWizard(src)
        self.wizard_pending = wizard

        for k in list(plt.rcParams):
            if k.startswith("keymap."):
                plt.rcParams[k] = [b for b in plt.rcParams[k]
                                   if b not in ("r", "z", "t", "c", "m", "s", "a", " ", "enter", "escape")]
        self.fig = plt.figure(figsize=(15, 9), facecolor=SURFACE)
        self.fig.canvas.manager.set_window_title("IMU rotation dashboard (ISM330BX)")
        gs = self.fig.add_gridspec(3, 3, width_ratios=(1.0, 1.0, 1.25), height_ratios=(1, 1, 0.55),
                                   left=0.03, right=0.98, top=0.95, bottom=0.05, wspace=0.18, hspace=0.32)
        self.cube_a = CubeView(self.fig.add_subplot(gs[0:2, 0], projection="3d"), "Host Mahony AHRS (primary)")
        self.cube_s = CubeView(self.fig.add_subplot(gs[0:2, 1], projection="3d"), "On-chip SFLP (diagnostic)")
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
        for key, col, ls, lab in (("r_a", C_X, "-", "roll AHRS"), ("p_a", C_Y, "-", "pitch AHRS"),
                                  ("y_a", C_Z, "-", "yaw AHRS"),
                                  ("r_s", C_X, ":", "roll SFLP"), ("p_s", C_Y, ":", "pitch SFLP"),
                                  ("y_s", C_Z, ":", "yaw SFLP")):
            (self.lines[key],) = self.ax_e.plot([], [], color=col, ls=ls, lw=2 if ls == "-" else 1.2, label=lab)
        for key, col, lab in (("gx", C_X, "x roll rate"), ("gy", C_Y, "y pitch rate"), ("gz", C_Z, "z yaw rate")):
            (self.lines[key],) = self.ax_g.plot([], [], color=col, lw=1.5, label=lab)
        for key, col, lab in (("ax", C_X, "forward"), ("ay", C_Y, "left"), ("az", C_Z, "up")):
            (self.lines[key],) = self.ax_a.plot([], [], color=col, lw=1.5, label=lab)
        for ax in (self.ax_e, self.ax_g, self.ax_a):
            ax.legend(loc="lower right", bbox_to_anchor=(1.0, 1.0), fontsize=8, ncol=3, frameon=False,
                      labelcolor=INK2, borderaxespad=0.1)
            ax.set_xlim(-self.window_s, 0)
        self.ax_a.set_xlabel("time [s]", color=INK2, fontsize=9)

        self.ax_s = self.fig.add_subplot(gs[2, 0:2])
        self.ax_s.axis("off")
        self.status = self.ax_s.text(0.0, 1.0, "waiting for packets…", va="top", ha="left",
                                     family="monospace", fontsize=9, color=INK)
        self.wiz_text = self.fig.text(0.03, 0.235, "", va="top", ha="left", fontsize=10.5, color=INK,
                                      family="monospace", zorder=10,
                                      bbox=dict(boxstyle="round,pad=0.6", fc="#fff6d6", ec="#d9b44a"))
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)

    # -- interaction
    def on_key(self, ev) -> None:
        k = (ev.key or "").lower()
        if k == "m":
            self.wiz.start()
            return
        if self.wiz.active and k in (" ", "s", "a", "enter", "escape"):
            self.wiz.key(k)
            return
        if k == "r":
            self.rec.stop() if self.rec.active else self.rec.start()
        elif k == "z" and self.last:
            self._zero_yaw()
        elif k == "t" and self.last:
            self.tare = {w: self._q(w, self.last) for w in ("sflp", "ahrs")}
        elif k == "c":
            self.tare = {"sflp": IDENT, "ahrs": IDENT}

    def _zero_yaw(self) -> None:
        self.tare = {w: q_yaw_only(self._q(w, self.last)) for w in ("sflp", "ahrs")}

    @staticmethod
    def _q(which: str, d: dict):
        return q_norm(tuple(d[f"q_{which}_{c}"] for c in "wxyz"))

    def _display_q(self, which: str, d: dict):
        return q_norm(q_mul(q_conj(self.tare[which]), self._q(which, d)))

    # -- animation
    def update(self, _frame):
        self.frame += 1
        while True:
            try:
                d = self.q.get_nowait()
            except queue.Empty:
                break
            self.last = d
            self.wiz.feed(d)
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
        self.wiz.poll_ack()

        now = time.monotonic()
        if now - self.rate_mark[0] >= 1.0:
            self.rx_rate = (self.src.packets - self.rate_mark[1]) / (now - self.rate_mark[0])
            self.rate_mark = (now, self.src.packets)

        d = self.last
        if d is None:
            self.wiz_text.set_visible(False)
            return []
        if self.wizard_pending and not self.wiz.active:
            self.wizard_pending = False
            self.wiz.start()
        if self.wiz.state == "done" and now - self.wiz.done_at > 0.5 and not self.wiz.yaw_zeroed:
            self._zero_yaw()
            self.wiz.yaw_zeroed = True
        if self.wiz.state != "done":
            self.wiz.yaw_zeroed = False
        if self.wiz.state == "done" and now - self.wiz.done_at > 6.0:
            self.wiz.state = "idle"

        ff = int(d["fusion_flags"])
        qs, qa = self._display_q("sflp", d), self._display_q("ahrs", d)
        suspect = bool(ff & FF_SFLP_SUSPECT)
        self.cube_a.update(qa, q_to_euler_deg(qa), bool(ff & FF_AHRS))
        self.cube_s.update(qs, q_to_euler_deg(qs), bool(ff & FF_SFLP), dim=suspect)
        cons = d.get("sflp_consistency_deg", math.nan)
        self.cube_s.title.set_text("On-chip SFLP (diagnostic)" + (
            f"  - INCONSISTENT {cons:.0f}° vs gravity" if suspect and not math.isnan(cons) else ""))
        self.cube_s.title.set_color(WARN_INK if suspect else INK)

        if self.frame % self.PLOT_EVERY == 0 and self.hist["t"]:
            t = self.hist["t"]
            t_end = t[-1]
            n = len(t)
            i0 = n - 1
            while i0 > 0 and t[i0 - 1] >= t_end - self.window_s:
                i0 -= 1
            step = max(1, (n - i0) // self.MAX_POINTS)
            idx = range(i0, n, step)
            tx = [t[i] - t_end for i in idx]
            for k, ln in self.lines.items():
                src = self.hist[k]
                ys = [src[i] for i in idx]
                if k in ("y_s", "y_a"):     # break the line at the ±180° wrap
                    ys = [math.nan if j and abs(v - ys[j - 1]) > 180 else v for j, v in enumerate(ys)]
                ln.set_data(tx, ys)
            if now - self.last_autoscale > 1.0:
                for ax in (self.ax_e, self.ax_g, self.ax_a):
                    ax.relim()
                    ax.autoscale_view(scalex=False)
                self.last_autoscale = now

        overlay = self.wiz.overlay_text()
        self.wiz_text.set_text(overlay)
        self.wiz_text.set_visible(bool(overlay))
        self.status.set_visible(not overlay)        # the prompt takes the status panel's place

        flags = int(d["flags"])
        core = " ".join(n for b, n in CORE_FLAGS.items() if flags & (1 << b)) or "-"
        path = "HW FIFO" if ff & FF_FIFO_STREAM else ("legacy driver" if ff & FF_LEGACY else "register poll")
        health = []
        if ff & FF_FIFO_OVR:
            health.append("FIFO OVERRUN")
        if ff & FF_DROPPED:
            health.append("DROPPED")
        tare = "none" if self.tare == {"sflp": IDENT, "ahrs": IDENT} else "active"
        att = "SFLP" if ff & FF_LIN_SFLP else "AHRS"
        lines = [
            f"cal {CAL_STATES.get(int(d['cal_state']), '?'):<11} {100 * d['cal_progress']:5.1f}%   "
            f"quality {d['quality_score']:5.1f}   stationary {d['stationary_score']:5.1f}   flags {core}",
            f"rate {d['sample_rate_hz']:6.1f} Hz (rx {self.rx_rate:6.1f}/s)   read path {path}   "
            f"driver dropped {int(d['driver_dropped'])}   UDP gaps {self.src.seq_gaps}   bad {self.src.bad}   "
            f"{' '.join(health)}",
            f"|a| {d['acc_norm_filt_mps2']:6.3f} m/s²   g_ref {d['g_ref_mps2']:6.3f}   "
            f"tilt consistency {d['tilt_consistency_deg']:5.2f}°   SFLP vs gravity {cons:5.1f}°   "
            f"gravity removal: {att}",
            f"SFLP gyro bias [dps] {d['gbias_sflp_x_dps']:+7.3f} {d['gbias_sflp_y_dps']:+7.3f} "
            f"{d['gbias_sflp_z_dps']:+7.3f}   temp {d['temp_c']:5.1f} °C   tare {tare}",
            f"recording {'ON  -> ' + self.rec.path.name + f' ({self.rec.rows} rows)' if self.rec.active else 'off'}"
            f"      keys: m mount-cal  r record  z zero-yaw  t tare  c clear  q quit",
        ]
        self.status.set_text("\n".join(lines))
        return []

    def run(self) -> None:
        self._anim = FuncAnimation(self.fig, self.update, interval=40, cache_frame_data=False)
        plt.show()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--csv", type=Path, default=None,
                    help="CSV output path (default imu_rotation_YYYYmmdd_HHMMSS.csv)")
    ap.add_argument("--no-record", action="store_true", help="start with recording off (toggle with 'r')")
    ap.add_argument("--no-wizard", action="store_true", help="do not start the mount calibration at launch")
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
        Dashboard(src, rec, args.window, wizard=not (args.no_wizard or args.replay)).run()
    finally:
        src.stop_event.set()
        rec.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
