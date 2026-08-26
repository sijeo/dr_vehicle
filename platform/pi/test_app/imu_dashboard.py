#!/usr/bin/env python3
"""Live UDP dashboard for the standalone DR IMU validation streamer. """
from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import socket
import struct
import sys
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Optional

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

MAGIC = 0x494D5531
VERSION = 1
FRAME = struct.Struct("!IHHIQ6i4I43fI")
FRAME_SIZE = FRAME.size
RAD2DEG = 180.0 / math.pi

CAL_STATES = {
    0: "UNINITIALIZED",
    1: "LOADING",
    2: "SETTLING",
    3: "COLLECTING",
    4: "RESTARTING",
    5: "VALID",
    6: "FAILED",
}

FLAGS = {
    0: "CAL_VALID",
    1: "STATIONARY",
    2: "ACC_NORM_OK",
    3: "GYRO_STILL",
    4: "BUMP",
    5: "RAW_SATURATION",
    6: "SAMPLE_GAP",
    7: "ACCEL_SCALE_BAD",
    8: "VARIANCE_BAD",
    9: "RATE_BAD",
    10: "CONFIG_MISMATCH",
    11: "FILTER_WARMUP",
    12: "QUALITY_GOOD",
    13: "QUALITY_MARGINAL",
    14: "QUALITY_BAD",
    15: "CAL_SAVE_FAILED",
}

@dataclass(frozen=True)
class Sample:
    seq: int
    monotonic_ns: int
    raw: tuple[int, ...]
    cal_state: int
    flags: int
    boot_samples: int
    cal_created: int
    floats: tuple[float, ...]
    received_s: float

    @property
    def accel_sensor(self) -> tuple[float, float, float]:
        return self.floats[0:3]

    @property
    def gyro_sensor_raw(self) -> tuple[float, float, float]:
        return self.floats[3:6]

    @property
    def accel_unfiltered(self) -> tuple[float, float, float]:
        return self.floats[6:9]

    @property
    def gyro_unfiltered(self) -> tuple[float, float, float]:
        return self.floats[9:12]

    @property
    def accel_filtered(self) -> tuple[float, float, float]:
        return self.floats[12:15]

    @property
    def gyro_filtered(self) -> tuple[float, float, float]:
        return self.floats[15:18]

    @property
    def accel_std(self) -> tuple[float, float, float]:
        return self.floats[21:24]

    @property
    def gyro_std(self) -> tuple[float, float, float]:
        return self.floats[27:30]

    @property
    def gyro_bias_counts(self)-> tuple[float, float, float]:
        return self.floats[30:33]

    @property
    def acc_norm_unfiltered(self) -> float:
        return self.floats[33]

    @property
    def acc_norm_filtered(self) -> float:
        return self.floats[34]

    @property
    def gyro_norm_filtered(self) -> float:
        return self.floats[36]

    @property
    def cal_progress(self) -> float:
        return self.floats[37]

    @property
    def sample_rate(self) -> float:
        return self.floats[38]

    @property
    def bump_duty(self) -> float:
        return self.floats[39]

    @property
    def cutoff(self) -> float:
        return self.floats[40]

    @property
    def quality_score(self) -> float:
        return self.floats[41]

    @property
    def stationary_score(self) -> float:
        return self.floats[42]


def parse_frame(data: bytes) -> Sample:
    if len(data) != FRAME_SIZE:
        raise ValueError(f"frame size {len(data)} != {FRAME_SIZE}")
    expected_crc = struct.unpack_from("!I", data, FRAME_SIZE - 4)[0]
    actual_crc = zlib.crc32(data[:-4]) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError("CRC Mismatch")
    values = FRAME.unpack(data)
    magic, version, frame_size, seq, mono_ns = values[:5]
    if magic != MAGIC or version != VERSION or frame_size != FRAME_SIZE:
        raise ValueError("Protocol Mismatch")
    raw = tuple(values[5:11])
    cal_state, flags, boot_samples, cal_created = values[11:15]
    floats = tuple(values[15:58])
    return Sample(
        seq=seq, 
        monotonic_ns=mono_ns,
        raw=raw,
        cal_state=cal_state,
        flags= flags,
        boot_samples=boot_samples,
        cal_created=cal_created,
        floats=floats,
        received_s=time.monotonic(),
    )

class Receiver(threading.Thread):
    def __init__(self, bind: str, port: int, out_queue: queue.Queue[Sample],
                 verbose: bool = False, summary_interval_s: float = 1.0) -> None:
        super().__init__(daemon=True)
        self.bind = bind
        self.port = port
        self.out_queue = out_queue
        self.stop_event = threading.Event()
        self.bad_frames = 0
        self.verbose = verbose                  # per-packet CLI print
        self.summary_interval_s = summary_interval_s
        # Counters — reset on each summary tick, accumulate in _total_*
        self._pkts_since = 0
        self._bytes_since = 0
        self._total_pkts = 0
        self._total_bytes = 0
        self._last_summary = 0.0
        self._last_seq = None
        self._gaps = 0

    def run(self) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            # -----------------------------------------------------------
            # Platform-conditional socket options.
            #
            # On Linux / macOS: SO_REUSEADDR is safe and useful — it
            #   lets us rebind immediately after a crash/restart instead
            #   of waiting ~60s for TIME_WAIT to clear.
            #
            # On Windows: SO_REUSEADDR has HIJACK semantics — any other
            #   process can silently bind to the same port and steal
            #   packets. If a stale Python or a system service was ever
            #   bound to 5005 first, this dashboard's bind() succeeds
            #   but packets are delivered to the SQUATTER, not us.
            #   SO_EXCLUSIVEADDRUSE prevents this: bind() fails loudly
            #   if anyone else has (or later tries to) bind the port.
            # -----------------------------------------------------------
            if sys.platform == "win32":
                try:
                    sock.setsockopt(socket.SOL_SOCKET,
                                    socket.SO_EXCLUSIVEADDRUSE, 1)
                except (OSError, AttributeError):
                    pass  # older Python — fall back to default behaviour
            else:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            try:
                sock.bind((self.bind, self.port))
            except OSError as e:
                # This is now noisy on purpose. On Windows this typically
                # means another process already owns the port (that's
                # exactly the packet-stealer we're trying to avoid).
                print(f"[dashboard] BIND FAILED on {self.bind}:{self.port}: {e}",
                      flush=True)
                print(f"[dashboard]   likely cause: another process is already bound.",
                      flush=True)
                if sys.platform == "win32":
                    print(f"[dashboard]   run in an ADMIN PowerShell to find it:",
                          flush=True)
                    print(f"[dashboard]     netstat -an -o -p UDP | findstr :{self.port}",
                          flush=True)
                else:
                    print(f"[dashboard]   run:  sudo ss -ulnp | grep :{self.port}",
                          flush=True)
                raise

            # Confirm the bind succeeded and print WHAT we're actually
            # listening on. Useful when --bind 0.0.0.0 gets narrowed by
            # the OS to a specific interface, or when the printed port
            # differs from --port (unlikely but happens with port 0).
            got = sock.getsockname()
            print(f"[dashboard] receiver bound on {got[0]}:{got[1]}  "
                  f"pid={os.getpid()}  platform={sys.platform}", flush=True)

            # Self-loopback probe. Send a small UDP packet from a temporary
            # sender socket to our own bind address; if we DON'T get it
            # back within 500 ms, packets are being intercepted between
            # the sender and our socket — usually a firewall / AV filter
            # / interface-mismatch problem. If we DO get it back, our
            # socket is definitely healthy and any missing external
            # packets are being dropped upstream (wrong dest IP, WFP
            # filter, etc.).
            try:
                probe = b"IMUSELFTEST"
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as tx:
                    probe_dst = ("127.0.0.1", got[1])
                    tx.sendto(probe, probe_dst)
                sock.settimeout(0.5)
                try:
                    data, addr = sock.recvfrom(2048)
                    if data.startswith(probe):
                        print(f"[dashboard] self-probe OK: received back from {addr}",
                              flush=True)
                    else:
                        # Not our probe — a real packet already arrived first.
                        # Queue it as a real sample if it parses.
                        try:
                            sample = parse_frame(data)
                            self.out_queue.put_nowait(sample)
                        except (ValueError, queue.Full):
                            pass
                        print(f"[dashboard] self-probe: real traffic already flowing (got {len(data)} B from {addr})",
                              flush=True)
                except socket.timeout:
                    print(f"[dashboard] SELF-PROBE FAILED: no packet returned to {got[0]}:{got[1]}",
                          flush=True)
                    print(f"[dashboard]   the socket itself is bound but a filter is dropping packets.",
                          flush=True)
                    print(f"[dashboard]   check Windows Firewall for '{got[1]}' + verify no AV product is filtering.",
                          flush=True)
            except OSError as e:
                print(f"[dashboard] self-probe skipped ({e})", flush=True)

            sock.settimeout(0.5)
            first_packet_reported = False
            self._last_summary = time.monotonic()
            while not self.stop_event.is_set():
                try:
                    data, addr = sock.recvfrom(2048)
                    # Count EVERY received packet — even ones that later
                    # fail parse. This is the true "arrived at socket"
                    # counter.
                    self._pkts_since  += 1
                    self._bytes_since += len(data)
                    self._total_pkts  += 1
                    self._total_bytes += len(data)

                    # Announce the FIRST packet — this proves the socket
                    # is receiving.
                    if not first_packet_reported:
                        print(f"[dashboard] FIRST PACKET received: {len(data)} bytes from {addr}",
                              flush=True)
                        first_packet_reported = True

                    # Per-packet dump when --verbose (spammy at 240 Hz).
                    if self.verbose:
                        print(f"[rx #{self._total_pkts:>6d}] {len(data)} B  from {addr[0]}:{addr[1]}",
                              flush=True)

                    sample = parse_frame(data)

                    # Track sequence gaps so the summary line can report
                    # true packet loss vs. what left the Pi.
                    if self._last_seq is not None:
                        delta = (sample.seq - self._last_seq) & 0xFFFFFFFF
                        if 1 < delta < 0x80000000:
                            self._gaps += delta - 1
                    self._last_seq = sample.seq

                    try:
                        self.out_queue.put_nowait(sample)
                    except queue.Full:
                        try:
                            self.out_queue.get_nowait()
                        except queue.Empty:
                            pass
                        self.out_queue.put_nowait(sample)
                except socket.timeout:
                    # No data this cycle — still emit periodic summary so
                    # the CLI shows "0 pkts/s" instead of going silent.
                    pass
                except (OSError, ValueError) as e:
                    self.bad_frames += 1
                    if self.bad_frames <= 5 or self.bad_frames % 500 == 0:
                        print(f"[dashboard] bad frame #{self.bad_frames}: {e}",
                              flush=True)

                # ---- Periodic summary (~ every summary_interval_s) ----
                now = time.monotonic()
                dt = now - self._last_summary
                if dt >= self.summary_interval_s:
                    rate_hz = self._pkts_since / dt if dt > 0 else 0.0
                    kbps    = (self._bytes_since * 8) / (dt * 1000.0) if dt > 0 else 0.0
                    print(f"[rx] {self._pkts_since:5d} pkts / {dt:.2f}s "
                          f"({rate_hz:6.1f} Hz, {kbps:6.1f} kbps)  "
                          f"total={self._total_pkts}  bad={self.bad_frames}  "
                          f"gaps={self._gaps}  qsize={self.out_queue.qsize()}",
                          flush=True)
                    self._pkts_since  = 0
                    self._bytes_since = 0
                    self._last_summary = now

    def stop(self) -> None:
        self.stop_event.set()

class CsvLogger:
    HEADER = [
        "host_time_s", "sequence", "imu_time_ns", "cal_state", "flags",
        "sample_rate_hz", "quality_score", "stationary_score", "cal_progress",
        "sensor_ax_mps2", "sensor_ay_mps2", "sensor_az_mps2",
        "sensor_gx_raw_degps", "sensor_gy_raw_degps", "sensor_gz_raw_degps",
        "unfiltered_ax_mps2", "unfiltered_ay_mps2", "unfiltered_az_mps2",
        "unfiltered_gx_degps", "unfiltered_gy_degps", "unfiltered_gz_degps",
        "filtered_ax_mps2", "filtered_ay_mps2", "filtered_az_mps2", "filtered_acc_norm_mps2",
        "filtered_gx_degps", "filtered_gy_degps", "filtered_gz_degps", "filtered_gyro_norm_degps",
        "acc_std_x", "acc_std_y", "acc_std_z",
        "gyro_std_x_degps", "gyro_std_y_degps", "gyro_std_z_degps",
        "bump_duty_percent", "bias_x_counts", "bias_y_counts", "bias_z_counts",
    ]

    def __init__(self, path: Optional[Path]) -> None:
        self.file = None
        self.writer = None
        if path:
            # Auto-create the parent directory so a path like
            # ~/imu_logs/session1.csv works on a fresh install without
            # a FileNotFoundError.
            path.parent.mkdir(parents=True, exist_ok=True)
            self.file = path.open("w", newline="", encoding="utf-8")
            self.writer = csv.writer(self.file)
            self.writer.writerow(self.HEADER)

    def write(self, s: Sample) -> None:
        if not self.writer:
            return 
        self.writer.writerow([
            time.time(), s.seq, s.monotonic_ns, s.cal_state, f"0x{s.flags:08x}",
             s.sample_rate, s.quality_score, s.stationary_score, s.cal_progress, 
             *s.accel_sensor, *(v * RAD2DEG for v in s.gyro_sensor_raw), 
             *s.accel_unfiltered, *(v * RAD2DEG for v in s.gyro_unfiltered),
             *s.accel_filtered, s.acc_norm_filtered, 
             *(v * RAD2DEG for v in s.gyro_filtered), s.gyro_norm_filtered * RAD2DEG,
             *s.accel_std, *(v * RAD2DEG for v in s.gyro_std), 
             s.bump_duty, *s.gyro_bias_counts,
        ])

    def close(self) -> None:
        if self.file:
            self.file.flush()
            self.file.close()


def active_flag_names(flags: int) -> str:
    names = [ name for bit, name in FLAGS.items() if flags & (1 << bit)]
    return ", ".join(names) if names else "none"

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind", default="0.0.0.0", help="local bind address")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--window", type=float, default=15.0, help="visible seconds")
    parser.add_argument("--csv", type=Path, help="optional CSV recording path")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="print every received packet to stdout (spammy at 240 Hz)")
    parser.add_argument("--summary-interval", type=float, default=1.0,
                        help="seconds between packet-count summary lines (default 1.0)")
    args = parser.parse_args()

    q: queue.Queue[Sample] = queue.Queue(maxsize=2000)
    receiver = Receiver(args.bind, args.port, q,
                        verbose=args.verbose,
                        summary_interval_s=args.summary_interval)
    receiver.start()
    logger = CsvLogger(args.csv)
    startup_monotonic = time.monotonic()   # used by the "WAITING for packets…" status

    max_points = max(200, int(args.window * 150))
    t: Deque[float] = deque(maxlen=max_points)
    accel = [deque(maxlen=max_points) for _ in range(3)]
    gyro = [deque(maxlen=max_points) for _ in range(3)]
    accel_std = [deque(maxlen=max_points) for _ in range(3)]
    gyro_std = [deque(maxlen=max_points) for _ in range(3)]
    acc_norm: Deque[float] = deque(maxlen=max_points)
    quality: Deque[float] = deque(maxlen=max_points)
    stationary: Deque[float] = deque(maxlen=max_points)
    first_imu_ns: Optional[int] = None
    latest: Optional[Sample] = None
    last_seq: Optional[int] = None
    packets = 0
    lost = 0

    fig, axes = plt.subplots(2, 2, figsize=[13, 8])
    fig.canvas.manager.set_window_title("Dead Reckoning IMU Validation")
    ax_acc, ax_gyro, ax_noise, ax_quality = axes.flat

    acc_lines = [ax_acc.plot([], [], label=label)[0] for label in ("Ax", "Ay", "Az")]
    norm_line = ax_acc.plot([], [], label="|a|")[0]
    ax_acc.axhline(9.80665, linestyle="--", linewidth=1)
    ax_acc.set_ylabel("Acceleration (m/s^2)")
    ax_acc.legend(loc="upper right")
    ax_acc.grid(True)

    gyro_lines = [ax_gyro.plot([], [], label=label)[0] for label in ("Gx", "Gy", "Gz")]
    ax_gyro.set_ylabel("Angular rate (deg/s)")
    ax_gyro.legend(loc="upper right")
    ax_gyro.grid(True)

    noise_lines = [ax_noise.plot([], [], label=label)[0] for label in ("cAx", "cAy", "cAz")]
    gyro_noise_lines = [ax_noise.plot([], [], linestyle="--", label=label)[0] for label in ("Gx deg/s", "Gy deg/s", "Gz deg/s")]
    ax_noise.set_ylabel("Rolling standard deviation")
    ax_noise.set_xlabel("Time (s)")
    ax_noise.legend(loc="upper right", ncol=2)
    ax_noise.grid(True)

    quality_line = ax_quality.plot([], [], label= "Quality %")[0]
    stationary_line = ax_quality.plot([], [], label="Stationary confidence %")[0]
    ax_quality.set_ylim(0, 105)
    ax_quality.set_xlabel("Time (s)")
    ax_quality.set_ylabel("Score (%)")
    ax_quality.legend(loc="lower right")
    ax_quality.grid(True)

    status = fig.text(0.01, 0.985, f"Listening on {args.bind}:{args.port}", va="top", family="monospace")
    fig.tight_layout(rect=(0, 0, 1, 0.93))

    def update(_:int):
        nonlocal first_imu_ns, latest, last_seq, packets, lost
        drained = 0
        while drained < 1000:
            try:
                s = q.get_nowait()
            except queue.Empty:
                break
            drained += 1
            latest = s
            packets += 1
            if last_seq is not None:
                delta = (s.seq - last_seq) & 0xFFFFFFFF
                if 1 < delta < 0x80000000:
                    lost += delta - 1
            last_seq = s.seq
            if first_imu_ns is None:
                first_imu_ns = s.monotonic_ns
            ts = (s.monotonic_ns - first_imu_ns) * 1e-9
            t.append(ts)
            for i in range(3):
                accel[i].append(s.accel_filtered[i])
                gyro[i].append(s.gyro_filtered[i] * RAD2DEG)
                accel_std[i].append(s.accel_std[i])
                gyro_std[i].append(s.gyro_std[i] * RAD2DEG)
            acc_norm.append(s.acc_norm_filtered)
            quality.append(s.quality_score)
            stationary.append(s.stationary_score)
            logger.write(s)

        if t:
            tx = list(t)
            for i, line in enumerate(acc_lines):
                line.set_data(tx, list(accel[i]))
            norm_line.set_data(tx, list(acc_norm))
            for i, line in enumerate(gyro_lines):
                line.set_data(tx, list(gyro[i]))
            for i, line in enumerate(noise_lines):
                line.set_data(tx, list(accel_std[i]))
            for i, line in enumerate(gyro_noise_lines):
                line.set_data(tx, list(gyro_std[i]))
            quality_line.set_data(tx, list(quality))
            stationary_line.set_data(tx, list(stationary))

            left = max(0.0, tx[-1] - args.window)
            right = max(args.window, tx[-1])
            for ax in (ax_acc, ax_gyro, ax_noise, ax_quality):
                ax.set_xlim(left, right)
            for ax in (ax_acc, ax_gyro, ax_noise):
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
        if latest:
            # Both operands are seconds — the previous code used
            # time.monotonic_ns() (nanoseconds int) and printed Age in
            # the billions.
            age = time.monotonic() - latest.received_s
            loss_pct = 100.0 * lost / max(1, packets + lost)
            status.set_text(
                f"State={CAL_STATES.get(latest.cal_state, latest.cal_state):<11} "
                f"Cal={100*latest.cal_progress:5.1f}% Rate={latest.sample_rate:6.1f} Hz "
                f"Quality={latest.quality_score:5.1f}% Stationary={latest.stationary_score:5.1f}% "
                f"Loss={loss_pct:5.2f}% BadFrames={receiver.bad_frames} Age={age:4.2f}s\n"
                f"Flags: {active_flag_names(latest.flags)}\n"
                f"Gyro bias counts: ({latest.gyro_bias_counts[0]:.2f}, "
                f"{latest.gyro_bias_counts[1]:.2f}, {latest.gyro_bias_counts[2]:.2f}) "
                f"LPF={latest.cutoff:.1f} Hz Bump Duty={latest.bump_duty:.1f}%"
            )
        else:
            # No packets have arrived yet — show elapsed idle time so the
            # operator can distinguish "streamer down" from "streamer up
            # but network path blocked".
            idle_s = time.monotonic() - startup_monotonic
            status.set_text(
                f"WAITING for packets on {args.bind}:{args.port}  "
                f"idle={idle_s:5.1f}s  BadFrames={receiver.bad_frames}\n"
                f"Hint: on the Pi, run  ./imu_streamer --host <dashboard-ip> --port {args.port}\n"
                f"      verify with:  sudo tcpdump -i any -n udp port {args.port}"
            )
        return [*acc_lines, norm_line, *gyro_lines, *noise_lines, *gyro_noise_lines, 
                quality_line, stationary_line, status]

    animation = FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)
    _ = animation
    try:
        plt.show()
    finally:
        receiver.stop()
        receiver.join(timeout=1.0)
        logger.close()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())





