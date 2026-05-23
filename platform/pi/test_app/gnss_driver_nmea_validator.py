#!/usr/bin/env python3
"""
gnss_driver_nmea_validator.py

Validate whether the Raspberry Pi GNSS kernel driver is parsing NMEA correctly.

It can:
  1. Read the driver snapshot through /dev/neo6m0 ioctl.
  2. Read the driver's sysfs values.
  3. Independently parse raw NMEA from a text file, stdin, or UART.
  4. Compare independently parsed NMEA values with driver ioctl/sysfs values.
  5. Report stale fixes, update rate, impossible speed jumps, and scaling errors.

No internet required.
Dependencies: Python standard library only.

Typical usage:
  sudo python3 gnss_driver_nmea_validator.py --ioctl /dev/neo6m0 --sysfs /sys/class/neo6m_gnss/neo6m0 --duration 60
  python3 gnss_driver_nmea_validator.py --nmea-file raw_nmea.txt
  sudo python3 gnss_driver_nmea_validator.py --serial /dev/ttyAMA0 --baud 9600 --duration 60
  sudo python3 gnss_driver_nmea_validator.py --serial /dev/ttyUSB0 --ioctl /dev/neo6m0 --compare --duration 60

Important:
  If the serdev kernel driver owns the GNSS UART, you usually cannot also open
  the same UART from userspace. For raw NMEA validation, either unload the driver,
  use a UART tap, or log raw NMEA before loading the driver.
"""

from __future__ import annotations

import argparse
import fcntl
import math
import os
import select
import struct
import sys
import termios
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

# ============================================================
# Driver ABI from neo6m_gnss_ioctl.h
# ============================================================

# Packed struct neo6m_gnss_fix:
# s64, s8, 5*s32, 2*u16, 5*u8, u16, bool, bool = 42 bytes.
FIX_STRUCT_FORMAT = "<qbiiiiiHHBBBBBH??"
FIX_STRUCT_SIZE = struct.calcsize(FIX_STRUCT_FORMAT)

NEO6M_GNSS_IOC_MAGIC = ord("N")
NEO6M_GNSS_IOC_NR_GET_FIX = 1
_IOC_NRBITS = 8
_IOC_TYPEBITS = 8
_IOC_SIZEBITS = 14
_IOC_NRSHIFT = 0
_IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
_IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
_IOC_DIRSHIFT = _IOC_SIZESHIFT + _IOC_SIZEBITS
_IOC_READ = 2


def _IOC(direction: int, type_: int, nr: int, size: int) -> int:
    return ((direction << _IOC_DIRSHIFT) |
            (type_ << _IOC_TYPESHIFT) |
            (nr << _IOC_NRSHIFT) |
            (size << _IOC_SIZESHIFT))


NEO6M_GNSS_IOC_GET_FIX = _IOC(
    _IOC_READ,
    NEO6M_GNSS_IOC_MAGIC,
    NEO6M_GNSS_IOC_NR_GET_FIX,
    FIX_STRUCT_SIZE,
)

# ============================================================
# Thresholds
# ============================================================

KNOT_TO_MPS = 0.514444
KMH_TO_MPS = 1.0 / 3.6
EARTH_RADIUS_M = 6371008.8

DEFAULT_LATLON_EPS_DEG = 1e-8
DEFAULT_LATLON_DIFF_FAIL_DEG = 2e-7
DEFAULT_ALT_DIFF_FAIL_M = 1.0
DEFAULT_SPEED_DIFF_FAIL_MPS = 0.25
DEFAULT_COURSE_DIFF_FAIL_DEG = 2.0
DEFAULT_HDOP_DIFF_FAIL = 0.05
DEFAULT_MAX_GNSS_SPEED_MPS = 55.0
DEFAULT_MAX_GNSS_ACCEL_MPS2 = 10.0
DEFAULT_MAX_POS_JUMP_M = 100.0


@dataclass
class Fix:
    source: str = ""
    monotonic_ns: Optional[int] = None
    host_time_s: float = 0.0
    have_fix: Optional[bool] = None
    lat_deg: Optional[float] = None
    lon_deg: Optional[float] = None
    alt_m: Optional[float] = None
    speed_mps: Optional[float] = None
    course_deg: Optional[float] = None
    hdop: Optional[float] = None
    utc_year: Optional[int] = None
    utc_mon: Optional[int] = None
    utc_day: Optional[int] = None
    utc_hour: Optional[int] = None
    utc_min: Optional[int] = None
    utc_sec: Optional[int] = None
    utc_millis: Optional[int] = None
    heading_valid: Optional[bool] = None
    hdop_valid: Optional[bool] = None
    raw_sentence_type: Optional[str] = None
    raw_line: Optional[str] = None
    sequence: int = 0

    def utc_string(self) -> str:
        if self.utc_hour is None:
            return "NA"
        date = "????-??-??"
        if self.utc_year is not None and self.utc_mon is not None and self.utc_day is not None:
            date = f"{self.utc_year:04d}-{self.utc_mon:02d}-{self.utc_day:02d}"
        ms = self.utc_millis if self.utc_millis is not None else 0
        return f"{date}T{self.utc_hour:02d}:{self.utc_min or 0:02d}:{self.utc_sec or 0:02d}.{ms:03d}Z"

    def has_position(self) -> bool:
        return self.lat_deg is not None and self.lon_deg is not None

    def copy(self) -> "Fix":
        return Fix(**self.__dict__)


@dataclass
class Stats:
    samples: int = 0
    ioctl_reads: int = 0
    sysfs_reads: int = 0
    nmea_lines: int = 0
    nmea_good_checksum: int = 0
    nmea_bad_checksum: int = 0
    nmea_parsed_sentences: int = 0
    compare_count: int = 0
    mismatch_count: int = 0
    stale_driver_count: int = 0
    fresh_driver_count: int = 0
    stale_nmea_count: int = 0
    fresh_nmea_count: int = 0
    speed_spike_count: int = 0
    accel_spike_count: int = 0
    pos_jump_count: int = 0
    messages: List[str] = field(default_factory=list)

    def warn(self, msg: str) -> None:
        self.messages.append("WARN: " + msg)
        print("WARN:", msg)

    def fail(self, msg: str) -> None:
        self.messages.append("FAIL: " + msg)
        print("FAIL:", msg)


def finite(x: Optional[float]) -> bool:
    return x is not None and math.isfinite(float(x))


def wrap_deg_180(x: float) -> float:
    while x > 180.0:
        x -= 360.0
    while x <= -180.0:
        x += 360.0
    return x


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1r = math.radians(lat1)
    lon1r = math.radians(lon1)
    lat2r = math.radians(lat2)
    lon2r = math.radians(lon2)
    dlat = lat2r - lat1r
    dlon = lon2r - lon1r
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1r) * math.cos(lat2r) * math.sin(dlon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1 - a)))
    return EARTH_RADIUS_M * c


def parse_float(s: Optional[str]) -> Optional[float]:
    if s is None:
        return None
    s = s.strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        return None


def parse_int(s: Optional[str]) -> Optional[int]:
    if s is None:
        return None
    s = s.strip()
    if not s:
        return None
    try:
        return int(float(s))
    except ValueError:
        return None


def nmea_checksum(line: str) -> Optional[int]:
    line = line.strip()
    if not line.startswith("$"):
        return None
    star = line.find("*")
    if star < 0:
        return None
    csum = 0
    for ch in line[1:star]:
        csum ^= ord(ch)
    return csum & 0xFF


def nmea_checksum_ok(line: str) -> bool:
    line = line.strip()
    if not line.startswith("$"):
        return False
    star = line.find("*")
    if star < 0 or star + 2 >= len(line):
        return False
    calc = nmea_checksum(line)
    try:
        got = int(line[star + 1:star + 3], 16)
    except ValueError:
        return False
    return calc == got


def strip_nmea_checksum(line: str) -> str:
    line = line.strip()
    star = line.find("*")
    if star >= 0:
        return line[:star]
    return line


def nmea_latlon_to_deg(value: str, hemi: str) -> Optional[float]:
    if not value or not hemi:
        return None
    value = value.strip()
    hemi = hemi.strip().upper()
    if "." not in value or len(value) < 4:
        return None
    dot = value.find(".")
    deg_digits = dot - 2
    if deg_digits not in (2, 3):
        return None
    try:
        deg = int(value[:deg_digits])
        minutes = float(value[deg_digits:])
    except ValueError:
        return None
    out = deg + minutes / 60.0
    if hemi in ("S", "W"):
        out = -out
    elif hemi not in ("N", "E"):
        return None
    return out


def parse_nmea_time_hhmmss(fix: Fix, t: str) -> None:
    if not t:
        return
    whole = t.split(".")[0]
    if len(whole) < 6:
        return
    try:
        fix.utc_hour = max(0, min(23, int(whole[0:2])))
        fix.utc_min = max(0, min(59, int(whole[2:4])))
        fix.utc_sec = max(0, min(60, int(whole[4:6])))
        fix.utc_millis = 0
        if "." in t:
            frac = t.split(".", 1)[1]
            frac = (frac + "000")[:3]
            fix.utc_millis = max(0, min(999, int(frac)))
    except ValueError:
        return


def parse_nmea_date_ddmmyy(fix: Fix, d: str) -> None:
    if not d or len(d) < 6:
        return
    try:
        dd = int(d[0:2])
        mo = int(d[2:4])
        yy = int(d[4:6])
    except ValueError:
        return
    fix.utc_day = max(1, min(31, dd))
    fix.utc_mon = max(1, min(12, mo))
    fix.utc_year = 1900 + yy if yy >= 80 else 2000 + yy


def utc_key(fix: Fix) -> Optional[Tuple[int, int, int, int]]:
    if fix.utc_hour is None or fix.utc_min is None or fix.utc_sec is None:
        return None
    return (int(fix.utc_hour), int(fix.utc_min), int(fix.utc_sec), int(fix.utc_millis or 0))


class NMEAParser:
    def __init__(self) -> None:
        self.fix = Fix(source="nmea")
        self.sequence = 0

    def parse_line(self, line: str, stats: Optional[Stats] = None) -> Optional[Fix]:
        line = line.strip()
        if not line:
            return None
        if stats:
            stats.nmea_lines += 1
        if not line.startswith("$"):
            return None
        if not nmea_checksum_ok(line):
            if stats:
                stats.nmea_bad_checksum += 1
            return None
        if stats:
            stats.nmea_good_checksum += 1

        fields = strip_nmea_checksum(line).split(",")
        if not fields:
            return None
        sentence = fields[0][1:]
        sentence_type = sentence[-3:].upper() if len(sentence) >= 3 else sentence.upper()
        f = self.fix.copy()
        f.host_time_s = time.monotonic()
        f.raw_sentence_type = sentence_type
        f.raw_line = line

        if sentence_type == "RMC":
            updated = self._parse_rmc(fields, f)
        elif sentence_type == "GGA":
            updated = self._parse_gga(fields, f)
        elif sentence_type == "VTG":
            updated = self._parse_vtg(fields, f)
        else:
            return None
        if not updated:
            return None

        self.sequence += 1
        f.sequence = self.sequence
        self.fix = f
        if stats:
            stats.nmea_parsed_sentences += 1
        return f.copy()

    def _parse_rmc(self, fields: List[str], f: Fix) -> bool:
        if len(fields) < 10:
            return False
        parse_nmea_time_hhmmss(f, fields[1])
        status = fields[2].strip().upper() if len(fields) > 2 else ""
        f.have_fix = status == "A"
        lat = nmea_latlon_to_deg(fields[3], fields[4]) if len(fields) > 5 else None
        lon = nmea_latlon_to_deg(fields[5], fields[6]) if len(fields) > 6 else None
        if lat is not None:
            f.lat_deg = lat
        if lon is not None:
            f.lon_deg = lon
        sog_knots = parse_float(fields[7]) if len(fields) > 7 else None
        if sog_knots is not None:
            f.speed_mps = max(0.0, sog_knots * KNOT_TO_MPS)
        cog = parse_float(fields[8]) if len(fields) > 8 else None
        if cog is not None:
            f.course_deg = cog % 360.0
            f.heading_valid = True
        else:
            f.heading_valid = False
        parse_nmea_date_ddmmyy(f, fields[9])
        return True

    def _parse_gga(self, fields: List[str], f: Fix) -> bool:
        if len(fields) < 10:
            return False
        parse_nmea_time_hhmmss(f, fields[1])
        lat = nmea_latlon_to_deg(fields[2], fields[3]) if len(fields) > 4 else None
        lon = nmea_latlon_to_deg(fields[4], fields[5]) if len(fields) > 5 else None
        if lat is not None:
            f.lat_deg = lat
        if lon is not None:
            f.lon_deg = lon
        fixq = parse_int(fields[6]) if len(fields) > 6 else 0
        if fixq is not None:
            f.have_fix = fixq > 0
        hdop = parse_float(fields[8]) if len(fields) > 8 else None
        if hdop is not None:
            f.hdop = hdop
            f.hdop_valid = True
        alt = parse_float(fields[9]) if len(fields) > 9 else None
        if alt is not None:
            f.alt_m = alt
        return True

    def _parse_vtg(self, fields: List[str], f: Fix) -> bool:
        updated = False
        if len(fields) > 1:
            cog = parse_float(fields[1])
            if cog is not None:
                f.course_deg = cog % 360.0
                f.heading_valid = True
                updated = True
        speed_mps = None
        if len(fields) > 5:
            knots = parse_float(fields[5])
            if knots is not None:
                speed_mps = max(0.0, knots * KNOT_TO_MPS)
        if speed_mps is None and len(fields) > 7:
            kmh = parse_float(fields[7])
            if kmh is not None:
                speed_mps = max(0.0, kmh * KMH_TO_MPS)
        if speed_mps is not None:
            f.speed_mps = speed_mps
            updated = True
        return updated


def read_ioctl_fix(dev_path: str = "/dev/neo6m0") -> Fix:
    fd = os.open(dev_path, os.O_RDONLY)
    try:
        buf = bytearray(FIX_STRUCT_SIZE)
        fcntl.ioctl(fd, NEO6M_GNSS_IOC_GET_FIX, buf, True)
        values = struct.unpack(FIX_STRUCT_FORMAT, bytes(buf))
    finally:
        os.close(fd)
    (
        monotonic_ns, have_fix, lat_e7, lon_e7, alt_mm, speed_mmps,
        course_deg_e5, hdop_x100, utc_year, utc_mon, utc_day, utc_hour,
        utc_min, utc_sec, utc_millis, heading_valid, hdop_valid,
    ) = values
    fix = Fix(source="ioctl")
    fix.monotonic_ns = int(monotonic_ns)
    fix.host_time_s = time.monotonic()
    fix.have_fix = bool(have_fix)
    fix.lat_deg = lat_e7 / 1e7
    fix.lon_deg = lon_e7 / 1e7
    fix.alt_m = alt_mm / 1000.0
    fix.speed_mps = speed_mmps / 1000.0
    fix.course_deg = course_deg_e5 / 1e5
    fix.hdop = hdop_x100 / 100.0
    fix.utc_year = int(utc_year)
    fix.utc_mon = int(utc_mon)
    fix.utc_day = int(utc_day)
    fix.utc_hour = int(utc_hour)
    fix.utc_min = int(utc_min)
    fix.utc_sec = int(utc_sec)
    fix.utc_millis = int(utc_millis)
    fix.heading_valid = bool(heading_valid)
    fix.hdop_valid = bool(hdop_valid)
    return fix


def read_sysfs_fix(sysfs_dir: str = "/sys/class/neo6m_gnss/neo6m0") -> Fix:
    base = Path(sysfs_dir)

    def read_text(name: str) -> Optional[str]:
        try:
            return (base / name).read_text().strip()
        except OSError:
            return None

    fix = Fix(source="sysfs")
    fix.host_time_s = time.monotonic()
    lat_e7 = parse_int(read_text("lat"))
    lon_e7 = parse_int(read_text("lon"))
    alt_mm = parse_int(read_text("alt_mm"))
    speed_mmps = parse_int(read_text("speed_mmps"))
    have_fix = parse_int(read_text("have_fix"))
    hdop_x100 = parse_int(read_text("hdop_x100"))
    course_deg_e5 = parse_int(read_text("course_deg_e5"))
    utc = read_text("utc")
    if lat_e7 is not None:
        fix.lat_deg = lat_e7 / 1e7
    if lon_e7 is not None:
        fix.lon_deg = lon_e7 / 1e7
    if alt_mm is not None:
        fix.alt_m = alt_mm / 1000.0
    if speed_mmps is not None:
        fix.speed_mps = speed_mmps / 1000.0
    if have_fix is not None:
        fix.have_fix = bool(have_fix)
    if hdop_x100 is not None:
        fix.hdop = hdop_x100 / 100.0
        fix.hdop_valid = True
    if course_deg_e5 is not None:
        fix.course_deg = course_deg_e5 / 1e5
        fix.heading_valid = True
    if utc:
        try:
            date, tm = utc.replace("Z", "").split("T")
            y, mo, d = date.split("-")
            hh, mm, rest = tm.split(":")
            ss, ms = rest.split(".", 1) if "." in rest else (rest, "0")
            fix.utc_year = int(y)
            fix.utc_mon = int(mo)
            fix.utc_day = int(d)
            fix.utc_hour = int(hh)
            fix.utc_min = int(mm)
            fix.utc_sec = int(ss)
            fix.utc_millis = int((ms + "000")[:3])
        except Exception:
            pass
    return fix


def position_changed(a: Optional[Fix], b: Fix, eps_deg: float) -> bool:
    if a is None:
        return True
    if not a.has_position() or not b.has_position():
        return False
    return abs((a.lat_deg or 0) - (b.lat_deg or 0)) > eps_deg or abs((a.lon_deg or 0) - (b.lon_deg or 0)) > eps_deg


def utc_changed(a: Optional[Fix], b: Fix) -> bool:
    if a is None:
        return True
    au = utc_key(a)
    bu = utc_key(b)
    if au is None or bu is None:
        return False
    return au != bu


def monotonic_changed(a: Optional[Fix], b: Fix) -> bool:
    if a is None:
        return True
    if a.monotonic_ns is None or b.monotonic_ns is None:
        return False
    return a.monotonic_ns != b.monotonic_ns


def is_fresh(prev: Optional[Fix], cur: Fix, eps_deg: float) -> bool:
    return position_changed(prev, cur, eps_deg) or utc_changed(prev, cur) or monotonic_changed(prev, cur)


def validate_physical(prev: Optional[Fix], cur: Fix, stats: Stats, args: argparse.Namespace) -> None:
    if cur.has_position():
        if not (-90.0 <= (cur.lat_deg or 999) <= 90.0):
            stats.fail(f"{cur.source}: latitude out of range: {cur.lat_deg}")
        if not (-180.0 <= (cur.lon_deg or 999) <= 180.0):
            stats.fail(f"{cur.source}: longitude out of range: {cur.lon_deg}")
    if finite(cur.speed_mps) and (cur.speed_mps or 0) > args.max_speed_mps:
        stats.speed_spike_count += 1
        stats.fail(f"{cur.source}: speed too high: {cur.speed_mps:.3f} m/s")
    if finite(cur.course_deg) and not (0.0 <= (cur.course_deg or 0.0) < 360.0):
        stats.fail(f"{cur.source}: course outside 0..360: {cur.course_deg}")
    if prev and prev.has_position() and cur.has_position():
        dt = cur.host_time_s - prev.host_time_s
        if dt > 0:
            jump = haversine_m(prev.lat_deg, prev.lon_deg, cur.lat_deg, cur.lon_deg)
            implied_speed = jump / dt
            if jump > args.max_pos_jump_m:
                stats.pos_jump_count += 1
                stats.warn(f"{cur.source}: large position jump {jump:.2f} m in {dt:.2f} s; implied {implied_speed:.2f} m/s")
            if implied_speed > args.max_speed_mps:
                stats.speed_spike_count += 1
                stats.warn(f"{cur.source}: position-implied speed too high {implied_speed:.2f} m/s")
            if finite(prev.speed_mps) and finite(cur.speed_mps):
                acc = abs((cur.speed_mps or 0) - (prev.speed_mps or 0)) / dt
                if acc > args.max_accel_mps2:
                    stats.accel_spike_count += 1
                    stats.warn(f"{cur.source}: speed acceleration spike {acc:.2f} m/s^2 prev={prev.speed_mps:.2f} cur={cur.speed_mps:.2f}")


def compare_fixes(driver: Fix, ref: Fix, stats: Stats, args: argparse.Namespace) -> None:
    stats.compare_count += 1
    mismatches = []
    if ref.have_fix is not None and driver.have_fix is not None and bool(ref.have_fix) != bool(driver.have_fix):
        mismatches.append(f"have_fix driver={driver.have_fix} nmea={ref.have_fix}")
    if ref.lat_deg is not None and driver.lat_deg is not None:
        d = abs(driver.lat_deg - ref.lat_deg)
        if d > args.latlon_diff_fail_deg:
            mismatches.append(f"lat diff {d:.10f} deg driver={driver.lat_deg:.10f} nmea={ref.lat_deg:.10f}")
    if ref.lon_deg is not None and driver.lon_deg is not None:
        d = abs(driver.lon_deg - ref.lon_deg)
        if d > args.latlon_diff_fail_deg:
            mismatches.append(f"lon diff {d:.10f} deg driver={driver.lon_deg:.10f} nmea={ref.lon_deg:.10f}")
    if ref.alt_m is not None and driver.alt_m is not None:
        d = abs(driver.alt_m - ref.alt_m)
        if d > args.alt_diff_fail_m:
            mismatches.append(f"alt diff {d:.3f} m driver={driver.alt_m:.3f} nmea={ref.alt_m:.3f}")
    if ref.speed_mps is not None and driver.speed_mps is not None:
        d = abs(driver.speed_mps - ref.speed_mps)
        if d > args.speed_diff_fail_mps:
            mismatches.append(f"speed diff {d:.3f} m/s driver={driver.speed_mps:.3f} nmea={ref.speed_mps:.3f}")
    if ref.course_deg is not None and driver.course_deg is not None:
        d = abs(wrap_deg_180(driver.course_deg - ref.course_deg))
        if d > args.course_diff_fail_deg:
            mismatches.append(f"course diff {d:.3f} deg driver={driver.course_deg:.3f} nmea={ref.course_deg:.3f}")
    if ref.hdop is not None and driver.hdop is not None:
        d = abs(driver.hdop - ref.hdop)
        if d > args.hdop_diff_fail:
            mismatches.append(f"HDOP diff {d:.3f} driver={driver.hdop:.3f} nmea={ref.hdop:.3f}")
    if mismatches:
        stats.mismatch_count += 1
        stats.fail("; ".join(mismatches))


def print_fix(prefix: str, fix: Fix) -> None:
    lat = "NA" if fix.lat_deg is None else f"{fix.lat_deg:.10f}"
    lon = "NA" if fix.lon_deg is None else f"{fix.lon_deg:.10f}"
    alt = "NA" if fix.alt_m is None else f"{fix.alt_m:.3f}"
    spd = "NA" if fix.speed_mps is None else f"{fix.speed_mps:.3f}"
    crs = "NA" if fix.course_deg is None else f"{fix.course_deg:.3f}"
    hdop = "NA" if fix.hdop is None else f"{fix.hdop:.2f}"
    print(f"{prefix} fix={int(bool(fix.have_fix)) if fix.have_fix is not None else 'NA'} lat={lat} lon={lon} alt_m={alt} speed_mps={spd} course_deg={crs} hdop={hdop} utc={fix.utc_string()} heading_valid={fix.heading_valid} hdop_valid={fix.hdop_valid}")


BAUD_MAP = {
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}


def open_serial_stdlib(path: str, baud: int) -> int:
    if baud not in BAUD_MAP:
        raise ValueError(f"Unsupported baud {baud}. Supported: {sorted(BAUD_MAP)}")
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = BAUD_MAP[baud]
    attrs[5] = BAUD_MAP[baud]
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def iter_serial_lines(path: str, baud: int, duration_s: Optional[float]) -> Iterable[str]:
    fd = open_serial_stdlib(path, baud)
    start = time.monotonic()
    buf = bytearray()
    try:
        while True:
            if duration_s is not None and time.monotonic() - start >= duration_s:
                break
            r, _, _ = select.select([fd], [], [], 0.2)
            if not r:
                continue
            try:
                data = os.read(fd, 1024)
            except BlockingIOError:
                continue
            if not data:
                continue
            for b in data:
                if b == ord("\r"):
                    continue
                if b == ord("\n"):
                    if buf:
                        yield buf.decode("ascii", errors="replace").strip()
                        buf.clear()
                    continue
                if len(buf) < 512:
                    buf.append(b)
                else:
                    buf.clear()
    finally:
        os.close(fd)


def iter_file_lines(path: str) -> Iterable[str]:
    with open(path, "r", encoding="ascii", errors="replace") as f:
        for line in f:
            yield line.strip()


def iter_stdin_lines() -> Iterable[str]:
    for line in sys.stdin:
        yield line.strip()


def run_nmea_only(lines: Iterable[str], stats: Stats, args: argparse.Namespace) -> None:
    parser = NMEAParser()
    prev_nmea = None
    last_print = 0.0
    for line in lines:
        fix = parser.parse_line(line, stats)
        if fix is None:
            continue
        if is_fresh(prev_nmea, fix, args.latlon_eps_deg):
            stats.fresh_nmea_count += 1
        else:
            stats.stale_nmea_count += 1
        validate_physical(prev_nmea, fix, stats, args)
        now = time.monotonic()
        if args.verbose or now - last_print >= args.print_period:
            print_fix("[NMEA]", fix)
            last_print = now
        prev_nmea = fix


def run_driver_only(stats: Stats, args: argparse.Namespace) -> None:
    start = time.monotonic()
    prev_ioctl = None
    prev_sysfs = None
    last_print = 0.0
    while True:
        now = time.monotonic()
        if args.duration is not None and now - start >= args.duration:
            break
        ioctl_fix = None
        sysfs_fix = None
        if args.ioctl:
            ioctl_fix = read_ioctl_fix(args.ioctl)
            stats.ioctl_reads += 1
            if is_fresh(prev_ioctl, ioctl_fix, args.latlon_eps_deg):
                stats.fresh_driver_count += 1
            else:
                stats.stale_driver_count += 1
            validate_physical(prev_ioctl, ioctl_fix, stats, args)
            prev_ioctl = ioctl_fix
        if args.sysfs:
            sysfs_fix = read_sysfs_fix(args.sysfs)
            stats.sysfs_reads += 1
            validate_physical(prev_sysfs, sysfs_fix, stats, args)
            prev_sysfs = sysfs_fix
        if ioctl_fix is not None and sysfs_fix is not None:
            compare_fixes(sysfs_fix, ioctl_fix, stats, args)
        if args.verbose or now - last_print >= args.print_period:
            if ioctl_fix is not None:
                print_fix("[IOCTL]", ioctl_fix)
            if sysfs_fix is not None:
                print_fix("[SYSFS]", sysfs_fix)
            last_print = now
        stats.samples += 1
        time.sleep(args.period)


def run_compare_live(lines: Iterable[str], stats: Stats, args: argparse.Namespace) -> None:
    parser = NMEAParser()
    prev_nmea = None
    prev_ioctl = None
    last_print = 0.0
    for line in lines:
        fix = parser.parse_line(line, stats)
        if fix is None:
            continue
        if is_fresh(prev_nmea, fix, args.latlon_eps_deg):
            stats.fresh_nmea_count += 1
        else:
            stats.stale_nmea_count += 1
        validate_physical(prev_nmea, fix, stats, args)
        prev_nmea = fix
        drv = read_ioctl_fix(args.ioctl)
        stats.ioctl_reads += 1
        if is_fresh(prev_ioctl, drv, args.latlon_eps_deg):
            stats.fresh_driver_count += 1
        else:
            stats.stale_driver_count += 1
        validate_physical(prev_ioctl, drv, stats, args)
        compare_fixes(drv, fix, stats, args)
        prev_ioctl = drv
        now = time.monotonic()
        if args.verbose or now - last_print >= args.print_period:
            print_fix("[NMEA]", fix)
            print_fix("[IOCTL]", drv)
            last_print = now


def print_summary(stats: Stats) -> None:
    print()
    print("=" * 78)
    print("GNSS DRIVER / NMEA VALIDATION SUMMARY")
    print("=" * 78)
    print(f"FIX_STRUCT_SIZE                 : {FIX_STRUCT_SIZE} bytes")
    print(f"NEO6M_GNSS_IOC_GET_FIX           : 0x{NEO6M_GNSS_IOC_GET_FIX:08X}")
    print()
    print("NMEA")
    print("-" * 78)
    print(f"NMEA lines seen                  : {stats.nmea_lines}")
    print(f"NMEA checksum OK                 : {stats.nmea_good_checksum}")
    print(f"NMEA checksum BAD                : {stats.nmea_bad_checksum}")
    print(f"NMEA parsed RMC/GGA/VTG          : {stats.nmea_parsed_sentences}")
    print(f"NMEA fresh fixes                 : {stats.fresh_nmea_count}")
    print(f"NMEA stale/repeated fixes        : {stats.stale_nmea_count}")
    print()
    print("DRIVER")
    print("-" * 78)
    print(f"ioctl reads                      : {stats.ioctl_reads}")
    print(f"sysfs reads                      : {stats.sysfs_reads}")
    print(f"driver fresh snapshots           : {stats.fresh_driver_count}")
    print(f"driver stale/repeated snapshots  : {stats.stale_driver_count}")
    print()
    print("COMPARISON / VALIDATION")
    print("-" * 78)
    print(f"comparisons                      : {stats.compare_count}")
    print(f"mismatches                       : {stats.mismatch_count}")
    print(f"speed spike warnings             : {stats.speed_spike_count}")
    print(f"acceleration spike warnings      : {stats.accel_spike_count}")
    print(f"position jump warnings           : {stats.pos_jump_count}")
    print()
    if stats.mismatch_count == 0 and stats.nmea_bad_checksum == 0 and stats.speed_spike_count == 0:
        print("RESULT                           : PASS for checks performed")
    else:
        print("RESULT                           : CHECK FAIL/WARN items above")
    if stats.messages:
        print()
        print("MESSAGES")
        print("-" * 78)
        for msg in stats.messages[-50:]:
            print(msg)
        if len(stats.messages) > 50:
            print(f"... {len(stats.messages) - 50} older messages omitted")
    print("=" * 78)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate GNSS NMEA parsing and compare against kernel driver ioctl/sysfs.")
    src = p.add_mutually_exclusive_group()
    src.add_argument("--nmea-file", help="Read raw NMEA sentences from a text file.")
    src.add_argument("--serial", help="Read raw NMEA sentences from a serial UART device.")
    src.add_argument("--stdin", action="store_true", help="Read raw NMEA sentences from stdin.")
    p.add_argument("--baud", type=int, default=9600, help="Serial baud rate. Default: 9600.")
    p.add_argument("--ioctl", default=None, help="Driver char device path, normally /dev/neo6m0.")
    p.add_argument("--sysfs", default=None, help="Driver sysfs path, normally /sys/class/neo6m_gnss/neo6m0.")
    p.add_argument("--compare", action="store_true", help="Compare raw NMEA parser against ioctl snapshot.")
    p.add_argument("--duration", type=float, default=60.0, help="Run duration for live modes. Default: 60 s.")
    p.add_argument("--period", type=float, default=1.0, help="Driver polling period. Default: 1 s.")
    p.add_argument("--print-period", type=float, default=1.0, help="Print period. Default: 1 s.")
    p.add_argument("--verbose", action="store_true", help="Print every parsed/read sample.")
    p.add_argument("--latlon-eps-deg", type=float, default=DEFAULT_LATLON_EPS_DEG)
    p.add_argument("--latlon-diff-fail-deg", type=float, default=DEFAULT_LATLON_DIFF_FAIL_DEG)
    p.add_argument("--alt-diff-fail-m", type=float, default=DEFAULT_ALT_DIFF_FAIL_M)
    p.add_argument("--speed-diff-fail-mps", type=float, default=DEFAULT_SPEED_DIFF_FAIL_MPS)
    p.add_argument("--course-diff-fail-deg", type=float, default=DEFAULT_COURSE_DIFF_FAIL_DEG)
    p.add_argument("--hdop-diff-fail", type=float, default=DEFAULT_HDOP_DIFF_FAIL)
    p.add_argument("--max-speed-mps", type=float, default=DEFAULT_MAX_GNSS_SPEED_MPS)
    p.add_argument("--max-accel-mps2", type=float, default=DEFAULT_MAX_GNSS_ACCEL_MPS2)
    p.add_argument("--max-pos-jump-m", type=float, default=DEFAULT_MAX_POS_JUMP_M)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    stats = Stats()
    if FIX_STRUCT_SIZE != 42:
        print(f"ERROR: Unexpected Python struct size {FIX_STRUCT_SIZE}; expected 42.", file=sys.stderr)
        return 2
    try:
        if args.nmea_file or args.serial or args.stdin:
            if args.nmea_file:
                lines = iter_file_lines(args.nmea_file)
            elif args.serial:
                lines = iter_serial_lines(args.serial, args.baud, args.duration)
            else:
                lines = iter_stdin_lines()
            if args.compare:
                if not args.ioctl:
                    print("ERROR: --compare requires --ioctl /dev/neo6m0", file=sys.stderr)
                    return 2
                run_compare_live(lines, stats, args)
            else:
                run_nmea_only(lines, stats, args)
        else:
            if not args.ioctl and not args.sysfs:
                print("ERROR: provide --ioctl, --sysfs, --nmea-file, --serial, or --stdin", file=sys.stderr)
                return 2
            run_driver_only(stats, args)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except PermissionError as e:
        print(f"ERROR: Permission denied: {e}. Try sudo or check device permissions.", file=sys.stderr)
        return 1
    except OSError as e:
        print(f"ERROR: OS error: {e}", file=sys.stderr)
        return 1
    print_summary(stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
