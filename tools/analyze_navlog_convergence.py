#!/usr/bin/env python3
"""
analyze_navlog_convergence.py

Offline regression analysis for ground-vehicle dead-reckoning navigation logs.

Input:
    navlog_YYYY-MM-DD_HH-MM-SS.csv

Expected useful columns:
    lat_deg
    lon_deg
    ekf_lat
    ekf_lon
    err_H
    gnss_used_pos
    gnss_used_vel
    snap_applied
    zupt
    reacq_active
    timestamp / time / time_s / t / similar

Outputs:
    - Console summary
    - Optional PNG plots
    - Optional one-row summary CSV

Dependencies:
    Python standard library
    pandas
    numpy
    matplotlib
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd

# Matplotlib is imported only when plots are requested.


# ============================================================
# Configurable acceptance thresholds
# ============================================================

EARTH_RADIUS_M = 6371008.8

# Treat row-to-row EKF-GNSS error change inside +/- this band as "flat".
DEFAULT_ERROR_DEADBAND_M = 1.0

# If lat/lon differ by less than this amount, treat it as repeated/stale.
# 1e-8 deg is roughly millimetre/cm-level depending on latitude.
DEFAULT_FRESH_LATLON_EPS_DEG = 1e-8

ERROR_THRESHOLDS_M = [5, 10, 20, 50, 100, 200, 500]

# These are intentionally conservative default regression thresholds.
# Tune them for your vehicle/test route.
PASS_MAX_MEDIAN_ERR_M = 20.0
PASS_MAX_P95_ERR_M = 100.0
PASS_MIN_WITHIN_20M_PCT = 70.0
PASS_MAX_DIVERGING_PCT = 50.0
PASS_MIN_GNSS_FRESH_RATE_HZ = 0.5
PASS_MAX_LONGEST_STALE_S = 10.0
PASS_MAX_SNAP_PCT = 2.0
PASS_MIN_GNSS_POS_UPDATES = 5


# ============================================================
# Data structures
# ============================================================

@dataclass
class AnalysisSummary:
    input_file: str

    total_rows: int
    valid_position_rows: int

    duration_s: float
    csv_mean_dt_s: float
    csv_median_dt_s: float
    csv_mean_rate_hz: float
    csv_median_rate_hz: float

    gnss_fresh_rows: int
    gnss_repeated_rows: int
    gnss_fresh_rate_hz: float
    longest_repeated_rows: int
    longest_repeated_period_s: float

    err_mean_m: float
    err_median_m: float
    err_p75_m: float
    err_p95_m: float
    err_max_m: float

    row_converging_count: int
    row_diverging_count: int
    row_flat_count: int
    row_converging_pct: float
    row_diverging_pct: float
    row_flat_pct: float

    gnss_update_count: int
    gnss_update_converging_count: int
    gnss_update_diverging_count: int
    gnss_update_flat_count: int
    gnss_update_converging_pct: float
    gnss_update_diverging_pct: float
    gnss_update_flat_pct: float

    between_update_segments: int
    between_update_diverging_count: int
    between_update_converging_count: int
    between_update_flat_count: int

    snap_count: int
    snap_pct: float
    gnss_used_pos_count: int
    gnss_used_vel_count: int
    zupt_count: int
    reacq_active_count: int

    pass_fail: str
    fail_reasons: str


# ============================================================
# Utility functions
# ============================================================

def find_column(df: pd.DataFrame, candidates: List[str], required: bool = False) -> Optional[str]:
    """
    Find a column by exact case-insensitive match first, then normalized match.
    """
    lower_map = {c.lower(): c for c in df.columns}
    for cand in candidates:
        if cand.lower() in lower_map:
            return lower_map[cand.lower()]

    def norm(s: str) -> str:
        return s.lower().replace(" ", "").replace("-", "_")

    norm_map = {norm(c): c for c in df.columns}
    for cand in candidates:
        if norm(cand) in norm_map:
            return norm_map[norm(cand)]

    if required:
        raise KeyError(f"Required column not found. Tried: {candidates}")
    return None


def numeric_series(df: pd.DataFrame, col: Optional[str], default: float = np.nan) -> pd.Series:
    if col is None:
        return pd.Series([default] * len(df), index=df.index, dtype=float)
    return pd.to_numeric(df[col], errors="coerce")


def bool_series(df: pd.DataFrame, col: Optional[str]) -> pd.Series:
    if col is None:
        return pd.Series([False] * len(df), index=df.index, dtype=bool)

    s = df[col]

    if pd.api.types.is_bool_dtype(s):
        return s.fillna(False).astype(bool)

    if pd.api.types.is_numeric_dtype(s):
        return pd.to_numeric(s, errors="coerce").fillna(0).astype(float) != 0.0

    text = s.astype(str).str.strip().str.lower()
    return text.isin(["1", "true", "yes", "y", "on", "applied", "used"])


def parse_time_seconds(df: pd.DataFrame) -> Tuple[pd.Series, str]:
    """
    Returns elapsed time in seconds and source description.
    If no usable time column is found, use row index as seconds.
    """
    candidates = [
        "timestamp",
        "time",
        "time_s",
        "t_s",
        "t",
        "sec",
        "seconds",
        "mono_s",
        "monotonic_s",
        "now_s",
        "unix_s",
        "utc_s",
        "log_time",
    ]

    col = find_column(df, candidates, required=False)

    if col is None:
        return pd.Series(np.arange(len(df), dtype=float), index=df.index), "row_index_assumed_1Hz"

    raw = df[col]

    # Try numeric time first.
    numeric = pd.to_numeric(raw, errors="coerce")
    if numeric.notna().sum() >= max(3, int(0.5 * len(df))):
        t = numeric.astype(float)
        t = t - t.dropna().iloc[0]
        return t, col

    # Try datetime.
    dt = pd.to_datetime(raw, errors="coerce", utc=True)
    if dt.notna().sum() >= max(3, int(0.5 * len(df))):
        t = (dt - dt.dropna().iloc[0]).dt.total_seconds()
        return t.astype(float), col

    # Fallback.
    return pd.Series(np.arange(len(df), dtype=float), index=df.index), "row_index_assumed_1Hz"


def haversine_m(lat1_deg: np.ndarray, lon1_deg: np.ndarray,
                lat2_deg: np.ndarray, lon2_deg: np.ndarray) -> np.ndarray:
    lat1 = np.radians(lat1_deg)
    lon1 = np.radians(lon1_deg)
    lat2 = np.radians(lat2_deg)
    lon2 = np.radians(lon2_deg)

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = np.sin(dlat / 2.0) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2.0) ** 2
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))
    return EARTH_RADIUS_M * c


def latlon_to_local_enu(lat_deg: pd.Series, lon_deg: pd.Series,
                        lat0_deg: float, lon0_deg: float) -> Tuple[np.ndarray, np.ndarray]:
    """
    Simple local tangent approximation suitable for short vehicle routes.
    Returns East, North in metres.
    """
    lat = np.radians(lat_deg.astype(float).to_numpy())
    lon = np.radians(lon_deg.astype(float).to_numpy())
    lat0 = math.radians(float(lat0_deg))
    lon0 = math.radians(float(lon0_deg))

    east = EARTH_RADIUS_M * math.cos(lat0) * (lon - lon0)
    north = EARTH_RADIUS_M * (lat - lat0)
    return east, north


def compute_fresh_gnss(lat: pd.Series,
                       lon: pd.Series,
                       eps_deg: float) -> pd.Series:
    """
    A row is fresh if lat/lon changed from previous row beyond eps.
    First valid row is treated as fresh.
    """
    lat_np = lat.to_numpy(dtype=float)
    lon_np = lon.to_numpy(dtype=float)

    fresh = np.zeros(len(lat_np), dtype=bool)
    prev_valid_idx = None

    for i in range(len(lat_np)):
        if not np.isfinite(lat_np[i]) or not np.isfinite(lon_np[i]):
            fresh[i] = False
            continue

        if prev_valid_idx is None:
            fresh[i] = True
            prev_valid_idx = i
            continue

        dlat = abs(lat_np[i] - lat_np[prev_valid_idx])
        dlon = abs(lon_np[i] - lon_np[prev_valid_idx])

        fresh[i] = (dlat > eps_deg) or (dlon > eps_deg)

        if fresh[i]:
            prev_valid_idx = i

    return pd.Series(fresh, index=lat.index)


def longest_repeated_run(fresh: pd.Series, t_s: pd.Series) -> Tuple[int, float]:
    """
    Finds longest consecutive run where fresh == False.
    Returns number of repeated rows and elapsed period in seconds.
    """
    fresh_np = fresh.fillna(False).to_numpy(dtype=bool)
    t_np = t_s.to_numpy(dtype=float)

    best_len = 0
    best_period = 0.0

    run_start = None
    run_len = 0

    for i, is_fresh in enumerate(fresh_np):
        if not is_fresh:
            if run_start is None:
                run_start = i
                run_len = 1
            else:
                run_len += 1
        else:
            if run_start is not None:
                run_end = i - 1
                period = safe_time_span(t_np, run_start, run_end)
                if run_len > best_len:
                    best_len = run_len
                    best_period = period
                run_start = None
                run_len = 0

    if run_start is not None:
        run_end = len(fresh_np) - 1
        period = safe_time_span(t_np, run_start, run_end)
        if run_len > best_len:
            best_len = run_len
            best_period = period

    return best_len, best_period


def safe_time_span(t_np: np.ndarray, start: int, end: int) -> float:
    if start < 0 or end < start or end >= len(t_np):
        return 0.0
    if np.isfinite(t_np[start]) and np.isfinite(t_np[end]):
        return max(0.0, float(t_np[end] - t_np[start]))
    return 0.0


def classify_delta(delta: pd.Series, deadband_m: float) -> Tuple[pd.Series, pd.Series, pd.Series]:
    converging = delta < -deadband_m
    diverging = delta > deadband_m
    flat = ~(converging | diverging)
    return converging.fillna(False), diverging.fillna(False), flat.fillna(False)


def pct(count: int, total: int) -> float:
    if total <= 0:
        return 0.0
    return 100.0 * float(count) / float(total)


def safe_percentile(values: pd.Series, q: float) -> float:
    v = pd.to_numeric(values, errors="coerce").dropna()
    if len(v) == 0:
        return float("nan")
    return float(np.percentile(v.to_numpy(dtype=float), q))


def safe_mean_dt(t_s: pd.Series) -> Tuple[float, float, float, float, float]:
    t = pd.to_numeric(t_s, errors="coerce").dropna().to_numpy(dtype=float)
    if len(t) < 2:
        return 0.0, float("nan"), float("nan"), 0.0, 0.0

    dt = np.diff(t)
    dt = dt[np.isfinite(dt)]
    dt = dt[dt > 0]

    if len(dt) == 0:
        duration = float(t[-1] - t[0]) if len(t) >= 2 else 0.0
        return duration, float("nan"), float("nan"), 0.0, 0.0

    duration = float(t[-1] - t[0])
    mean_dt = float(np.mean(dt))
    median_dt = float(np.median(dt))
    mean_rate = 1.0 / mean_dt if mean_dt > 0 else 0.0
    median_rate = 1.0 / median_dt if median_dt > 0 else 0.0
    return duration, mean_dt, median_dt, mean_rate, median_rate


# ============================================================
# Core analysis
# ============================================================

def analyze_csv(csv_path: Path,
                deadband_m: float,
                fresh_eps_deg: float) -> Tuple[pd.DataFrame, AnalysisSummary]:
    df = pd.read_csv(csv_path)

    # Locate columns.
    lat_col = find_column(df, ["lat_deg", "lat", "gnss_lat", "gps_lat"], required=True)
    lon_col = find_column(df, ["lon_deg", "lon", "lng", "longitude", "gnss_lon", "gps_lon"], required=True)
    ekf_lat_col = find_column(df, ["ekf_lat", "ekf_lat_deg"], required=True)
    ekf_lon_col = find_column(df, ["ekf_lon", "ekf_lon_deg"], required=True)

    err_col = find_column(df, ["err_H", "err_h", "error_h", "ekf_gnss_err_m"], required=False)

    gnss_used_pos_col = find_column(df, ["gnss_used_pos", "gnss_pos_update_applied"], required=False)
    gnss_used_vel_col = find_column(df, ["gnss_used_vel", "gnss_vel_update_applied"], required=False)
    snap_col = find_column(df, ["snap_applied", "snap"], required=False)
    zupt_col = find_column(df, ["zupt", "zupt_active"], required=False)
    reacq_col = find_column(df, ["reacq_active", "reacquisition_active"], required=False)

    lat = numeric_series(df, lat_col)
    lon = numeric_series(df, lon_col)
    ekf_lat = numeric_series(df, ekf_lat_col)
    ekf_lon = numeric_series(df, ekf_lon_col)

    t_s, time_source = parse_time_seconds(df)
    df["_analysis_t_s"] = t_s

    valid_pos = (
        lat.notna() & lon.notna() &
        ekf_lat.notna() & ekf_lon.notna() &
        np.isfinite(lat) & np.isfinite(lon) &
        np.isfinite(ekf_lat) & np.isfinite(ekf_lon)
    )

    if err_col is not None:
        err_h = numeric_series(df, err_col)
        # If err_H is missing or partially invalid, fill from haversine.
        computed_err = pd.Series(
            haversine_m(lat.to_numpy(dtype=float),
                        lon.to_numpy(dtype=float),
                        ekf_lat.to_numpy(dtype=float),
                        ekf_lon.to_numpy(dtype=float)),
            index=df.index,
        )
        err_h = err_h.where(err_h.notna(), computed_err)
    else:
        err_h = pd.Series(
            haversine_m(lat.to_numpy(dtype=float),
                        lon.to_numpy(dtype=float),
                        ekf_lat.to_numpy(dtype=float),
                        ekf_lon.to_numpy(dtype=float)),
            index=df.index,
        )

    err_h = err_h.where(valid_pos, np.nan)
    df["_analysis_err_H_m"] = err_h

    gnss_used_pos = bool_series(df, gnss_used_pos_col)
    gnss_used_vel = bool_series(df, gnss_used_vel_col)
    snap_applied = bool_series(df, snap_col)
    zupt = bool_series(df, zupt_col)
    reacq_active = bool_series(df, reacq_col)

    fresh = compute_fresh_gnss(lat, lon, fresh_eps_deg)
    fresh = fresh.where(lat.notna() & lon.notna(), False)
    df["_analysis_gnss_fresh"] = fresh.astype(int)
    df["_analysis_gnss_repeated"] = (~fresh).astype(int)

    longest_rep_rows, longest_rep_period_s = longest_repeated_run(fresh, t_s)

    duration_s, mean_dt_s, median_dt_s, mean_rate_hz, median_rate_hz = safe_mean_dt(t_s)

    gnss_fresh_rows = int(fresh.sum())
    gnss_repeated_rows = int((~fresh).sum())
    gnss_fresh_rate_hz = float(gnss_fresh_rows / duration_s) if duration_s > 0 else 0.0

    # Row-to-row convergence.
    err_delta = err_h.diff()
    df["_analysis_err_delta_m"] = err_delta

    row_conv, row_div, row_flat = classify_delta(err_delta, deadband_m)
    # First row cannot be classified.
    if len(row_flat) > 0:
        row_flat.iloc[0] = False

    df["_analysis_converging"] = row_conv.astype(int)
    df["_analysis_diverging"] = row_div.astype(int)
    df["_analysis_flat"] = row_flat.astype(int)

    comparable_rows = int((err_delta.notna()).sum())
    row_converging_count = int(row_conv.sum())
    row_diverging_count = int(row_div.sum())
    row_flat_count = int(row_flat.sum())

    # Convergence at GNSS update rows.
    update_mask = gnss_used_pos & err_delta.notna()
    update_delta = err_delta.where(update_mask)

    upd_conv, upd_div, upd_flat = classify_delta(update_delta, deadband_m)

    gnss_update_count = int(gnss_used_pos.sum())
    gnss_update_comparable = int(update_mask.sum())
    gnss_update_converging_count = int(upd_conv.sum())
    gnss_update_diverging_count = int(upd_div.sum())
    gnss_update_flat_count = int(upd_flat.sum())

    # Between GNSS updates:
    # For each pair of accepted GNSS position update rows, compare error at the
    # previous update row to error immediately before the next update row.
    update_indices = list(np.where(gnss_used_pos.to_numpy(dtype=bool))[0])
    between_segments = 0
    between_div = 0
    between_conv = 0
    between_flat = 0

    err_np = err_h.to_numpy(dtype=float)

    for a, b in zip(update_indices[:-1], update_indices[1:]):
        if b <= a + 1:
            continue

        start_err = err_np[a]
        end_err = err_np[b - 1]

        if not np.isfinite(start_err) or not np.isfinite(end_err):
            continue

        d = end_err - start_err
        between_segments += 1

        if d > deadband_m:
            between_div += 1
        elif d < -deadband_m:
            between_conv += 1
        else:
            between_flat += 1

    valid_err = err_h.dropna()

    within: Dict[int, float] = {}
    for th in ERROR_THRESHOLDS_M:
        within[th] = pct(int((valid_err <= th).sum()), len(valid_err))

    snap_count = int(snap_applied.sum())
    snap_pct = pct(snap_count, len(df))

    gnss_used_pos_count = int(gnss_used_pos.sum())
    gnss_used_vel_count = int(gnss_used_vel.sum())
    zupt_count = int(zupt.sum())
    reacq_active_count = int(reacq_active.sum())

    # PASS/FAIL rules.
    fail_reasons: List[str] = []

    err_median = safe_percentile(valid_err, 50)
    err_p95 = safe_percentile(valid_err, 95)
    within_20 = within[20]

    row_div_pct = pct(row_diverging_count, comparable_rows)
    row_conv_pct = pct(row_converging_count, comparable_rows)
    row_flat_pct = pct(row_flat_count, comparable_rows)

    if np.isfinite(err_median) and err_median > PASS_MAX_MEDIAN_ERR_M:
        fail_reasons.append(f"median error {err_median:.2f} m > {PASS_MAX_MEDIAN_ERR_M:.2f} m")

    if np.isfinite(err_p95) and err_p95 > PASS_MAX_P95_ERR_M:
        fail_reasons.append(f"p95 error {err_p95:.2f} m > {PASS_MAX_P95_ERR_M:.2f} m")

    if within_20 < PASS_MIN_WITHIN_20M_PCT:
        fail_reasons.append(f"within 20 m {within_20:.1f}% < {PASS_MIN_WITHIN_20M_PCT:.1f}%")

    if row_div_pct > PASS_MAX_DIVERGING_PCT:
        fail_reasons.append(f"row divergence {row_div_pct:.1f}% > {PASS_MAX_DIVERGING_PCT:.1f}%")

    if gnss_fresh_rate_hz < PASS_MIN_GNSS_FRESH_RATE_HZ:
        fail_reasons.append(f"GNSS fresh rate {gnss_fresh_rate_hz:.3f} Hz < {PASS_MIN_GNSS_FRESH_RATE_HZ:.3f} Hz")

    if longest_rep_period_s > PASS_MAX_LONGEST_STALE_S:
        fail_reasons.append(f"longest stale period {longest_rep_period_s:.1f} s > {PASS_MAX_LONGEST_STALE_S:.1f} s")

    if snap_pct > PASS_MAX_SNAP_PCT:
        fail_reasons.append(f"snap percentage {snap_pct:.2f}% > {PASS_MAX_SNAP_PCT:.2f}%")

    if gnss_used_pos_count < PASS_MIN_GNSS_POS_UPDATES:
        fail_reasons.append(f"GNSS position updates {gnss_used_pos_count} < {PASS_MIN_GNSS_POS_UPDATES}")

    pass_fail = "PASS" if not fail_reasons else "FAIL"

    summary = AnalysisSummary(
        input_file=str(csv_path),

        total_rows=int(len(df)),
        valid_position_rows=int(valid_pos.sum()),

        duration_s=float(duration_s),
        csv_mean_dt_s=float(mean_dt_s),
        csv_median_dt_s=float(median_dt_s),
        csv_mean_rate_hz=float(mean_rate_hz),
        csv_median_rate_hz=float(median_rate_hz),

        gnss_fresh_rows=gnss_fresh_rows,
        gnss_repeated_rows=gnss_repeated_rows,
        gnss_fresh_rate_hz=float(gnss_fresh_rate_hz),
        longest_repeated_rows=int(longest_rep_rows),
        longest_repeated_period_s=float(longest_rep_period_s),

        err_mean_m=float(valid_err.mean()) if len(valid_err) else float("nan"),
        err_median_m=float(err_median),
        err_p75_m=float(safe_percentile(valid_err, 75)),
        err_p95_m=float(err_p95),
        err_max_m=float(valid_err.max()) if len(valid_err) else float("nan"),

        row_converging_count=row_converging_count,
        row_diverging_count=row_diverging_count,
        row_flat_count=row_flat_count,
        row_converging_pct=float(row_conv_pct),
        row_diverging_pct=float(row_div_pct),
        row_flat_pct=float(row_flat_pct),

        gnss_update_count=gnss_update_count,
        gnss_update_converging_count=gnss_update_converging_count,
        gnss_update_diverging_count=gnss_update_diverging_count,
        gnss_update_flat_count=gnss_update_flat_count,
        gnss_update_converging_pct=pct(gnss_update_converging_count, gnss_update_comparable),
        gnss_update_diverging_pct=pct(gnss_update_diverging_count, gnss_update_comparable),
        gnss_update_flat_pct=pct(gnss_update_flat_count, gnss_update_comparable),

        between_update_segments=between_segments,
        between_update_diverging_count=between_div,
        between_update_converging_count=between_conv,
        between_update_flat_count=between_flat,

        snap_count=snap_count,
        snap_pct=float(snap_pct),
        gnss_used_pos_count=gnss_used_pos_count,
        gnss_used_vel_count=gnss_used_vel_count,
        zupt_count=zupt_count,
        reacq_active_count=reacq_active_count,

        pass_fail=pass_fail,
        fail_reasons="; ".join(fail_reasons) if fail_reasons else "none",
    )

    # Attach useful metadata for printing/plotting.
    df.attrs["time_source"] = time_source
    df.attrs["within_thresholds_pct"] = within

    # Store local ENU for plotting.
    valid_gnss = lat.notna() & lon.notna()
    if valid_gnss.any():
        first_idx = valid_gnss[valid_gnss].index[0]
        lat0 = float(lat.loc[first_idx])
        lon0 = float(lon.loc[first_idx])

        gnss_e, gnss_n = latlon_to_local_enu(lat, lon, lat0, lon0)
        ekf_e, ekf_n = latlon_to_local_enu(ekf_lat, ekf_lon, lat0, lon0)

        df["_analysis_gnss_E_m"] = gnss_e
        df["_analysis_gnss_N_m"] = gnss_n
        df["_analysis_ekf_E_m"] = ekf_e
        df["_analysis_ekf_N_m"] = ekf_n

    return df, summary


# ============================================================
# Reporting
# ============================================================

def print_summary(summary: AnalysisSummary, within: Dict[int, float], time_source: str) -> None:
    print()
    print("=" * 72)
    print("NAVLOG CONVERGENCE ANALYSIS")
    print("=" * 72)
    print(f"Input file                 : {summary.input_file}")
    print(f"Time source                : {time_source}")
    print(f"Total rows                 : {summary.total_rows}")
    print(f"Valid position rows        : {summary.valid_position_rows}")
    print(f"Duration                   : {summary.duration_s:.3f} s ({summary.duration_s / 60.0:.2f} min)")
    print(f"CSV mean dt / rate         : {summary.csv_mean_dt_s:.4f} s / {summary.csv_mean_rate_hz:.3f} Hz")
    print(f"CSV median dt / rate       : {summary.csv_median_dt_s:.4f} s / {summary.csv_median_rate_hz:.3f} Hz")

    print()
    print("GNSS FRESHNESS")
    print("-" * 72)
    print(f"Fresh lat/lon rows         : {summary.gnss_fresh_rows}")
    print(f"Repeated/stale rows        : {summary.gnss_repeated_rows}")
    print(f"Approx fresh GNSS rate     : {summary.gnss_fresh_rate_hz:.3f} Hz")
    print(f"Longest repeated rows      : {summary.longest_repeated_rows}")
    print(f"Longest repeated period    : {summary.longest_repeated_period_s:.3f} s")

    print()
    print("EKF-GNSS HORIZONTAL ERROR")
    print("-" * 72)
    print(f"Mean error                 : {summary.err_mean_m:.3f} m")
    print(f"Median error               : {summary.err_median_m:.3f} m")
    print(f"75th percentile            : {summary.err_p75_m:.3f} m")
    print(f"95th percentile            : {summary.err_p95_m:.3f} m")
    print(f"Max error                  : {summary.err_max_m:.3f} m")

    print()
    print("ERROR THRESHOLD COVERAGE")
    print("-" * 72)
    for th in ERROR_THRESHOLDS_M:
        print(f"Rows within {th:>3} m          : {within.get(th, 0.0):6.2f}%")

    print()
    print("ROW-TO-ROW CONVERGENCE, 1 M DEFAULT DEADBAND")
    print("-" * 72)
    print(f"Converging rows            : {summary.row_converging_count} ({summary.row_converging_pct:.2f}%)")
    print(f"Diverging rows             : {summary.row_diverging_count} ({summary.row_diverging_pct:.2f}%)")
    print(f"Flat rows                  : {summary.row_flat_count} ({summary.row_flat_pct:.2f}%)")

    print()
    print("AT GNSS POSITION UPDATE ROWS")
    print("-" * 72)
    print(f"GNSS position updates      : {summary.gnss_update_count}")
    print(f"Update rows converging     : {summary.gnss_update_converging_count} ({summary.gnss_update_converging_pct:.2f}%)")
    print(f"Update rows diverging      : {summary.gnss_update_diverging_count} ({summary.gnss_update_diverging_pct:.2f}%)")
    print(f"Update rows flat           : {summary.gnss_update_flat_count} ({summary.gnss_update_flat_pct:.2f}%)")

    print()
    print("BETWEEN GNSS POSITION UPDATES")
    print("-" * 72)
    print(f"Comparable segments        : {summary.between_update_segments}")
    print(f"Segments diverging         : {summary.between_update_diverging_count}")
    print(f"Segments converging        : {summary.between_update_converging_count}")
    print(f"Segments flat              : {summary.between_update_flat_count}")

    print()
    print("EVENT COUNTS")
    print("-" * 72)
    print(f"Snap events                : {summary.snap_count} ({summary.snap_pct:.2f}%)")
    print(f"GNSS used position count   : {summary.gnss_used_pos_count}")
    print(f"GNSS used velocity count   : {summary.gnss_used_vel_count}")
    print(f"ZUPT count                 : {summary.zupt_count}")
    print(f"Reacq active count         : {summary.reacq_active_count}")

    print()
    print("FINAL REGRESSION RESULT")
    print("-" * 72)
    print(f"Result                     : {summary.pass_fail}")
    print(f"Reason(s)                  : {summary.fail_reasons}")
    print("=" * 72)
    print()


def write_summary_csv(summary: AnalysisSummary, output_path: Path) -> None:
    data = asdict(summary)
    pd.DataFrame([data]).to_csv(output_path, index=False)
    print(f"Wrote summary CSV: {output_path}")


# ============================================================
# Plotting
# ============================================================

def make_plots(df: pd.DataFrame, out_dir: Path, stem: str) -> None:
    import matplotlib.pyplot as plt

    out_dir.mkdir(parents=True, exist_ok=True)

    t = df["_analysis_t_s"]
    err = df["_analysis_err_H_m"]

    # 1. Error versus time.
    fig = plt.figure(figsize=(11, 5))
    plt.plot(t, err)
    plt.xlabel("Time (s)")
    plt.ylabel("EKF-GNSS horizontal error (m)")
    plt.title("EKF-GNSS Error versus Time")
    plt.grid(True)
    path = out_dir / f"{stem}_error_vs_time.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)

    # 2. Converging/diverging trend.
    trend = np.zeros(len(df), dtype=float)
    trend[df["_analysis_converging"].astype(bool).to_numpy()] = -1.0
    trend[df["_analysis_diverging"].astype(bool).to_numpy()] = 1.0

    fig = plt.figure(figsize=(11, 4))
    plt.plot(t, trend, marker=".", linestyle="none")
    plt.yticks([-1, 0, 1], ["converging", "flat", "diverging"])
    plt.xlabel("Time (s)")
    plt.title("Converging / Diverging Classification")
    plt.grid(True)
    path = out_dir / f"{stem}_convergence_trend.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)

    # 3. GNSS fresh/repeated state.
    fresh = df["_analysis_gnss_fresh"].astype(int)
    fig = plt.figure(figsize=(11, 4))
    plt.plot(t, fresh, marker=".", linestyle="none")
    plt.yticks([0, 1], ["repeated/stale", "fresh"])
    plt.xlabel("Time (s)")
    plt.title("GNSS Fresh Position Detection")
    plt.grid(True)
    path = out_dir / f"{stem}_gnss_freshness.png"
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)

    # 4. EKF and GNSS track overlay in local ENU.
    required = [
        "_analysis_gnss_E_m",
        "_analysis_gnss_N_m",
        "_analysis_ekf_E_m",
        "_analysis_ekf_N_m",
    ]

    if all(c in df.columns for c in required):
        fig = plt.figure(figsize=(8, 8))
        plt.plot(df["_analysis_gnss_E_m"], df["_analysis_gnss_N_m"], label="GNSS")
        plt.plot(df["_analysis_ekf_E_m"], df["_analysis_ekf_N_m"], label="EKF")
        plt.xlabel("East (m)")
        plt.ylabel("North (m)")
        plt.title("GNSS and EKF Track Overlay")
        plt.axis("equal")
        plt.grid(True)
        plt.legend()
        path = out_dir / f"{stem}_track_overlay_enu.png"
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)

    print(f"Wrote plots to: {out_dir}")


# ============================================================
# Main
# ============================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Analyze EKF-GNSS convergence from navigation CSV logs."
    )

    parser.add_argument(
        "csv",
        type=str,
        help="Input navlog CSV file, e.g. navlog_2026-05-07_00-45-26.csv",
    )

    parser.add_argument(
        "--deadband-m",
        type=float,
        default=DEFAULT_ERROR_DEADBAND_M,
        help=f"Deadband for convergence/divergence classification in metres. Default: {DEFAULT_ERROR_DEADBAND_M}",
    )

    parser.add_argument(
        "--fresh-eps-deg",
        type=float,
        default=DEFAULT_FRESH_LATLON_EPS_DEG,
        help=f"Lat/lon change threshold for fresh GNSS detection in degrees. Default: {DEFAULT_FRESH_LATLON_EPS_DEG}",
    )

    parser.add_argument(
        "--plots",
        action="store_true",
        help="Generate PNG plots.",
    )

    parser.add_argument(
        "--out-dir",
        type=str,
        default=None,
        help="Output directory for plots. Default: <csv_stem>_analysis",
    )

    parser.add_argument(
        "--summary-csv",
        type=str,
        default=None,
        help="Optional output summary CSV path.",
    )

    parser.add_argument(
        "--annotated-csv",
        type=str,
        default=None,
        help="Optional output CSV with analysis columns appended.",
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    csv_path = Path(args.csv)
    if not csv_path.exists():
        print(f"ERROR: input CSV does not exist: {csv_path}", file=sys.stderr)
        return 2

    try:
        df, summary = analyze_csv(
            csv_path=csv_path,
            deadband_m=args.deadband_m,
            fresh_eps_deg=args.fresh_eps_deg,
        )
    except Exception as exc:
        print(f"ERROR: failed to analyze CSV: {exc}", file=sys.stderr)
        return 1

    within = df.attrs.get("within_thresholds_pct", {})
    time_source = df.attrs.get("time_source", "unknown")

    print_summary(summary, within, time_source)

    if args.summary_csv:
        write_summary_csv(summary, Path(args.summary_csv))

    if args.annotated_csv:
        annotated_path = Path(args.annotated_csv)
        df.to_csv(annotated_path, index=False)
        print(f"Wrote annotated CSV: {annotated_path}")

    if args.plots:
        out_dir = Path(args.out_dir) if args.out_dir else Path(f"{csv_path.stem}_analysis")
        make_plots(df, out_dir=out_dir, stem=csv_path.stem)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())