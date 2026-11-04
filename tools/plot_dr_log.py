#!/usr/bin/env python3
"""Analyse a navlog_*.csv written by dr_dead_reckoning_app (platform/pi).

Answers "is the dead reckoning working?":
  * summary: duration, distance, time in each mode, GNSS usage, stops (ZUPT/ZARU),
    final bias estimates (yaw-rate bias in deg/min, accel bias, yaw scale)
  * every GNSS outage (simulated with --outage-test, or real): duration,
    distance driven, DR error at the end of the outage, error as % of distance
  * plots (PNG): track, error vs time, speed, heading, biases, mode timeline
  * KML with the GNSS and DR tracks (open in Google Earth)

Usage:
  python tools/plot_dr_log.py navlog_2026-11-05_10-00-00.csv            # PNGs + KML next to the CSV
  python tools/plot_dr_log.py navlog.csv --show                        # also open the plot windows
  python tools/plot_dr_log.py navlog.csv --out results/run1

What good looks like (road vehicle, this IMU):
  * outage error grows smoothly and stays within ~2-3 % of the distance driven
    for 30 s outages; heading error at the end of an outage < ~1 deg
  * bg settles (|bg| < ~0.01 deg/s) and stays put; it should not jump at stops
  * no repeated re-anchoring (snap_applied) while GNSS is good
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

# Validated categorical slots 1-3 (dataviz reference palette) + ink.
C1, C2, C3, INK2, GRID = "#2a78d6", "#eb6834", "#1baf7a", "#52514e", "#e4e3df"


def segments(mask: np.ndarray):
    """(start, end) index pairs of runs where mask is True (end inclusive)."""
    m = np.r_[False, mask.astype(bool), False]
    d = np.diff(m.astype(int))
    return list(zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1))


def analyse(df: pd.DataFrame) -> dict:
    t = df["time_s"].to_numpy(float)
    dt = np.r_[0.0, np.diff(t)]
    speed = df["speed_mps"].to_numpy(float)
    aligned = df["mode"].isin(["NAV_GNSS", "NAV_DR"]).to_numpy()
    out = {"duration_s": t[-1] - t[0] if len(t) else 0.0,
           "distance_m": float(np.nansum(np.abs(speed) * dt * aligned))}
    out["modes_pct"] = (df["mode"].value_counts(normalize=True) * 100).round(1).to_dict()
    out["gnss_pos_used"] = int(df.get("gnss_used_pos", pd.Series(dtype=int)).sum())
    out["gnss_rejects"] = int(df.get("gnss_reject", pd.Series(dtype=int)).sum())
    out["reanchors"] = int(df.get("snap_applied", pd.Series(dtype=int)).sum())
    zupt = df.get("zupt_active", pd.Series(0, index=df.index)).to_numpy(int).astype(bool)
    out["stopped_s"] = float(np.sum(dt[zupt]))
    out["stops"] = len(segments(zupt))
    last = df[aligned].tail(1)
    if len(last):
        out["bg_deg_per_min"] = float(last["bg_dps"].iloc[0] * 60)
        out["ba_mps2"] = float(last["ba_mps2"].iloc[0])
        out["sg"] = float(last["sg"].iloc[0])

    # outages: simulated (outage_sim) or real (aligned, outage_s > 2)
    sim = df.get("outage_sim", pd.Series(0, index=df.index)).to_numpy(int).astype(bool)
    real = aligned & (df["outage_s"].to_numpy(float) > 2.0) & ~sim
    rows = []
    for kind, mask in (("simulated", sim & aligned), ("real", real)):
        for a, b in segments(mask):
            if t[b] - t[a] < 3.0:
                continue
            dist = float(np.nansum(np.abs(speed[a:b + 1]) * dt[a:b + 1]))
            # error at the end: the last row in the segment with a valid GNSS reference
            seg = df.iloc[a:b + 1]
            seg = seg[np.isfinite(seg["err_H"])]
            if not len(seg):
                continue
            err = float(seg["err_H"].iloc[-1])
            hdg_err = np.nan
            mv = seg[seg["gnss_speed_mps"] > 3.0]
            if len(mv):
                h = mv["heading_cog_deg"].iloc[-1] - mv["course_deg"].iloc[-1]
                hdg_err = (h + 180.0) % 360.0 - 180.0
            rows.append(dict(kind=kind, t_start=t[a] - t[0], duration_s=t[b] - t[a], distance_m=dist,
                             end_err_m=err, err_pct=100 * err / dist if dist > 20 else np.nan,
                             heading_err_deg=hdg_err))
    out["outages"] = pd.DataFrame(rows)
    return out


def plot(df: pd.DataFrame, res: dict, out_base: Path, show: bool) -> list[Path]:
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = df["time_s"].to_numpy(float) - df["time_s"].iloc[0]
    sim = df.get("outage_sim", pd.Series(0, index=df.index)).to_numpy(int).astype(bool)
    outage = sim | (df["outage_s"].to_numpy(float) > 2.0)
    files = []

    def style(ax, title, ylab):
        ax.set_title(title, loc="left", fontsize=10)
        ax.set_ylabel(ylab, color=INK2)
        ax.grid(True, color=GRID)
        for a, b in segments(outage):
            ax.axvspan(t[a], t[b], color="#f3d9cc", lw=0, zorder=0)

    # 1) track
    fig, ax = plt.subplots(figsize=(8, 8))
    g = df[np.isfinite(df["gnss_E"])]
    ax.plot(g["gnss_E"], g["gnss_N"], ".", ms=2, color=INK2, label="GNSS fixes")
    nav = df[df["mode"].isin(["NAV_GNSS", "NAV_DR"])]
    ax.plot(nav["E"], nav["N"], "-", lw=1.5, color=C1, label="DR solution")
    for k, (a, b) in enumerate(segments(outage)):
        seg = df.iloc[a:b + 1]
        ax.plot(seg["E"], seg["N"], "-", lw=2.5, color=C2, label="during GNSS outage" if k == 0 else None)
    ax.set_aspect("equal"); ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
    ax.set_title("Track", loc="left"); ax.grid(True, color=GRID); ax.legend()
    files.append(out_base.with_name(out_base.name + "_track.png")); fig.savefig(files[-1], dpi=120)

    # 2) time series
    fig, axs = plt.subplots(5, 1, figsize=(12, 13), sharex=True)
    axs[0].plot(t, df["err_H"], color=C1, lw=1)
    style(axs[0], "Horizontal DR - GNSS difference (shaded: GNSS outage)", "m")
    axs[1].plot(t, df["gnss_speed_mps"], ".", ms=2, color=INK2, label="GNSS")
    axs[1].plot(t, df["speed_mps"], color=C1, lw=1, label="DR")
    style(axs[1], "Speed", "m/s"); axs[1].legend(loc="upper right", frameon=False)
    mv = df["gnss_speed_mps"] > 3.0
    hdg = df["heading_cog_deg"].to_numpy(float).copy()
    hdg[1:][np.abs(np.diff(hdg)) > 180.0] = np.nan          # break the line at the 0/360 wrap
    axs[2].plot(t[mv], df["course_deg"][mv], ".", ms=2, color=INK2, label="GNSS course")
    axs[2].plot(t, hdg, color=C1, lw=1, label="DR heading")
    style(axs[2], "Heading (compass, deg)", "deg"); axs[2].legend(loc="upper right", frameon=False)
    axs[3].plot(t, df["bg_dps"] * 60, color=C1, lw=1.2, label="yaw-rate bias [deg/min]")
    axs[3].plot(t, df["ba_mps2"] * 10, color=C2, lw=1.2, label="accel bias [x10 m/s²]")
    axs[3].plot(t, df["sg"] * 100, color=C3, lw=1.2, label="yaw scale error [%]")
    style(axs[3], "Estimated sensor errors", "-"); axs[3].legend(loc="upper right", frameon=False)
    modes = ["INIT", "WAIT_GNSS", "ALIGNING", "NAV_DR", "NAV_GNSS"]
    axs[4].plot(t, df["mode"].map({m: i for i, m in enumerate(modes)}), color=C1, lw=1.2, drawstyle="steps-post")
    zupt = df.get("zupt_active", pd.Series(0, index=df.index)).to_numpy(int)
    axs[4].fill_between(t, -0.5, -0.5 + 0.4 * zupt, step="post", color=C3, alpha=0.6, label="stopped (ZUPT/ZARU)")
    axs[4].set_yticks(range(len(modes))); axs[4].set_yticklabels(modes)
    style(axs[4], "Mode", ""); axs[4].legend(loc="upper right", frameon=False); axs[4].set_xlabel("time [s]")
    fig.tight_layout()
    files.append(out_base.with_name(out_base.name + "_timeseries.png")); fig.savefig(files[-1], dpi=110)

    # 3) outage error vs distance
    o = res["outages"]
    if len(o):
        fig, ax = plt.subplots(figsize=(7, 5))
        for kind, col in (("simulated", C1), ("real", C2)):
            s = o[o.kind == kind]
            if len(s):
                ax.plot(s["distance_m"], s["end_err_m"], "o", color=col, label=f"{kind} outages")
        xs = np.linspace(0, max(10.0, o["distance_m"].max()), 50)
        ax.plot(xs, 0.02 * xs, "--", color=INK2, lw=1, label="2 % of distance")
        ax.set_xlabel("distance driven during the outage [m]"); ax.set_ylabel("DR error at the end [m]")
        ax.set_title("DR error after GNSS outages", loc="left"); ax.grid(True, color=GRID); ax.legend()
        files.append(out_base.with_name(out_base.name + "_outages.png")); fig.savefig(files[-1], dpi=120)
    if show:
        plt.show()
    return files


def write_kml(df: pd.DataFrame, path: Path) -> None:
    def coords(lat, lon):
        ok = np.isfinite(lat) & np.isfinite(lon)
        return " ".join(f"{b:.8f},{a:.8f},0" for a, b in zip(lat[ok], lon[ok]))
    nav = df[df["mode"].isin(["NAV_GNSS", "NAV_DR"])]
    g = df[df.get("gnss_reject", 0) == 0]
    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2"><Document><name>{path.stem}</name>
<Style id="gnss"><LineStyle><color>ff4e5152</color><width>2</width></LineStyle></Style>
<Style id="dr"><LineStyle><color>ffd6782a</color><width>3</width></LineStyle></Style>
<Placemark><name>GNSS</name><styleUrl>#gnss</styleUrl><LineString><tessellate>1</tessellate>
<coordinates>{coords(g['lat_deg'].to_numpy(float), g['lon_deg'].to_numpy(float))}</coordinates></LineString></Placemark>
<Placemark><name>Dead reckoning</name><styleUrl>#dr</styleUrl><LineString><tessellate>1</tessellate>
<coordinates>{coords(nav['ekf_lat'].to_numpy(float), nav['ekf_lon'].to_numpy(float))}</coordinates></LineString></Placemark>
</Document></kml>"""
    path.write_text(kml, encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("navlog", type=Path)
    ap.add_argument("--out", type=Path, default=None, help="output prefix (default: next to the CSV)")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    df = pd.read_csv(args.navlog)
    if "time_s" not in df or "mode" not in df:
        print("not a dr_dead_reckoning_app navlog (missing time_s/mode)", file=sys.stderr)
        return 2
    res = analyse(df)
    base = args.out or args.navlog.with_suffix("")
    base.parent.mkdir(parents=True, exist_ok=True)

    print(f"duration {res['duration_s'] / 60:.1f} min, distance {res['distance_m'] / 1000:.2f} km (navigating)")
    print("time in modes [%]:", res["modes_pct"])
    print(f"GNSS positions used {res['gnss_pos_used']}, rejected {res['gnss_rejects']}, re-anchors {res['reanchors']}")
    print(f"stops: {res['stops']} ({res['stopped_s']:.0f} s with ZUPT/ZARU)")
    if "bg_deg_per_min" in res:
        print(f"final estimates: yaw-rate bias {res['bg_deg_per_min']:+.3f} deg/min, "
              f"accel bias {res['ba_mps2']:+.3f} m/s², yaw scale {100 * res['sg']:+.2f} %")
    o = res["outages"]
    if len(o):
        print("\nGNSS outages:")
        print(o.round({"t_start": 0, "duration_s": 1, "distance_m": 0, "end_err_m": 2, "err_pct": 2,
                       "heading_err_deg": 2}).to_string(index=False))
        sim = o[o.kind == "simulated"]
        if len(sim):
            print(f"\nsimulated outages: median error {sim.end_err_m.median():.2f} m, "
                  f"worst {sim.end_err_m.max():.2f} m, median {sim.err_pct.median():.2f} % of distance")
    else:
        print("\nno GNSS outages in this log (use --outage-test on the DR app to measure DR accuracy)")

    files = plot(df, res, base, args.show)
    kml = base.with_name(base.name + "_tracks.kml")
    write_kml(df, kml)
    print("\nwrote:", *[str(f) for f in files + [kml]], sep="\n  ")
    return 0


if __name__ == "__main__":
    sys.exit(main())
