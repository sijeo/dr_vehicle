#!/usr/bin/env python3
"""Offline stationary-noise, filter, spectrum, and Allan-deviation analysis."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

def allan_deviation(x: np.ndarray, fs:float) -> tuple[np.ndarray, np.ndarray]:
    n = len(x)
    if n < 32:
        return np.array([]), np.array([])
    max_m = max(2, n // 10)
    ms = np.unique(np.logspace(0, np.log10(max_m), 40).astype(int))
    taus = []
    adevs = []
    for m in ms:
        k = n // m
        if k < 3:
            continue
        y = x[: k * m].reshape(k, m).mean(axis=1)
        diff = np.diff(y)
        avar = 0.5 * np.mean(diff * diff)
        if avar > 0:
            taus.append(m / fs)
            adevs.append(np.sqrt(avar))
    return np.asarray(taus), np.asarray(adevs)

def one_sided_psd(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    x = x - np.mean(x)
    n = len(x)
    window = np.hanning(n)
    denom = fs * np.sum(window * window)
    spectrum = np.fft.rfft(x * window)
    psd = (np.abs(spectrum) ** 2) / denom
    if n > 2:
        psd[1:-1] *= 2.0
    return np.fft.rfftfreq(n, 1.0 / fs), psd

def stats(values: np.ndarray) -> dict[str, float]:
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "rms": float(np.sqrt(np.mean(values * values))),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
        "p95_abs": float(np.percentile(np.abs(values), 95)),
        }

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--out", type=Path, default=Path("imu_analysis"))
    parser.add_argument("--skip", type=float, default=5.0, help="seconds to skip at start")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    data = np.genfromtxt(args.csv, delimiter=",", names=True, dtype=None, encoding="utf-8")
    if data.size < 100:
        raise SystemExit("Need at least 100 Samples")

    t_ns = np.asarray(data["imu_time_ns"], dtype=np.float64)
    t = (t_ns - t_ns[0]) * 1e-9
    keep = t >= args.skip
    t = t[keep]
    dt = np.diff(t)
    fs = 1.0 / np.median(dt)

    acc_u = np.column_stack([data[f"unfiltered_a{axis}_mps2"] for axis in "xyz"])[keep]
    acc_f = np.column_stack([data[f"filtered_a{axis}_mps2"] for axis in "xyz"])[keep]
    gyro_u = np.column_stack([data[f"unfiltered_g{axis}_degps"] for axis in "xyz"])[keep]
    gyro_f = np.column_stack([data[f"filtered_g{axis}_degps"] for axis in "xyz"])[keep]
    acc_norm = np.asarray(data["filtered_acc_norm_mps2"], dtype=float)[keep]

    summary: dict[str, object] = {
        "samples": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "sample_rate_hz_median": float(fs),
        "sample_rate_hz_p05": float(1.0 / np.percentile(dt, 95)),
        "sample_rate_hz_p95": float(1.0 / np.percentile(dt, 5)),
        "sample_gaps_over_2x_nominal": int(np.sum(dt > 2.0 / fs)),
        "acc_norm": stats(acc_norm),
        "acc_norm_error_mean_mps2": float(np.mean(acc_norm) - 9.80665),
        "axes": {},
    }
    for i, axis in enumerate("xyz"):
        summary["axes"][f"acc_unfiltered_{axis}"] = stats(acc_u[:, i])
        summary["axes"][f"acc_filtered_{axis}"] = stats(acc_f[:, i])
        summary["axes"][f"gyro_unfiltered_{axis}_degps"] = stats(gyro_u[:, i])
        summary["axes"][f"gyro_filtered_{axis}_degps"] = stats(gyro_f[:, i])

    with (args.out / "summary.json").open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for i, axis in enumerate("XYZ"):
        axes[0].plot(t, acc_u[:, i], alpha=0.45, label=f"{axis} unfiltered")
        axes[0].plot(t, acc_f[:, i], label=f"{axis} filtered")
        axes[1].plot(t, gyro_u[:, i], alpha=0.45, label=f"{axis} unfiltered")
        axes[1].plot(t, gyro_f[:, i], label=f"{axis} filtered")
    axes[0].set_ylabel("Acceleration (m/s^2)")
    axes[1].set_ylabel("Gyro (deg/s)")
    axes[1].set_xlabel("Time (s)")

    for ax in axes:
        ax.grid(True)
        ax.legend(ncol=3)
    fig.tight_layout()
    fig.savefig(args.out / "timeseries.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    for i, axis in enumerate("XYZ"):
        f, p = one_sided_psd(acc_u[:, i], fs)
        axes[0].loglog(f[1:], np.sqrt(p[1:]), label=axis)
        f, p = one_sided_psd(gyro_u[:, i], fs)
        axes[1].loglog(f[1:], np.sqrt(p[1:]), label=axis)
    axes[0].set_ylabel("Accel ASD (mps2psqrtHz)")
    axes[1].set_ylabel("Gyro ASD (deg/s/sqrtHz)")
    axes[1].set_xlabel("Frequency (Hz)")
    for ax in axes:
        ax.grid(True, which="both")
        ax.legend()
    fig.tight_layout()
    fig.savefig(args.out / "sprectrum.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))
    allan_summary = {}
    for i, axis in enumerate("XYZ"):
        tau, adev = allan_deviation(acc_u[:, i], fs)
        if len(tau):
            axes[0].loglog(tau, adev, marker=".", label=axis)
            j = int(np.argmin(adev))
            allan_summary[f"acc_{axis.lower()}"] = {"min_adev": float(adev[j]), "tau_s": float(tau[j])}
        tau, adev = allan_deviation(gyro_u[:, i], fs)
        if len(tau):
            axes[1].loglog(tau, adev, marker=".", label=axis)
            j = int(np.argmin(adev))
            allan_summary[f"gyro_{axis.lower()}_degps"] = {"min_adev": float(adev[j]), "tau_s": float(tau[j])}
        axes[0].set_ylabel("Accel Allan Deviation (mps2)")
        axes[1].set_ylabel("Gyro Allan Deviation (degps)")
        axes[1].set_xlabel("Averate Time t (s)")

        for ax in axes:
            ax.grid(True, which="both")
            ax.legend()
        fig.tight_layout()
        fig.savefig(args.out / "allan_deviation.png", dpi=160)
        plt.close(fig)

        summary["allan_minima"] = allan_summary
        with( args.out / "summary.json").open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2)

        print(json.dumps(summary, indent=2))
        print(f"Wrote Analysis to {args.out}")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())

    
        
