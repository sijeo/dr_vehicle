# Standalone IMU Calibration and Filter Validation

This package isolates the IMU path from the dead-reckoning application so calibration and filtering can be tested without GNSS, AHRS, EKF, ZUPT, or position feedback hiding sensor defects.

It uses the exact ±8 g LSQ accelerometer matrix and offset from the uploaded application and the same default FRD-to-FLU mount rotation. The implementation then corrects the sequencing, validation, filter, frame, and persistence problems documented in `docs/PRODUCTION_REVIEW.md`.

## Contents

- `include/imu_core.h`, `src/imu_core.c` — reusable calibration, preprocessing, median, LPF, rolling statistics, calibration persistence, and quality flags.
- `src/imu_streamer.c` — Raspberry Pi reader and UDP sender. Each successful driver read is processed exactly once.
- `include/imu_wire.h`, `src/imu_wire.c` — fixed-size versioned UDP frame with sequence number, monotonic timestamp, explicit network byte order, and CRC32.
- `dashboard/imu_dashboard.py` — live plots, calibration progress, ODR, packet loss, quality flags, filtered signals, rolling noise, and CSV recording.
- `dashboard/analyze_imu_log.py` — offline time-series, PSD/amplitude spectral density, Allan deviation, and JSON statistics.
- `src/imu_simulator.c` — desktop test source that exercises calibration and sends valid network frames.
- `tests/test_imu_core.c` — deterministic synthetic calibration and protocol test.
- `docs/ORIGINAL_IMU_AUDIT_EXCERPT.txt` — exact IMU-related excerpts with original line numbers.

## Build on the Raspberry Pi

The existing project’s `mpu6050_ioctl.h` is required because it defines the actual driver ABI.

```bash
cd imu_validation_tool
make DRIVER_INCLUDE=/home/sijeo/dr_vehicle
```

Change `DRIVER_INCLUDE` to the directory containing `mpu6050_ioctl.h`.

## Start the PC dashboard

```bash
python3 -m venv .venv
. .venv/bin/activate            # Windows: .venv\\Scripts\\activate
pip install matplotlib numpy
python dashboard/imu_dashboard.py --bind 0.0.0.0 --port 5005 --csv stationary_test.csv
```

Allow inbound UDP port 5005 through the PC firewall.

## Start the Raspberry Pi streamer

Replace the address with the PC’s LAN address.

```bash
sudo ./imu_streamer \
  --host 192.168.1.50 \
  --port 5005 \
  --device /dev/mpu6050-0 \
  --cal-file /home/sijeo/dr_vehicle/imu_validation_cal.bin \
  --recalibrate
```

Keep the complete IMU assembly stationary until the dashboard shows `VALID` and calibration reaches 100%. The default sequence is 2 seconds of settling followed by 20 seconds of accepted stationary samples.

After a valid calibration has been saved, omit `--recalibrate` to load it. The file is rejected when the calibration-relevant configuration CRC changes.

## Test without the Raspberry Pi

Terminal 1:

```bash
python dashboard/imu_dashboard.py --port 5005 --csv simulated.csv
```

Terminal 2:

```bash
./imu_simulator --host 127.0.0.1 --port 5005 --fast-cal
```

The simulator calibrates, produces smooth acceleration and yaw-rate motion, and periodically injects a vertical bump.

## Offline analysis

Record a stationary CSV for at least 30–60 minutes, then run:

```bash
python dashboard/analyze_imu_log.py stationary_test.csv --out imu_analysis --skip 10
```

Outputs:

- `summary.json`
- `timeseries.png`
- `spectrum.png`
- `allan_deviation.png`

## What the live dashboard should prove

- **Rate:** observed ODR is stable and close to the configured 100 Hz.
- **Uniqueness:** packet sequence advances by one; loss and repeated processing are visible.
- **Scale:** stationary acceleration norm is close to 9.80665 m/s² before any navigation correction.
- **Signs and axes:** a forward tilt/motion appears on the expected axis; left/right and up/down signs match FLU.
- **Bias:** after calibration, stationary gyro means remain close to zero.
- **Noise:** rolling standard deviation is stable and the LPF reduces high-frequency noise without hiding clipping or scale faults.
- **Integrity:** saturation, time gaps, CRC errors, calibration-file mismatch, and failed persistence are explicit flags.

## Provisional bench gates

These are starting gates, not final production specifications. Confirm them across multiple boards and temperatures.

| Metric | Provisional gate |
|---|---:|
| Observed ODR | 98–102 Hz for a configured 100 Hz test |
| UDP loss on a quiet wired/LAN test | < 0.1% |
| Stationary acceleration norm mean | 9.80665 ± 0.15 m/s² |
| Stationary filtered gyro residual mean | < 0.05 deg/s per axis |
| Stationary unfiltered gyro standard deviation | < 0.10 deg/s per axis |
| Stationary unfiltered accel standard deviation | < 0.05 m/s² per axis on a quiet bench |
| Raw saturation | zero samples |
| Bump duty during bench calibration | 0% |
| Calibration/configuration hard-fault flags | none |

A vehicle with the engine running needs a separate vibration acceptance profile. Do not relax a bench scale test merely to make an installed vehicle pass.

## Important limitation

An IMU alone cannot distinguish a perfectly constant angular rotation from a constant gyro bias. Production boot calibration therefore needs either a controlled stationary procedure or independent stationary evidence such as wheel speed, parking brake, transmission state, or fresh GNSS speed. Runtime bias learning is intentionally not enabled in this isolated tool.
