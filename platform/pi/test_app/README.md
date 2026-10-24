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

## ISM330BX on-chip fusion (SFLP) + rotation dashboard

The IMU is an ST **ISM330BX** (WHO_AM_I 0x71) mounted **FLU**. `mpu6050_char.ko` now:

- reads accel + gyro through the **hardware FIFO** (no lost samples; times reconstructed on the
  chip clock, trimmed by `INTERNAL_FREQ_FINE`), default `fifo_stream=1`;
- enables the on-chip **gyro LPF1** (`gyro_lpf1_bw=6` ≈ 27 Hz @ 240 Hz) and **accel LPF2**
  (`accel_lpf2_bw=2` = ODR/20 ≈ 12 Hz) to stop vibration aliasing;
- runs the **SFLP** game rotation vector, gravity vector and gyro-bias estimate (`sflp_odr=120`)
  and attaches the latest values to every sample;
- keeps `struct mpu6050_sample` as the default `read()` format (DR app unchanged);
  `MPU6050_IOC_SET_READ_FMT(MPU6050_FMT_EXT)` gives `struct mpu6050_ext_sample`.

Runtime knobs: `/sys/class/mpu6050/mpu6050-0/{gyro_lpf1_bw,accel_lpf2_bw,sflp_enable,sflp_odr_hz,
sflp_fifo_mask,fifo_stream,sflp_reset,stream_stats}`. Needs `dtparam=i2c_arm_baudrate=400000`.

```bash
# Pi
sudo insmod mpu6050_char.ko            # optional: gyro_lpf1_bw=7 sflp_odr=240 ...
./imu_streamer --host <PC IP> --gyro-fs 500     # v2 frames (SFLP + host Mahony + gravity-free accel)
# PC
python imu_rotation_dashboard.py --csv drive1.csv   # r: record, z: zero yaw, t: tare, c: clear, q: quit
python imu_rotation_dashboard.py --replay drive1.csv --speed 2
```

**Frame / FLU check.** The streamer maps sensor to vehicle FLU with `--mount flu` (default, identity) or `--mount frd`, plus `--mount-rpy R,P,Y` trim from the field calibration (`../IMU_FIELD_CALIBRATION.md`). After 2 s at rest it prints `[FLU check]`: gravity must be on +z; the residual roll/pitch it reports is what `--mount-rpy` takes. It also warns if |g| is off by more than 1 %.

`--wire v1` keeps the old 236-byte frame for `imu_dashboard.py`. `--gyro-fs` changes the
driver-wide scale: update the DR app's gyro LSB/dps if you keep it.

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
