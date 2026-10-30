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

- reads accel + gyro through the **hardware FIFO**, 4 words per I²C transfer
  (`fifo_burst_words=4`), with sample times reconstructed on the chip clock (trimmed by
  `INTERNAL_FREQ_FINE`, spacing never compressed); default `fifo_stream=1`;
- enables the on-chip **gyro LPF1** (`gyro_lpf1_bw=6` ≈ 27 Hz @ 240 Hz) and **accel LPF2**
  (`accel_lpf2_bw=2` = ODR/20 ≈ 12 Hz) to stop vibration aliasing;
- runs the **SFLP** block (`sflp_odr=120`) and by default batches only its **gyro-bias** estimate
  (`sflp_fifo_mask=4`), which the host AHRS uses live. The SFLP quaternion is not a valid attitude
  on this part (see below); `sflp_fifo_mask=7` / `--sflp-mask 7` brings it back for diagnostics;
- keeps `struct mpu6050_sample` as the default `read()` format (DR app unchanged);
  `MPU6050_IOC_SET_READ_FMT(MPU6050_FMT_EXT)` gives `struct mpu6050_ext_sample`.

Runtime knobs: `/sys/class/mpu6050/mpu6050-0/{gyro_lpf1_bw,accel_lpf2_bw,sflp_enable,sflp_odr_hz,
sflp_fifo_mask,fifo_stream,sflp_reset,stream_stats}`.

### Step by step

1. **I²C at 400 kHz** (once). In `/boot/config.txt` (or `/boot/firmware/config.txt`):
   `dtparam=i2c_arm_baudrate=400000`, then reboot. Check it:
   `xxd /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency` must show `0006 1a80` (400000).
   At 100 kHz the FIFO cannot be emptied fast enough and the dashboard shows FIFO OVERRUN.
2. **Load the driver** (rebuild it after pulling these changes):
   `sudo insmod mpu6050_char.ko` and confirm in `dmesg` that the line `signal chain: ... read path=HW FIFO`
   is printed.
3. **Start the dashboard on the PC**: `python imu_rotation_dashboard.py --csv bench.csv`
   (allow inbound UDP 5005 through the firewall).
4. **Start the streamer on the Pi**: `./imu_streamer --host <PC IP> --gyro-fs 500`.
   Leave the IMU still for ~22 s on first start (boot gyro-bias calibration).
5. **Mount calibration wizard** (starts on the dashboard as soon as data arrives; `m` reruns it).
   Follow the yellow prompt in the lower-left panel:

   | Step | Bench (board in hand / on a table) | Vehicle | Key |
   |---|---|---|---|
   | 1 LEVEL | lay it flat, forward end away from you, hands off | park on level ground, engine off | SPACE |
   | 2 TURN 180 | rotate it half a turn on the same spot | turn around, park on the same marks | SPACE (S = skip) |
   | 3 FORWARD | without turning it, tip the front end down 20–40° and hold | press A, then accelerate firmly straight ahead for ~3 s | SPACE / A |
   | review | check roll/pitch/yaw of the correction and the surface slope | same | ENTER = apply, ESC = discard |

   Each still capture needs 3 s without movement; if you move, it restarts and says so.
   ENTER sends the result to the streamer, which applies it immediately, saves it to
   `imu_mount.txt` next to the binary, and keeps the gyro-bias calibration valid. Yaw is zeroed
   automatically afterwards. Skipping step 2 leaves the surface slope inside roll/pitch (the
   review says so). The wizard also fixes a wrong `--mount` preset (e.g. board upside down).
6. **Check** (keys: `z` zero yaw, `t` tare, `c` clear, `r` record): nose down → pitch +, right side
   down → roll +, left turn → yaw +; rate ≈ 240 Hz, "driver dropped" not increasing, no FIFO OVERRUN;
   at rest the level-frame accel stays within about ±0.03 m/s².
7. **Review offline**: `python imu_rotation_dashboard.py --replay bench.csv --speed 2`.

The mount is now persistent: the next `./imu_streamer` run loads `imu_mount.txt` automatically
(command-line `--mount` / `--mount-rpy` override it; `--mount-file` changes the path). Rerun the
wizard whenever the board is moved or remounted. `--no-wizard` skips it on the dashboard.

### Reading the two cubes

- **Host Mahony AHRS (primary)** — fuses the gyro and accelerometer on the Pi; its gyro bias comes
  live from the SFLP estimate (status line "AHRS bias: SFLP (live)"), so it follows temperature.
  This is the attitude used for gravity removal.
- **On-chip SFLP** — with the default mask the cube reads "quaternion not batched". With
  `--sflp-mask 7` it shows the SFLP attitude for diagnosis; expect "INCONSISTENT ~90°".
  The 2026-10-27/30 recordings show why: the SFLP integrates the gyro correctly, but takes its gravity
  reference from the accelerometer with X and Z exchanged (the chip delivers accel as Z,Y,X). Its
  gravity output is derived from the same quaternion. DS14588 has no register to remap this, so treat
  the SFLP attitude as unusable on this part (worth raising with ST). Its gyro-bias output is correct:
  within ~0.02 dps of the true bias through a 4 °C warm-up, where the boot calibration drifted 0.09 dps.

### What the 2026-10-27 recordings showed (and what changed)

| Observation | Cause | Fix |
|---|---|---|
| ~140 of 240 samples/s, `driver dropped` 95k, FIFO OVERRUN every ~150 ms | one I²C transfer per FIFO word plus a status poll per half period | 4-word bursts, fewer status polls; check 400 kHz I²C |
| dt down to 1 ns, sample_rate_hz ≈ 10⁶, cube lag/jumps | time reconstruction collapsed while the FIFO was backlogged | spacing clamped ≥ 15/16 T, re-sync only on real gaps |
| SFLP gravity 90° from the accelerometer | gravity word is Z,Y,X ordered (+g up, verified 2026-10-30) | decoded accordingly |
| SFLP cube wrong when tilted, "INCONSISTENT 89–107°" (2026-10-30) | SFLP fuses the gyro with an X↔Z-swapped accelerometer | quaternion no longer batched by default; consistency now checked against the accelerometer |
| AHRS yaw drift ~12° in 213 s at rest (2026-10-30) | boot gyro bias goes stale as the board warms | AHRS uses the SFLP gyro bias live (est. ~3°) |
| ~1 % duplicate rows (same seq) in the CSV | UDP datagrams duplicated on the network (Wi-Fi) | dashboard drops duplicates ("dup" counter) |
| drops/overruns return with `--sflp-odr 240` | doubles SFLP FIFO traffic | keep 120; default mask 4 |
| board orientation uncertain | no install calibration | guided mount wizard + persistent `imu_mount.txt` |

**Frame / FLU check.** After 2 s at rest the streamer prints `[FLU check]`: gravity must be on +z
in the vehicle frame (true after the wizard). It also warns if |g| is off by more than 1 %.

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
