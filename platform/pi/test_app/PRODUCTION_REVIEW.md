# Production Review of the IMU Path

Source reviewed: `dr_dead_reckoning_app_ins15_v2(2).c` (5,960 lines).

## Decision

The current IMU path contains several strong ideas—six-position LSQ accelerometer coefficients, CRC-protected calibration, atomic rename, a boot stillness state machine, median filtering, low-pass filtering, quality gating, and a separate AHRS. It is **not yet production-grade as an integrated path**, because several implementation defects can make clean-looking output misleading or can corrupt a valid calibration.

The standalone validation module in this package preserves the LSQ coefficients and mount convention, but deliberately changes calibration sequencing, timestamp handling, filter construction, validation rules, and persistence.

## Critical findings

### 1. The same IMU sample can be processed more than once

The IMU thread writes `C->imu_raw` and sets `C->have_imu = true`. The fusion thread uses a 5 ms timed condition wait, but `have_imu` is not consumed or paired with a sequence number before processing. A timeout can therefore cause the latest sample to be filtered, calibrated, and integrated again.

Impact:

- apparent fusion rate can exceed the actual sensor ODR;
- boot sample count and calibration duration are wrong;
- median/LPF behavior is distorted;
- gyro integration and EKF prediction can integrate duplicate data;
- repeated values can look artificially quiet.

Required fix: assign every driver sample a monotonically increasing sample sequence or timestamp, copy it once, mark it consumed, release the mutex, and reject duplicate sequence values.

### 2. Runtime gyro calibration crosses coordinate frames incorrectly

`runtime_gyro_cal_update()` receives `gyro_vehicle_rps`, after `mount_R_bv` has been applied, then adds the residual directly to `g_cal.gyro_bias_counts[]`, which is stored in raw sensor-axis counts. With the default `diag(1,-1,-1)` mount, Y and Z corrections have the wrong sign. A general permutation/rotation is even more incorrect.

Impact: runtime learning can drive sensor-frame bias away from zero instead of toward it.

Required fix: estimate/update the bias in the sensor frame before mount rotation, or transform the vehicle-frame residual with `R_bv^T` and correctly undo gyro scale before converting to counts.

### 3. Loaded accelerometer calibration is erased in memory and can be lost on the next save

After loading `g_imu_cal.accel_bias`, the startup path subtracts it from `g_cal.accel_O`. It then calls `imu_calib_sync_from_g_cal()`, which sets every `cal->accel_bias[i]` to zero. A later clean-exit save can therefore persist zero residual bias. The current run looks corrected, but the following run can lose the correction.

Required fix: choose one canonical representation. Do not keep the same correction simultaneously as a modified LSQ offset and as a persisted residual field. The standalone module keeps fixed factory LSQ coefficients and persists only the dynamic gyro bias.

### 4. A known 2× accelerometer scale error is permitted to pass calibration

The current validity range accepts stationary norms from 4 to 16 m/s² specifically to allow half- or double-scale data. The single-pose offset trim can then force the current orientation to have a gravity-like magnitude, while other orientations remain wrong.

Impact: calibration may report success even though acceleration scale is unusable for navigation.

Required fix: scale/range mismatch must be a hard failure. The standalone default accepts 8.8–10.8 m/s² and does not perform a single-pose accelerometer trim.

### 5. A persistent scale fault is cleared as if it were a transient bump condition

A scale warning sets `g_imu_quality_degraded`, but successful boot calibration later clears it. The same global is also cleared when bump duty falls below 5%. Permanent configuration faults and transient road-vibration status need separate flags.

Required fix: use independent, latched fault classes such as `CONFIG_SCALE_ERROR`, `CAL_INVALID`, and `TRANSIENT_BUMP`.

## High-priority findings

### 6. The LPF comment and actual cutoff disagree

At 100 Hz, `y += 0.3(x-y)` has an equivalent continuous-time pole near 5.68 Hz, not about 20 Hz. Fixed alpha also changes the cutoff whenever the real sample interval changes.

Required fix: define cutoff in Hz and calculate `alpha = 1 - exp(-2π f_c dt)` for each valid sample interval. The standalone default is 6 Hz.

### 7. Boot calibration evaluates already-filtered and potentially bump-clamped acceleration

The fusion loop preprocesses, median-filters, low-pass-filters, and conditionally clamps acceleration before passing `acc_b` to `apply_poweron_calibration()`. That hides raw noise and can make the recorded calibration variance artificially good.

Required fix: boot acceptance statistics must use unfiltered LSQ output. Filtering is for navigation and display, not for proving sensor quality.

### 8. Single-pose accelerometer correction is not identifiable

A stationary sample provides one gravity vector. It cannot independently solve three accelerometer offsets, three scale factors, and cross-axis errors. Projecting the measured vector to `g` only corrects that pose and can mask a scale problem.

Required fix: treat the six-position LSQ calibration as the factory accelerometer calibration. At boot, validate acceleration norm and variance; do not alter the LSQ offset unless the orientation is controlled and the correction model is explicitly limited.

### 9. Calibration validation does not scan all floating-point quality fields

The NaN/Inf scan stops at `temperature_c`; later boot quality metrics are outside the scan. Comparisons against NaN can evaluate false and allow invalid values to pass.

Required fix: validate each field explicitly or serialize/validate a fixed schema.

### 10. The mount matrix is not verified as a rotation

Validation only checks that values are finite and have magnitude below 2. It does not check unit row/column norms, orthogonality, or determinant +1.

Required fix: enforce `R R^T ≈ I` and `det(R) ≈ +1`. The standalone module does this.

### 11. Gyro-bias limits contradict the stated sensor behavior

Comments say a unit may have up to ±15 deg/s zero-rate offset, while `CAL_SANITY_GYRO_BIAS_MAX` rejects anything above 5 deg/s.

Required fix: choose limits from the exact sensor datasheet, configured full-scale, temperature range, and observed population. The standalone provisional limit is 15 deg/s per axis, accompanied by variance and repeatability checks.

### 12. The fusion mutex is held during heavy processing

The fusion thread keeps the context mutex while calibration, filters, AHRS, EKF, GNSS handling, and logging run. That can block the producer threads and cause sample loss or timing jitter.

Required fix: lock only long enough to copy the latest sample and associated metadata, then unlock before all numeric work and I/O.

### 13. Runtime calibration treats missing GNSS as “GNSS speed low”

`(!C->gnss_have_fix) || speed < threshold` allows runtime bias learning when GNSS is absent. That is weaker than the comment stating both GNSS and EKF confirm stationary.

Required fix: require an independent stationary source—wheel speed, parking brake, transmission state, or fresh GNSS speed—before changing a persisted runtime bias.

### 14. No calibration/configuration fingerprint exists in the original blob

A calibration can be loaded after changing full-scale, LSQ constants, ODR, or mount matrix.

Required fix: store and validate a configuration fingerprint. The standalone blob includes a CRC of its calibration-relevant configuration.

## Medium-priority findings

- `BUMP_GYRO_THR_RS`, `BOOT_CAL_N_SAMPLES`, and `CAL_LED_BLINK_HZ` are defined but not used as their names imply.
- Median state and LPF state are static locals, making the pipeline non-reentrant and difficult to reset or unit-test.
- `medf5()` is fixed at five elements while `MEDFILT_LEN` looks configurable.
- `imu_calib_sync_from_g_cal()` is not idempotent because it multiplies `gyro_scale[2]`; repeated calls can compound scale.
- Atomic calibration save does not handle partial writes and does not `fsync()` the parent directory after rename.
- Calibration temperature is hardcoded to 25 °C and there is no temperature compensation model.
- The LSQ matrix comment says it includes a PCB permutation while a separate mount rotation is also applied. A six-face axis/sign test is required to prove that no permutation is being applied twice.
- Bump clamping is appropriate only for a navigation-specific path. A sensor qualification tool should show and log the unclamped signal so bad mounting cannot be hidden.

## Production acceptance work still required

1. Verify the IMU driver ODR and full-scale registers at runtime and include them in telemetry.
2. Perform a six-face accelerometer test and ±axis rotation test to confirm matrix and mount signs.
3. Capture at least 30–60 minutes of stationary raw data over the operating temperature range.
4. Calculate PSD and Allan deviation, then derive process noise from measured units rather than comments or nominal datasheet values.
5. Establish population-based pass/fail limits across multiple boards.
6. Run replay tests using recorded raw data and deterministic expected outputs.
7. Add fault injection: duplicate sample, dropped sample, time jump, saturation, CRC corruption, wrong full-scale, and wrong mount matrix.
8. Only after these pass should the same module replace the main application’s local IMU block.
