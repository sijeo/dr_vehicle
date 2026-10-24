# ISM330BX field calibration (no six-position calibration)

How to calibrate the IMU after it is mounted in the vehicle, without the six-position
calibration. Each parameter the dead-reckoning (DR) solution needs comes from something the
vehicle can do: stand still, turn around, drive straight, drive in circles, and keep driving
with GNSS.

Applies to: the ISM330BX mounted FLU (x forward, y left, z up), driver `mpu6050_char.ko`,
`test_app/imu_streamer` + `imu_rotation_dashboard.py` for capture, and
`dr_dead_reckoning_app_ins15_v3.c` as the consumer.

---

## 1. What actually needs calibrating

The six-position calibration fits a full 3x3 scale/cross-axis matrix plus offsets. For a
vehicle DR system most of that matrix contributes little. The table shows which errors matter
and how each one can be observed in the field.

| Error | ISM330BX spec | Effect on DR if left uncorrected | Observable in the field by |
|---|---|---|---|
| **Mount roll / pitch** (sensor vs vehicle) | install tolerance, typically 1–3° | 2° pitch puts 0.34 m/s² of gravity into forward accel, about **17 m of error after 10 s** | static two-heading test (§3) |
| **Mount yaw** | install tolerance | forward accel leaks into lateral; centripetal accel leaks into forward | straight-line accel/brake drive (§4) |
| **Gyro bias** | ±2 dps zero-rate, ±0.03 dps/°C | 0.1 dps gives 6° of heading drift per minute | every stop (§3, §6) |
| **Accel bias** | ±20 mg zero-g, ±0.7 mg/°C | 10 mg integrates to 5 m in 10 s | lumped into leveling at stops; residual estimated by the EKF with GNSS (§6) |
| **Gyro z scale** | ±1 % | 0.9° heading error per 90° turn | circle test or GNSS course over turns (§5) |
| **Accel scale** | ±1 % | 1 % of a 3 m/s² braking event is 0.03 m/s² (small); a uniform scale error does not matter once gravity uses the measured g reference | static \|a\| vs local g (§3) |
| Cross-axis / non-orthogonality | ±0.5 % accel, ±1.2 % gyro | second order for a road vehicle | not calibrated; absorbed by the EKF |

**Conclusion:** the important parameters are the **mount rotation** (`mount_R_bv`), the **gyro
bias** (with its temperature dependence) and the **gyro z scale**. The accel matrix can stay at
the ST factory sensitivity (0.244 mg/LSB at ±8 g), with biases handled at stops and in the
EKF.

> **The 5.6 % low |a| in `imu_test_log.csv` (9.25 vs about 9.78 m/s²) is outside the ±1 %
> sensitivity spec. Field calibration should not be used to hide that.** Do the
> pre-installation bench check (§2) on that unit first. If it is confirmed, treat the board as
> suspect: check soldering stress and board flex, or replace the part.

---

## 2. Pre-installation bench check (once per board, 2 minutes)

This is optional but strongly recommended. It is not a six-position calibration. It only
confirms the part is inside its datasheet limits, so the field procedure can trust the
factory sensitivity.

1. Lay the board flat, z up, and stream for 20 s. Record the mean raw `az_up`.
2. Flip it, z down, and stream for 20 s. Record the mean raw `az_down`.
3. z sensitivity = `(az_up - az_down) / 2` LSB per g. Expect **4098 LSB/g ±1 %** at ±8 g
   (0.244 mg/LSB). z offset = `(az_up + az_down) / 2` LSB. Expect within ±82 LSB (±20 mg).
4. If either value is out of range, run the ST self-test (CTRL10 `ST_XL`/`ST_G`, limits
   20–1700 mg) to separate a broken part from a mechanical-stress problem. Do not install a
   failing unit.

If you want this repeated per board, store the z scale and offset as a factory record in the
calibration file (§7).

---

## 3. Install calibration step 1: static two-heading test (≈ 3 min, vehicle parked)

This gives the **mount roll and pitch**, the **gyro bias** at the current temperature, and an
**|a| check**.

Ground slope and mount tilt look the same from a single parked position. Parking the same
vehicle at the same spot turned **180°** flips the sign of the ground slope in the vehicle
frame but not the mount tilt, so the two separate.

Procedure (engine **off**: idle vibration raises the noise, and the engine-on case is handled
in §6):

1. Park on firm, roughly level ground (slope under about 3°). Mark the wheel positions.
2. Wait 60 s for the sensor temperature to settle after power-up. Then capture **30 s** at
   rest: heading A.
3. Drive out and park on the same marks facing the opposite way. Capture **30 s**: heading
   A+180°.
4. Reject a capture if accel std exceeds 0.02 m/s² per axis or gyro std exceeds 0.05 dps
   (someone moved the vehicle or a door slammed). Repeat that capture.

Computation, where `f1` and `f2` are the mean sensor-frame accel vectors with the nominal FLU
mount already applied:

```
roll_i  = atan2(f_i.y, f_i.z)
pitch_i = atan2(-f_i.x, sqrt(f_i.y^2 + f_i.z^2))           (REP-103: + = nose down)

mount_roll   = (roll_1  + roll_2 ) / 2                     ground slope cancels
mount_pitch  = (pitch_1 + pitch_2) / 2
ground_roll  = (roll_1  - roll_2 ) / 2                     log only (sanity check < 3°)
ground_pitch = (pitch_1 - pitch_2) / 2

gyro_bias    = mean of the gyro over both captures (raw counts), with temp_c
g_meas       = (|f1| + |f2|) / 2
```

Checks:

- `|g_meas - g_local| / g_local < 1 %`, where g_local is local gravity:
  `9.780327·(1 + 0.0053024·sin²φ − 0.0000058·sin²2φ) − 3.086e-6·h`
  (φ = latitude, h = altitude in m; about 9.782–9.791 m/s² across India). If this check
  fails, go back to §2.
- Gyro bias per axis within ±2 dps (the spec). Each capture's bias should repeat to within
  0.02 dps.

Note: x/y accel bias and mount tilt cannot be separated with the vehicle on the ground. For
DR this does not matter, because the combination is exactly what levels the accelerometer.
The EKF trims the remainder (§6).

---

## 4. Install calibration step 2: mount yaw from a straight drive (≈ 5 min)

Drive a straight, flat road (or an empty lot) with **5–10 firm accelerate/brake cycles**
(0→30→0 km/h). Keep the steering straight.

Use samples where |yaw rate| < 1 dps and the horizontal accel exceeds 1 m/s². Take `a_x` and
`a_y` as level-frame accel with gravity removed, after applying §3's roll/pitch. That is the
streamer's `lin_lvl_fwd/left` columns once the new mount is loaded.

```
k          = Σ(a_x·a_y) / Σ(a_x²)          slope of lateral vs forward accel
mount_yaw  = -atan(k)                       sensor x rotated by mount_yaw from vehicle x
```

Expect the result to repeat to within 0.3° across runs.

Cross-check in a turn: centripetal accel `v·ω_z` (v from GNSS) must appear on +y for a left
turn, with no x component.

Final mount rotation, sensor to vehicle, ZYX:

```
mount_R_bv = Rz(mount_yaw) · Ry(mount_pitch) · Rx(mount_roll) · R_nominal
R_nominal  = I for the FLU installation
```

If you have GNSS Doppler speed, also fit `a_x_imu = s·(dv_gnss/dt) + b` over the same
segments. Low-pass `dv/dt` to the IMU bandwidth and align the timestamps first. The slope `s`
should be within 1 %, and `b` is the forward accel bias. Store `b` only as an initial value
for the EKF.

---

## 5. Install calibration step 3: gyro z scale, circle test (≈ 5 min)

In an empty lot, align the vehicle with a painted line and capture 5 s stationary (bias
refresh). Drive **3 full circles counter-clockwise**, stop aligned with the same line, capture
5 s, then drive **3 circles clockwise** and stop aligned again.

```
Δψ_ccw = ∫(ω_z - b_z) dt   over the CCW leg      true value +3·360° = +1080°
Δψ_cw  = ∫(ω_z - b_z) dt   over the CW leg       true value -1080°
gyro_z_scale = (Δψ_ccw - Δψ_cw) / 2160°          any residual bias cancels between directions
```

Alignment to the painted line within ±2° gives about 0.1 % scale accuracy. The expected value
is 1.000 ± 0.01; reject anything outside ±2 %.

GNSS alternative while driving normally: over turns with speed > 5 m/s, compare integrated
gyro heading change with the GNSS course change. Average left and right turns separately and
use more than 10 turns.

x/y gyro scale is not needed for planar DR. Leave it at 1.0.

---

## 6. Continuous (online) calibration while in service

The parameters from §3–5 are initial values. The following keep them correct as temperature,
ageing and vehicle loading change.

1. **Gyro bias at every stop (ZUPT / zero-rate update).**
   - Stationary when all of these hold for ≥ 2 s: GNSS speed < 0.1 m/s, accel std
     < 0.05 m/s², and gyro std < 0.1 dps (engine on) or < 0.03 dps (engine off).
   - Update `b_g ← b_g + α·(mean ω − b_g)` with α = 0.2 per stop.
   - Also feed it to the EKF as a zero-angular-rate measurement.
   - The DR app's `ZUPT_GYRO_THR = 5 dps` is too loose to *estimate* bias. Use the tighter
     std thresholds above for the bias update and keep the loose one for velocity ZUPT only.
2. **Temperature model.** Log `(temp_c, b_g)` at every stop; the stream now carries `temp_c`.
   Once the logged temperatures span ≥ 10 °C, fit `b_g(T) = b0 + k·(T − T0)` per axis and
   apply it between stops. The spec allows up to 0.03 dps/°C, which is 1.5 dps over a 50 °C
   cabin swing.
3. **EKF states.** Keep accel bias `ba` and gyro bias `bg` as random-walk states (the DR app
   already does). GNSS velocity updates make the horizontal accel bias observable during
   acceleration and turns. Optionally add gyro z scale as a state.
4. **Non-holonomic constraint (NHC).** A road vehicle has ≈ 0 lateral and vertical velocity
   in its own frame. Add `v_y ≈ 0, v_z ≈ 0` pseudo-measurements (σ ≈ 0.1–0.3 m/s) whenever
   speed > 2 m/s. This keeps mount yaw/pitch errors and accel bias observable in service and
   is the single biggest DR improvement after leveling.
5. **SFLP cross-check.** The SFLP gyro-bias output (`gbias_sflp_*` in the CSV) is an
   independent estimate. Flag a fault if it differs from `b_g` by more than 0.2 dps for more
   than 60 s.
6. **Re-leveling at long stops.** At each stop of ≥ 10 s, compare the measured gravity
   direction with the expected one. A persistent change > 0.5° means the mount has moved
   (bracket loosened or after a repair). Raise a "recalibrate mount" flag rather than
   silently absorbing it.

---

## 7. Where the results go

Save the results as `ism330_vehicle_calibration.json` next to the DR binary on the Pi
(`/home/sijeo/dr_vehicle/`), versioned per vehicle:

```json
{
  "version": 1,
  "device": "ISM330BX",
  "vehicle_id": "…",
  "created_utc": "2026-10-25T10:00:00Z",
  "fs": { "accel_g": 8, "gyro_dps": 500 },
  "accel_C": [[0.0023929, 0, 0], [0, 0.0023929, 0], [0, 0, 0.0023929]],
  "accel_O": [0.0, 0.0, 0.0],
  "accel_z_bench": { "lsb_per_g": 4098.0, "offset_lsb": 0.0 },
  "mount": { "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
             "R_bv": [[1,0,0],[0,1,0],[0,0,1]] },
  "gyro_bias_counts": [0.0, 0.0, 0.0],
  "gyro_bias_temp_c": 30.0,
  "gyro_bias_tempco_counts_per_c": [0.0, 0.0, 0.0],
  "gyro_scale": [1.0, 1.0, 1.0],
  "g_local_mps2": 9.785,
  "checks": { "g_meas_mps2": 0.0, "ground_slope_deg": [0.0, 0.0],
              "yaw_fit_repeat_deg": 0.0, "circle_scale_ccw_cw": [0.0, 0.0] }
}
```

`accel_C` is the **nominal** sensitivity `0.244e-3 · 9.80665` m/s² per LSB at ±8 g (scale
linearly for other ranges), so no six-position fit is needed. If the §2 bench check was done,
put its z sensitivity in `accel_C[2][2]` and its offset (converted to m/s²) in `accel_O[2]`.

| JSON field | DR app (`dr_dead_reckoning_app_ins15_v3.c`) | Test pipeline (`test_app/imu_core.c`) |
|---|---|---|
| `accel_C`, `accel_O` | `g_cal.accel_C / accel_O` in `cal_set_defaults_from_lsq()` (~line 1738) | `cfg->accel_C / accel_O` in `imu_config_set_dr_defaults()` |
| `mount.R_bv` | `imu_calib_t.mount_R_bv` (~line 1531, default set ~line 1629) | `cfg->mount_R_bv` |
| `gyro_bias_counts` (+ tempco) | `imu_calib_t.gyro_bias` (convert counts → rad/s); initial EKF `bg` | boot-cal `gyro_bias_counts` (seed) |
| `gyro_scale` | gyro scale before bias removal | `cal.gyro_scale` |
| `fs.gyro_dps` | `GYRO_LSB_PER_DPS` must match the driver FS | streamer reads it from the driver |
| `g_local_mps2` | gravity constant in ZUPT / mechanisation | `cfg->gravity_mps2` |

> **Mount default mismatch.** The DR app's default `mount_R_bv = diag(1,-1,-1)` assumes an
> **FRD** sensor (lines ~1485–1629), but this installation is **FLU**, where the nominal mount
> is **identity**, as `imu_core.c` already uses. Fix the DR app default, or always load
> `R_bv` from the JSON. Otherwise y and z are inverted and every other calibration step will
> look wrong.

Loading order at boot: read the JSON and validate it (version, FS match, |R_bv| orthonormal,
values inside the acceptance limits). Then apply `accel_C/O` and `R_bv`. Then run the existing
boot stationary gyro calibration only to *refresh* `b_g`; if the vehicle is not stationary at
boot, start from the stored bias and its temperature model. Keep updating at stops (§6) and
write the refreshed bias and tempco back to the JSON no more than once per drive.

---

## 8. Acceptance limits (install sign-off)

| Item | Pass |
|---|---|
| \|g_meas − g_local\| / g_local | < 1 % |
| ground slope (two-heading) | < 3° (otherwise move to flatter ground) |
| mount roll / pitch repeatability (repeat §3) | < 0.2° |
| mount yaw repeatability (repeat §4) | < 0.3° |
| gyro bias | within ±2 dps; per-capture repeatability < 0.02 dps |
| gyro z scale | 1.00 ± 0.02; CCW vs CW agree within 0.3 % |
| stationary level-frame horizontal accel after cal | < 0.03 m/s² |

## 9. When to recalibrate

Recalibrate after any of the following:
- the IMU board or bracket is removed or replaced;
- the §6.6 re-leveling flag is raised;
- the gyro z scale drifts more than 0.5 % from its stored value;
- the gyro full-scale range is changed (redo §5; also redo §2 if the accel range changed).

Gyro bias and its temperature model are maintained automatically and never require a manual
recalibration.
