# Source Extraction Map

This map identifies the IMU-related regions in the uploaded dead-reckoning source. The exact text, with original line numbers, is in `ORIGINAL_IMU_AUDIT_EXCERPT.txt`.

| Original lines | Content | Standalone destination |
|---:|---|---|
| 101–105 | device path, expected rate, default `dt` | `imu_streamer.c`, `imu_config_t` |
| 149–216 | stillness, ZUPT, dynamics, median/LPF, bump constants | `imu_config_set_dr_defaults()` |
| 235–419 | gyro sensitivity, persistence, boot/runtime calibration, AHRS and IMU noise constants | `imu_core.h/.c`; review document for EKF-only constants |
| 443–452 | five-sample median helper | `median5()` |
| 1090–1345 | calibration LED and global state machine | intentionally excluded from core; UI receives calibration state over network |
| 1350–1806 | calibration structures, LSQ coefficients, raw-to-SI conversion | `imu_config_t`, `raw_to_sensor()` |
| 1807–1860 | centralized preprocessing and mount rotation | `calibrated_to_vehicle()` |
| 1861–1984 | runtime gyro learner | not enabled in the standalone test; unsafe without independent stationary evidence |
| 1985–2335 | AHRS complementary filter | not part of sensor qualification core; retain as a separate navigation layer |
| 2336–2635 | stationary accumulator and power-on calibration | `update_boot_calibration()`, `finalize_boot_calibration()` |
| 2710–2850 | GNSS turn-segment gyro-Z scale learner | excluded from isolated IMU test; requires validated GNSS and controlled integration |
| 2870–3230 | EKF IMU process-noise and mechanization | navigation layer, not sensor extraction |
| 3588–3618 | IMU driver read thread | simplified to one blocking read per processed sample in `imu_streamer.c` |
| 4300–4510 | median, LPF, vibration/bump path | explicit pipeline state in `imu_core.c` |
| 4511–4625 | ZUPT conditions, runtime calibration call, boot-cal call | quality flags and boot calibration in `imu_core.c` |
| 4825–4870 | AHRS update and gravity removal | excluded from isolated sensor qualification |
| 4948–4985 | yaw-scale learner input | excluded from isolated sensor qualification |
| 5860–5930 | load, mirror, sync and boot-cal startup | versioned/fingerprinted calibration load in `imu_core.c` |

## What was intentionally not copied

The test module does not contain GNSS, EKF, NHC, ZUPT state updates, attitude estimation, or position integration. Those layers can make an IMU look better or worse and would prevent an isolated assessment of raw calibration and filtering.
