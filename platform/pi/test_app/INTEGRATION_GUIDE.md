# Integration Guide for the Main Dead-Reckoning Application

Do not paste the standalone code back into the fusion loop. Integrate it as a module with one sample-in/one result-out ownership rule.

## Required data flow

1. The IMU reader performs one blocking read from `/dev/mpu6050-0`.
2. It attaches a driver or monotonic timestamp and increments a sample sequence.
3. The sample is pushed into a bounded single-producer/single-consumer queue.
4. The navigation thread pops each sequence exactly once.
5. `imu_pipeline_process()` produces calibrated unfiltered and filtered vehicle-frame data plus quality flags.
6. AHRS and EKF consume only results marked calibration-valid and free of hard faults.
7. Telemetry publishes the same `imu_output_t`; it must not recalculate calibration independently.

## Merge sequence

### Stage A — qualification only

Run `imu_streamer` and the dashboard while the current dead-reckoning app is stopped. Confirm scale, signs, ODR, packet loss, stationary bias, noise, bump duty, and long-duration stability.

### Stage B — shadow mode

Link `imu_core.c` into the main application but do not use its filtered result for navigation. Log both the old path and new path against the same unique sample sequence. Compare every field offline.

### Stage C — navigation substitution

Replace the old `calib_accel()`, `calib_gyro()`, static median buffers, static LPF state, and boot calibration with the module. Keep AHRS/EKF unchanged initially.

### Stage D — retune

Once the sample path is deterministic, re-estimate EKF process noise from stationary and vehicle recordings. Do not tune the EKF to compensate for an IMU path defect.

## API boundary recommendation

- `imu_raw_sample_t`: raw counts and timestamp from the driver.
- `imu_output_t`: sensor SI, vehicle SI, filtered SI, quality metrics, calibration status.
- `imu_calibration_t`: the only persisted dynamic calibration.
- `imu_config_t`: full-scale, sensitivity, LSQ coefficients, mount, ODR, cutoff, and acceptance limits.

Never let AHRS, EKF, dashboard, and persistence maintain separate copies of gyro bias or accelerometer offset.
