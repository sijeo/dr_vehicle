# Validation Performed in This Environment

The following checks were completed before packaging:

1. `imu_core.c` and `imu_wire.c` compiled with GCC using:
   - C11
   - `-Wall -Wextra -Wpedantic -Wconversion -Wshadow`
2. Synthetic stationary calibration test passed:
   - calibration state reached `VALID`;
   - observed rate was 100.00 Hz;
   - filtered acceleration norm was 9.8062 m/s²;
   - network frame size was exactly 236 bytes.
3. AddressSanitizer and UndefinedBehaviorSanitizer test passed.
4. `imu_streamer.c` compiled against a temporary ABI-compatible header stub. It still must be rebuilt on the Raspberry Pi against the real project `mpu6050_ioctl.h`.
5. C simulator to Python receiver interoperability passed:
   - network byte order decoded correctly;
   - CRC32 verified;
   - sequence, sample rate, calibration state, and float fields decoded correctly.
6. A 691-frame simulated CSV capture was analyzed successfully:
   - median receive/sample rate approximately 98.5 Hz under non-real-time desktop scheduling;
   - `summary.json`, time-series, spectrum, and Allan-deviation plots were generated.

Not yet tested here:

- the real `/dev/mpu6050-0` driver ABI;
- actual Raspberry Pi scheduling and packet-loss behavior;
- physical axis/mount signs;
- real ISM330DLC ODR/full-scale register configuration;
- real bench noise, temperature drift, and on-vehicle vibration.
