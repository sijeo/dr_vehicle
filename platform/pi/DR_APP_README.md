# Vehicle dead reckoning application

`dr_dead_reckoning_app_ins15_v3.c` is the final DR application. It runs headless on the Raspberry Pi
in the vehicle, starts at power-on, and reports its state on one LED. One push button (recommended)
or an SSH command starts the mount calibration.

It uses the IMU chain that was proven on the bench with `test_app/imu_streamer` and
`imu_rotation_dashboard.py`. The same source files are compiled in, not re-implemented:

```
mpu6050_char.ko   HW FIFO (no lost samples), reconstructed sample times, SFLP gyro bias
   └ imu_core     accel scale, sensor→vehicle mount, boot gyro cal, median + LPF      test_app/imu_core.c
   └ imu_fusion   Mahony AHRS with the live SFLP gyro bias, gravity removal            test_app/imu_fusion.c
   └ dr_nav       vehicle EKF: position, speed, heading, yaw-rate bias, accel bias,    dr_nav.c
                  yaw-rate scale; GNSS position/velocity, ZUPT + ZARU at stops
neo6m_gnss_serdev.ko → GNSS thread (v2 ioctl, freshness by sequence counter, validation)
```

**Yaw drift.** The bench showed 0.28–0.29°/min of residual heading drift with the SFLP bias alone.
The nav filter estimates that residual as the `bg` state and corrects it:
- **at every stop (ZARU)**: while the vehicle is stationary, the yaw rate must be zero, so whatever the
  gyro reads is bias;
- **while driving with GNSS**: GNSS velocity corrects the heading directly.

In simulation, with the bench's residual bias, ZARU brought the drift down from 0.29 to about 0.01°/min.
So a GNSS outage after even one stop starts with a well-calibrated gyro. On the vehicle, stop now and
then during long tests (traffic lights count).

---

## 1. Hardware

| Item | Notes |
|---|---|
| IMU | ISM330BX on I²C-1 at **400 kHz**: `dtparam=i2c_arm_baudrate=400000` in `/boot/config.txt`. Mount it rigidly (no foam), x roughly forward and z up. The calibration takes out the exact angles. |
| GNSS | TAU1204 via `neo6m_gnss_serdev.ko` (`/dev/neo6m0`). Antenna on the roof with a clear sky view. |
| LED | Existing calibration LED on **BCM 13** (pin 33), active high. |
| **Button (recommended, the only UI to add)** | Momentary, normally-open push button between **BCM 26 (pin 37)** and **GND (pin 39)**. Add `gpio=26=ip,pu` to `/boot/config.txt` for the internal pull-up; the loader also sets it. Optionally add 100 nF across the switch. Put it where the driver can reach it **while parked**. |
| Power | The app fsyncs its logs every 10 s and saves its state while parked, so a cut costs at most 10 s of log. To avoid SD-card damage, prefer a clean shutdown (button held 10 s) or a UPS/supercap HAT with ignition sense. |

Nothing else is needed. A laptop is **optional**: live view and calibration over Wi-Fi with `--udp`
(section 7).

## 2. Build and install

On the Linux build machine (same toolchain as before):

```bash
cd platform/pi
make                 # mpu6050_char.ko (driver)
make app_v4          # dr_dead_reckoning_app  (app + dr_nav + test_app IMU chain)
make deploy deploy_dr PI_HOST=sijeo@<pi-ip>    # .ko, app, loader script, systemd unit
```

On the Pi, once:

```bash
cd /home/sijeo/dr_vehicle
chmod +x load_dr_stack.sh dr_dead_reckoning_app
sudo cp dr_app.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now dr_app        # starts now and at every boot
journalctl -u dr_app -f                   # live text log (same as debug_*.log)
```

To run it by hand instead: `sudo systemctl stop dr_app; sudo ./load_dr_stack.sh [options]`.
`./dr_dead_reckoning_app --help` lists all options. The useful ones:

| Option | Default | Use |
|---|---|---|
| `--outage-test P,L` | off | ignore GNSS for L s every P s, to measure DR accuracy (section 8) |
| `--udp HOST:PORT` | off | live frames to `imu_rotation_dashboard.py` on a laptop |
| `--mount-cal` | off | start the mount calibration at power-up (no button) |
| `--gyro-fs DPS` | 500 | gyro range (500 gives 17.5 mdps/LSB) |
| `--gnss-latency S` | 0.10 | GNSS output delay compensated in the position update |
| `--gnss-pos-sigma M` | 1.5 | GNSS position noise (scaled up with HDOP) |
| `--imu-log-hz N` | 50 | rate of `imuframes_*.bin` (0 = off) |
| `--button-gpio N` / `--led-gpio N` / `--gpio-base N` | 26 / 13 / 0 | pins; `--gpio-base 512` on kernels ≥ 6.6 |

## 3. What happens at power-on (no action needed)

1. **IMU start** (LED 1 Hz blink). The app needs either the boot gyro calibration (about 22 s still)
   or 5 s of the live SFLP gyro bias, so it is ready within seconds even if the vehicle is already
   moving.
2. **Waiting for GNSS** (short flash every second). The first good fix sets the local map origin.
3. **Heading alignment.** The heading comes from the GNSS course the first time the vehicle drives
   faster than **4 m/s (~15 km/h)**. If the vehicle has not moved since the last parking, the heading
   saved at parking (`dr_state.txt`) is used immediately. That also allows starting **without GNSS**,
   e.g. in a garage: after 30 s the app starts from the saved position.
4. **Navigating** (LED steady on). In a GNSS outage it keeps going on the IMU (LED on with a short gap
   each second). When GNSS returns, fixes are gated by NIS. If GNSS disagrees for 5 fixes in a row, the
   app re-anchors to GNSS.

## 4. LED and button

| LED | Meaning |
|---|---|
| blink 1 Hz (0.5 s on / off) | IMU starting / gyro calibrating |
| short flash every 1 s | waiting for GNSS, or positioned but heading not yet known (drive > 15 km/h) |
| **steady on** | navigating with GNSS |
| on, with a short gap every 1 s | navigating on dead reckoning (GNSS outage) |
| double flash every 3 s (on top of the above) | **mount not calibrated yet**: run section 5 |
| N quick flashes + pause | mount calibration in progress, step N (1 = level, 2 = turned, 3 = forward) |
| on 3 s / fast 5 Hz for 3 s | mount calibration succeeded / failed |
| fast 5 Hz continuously | fatal: IMU missing (check the driver; systemd retries every 3 s) |

| Button | Action |
|---|---|
| short press (< 3 s) | **marker**: written to `navlog` (`marker` column) and the debug log. Use it to tag events on a drive ("tunnel entry", "bad bump", ...). During calibration step 2: skip the turn. |
| hold 3 s (LED goes solid), release | start the mount calibration; during a calibration: abort it |
| hold 10 s (LED goes off), release | clean shutdown of the Pi |

## 5. Calibration

### Automatic, every drive (nothing to do)
- **Gyro bias**: the live SFLP estimate (tracks temperature), plus **ZARU at every stop**, plus GNSS
  heading while moving. The boot calibration (`imu_calibration.bin`) is the fallback.
- **Accelerometer**: the factory scale is used; gravity is removed with the measured |g|, so the
  board's 5 % scale error does not bias the forward acceleration. The forward-accel bias `ba` is
  estimated from GNSS speed.
- **Yaw-rate scale** (`sg`): estimated from GNSS in turns.

### Mount calibration (once per installation, and after the IMU is moved)
This tells the app how the board sits in the vehicle (roll, pitch, yaw). It is stored in
`/home/sijeo/dr_vehicle/imu_mount.txt`. Without it the app assumes the board is exactly FLU.

**In the vehicle, with the button (no laptop):**
1. Park on reasonably level ground (a flat car park). Engine on or off.
2. Hold the button 3 s and release. The LED shows **1 flash** (step 1).
3. **Stay still ~5 s.** The step completes on its own; the LED goes to **2 flashes**.
4. **Turn around and park on the same spot facing the opposite way** (three-point turn; use the same
   wheel marks). Stop and **stay still ~5 s**. The turn must be 150–210°. If you cannot turn around,
   short-press the button to skip this step (the surface slope then stays in roll/pitch).
   The LED goes to **3 flashes**.
5. **Pull away forward firmly in a straight line** (≥ 0.1 g for about 1 s, then drive normally).
   Not in reverse.
6. LED **on for 3 s** = done and saved. Fast blinking for 3 s = failed (see the debug log); repeat
   from step 2. The whole procedure times out after 15 minutes.

**Without a button:** over SSH, `touch /home/sijeo/dr_vehicle/START_MOUNT_CAL` starts the same
procedure (then follow steps 3–6). Alternatively, start the app with `--mount-cal`.

**On the bench / with a laptop:** start the app with `--udp <laptop-ip>:5005` (or use
`imu_streamer`) and run `imu_rotation_dashboard.py`; its wizard (key `m`) writes the same
`imu_mount.txt`. The file format is shared, so a bench calibration done with `imu_streamer` carries
over to the DR app **if the board is not moved between them**.

The result is in the debug log: `MOUNTCAL done: correction roll .. pitch .. yaw ..`. A few degrees is
normal. A large correction (e.g. 90° yaw) means the board is mounted rotated; that is fine, it is
compensated.

## 6. Files

In `/home/sijeo/dr_vehicle/` (keep across runs):

| File | Content |
|---|---|
| `imu_mount.txt` | sensor→vehicle rotation (mount calibration) |
| `imu_calibration.bin` | boot gyro calibration (rebuilt automatically) |
| `dr_state.txt` | last parked position, heading and bias estimates (start-up without GNSS) |
| `START_MOUNT_CAL` | create it to start the mount calibration; the app deletes it |

In `/home/sijeo/nav_logs/`, one set per run (`<stamp>` = start time):

| File | Rate / size | Content / use |
|---|---|---|
| `navlog_<stamp>.csv` | 10 Hz, ~18 MB/h | DR solution, GNSS, errors, filter states, flags: **main analysis file** |
| `imuframes_<stamp>.bin` | 50 Hz, ~67 MB/h | IMU frames (AHRS, raw, filtered): `imu_rotation_dashboard.py --replay` |
| `debug_<stamp>.log` | small | events: modes, alignment, GNSS rejects, re-anchors, mount calibration, markers, 10 s status |

Main `navlog` columns:

| Column(s) | Meaning |
|---|---|
| `time_s`, `mode` | boot time; INIT / WAIT_GNSS / ALIGNING / NAV_GNSS / NAV_DR |
| `ekf_lat`, `ekf_lon`, `E`, `N` | DR position (deg; m in the local plane) |
| `speed_mps`, `heading_cog_deg` | DR speed and heading (compass, 0 = North, clockwise) |
| `bg_dps`, `ba_mps2`, `sg` | yaw-rate bias, forward accel bias, yaw-rate scale error |
| `sig_pos_m`, `sig_heading_deg` | filter 1-σ position / heading uncertainty |
| `lat_deg`, `lon_deg`, `gnss_E`, `gnss_N`, `gnss_speed_mps`, `course_deg`, `hdop` | last GNSS fix |
| `err_E`, `err_N`, `err_H` | DR minus GNSS (m); during `--outage-test` this is the DR error |
| `gnss_used_pos`, `gnss_used_vel`, `gnss_reject`, `nis_pos`, `nis_vel`, `snap_applied` | GNSS usage and gating |
| `outage_s`, `outage_sim` | time since the last used GNSS; 1 while a test outage is active |
| `zupt_active`, `zaru_active` | vehicle detected stationary |
| `a_fwd_mps2`, `w_up_dps`, `roll_deg`, `pitch_deg`, `ahrs_yaw_deg` | IMU inputs to the filter, AHRS attitude |
| `temp_c`, `imu_quality`, `ahrs_bias_sflp`, `driver_dropped`, `fifo_overrun` | IMU health |
| `mountcal_step`, `marker` | calibration step, button marker count |

Copy the logs to a PC with: `scp -r sijeo@<pi-ip>:/home/sijeo/nav_logs .`

## 7. Analysis: is the DR working?

```bash
python tools/plot_dr_log.py nav_logs/navlog_<stamp>.csv           # summary + PNGs + KML
python platform/pi/test_app/imu_rotation_dashboard.py --replay nav_logs/imuframes_<stamp>.bin --speed 4
```

`plot_dr_log.py` prints the summary, the outage table (duration, distance, end error, % of distance,
heading error) and writes:
- `_track.png`: GNSS and DR tracks, outages in orange;
- `_timeseries.png`: error, speed, heading, biases, mode / stops;
- `_outages.png`: error vs distance against a 2 % line;
- `_tracks.kml`: open in Google Earth.

**Live in the vehicle (optional laptop):** start the app with `--udp <laptop-ip>:5005` and run
`imu_rotation_dashboard.py --no-wizard` on the laptop. The cubes and plots work as on the bench,
and the dashboard wizard can calibrate the mount over Wi-Fi.

What good looks like:
- **outage test:** 30 s outages end within ~2–3 % of the distance driven; heading error at the end
  < ~1°. The simulation of this design gave a median of 0.4 % and a worst case of 1.5 %; expect
  real roads to be worse than the simulation.
- `bg_dps` settles within a few minutes and stays small (|bg| × 60 < ~0.1°/min); it moves at stops,
  not between them.
- `ba_mps2` within ±0.1 m/s²; `sg` within ±2 %.
- `driver_dropped` constant, `fifo_overrun` 0, `ahrs_bias_sflp` 1.
- no repeated `snap_applied` while GNSS is good (HDOP < 2).
- stopped → `zupt_active` 1 within ~1 s; speed reads 0.00.

## 8. Readiness checklist before the first vehicle test

Bench (Pi on the desk, IMU board in hand):
1. `dmesg`: `signal chain: ... SFLP on ... read path=HW FIFO`; I²C at 400 kHz
   (`xxd /sys/class/i2c-adapter/i2c-1/of_node/clock-frequency` → `0006 1a80`).
2. `sudo ./load_dr_stack.sh` runs; the debug log shows `IMU ready`, then `ENU origin` once GNSS
   has a fix near a window; the LED pattern matches section 4.
3. Button: short press → `MARKER 1` in the log; 3 s → `MOUNTCAL started` (then hold 3 s again to abort).
4. 10-minute still test: `navlog` `speed_mps` stays ≈ 0 and `zupt_active` = 1. With `--outage-test 120,60`
   the DR position must not walk away while parked (ZUPT holds it).
5. Unplug the GNSS antenna while running: mode → NAV_DR (LED gap pattern); plug it back → NAV_GNSS.

Vehicle, parked:
6. IMU rigidly fixed; cables strain-relieved; antenna on the roof.
7. `systemctl enable dr_app`; power-cycle the vehicle supply. The app starts by itself and the LED
   reaches "short flash" or "steady".
8. Run the **mount calibration** (section 5). Check `MOUNTCAL done` in the log and that the LED no
   longer double-flashes.

First drives (a second person handles the laptop/notes; never touch the button while driving):
9. **Car park:** 2 laps each way, a few hard stops. Check `heading_cog_deg` against `course_deg`,
   `zupt_active` at stops, and the marker for each lap.
10. **Road, 20–30 min, GNSS good:** confirm NAV_GNSS most of the time, `bg_dps` settles, small `err_H`
    (≈ GNSS noise, 1–3 m).
11. **Outage test:** set `ExecStart=... --outage-test 180,30` in the service (or run by hand) and drive
    30 min with normal stops. Then run `plot_dr_log.py` and check the outage table against section 7.
    Then try 60 s outages (`--outage-test 240,60`).
12. **Real outages** (underpass, tunnel, tall buildings): mark them with the button. The table lists
    them as "real".

## 9. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| LED 5 Hz forever, `IMU: cannot open` | driver not loaded / wrong overlay; `dmesg`, `ls /dev/mpu6050-0` |
| stays WAIT_GNSS although the receiver has a fix; debug log `GNSS: FATAL ioctl mismatch`; `test_neo6m_gnss_serdev` says "Inappropriate ioctl for device" | the loaded `neo6m_gnss_serdev.ko` was built from an older `neo6m_gnss_ioctl.h` (the ioctl numbers encode the struct sizes). Rebuild it (`make` in `platform/pi` now builds both drivers), `sudo rmmod neo6m_gnss_serdev && sudo insmod neo6m_gnss_serdev.ko`; `dmesg \| grep "ioctl ABI"` must show `GET_FIX=0x80264e01`, `GET_FIX_V2=0x80424e02` |
| receiver LED shows a fix but `have_fix` stays 0 | check `cat /sys/class/neo6m_gnss/neo6m0/{baud,rx_bytes,last_line,parser_stats}`. **baud "auto-scanning" / `last_line` garbage**: wrong UART rate; the driver auto-detects it, and `insmod neo6m_gnss_serdev.ko baud=115200` fixes it. **`last_line` readable but RMC field 2 = `V` / GGA quality 0**: the receiver has no navigation fix yet; its LED may only show time/PPS lock. **`rx_bytes` not increasing**: wiring (TX→RX), the UART overlay, or the serial console still owning the UART |
| never leaves "short flash" | no GNSS fix (antenna, sky view), or the vehicle has not exceeded 15 km/h yet |
| `fifo_overrun` / `driver_dropped` rising | I²C not at 400 kHz; or keep `--sflp-mask 4` (default) |
| heading wrong by ~180° after reversing out at start | heading came from the saved state but the vehicle was moved while off; it corrects at the first drive > 15 km/h |
| mount calibration fails "forward acceleration too small" | pull away more firmly, in a straight line, within 4 s of moving off |
| mount calibration never leaves step 2 | the turn was not 150–210°, or the vehicle was not still for 5 s; short-press to skip |
| many `GNSS reject: pos_jump` | multipath (buildings); expected occasionally; DR bridges them |
| `snap_applied` repeating | DR and GNSS disagree persistently: check the mount calibration, or GNSS latency (`--gnss-latency`) |
| button does nothing | pull-up missing (`gpio=26=ip,pu`), wrong pin, or kernel ≥ 6.6 needs `--gpio-base 512` |

## 10. Where to tune

| What | Where |
|---|---|
| filter noise, ZUPT/ZARU strength, NIS gate, re-anchor count | `dr_nav_default_cfg()` in `dr_nav.c` |
| stationary detection, mount-calibration thresholds | `STILL_*`, `MC_*` defines at the top of the app |
| GNSS validation limits | `GNSS_*` / `MAX_GNSS_*` defines at the top of the app |
| AHRS gains | `ahrs_kp` / `ahrs_ki` in `main()` (0.3 / 0: vehicle, bias from SFLP) |

Known limits: planar navigation (altitude is logged from GNSS only); no wheel-speed input; no GNSS
lever-arm correction (mount the antenna above the IMU if possible); the SFLP attitude is not used
(see `test_app/README.md`).
