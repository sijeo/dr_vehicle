"""
Vehicle + IMU + GNSS 2D simulator in Pygame with EKF overlay
- Vehicle (arrow) is always centered on screen and always moves forward.
- Grid behaves like latitude/longitude (labels are LLA derived from ENU).
- Sliders set target speed (m/s), longitudinal acceleration (m/s^2), and yaw-rate (deg/s).
- IMU (6-axis) outputs simulated body-frame accel (ax, ay, az) and gyro (gx, gy, gz) with noise.
- GNSS outputs noisy latitude/longitude at a lower rate (1 Hz by default) *and* ENU meters.
- EKF (15-state error-state INS with GPS updates) consumes IMU + GNSS and renders an
  estimated dotted trail alongside the true trail so you can see convergence.
- Stats panel shows true state, latest sensors, and estimated state.

Run:
    pip install pygame numpy
    python vehicle_sim.py

Keys:
    R: reset
    Space: pause/resume

Author: ChatGPT
"""
import math
import random
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import pygame

# Import your EKF implementation (ekf.py placed next to this script)
from ekf import EKF15State, EKFConfig  # uses your uploaded filter

# ------------------------------- Utilities ----------------------------------
EARTH_R = 6378137.0  # meters


def enu_to_lla(x_e, y_n, lat0, lon0):
    """Small-angle ENU->LLA around (lat0, lon0) in radians.
    x_e: meters East, y_n: meters North
    Returns lat, lon in degrees.
    """
    lat = lat0 + y_n / EARTH_R
    lon = lon0 + x_e / (EARTH_R * math.cos(lat0))
    return math.degrees(lat), math.degrees(lon)


def quat_to_yaw_rad(q: np.ndarray) -> float:
    """Yaw from quaternion [w, x, y, z] (ENU, z-up)."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# ------------------------------ Slider UI -----------------------------------
class Slider:
    def __init__(self, rect, min_val, max_val, init_val, label, step=None):
        self.rect = pygame.Rect(rect)
        self.min = min_val
        self.max = max_val
        self.val = float(init_val)
        self.label = label
        self.dragging = False
        self.step = step

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(event.pos):
            self.dragging = True
            self._update_from_pos(event.pos[0])
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._update_from_pos(event.pos[0])

    def _update_from_pos(self, x):
        t = (x - self.rect.x) / self.rect.w
        t = max(0.0, min(1.0, t))
        v = self.min + t * (self.max - self.min)
        if self.step:
            v = round(v / self.step) * self.step
        self.val = v

    def draw(self, surf, font):
        # Track
        pygame.draw.rect(surf, (210, 210, 210), self.rect, border_radius=6)
        # Thumb position
        t = (self.val - self.min) / (self.max - self.min)
        cx = int(self.rect.x + t * self.rect.w)
        thumb = pygame.Rect(0, 0, 12, self.rect.h + 6)
        thumb.center = (cx, self.rect.centery)
        pygame.draw.rect(surf, (80, 80, 80), thumb, border_radius=6)
        # Text
        label = f"{self.label}: {self.val: .2f}"
        text = font.render(label, True, (30, 30, 30))
        surf.blit(text, (self.rect.x, self.rect.y - 20))


# ------------------------------ Sensors -------------------------------------
@dataclass
class IMUReading:
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float


class IMUSim:
    def __init__(self, accel_noise=0.03, gyro_noise=0.002):
        self.accel_noise = accel_noise  # m/s^2 std
        self.gyro_noise = gyro_noise    # rad/s std

    def sample(self, a_long, v, yaw_rate):
        """Return body-frame specific forces (ax, ay, az) and body rates (gx, gy, gz).
        Simplified planar model:
          ax ≈ a_long + noise
          ay ≈ v * yaw_rate + noise   (lateral/centripetal)
          az ≈ 0
          gx ≈ 0, gy ≈ 0, gz ≈ yaw_rate + noise
        """
        ax = a_long + random.gauss(0, self.accel_noise)
        ay = v * yaw_rate + random.gauss(0, self.accel_noise)
        az = 0.0 + random.gauss(0, 0.005)
        gx = 0.0 + random.gauss(0, self.gyro_noise)
        gy = 0.0 + random.gauss(0, self.gyro_noise)
        gz = yaw_rate + random.gauss(0, self.gyro_noise)
        return IMUReading(ax, ay, az, gx, gy, gz)


@dataclass
class GNSSReading:
    lat_deg: float
    lon_deg: float
    enu_x: float
    enu_y: float
    tstamp: float


class GNSSSim:
    def __init__(self, lat0_deg=12.9716, lon0_deg=77.5946, rate_hz=1.0,
                 pos_noise_std=0.8):  # meters std
        self.lat0 = math.radians(lat0_deg)
        self.lon0 = math.radians(lon0_deg)
        self.dt = 1.0 / rate_hz
        self.timer = 0.0
        self.pos_noise_std = pos_noise_std
        self.latest = None

    def maybe_sample(self, dt, x_e, y_n, now):
        self.timer += dt
        produced = None
        if self.timer >= self.dt:
            self.timer -= self.dt
            # Add measurement noise in ENU meters then convert
            nx = random.gauss(0, self.pos_noise_std)
            ny = random.gauss(0, self.pos_noise_std)
            lat, lon = enu_to_lla(x_e + nx, y_n + ny, self.lat0, self.lon0)
            self.latest = GNSSReading(lat, lon, x_e + nx, y_n + ny, now)
            produced = self.latest
        return produced


# ------------------------------ Vehicle -------------------------------------
class Vehicle:
    def __init__(self):
        self.x = 0.0  # East (m)
        self.y = 0.0  # North (m)
        self.yaw = 0.0  # rad, 0 means East, positive CCW
        self.v = 0.0  # m/s
        self.trail = deque(maxlen=10_000)

    def reset(self):
        self.__init__()

    def step(self, dt, a_long, yaw_rate):
        # Update speed and heading
        self.v += a_long * dt
        self.v = max(0.0, self.v)  # always forward
        self.yaw += yaw_rate * dt
        # normalize
        if self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        elif self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        # Integrate position (world frame)
        dx = self.v * math.cos(self.yaw) * dt
        dy = self.v * math.sin(self.yaw) * dt
        self.x += dx
        self.y += dy
        self.trail.append((self.x, self.y))


# ------------------------------ Drawing -------------------------------------
ARROW_POLY = [
    (20, 0),   # tip
    (-16, -10),
    (-8, 0),
    (-16, 10),
]


def draw_arrow_centered(surf, yaw):
    cx, cy = surf.get_width() // 2, surf.get_height() // 2
    # Build rotated polygon
    pts = []
    c, s = math.cos(yaw), math.sin(yaw)
    for (x, y) in ARROW_POLY:
        xr = x * c - y * s
        yr = x * s + y * c
        pts.append((cx + int(xr), cy + int(yr)))
    pygame.draw.polygon(surf, (30, 120, 250), pts)
    pygame.draw.polygon(surf, (0, 0, 0), pts, 2)


def world_to_screen(wx, wy, cam_offset, surf):
    cx, cy = surf.get_width() // 2, surf.get_height() // 2
    sx = cx + int((wx - cam_offset[0]))
    sy = cy - int((wy - cam_offset[1]))
    return sx, sy


def draw_trail(surf, points, cam_offset, color=(100, 0, 180), width=2):
    pts = [world_to_screen(x, y, cam_offset, surf) for (x, y) in points]
    if len(pts) > 1:
        pygame.draw.lines(surf, color, False, pts, width)


def draw_dotted_trail(surf, points, cam_offset, every=6, radius=2, color=(20, 180, 20)):
    for i, (wx, wy) in enumerate(points):
        if i % every == 0:
            sx, sy = world_to_screen(wx, wy, cam_offset, surf)
            pygame.draw.circle(surf, color, (sx, sy), radius)


def draw_grid(surf, cam_offset, lat0_rad, lon0_rad, meters_per_grid=20.0):
    w, h = surf.get_size()
    cx, cy = w // 2, h // 2

    min_x = cam_offset[0] - w // 2
    max_x = cam_offset[0] + w // 2
    min_y = cam_offset[1] - h // 2
    max_y = cam_offset[1] + h // 2

    start_x = math.floor(min_x / meters_per_grid) * meters_per_grid
    start_y = math.floor(min_y / meters_per_grid) * meters_per_grid

    grid_col = (220, 220, 220)
    bold_col = (200, 200, 200)
    txt_col = (120, 120, 120)

    font = pygame.font.SysFont("consolas", 14)

    x = start_x
    while x <= max_x:
        sx = cx + int(x - cam_offset[0])
        color = bold_col if (abs((x / meters_per_grid) % 5) < 1e-6) else grid_col
        pygame.draw.line(surf, color, (sx, 0), (sx, h))
        lat_lbl, lon_lbl = enu_to_lla(x, cam_offset[1], lat0_rad, lon0_rad)
        label = f"lon {lon_lbl:.6f}"
        surf.blit(font.render(label, True, txt_col), (sx + 3, 3))
        x += meters_per_grid

    y = start_y
    while y <= max_y:
        sy = cy - int(y - cam_offset[1])
        color = bold_col if (abs((y / meters_per_grid) % 5) < 1e-6) else grid_col
        pygame.draw.line(surf, color, (0, sy), (w, sy))
        lat_lbl, lon_lbl = enu_to_lla(cam_offset[0], y, lat0_rad, lon0_rad)
        label = f"lat {lat_lbl:.6f}"
        surf.blit(font.render(label, True, txt_col), (3, sy - 16))
        y += meters_per_grid


# ------------------------------ Main Loop -----------------------------------
class App:
    def __init__(self):
        pygame.init()
        self.W, self.H = 1100, 700
        self.screen = pygame.display.set_mode((self.W, self.H))
        pygame.display.set_caption("Vehicle + IMU + GNSS + EKF (lat/lon grid)")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("consolas", 18)
        self.small = pygame.font.SysFont("consolas", 14)

        # World & sensors
        self.lat0_deg, self.lon0_deg = 12.9716, 77.5946  # Bengaluru
        self.lat0_rad, self.lon0_rad = math.radians(self.lat0_deg), math.radians(self.lon0_deg)

        self.vehicle = Vehicle()
        self.imu = IMUSim(accel_noise=0.05, gyro_noise=0.003)
        self.gnss = GNSSSim(lat0_deg=self.lat0_deg, lon0_deg=self.lon0_deg, rate_hz=1.0, pos_noise_std=0.8)

        # EKF
        self.ekf = EKF15State(EKFConfig())
        self.est_trail = deque(maxlen=10_000)

        self.paused = False
        self.latest_imu = None
        self.latest_gnss = None

        # Error metrics (for stats)
        self.err_hist = deque(maxlen=60*60*10)  # ~10 minutes at 60 Hz; (t, ex, ey, |e|)

        # UI sliders (x, y, w, h)
        panel_x, panel_w = 20, 320
        self.sld_speed = Slider((panel_x, 60, panel_w, 16), 0.0, 20.0, 5.0, "Target speed (m/s)", step=0.1)
        self.sld_accel = Slider((panel_x, 110, panel_w, 16), -5.0, 5.0, 0.0, "Longitudinal accel (m/s^2)", step=0.05)
        self.sld_yaw = Slider((panel_x, 160, panel_w, 16), -3.14, 3.14, 0.0, "Yaw rate (rad/s)", step=0.01)

    def reset(self):
        self.vehicle.reset()
        self.ekf = EKF15State(EKFConfig())
        self.est_trail.clear()
        self.latest_imu = None
        self.latest_gnss = None

    def run(self):
        while True:
            dt = self.clock.tick(60) / 1000.0  # seconds
            now = time.time()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    return
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.reset()
                    if event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                # slider events
                self.sld_speed.handle_event(event)
                self.sld_accel.handle_event(event)
                self.sld_yaw.handle_event(event)

            # Physics & sensors
            a_long = float(self.sld_accel.val)
            yaw_rate = math.radians(self.sld_yaw.val)

            if not self.paused:
                # Step true vehicle
                self.vehicle.step(dt, a_long, yaw_rate)

                # IMU @ render rate -> EKF predict
                self.latest_imu = self.imu.sample(a_long=a_long, v=self.vehicle.v, yaw_rate=yaw_rate)
                accel_meas = np.array([self.latest_imu.ax, self.latest_imu.ay, self.latest_imu.az], dtype=float)
                gyro_meas  = np.array([self.latest_imu.gx, self.latest_imu.gy, self.latest_imu.gz], dtype=float)
                self.ekf.predict(gyro_meas, accel_meas, dt)

                # GNSS @ 1 Hz -> EKF update
                sample = self.gnss.maybe_sample(dt, self.vehicle.x, self.vehicle.y, now)
                if sample:
                    self.latest_gnss = sample
                    gps_pos = np.array([sample.enu_x, sample.enu_y, 0.0], dtype=float)
                    self.ekf.update_gps(gps_pos)

                # Save estimated trail
                self.est_trail.append((float(self.ekf.p[0]), float(self.ekf.p[1])))

                # Error tracking
                ex = float(self.ekf.p[0] - self.vehicle.x)
                ey = float(self.ekf.p[1] - self.vehicle.y)
                e  = math.hypot(ex, ey)
                self.err_hist.append((time.time(), ex, ey, e))

            # Draw
            self.screen.fill((245, 245, 245))

            # Camera follows vehicle (keep centered)
            cam = (self.vehicle.x, self.vehicle.y)

            # Grid
            draw_grid(self.screen, cam, self.lat0_rad, self.lon0_rad, meters_per_grid=20.0)

            # Trails
            draw_trail(self.screen, self.vehicle.trail, cam, color=(100, 0, 180), width=2)  # true (solid)
            draw_dotted_trail(self.screen, self.est_trail, cam, every=6, radius=2, color=(20, 180, 20))  # est (dotted)

            # Vehicle glyph
            draw_arrow_centered(self.screen, self.vehicle.yaw)

            # UI & stats
            self._draw_ui()

            pygame.display.flip()

    # -------------------------- UI & Stats ---------------------------------
    def _draw_ui(self):
        # Left control panel
        panel = pygame.Rect(10, 10, 350, 200)
        pygame.draw.rect(self.screen, (255, 255, 255), panel, border_radius=10)
        pygame.draw.rect(self.screen, (0, 0, 0), panel, 2, border_radius=10)
        self.screen.blit(self.font.render("Controls", True, (0, 0, 0)), (20, 20))
        self.sld_speed.draw(self.screen, self.small)
        self.sld_accel.draw(self.screen, self.small)
        self.sld_yaw.draw(self.screen, self.small)

        # Compute error stats
        nowt = time.time()
        # instantaneous
        ex_i = float(self.ekf.p[0] - self.vehicle.x)
        ey_i = float(self.ekf.p[1] - self.vehicle.y)
        e_i  = math.hypot(ex_i, ey_i)
        # windowed stats (last 10s)
        win_s = 10.0
        ex2_sum = ey2_sum = e2_sum = 0.0
        n_win = 0
        e_max = 0.0
        for (t, ex, ey, e) in reversed(self.err_hist):
            if nowt - t > win_s:
                break
            ex2_sum += ex*ex
            ey2_sum += ey*ey
            e2_sum  += e*e
            e_max = max(e_max, e)
            n_win += 1
        rmse_e = math.sqrt(e2_sum / n_win) if n_win > 0 else 0.0
        rmse_ex = math.sqrt(ex2_sum / n_win) if n_win > 0 else 0.0
        rmse_ey = math.sqrt(ey2_sum / n_win) if n_win > 0 else 0.0

        # Bottom-right stats panel
        sp_w, sp_h = 520, 300
        sp = pygame.Rect(self.W - sp_w - 10, self.H - sp_h - 10, sp_w, sp_h)
        pygame.draw.rect(self.screen, (255, 255, 255), sp, border_radius=10)
        pygame.draw.rect(self.screen, (0, 0, 0), sp, 2, border_radius=10)

        # True values
        true_lat, true_lon = enu_to_lla(self.vehicle.x, self.vehicle.y, self.lat0_rad, self.lon0_rad)
        # Estimated values
        est_lat, est_lon = enu_to_lla(float(self.ekf.p[0]), float(self.ekf.p[1]), self.lat0_rad, self.lon0_rad)
        est_yaw = quat_to_yaw_rad(self.ekf.q)

        lines = []
        lines.append("Stats (true / sensors / EKF)")
        lines.append(f"True ENU: x={self.vehicle.x:7.2f} m  y={self.vehicle.y:7.2f} m | v={self.vehicle.v:5.2f} m/s  yaw={math.degrees(self.vehicle.yaw):6.2f}°")
        lines.append(f"True LLA: lat={true_lat:.6f}°, lon={true_lon:.6f}°")
        if self.latest_imu:
            imu = self.latest_imu
            lines.append(f"IMU accel (ax,ay,az) m/s^2: {imu.ax: .3f}, {imu.ay: .3f}, {imu.az: .3f}")
            lines.append(f"IMU gyro  (gx,gy,gz) rad/s: {imu.gx: .4f}, {imu.gy: .4f}, {imu.gz: .4f}")
        else:
            lines.append("IMU: …")
            lines.append("")
        if self.latest_gnss:
            g = self.latest_gnss
            ago = time.time() - g.tstamp
            lines.append(f"GNSS LLA: lat={g.lat_deg:.6f}°, lon={g.lon_deg:.6f}°  ({ago: .1f}s ago)")
        else:
            lines.append("GNSS: waiting…")
        lines.append(f"EKF ENU: x={self.ekf.p[0]:7.2f} m  y={self.ekf.p[1]:7.2f} m | |yaw={math.degrees(est_yaw):6.2f}°")
        lines.append(f"EKF LLA: lat={est_lat:.6f}°, lon={est_lon:.6f}°")

        # Error block
        lines.append("— Position Error —")
        lines.append(f"Instant:  ex={ex_i: .2f} m  ey={ey_i: .2f} m  |  |e|={e_i: .2f} m")
        if n_win > 0:
            lines.append(f"RMSE (last {int(win_s)}s):  ex={rmse_ex: .2f} m  ey={rmse_ey: .2f} m  |  |e|={rmse_e: .2f} m  (n={n_win})")
            lines.append(f"Max |e| (last {int(win_s)}s): {e_max: .2f} m")
        else:
            lines.append("RMSE: waiting for samples…")

        # Render lines
        x0, y0 = sp.x + 12, sp.y + 12
        for i, txt in enumerate(lines):
            self.screen.blit(self.small.render(txt, True, (10, 10, 10)), (x0, y0 + i * 18))

        # Footer
        footer = "[R]eset  [Space] Pause/Run  |  True trail: solid purple  |  EKF: green dots"
        self.screen.blit(self.small.render(footer, True, (100, 100, 100)), (sp.x + 12, sp.bottom - 24))


if __name__ == "__main__":
    App().run()
