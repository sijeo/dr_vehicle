#!/usr/bin/env python3
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

# WGS84 constants
a = 6378137.0
f = 1 / 298.257223563
e2 = f * (2 - f)

def lla_to_ecef(lat, lon, alt):
    """lat, lon in radians; alt in meters."""
    s = np.sin(lat)
    c = np.cos(lat)
    N = a / np.sqrt(1 - e2 * s * s)
    x = (N + alt) * c * np.cos(lon)
    y = (N + alt) * c * np.sin(lon)
    z = (N * (1 - e2) + alt) * s
    return np.array([x, y, z])

def ecef_to_enu(e, ref_lat, ref_lon, ref_ecef):
    """Convert ECEF vector to ENU using reference point."""
    sl = np.sin(ref_lat); cl = np.cos(ref_lat)
    so = np.sin(ref_lon); co = np.cos(ref_lon)
    dx = e[0] - ref_ecef[0]
    dy = e[1] - ref_ecef[1]
    dz = e[2] - ref_ecef[2]
    E = -so*dx + co*dy
    N = -cl*co*dx - cl*so*dy + sl*dz
    U =  sl*co*dx + sl*so*dy + cl*dz
    return np.array([E, N, U])


# ------------------------------------------
# Read CSV file
# ------------------------------------------
if len(sys.argv) < 2:
    print("Usage: plot_nav.py <logfile.csv>")
    sys.exit(1)

csvfile = sys.argv[1]

# Columns (in order)
# time_s, E,N,U, Vx,Vy,Vz, Ax,Ay,Az,
# imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz,
# gnss_fix, lat,lon,alt, speed, heading, hdop, outage_s

E_dr = []
N_dr = []

gnss_fix_list = []
gnss_lat = []
gnss_lon = []
gnss_alt = []

with open(csvfile, "r") as f:
    reader = csv.reader(f)
    header = next(reader, None)   # skip header

    for row in reader:
        if len(row) < 25:
            continue

        # DR position (ENU)
        try:
            E_dr.append(float(row[1]))
            N_dr.append(float(row[2]))
        except:
            E_dr.append(np.nan)
            N_dr.append(np.nan)

        # GNSS fields
        fix = int(row[16])
        lat = float(row[17])
        lon = float(row[18])
        alt = float(row[19])

        gnss_fix_list.append(fix)
        gnss_lat.append(lat)
        gnss_lon.append(lon)
        gnss_alt.append(alt)

E_dr = np.array(E_dr)
N_dr = np.array(N_dr)

# ------------------------------------------
# Convert GNSS lat/lon into ENU (same origin)
# ------------------------------------------
# Find the first FIX = 1 entry to set origin
try:
    idx0 = gnss_fix_list.index(1)
except ValueError:
    print("No GNSS fix found in log! Cannot plot GNSS path.")
    idx0 = None

if idx0 is not None:
    ref_lat = np.radians(gnss_lat[idx0])
    ref_lon = np.radians(gnss_lon[idx0])
    ref_alt = gnss_alt[idx0]

    ref_ecef = lla_to_ecef(ref_lat, ref_lon, ref_alt)

    E_gn = []
    N_gn = []

    for fix, lat_deg, lon_deg, alt_m in zip(gnss_fix_list, gnss_lat, gnss_lon, gnss_alt):
        if fix == 1:
            lat_r = np.radians(lat_deg)
            lon_r = np.radians(lon_deg)
            ecef = lla_to_ecef(lat_r, lon_r, alt_m)
            enu = ecef_to_enu(ecef, ref_lat, ref_lon, ref_ecef)
            E_gn.append(enu[0])
            N_gn.append(enu[1])
        else:
            E_gn.append(np.nan)
            N_gn.append(np.nan)

    E_gn = np.array(E_gn)
    N_gn = np.array(N_gn)
else:
    E_gn = N_gn = None


# ------------------------------------------
# Plot
# ------------------------------------------
plt.figure(figsize=(8, 8))
plt.plot(E_dr, N_dr, 'b-', label="Dead Reckoning (DR)", linewidth=2)

if E_gn is not None:
    plt.scatter(E_gn, N_gn, s=12, c='red', label="GNSS Fix", alpha=0.7)

plt.xlabel("East (m)")
plt.ylabel("North (m)")
plt.title("Vehicle Trajectory (DR vs GNSS)")
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.tight_layout()
plt.show()


#Sample run command: python3 plot_nav.py navlog_2025-01-20_14-33-10.csv
