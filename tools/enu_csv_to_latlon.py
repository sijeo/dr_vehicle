import pandas as pd
import numpy as np
import sys
import os

# ---- WGS84 constants ----
a = 6378137.0
f = 1 / 298.257223563
b = a * (1 - f)
e2 = (a*a - b*b) / (a*a)

def lla_to_ecef(lat, lon, h):
    lat, lon = np.radians(lat), np.radians(lon)
    N = a / np.sqrt(1 - e2*np.sin(lat)**2)
    x = (N + h) * np.cos(lat) * np.cos(lon)
    y = (N + h) * np.cos(lat) * np.sin(lon)
    z = (N*(1-e2) + h) * np.sin(lat)
    return np.array([x, y, z])

def ecef_to_lla(x, y, z):
    # Iterative method for better accuracy
    lon = np.arctan2(y, x)
    p = np.sqrt(x*x + y*y)

    # Initial latitude
    lat = np.arctan2(z, p*(1 - e2))

    # Iterate to refine latitude
    for _ in range(10):
        N = a / np.sqrt(1 - e2*np.sin(lat)**2)
        h = p / np.cos(lat) - N
        lat_new = np.arctan2(z, p * (1 - e2 * (N / (N + h))))
        if np.abs(lat_new - lat) < 1e-12:
            break
        lat = lat_new

    N = a / np.sqrt(1 - e2*np.sin(lat)**2)
    h = p / np.cos(lat) - N

    return np.degrees(lat), np.degrees(lon), h

def enu_to_ecef(e, n, u, lat0, lon0, h0):
    """
    Convert ENU coordinates to ECEF coordinates.
    e, n, u: ENU coordinates (meters)
    lat0, lon0, h0: Reference point (degrees, meters)
    Returns: x, y, z in ECEF (meters)
    """
    # Convert reference point to ECEF
    x0, y0, z0 = lla_to_ecef(lat0, lon0, h0)

    # Convert reference lat/lon to radians
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)

    # ENU-to-ECEF rotation matrix (transpose of the ECEF-to-ENU matrix).
    # This matrix IS already ENU→ECEF — apply directly with R @ enu.
    R = np.array([
        [-np.sin(lon0_rad), -np.sin(lat0_rad)*np.cos(lon0_rad), np.cos(lat0_rad)*np.cos(lon0_rad)],
        [np.cos(lon0_rad), -np.sin(lat0_rad)*np.sin(lon0_rad), np.cos(lat0_rad)*np.sin(lon0_rad)],
        [0, np.cos(lat0_rad), np.sin(lat0_rad)]
    ])

    enu_vector = np.array([e, n, u])

    # R is ENU→ECEF, apply directly (do NOT use R.T)
    ecef_offset = R @ enu_vector
    ecef_coords = np.array([x0, y0, z0]) + ecef_offset

    return ecef_coords

# ---- Main ----
csv_file = sys.argv[1] if len(sys.argv) > 1 else "navlog.csv"
if not os.path.exists(csv_file):
    print(f"ERROR: File not found: {csv_file}")
    sys.exit(1)

df = pd.read_csv(csv_file)
basename = os.path.splitext(os.path.basename(csv_file))[0]

# ---- ENU origin (from first GNSS fix — matches C code nav init) ----
lat0 = df.iloc[0]["lat_deg"]
lon0 = df.iloc[0]["lon_deg"]
h0   = df.iloc[0]["alt_m"]

print(f"Input: {csv_file} ({len(df)} rows)")
print(f"ENU reference: lat={lat0:.7f}, lon={lon0:.7f}, alt={h0:.1f} m")

# Round-trip test: ENU(0,0,0) should give back the reference point
test_ecef = enu_to_ecef(0, 0, 0, lat0, lon0, h0)
test_lat, test_lon, test_h = ecef_to_lla(*test_ecef)
print(f"Round-trip check: dlat={test_lat-lat0:.10f}, dlon={test_lon-lon0:.10f} (should be ~0)")

# ---- Convert ENU → lat/lon ----
ekf_lats, ekf_lons = [], []

for idx, r in df.iterrows():
    ecef = enu_to_ecef(r.E, r.N, r.U, lat0, lon0, h0)
    lat, lon, alt = ecef_to_lla(*ecef)
    ekf_lats.append(lat)
    ekf_lons.append(lon)

    if idx < 3:
        print(f"  Row {idx}: ENU=({r.E:.2f}, {r.N:.2f}, {r.U:.2f}) -> lat={lat:.7f}, lon={lon:.7f}")

# ---- Output: EKF trajectory ----
out_ekf = f"{basename}_ekf_trajectory.csv"
pd.DataFrame({"lat": ekf_lats, "lon": ekf_lons}).to_csv(out_ekf, index=False)
print(f"\nEKF trajectory: {out_ekf} ({len(ekf_lats)} points)")

# ---- Output: Raw GNSS trajectory (for comparison) ----
# Filter to rows with valid GNSS fix and non-zero lat/lon
gnss_mask = (df["gnss_fix"] == 1) & (df["lat_deg"].abs() > 0.1) & (df["lon_deg"].abs() > 0.1)
gnss_df = df[gnss_mask]
out_gnss = f"{basename}_gnss_trajectory.csv"
gnss_df[["lat_deg", "lon_deg"]].rename(columns={"lat_deg": "lat", "lon_deg": "lon"}).to_csv(out_gnss, index=False)
print(f"GNSS trajectory: {out_gnss} ({len(gnss_df)} points)")

# ---- Output: Combined (both on one map) ----
out_combined = f"{basename}_combined.csv"
combined_rows = []
for i in range(len(ekf_lats)):
    combined_rows.append({"lat": ekf_lats[i], "lon": ekf_lons[i], "source": "EKF"})
for _, r in gnss_df.iterrows():
    combined_rows.append({"lat": r["lat_deg"], "lon": r["lon_deg"], "source": "GNSS"})
pd.DataFrame(combined_rows).to_csv(out_combined, index=False)
print(f"Combined:        {out_combined} ({len(combined_rows)} points)")

# ---- Summary stats ----
err_h = df["err_H"]
print(f"\nHorizontal error stats (err_H from CSV):")
print(f"  Mean:   {err_h.mean():.1f} m")
print(f"  Median: {err_h.median():.1f} m")
print(f"  Max:    {err_h.max():.1f} m")
print(f"  <5m:    {(err_h < 5).sum()}/{len(err_h)} ({100*(err_h < 5).mean():.0f}%)")

nis = df["nis_pos"]
used = df["gnss_used_pos"]
print(f"\nGNSS acceptance: {used.sum()}/{len(used)} ({100*used.mean():.0f}%) rows used position update")
print(f"NIS stats: mean={nis.mean():.0f}, median={nis.median():.1f}, max={nis.max():.0f}")


'''
Usage:
  python enu_csv_to_latlon.py navlog_2026-03-28_07-31-43.csv

Output files:
  navlog_2026-03-28_07-31-43_ekf_trajectory.csv   - EKF positions as lat/lon
  navlog_2026-03-28_07-31-43_gnss_trajectory.csv   - Raw GNSS lat/lon
  navlog_2026-03-28_07-31-43_combined.csv           - Both with "source" column

To plot on Google Maps:
  1. Open https://www.google.com/mymaps
  2. Create new map
  3. Import -> Upload the _combined.csv file
  4. Choose: Latitude = lat, Longitude = lon
  5. Style by "source" column to see EKF vs GNSS in different colours
'''
