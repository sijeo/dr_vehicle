import pandas as pd
import numpy as np

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
    for _ in range(10):  # Increased iterations for better accuracy
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
    Convert ENU coordinates to ECEF coordinates
    e, n, u: ENU coordinates (meters)
    lat0, lon0, h0: Reference point (degrees, meters)
    Returns: x, y, z in ECEF (meters)
    """
    # Convert reference point to ECEF
    x0, y0, z0 = lla_to_ecef(lat0, lon0, h0)
    
    # Convert reference lat/lon to radians
    lat0_rad = np.radians(lat0)
    lon0_rad = np.radians(lon0)
    
    # ENU to ECEF transformation matrix (transpose of ECEF to ENU)
    R = np.array([
        [-np.sin(lon0_rad), -np.sin(lat0_rad)*np.cos(lon0_rad), np.cos(lat0_rad)*np.cos(lon0_rad)],
        [np.cos(lon0_rad), -np.sin(lat0_rad)*np.sin(lon0_rad), np.cos(lat0_rad)*np.sin(lon0_rad)],
        [0, np.cos(lat0_rad), np.sin(lat0_rad)]
    ])
    
    # ENU vector
    enu_vector = np.array([e, n, u])
    
    # Convert to ECEF
    ecef_offset = R.T @ enu_vector  # Note: using transpose of R
    ecef_coords = np.array([x0, y0, z0]) + ecef_offset
    
    return ecef_coords

# ---- Load CSV ----
df = pd.read_csv("navlog.csv")   # your logged CSV

# ---- ENU origin (from first GNSS fix) ----
lat0 = df.iloc[0]["lat_deg"]
lon0 = df.iloc[0]["lon_deg"]
h0   = df.iloc[0]["alt_m"]

print(f"Reference point: lat={lat0}, lon={lon0}, h={h0}")

# Test the conversion with the origin point
print("\nTesting conversion with origin point (should give back original coordinates):")
test_ecef = enu_to_ecef(0, 0, 0, lat0, lon0, h0)
test_lat, test_lon, test_h = ecef_to_lla(*test_ecef)
print(f"Original: lat={lat0}, lon={lon0}")
print(f"Converted back: lat={test_lat}, lon={test_lon}")
print(f"Difference: dlat={test_lat-lat0}, dlon={test_lon-lon0}")

lats, lons, alts = [], [], []

for idx, r in df.iterrows():
    ecef = enu_to_ecef(r.E, r.N, r.U, lat0, lon0, h0)
    lat, lon, alt = ecef_to_lla(*ecef)
    lats.append(lat)
    lons.append(lon)
    alts.append(alt)
    
    # Debug first few points
    if idx < 3:
        print(f"\nPoint {idx}:")
        print(f"  ENU: E={r.E:.2f}, N={r.N:.2f}, U={r.U:.2f}")
        print(f"  Result: lat={lat:.6f}, lon={lon:.6f}")

df["lat"] = lats
df["lon"] = lons
df["alt"] = alts

# Save Google Maps compatible file
df[["lat", "lon"]].to_csv("trajectory_google_maps.csv", index=False)

print(f"\nSaved trajectory_google_maps.csv with {len(df)} points")
print(f"Latitude range: {min(lats):.6f} to {max(lats):.6f}")
print(f"Longitude range: {min(lons):.6f} to {max(lons):.6f}")


'''
1. Open https://www.google.com/mymaps
2. Create new map
3. Import -> Upload trajectory_google_maps.csv
4. Choose:
    1. Latitude = lat
    2. Longitude = lon
5. Select Line String (path)

'''