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
    lon = np.arctan2(y, x)
    p = np.sqrt(x*x + y*y)
    lat = np.arctan2(z, p*(1-e2))
    for _ in range(5):
        N = a / np.sqrt(1 - e2*np.sin(lat)**2)
        lat = np.arctan2(z + e2*N*np.sin(lat), p)
    h = p/np.cos(lat) - N
    return np.degrees(lat), np.degrees(lon), h

def enu_to_ecef(E, N, U, lat0, lon0, h0):
    lat0, lon0 = np.radians(lat0), np.radians(lon0)
    x0, y0, z0 = lla_to_ecef(lat0, lon0, h0)
    R = np.array([
        [-np.sin(lon0), -np.sin(lat0)*np.cos(lon0),  np.cos(lat0)*np.cos(lon0)],
        [ np.cos(lon0), -np.sin(lat0)*np.sin(lon0),  np.cos(lat0)*np.sin(lon0)],
        [ 0,             np.cos(lat0),               np.sin(lat0)]
    ])
    ecef = np.array([x0, y0, z0]) + R @ np.array([E, N, U])
    return ecef

# ---- Load CSV ----
df = pd.read_csv("navlog.csv")   # your logged CSV

# ---- ENU origin (from first GNSS fix) ----
lat0 = df.iloc[0]["lat_deg"]
lon0 = df.iloc[0]["lon_deg"]
h0   = df.iloc[0]["alt_m"]

lats, lons = [], []

for _, r in df.iterrows():
    ecef = enu_to_ecef(r.E, r.N, r.U, lat0, lon0, h0)
    lat, lon, _ = ecef_to_lla(*ecef)
    lats.append(lat)
    lons.append(lon)

df["lat"] = lats
df["lon"] = lons

# Save Google Maps compatible file
df[["lat", "lon"]].to_csv("trajectory_google_maps.csv", index=False)

print("Saved trajectory_google_maps.csv")



'''
1. Open https://www.google.com/mymaps
2. Create new map
3. Import -> Upload trajectory_google_maps.csv
4. Choose:
    1. Latitude = lat
    2. Longitude = lon
5. Select Line String (path)

'''