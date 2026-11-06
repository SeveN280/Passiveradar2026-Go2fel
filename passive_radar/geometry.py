from dataclasses import dataclass

import numpy as np

C = 299_792_458.0

_A = 6378137.0
_F = 1 / 298.257223563
_E2 = _F * (2 - _F)


@dataclass(frozen=True)
class Site:

    lat: float
    lon: float
    alt: float


def lla_to_ecef(lat, lon, alt):
    lat, lon = np.radians(lat), np.radians(lon)
    n = _A / np.sqrt(1 - _E2 * np.sin(lat) ** 2)
    return np.array([
        (n + alt) * np.cos(lat) * np.cos(lon),
        (n + alt) * np.cos(lat) * np.sin(lon),
        (n * (1 - _E2) + alt) * np.sin(lat),
    ])


def ecef_to_lla(xyz):
    x, y, z = xyz
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1 - _E2))
    for _ in range(8):
        n = _A / np.sqrt(1 - _E2 * np.sin(lat) ** 2)
        alt = p / np.cos(lat) - n
        lat = np.arctan2(z, p * (1 - _E2 * n / (n + alt)))
    n = _A / np.sqrt(1 - _E2 * np.sin(lat) ** 2)
    alt = p / np.cos(lat) - n
    return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)


def _ecef_to_enu_matrix(ref: Site):
    lat, lon = np.radians(ref.lat), np.radians(ref.lon)
    sl, cl, so, co = np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon)
    return np.array([
        [-so, co, 0.0],
        [-sl * co, -sl * so, cl],
        [cl * co, cl * so, sl],
    ])


def lla_to_enu(lat, lon, alt, ref: Site):
    d = lla_to_ecef(lat, lon, alt) - lla_to_ecef(ref.lat, ref.lon, ref.alt)
    return _ecef_to_enu_matrix(ref) @ d


def enu_to_lla(enu, ref: Site):
    xyz = _ecef_to_enu_matrix(ref).T @ np.asarray(enu, float) + lla_to_ecef(ref.lat, ref.lon, ref.alt)
    return ecef_to_lla(xyz)


def azimuth_deg(enu):
    return float(np.degrees(np.arctan2(enu[0], enu[1])) % 360)


def bistatic_range_doppler(tx, rx, tgt, vel, wavelength):
    tx, rx, tgt, vel = (np.asarray(a, float) for a in (tx, rx, tgt, vel))
    baseline = np.linalg.norm(tx - rx)
    r1 = np.linalg.norm(tgt - tx)
    r2 = np.linalg.norm(tgt - rx)
    u1 = (tgt - tx) / r1
    u2 = (tgt - rx) / r2
    path_rate = vel @ (u1 + u2)
# fD > 0: the echo path is getting shorter (target approaching)
    return float(r1 + r2 - baseline), float(-path_rate / wavelength)


def locate_on_ellipse(tx, rx, rb, az_deg):
    tx2, rx2 = np.asarray(tx, float)[:2], np.asarray(rx, float)[:2]
    s = rb + np.linalg.norm(tx2 - rx2)
    u = np.array([np.sin(np.radians(az_deg)), np.cos(np.radians(az_deg))])
    d = rx2 - tx2
    t = (s ** 2 - d @ d) / (2 * (d @ u + s))
    return rx2 + t * u


def ellipse_points(tx, rx, rb, n=361):
    f1, f2 = np.asarray(tx, float)[:2], np.asarray(rx, float)[:2]
    L = np.linalg.norm(f1 - f2)
    a = (rb + L) / 2
    c = L / 2
    b = np.sqrt(max(a * a - c * c, 0.0))
    center = (f1 + f2) / 2
    ang = np.arctan2(*(f2 - f1)[::-1])
    t = np.linspace(0, 2 * np.pi, n)
    x, y = a * np.cos(t), b * np.sin(t)
    ca, sa = np.cos(ang), np.sin(ang)
    return np.stack([center[0] + ca * x - sa * y, center[1] + sa * x + ca * y], axis=1)
