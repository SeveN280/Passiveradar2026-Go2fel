import json
import urllib.request

import numpy as np

from .geometry import Site, bistatic_range_doppler, lla_to_enu

FT = 0.3048
KT = 0.514444


def load_aircraft(source):
    if str(source).startswith("http"):
        with urllib.request.urlopen(source, timeout=5) as r:
            data = json.load(r)
    else:
        with open(source) as fh:
            data = json.load(fh)
    out = []
    for a in data.get("aircraft", []):
        if not all(k in a for k in ("lat", "lon", "gs", "track")):
            continue
        alt_ft = a.get("alt_geom", a.get("alt_baro"))
        if not isinstance(alt_ft, (int, float)):
            continue
        rate = a.get("geom_rate", a.get("baro_rate", 0)) or 0
        out.append(dict(
            hex=a.get("hex", "?"), flight=(a.get("flight") or "").strip() or a.get("hex", "?"),
            lat=a["lat"], lon=a["lon"], alt_m=alt_ft * FT,
            gs_mps=a["gs"] * KT, track_deg=a["track"], vrate_mps=rate * FT / 60,
        ))
    return out


def velocity_enu(ac):
    tr = np.radians(ac["track_deg"])
    return np.array([ac["gs_mps"] * np.sin(tr), ac["gs_mps"] * np.cos(tr), ac["vrate_mps"]])


def predict(aircraft, tx: Site, rx: Site, wavelength):
    tx_enu = lla_to_enu(tx.lat, tx.lon, tx.alt, rx)
    preds = []
    for ac in aircraft:
        pos = lla_to_enu(ac["lat"], ac["lon"], ac["alt_m"], rx)
        rb, fd = bistatic_range_doppler(tx_enu, np.zeros(3), pos, velocity_enu(ac), wavelength)
        preds.append({**ac, "rb_m": rb, "fd_hz": fd, "enu": pos})
    return preds


def match(detections, predictions, rb_tol_m=300.0, fd_tol_hz=15.0):
    for d in detections:
        best, best_cost = None, np.inf
        for p in predictions:
            drb, dfd = abs(d["rb_m"] - p["rb_m"]), abs(d["fd_hz"] - p["fd_hz"])
            if drb <= rb_tol_m and dfd <= fd_tol_hz:
                cost = (drb / rb_tol_m) ** 2 + (dfd / fd_tol_hz) ** 2
                if cost < best_cost:
                    best, best_cost = p, cost
        d["adsb"] = best["flight"] if best else None
        d["adsb_error"] = (d["rb_m"] - best["rb_m"], d["fd_hz"] - best["fd_hz"]) if best else None
    return detections


def classify(det, wavelength, max_drone_speed=35.0):
    if det.get("adsb"):
        return "AIRCRAFT"
    v_min = abs(det["fd_hz"]) * wavelength / 2
    return "DRONE?" if v_min <= max_drone_speed else "UNKNOWN"
