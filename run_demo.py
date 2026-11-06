import json
import time
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap

from passive_radar import adsb, iq, processing as proc, simulate as sim
from passive_radar.config import RX_SITE, TX_SITE, RadarConfig
from passive_radar.geometry import (azimuth_deg, ellipse_points, lla_to_enu,
                                    locate_on_ellipse)

OUT = Path(__file__).resolve().parent / "out"
OUT.mkdir(exist_ok=True)


def save_fig(fig, name, dpi=170):
    import io
    buf = io.BytesIO()
    fig.savefig(buf, format="png", dpi=dpi)
    plt.close(fig)
    target = OUT / name
    try:
        target.write_bytes(buf.getvalue())
    except OSError as e:
        alt = OUT / f"{target.stem}_{time.strftime('%H%M%S')}{target.suffix}"
        alt.write_bytes(buf.getvalue())
        print(f"WARNING: {target.name} is locked ({e.__class__.__name__}) - "
              f"saved as {alt.name}. Close the image viewer / pause OneDrive.")
        target = alt
    print(f"saved {target}")

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
SEQ = LinearSegmentedColormap.from_list("seq_blue", [SURFACE, "#9cc3ef", S1, "#0d2f5c"])
plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
    "text.color": INK, "axes.titlecolor": INK, "axes.grid": True, "grid.color": GRID,
    "grid.linewidth": 0.6, "font.size": 11, "axes.titlesize": 13, "axes.spines.top": False,
    "axes.spines.right": False, "lines.linewidth": 2, "legend.frameon": False,
})


def fake_adsb_feed(path):
    data = {"now": time.time(), "aircraft": [
        {"hex": "48af01", "flight": "LOT3921 ", "lat": 49.9450, "lon": 19.5970,
         "alt_geom": 8200, "gs": 230, "track": 255, "geom_rate": 1500},
        {"hex": "4b1817", "flight": "SWR1484 ", "lat": 50.3200, "lon": 20.4500,
         "alt_geom": 37000, "gs": 455, "track": 118, "geom_rate": 0},
    ]}
    Path(path).write_text(json.dumps(data, indent=1))


def build_scenario(cfg, tx_enu, preds, rng):
    sc = []
    az = np.radians(262)
    pos = np.array([700 * np.sin(az), 700 * np.cos(az), 80.0])
    vel = -12.0 * np.array([np.sin(az), np.cos(az), 0.0])
    sc.append(sim.Scatterer("drone (DJI-class, 0.01 m2)", pos, vel, 0.01))
    lot = next(p for p in preds if p["flight"] == "LOT3921")
    sc.append(sim.Scatterer("aircraft LOT3921", lot["enu"], adsb.velocity_enu(lot), 20.0))
    for k in range(15):
        r, a = rng.uniform(200, 6000), rng.uniform(0, 2 * np.pi)
        sc.append(sim.Scatterer(f"clutter {k}", np.array([r * np.sin(a), r * np.cos(a), 20.0]),
                                np.zeros(3), 10 ** rng.uniform(1, 3.5), kind="clutter"))
    return sc


def detection_range_curve(cfg, tx_enu, budget):
    IMPL_LOSS_DB = 0.0
    step = 1 / 127.5
    sqnr = 10 ** (-cfg.adc_backoff_db / 10) / (step ** 2 / 6)
    p_surv_total = 10 ** ((budget["surv_total_dbm"] - 30) / 10)
    floor = 10 ** ((budget["noise_dbm"] - 30) / 10) + p_surv_total / sqnr
    az = np.radians(cfg.surv_azimuth_deg)
    dists = np.linspace(100, 5000, 600)
    curves = {}
    for rcs in (0.001, 0.01, 0.1):
        snr = [10 * np.log10(sim.echo_power_w(cfg, tx_enu,
                                              np.array([d * np.sin(az), d * np.cos(az), 80.0]), rcs) / floor)
               + budget["processing_gain_db"] - IMPL_LOSS_DB for d in dists]
        curves[rcs] = np.array(snr)
    return dists, curves, 10 * np.log10(floor) + 30, IMPL_LOSS_DB


def main():
    cfg = RadarConfig()
    rng = np.random.default_rng(7)
    lam = cfg.wavelength
    tx_enu = lla_to_enu(TX_SITE.lat, TX_SITE.lon, TX_SITE.alt, RX_SITE)

    print(f"Baseline TX-RX: {np.linalg.norm(tx_enu) / 1e3:.1f} km, TX bearing {azimuth_deg(tx_enu):.0f} deg")
    print(f"lambda = {lam:.3f} m, range bin = {cfg.range_bin_m:.0f} m, "
          f"Doppler resolution = {1 / cfg.cpi_s:.1f} Hz, processing gain = "
          f"{sim.processing_gain_db(cfg):.1f} dB")

    fake_adsb_feed(OUT / "aircraft.json")
    preds = adsb.predict(adsb.load_aircraft(OUT / "aircraft.json"), TX_SITE, RX_SITE, lam)

    scen = build_scenario(cfg, tx_enu, preds, rng)
    ref_u8, surv_u8, truth, budget = sim.simulate_capture(cfg, tx_enu, scen, rng, sample_offset=37)
    ref_u8.tofile(OUT / "sim_ref.bin")
    surv_u8.tofile(OUT / "sim_surv.bin")
    print("\nLink budget:", {k: round(v, 1) for k, v in budget.items()})
    for t in truth:
        if t["kind"] == "target":
            print(f"  {t['name']:28s} Rb={t['rb_m']:7.0f} m  fD={t['fd_hz']:7.1f} Hz  "
                  f"echo={t['echo_dbm']:6.1f} dBm  SNR_in={t['snr_in_db']:6.1f} dB")

    t0 = time.perf_counter()
    ref = iq.load_rtlsdr_u8(OUT / "sim_ref.bin")
    surv = iq.load_rtlsdr_u8(OUT / "sim_surv.bin")
    lag = iq.estimate_lag(ref, surv)
    ref, surv = iq.align(ref, surv, lag)
    t1 = time.perf_counter()
    rd_raw, dop, rng_m = proc.range_doppler_map(ref, surv, cfg.fs_hz, cfg.n_range_bins, cfg.doppler_max_hz)
    t2 = time.perf_counter()
    clean = proc.eca_b(ref, surv, cfg.eca_taps, int(cfg.eca_batch_s * cfg.fs_hz))
    t3 = time.perf_counter()
    ref_t = proc.taper_reference(ref, cfg.usable_bw_frac)
    rd, dop, rng_m = proc.range_doppler_map(ref_t, clean, cfg.fs_hz, cfg.n_range_bins, cfg.doppler_max_hz)
    dets = proc.detect(rd, dop, rng_m, cfg)
    t4 = time.perf_counter()
    dets = adsb.match(dets, preds)
    for d in dets:
        d["label"] = adsb.classify(d, lam)
    print(f"\nChannel offset found: {lag} samples (simulated 37)")
    print(f"Timing: align {t1 - t0:.2f}s, ECA {t3 - t2:.2f}s, CAF+CFAR {t4 - t3:.2f}s "
          f"for a {cfg.cpi_s}s CPI")

    print("\nDetections:")
    for d in dets:
        print(f"  {d['label']:9s} Rb={d['rb_m']:7.0f} m  fD={d['fd_hz']:7.1f} Hz  "
              f"SNR={d['snr_db']:5.1f} dB  ADS-B={d['adsb']}")

    db_raw, db = proc.to_db_map(rd_raw), proc.to_db_map(rd)
    ext = [rng_m[0] / 1e3, rng_m[-1] / 1e3, dop[0], dop[-1]]
    vmax = max(db.max(), 30)

    fig, axs = plt.subplots(1, 2, figsize=(13, 5.2), sharey=True, constrained_layout=True)
    for ax, m, title in ((axs[0], db_raw, "Before clutter cancellation"),
                         (axs[1], db, "After ECA clutter cancellation")):
        im = ax.imshow(m, aspect="auto", origin="lower", extent=ext, cmap=SEQ, vmin=0, vmax=vmax,
                       interpolation="nearest")
        ax.set_title(title, loc="left")
        ax.set_xlabel("Bistatic range [km]")
        ax.grid(False)
    axs[0].set_ylabel("Bistatic Doppler [Hz]")
    fig.colorbar(im, ax=axs, label="Power above noise floor [dB]", shrink=0.9)
    save_fig(fig, "fig_rd_before_after.png")

    fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
    im = ax.imshow(db, aspect="auto", origin="lower", extent=ext, cmap=SEQ, vmin=0, vmax=vmax,
                   interpolation="nearest")
    ax.grid(False)
    fig.colorbar(im, ax=ax, label="Power above noise floor [dB]")
    for p in preds:
        if rng_m[0] <= p["rb_m"] <= rng_m[-1] and abs(p["fd_hz"]) <= cfg.doppler_max_hz:
            ax.plot(p["rb_m"] / 1e3, p["fd_hz"], marker="x", ms=14, mew=2.5, color=S2, ls="none",
                    label="ADS-B prediction")
    for d in dets:
        ax.plot(d["rb_m"] / 1e3, d["fd_hz"], marker="o", ms=16, mfc="none", mew=2, color=INK,
                ls="none", label="Radar detection (CFAR)")
        ax.annotate(f"{d['label']}  {d['snr_db']:.0f} dB" + (f"\n{d['adsb']}" if d["adsb"] else ""),
                    (d["rb_m"] / 1e3, d["fd_hz"]), xytext=(12, 10), textcoords="offset points",
                    color=INK, fontsize=11, fontweight="bold")
    h, l = ax.get_legend_handles_labels()
    uniq = dict(zip(l, h))
    ax.legend(uniq.values(), uniq.keys(), loc="lower right")
    ax.set_xlabel("Bistatic range [km]")
    ax.set_ylabel("Bistatic Doppler [Hz]")
    ax.set_title("Range-Doppler map, 0.5 s integration: drone found, aircraft confirmed by ADS-B",
                 loc="left")
    save_fig(fig, "fig_rd_detections.png")

    fig, ax = plt.subplots(figsize=(13, 7.2), constrained_layout=True)
    b = np.radians(cfg.surv_azimuth_deg)
    for side in (-1, 1):
        a = b + side * np.radians(cfg.surv_beamwidth_deg / 2)
        ax.plot([0, 12 * np.sin(a)], [0, 12 * np.cos(a)], color=INK2, lw=1, ls="--")
    label_offsets = {"AIRCRAFT": (-13.5, 7.0), "DRONE?": (2.0, -7.0)}
    for d in dets:
        e = ellipse_points(tx_enu, np.zeros(3), d["rb_m"]) / 1e3
        ax.plot(e[:, 0], e[:, 1], color=S1, lw=1.2, alpha=0.7)
        est = locate_on_ellipse(tx_enu, np.zeros(3), d["rb_m"], cfg.surv_azimuth_deg) / 1e3
        ax.plot(*est, marker="o", ms=11, mfc="none", mew=2, color=S1, ls="none",
                label="Radar estimate (ellipse x beam axis)")
        name = d["adsb"] or d["label"]
        ax.annotate(f"{name}: Rb {d['rb_m'] / 1e3:.2f} km, fD {d['fd_hz']:.0f} Hz", est,
                    xytext=label_offsets.get(d["label"], (2, -6)), textcoords="data", fontsize=10,
                    arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    for t, s in zip(truth, scen):
        if t["kind"] == "target":
            ax.plot(s.pos[0] / 1e3, s.pos[1] / 1e3, marker="^", ms=9, color=S2, ls="none",
                    label="True position (simulation)")
    ax.plot(*(tx_enu[:2] / 1e3), marker="s", ms=11, color=INK, ls="none")
    ax.annotate("TX Chorągwica\nMUX4 474 MHz, 100 kW", tx_enu[:2] / 1e3, xytext=(-14, 14),
                textcoords="offset points", fontsize=10, ha="right")
    ax.plot(0, 0, marker="D", ms=9, color=S3, ls="none")
    ax.annotate("RX (Yagi + ATK 5)", (0, 0), xytext=(2.5, -2.0), textcoords="data", fontsize=10,
                arrowprops=dict(arrowstyle="-", color=INK2, lw=0.8))
    ax.text(-13.5, -4.2, f"Yagi beam ({cfg.surv_beamwidth_deg:.0f}°)", color=INK2)
    h, l = ax.get_legend_handles_labels()
    uniq = dict(zip(l, h))
    ax.legend(uniq.values(), uniq.keys(), loc="upper right")
    ax.set_aspect("equal")
    ax.set_xlim(-15, 33)
    ax.set_ylim(-12, 12)
    ax.set_xlabel("East [km]")
    ax.set_ylabel("North [km]")
    ax.set_title("Geometry: each detection lies on an ellipse with foci TX and RX", loc="left")
    save_fig(fig, "fig_geometry.png")

    dists, curves, floor_dbm, _ = detection_range_curve(cfg, tx_enu, budget)
    REAL_MARGIN_DB = 8.0
    drone_t = truth[0]
    drone_det = next((d for d in dets if d["label"] == "DRONE?"), None)
    calib = 0.0
    if drone_det:
        theory_at_drone = np.interp(drone_t["dist_rx_m"], dists, curves[drone_t["rcs_m2"]])
        calib = drone_det["snr_db"] - theory_at_drone
        print(f"Calibration: simulation is {calib:+.1f} dB vs. theory")
    curves = {k: v + calib - REAL_MARGIN_DB for k, v in curves.items()}
    impl_loss = REAL_MARGIN_DB
    fig, ax = plt.subplots(figsize=(9, 5.2), constrained_layout=True)
    for (rcs, snr), col in zip(curves.items(), (S1, S2, S3)):
        ax.plot(dists, snr, color=col, label=f"RCS {rcs:g} m²")
        ax.annotate(f"{rcs:g} m²", (dists[-1], snr[-1]), xytext=(6, 0), textcoords="offset points",
                    color=INK, va="center")
        rmax = dists[snr >= 13].max() if np.any(snr >= 13) else 0
        print(f"Predicted max detection range, RCS {rcs:g} m2: {rmax:.0f} m")
    if drone_det:
        ax.plot(drone_t["dist_rx_m"], drone_det["snr_db"] - impl_loss, marker="o", ms=9, color=INK,
                ls="none", label=f"Simulated drone (− {impl_loss:.0f} dB real-world margin)")
    ax.axhline(13, color=INK2, lw=1, ls="--")
    ax.text(3200, 14, "detection threshold ≈ 13 dB", color=INK2)
    ax.set_xlabel("Distance from receiver along Yagi boresight [m]")
    ax.set_ylabel("SNR after 0.5 s integration [dB]")
    ax.set_title(f"How far can it see a drone? (calibrated on simulation, −{impl_loss:.0f} dB margin)",
                 loc="left")
    ax.set_xlim(0, 5400)
    ax.set_ylim(-15, 40)
    ax.legend(loc="upper right")
    save_fig(fig, "fig_detection_range.png")

    summary = dict(config={k: v for k, v in cfg.__dict__.items() if k != "extra"}, budget=budget,
                   effective_floor_dbm=floor_dbm, lag=lag,
                   truth=[t for t in truth if t["kind"] == "target"],
                   detections=[{k: v for k, v in d.items() if k != "bin"} for d in dets],
                   adsb_predictions=[{k: v for k, v in p.items() if k != "enu"} for p in preds])
    (OUT / "results.json").write_text(json.dumps(summary, indent=1, default=float))
    print(f"\nEffective noise floor (thermal + 8-bit quantisation): {floor_dbm:.1f} dBm")


if __name__ == "__main__":
    main()
