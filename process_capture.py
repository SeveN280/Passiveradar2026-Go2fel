import argparse

import numpy as np

from passive_radar import adsb, iq, processing as proc
from passive_radar.alert import AlertLink, format_line
from passive_radar.config import RX_SITE, TX_SITE, RadarConfig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--surv", required=True)
    ap.add_argument("--format", choices=["u8", "cf32"], default="u8")
    ap.add_argument("--lag", type=int, help="known channel offset (skip estimation)")
    ap.add_argument("--search", type=int, default=50000)
    ap.add_argument("--adsb", help="aircraft.json path or URL")
    ap.add_argument("--serial", help="ESP32 port, e.g. /dev/ttyUSB0")
    ap.add_argument("--plot", help="save range-Doppler map PNG")
    a = ap.parse_args()

    cfg = RadarConfig()
    load = iq.load_rtlsdr_u8 if a.format == "u8" else iq.load_cf32
    ref, surv = load(a.ref), load(a.surv)
    lag = a.lag if a.lag is not None else iq.estimate_lag(ref, surv, search=a.search)
    ref, surv = iq.align(ref, surv, lag)
    n = min(len(ref), cfg.n_samples)
    ref, surv = ref[:n], surv[:n]
    print(f"channel offset: {lag} samples, CPI {n / cfg.fs_hz:.2f} s")

    clean = proc.eca_b(ref, surv, cfg.eca_taps, int(cfg.eca_batch_s * cfg.fs_hz))
    rd, dop, rng = proc.range_doppler_map(proc.taper_reference(ref, cfg.usable_bw_frac), clean,
                                          cfg.fs_hz, cfg.n_range_bins, cfg.doppler_max_hz)
    dets = proc.detect(rd, dop, rng, cfg)

    preds = []
    if a.adsb:
        preds = adsb.predict(adsb.load_aircraft(a.adsb), TX_SITE, RX_SITE, cfg.wavelength)
        dets = adsb.match(dets, preds)
    link = AlertLink(a.serial)
    for d in dets:
        lab = adsb.classify(d, cfg.wavelength)
        print(f"{lab:9s} Rb={d['rb_m']:7.0f} m fD={d['fd_hz']:7.1f} Hz SNR={d['snr_db']:5.1f} dB "
              f"ADS-B={d.get('adsb')}")
        link.send(format_line(d, lab))

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 6), constrained_layout=True)
        im = ax.imshow(proc.to_db_map(rd), aspect="auto", origin="lower", cmap="Blues", vmin=0,
                       extent=[rng[0] / 1e3, rng[-1] / 1e3, dop[0], dop[-1]])
        for p in preds:
            ax.plot(p["rb_m"] / 1e3, p["fd_hz"], "x", color="#eb6834", ms=12, mew=2)
        for d in dets:
            ax.plot(d["rb_m"] / 1e3, d["fd_hz"], "o", mfc="none", color="k", ms=14, mew=2)
        ax.set_xlim(rng[0] / 1e3, rng[-1] / 1e3)
        ax.set_ylim(dop[0], dop[-1])
        ax.set_xlabel("Bistatic range [km]")
        ax.set_ylabel("Bistatic Doppler [Hz]")
        fig.colorbar(im, label="dB above noise floor")
        fig.savefig(a.plot, dpi=150)


if __name__ == "__main__":
    main()
