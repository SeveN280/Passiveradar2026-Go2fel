from dataclasses import dataclass

import numpy as np
from scipy import fft as sfft

from .config import RadarConfig
from .geometry import C, azimuth_deg, bistatic_range_doppler

K_BOLTZMANN = 1.380649e-23
T0 = 290.0


@dataclass
class Scatterer:
    name: str
    pos: np.ndarray
    vel: np.ndarray
    rcs_m2: float
    kind: str = "target"


def surv_gain_lin(az_deg, cfg: RadarConfig):
    d = (az_deg - cfg.surv_azimuth_deg + 180) % 360 - 180
    g_db = cfg.surv_gain_dbi - 12 * (d / cfg.surv_beamwidth_deg) ** 2
    g_db = max(g_db, cfg.surv_gain_dbi + cfg.surv_backlobe_db)
    return 10 ** (g_db / 10)


def eirp_in_band_w(cfg: RadarConfig):
    return cfg.erp_w * 1.64 * cfg.captured_fraction


def noise_power_w(cfg: RadarConfig):
    return K_BOLTZMANN * T0 * cfg.fs_hz * 10 ** (cfg.noise_figure_db / 10)


def direct_powers_w(cfg: RadarConfig, tx_enu):
    lam = cfg.wavelength
    L = np.linalg.norm(tx_enu)
    fspl = (lam / (4 * np.pi * L)) ** 2
    loss = 10 ** (-cfg.direct_extra_loss_db / 10)
    p_ref = eirp_in_band_w(cfg) * 10 ** (cfg.ref_gain_dbi / 10) * fspl * loss
    p_surv = eirp_in_band_w(cfg) * surv_gain_lin(azimuth_deg(tx_enu), cfg) * fspl * loss
    return p_ref, p_surv


def echo_power_w(cfg: RadarConfig, tx_enu, pos, rcs_m2):
    lam = cfg.wavelength
    r1 = np.linalg.norm(np.asarray(pos) - tx_enu)
    r2 = np.linalg.norm(pos)
    g = surv_gain_lin(azimuth_deg(pos), cfg)
    loss = 10 ** (-cfg.echo_extra_loss_db / 10)
    return eirp_in_band_w(cfg) * g * lam ** 2 * rcs_m2 / ((4 * np.pi) ** 3 * r1 ** 2 * r2 ** 2) * loss


def processing_gain_db(cfg: RadarConfig):
    return 10 * np.log10(cfg.n_samples)


def dvbt_like_ofdm(n, fs, rng, nfft=2048, cp_frac=0.25, occupied=0.9):
    ncp = int(nfft * cp_frac)
    nsym = int(np.ceil(n / (nfft + ncp)))
    nocc = int(nfft * occupied) // 2 * 2
    idx = np.r_[1:nocc // 2 + 1, nfft - nocc // 2:nfft]
    lv = np.arange(-7, 8, 2) / np.sqrt(42)
    X = np.zeros((nsym, nfft), np.complex128)
    X[:, idx] = rng.choice(lv, (nsym, len(idx))) + 1j * rng.choice(lv, (nsym, len(idx)))
    x = np.fft.ifft(X, axis=1) * np.sqrt(nfft ** 2 / len(idx)) / np.sqrt(nfft)
    x = np.concatenate([x[:, -ncp:], x], axis=1).ravel()[:n]
    return x / np.sqrt(np.mean(np.abs(x) ** 2))


# AGC + 8-bit I/Q quantiser, like the RTL-SDR
def adc_rtlsdr_u8(z, backoff_db, rng=None):
    rms = np.sqrt(np.mean(np.abs(z) ** 2))
    z = z * (10 ** (-backoff_db / 20) / rms)
    iq = np.empty(2 * len(z), np.float64)
    iq[0::2], iq[1::2] = z.real, z.imag
    return np.clip(np.round(iq * 127.5 + 127.5), 0, 255).astype(np.uint8)


def simulate_capture(cfg: RadarConfig, tx_enu, scatterers, rng, sample_offset=0):
    n = cfg.n_samples + abs(sample_offset)
    fs = cfg.fs_hz
    x = dvbt_like_ofdm(n, fs, rng)
    X = sfft.fft(x, workers=-1)
    f = sfft.fftfreq(n, 1 / fs)
    t = np.arange(n) / fs
    lam = cfg.wavelength
    rx_enu = np.zeros(3)

    p_ref, p_surv = direct_powers_w(cfg, tx_enu)
    p_noise = noise_power_w(cfg)
    ref = np.sqrt(p_ref) * x
    surv = np.sqrt(p_surv) * x * np.exp(1j * rng.uniform(0, 2 * np.pi))

    truth = []
    pg = processing_gain_db(cfg)
    for s in scatterers:
        rb, fd = bistatic_range_doppler(tx_enu, rx_enu, s.pos, s.vel, lam)
        p = echo_power_w(cfg, tx_enu, s.pos, s.rcs_m2)
        echo = sfft.ifft(X * np.exp(-2j * np.pi * f * rb / C), workers=-1)
        surv += np.sqrt(p) * echo * np.exp(2j * np.pi * fd * t + 1j * rng.uniform(0, 2 * np.pi))
        truth.append(dict(name=s.name, kind=s.kind, rb_m=rb, fd_hz=fd, rcs_m2=s.rcs_m2,
                          echo_dbm=10 * np.log10(p) + 30,
                          snr_in_db=10 * np.log10(p / p_noise),
                          snr_out_thermal_db=10 * np.log10(p / p_noise) + pg,
                          dist_rx_m=float(np.linalg.norm(s.pos)),
                          az_deg=azimuth_deg(s.pos)))

    noise = lambda: np.sqrt(p_noise / 2) * (rng.standard_normal(n) + 1j * rng.standard_normal(n))
    ref += noise()
    surv += noise()

    if sample_offset > 0:
        surv = surv[sample_offset:]
        ref = ref[:cfg.n_samples]
    elif sample_offset < 0:
        ref = ref[-sample_offset:]
        surv = surv[:cfg.n_samples]

    budget = dict(surv_total_dbm=10 * np.log10(np.mean(np.abs(surv) ** 2)) + 30,
                  direct_ref_dbm=10 * np.log10(p_ref) + 30,
                  direct_surv_dbm=10 * np.log10(p_surv) + 30,
                  noise_dbm=10 * np.log10(p_noise) + 30,
                  processing_gain_db=pg)
    return (adc_rtlsdr_u8(ref, cfg.adc_backoff_db),
            adc_rtlsdr_u8(surv, cfg.adc_backoff_db), truth, budget)
