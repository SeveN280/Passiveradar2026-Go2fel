import numpy as np
from numpy.lib.stride_tricks import sliding_window_view
from scipy import fft as sfft
from scipy.ndimage import convolve, maximum_filter

from .geometry import C


def eca_b(ref, surv, n_taps, batch_len, neg_taps=4, dtype=np.complex64, loading=1e-9):
    n = len(surv)
    out = np.empty(n, np.complex64)
    K = n_taps + neg_taps
    refp = np.concatenate([np.zeros(n_taps - 1, ref.dtype), ref,
                           np.zeros(neg_taps, ref.dtype)]).astype(dtype)
# column k = reference delayed by k samples (static clutter model)
    windows = sliding_window_view(refp, K)
    for start in range(0, n, batch_len):
        stop = min(start + batch_len, n)
        X = windows[start:stop, ::-1]
        s = surv[start:stop].astype(dtype)
        Xh = X.conj().T
        R = (Xh @ X).astype(np.complex128)
        p = (Xh @ s).astype(np.complex128)
        R += loading * np.trace(R).real / K * np.eye(K)
        w = np.linalg.solve(R, p)
        out[start:stop] = s - X @ w.astype(dtype)
    return out


# Hann taper of the reference spectrum: suppresses range sidelobes
def taper_reference(ref, band_frac=0.9):
    n = len(ref)
    R = sfft.fft(ref, workers=-1)
    f = sfft.fftfreq(n)
    half = band_frac / 2
    w = np.where(np.abs(f) < half, 0.5 + 0.5 * np.cos(np.pi * f / half), 0.0)
    return sfft.ifft(R * w, workers=-1).astype(np.complex64)


def range_doppler_map(ref, surv, fs, n_range, doppler_max_hz):
    L = int(round(fs / (2 * doppler_max_hz)))
    nb = (len(surv) - n_range) // L
    N = nb * L
    rc = np.conj(ref[:N])
    acc = np.empty((nb, n_range), np.complex64)
    for d in range(n_range):
        acc[:, d] = (surv[d:d + N] * rc).reshape(nb, L).sum(axis=1)
    win = np.hanning(nb).astype(np.float32)
    rd = sfft.fftshift(sfft.fft(acc * win[:, None], axis=0, workers=-1), axes=0)
    dop = sfft.fftshift(sfft.fftfreq(nb, L / fs))
    rng = np.arange(n_range) * C / fs
    return rd, dop, rng


def ca_cfar(power, guard, train, pfa):
    gd, gr = guard
    td, tr = train
    kd, kr = gd + td, gr + tr
    kernel = np.ones((2 * kd + 1, 2 * kr + 1))
    kernel[td:td + 2 * gd + 1, tr:tr + 2 * gr + 1] = 0
    n_train = kernel.sum()
    noise = convolve(power, kernel / n_train, mode="wrap")
    alpha = n_train * (pfa ** (-1 / n_train) - 1)
    return power > alpha * noise, noise


def detect(rd, dop, rng, cfg):
    power = np.abs(rd) ** 2
    mask, noise = ca_cfar(power, cfg.cfar_guard, cfg.cfar_train, cfg.cfar_pfa)
    mask &= np.abs(dop)[:, None] > cfg.zero_doppler_notch_hz
    mask &= power == maximum_filter(power, size=(7, 5), mode="wrap")
    dets = []
    for i, j in zip(*np.nonzero(mask)):
        def interp(a, b, c):
            den = a - 2 * b + c
            return 0.0 if den == 0 else 0.5 * (a - c) / den
        lp = np.log(power + 1e-30)
        dj = interp(lp[i, j - 1], lp[i, j], lp[i, j + 1]) if 0 < j < len(rng) - 1 else 0.0
        di = interp(lp[i - 1, j], lp[i, j], lp[(i + 1) % len(dop), j])
        dets.append(dict(
            rb_m=float(rng[j] + dj * (rng[1] - rng[0])),
            fd_hz=float(dop[i] + di * (dop[1] - dop[0])),
            snr_db=float(10 * np.log10(power[i, j] / noise[i, j])),
            bin=(int(i), int(j)),
        ))
    return sorted(dets, key=lambda d: -d["snr_db"])


def to_db_map(rd):
    p = np.abs(rd) ** 2
    return 10 * np.log10(p / np.median(p) + 1e-12)
