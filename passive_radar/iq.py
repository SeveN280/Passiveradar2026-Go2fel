import numpy as np
from scipy import fft as sfft


def load_rtlsdr_u8(path_or_array, max_samples=None):
    raw = path_or_array if isinstance(path_or_array, np.ndarray) else np.fromfile(path_or_array, np.uint8)
    if max_samples:
        raw = raw[:2 * max_samples]
    raw = raw[: len(raw) // 2 * 2].astype(np.float32)
    z = (raw[0::2] - 127.5) + 1j * (raw[1::2] - 127.5)
    z /= 127.5
    return z.astype(np.complex64)


def load_cf32(path, max_samples=None):
    z = np.fromfile(path, np.complex64)
    return z[:max_samples] if max_samples else z


def estimate_lag(ref, surv, search=5000, n=2 ** 18):
    a = ref[:n] - ref[:n].mean()
    b = surv[:n] - surv[:n].mean()
    m = sfft.next_fast_len(2 * n)
    xc = sfft.ifft(sfft.fft(a, m) * np.conj(sfft.fft(b, m)))
    lags = np.r_[-search:search + 1]
    vals = np.abs(np.r_[xc[-search:], xc[:search + 1]])
    i_max = int(np.argmax(vals))
    strong = vals > vals[i_max] * 10 ** (-12 / 20)
    local = np.r_[False, (vals[1:-1] >= vals[:-2]) & (vals[1:-1] >= vals[2:]), False]
    window = (lags >= lags[i_max]) & (lags <= lags[i_max] + 128)
# the direct signal arrives first: strong peak with the largest lag
    cand = np.nonzero(strong & local & window)[0]
    return int(lags[cand.max()]) if len(cand) else int(lags[i_max])


def align(ref, surv, lag):
    if lag > 0:
        ref = ref[lag:]
    elif lag < 0:
        surv = surv[-lag:]
    n = min(len(ref), len(surv))
    return ref[:n], surv[:n]
