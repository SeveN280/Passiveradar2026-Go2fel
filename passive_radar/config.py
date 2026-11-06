from dataclasses import dataclass, field

from .geometry import C, Site

# Choragwica, MUX4 (channel 21, 474 MHz, 100 kW ERP)
TX_SITE = Site(lat=49.95862, lon=20.08194, alt=420 + 250)

# Paszkowka
RX_SITE = Site(lat=49.9383, lon=19.6783, alt=280 + 10)


@dataclass
class RadarConfig:
    fc_hz: float = 474e6
    erp_w: float = 100e3
    channel_bw_hz: float = 7.61e6

    fs_hz: float = 2.4e6
    usable_bw_frac: float = 0.9
    noise_figure_db: float = 6.0
    ref_gain_dbi: float = 6.0

    surv_gain_dbi: float = 7.0
    surv_beamwidth_deg: float = 60.0
    surv_backlobe_db: float = -13.0
    surv_azimuth_deg: float = 266.0  # back of the Yagi faces the transmitter

    direct_extra_loss_db: float = 15.0
    echo_extra_loss_db: float = 6.0

    cpi_s: float = 0.5
    n_range_bins: int = 120
    doppler_max_hz: float = 500.0
    eca_taps: int = 64
    eca_batch_s: float = 0.125
    cfar_pfa: float = 1e-6
    cfar_guard: tuple = (3, 2)
    cfar_train: tuple = (10, 4)
    zero_doppler_notch_hz: float = 4.0
    adc_backoff_db: float = 12.0

    extra: dict = field(default_factory=dict)

    @property
    def wavelength(self):
        return C / self.fc_hz

    @property
    def range_bin_m(self):
        return C / self.fs_hz

    @property
    def n_samples(self):
        return int(round(self.cpi_s * self.fs_hz))

    @property
    def captured_fraction(self):
        return min(1.0, self.fs_hz * self.usable_bw_frac / self.channel_bw_hz)
