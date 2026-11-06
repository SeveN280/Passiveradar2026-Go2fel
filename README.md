# Passive anti-drone radar (DVB-T, 474 MHz, go2fel Praha)

A passive bistatic radar that uses the TV transmitter **RTCN Kraków/Chorągwica,
MUX4, channel 21 (474 MHz, 100 kW ERP)** as its "illuminator". It transmits
nothing: it listens to the TV signal directly (reference antenna) and to its
echoes from moving objects (surveillance antenna). ADS-B aircraft data is used as
ground truth, and alerts go to an ESP32 + LoRa node.

## Hardware

| Part | Role |
|---|---|
| Home-made 3-element Yagi-Uda, SWR 1.6 at 474 MHz, **horizontal** polarisation | surveillance antenna, pointed away from the transmitter |
| ATK 5 | reference antenna, pointed at Chorągwica |
| 2-channel coherent SDR (2× RTL-SDR on a shared clock, or KrakenSDR) | *planned*: synchronous sampling of both antennas |
| RTL-SDR + 1090 MHz antenna | ADS-B receiver (readsb/dump1090) |
| Raspberry Pi | signal processing |
| ESP32 + LoRa, 3D-printed case | remote alert node |

## Processing chain

1. **Align channels** – the direct signal is found in both channels by cross-correlation
   (the strong peak with the largest lag, since the direct path always arrives first).
2. **Clutter cancellation (ECA-B)** – least-squares removal of the direct signal and
   all static echoes (buildings, terrain), re-estimated every 0.125 s.
3. **Cross-ambiguity function** – correlates the surveillance signal with delayed,
   Doppler-shifted copies of the reference → range-Doppler map. The reference is
   spectrally tapered (Hann) to suppress range sidelobes.
4. **2-D CA-CFAR** detection, P_fa = 10⁻⁶, sub-bin peak interpolation.
5. **ADS-B matching** – each aircraft's position/velocity is converted to the expected
   bistatic range and Doppler; detections that match are aircraft, moving detections
   without ADS-B are drone candidates.
6. **Alert** – one line per detection over serial to the ESP32, which sends it by LoRa:
   `PR,<unix_time>,<label>,<rb_m>,<fd_hz>,<snr_db>,<adsb>`.

## Key numbers

| Quantity | Value |
|---|---|
| Wavelength | 0.632 m |
| TX–RX baseline (RX = Paszkówka) | 29 km |
| Range bin (2.4 MS/s) | 125 m bistatic |
| Integration time / Doppler resolution | 0.5 s / 2 Hz |
| Processing gain | 60.8 dB |
| Max. Doppler of a 15 m/s drone | ±47 Hz |
| Predicted drone range, RCS 0.01 m² (calibrated, −8 dB margin) | ≈ 1 km |
| Predicted drone range, RCS 0.001 m² | ≈ 0.3 km |

## Simulation result (`python run_demo.py`)

| Target | True Rb / fD | Detected Rb / fD | SNR |
|---|---|---|---|
| Drone, 0.01 m², 700 m, 12 m/s | 1403 m / 37.8 Hz | 1401 m / 37.8 Hz | 23.8 dB |
| Aircraft LOT3921 (ADS-B) | 12115 m / −349.4 Hz | 12116 m / −349.4 Hz | 34.9 dB |

No false alarms. Processing a 0.5 s block takes ≈ 3.5 s on a cloud CPU (most of
it the clutter filter), so expect it to be several times slower on a Raspberry Pi.
Real-time operation will need optimisation.

## Known limitations (honest list)

- **A hovering drone has zero Doppler** and is removed together with the static clutter;
  only rotor micro-Doppler could reveal it (future work).
- One antenna gives only a coarse bearing (the Yagi beam, ~50°); the position is
  "right distance, uncertain angle". Two surveillance antennas would give a direction.
- Cars, birds and swaying trees also move: the simple speed rule cannot fully
  separate them from drones.
- The 8-bit RTL-SDR, not thermal noise, limits sensitivity: its quantisation noise is
  set by the strong direct signal.
- The simulation is idealised (flat terrain, constant RCS, no multipath of the target).

## Files

```
passive_radar/   config.py  geometry.py  simulate.py  iq.py  processing.py  adsb.py  alert.py
run_demo.py      full simulated demonstration, writes figures to out/
process_capture.py   same chain on real recordings (+ ADS-B URL, + serial to ESP32)
esp32/           LoRa alert link: radar_node (serial → encrypted LoRa) and
                 radar_gateway (LoRa → private Telegram group); 
```

Requirements: Python 3.9+, `numpy`, `scipy`, `matplotlib` (`pyserial` for the ESP32 link).
Receiver site (Paszkówka), Yagi direction (266°, west) and the antenna values (Yagi 7 dBi / F/B 13 dB, ATK-5 6 dBi) are set in `passive_radar/config.py`.
