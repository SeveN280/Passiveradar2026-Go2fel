import time


def format_line(det, label):
    return "PR,{t},{lab},{rb:.0f},{fd:.1f},{snr:.1f},{ac}\n".format(
        t=int(time.time()), lab=label, rb=det["rb_m"], fd=det["fd_hz"],
        snr=det["snr_db"], ac=det.get("adsb") or "-")


class AlertLink:

    def __init__(self, port=None, baud=115200):
        self.ser = None
        if port:
            import serial
            self.ser = serial.Serial(port, baud, timeout=1)

    def send(self, line):
        if self.ser:
            self.ser.write(line.encode())
        else:
            print("ALERT>", line.strip())
