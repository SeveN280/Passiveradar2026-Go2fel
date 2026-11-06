# ESP32 LoRa alert link

```
Raspberry Pi ──USB──> radar_node (ESP32 + LoRa) ~~~ LoRa 434.4 MHz ~~~> radar_gateway (ESP32 + LoRa + Wi-Fi) ──> private Telegram group
```

* **radar_node** – next to the Pi. Reads `PR,...` lines from `alert.py`, encrypts them and sends them by LoRa.
* **radar_gateway** – anywhere with Wi-Fi within LoRa range. Checks and decrypts packets and posts readable alerts to a private Telegram group (or a Discord channel).

## Security and radio rules

| | |
|---|---|
| Encryption | AES-128-GCM with a key only your two boards know. Outsiders can neither read alerts nor fake them: a changed or forged packet fails the 16-byte tag check. |
| Band | 434.4 MHz in the EU 433.05–434.79 MHz SRD band: licence-free, max 10 mW, 10 % duty cycle. (868 MHz boards: 869.525 MHz, 10 %.) |
| Duty cycle | The node pauses 9× each packet's airtime. One alert packet takes about 0.54 s at SF9, so at most one alert every ~5.4 s. |
| Priority | While waiting, the node keeps only the most important message: `DRONE?` > `UNKNOWN` > `AIRCRAFT` (aircraft are not sent by default). |
| Heartbeat | Every 5 min the node sends `HB,...`. If the gateway hears nothing for 15 min it posts a warning to the group. |
| Spam control | The same target reported again within 60 s is not posted twice. |

## What you need

* 2 × ESP32 + LoRa. Default (`BOARD_DEVKIT_433`): ESP32 DevKit with a **433 MHz SX1278 module** (e.g. Ra-02) wired like this:

  | Module | NSS | RST | SCK | MOSI | MISO | DIO0 |
  |---|---|---|---|---|---|---|
  | ESP32 GPIO | 2 | 4 | 16 | 17 | 5 | 19 |

  Power the module from **3.3 V**, never 5 V. Put a capacitor across the module's 3.3 V and GND pins, as close to the module as possible: the radio draws current in short bursts while transmitting, and the capacitor keeps the voltage stable (a common choice is 10–100 µF electrolytic plus 100 nF ceramic). Both boards must be on the same band: a 433 MHz board cannot talk to an 868 MHz one. Ready-made boards (TTGO LoRa32, Heltec V3) are selectable at the top of `radar_lora.h`.
* A 433 MHz antenna on each module (never transmit without an antenna).
* Notes on these pins: GPIO2 also drives the DevKit's blue LED (it will flicker with radio traffic; the code does not use it). GPIO16/17 are free on ESP32-WROOM boards but taken by PSRAM on WROVER boards. If flashing ever fails, unplug the module's NSS wire (GPIO2) while uploading.
* Arduino IDE 2 with the **esp32** board package (Espressif) and the **RadioLib** library (Library Manager).

## Setup

1. **Key.** Generate 16 random bytes:
   `python -c "import os;print(', '.join(f'0x{b:02x}' for b in os.urandom(16)))"`
2. **Secrets.** In *both* sketch folders copy `secrets.example.h` to `secrets.h` and paste the **same** key. `secrets.h` is in `.gitignore`.
3. **Private Telegram group.**
   1. In Telegram open **@BotFather**, send `/newbot`, follow the steps and copy the token → `TG_BOT_TOKEN`.
   2. Create a private group and add your bot to it.
   3. In the group send `/start@YourBotName`.
   4. Open `https://api.telegram.org/bot<TOKEN>/getUpdates` in a browser and find `"chat":{"id":-100...}` → `TG_CHAT_ID` (keep the minus sign).
   5. Put your Wi-Fi name and password in `WIFI_SSID` / `WIFI_PASS`.
4. **Flash** `radar_node` to one board and `radar_gateway` to the other (board: your ESP32 model, 115200 baud). The group receives *“Radar gateway online”*.
5. **Test without the radar:** open the node's Serial Monitor (115200, newline) and type
   `PR,1762113600,DRONE?,1402,38.5,30.7,-` → the node answers `OK queued`, and the alert appears in the group within seconds.
6. **Connect to the Pi:** plug the node into the Raspberry Pi and run
   `python process_capture.py ... --serial /dev/ttyUSB0` (or `/dev/ttyACM0`; check with `ls /dev/tty*`).

## Files

* `radar_node/radar_node.ino`, `radar_gateway/radar_gateway.ino` – the two programs
* `radar_lora.h` – shared radio settings, pins and the packet format (**keep both copies identical**)
* `lora_packet.py` – the same packet format in Python, with a self-test (`pip install cryptography`)

## Known limits

* Telegram's certificate is not checked (`setInsecure()`), to keep setup simple. For real use, load the root certificate with `setCACert()`.
* No replay protection: a recorded packet could be re-sent later. A message counter would fix this.
* Not yet tested on hardware; the packet format is tested in `lora_packet.py`.
