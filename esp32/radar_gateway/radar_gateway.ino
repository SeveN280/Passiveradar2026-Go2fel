#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <time.h>
#include "radar_lora.h"

#define USE_TELEGRAM      true
#define SILENCE_ALARM_MIN 15
#define REPEAT_SUPPRESS_S 60
#define WAVELENGTH_M      0.6325

static volatile bool packetFlag = false;
static uint32_t lastHeardMs = 0;
static bool     silenceReported = false;
static uint32_t rejected = 0;
static float    lastAlertRb = -1e9;
static uint32_t lastAlertMs = 0;

static void IRAM_ATTR onPacket() { packetFlag = true; }

static String jsonEscape(const String& s) {
  String o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if ((uint8_t)c >= 0x20) o += c;
  }
  return o;
}

static bool postToGroup(const String& text) {
  if (WiFi.status() != WL_CONNECTED) return false;
  WiFiClientSecure tls;
// server certificate not checked; use setCACert() for real deployments
  tls.setInsecure();
  HTTPClient http;
  String url, body;
#if USE_TELEGRAM
  url  = String("https://api.telegram.org/bot") + TG_BOT_TOKEN + "/sendMessage";
  body = String("{\"chat_id\":\"") + TG_CHAT_ID + "\",\"text\":\"" + jsonEscape(text) + "\"}";
#else
  url  = DISCORD_WEBHOOK_URL;
  body = String("{\"content\":\"") + jsonEscape(text) + "\"}";
#endif
  if (!http.begin(tls, url)) return false;
  http.addHeader("Content-Type", "application/json");
  int code = http.POST(body);
  http.end();
  Serial.printf("# posted, HTTP %d\n", code);
  return code >= 200 && code < 300;
}

static String formatAlert(char* line, float rssi, float lsnr, bool* isNew) {
  char* f[7] = {0};
  int n = 0;
  for (char* tok = strtok(line, ","); tok && n < 7; tok = strtok(nullptr, ",")) f[n++] = tok;
  if (n < 7) return String("Radar message (unparsed): ") + line;

  time_t t = (time_t)atol(f[1]);
  struct tm tmv;
  gmtime_r(&t, &tmv);
  char when[32];
  strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S UTC", &tmv);

  float rb = atof(f[3]), fd = atof(f[4]);
  float vmin = fabs(fd) * WAVELENGTH_M / 2;

  *isNew = !(fabs(rb - lastAlertRb) < 300 && millis() - lastAlertMs < REPEAT_SUPPRESS_S * 1000UL);
  lastAlertRb = rb;
  lastAlertMs = millis();

  String head;
  if (!strcmp(f[2], "DRONE?"))        head = "ALERT: possible drone";
  else if (!strcmp(f[2], "AIRCRAFT")) head = String("Aircraft ") + f[6] + " (confirmed by ADS-B)";
  else                                head = "Unidentified moving target";

  String m = head + "\n";
  m += String("Bistatic range: ") + String(rb, 0) + " m\n";
  m += String("Doppler: ") + (fd >= 0 ? "+" : "") + String(fd, 1) + " Hz ("
     + (fd >= 0 ? "approaching" : "moving away") + ", at least " + String(vmin, 0) + " m/s)\n";
  m += String("Radar SNR: ") + f[5] + " dB\n";
  m += String("Time: ") + when + "\n";
  m += String("LoRa link: RSSI ") + String(rssi, 0) + " dBm, SNR " + String(lsnr, 1) + " dB";
  return m;
}

static void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("# Wi-Fi");
  for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(500); Serial.print("."); }
  Serial.println(WiFi.status() == WL_CONNECTED ? " connected" : " FAILED (will retry)");
}

void setup() {
  Serial.begin(115200);
  ledInit();
  delay(500);
  Serial.println("# radar_gateway starting");
  connectWiFi();
  while (!radioInit()) delay(5000);
  radio.setPacketReceivedAction(onPacket);
  radio.startReceive();
  lastHeardMs = millis();
  postToGroup("Radar gateway online, listening on LoRa.");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 30000) { lastTry = millis(); connectWiFi(); }
  }

  if (packetFlag) {
    packetFlag = false;
    uint8_t pkt[PKT_MAX + 8];
    size_t len = radio.getPacketLength();
    int st = (len > 0 && len <= sizeof(pkt)) ? radio.readData(pkt, len) : RADIOLIB_ERR_PACKET_TOO_LONG;
    float rssi = radio.getRSSI(), lsnr = radio.getSNR();
    radio.startReceive();

    char text[PKT_MAX_TEXT + 1];
    if (st != RADIOLIB_ERR_NONE || !packetOpen(pkt, len, text, sizeof(text))) {
      rejected++;
      Serial.printf("# rejected packet (%d total): not ours or damaged\n", (int)rejected);
      return;
    }
    ledSet(true);
    lastHeardMs = millis();
    Serial.printf("# got: %s  (RSSI %.0f dBm)\n", text, rssi);

    if (silenceReported) {
      postToGroup("Radar node is back online.");
      silenceReported = false;
    }
    if (!strncmp(text, "PR,", 3)) {
      bool isNew = true;
      String msg = formatAlert(text, rssi, lsnr, &isNew);
      if (isNew) postToGroup(msg);
      else Serial.println("# repeat of the same target, not posted");
    }
    ledSet(false);
  }

  if (!silenceReported && millis() - lastHeardMs > SILENCE_ALARM_MIN * 60000UL) {
    postToGroup(String("Warning: no signal from the radar node for ") + SILENCE_ALARM_MIN + " minutes.");
    silenceReported = true;
  }
}
