#include "radar_lora.h"

#define SEND_AIRCRAFT  false
#define HEARTBEAT_S    300

static char     lineBuf[PKT_MAX_TEXT + 2];
static size_t   lineLen = 0;
static char     pending[PKT_MAX_TEXT + 1];
static int      pendingPrio = -1;
static uint32_t nextTxMs = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t sentCount = 0, droppedCount = 0;

static int labelPriority(const char* line) {
  const char* p = strchr(line, ',');
  if (!p) return -1;
  p = strchr(p + 1, ',');
  if (!p) return -1;
  p++;
  if (!strncmp(p, "DRONE?", 6))   return 2;
  if (!strncmp(p, "UNKNOWN", 7))  return 1;
  if (!strncmp(p, "AIRCRAFT", 8)) return SEND_AIRCRAFT ? 0 : -1;
  return 1;
}

static void queueMessage(const char* text, int prio) {
  if (pendingPrio >= 0 && prio < pendingPrio) {
    droppedCount++;
    Serial.println("OK dropped (lower priority)");
    return;
  }
  if (pendingPrio >= 0) droppedCount++;
  strncpy(pending, text, PKT_MAX_TEXT);
  pending[PKT_MAX_TEXT] = '\0';
  pendingPrio = prio;
  Serial.println("OK queued");
}

static void handleLine(char* line) {
  if (strncmp(line, "PR,", 3) != 0) {
    Serial.println("ERR expected PR,... line");
    return;
  }
  int prio = labelPriority(line);
  if (prio < 0) { Serial.println("OK ignored"); return; }
  queueMessage(line, prio);
}

static void transmitPending() {
  uint8_t pkt[PKT_MAX];
  size_t len = packetSeal(pending, pkt);
  if (len == 0) { Serial.println("# seal failed"); pendingPrio = -1; return; }

  ledSet(true);
  int st = radio.transmit(pkt, len);
  ledSet(false);

  uint32_t airMs = radio.getTimeOnAir(len) / 1000 + 1;
// stay silent for 9x the airtime: 10 % duty cycle
  nextTxMs = millis() + airMs * (100 / DUTY_CYCLE_PCT - 1);
  if (st == RADIOLIB_ERR_NONE) {
    sentCount++;
    Serial.printf("# sent %u bytes, airtime %lu ms: %s\n", (unsigned)len,
                  (unsigned long)airMs, pending);
  } else {
    Serial.printf("# transmit failed, code %d\n", st);
  }
  pendingPrio = -1;
}

void setup() {
  Serial.begin(115200);
  ledInit();
  delay(500);
  Serial.println("# radar_node starting");
  while (!radioInit()) delay(5000);
  Serial.printf("# LoRa %.3f MHz SF%d BW%.0f kHz, ready\n", LORA_FREQ_MHZ, LORA_SF, LORA_BW_KHZ);
  lastHeartbeatMs = millis() - (HEARTBEAT_S - 10) * 1000UL;
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0) handleLine(lineBuf);
      lineLen = 0;
    } else if (lineLen < sizeof(lineBuf) - 1) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0;
      Serial.println("ERR line too long");
    }
  }

  if (millis() - lastHeartbeatMs >= HEARTBEAT_S * 1000UL && pendingPrio < 0) {
    snprintf(pending, sizeof(pending), "HB,%lu,%lu,%lu", (unsigned long)(millis() / 1000),
             (unsigned long)sentCount, (unsigned long)droppedCount);
    pendingPrio = 0;
    lastHeartbeatMs = millis();
  }

  if (pendingPrio >= 0 && (int32_t)(millis() - nextTxMs) >= 0) transmitPending();
}
