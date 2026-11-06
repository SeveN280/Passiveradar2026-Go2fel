#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "mbedtls/gcm.h"
#include "secrets.h"

#define BOARD_DEVKIT_433

#if defined(BOARD_DEVKIT_433)
  #define LORA_NSS   2  // GPIO2 is a boot strapping pin: unplug NSS if flashing fails
  #define LORA_RST   4
  #define LORA_SCK  16
  #define LORA_MOSI 17
  #define LORA_MISO  5
  #define LORA_DIO0 19
  #define LED_PIN   -1
  #define LORA_433MHZ
  static SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, RADIOLIB_NC);
#elif defined(BOARD_TTGO_LORA32)
  #define LORA_SCK   5
  #define LORA_MISO 19
  #define LORA_MOSI 27
  #define LORA_NSS  18
  #define LORA_RST  14
  #define LORA_DIO0 26
  #define LORA_DIO1 33
  #define LED_PIN   25
  static SX1276 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RST, LORA_DIO1);
#elif defined(BOARD_HELTEC_V3)
  #define LORA_SCK   9
  #define LORA_MISO 11
  #define LORA_MOSI 10
  #define LORA_NSS   8
  #define LORA_RST  12
  #define LORA_DIO1 14
  #define LORA_BUSY 13
  #define LED_PIN   35
  static SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
#else
  #error "Select a board at the top of radar_lora.h"
#endif

// EU 433.05-434.79 MHz: max 10 mW ERP, 10 % duty cycle
#if defined(LORA_433MHZ)
  #define LORA_FREQ_MHZ   434.4
  #define LORA_POWER_DBM  10
#else
// EU 869.4-869.65 MHz: 10 % duty cycle
  #define LORA_FREQ_MHZ   869.525
  #define LORA_POWER_DBM  14
#endif
#define LORA_BW_KHZ     125.0
#define LORA_SF         9
#define LORA_CR         7
#define LORA_SYNC       0x12
#define DUTY_CYCLE_PCT  10

inline void ledInit()        { if (LED_PIN >= 0) pinMode(LED_PIN, OUTPUT); }
inline void ledSet(bool on)  { if (LED_PIN >= 0) digitalWrite(LED_PIN, on ? HIGH : LOW); }

inline bool radioInit() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
#if defined(BOARD_HELTEC_V3)
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC,
                       LORA_POWER_DBM, 8, 1.8, false);  // Heltec V3: 1.8 V TCXO
#else
  int st = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR, LORA_SYNC,
                       LORA_POWER_DBM, 8, 0);
#endif
  if (st != RADIOLIB_ERR_NONE) {
    Serial.printf("# radio init failed, code %d\n", st);
    return false;
  }
  radio.setCRC(true);
  return true;
}

// packet: "RD" | version | 12-byte nonce | AES-128-GCM ciphertext | 16-byte tag
#define PKT_MAGIC0   'R'
#define PKT_MAGIC1   'D'
#define PKT_VERSION  1
#define PKT_HDR      3
#define PKT_NONCE    12
#define PKT_TAG      16
#define PKT_MAX_TEXT 96
#define PKT_MAX      (PKT_HDR + PKT_NONCE + PKT_MAX_TEXT + PKT_TAG)

inline size_t packetSeal(const char* text, uint8_t* pkt) {
  size_t n = strlen(text);
  if (n == 0 || n > PKT_MAX_TEXT) return 0;
  pkt[0] = PKT_MAGIC0; pkt[1] = PKT_MAGIC1; pkt[2] = PKT_VERSION;
  uint8_t* nonce = pkt + PKT_HDR;
  for (int i = 0; i < PKT_NONCE; i += 4) {
    uint32_t r = esp_random();
    memcpy(nonce + i, &r, 4);
  }
  uint8_t* ct  = nonce + PKT_NONCE;
  uint8_t* tag = ct + n;
  mbedtls_gcm_context g;
  mbedtls_gcm_init(&g);
  int rc = mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, RADAR_KEY, 128);
  if (rc == 0)
    rc = mbedtls_gcm_crypt_and_tag(&g, MBEDTLS_GCM_ENCRYPT, n, nonce, PKT_NONCE,
                                   pkt, PKT_HDR,
                                   (const uint8_t*)text, ct, PKT_TAG, tag);
  mbedtls_gcm_free(&g);
  return rc == 0 ? PKT_HDR + PKT_NONCE + n + PKT_TAG : 0;
}

inline bool packetOpen(const uint8_t* pkt, size_t len, char* text, size_t cap) {
  if (len < PKT_HDR + PKT_NONCE + 1 + PKT_TAG) return false;
  if (pkt[0] != PKT_MAGIC0 || pkt[1] != PKT_MAGIC1 || pkt[2] != PKT_VERSION) return false;
  size_t n = len - PKT_HDR - PKT_NONCE - PKT_TAG;
  if (n + 1 > cap) return false;
  const uint8_t* nonce = pkt + PKT_HDR;
  const uint8_t* ct    = nonce + PKT_NONCE;
  const uint8_t* tag   = ct + n;
  mbedtls_gcm_context g;
  mbedtls_gcm_init(&g);
  int rc = mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, RADAR_KEY, 128);
  if (rc == 0)
    rc = mbedtls_gcm_auth_decrypt(&g, n, nonce, PKT_NONCE, pkt, PKT_HDR,
                                  tag, PKT_TAG, ct, (uint8_t*)text);
  mbedtls_gcm_free(&g);
  if (rc != 0) return false;
  text[n] = '\0';
  return true;
}
