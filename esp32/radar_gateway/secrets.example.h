// narazie zostawic
#pragma once
#include <stdint.h>


static const uint8_t RADAR_KEY[16] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};


#define WIFI_SSID     "your-wifi"
#define WIFI_PASS     "your-wifi-password"


#define TG_BOT_TOKEN  "123456789:AA..."
#define TG_CHAT_ID    "-100..."

// Discord alternative (USE_TELEGRAM false in radar_gateway.ino):
#define DISCORD_WEBHOOK_URL "https://discord.com/api/webhooks/..."
