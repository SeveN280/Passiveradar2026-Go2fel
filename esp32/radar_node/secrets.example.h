// Copy this file to secrets.h (same folder) and put in your own values.
// secrets.h is in .gitignore: never publish your real key or tokens.
#pragma once
#include <stdint.h>

// 16 random bytes, IDENTICAL on the node and the gateway. Make your own:
//   python -c "import os;print(', '.join(f'0x{b:02x}' for b in os.urandom(16)))"
static const uint8_t RADAR_KEY[16] = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// ---- gateway only (the node ignores these) ----
#define WIFI_SSID     "your-wifi"
#define WIFI_PASS     "your-wifi-password"

// Telegram: token from @BotFather; chat id of your private group (negative number,
// e.g. -1001234567890). How to find it: see README, "Private Telegram group".
#define TG_BOT_TOKEN  "123456789:AA..."
#define TG_CHAT_ID    "-100..."

// Discord alternative (USE_TELEGRAM false in radar_gateway.ino):
#define DISCORD_WEBHOOK_URL "https://discord.com/api/webhooks/..."
