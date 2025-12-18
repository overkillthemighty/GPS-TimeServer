#pragma once // <--- CRITICAL: Prevents redefinition errors

// ESP32TimeServerKeySettings.h
// MIT License — 2025
// Project key settings (pins, speeds, toggles). Keep this file device-specific.

// versioning
const char* FIRMWARE_VERSION = "0.5";

// ---------- Serial / Debug ----------
const int  SerialMonitorSpeed = 115200;
const bool debugIsOn          = true;      // set false for production

// ---------- GPS (UART1) ----------
const int RXPin   = 17;              // GPS TX -> ESP32 RXPin
const int TXPin   = 18;              // GPS RX -> ESP32 TXPin
const int PPSPin  = 2;               // PPS input pin
const uint32_t GPSBaud = 9600;       // NEO-6M default baud

// ---------- Ethernet (W5500 via SPI) ----------
#define ETH_PHY_TYPE  ETH_PHY_W5500
#define ETH_PHY_ADDR  1
#define ETH_PHY_CS    14
#define ETH_PHY_IRQ   10
#define ETH_PHY_RST    9
#define ETH_SPI_SCK   13
#define ETH_SPI_MISO  12
#define ETH_SPI_MOSI  11

// Hostname (ETH)
const char* ETH_HOSTNAME = "GPS-NTP";

// ---------- Network Configuration (Defaults) ----------
const bool default_ForceStatic = false; // If false, tries DHCP first, then falls back to Static
// Default Static / Fallback Settings
const uint8_t default_ip[4]   = { 192, 168, 1, 200 };
const uint8_t default_gw[4]   = { 192, 168, 1, 1 };
const uint8_t default_sn[4]   = { 255, 255, 255, 0 };
const uint8_t default_dns[4]  = { 8, 8, 8, 8 };

// ---------- NTP ----------
const uint16_t NTP_PORT = 123;

// ---------- Time refresh / guard rails ----------
const unsigned long periodicTimeRefreshPeriodMs = 15UL * 1000UL; // refresh from GPS every 15s
const double safeguardThresholdSeconds         = 0.20;           // +/- 200ms sanity window
const long RTCLatencyCorrectionMs = 0;

// ---------- Web ----------
const uint16_t WEB_PORT               = 80;
const size_t   LOG_RING_CAPACITY      = 200;     // last N log entries served via web
const bool     webEnabled             = true;    // can disable if you want ultra-lean
const bool     otaEnabled             = true;    // HTTP upload firmware via /update