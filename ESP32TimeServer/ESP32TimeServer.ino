/*
  ESP32TimeServer.ino
  MIT License — 2025

  Headless ESP32-S3 + W5500 NTP Stratum-1 server (UTC-only) using GPS PPS.
  - ETH-only (Wi-Fi and BT explicitly disabled)
  - Static IP or DHCP support
  - Serial Menu for Network Configuration (Parity with Web UI)
*/

#include <ETH.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Time.h>
#include <TinyGPSPlus.h>
#include <TimeLib.h>
#include <WebServer.h>     
#include <Update.h>        
#include <Preferences.h>
#include "ESP32TimeServerKeySettings.h"

// ---------- Ethernet pins ----------
bool eth_connected = false;
bool eth_got_IP    = false;
String ip          = "";

// ---------- Preferences & Config ----------
Preferences prefs;
bool useStaticIP_Run = false;
IPAddress ip_Run, gw_Run, sn_Run, dns_Run;

// ---------- Timekeeping ----------
ESP32Time rtc(0);  
TinyGPSPlus gps;
HardwareSerial GPSDevice(1);

// PPS signalling
volatile bool ppsFlag = false;
portMUX_TYPE ppsMux = portMUX_INITIALIZER_UNLOCKED;

// ---------- NTP ----------
#define NTP_PACKET_SIZE 48
WiFiUDP Udp;
byte packetBuffer[NTP_PACKET_SIZE];
volatile uint64_t ntpServedCount = 0;

// ---------- Guard rails / sync ----------
volatile bool rtcTimeInitialized = false;
volatile bool SafeGuardTripped   = false;

// freeRTOS
SemaphoreHandle_t mutex;
TaskHandle_t gpsTaskHandle = NULL;
int gpsCoreID = -1;

// ---------- Web Server Instance ----------
WebServer server(WEB_PORT);
String logRing[LOG_RING_CAPACITY];
size_t logHead = 0;
size_t logSize = 0;

// Jitter/offset stats
double lastUpdateDeltaSec = 0.0;
double ewmaOffsetSec      = 0.0;
double maxAbsOffsetSec    = 0.0;

// ---------- Constants ----------
const unsigned long oneSecond_inMs  = 1000UL;
const long          oneSecond_inUsL = 1000000L;
const double        oneSecond_inUsD = 1000000.0;

// ---------- Helper (Used by Core, Web, and Serial) ----------
void logMsg(const String& s) {
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    if (debugIsOn) Serial.println(s);
    logRing[logHead] = s;
    logHead = (logHead + 1) % LOG_RING_CAPACITY;
    if (logSize < LOG_RING_CAPACITY) logSize++;
    xSemaphoreGive(mutex);
  }
}

void GetUTC_DateAndTimeStrings(time_t UTC_Time, String &dateString, String &timeString) {
  dateString = String(year(UTC_Time));
  if (month(UTC_Time) < 10) dateString += "-0"; else dateString += "-";
  dateString += String(month(UTC_Time));
  if (day(UTC_Time) < 10) dateString += "-0"; else dateString += "-";
  dateString += String(day(UTC_Time));
  
  timeString = (hour(UTC_Time) < 10 ? "0" : "") + String(hour(UTC_Time));
  timeString += ":";
  timeString += (minute(UTC_Time) < 10 ? "0" : "") + String(minute(UTC_Time));
  timeString += ":";
  timeString += (second(UTC_Time) < 10 ? "0" : "") + String(second(UTC_Time));
}

// *** INCLUDE WEB HANDLER AFTER DEFINING GLOBALS ***
#include "WebServerHandler.h"

// ---------- Serial Menu Logic ----------
void processSerialMenu() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') return;

    Serial.println("\n--- Serial Command: " + String(cmd) + " ---");

    switch (cmd) {
      case 'h':
      case '?':
        Serial.println("Available Commands:");
        Serial.println(" [s] Status  - Show current time & GPS stats");
        Serial.println(" [i] Info    - Show network & system info");
        Serial.println(" [n] Network - Configure IP settings (Wizard)");
        Serial.println(" [l] Logs    - Dump the system log buffer");
        Serial.println(" [r] Reboot  - Restart device");
        Serial.println(" [w] Wipe    - Reset to factory defaults");
        break;

      case 'n': {
        Serial.println("--- Network Configuration Wizard ---");
        Serial.setTimeout(15000); 

        Serial.print("Enter Static IP (e.g. 192.168.1.200): ");
        String s_ip = Serial.readStringUntil('\n'); s_ip.trim();
        Serial.println(s_ip);

        Serial.print("Enter Gateway (e.g. 192.168.1.1): ");
        String s_gw = Serial.readStringUntil('\n'); s_gw.trim();
        Serial.println(s_gw);

        Serial.print("Enter Subnet (e.g. 255.255.255.0): ");
        String s_sn = Serial.readStringUntil('\n'); s_sn.trim();
        Serial.println(s_sn);

        Serial.print("Force Static IP? (y/n): ");
        String s_st = Serial.readStringUntil('\n'); s_st.trim();
        Serial.println(s_st);

        IPAddress nIP, nGW, nSN;
        if (nIP.fromString(s_ip) && nGW.fromString(s_gw) && nSN.fromString(s_sn)) {
          prefs.begin("net_config", false);
          prefs.putBool("static", (s_st == "y" || s_st == "Y"));
          prefs.putUInt("ip", (uint32_t)nIP);
          prefs.putUInt("gw", (uint32_t)nGW);
          prefs.putUInt("sn", (uint32_t)nSN);
          prefs.putUInt("dns", (uint32_t)IPAddress(8,8,8,8)); 
          prefs.end();
          Serial.println("Saved. Rebooting...");
          delay(1000);
          ESP.restart();
        } else {
          Serial.println("ERROR: Invalid IP format. Aborting.");
        }
        Serial.setTimeout(1000);
        break;
      }

      case 's': {
        String d, t;
        GetUTC_DateAndTimeStrings(rtc.getEpoch(), d, t);
        Serial.printf("Time: %s %s UTC | Locked: %s\n", d.c_str(), t.c_str(), rtcTimeInitialized ? "YES" : "NO");
        Serial.printf("Sats: %d | HDOP: %.2f | Guard: %s\n", gps.satellites.value(), gps.hdop.value() / 100.0, SafeGuardTripped ? "TRIPPED" : "OK");
        break;
      }

      case 'i':
        Serial.println("IP: " + ip + " (" + (useStaticIP_Run ? "Static" : "DHCP") + ")");
        Serial.printf("Uptime: %lu s | NTP Served: %llu\n", millis() / 1000UL, ntpServedCount);
        Serial.printf("Cores: GPS:%d / Sys:%d\n", gpsCoreID, xPortGetCoreID());
        break;

      case 'l':
        if (xSemaphoreTake(mutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
          size_t idx = (logSize == LOG_RING_CAPACITY) ? logHead : 0;
          for (size_t i = 0; i < logSize; ++i) {
            Serial.println(logRing[idx]);
            idx = (idx + 1) % LOG_RING_CAPACITY;
          }
          xSemaphoreGive(mutex);
        }
        break;

      case 'r': ESP.restart(); break;
      case 'w':
        prefs.begin("net_config", false); prefs.clear(); prefs.end();
        Serial.println("Settings wiped. Rebooting...");
        delay(500); ESP.restart();
        break;
    }
    Serial.println("--------------------------");
  }
}

// ---------- Core Logic ----------
void turnOffWifiAndBluetooth() {
  WiFi.mode(WIFI_OFF);
  btStop();
}

void loadNetworkSettings() {
  prefs.begin("net_config", true);
  useStaticIP_Run = prefs.getBool("static", default_ForceStatic);
  auto loadIP = [&](const char* key, const uint8_t* def) -> IPAddress {
    uint32_t val = prefs.getUInt(key, 0);
    if (val == 0) return IPAddress(def);
    return IPAddress(val);
  };
  ip_Run  = loadIP("ip", default_ip);
  gw_Run  = loadIP("gw", default_gw);
  sn_Run  = loadIP("sn", default_sn);
  dns_Run = loadIP("dns", default_dns);
  prefs.end();
}

void setupSerial() {
  Serial.begin(SerialMonitorSpeed);
  if (debugIsOn) {
    delay(300);
    Serial.println();
    Serial.println("Type h for a list of commands.");
    Serial.println("ESP32 Time Server starting setup ...");
  }
}

void setupGPS() {
  GPSDevice.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  delay(100);
  logMsg("GPS UART initialized at " + String(GPSBaud) + " baud.");
}

void IRAM_ATTR ppsHandlerRising() {
  portENTER_CRITICAL_ISR(&ppsMux);
  ppsFlag = true;
  portEXIT_CRITICAL_ISR(&ppsMux);
}

static inline bool gpsDataValidNow() {
  return gps.date.isValid() && gps.time.isValid()
      && gps.location.isValid() && gps.location.age() < 1000
      && gps.satellites.isValid() && gps.satellites.value() >= 4;
}

void setDateAndTimeFromGPS(void* parameter) {
  const double guardLow  = -safeguardThresholdSeconds;
  const double guardHigh =  safeguardThresholdSeconds;
  bool firstSet = true;
  int consecutiveFailures = 0; 
  gpsCoreID = xPortGetCoreID();

  for (;;) {
    while (GPSDevice.available() > 0) { GPSDevice.read(); }
    while (!gpsDataValidNow()) {
      while (GPSDevice.available() > 0) { gps.encode(GPSDevice.read()); }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    ppsFlag = false;
    unsigned long ppsStart = millis();
    while (millis() - ppsStart < 1100) {
      if (ppsFlag) break;
      vTaskDelay(1);
    }

    struct tm wt;
    wt.tm_year = gps.date.year();
    wt.tm_mon  = gps.date.month();
    wt.tm_mday = gps.date.day();
    wt.tm_hour = gps.time.hour();
    wt.tm_min  = gps.time.minute();
    wt.tm_sec  = gps.time.second();

    wt.tm_year -= 1900;
    wt.tm_mon  -= 1;
    time_t candidate = mktime(&wt);
    candidate += 1;

    unsigned long pegStartUs = micros();
    bool okToUpdate = true;
    double updateDelta = 0;

    if (!firstSet) {
      time_t currentRTC_t = rtc.getEpoch();
      updateDelta = (double)currentRTC_t - (double)candidate;
      okToUpdate = (updateDelta >= guardLow && updateDelta <= guardHigh);
    }

    if (okToUpdate) {
      if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        unsigned long pegEndUs = micros();
        long finalMicros = (long)(pegEndUs - pegStartUs) + (RTCLatencyCorrectionMs * 1000L);
        rtc.setTime((uint32_t)candidate, (int)finalMicros);
        lastUpdateDeltaSec = updateDelta;
        ewmaOffsetSec = (firstSet ? 0.0 : (0.2 * fabs(updateDelta) + 0.8 * ewmaOffsetSec));
        xSemaphoreGive(mutex);
      }
      SafeGuardTripped = false;
      firstSet = false;
      rtcTimeInitialized = true;
      vTaskDelay(periodicTimeRefreshPeriodMs / portTICK_PERIOD_MS);
    } else {
      SafeGuardTripped = true;
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void EthEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START: ETH.setHostname(ETH_HOSTNAME); break;
    case ARDUINO_EVENT_ETH_CONNECTED: eth_connected = true; break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_got_IP = true; ip = ETH.localIP().toString();
      logMsg("ETH IP: " + ip); break;
    case ARDUINO_EVENT_ETH_DISCONNECTED: eth_connected = false; eth_got_IP = false; break;
    default: break;
  }
}

bool setupEitherNetWithRetry() {
  SPI.begin(ETH_SPI_SCK, ETH_SPI_MISO, ETH_SPI_MOSI);
  WiFi.onEvent(EthEvent);
  pinMode(ETH_PHY_RST, OUTPUT);
  digitalWrite(ETH_PHY_RST, LOW); delay(10);
  digitalWrite(ETH_PHY_RST, HIGH); delay(10);

  ETH.begin(ETH_PHY_TYPE, ETH_PHY_ADDR, ETH_PHY_CS, ETH_PHY_IRQ, ETH_PHY_RST, SPI);
  if (useStaticIP_Run) ETH.config(ip_Run, gw_Run, sn_Run, dns_Run);
  
  unsigned long t0 = millis();
  while (!eth_got_IP && (millis() - t0) < 10000UL) delay(10);

  if (!eth_got_IP && !useStaticIP_Run) {
    ETH.config(ip_Run, gw_Run, sn_Run, dns_Run);
    t0 = millis();
    while (!eth_got_IP && (millis() - t0) < 8000UL) delay(10);
  }
  return eth_got_IP;
}

uint64_t getCurrentTimeInNTP64BitFormat() {
  const uint64_t seconds_1900_to_1970 = 2208988800ULL;
  uint64_t sec = seconds_1900_to_1970 + (uint64_t)rtc.getEpoch();
  uint32_t frac = (uint32_t)((double)rtc.getMicros() * 4294.967296);
  return (sec << 32) | frac;
}

void processNTPRequests() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;
  Udp.read(packetBuffer, NTP_PACKET_SIZE);
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    for (int i = 0; i < 8; ++i) packetBuffer[24 + i] = packetBuffer[40 + i];
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(packetBuffer, NTP_PACKET_SIZE);
    Udp.endPacket();
    ntpServedCount++;
    xSemaphoreGive(mutex);
  }
}

void setup() {
  setupSerial();
  mutex = xSemaphoreCreateMutex();
  loadNetworkSettings();
  turnOffWifiAndBluetooth();
  pinMode(PPSPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPSPin), ppsHandlerRising, RISING);
  setupGPS();
  xTaskCreatePinnedToCore(setDateAndTimeFromGPS, "GPS", 3072, NULL, 20, &gpsTaskHandle, 0);

  unsigned long t0 = millis();
  while (!rtcTimeInitialized && (millis() - t0) < 30000UL) delay(10);
  
  if (setupEitherNetWithRetry()) {
    Udp.begin(NTP_PORT);
    startWeb();
    Serial.println("\nSystem Ready. Type 'h' for menu.");
  }
}

void loop() {
  processNTPRequests();
  processWebLoop();
  processSerialMenu(); 
}