# ESP32 MasterClock (Stratum-1 NTP via GPS PPS)

A professional-grade **ESP32-S3 + W5500** Ethernet device serving **UTC-only** NTP time as a Stratum‑1 source, disciplined by **GPS PPS**. 

This project is optimized for the **Waveshare ESP32-S3-ETH** development board and leverages the ESP32's dual-core architecture to ensure microsecond-level timing precision.

---

## 💻 Arduino IDE Configuration

To ensure compatibility with the board's 16MB Flash and PSRAM, use these exact settings in the **Tools** menu:

* **Board:** `ESP32-S3 Dev Module`
* **USB CDC On Boot:** `Enabled` (Required for the Serial Menu to work)
* **Flash Size:** `16MB (128Mb)`
* **Partition Scheme:** `16M Flash (3MB APP/9.9MB FATFS)`
* **PSRAM:** `OPI PSRAM`
* **Upload Speed:** `921600`

---

## 🚀 Key Features

### ⏱️ High-Precision Timing
- **Stratum-1 Source:** Synchronizes directly to GPS atomic clocks.
- **Dual-Core Execution:** - **Core 0:** Dedicated strictly to GPS NMEA parsing and high-priority PPS interrupt handling.
    - **Core 1:** Handles the Ethernet stack, NTP UDP requests, and the Web/Serial interfaces.
- **PPS Discipline:** RTC is corrected microsecond-by-microsecond upon every GPS pulse.

### 🌐 Networking & Reliability
- **Ethernet Only:** Wi-Fi and Bluetooth are disabled to eliminate radio-induced jitter and interrupt latency.
- **Smart Fallback:** Attempts DHCP on boot; if unsuccessful (10s timeout), falls back to the saved Static IP.
- **Serial Configuration Menu:** Full parity with the web interface for management via USB Serial.

---

## 🛠️ Hardware Setup

### Waveshare ESP32-S3-ETH Pinout
The default configuration in `ESP32TimeServerKeySettings.h` is mapped as follows:

| Component | Pin | Function |
| :--- | :--- | :--- |
| **GPS TX** | GPIO 17 | UART1 RX |
| **GPS RX** | GPIO 18 | UART1 TX |
| **GPS PPS** | GPIO 2 | Interrupt Pin |
| **ETH CS** | GPIO 14 | SPI Chip Select |
| **ETH RST** | GPIO 9 | Hardware Reset |
| **ETH IRQ** | GPIO 10 | Interrupt |



---

## ⌨️ Serial Menu Usage

Connect to the device via the Serial Monitor at **115200 baud**. Press **'h'** to see the command list:

* **`[s] Status`**: View current UTC time, GPS lock status, and satellite count.
* **`[i] Info`**: View active IP address and system uptime.
* **`[n] Network`**: **Configuration Wizard**. Use this to set Static IP, Gateway, and Subnet settings.
* **`[l] Logs`**: Dump the system event log from the internal ring buffer.
* **`[w] Wipe`**: Reset all network settings to factory defaults.
* **`[r] Reboot`**: Software restart.

---

## 📡 Required Libraries

Install these via the Arduino Library Manager:
1. `TinyGPSPlus` by Mikal Hart
2. `ESP32Time` by fbiego
3. `Time` by Paul Stoffregen

---

## 📄 License
MIT License — 2025. Created for high-reliability network time synchronization.