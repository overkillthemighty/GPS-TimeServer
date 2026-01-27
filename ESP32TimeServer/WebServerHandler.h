#pragma once

// Version 1.1

#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include "ESP32TimeServerKeySettings.h"

// =============================================================
// EXTERNAL VARIABLES (Defined in ESP32TimeServer.ino)
// =============================================================
extern WebServer server;
extern ESP32Time rtc;
extern TinyGPSPlus gps;
extern volatile uint64_t ntpServedCount;
extern double lastUpdateDeltaSec;
extern double ewmaOffsetSec;
extern double maxAbsOffsetSec;
extern volatile bool SafeGuardTripped;
extern volatile bool rtcTimeInitialized;
extern int gpsCoreID;

extern String ip;
extern bool useStaticIP_Run;
extern IPAddress ip_Run, gw_Run, sn_Run, dns_Run;

extern String logRing[];
extern size_t logHead;
extern size_t logSize;
extern SemaphoreHandle_t mutex;
extern Preferences prefs;

// External Helper Functions
extern void logMsg(const String& s);
extern void GetUTC_DateAndTimeStrings(time_t UTC_Time, String &dateString, String &timeString);

// =============================================================
// HTML HELPERS
// =============================================================
String htmlHeader(const String& title) {
  return String(F(
    "<!DOCTYPE html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:system-ui,sans-serif;margin:0;padding:20px;background:#f4f4f9;color:#333}"
    ".cont{max-width:600px;margin:0 auto;background:#fff;padding:20px;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,0.1)}"
    "h2{margin-top:0;border-bottom:2px solid #0078d7;padding-bottom:10px;color:#0078d7}"
    "details{margin-bottom:12px;border:1px solid #ddd;border-radius:4px}"
    "summary{padding:10px;cursor:pointer;background:#eee;font-weight:600;color:#555;outline:none}"
    "details[open] summary{border-bottom:1px solid #ddd}"
    ".grp{padding:10px}"
    ".row{display:flex;justify-content:space-between;padding:8px 0;border-bottom:1px solid #eee}"
    ".lbl{font-weight:600;color:#666}"
    ".val{font-family:monospace;font-size:1.1em}"
    ".ok{color:#0a0;font-weight:bold}.bad{color:#d00;font-weight:bold}"
    ".tt{text-decoration:underline dotted #999;cursor:help;position:relative}"
    ".tt:hover::after{content:attr(title);position:absolute;bottom:100%;left:50%;transform:translateX(-50%);background:#333;color:#fff;padding:5px 10px;border-radius:4px;font-size:0.8em;white-space:nowrap;z-index:10;pointer-events:none}"
    "button,input[type=file]{margin-top:10px}"
    "#logs{background:#222;color:#0f0;padding:10px;border-radius:4px;height:300px;overflow-y:auto;font-size:0.85em;margin-top:10px}"
    "input[type=text]{padding:4px;border:1px solid #ccc;border-radius:3px;width:120px;font-family:monospace}"
    "</style><title>")) + title + F("</title></head><body><div class='cont'>");
}

String htmlFooter() {
  return F("</div></body></html>");
}

String makeRow(String label, String value, String id = "", String tooltip = "") {
  String s = "<div class='row'><span class='lbl'";
  if (tooltip.length() > 0) s += " title='" + tooltip + "' class='tt'";
  s += ">" + label + "</span><span class='val'";
  if (id.length() > 0) s += " id='" + id + "'";
  s += ">" + value + "</span></div>";
  return s;
}

// =============================================================
// HANDLERS
// =============================================================

// JSON Endpoint for AJAX
void handleStatus() {
  String dStr, tStr;
  GetUTC_DateAndTimeStrings(rtc.getEpoch(), dStr, tStr);
  
  String json = "{";
  json += "\"utc\":\"" + dStr + " " + tStr + "\",";
  json += "\"sync\":\"" + String(rtcTimeInitialized ? "<span class='ok'>LOCKED</span>" : "<span class='bad'>SYNCING</span>") + "\",";
  json += "\"sats\":" + String(gps.satellites.value()) + ",";
  json += "\"hdop\":\"" + String(gps.hdop.value() / 100.0) + "\",";
  json += "\"req\":" + String((uint32_t)ntpServedCount) + ",";
  json += "\"off\":\"" + String(lastUpdateDeltaSec, 6) + " s\",";
  json += "\"jit\":\"" + String(ewmaOffsetSec, 6) + " s\",";
  json += "\"grd\":\"" + String(SafeGuardTripped ? "<span class='bad'>TRIPPED</span>" : "<span class='ok'>OK</span>") + "\",";
  json += "\"cores\":\"GPS:" + String(gpsCoreID) + " / Sys:" + String(xPortGetCoreID()) + "\",";
  json += "\"up\":\"" + String(millis() / 1000UL) + " s\"";
  json += "}";
  server.send(200, "application/json", json);
}

// Raw Text Endpoint for Logs
void handleLogData() {
  String s = "";
  if (xSemaphoreTake(mutex, 100 / portTICK_PERIOD_MS) == pdTRUE) {
    size_t count = logSize;
    size_t idx = (logSize == LOG_RING_CAPACITY) ? logHead : 0;
    for (size_t i = 0; i < count; ++i) {
        s += logRing[idx] + "\n";
        idx = (idx + 1) % LOG_RING_CAPACITY;
    }
    xSemaphoreGive(mutex);
  } else {
    s = "Log busy...";
  }
  server.send(200, "text/plain", s);
}

void handleRoot() {
  String dStr, tStr;
  GetUTC_DateAndTimeStrings(rtc.getEpoch(), dStr, tStr);

  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");

  String s = htmlHeader("ESP32 Stratum-1");

  // --- TIME SOURCE (Default Open) ---
  s += "<details open><summary>Time Source</summary><div class='grp'>";
  s += makeRow("Current UTC", dStr + " " + tStr, "utc");
  s += makeRow("Stratum", "1", "", "Distance from reference clock");
  s += makeRow("Source", "GPS PPS", "", "Disciplined by Pulse-Per-Second signal");
  s += makeRow("Sync Status", (rtcTimeInitialized ? "<span class='ok'>LOCKED</span>" : "<span class='bad'>SYNCING</span>"), "sync", "Locked means RTC is synchronized to PPS");
  s += makeRow("Satellites", String(gps.satellites.value()), "sats");
  s += makeRow("HDOP", String(gps.hdop.value() / 100.0), "hdop");
  s += "</div></details>";

  // --- PERFORMANCE (Collapsed) ---
  s += "<details><summary>Performance</summary><div class='grp'>";
  s += makeRow("Requests Served", String((uint32_t)ntpServedCount), "req");
  s += makeRow("Last Offset", String(lastUpdateDeltaSec, 6) + " s", "off", "Difference between internal RTC and GPS time at last PPS");
  s += makeRow("Avg Jitter", String(ewmaOffsetSec, 6) + " s", "jit", "Exponential Weighted Moving Average");
  s += makeRow("SafeGuard", (SafeGuardTripped ? "<span class='bad'>TRIPPED</span>" : "<span class='ok'>OK</span>"), "grd", "If tripped, GPS data was suspicious");
  s += "</div></details>";

  // --- SYSTEM (Collapsed) ---
  s += "<details><summary>System</summary><div class='grp'>";
  s += makeRow("IP Address", ip);
  s += makeRow("Active Cores", "GPS:" + String(gpsCoreID) + " / Sys:" + String(xPortGetCoreID()), "cores");
  s += makeRow("Uptime", String(millis() / 1000UL) + " s", "up");
  s += makeRow("Firmware", "v" + String(FIRMWARE_VERSION));
  s += "</div></details>";

  // --- NETWORK CONFIG (Collapsed) ---
  s += "<details><summary>Network Settings</summary><div class='grp'>";
  s += "<form action='/save_net' method='POST'>";
  s += makeRow("IP", "<input name='ip' value='" + ip_Run.toString() + "'>");
  s += makeRow("Gateway", "<input name='gw' value='" + gw_Run.toString() + "'>");
  s += makeRow("Subnet", "<input name='sn' value='" + sn_Run.toString() + "'>");
  s += makeRow("Force Static", String("<input type='checkbox' name='static'") + (useStaticIP_Run ? " checked" : "") + ">", "", "Check to disable DHCP completely");
  s += "<button type='submit'>Save & Reboot</button>";
  s += "</form></div></details>";

  // --- TOOLS (Collapsed) ---
  s += "<details><summary>Tools</summary><div class='grp'>";
  s += "<label style='cursor:pointer'><input type='checkbox' id='autoRef' onchange='toggleAutoRefresh(this)'> <strong>Live Updates</strong></label><br>";
  s += "<a href='/logs'><button>View System Logs</button></a>";
  
  if (otaEnabled) {
    s += "<hr><h4>Update Firmware</h4>"
         "<form id='otaForm'>"
         "<input type='file' id='firmware' accept='.bin'><br>"
         "<progress id='progressBar' value='0' max='100' style='width:100%;display:none'></progress>"
         "<div id='status'></div>"
         "<button type='button' id='uploadBtn' onclick='uploadFirmware()'>Upload .bin</button>"
         "</form>";
  }
  s += "</div></details>";

  // Intelligent JS Refresh
  s += F("<script>"
       "var tm;"
       "function toggleAutoRefresh(cb){"
       "  localStorage.setItem('ar',cb.checked);"
       "  if(cb.checked){"
       "    tm=setInterval(function(){"
       "      fetch('/status').then(r=>r.json()).then(d=>{"
       "        for(var k in d) if(document.getElementById(k)) document.getElementById(k).innerHTML=d[k];"
       "      }).catch(e=>console.log(e));"
       "    }, 2000);"
       "  } else { clearInterval(tm); }"
       "}"
       "if(localStorage.getItem('ar')==='true'){"
       "  document.getElementById('autoRef').checked=true;"
       "  toggleAutoRefresh({checked:true});"
       "}"
       "function uploadFirmware(){"
       " var f=document.getElementById('firmware').files[0];"
       " if(!f){alert('Select file');return;}"
       " var b=document.getElementById('uploadBtn'),p=document.getElementById('progressBar'),st=document.getElementById('status');"
       " b.disabled=true;p.style.display='block';st.innerHTML='Uploading...';"
       " var x=new XMLHttpRequest();"
       " x.open('POST','/update',true);"
       " x.upload.onprogress=(e)=>{p.value=(e.loaded/e.total)*100};"
       " x.onload=()=>{if(x.status==200){st.innerHTML='Success! Rebooting...';setTimeout(()=>location.reload(),8000)}else{st.innerHTML='Err';b.disabled=false}};"
       " var d=new FormData();d.append('firmware',f);x.send(d);}"
       "</script>");

  s += htmlFooter();
  server.send(200, "text/html", s);
}

void handleLogs() {
  String s = htmlHeader("System Logs");
  s += "<div id='logs'>Loading...</div><br>";
  s += "<label><input type='checkbox' id='logRef' onchange='toggleLogRef(this)' checked> Auto-Scroll</label> ";
  s += "<button onclick='location.href=\"/\"'>Back</button>";
  
  s += F("<script>"
         "var tm; var box=document.getElementById('logs');"
         "function fetchLogs(){"
         "  fetch('/log_data').then(r=>r.text()).then(t=>{"
         "    box.innerHTML='<pre>'+t+'</pre>';"
         "    if(document.getElementById('logRef').checked) box.scrollTop=box.scrollHeight;"
         "  });"
         "}"
         "function toggleLogRef(cb){ if(cb.checked) fetchLogs(); }"
         "tm=setInterval(function(){ if(document.getElementById('logRef').checked) fetchLogs(); }, 2000);"
         "fetchLogs();"
         "</script>");
         
  s += htmlFooter();
  server.send(200, "text/html", s);
}

void handleNotFound() { server.send(404, "text/plain", "Not Found"); }

void handleUpdate() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    logMsg("OTA Start: " + upload.filename);
    if (!Update.begin()) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) logMsg("OTA Success"); else Update.printError(Serial);
  }
}

void handleUpdateDone() {
  if (Update.hasError()) server.send(500, "text/plain", "OTA failed");
  else { server.send(200, "text/plain", "OK"); delay(500); ESP.restart(); }
}

void handleSaveNet() {
  if (!server.hasArg("ip")) { server.send(400, "text/plain", "Bad Args"); return; }
  IPAddress newIP, newGW, newSN;
  if (!newIP.fromString(server.arg("ip")) || !newGW.fromString(server.arg("gw")) || !newSN.fromString(server.arg("sn"))) {
    server.send(400, "text/plain", "Invalid IP"); return;
  }
  
  prefs.begin("net_config", false); 
  prefs.putBool("static", server.hasArg("static"));
  prefs.putUInt("ip", (uint32_t)newIP);
  prefs.putUInt("gw", (uint32_t)newGW);
  prefs.putUInt("sn", (uint32_t)newSN);
  prefs.putUInt("dns", (uint32_t)IPAddress(8,8,8,8)); 
  prefs.end();

  String s = htmlHeader("Saved");
  s += "Settings saved. Device is rebooting to apply...<br>If IP changed, please navigate to new IP: " + newIP.toString();
  s += htmlFooter();
  server.send(200, "text/html", s);
  
  delay(1000);
  ESP.restart();
}

void startWeb() {
  if (!webEnabled) return;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/logs", HTTP_GET, handleLogs);
  server.on("/log_data", HTTP_GET, handleLogData);
  server.on("/save_net", HTTP_POST, handleSaveNet);
  if (otaEnabled) {
    server.on("/update", HTTP_POST, handleUpdateDone, handleUpdate);
  }
  server.onNotFound(handleNotFound);
  server.begin();
  logMsg("Web server listening on port " + String(WEB_PORT));
}

void processWebLoop() {
  if (webEnabled) server.handleClient();

}
