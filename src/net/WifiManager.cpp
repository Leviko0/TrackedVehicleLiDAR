#include "net/WifiManager.h"

#include <ESPmDNS.h>
#include <WiFi.h>

WifiManager::WifiManager(const WifiConfig& config) : config_(config) {}

void WifiManager::begin() {
  WiFi.persistent(false);
  WiFi.setHostname(config_.hostname);

  if (!connectStation()) {
    startAccessPoint();
  }

  // Power saving adds latency to every packet; the vehicle needs snappy control.
  WiFi.setSleep(false);
  startMdns();
}

bool WifiManager::connectStation() {
  if (config_.staSsid == nullptr || config_.staSsid[0] == '\0') return false;

  Serial.printf("[wifi] connecting to \"%s\"", config_.staSsid);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(config_.staSsid, config_.staPassword);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > config_.staTimeoutMs) {
      Serial.println(" failed");
      WiFi.disconnect(true);
      return false;
    }
    Serial.print('.');
    delay(250);
  }
  Serial.println(" ok");
  accessPoint_ = false;
  return true;
}

void WifiManager::startAccessPoint() {
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(config_.apSsid, config_.apPassword)) {
    Serial.println("[wifi] failed to start access point");
  }
  accessPoint_ = true;
  Serial.printf("[wifi] access point \"%s\" started\n", config_.apSsid);
}

void WifiManager::startMdns() {
  if (MDNS.begin(config_.hostname)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[wifi] mDNS: http://%s.local\n", config_.hostname);
  }
}

IPAddress WifiManager::ip() const {
  return accessPoint_ ? WiFi.softAPIP() : WiFi.localIP();
}

String WifiManager::ssid() const {
  return accessPoint_ ? String(config_.apSsid) : WiFi.SSID();
}
