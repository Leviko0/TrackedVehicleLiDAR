#pragma once

#include <Arduino.h>
#include <IPAddress.h>

struct WifiConfig {
  const char* staSsid;       // network to join; empty -> go straight to AP mode
  const char* staPassword;
  uint32_t staTimeoutMs;
  const char* apSsid;        // access point opened when joining fails
  const char* apPassword;
  const char* hostname;      // mDNS name, reachable as <hostname>.local
};

// Joins an existing Wi-Fi network if configured, otherwise (or on failure)
// opens an access point so the phone can connect to the vehicle directly.
class WifiManager {
 public:
  explicit WifiManager(const WifiConfig& config);

  // Blocks until connected or the access point is up.
  void begin();

  bool isAccessPoint() const { return accessPoint_; }
  IPAddress ip() const;
  String ssid() const;

 private:
  bool connectStation();
  void startAccessPoint();
  void startMdns();

  WifiConfig config_;
  bool accessPoint_ = false;
};
