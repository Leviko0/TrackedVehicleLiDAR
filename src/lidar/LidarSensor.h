#pragma once

#include <Arduino.h>
#include <CoinD6Decoder.h>
#include <PolarScan.h>

struct LidarConfig {
  uint8_t rxPin;
  uint8_t txPin;
  uint32_t baudRate;
  size_t rxBufferBytes;
  uint32_t scanTimeoutMs;
  uint32_t restartIntervalMs;
  lidarcore::LidarMount mount;
  lidarcore::ScanFilter filter;
};

// COIN-D6 LiDAR on a hardware UART: starts the sensor, decodes its data
// stream and provides complete 360° scans in the vehicle frame.
class LidarSensor {
 public:
  LidarSensor(HardwareSerial& serial, const LidarConfig& config);

  void begin();

  // Reads all pending bytes. Call from loop() as often as possible.
  void update();

  // Copies the newest complete scan; false if there is none since last call.
  bool takeScan(lidarcore::PolarScan& out);

  // True while complete scans keep arriving.
  bool isOnline() const;

  // Revolutions per second, averaged.
  float scanRateHz() const { return scanRateHz_; }

  uint32_t checksumErrors() const { return decoder_.checksumErrors(); }

 private:
  void sendStart();

  HardwareSerial& serial_;
  LidarConfig config_;
  lidarcore::ScanBuilder builder_;
  lidarcore::CoinD6Decoder decoder_;

  uint32_t lastScanMs_ = 0;
  uint32_t lastStartMs_ = 0;
  uint32_t seenScans_ = 0;
  float scanRateHz_ = 0.0f;
  bool everOnline_ = false;
};
