#include "lidar/LidarSensor.h"

LidarSensor::LidarSensor(HardwareSerial& serial, const LidarConfig& config)
    : serial_(serial),
      config_(config),
      builder_(config.mount, config.filter),
      decoder_(builder_) {}

void LidarSensor::begin() {
  serial_.setRxBufferSize(config_.rxBufferBytes);
  serial_.begin(config_.baudRate, SERIAL_8N1, config_.rxPin, config_.txPin);
  sendStart();
}

void LidarSensor::sendStart() {
  serial_.write(lidarcore::CoinD6Decoder::START_COMMAND,
                sizeof(lidarcore::CoinD6Decoder::START_COMMAND));
  lastStartMs_ = millis();
}

void LidarSensor::update() {
  uint8_t chunk[256];
  int available = serial_.available();
  while (available > 0) {
    const size_t n = serial_.readBytes(chunk, min(available, static_cast<int>(sizeof(chunk))));
    decoder_.feed(chunk, n);
    available = serial_.available();
  }

  const uint32_t now = millis();
  const uint32_t completed = builder_.completedScans();
  if (completed != seenScans_) {
    if (lastScanMs_ != 0 && now > lastScanMs_) {
      const float instant = 1000.0f * (completed - seenScans_) / (now - lastScanMs_);
      scanRateHz_ = scanRateHz_ == 0.0f ? instant : 0.8f * scanRateHz_ + 0.2f * instant;
    }
    seenScans_ = completed;
    lastScanMs_ = now;
    if (!everOnline_) {
      everOnline_ = true;
      Serial.println("[lidar] receiving scans");
    }
  }

  // Sensor silent (not started yet, or power came late): ask it to start again.
  if (!isOnline() && now - lastStartMs_ >= config_.restartIntervalMs) {
    decoder_.reset();
    sendStart();
  }
}

bool LidarSensor::takeScan(lidarcore::PolarScan& out) { return builder_.takeScan(out); }

bool LidarSensor::isOnline() const {
  return lastScanMs_ != 0 && millis() - lastScanMs_ <= config_.scanTimeoutMs;
}
