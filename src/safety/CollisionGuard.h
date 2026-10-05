#pragma once

#include <Arduino.h>
#include <PolarScan.h>

#include <atomic>

#include "drive/DriveController.h"

struct GuardConfig {
  float vehicleHalfWidthMm;
  float vehicleHalfLengthMm;
  float sideMarginMm;
  float stopDistanceMm;
  float slowDistanceMm;
  float noObstacleMm;  // reported clearance when the corridor is empty
};

// Uses LiDAR scans to limit the throttle in the driving direction:
// slows down when something is ahead (or behind) and stops before hitting it.
class CollisionGuard {
 public:
  CollisionGuard(DriveController& drive, const GuardConfig& config, bool enabled);

  // Feed every new scan.
  void onScan(const lidarcore::PolarScan& scan);

  // Call while no LiDAR data arrives: the guard cannot see anything then
  // and stops limiting (the phone shows the LiDAR as offline).
  void onLidarOffline();

  void setEnabled(bool enabled);
  bool enabled() const { return enabled_; }

  float frontClearanceMm() const { return front_; }
  float rearClearanceMm() const { return rear_; }

 private:
  void apply();

  DriveController& drive_;
  GuardConfig config_;
  std::atomic<bool> enabled_;
  float front_ = -1.0f;  // -1 = unknown
  float rear_ = -1.0f;
  bool haveData_ = false;
};
