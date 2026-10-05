#include "safety/CollisionGuard.h"

#include <Proximity.h>

CollisionGuard::CollisionGuard(DriveController& drive, const GuardConfig& config, bool enabled)
    : drive_(drive), config_(config), enabled_(enabled) {}

void CollisionGuard::onScan(const lidarcore::PolarScan& scan) {
  front_ = lidarcore::corridorClearance(scan, true, config_.vehicleHalfWidthMm,
                                        config_.vehicleHalfLengthMm, config_.sideMarginMm,
                                        config_.noObstacleMm);
  rear_ = lidarcore::corridorClearance(scan, false, config_.vehicleHalfWidthMm,
                                       config_.vehicleHalfLengthMm, config_.sideMarginMm,
                                       config_.noObstacleMm);
  haveData_ = true;
  apply();
}

void CollisionGuard::onLidarOffline() {
  front_ = -1.0f;
  rear_ = -1.0f;
  haveData_ = false;
  apply();
}

void CollisionGuard::setEnabled(bool enabled) {
  // Called from the web server task; takes effect with the next scan.
  enabled_ = enabled;
}

void CollisionGuard::apply() {
  if (!enabled_ || !haveData_) {
    drive_.setThrottleLimits(1.0f, 1.0f);
    return;
  }
  drive_.setThrottleLimits(
      lidarcore::speedFactorForClearance(front_, config_.stopDistanceMm, config_.slowDistanceMm),
      lidarcore::speedFactorForClearance(rear_, config_.stopDistanceMm, config_.slowDistanceMm));
}
