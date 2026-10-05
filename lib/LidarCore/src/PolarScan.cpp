#include "PolarScan.h"

#include <cmath>

namespace lidarcore {

namespace {
constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;
constexpr float RAD_TO_DEG = 180.0f / 3.14159265358979f;
}  // namespace

ScanBuilder::ScanBuilder(const LidarMount& mount, const ScanFilter& filter)
    : mount_(mount), filter_(filter) {}

void ScanBuilder::onRevolutionStart() {
  // The first ring start only synchronises; everything before it is a partial
  // revolution.
  if (started_ && samplesThisRevolution_ >= filter_.minSamplesPerRevolution) {
    completed_ = building_;
    hasNewScan_ = true;
    ++completedScans_;
  }
  started_ = true;
  building_ = PolarScan();
  samplesThisRevolution_ = 0;
}

void ScanBuilder::onSample(float angleDeg, uint16_t distanceMm) {
  if (!started_) return;
  ++samplesThisRevolution_;
  if (distanceMm == 0 || distanceMm > filter_.maxRangeMm) return;

  // Sensor frame -> vehicle frame (clockwise angle from straight ahead).
  const float sensorAngle = mount_.clockwise ? angleDeg : -angleDeg;
  const float bearing = (sensorAngle + mount_.yawDeg) * DEG_TO_RAD;
  const float x = mount_.offsetXmm + distanceMm * std::sin(bearing);
  const float y = mount_.offsetYmm + distanceMm * std::cos(bearing);

  // Ignore returns from the vehicle itself.
  if (std::fabs(x) <= filter_.vehicleHalfWidthMm + filter_.footprintMarginMm &&
      std::fabs(y) <= filter_.vehicleHalfLengthMm + filter_.footprintMarginMm) {
    return;
  }

  float vehicleAngle = std::atan2(x, y) * RAD_TO_DEG;
  if (vehicleAngle < 0.0f) vehicleAngle += 360.0f;
  int bin = static_cast<int>(vehicleAngle);
  if (bin >= PolarScan::BINS) bin = 0;

  const float range = std::sqrt(x * x + y * y);
  const uint16_t rangeMm = range >= 65535.0f ? 65535 : static_cast<uint16_t>(range + 0.5f);
  uint16_t& slot = building_.distanceMm[bin];
  if (slot == 0 || rangeMm < slot) slot = rangeMm;
  ++building_.pointCount;
}

bool ScanBuilder::takeScan(PolarScan& out) {
  if (!hasNewScan_) return false;
  out = completed_;
  hasNewScan_ = false;
  return true;
}

}  // namespace lidarcore
