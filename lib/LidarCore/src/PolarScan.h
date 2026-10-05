#pragma once

#include <cstdint>

#include "CoinD6Decoder.h"

namespace lidarcore {

// One revolution in the VEHICLE frame, centred on the vehicle's middle.
//   bin i covers [i, i+1) degrees, 0° = straight ahead, angles grow clockwise
//   (seen from above), so 90° = right side, 180° = behind.
//   Each bin holds the nearest return in millimetres, 0 = nothing seen.
struct PolarScan {
  static constexpr int BINS = 360;
  uint16_t distanceMm[BINS] = {};
  uint16_t pointCount = 0;  // valid returns that went into this scan
};

// Where and how the LiDAR is mounted on the vehicle.
struct LidarMount {
  float yawDeg;       // vehicle angle the sensor's 0° points to (180 = backwards)
  bool clockwise;     // true if the sensor's angles grow clockwise seen from above
  float offsetXmm;    // sensor position relative to the vehicle centre, +x = right
  float offsetYmm;    //                                                   +y = forward
};

struct ScanFilter {
  float vehicleHalfWidthMm;   // returns inside the vehicle footprint ...
  float vehicleHalfLengthMm;
  float footprintMarginMm;    // ... plus this margin are the vehicle itself
  uint16_t maxRangeMm;
  uint16_t minSamplesPerRevolution;  // shorter revolutions are discarded
};

// Turns decoded samples into PolarScans: transforms them into the vehicle
// frame, drops self-hits and keeps the nearest return per degree.
class ScanBuilder : public PointSink {
 public:
  ScanBuilder(const LidarMount& mount, const ScanFilter& filter);

  void onRevolutionStart() override;
  void onSample(float angleDeg, uint16_t distanceMm) override;

  // Copies the latest completed scan into `out`. Returns false if no new scan
  // has been completed since the previous call.
  bool takeScan(PolarScan& out);

  uint32_t completedScans() const { return completedScans_; }

 private:
  LidarMount mount_;
  ScanFilter filter_;
  PolarScan building_;
  PolarScan completed_;
  uint16_t samplesThisRevolution_ = 0;
  bool hasNewScan_ = false;
  bool started_ = false;
  uint32_t completedScans_ = 0;
};

}  // namespace lidarcore
