#include "Proximity.h"

#include <cmath>

namespace lidarcore {

namespace {
constexpr float DEG_TO_RAD = 3.14159265358979f / 180.0f;
}  // namespace

float corridorClearance(const PolarScan& scan, bool forward, float halfWidthMm,
                        float halfLengthMm, float sideMarginMm, float noObstacleMm) {
  const float corridorHalfWidth = halfWidthMm + sideMarginMm;
  float clearance = noObstacleMm;

  for (int bin = 0; bin < PolarScan::BINS; ++bin) {
    const uint16_t distance = scan.distanceMm[bin];
    if (distance == 0) continue;

    const float angle = (bin + 0.5f) * DEG_TO_RAD;
    const float x = distance * std::sin(angle);
    float y = distance * std::cos(angle);
    if (!forward) y = -y;

    if (y <= 0.0f || std::fabs(x) > corridorHalfWidth) continue;
    const float gap = y - halfLengthMm;
    const float clamped = gap < 0.0f ? 0.0f : gap;
    if (clamped < clearance) clearance = clamped;
  }
  return clearance;
}

float speedFactorForClearance(float clearanceMm, float stopMm, float slowMm) {
  if (clearanceMm <= stopMm) return 0.0f;
  if (clearanceMm >= slowMm || slowMm <= stopMm) return 1.0f;
  return (clearanceMm - stopMm) / (slowMm - stopMm);
}

}  // namespace lidarcore
