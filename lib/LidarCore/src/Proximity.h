#pragma once

#include "PolarScan.h"

namespace lidarcore {

// Free distance in front of / behind the vehicle inside its driving corridor
// (the strip as wide as the vehicle plus a side margin).
//   Measured from the vehicle's front (or rear) edge to the nearest return.
//   Returns `noObstacleMm` if the corridor is empty.
float corridorClearance(const PolarScan& scan, bool forward, float halfWidthMm,
                        float halfLengthMm, float sideMarginMm, float noObstacleMm);

// Speed factor for a given clearance:
//   <= stopMm -> 0, >= slowMm -> 1, linear in between.
float speedFactorForClearance(float clearanceMm, float stopMm, float slowMm);

}  // namespace lidarcore
