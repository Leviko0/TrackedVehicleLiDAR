#include "DriveMath.h"

#include <cmath>

namespace drivecore {

float clamp(float value, float lo, float hi) {
  if (std::isnan(value)) return 0.0f;
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

float clampUnit(float value) { return clamp(value, -1.0f, 1.0f); }

float applyDeadband(float value, float deadband) {
  value = clampUnit(value);
  deadband = clamp(deadband, 0.0f, 0.99f);
  const float magnitude = std::fabs(value);
  if (magnitude <= deadband) return 0.0f;
  const float scaled = (magnitude - deadband) / (1.0f - deadband);
  return value < 0.0f ? -scaled : scaled;
}

TrackSpeeds mixArcade(float throttle, float turn) {
  throttle = clampUnit(throttle);
  turn = clampUnit(turn);

  float left = throttle + turn;
  float right = throttle - turn;

  const float largest = std::fmax(std::fabs(left), std::fabs(right));
  if (largest > 1.0f) {
    left /= largest;
    right /= largest;
  }
  TrackSpeeds speeds;
  speeds.left = left;
  speeds.right = right;
  return speeds;
}

float slewToward(float current, float target, float maxStep) {
  if (maxStep <= 0.0f) return current;
  const float delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

float speedToDuty(float magnitude, float minDuty) {
  magnitude = clamp(magnitude, 0.0f, 1.0f);
  if (magnitude <= 0.0f) return 0.0f;
  minDuty = clamp(minDuty, 0.0f, 1.0f);
  return minDuty + (1.0f - minDuty) * magnitude;
}

}  // namespace drivecore
