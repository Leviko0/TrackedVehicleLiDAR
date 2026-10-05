#pragma once

// Pure, hardware independent math for a differential (tank) drive.
// Kept free of Arduino dependencies so it can be unit tested on the host.

namespace drivecore {

// Speeds of both tracks, each in [-1, 1]. Positive = forward.
struct TrackSpeeds {
  float left = 0.0f;
  float right = 0.0f;
};

// Clamp to [lo, hi].
float clamp(float value, float lo, float hi);

// Clamp to [-1, 1].
float clampUnit(float value);

// Returns 0 inside the deadband and rescales the rest so the output still
// covers the full range: deadband..1 -> 0..1.
float applyDeadband(float value, float deadband);

// Arcade style mixing.
//   throttle: +1 forward, -1 backward
//   turn:     +1 clockwise (turn right), -1 counter-clockwise (turn left)
// throttle = 0, turn = +-1 spins the vehicle on the spot.
// The result is scaled down (keeping the ratio) if a track would exceed 1.
TrackSpeeds mixArcade(float throttle, float turn);

// Moves `current` towards `target` by at most `maxStep`.
float slewToward(float current, float target, float maxStep);

// Like slewToward, but slowing down (moving towards zero) may use a larger
// step than speeding up. A change of direction first brakes to zero.
float rampToward(float current, float target, float accelStep, float decelStep);

// Maps a speed magnitude in [0, 1] to a PWM duty cycle in [minDuty, 1].
// A magnitude of 0 always yields 0 (motor off).
float speedToDuty(float magnitude, float minDuty);

}  // namespace drivecore
