#pragma once

#include <Arduino.h>
#include <DriveMath.h>

#include "motor/Motor.h"

struct DriveConfig {
  float deadband;          // joystick deadband (0..1)
  float rampPerSecond;     // max change of track speed per second
  uint32_t commandTimeoutMs;  // failsafe: stop when commands stop arriving
  float defaultSpeedLimit; // 0..1
};

// Turns (throttle, turn) commands into smooth track speeds for two motors.
//
// Thread safety: the setters may be called from the web server task while
// update() runs in loop(); shared state is guarded by a spinlock.
class DriveController {
 public:
  DriveController(Motor& left, Motor& right, const DriveConfig& config);

  void begin();

  // throttle: +1 forward / -1 backward, turn: +1 clockwise / -1 counter-clockwise.
  // Also feeds the failsafe watchdog.
  void setCommand(float throttle, float turn);

  // Ramp down to standstill.
  void stop();

  // Cut motor power immediately, skipping the ramp.
  void emergencyStop();

  // Keeps the failsafe from triggering without changing the command.
  void keepAlive();

  void setSpeedLimit(float limit);
  float speedLimit() const;

  // Call as often as possible from loop().
  void update();

  float leftOutput() const { return left_; }
  float rightOutput() const { return right_; }
  bool failsafeActive() const { return failsafe_; }

 private:
  Motor& leftMotor_;
  Motor& rightMotor_;
  DriveConfig config_;

  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  // --- guarded by lock_ ---
  float targetThrottle_ = 0.0f;
  float targetTurn_ = 0.0f;
  float speedLimit_;
  uint32_t lastCommandMs_ = 0;
  bool hasCommand_ = false;
  bool emergencyStopRequested_ = false;
  // ------------------------

  // Only touched by update() (loop task).
  float left_ = 0.0f;
  float right_ = 0.0f;
  bool failsafe_ = true;
  uint32_t lastUpdateMs_ = 0;
};
