#include "drive/DriveController.h"

DriveController::DriveController(Motor& left, Motor& right, const DriveConfig& config)
    : leftMotor_(left),
      rightMotor_(right),
      config_(config),
      speedLimit_(drivecore::clamp(config.defaultSpeedLimit, 0.0f, 1.0f)) {}

void DriveController::begin() {
  leftMotor_.begin();
  rightMotor_.begin();
  lastUpdateMs_ = millis();
}

void DriveController::setCommand(float throttle, float turn) {
  const uint32_t now = millis();
  portENTER_CRITICAL(&lock_);
  targetThrottle_ = drivecore::clampUnit(throttle);
  targetTurn_ = drivecore::clampUnit(turn);
  lastCommandMs_ = now;
  hasCommand_ = true;
  portEXIT_CRITICAL(&lock_);
}

void DriveController::stop() { setCommand(0.0f, 0.0f); }

void DriveController::emergencyStop() {
  portENTER_CRITICAL(&lock_);
  targetThrottle_ = 0.0f;
  targetTurn_ = 0.0f;
  emergencyStopRequested_ = true;
  portEXIT_CRITICAL(&lock_);
}

void DriveController::keepAlive() {
  const uint32_t now = millis();
  portENTER_CRITICAL(&lock_);
  lastCommandMs_ = now;
  portEXIT_CRITICAL(&lock_);
}

void DriveController::setSpeedLimit(float limit) {
  portENTER_CRITICAL(&lock_);
  speedLimit_ = drivecore::clamp(limit, 0.0f, 1.0f);
  portEXIT_CRITICAL(&lock_);
}

float DriveController::speedLimit() const {
  portENTER_CRITICAL(&lock_);
  const float limit = speedLimit_;
  portEXIT_CRITICAL(&lock_);
  return limit;
}

void DriveController::update() {
  const uint32_t now = millis();
  const float dt = (now - lastUpdateMs_) / 1000.0f;
  lastUpdateMs_ = now;

  portENTER_CRITICAL(&lock_);
  const float throttle = targetThrottle_;
  const float turn = targetTurn_;
  const float limit = speedLimit_;
  const uint32_t lastCommandMs = lastCommandMs_;
  const bool hasCommand = hasCommand_;
  const bool emergencyStop = emergencyStopRequested_;
  emergencyStopRequested_ = false;
  portEXIT_CRITICAL(&lock_);

  failsafe_ = !hasCommand || (now - lastCommandMs) > config_.commandTimeoutMs;

  // Safety stops bypass the ramp.
  if (failsafe_ || emergencyStop) {
    left_ = 0.0f;
    right_ = 0.0f;
    leftMotor_.coast();
    rightMotor_.coast();
    return;
  }

  drivecore::TrackSpeeds goal = drivecore::mixArcade(
      drivecore::applyDeadband(throttle, config_.deadband),
      drivecore::applyDeadband(turn, config_.deadband));

  const float maxStep = config_.rampPerSecond * dt;
  left_ = drivecore::slewToward(left_, goal.left * limit, maxStep);
  right_ = drivecore::slewToward(right_, goal.right * limit, maxStep);

  leftMotor_.setSpeed(left_);
  rightMotor_.setSpeed(right_);
}
