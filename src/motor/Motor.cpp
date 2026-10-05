#include "motor/Motor.h"

#include <DriveMath.h>

Motor::Motor(const MotorConfig& config)
    : config_(config), maxDuty_((1UL << config.pwmResolutionBits) - 1) {}

void Motor::begin() {
  pinMode(config_.in1Pin, OUTPUT);
  pinMode(config_.in2Pin, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(config_.enablePin, config_.pwmFrequencyHz,
                    config_.pwmResolutionBits, config_.pwmChannel);
#else
  ledcSetup(config_.pwmChannel, config_.pwmFrequencyHz, config_.pwmResolutionBits);
  ledcAttachPin(config_.enablePin, config_.pwmChannel);
#endif

  coast();
}

void Motor::setSpeed(float speed) {
  speed_ = drivecore::clampUnit(speed);

  const float signedSpeed = config_.inverted ? -speed_ : speed_;
  const float duty = drivecore::speedToDuty(fabsf(signedSpeed), config_.minDuty);
  if (duty <= 0.0f) {
    coast();
    return;
  }

  const bool forward = signedSpeed > 0.0f;
  digitalWrite(config_.in1Pin, forward ? HIGH : LOW);
  digitalWrite(config_.in2Pin, forward ? LOW : HIGH);
  writeDuty(duty);
}

void Motor::coast() {
  speed_ = 0.0f;
  digitalWrite(config_.in1Pin, LOW);
  digitalWrite(config_.in2Pin, LOW);
  writeDuty(0.0f);
}

void Motor::brake() {
  speed_ = 0.0f;
  digitalWrite(config_.in1Pin, HIGH);
  digitalWrite(config_.in2Pin, HIGH);
  writeDuty(1.0f);
}

void Motor::writeDuty(float duty) {
  const uint32_t value = static_cast<uint32_t>(duty * maxDuty_ + 0.5f);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(config_.enablePin, value);
#else
  ledcWrite(config_.pwmChannel, value);
#endif
}
