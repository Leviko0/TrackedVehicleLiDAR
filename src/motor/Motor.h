#pragma once

#include <Arduino.h>

// One channel of an L298N H-bridge driving a DC motor.
//   enable -> ENA/ENB (PWM speed)
//   in1/in2 -> IN1/IN2 or IN3/IN4 (direction)
struct MotorConfig {
  uint8_t enablePin;
  uint8_t in1Pin;
  uint8_t in2Pin;
  uint8_t pwmChannel;
  uint32_t pwmFrequencyHz;
  uint8_t pwmResolutionBits;
  float minDuty;   // duty cycle the motor needs to start moving
  bool inverted;   // swap direction in software
};

class Motor {
 public:
  explicit Motor(const MotorConfig& config);

  // Configures the pins and the PWM peripheral. Call once in setup().
  void begin();

  // speed in [-1, 1]: positive = forward, 0 = coast.
  void setSpeed(float speed);

  // Both direction pins low: motor spins freely.
  void coast();

  // Both direction pins high with full enable: motor is shorted and holds.
  void brake();

  float speed() const { return speed_; }

 private:
  void writeDuty(float duty);

  MotorConfig config_;
  uint32_t maxDuty_;
  float speed_ = 0.0f;
};
