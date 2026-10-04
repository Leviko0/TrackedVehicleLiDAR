// Tracked vehicle controller for the ESP32-S3.
//
// Hosts a web page that turns a phone into a remote control:
//   phone (joystick / buttons) --WebSocket--> WebInterface --> DriveController --> Motors
//
// All tunables live in include/Config.h.

#include <Arduino.h>

#include "Config.h"
#include "drive/DriveController.h"
#include "motor/Motor.h"
#include "net/WifiManager.h"
#include "web/WebInterface.h"

namespace {

Motor leftMotor({
    config::pins::LEFT_ENABLE,
    config::pins::LEFT_IN1,
    config::pins::LEFT_IN2,
    config::motor::LEFT_PWM_CHANNEL,
    config::motor::PWM_FREQUENCY_HZ,
    config::motor::PWM_RESOLUTION_BITS,
    config::motor::MIN_DUTY,
    config::motor::LEFT_INVERTED,
});

Motor rightMotor({
    config::pins::RIGHT_ENABLE,
    config::pins::RIGHT_IN1,
    config::pins::RIGHT_IN2,
    config::motor::RIGHT_PWM_CHANNEL,
    config::motor::PWM_FREQUENCY_HZ,
    config::motor::PWM_RESOLUTION_BITS,
    config::motor::MIN_DUTY,
    config::motor::RIGHT_INVERTED,
});

DriveController drive(leftMotor, rightMotor, {
    config::drive::DEADBAND,
    config::drive::RAMP_PER_SECOND,
    config::drive::COMMAND_TIMEOUT_MS,
    config::drive::DEFAULT_SPEED_LIMIT,
});

WifiManager wifi({
    config::wifi::STA_SSID,
    config::wifi::STA_PASSWORD,
    config::wifi::STA_CONNECT_TIMEOUT_MS,
    config::wifi::AP_SSID,
    config::wifi::AP_PASSWORD,
    config::wifi::HOSTNAME,
});

WebInterface web(config::web::HTTP_PORT, config::web::TELEMETRY_INTERVAL_MS);

void handleCommand(const drivecore::Command& command) {
  switch (command.type) {
    case drivecore::CommandType::Drive:
      drive.setCommand(command.throttle, command.turn);
      break;
    case drivecore::CommandType::Stop:
      drive.emergencyStop();
      break;
    case drivecore::CommandType::SpeedLimit:
      drive.setSpeedLimit(command.value);
      break;
    case drivecore::CommandType::Ping:
      drive.keepAlive();
      break;
    case drivecore::CommandType::Invalid:
      break;
  }
}

String buildTelemetry() {
  char json[96];
  snprintf(json, sizeof(json), "{\"l\":%.2f,\"r\":%.2f,\"lim\":%.2f,\"fs\":%s}",
           drive.leftOutput(), drive.rightOutput(), drive.speedLimit(),
           drive.failsafeActive() ? "true" : "false");
  return String(json);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[main] tracked vehicle starting");

  // Motors first, so they are guaranteed off while Wi-Fi comes up.
  drive.begin();

  wifi.begin();
  Serial.printf("[main] open http://%s/ on your phone (Wi-Fi \"%s\")\n",
                wifi.ip().toString().c_str(), wifi.ssid().c_str());

  web.onCommand(handleCommand);
  web.onTelemetry(buildTelemetry);
  web.begin();
}

void loop() {
  drive.update();
  web.update();
  delay(5);
}
