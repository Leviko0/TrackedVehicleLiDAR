// Tracked vehicle controller for the ESP32-S3.
//
// Hosts a web page that turns a phone into a remote control and shows what
// the LiDAR sees:
//   phone (joystick) --WebSocket--> WebInterface --> DriveController --> Motors
//   LiDAR --UART--> LidarSensor --scan--> CollisionGuard --limits--> DriveController
//                                    \--> WebInterface --> phone (point map)
//
// All tunables live in include/Config.h.

#include <Arduino.h>

#include "Config.h"
#include "drive/DriveController.h"
#include "lidar/LidarSensor.h"
#include "motor/Motor.h"
#include "net/WifiManager.h"
#include "safety/CollisionGuard.h"
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
    config::drive::ACCEL_PER_SECOND,
    config::drive::BRAKE_PER_SECOND,
    config::drive::COMMAND_TIMEOUT_MS,
    config::drive::DEFAULT_SPEED_LIMIT,
});

LidarSensor lidar(Serial1, {
    config::pins::LIDAR_RX,
    config::pins::LIDAR_TX,
    config::lidar::BAUD_RATE,
    config::lidar::RX_BUFFER_BYTES,
    config::lidar::SCAN_TIMEOUT_MS,
    config::lidar::RESTART_INTERVAL_MS,
    {
        config::lidar::MOUNT_YAW_DEG,
        config::lidar::ANGLES_CLOCKWISE,
        config::lidar::MOUNT_OFFSET_X_MM,
        config::lidar::MOUNT_OFFSET_Y_MM,
    },
    {
        config::vehicle::WIDTH_MM / 2,
        config::vehicle::LENGTH_MM / 2,
        config::lidar::FOOTPRINT_MARGIN_MM,
        config::lidar::MAX_RANGE_MM,
        config::lidar::MIN_SAMPLES_PER_REVOLUTION,
    },
});

CollisionGuard guard(drive, {
    config::vehicle::WIDTH_MM / 2,
    config::vehicle::LENGTH_MM / 2,
    config::guard::SIDE_MARGIN_MM,
    config::guard::STOP_DISTANCE_MM,
    config::guard::SLOW_DISTANCE_MM,
    config::lidar::MAX_RANGE_MM,
}, config::guard::ENABLED_BY_DEFAULT);

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
    case drivecore::CommandType::Guard:
      guard.setEnabled(command.value > 0.5f);
      break;
    case drivecore::CommandType::Ping:
      drive.keepAlive();
      break;
    case drivecore::CommandType::Invalid:
      break;
  }
}

String buildTelemetry() {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"l\":%.2f,\"r\":%.2f,\"lim\":%.2f,\"fs\":%s,"
           "\"li\":%s,\"hz\":%.1f,\"gd\":%s,\"cf\":%.0f,\"cr\":%.0f}",
           drive.leftOutput(), drive.rightOutput(), drive.speedLimit(),
           drive.failsafeActive() ? "true" : "false",
           lidar.isOnline() ? "true" : "false", lidar.scanRateHz(),
           guard.enabled() ? "true" : "false",
           guard.frontClearanceMm(), guard.rearClearanceMm());
  return String(json);
}

// Static settings the page needs to draw the vehicle and the guard zones.
String buildGreeting() {
  char json[160];
  snprintf(json, sizeof(json),
           "{\"cfg\":{\"w\":%.0f,\"len\":%.0f,\"stop\":%.0f,\"slow\":%.0f,"
           "\"margin\":%.0f,\"guard\":%s}}",
           config::vehicle::WIDTH_MM, config::vehicle::LENGTH_MM,
           config::guard::STOP_DISTANCE_MM, config::guard::SLOW_DISTANCE_MM,
           config::guard::SIDE_MARGIN_MM, guard.enabled() ? "true" : "false");
  return String(json);
}

// Binary scan message: 'S', 0, then 360 x uint16 little endian distances (mm).
void sendScan(const lidarcore::PolarScan& scan) {
  static uint8_t message[2 + 2 * lidarcore::PolarScan::BINS];
  message[0] = 'S';
  message[1] = 0;
  for (int i = 0; i < lidarcore::PolarScan::BINS; ++i) {
    message[2 + 2 * i] = scan.distanceMm[i] & 0xFF;
    message[3 + 2 * i] = scan.distanceMm[i] >> 8;
  }
  web.broadcastBinary(message, sizeof(message));
}

void updateLidar() {
  static lidarcore::PolarScan scan;
  static uint32_t lastSentMs = 0;

  lidar.update();
  if (lidar.takeScan(scan)) {
    guard.onScan(scan);
    const uint32_t now = millis();
    if (now - lastSentMs >= config::web::SCAN_INTERVAL_MS) {
      lastSentMs = now;
      sendScan(scan);
    }
  } else if (!lidar.isOnline()) {
    guard.onLidarOffline();
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[main] tracked vehicle starting");

  // Motors first, so they are guaranteed off while Wi-Fi comes up.
  drive.begin();
  lidar.begin();

  wifi.begin();
  Serial.printf("[main] open http://%s/ on your phone (Wi-Fi \"%s\")\n",
                wifi.ip().toString().c_str(), wifi.ssid().c_str());

  web.onCommand(handleCommand);
  web.onTelemetry(buildTelemetry);
  web.onGreeting(buildGreeting);
  web.begin();
}

void loop() {
  updateLidar();
  drive.update();
  web.update();
  delay(2);
}
