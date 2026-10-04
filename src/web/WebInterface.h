#pragma once

#include <Arduino.h>
#include <CommandParser.h>
#include <ESPAsyncWebServer.h>

#include <functional>

// Serves the control page and exchanges messages with it over a WebSocket.
//   - incoming text frames are parsed into drivecore::Command objects
//   - telemetry JSON is pushed to all clients periodically
class WebInterface {
 public:
  using CommandHandler = std::function<void(const drivecore::Command&)>;
  using TelemetryProvider = std::function<String()>;

  WebInterface(uint16_t port, uint32_t telemetryIntervalMs);

  void onCommand(CommandHandler handler) { commandHandler_ = std::move(handler); }
  void onTelemetry(TelemetryProvider provider) { telemetryProvider_ = std::move(provider); }

  void begin();

  // Call from loop(): sends telemetry and cleans up stale clients.
  void update();

  size_t clientCount() const;

 private:
  void handleWsEvent(AsyncWebSocketClient* client, AwsEventType type, void* arg,
                     uint8_t* data, size_t len);
  void handleWsMessage(void* arg, uint8_t* data, size_t len);
  void dispatch(const drivecore::Command& command);

  AsyncWebServer server_;
  AsyncWebSocket socket_;
  uint32_t telemetryIntervalMs_;
  uint32_t lastTelemetryMs_ = 0;
  uint32_t lastCleanupMs_ = 0;

  CommandHandler commandHandler_;
  TelemetryProvider telemetryProvider_;
};
