#include "web/WebInterface.h"

#include "web/WebPage.h"

namespace {
constexpr size_t MAX_MESSAGE_LENGTH = 64;
constexpr uint32_t CLEANUP_INTERVAL_MS = 1000;
}  // namespace

WebInterface::WebInterface(uint16_t port, uint32_t telemetryIntervalMs)
    : server_(port), socket_("/ws"), telemetryIntervalMs_(telemetryIntervalMs) {}

void WebInterface::begin() {
  socket_.onEvent([this](AsyncWebSocket*, AsyncWebSocketClient* client, AwsEventType type,
                         void* arg, uint8_t* data, size_t len) {
    handleWsEvent(client, type, arg, data, len);
  });
  server_.addHandler(&socket_);

  server_.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/html", web::INDEX_HTML);
  });

  // Anything else (e.g. captive portal probes) leads to the control page.
  server_.onNotFound([](AsyncWebServerRequest* request) { request->redirect("/"); });

  server_.begin();
}

void WebInterface::update() {
  const uint32_t now = millis();

  if (now - lastCleanupMs_ >= CLEANUP_INTERVAL_MS) {
    lastCleanupMs_ = now;
    socket_.cleanupClients();
  }

  if (telemetryProvider_ && socket_.count() > 0 &&
      now - lastTelemetryMs_ >= telemetryIntervalMs_) {
    lastTelemetryMs_ = now;
    if (socket_.availableForWriteAll()) {
      socket_.textAll(telemetryProvider_());
    }
  }
}

size_t WebInterface::clientCount() const { return socket_.count(); }

void WebInterface::broadcastBinary(const uint8_t* data, size_t length) {
  if (socket_.count() > 0 && socket_.availableForWriteAll()) {
    socket_.binaryAll(data, length);
  }
}

void WebInterface::handleWsEvent(AsyncWebSocketClient* client, AwsEventType type, void* arg,
                                 uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("[web] client #%u connected from %s\n", client->id(),
                    client->remoteIP().toString().c_str());
      if (greetingProvider_) client->text(greetingProvider_());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("[web] client #%u disconnected\n", client->id());
      // Never keep driving because a controlling phone went away.
      {
        drivecore::Command stop;
        stop.type = drivecore::CommandType::Stop;
        dispatch(stop);
      }
      break;
    case WS_EVT_DATA:
      handleWsMessage(arg, data, len);
      break;
    default:
      break;
  }
}

void WebInterface::handleWsMessage(void* arg, uint8_t* data, size_t len) {
  const auto* info = static_cast<AwsFrameInfo*>(arg);
  // Commands are tiny: only accept complete, unfragmented text frames.
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) return;
  if (len == 0 || len > MAX_MESSAGE_LENGTH) return;

  char buffer[MAX_MESSAGE_LENGTH + 1];
  memcpy(buffer, data, len);
  buffer[len] = '\0';

  dispatch(drivecore::parseCommand(buffer));
}

void WebInterface::dispatch(const drivecore::Command& command) {
  if (command.type != drivecore::CommandType::Invalid && commandHandler_) {
    commandHandler_(command);
  }
}
