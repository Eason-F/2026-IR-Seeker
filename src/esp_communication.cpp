#include "esp_communication.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <cstring>

#include "bluetooth_coordination.hpp"
#include "config.hpp"
#include "uart_communication.hpp"

namespace esp_communication {
namespace {

constexpr uint8_t BROADCAST_ADDRESS[6] = {
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

constexpr uint8_t RECEIVE_QUEUE_SIZE = 4;
RobotPacket receiveQueue[RECEIVE_QUEUE_SIZE];
volatile uint8_t receiveHead = 0;
volatile uint8_t receiveTail = 0;
portMUX_TYPE receiveMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t lastRelayMs = 0;

void onEspNowReceive(const uint8_t *, const uint8_t *data, int length) {
  if (data == nullptr || length != static_cast<int>(sizeof(RobotPacket))) {
    return;
  }

  portENTER_CRITICAL(&receiveMux);
  const uint8_t nextHead =
      static_cast<uint8_t>((receiveHead + 1) % RECEIVE_QUEUE_SIZE);
  if (nextHead != receiveTail) {
    memcpy(&receiveQueue[receiveHead], data, sizeof(RobotPacket));
    receiveHead = nextHead;
  }
  portEXIT_CRITICAL(&receiveMux);
}

void onUartPacket(uart_communication::PacketType type,
                  const uint8_t *payload, uint8_t payloadLength) {
  if (type != uart_communication::PacketType::TEENSY_TO_BLUETOOTH ||
      payloadLength != sizeof(RobotPacket)) {
    return;
  }

  esp_now_send(BROADCAST_ADDRESS, payload, payloadLength);
}

bool dequeuePacket(RobotPacket &packet) {
  bool available = false;
  portENTER_CRITICAL(&receiveMux);
  if (receiveTail != receiveHead) {
    memcpy(&packet, &receiveQueue[receiveTail], sizeof(packet));
    receiveTail =
        static_cast<uint8_t>((receiveTail + 1) % RECEIVE_QUEUE_SIZE);
    available = true;
  }
  portEXIT_CRITICAL(&receiveMux);
  return available;
}

}  // namespace

bool begin() {
  WiFi.mode(WIFI_STA);
  if (esp_wifi_set_channel(config::ESP_NOW_CHANNEL,
                           WIFI_SECOND_CHAN_NONE) != ESP_OK) {
    return false;
  }
  if (esp_now_init() != ESP_OK) return false;

  esp_now_register_recv_cb(onEspNowReceive);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDRESS, sizeof(BROADCAST_ADDRESS));
  peer.channel = config::ESP_NOW_CHANNEL;
  peer.encrypt = false;
  const bool peerAdded = esp_now_add_peer(&peer) == ESP_OK;
  lastRelayMs = millis();
  return peerAdded;
}

void update() {
  uart_communication::receivePackets(onUartPacket);

  const uint32_t now = millis();
  if (static_cast<uint32_t>(now - lastRelayMs) <
      config::ESP_NOW_RELAY_INTERVAL_MS) {
    return;
  }
  lastRelayMs = now;

  RobotPacket packet;
  if (dequeuePacket(packet)) {
    uart_communication::sendBluetoothToTeensy(
        reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  }
}

}
