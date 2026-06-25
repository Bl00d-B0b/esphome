#ifndef USE_ESP32

// component.h first: it pulls in esphome/core/defines.h which sets USE_BLUETOOTH_PROXY
// before the guards in the headers below are evaluated.
#include "esphome/core/component.h"
#include "bluetooth_proxy_libretiny.h"

#include "esphome/components/api/api_connection.h"
#include "esphome/core/log.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstring>

namespace esphome::bluetooth_proxy {

static const char *const TAG = "bluetooth_proxy";

// Looked up by api_connection.cpp; assigned in setup().
BluetoothProxy *global_bluetooth_proxy = nullptr;  // NOLINT

void BluetoothProxy::setup() {
  global_bluetooth_proxy = this;
  // Reserve up front so on_advertisement()'s push_back never allocates inside the
  // FreeRTOS critical section (BLE task context).
  this->ingest_queue_.reserve(MAX_INGEST_QUEUE_SIZE);
  this->ingest_drain_.reserve(MAX_INGEST_QUEUE_SIZE);
  // Wire the hub's raw-advertisement stream into this proxy (BLE RTOS task context).
  this->hub_->set_proxy_scan_result_callback(
      [this](const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len) {
        this->on_advertisement(mac, rssi, addr_type, data, data_len);
      });
}

void BluetoothProxy::dump_config() {
  ESP_LOGCONFIG(TAG, "Bluetooth Proxy:");
  char mac[18];
  this->get_bluetooth_mac_address_pretty(mac);
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac);
  ESP_LOGCONFIG(TAG, "  Scan mode: %s", this->hub_->proxy_scan_active() ? "active" : "passive");
}

void BluetoothProxy::get_bluetooth_mac_address_pretty(char output[18]) {
  uint8_t mac[6];
  this->hub_->get_proxy_mac(mac);  // hub returns printable (MSB-first) order
  snprintf(output, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void BluetoothProxy::subscribe_api_connection(api::APIConnection *conn, uint32_t /*flags*/) {
  this->api_connection_ = conn;
  ESP_LOGI(TAG, "Home Assistant subscribed to BLE advertisements");

  // Do not start scanning here — exactly like the ESP32 proxy. Scanning is driven by the
  // hub's scan_parameters and the user's api: start_scan automation. Report current state.
  api::BluetoothScannerStateResponse state_resp;
  state_resp.state = this->hub_->proxy_scan_running() ? api::enums::BLUETOOTH_SCANNER_STATE_RUNNING
                                                      : api::enums::BLUETOOTH_SCANNER_STATE_IDLE;
  state_resp.mode = this->hub_->proxy_scan_active() ? api::enums::BLUETOOTH_SCANNER_MODE_ACTIVE
                                                    : api::enums::BLUETOOTH_SCANNER_MODE_PASSIVE;
  state_resp.configured_mode = state_resp.mode;
  conn->send_message(state_resp);
}

void BluetoothProxy::unsubscribe_api_connection(api::APIConnection *conn) {
  if (this->api_connection_ != conn)
    return;
  this->api_connection_ = nullptr;
  // Discard buffered advertisements so they don't leak to the next subscriber.
  vTaskSuspendAll();
  this->ingest_queue_.clear();
  xTaskResumeAll();
  ESP_LOGI(TAG, "Home Assistant unsubscribed from BLE advertisements");
}

// BLE RTOS task context: buffer the advertisement; loop() drains it on the main task.
// Does NOT touch api_connection_ or the shared response_ here (those are main-task owned).
void BluetoothProxy::on_advertisement(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                      uint16_t data_len) {
  QueuedAdv qa;
  qa.address = 0;
  for (int i = 0; i < 6; i++)
    qa.address |= static_cast<uint64_t>(mac[i]) << (i * 8);
  qa.rssi = static_cast<int32_t>(rssi);
  qa.address_type = addr_type;
  qa.data_len = (data_len <= sizeof(qa.data)) ? data_len : static_cast<uint16_t>(sizeof(qa.data));
  memcpy(qa.data, data, qa.data_len);

  vTaskSuspendAll();
  if (this->ingest_queue_.size() < MAX_INGEST_QUEUE_SIZE) {
    this->ingest_queue_.push_back(qa);
  } else {
    this->ingest_dropped_++;
  }
  xTaskResumeAll();
}

void BluetoothProxy::loop() {
  if (this->ingest_queue_.empty())
    return;

  // Swap the BLE-task buffer out under a critical section, then dispatch on the main task.
  vTaskSuspendAll();
  this->ingest_drain_.swap(this->ingest_queue_);
  uint16_t dropped = this->ingest_dropped_;
  this->ingest_dropped_ = 0;
  xTaskResumeAll();

  // Feed the SHARED batching core (BluetoothProxyAdvertisements) on the main task — the
  // exact same add_advertisement()/flush path the ESP32 proxy uses. add_advertisement()
  // discards if no API client is subscribed.
  for (const auto &qa : this->ingest_drain_)
    this->add_advertisement(qa.address, qa.rssi, qa.address_type, qa.data, qa.data_len);
  this->flush_pending_advertisements_();
  this->ingest_drain_.clear();

  if (dropped != 0)
    ESP_LOGW(TAG, "Advertisement ingest overflow; %u dropped", dropped);
}

}  // namespace esphome::bluetooth_proxy

#endif  // !USE_ESP32
