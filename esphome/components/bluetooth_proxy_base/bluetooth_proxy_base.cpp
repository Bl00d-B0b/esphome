// bluetooth_proxy_base.cpp
//
// Shared advertisement-forwarding implementation for LibreTiny BLE hubs.
// See bluetooth_proxy_base.h for the design.

#ifndef USE_ESP32

// component.h first: it pulls in esphome/core/defines.h which sets
// USE_BLUETOOTH_PROXY before the #ifdef guard in bluetooth_proxy_base.h is evaluated.
#include "esphome/core/component.h"
#include "bluetooth_proxy_base.h"
#include "esphome/core/log.h"

// global_bluetooth_proxy is looked up by api_connection.cpp to route BLE
// subscription messages. Defined here (shared) regardless of USE_BLUETOOTH_PROXY,
// matching the symbol api_connection.cpp expects whenever the proxy is compiled in.
namespace esphome::bluetooth_proxy {
BluetoothProxy *global_bluetooth_proxy = nullptr;  // NOLINT
}  // namespace esphome::bluetooth_proxy

#ifdef USE_BLUETOOTH_PROXY

#include "FreeRTOS.h"
#include "task.h"

namespace esphome::bluetooth_proxy_base {

static const char *const TAG = "bluetooth_proxy_base";

void BluetoothProxyBase::setup() {
  bluetooth_proxy::global_bluetooth_proxy = this;
  // Wire the hub's raw-advertisement stream into this proxy (BLE RTOS task context).
  this->hub_->set_proxy_scan_result_callback(
      [this](const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len) {
        this->on_advertisement(mac, rssi, addr_type, data, data_len);
      });
}

void BluetoothProxyBase::dump_config() {
  ESP_LOGCONFIG(TAG, "Bluetooth Proxy:");
  char mac[18];
  this->get_bluetooth_mac_address_pretty(mac);
  ESP_LOGCONFIG(TAG, "  MAC: %s", mac);
  ESP_LOGCONFIG(TAG, "  Scan mode: %s", this->hub_->proxy_scan_active() ? "active" : "passive");
}

uint32_t BluetoothProxyBase::get_feature_flags() const {
  // Advertisement proxy. Active GATT connections are not supported on LibreTiny
  // hubs, so FEATURE_ACTIVE_CONNECTIONS is intentionally not set.
  return FEATURE_PASSIVE_SCAN | FEATURE_RAW_ADVERTISEMENTS | FEATURE_STATE_AND_MODE;
}

void BluetoothProxyBase::get_bluetooth_mac_address_pretty(char output[18]) {
  uint8_t mac[6];
  this->hub_->get_proxy_mac(mac);  // hub returns printable (MSB-first) order
  snprintf(output, 18, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void BluetoothProxyBase::subscribe_api_connection(api::APIConnection *conn, uint32_t /*flags*/) {
  this->api_connection_ = conn;
  ESP_LOGI(TAG, "Home Assistant subscribed to BLE advertisements");

  // Do not start scanning here — exactly like the ESP32 bluetooth_proxy. Scanning is
  // driven by the hub's scan_parameters and the user's api: start_scan automation.

  // Report current scanner state to HA.
  api::BluetoothScannerStateResponse state_resp;
  state_resp.state = this->hub_->proxy_scan_running() ? api::enums::BLUETOOTH_SCANNER_STATE_RUNNING
                                                      : api::enums::BLUETOOTH_SCANNER_STATE_IDLE;
  state_resp.mode = this->hub_->proxy_scan_active() ? api::enums::BLUETOOTH_SCANNER_MODE_ACTIVE
                                                    : api::enums::BLUETOOTH_SCANNER_MODE_PASSIVE;
  state_resp.configured_mode = state_resp.mode;
  conn->send_message(state_resp);
}

void BluetoothProxyBase::unsubscribe_api_connection(api::APIConnection *conn) {
  if (this->api_connection_ != conn)
    return;
  // Mirror the ESP32 proxy: do not stop scanning on unsubscribe; that is driven by
  // the user's api: on_client_disconnected stop_scan automation.
  this->api_connection_ = nullptr;
  vTaskSuspendAll();
  this->fill_len_ = 0;  // discard buffered advertisements; don't leak to the next subscriber
  xTaskResumeAll();
  ESP_LOGI(TAG, "Home Assistant unsubscribed from BLE advertisements");
}

// Called from the BLE RTOS task: buffer the advertisement; the main loop sends.
// Deliberately does NOT read api_connection_ here — that pointer is owned by the
// main task (subscribe/unsubscribe/loop), so the BLE task never races on it; an
// unsubscribed buffer is simply discarded by loop()/unsubscribe.
void BluetoothProxyBase::on_advertisement(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                          uint16_t data_len) {
  vTaskSuspendAll();
  if (this->fill_len_ >= BLUETOOTH_PROXY_ADVERTISEMENT_BATCH_SIZE) {
    this->batch_dropped_count_++;
    xTaskResumeAll();
    return;
  }
  auto &adv = this->responses_[this->fill_idx_].advertisements[this->fill_len_];

  adv.address = 0;
  for (int i = 0; i < 6; i++)
    adv.address |= static_cast<uint64_t>(mac[i]) << (i * 8);

  adv.rssi = static_cast<int32_t>(rssi);
  adv.address_type = addr_type;

  uint16_t copy_len = (data_len <= sizeof(adv.data)) ? data_len : static_cast<uint16_t>(sizeof(adv.data));
  memcpy(adv.data, data, copy_len);
  adv.data_len = copy_len;

  this->fill_len_++;
  xTaskResumeAll();
}

void BluetoothProxyBase::loop() {
  // Benign racy peek (fill_len_ is volatile): only the BLE task increments it, only
  // this task zeroes it — a stale read just defers the flush by one pass.
  if (this->fill_len_ == 0)
    return;

  vTaskSuspendAll();
  auto &resp = this->responses_[this->fill_idx_];
  resp.advertisements_len = this->fill_len_;
  this->fill_idx_ ^= 1;
  this->fill_len_ = 0;
  uint32_t dropped = this->batch_dropped_count_;
  this->batch_dropped_count_ = 0;
  xTaskResumeAll();

  if (this->api_connection_ != nullptr)
    this->api_connection_->send_message(resp);

  if (dropped != 0)
    ESP_LOGW(TAG, "Advertisement batch overflow; %lu dropped", static_cast<unsigned long>(dropped));
}

}  // namespace esphome::bluetooth_proxy_base

#endif  // USE_BLUETOOTH_PROXY

#endif  // !USE_ESP32
