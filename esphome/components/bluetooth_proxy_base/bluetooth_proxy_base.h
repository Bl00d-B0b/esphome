// bluetooth_proxy_base.h
//
// Generic, hub-agnostic Bluetooth proxy for LibreTiny BLE hubs (BK7231N /
// bk72xx_ble_tracker, LN882H / ln882h_ble_tracker, …).
//
// A single proxy works with any tracker: the tracker implements the BleProxyHub
// interface (bluetooth_proxy_hub.h) and the proxy is given a pointer to it. The
// proxy buffers raw advertisements from the BLE RTOS task and flushes them to Home
// Assistant from the main loop — at most one API message per loop pass — exactly
// like the ESP32 bluetooth_proxy. There are no per-chip proxy subclasses; only the
// scan source, adapter MAC and scan state differ, and those come from the hub.

#pragma once

#ifndef USE_ESP32

// Always available: the neutral hub interface (implemented by trackers) and the
// off-ESP32 bluetooth_proxy::BluetoothProxy shim that api_connection.cpp needs.
#include "bluetooth_proxy_hub.h"
#include "bluetooth_proxy_compat.h"

#ifdef USE_BLUETOOTH_PROXY

#include "esphome/components/api/api_connection.h"
#include "esphome/components/api/api_pb2.h"
#include "esphome/core/component.h"

#include <cstdint>

namespace esphome::bluetooth_proxy_base {

// BLE feature flags reported to Home Assistant via DeviceInfoResponse.
// Values match the BluetoothProxyFeature enum in ESPHome's bluetooth_proxy.h.
static constexpr uint32_t FEATURE_PASSIVE_SCAN = 1u << 0;
static constexpr uint32_t FEATURE_RAW_ADVERTISEMENTS = 1u << 5;
static constexpr uint32_t FEATURE_STATE_AND_MODE = 1u << 6;

class BluetoothProxyBase : public Component, public bluetooth_proxy::BluetoothProxy {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // The tracker hub that supplies advertisements, MAC and scan state.
  void set_hub(BleProxyHub *hub) { this->hub_ = hub; }

  // Called by the hub (BLE RTOS task context) for each raw advertisement. Buffers
  // into the active double-buffer under a scheduler critical section; the main loop
  // drains and sends. An unsubscribed buffer is simply discarded.
  void on_advertisement(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);

  // ---- bluetooth_proxy::BluetoothProxy interface ----
  uint32_t get_feature_flags() const override;
  void get_bluetooth_mac_address_pretty(char output[18]) override;
  api::APIConnection *get_api_connection() const override { return this->api_connection_; }
  void subscribe_api_connection(api::APIConnection *conn, uint32_t flags) override;
  void unsubscribe_api_connection(api::APIConnection *conn) override;

 protected:
  BleProxyHub *hub_{nullptr};
  api::APIConnection *api_connection_{nullptr};

  // Advertisement batching (mirrors ESP32 bluetooth_proxy): a double buffer so the
  // BLE task keeps filling one side while the main loop serialises the other.
  api::BluetoothLERawAdvertisementsResponse responses_[2];
  uint8_t fill_idx_{0};
  volatile uint8_t fill_len_{0};  // written by the BLE task (under critical), peeked by loop()
  uint32_t batch_dropped_count_{0};
};

}  // namespace esphome::bluetooth_proxy_base

#endif  // USE_BLUETOOTH_PROXY
#endif  // !USE_ESP32
