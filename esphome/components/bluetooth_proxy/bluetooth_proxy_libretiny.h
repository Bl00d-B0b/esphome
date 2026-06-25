#pragma once

// bluetooth_proxy_libretiny.h
//
// The LibreTiny (non-ESP32) face of the hub-agnostic bluetooth_proxy. Same component,
// same class name (esphome::bluetooth_proxy::BluetoothProxy) and same API surface as the
// ESP32 proxy, so api_connection.cpp talks to one type on every platform. It inherits the
// shared BluetoothProxyAdvertisements core (identical batching/flush as ESP32) and is fed
// raw advertisements by a tracker hub (BleProxyHub). GATT active connections are ESP32-only,
// so those API entry points are no-ops here.
//
// Threading: the tracker delivers advertisements from the BLE RTOS task. on_advertisement()
// buffers them under a FreeRTOS critical section; loop() drains that buffer on the main task
// and feeds add_advertisement(), so the shared batching core stays single-threaded — exactly
// like ESP32, where esp32_ble_tracker already delivers on the main loop.

#ifndef USE_ESP32

#include "bluetooth_proxy_advertisements.h"
#include "bluetooth_proxy_hub.h"
#include "esphome/components/api/api_pb2.h"
#include "esphome/core/component.h"

#include <cstdint>
#include <vector>

namespace esphome::api {
class APIConnection;
}  // namespace esphome::api

namespace esphome::bluetooth_proxy {

class BluetoothProxy : public BluetoothProxyAdvertisements, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  // The tracker hub that supplies advertisements, adapter MAC and scan state.
  void set_hub(BleProxyHub *hub) { this->hub_ = hub; }

  // ---- bluetooth_proxy::BluetoothProxy API surface (called by api_connection.cpp) ----
  uint32_t get_feature_flags() const {
    // Advertisement proxy only: active GATT connections are ESP32-only.
    return FEATURE_PASSIVE_SCAN | FEATURE_RAW_ADVERTISEMENTS | FEATURE_STATE_AND_MODE;
  }
  void get_bluetooth_mac_address_pretty(char output[18]);
  void subscribe_api_connection(api::APIConnection *conn, uint32_t flags);
  void unsubscribe_api_connection(api::APIConnection *conn);

  // GATT / active-connection entry points — no-ops off-ESP32 (never reached while
  // FEATURE_ACTIVE_CONNECTIONS is absent), present only so api_connection.cpp links.
  void bluetooth_device_request(const api::BluetoothDeviceRequest &) {}
  void bluetooth_gatt_read(const api::BluetoothGATTReadRequest &) {}
  void bluetooth_gatt_write(const api::BluetoothGATTWriteRequest &) {}
  void bluetooth_gatt_read_descriptor(const api::BluetoothGATTReadDescriptorRequest &) {}
  void bluetooth_gatt_write_descriptor(const api::BluetoothGATTWriteDescriptorRequest &) {}
  void bluetooth_gatt_send_services(const api::BluetoothGATTGetServicesRequest &) {}
  void bluetooth_gatt_notify(const api::BluetoothGATTNotifyRequest &) {}
  void bluetooth_set_connection_params(const api::BluetoothSetConnectionParamsRequest &) {}
  void bluetooth_scanner_set_mode(bool /*active*/) {}
  void send_connections_free() {}
  void send_connections_free(api::APIConnection *) {}

  // Called by the hub from the BLE RTOS task for each raw advertisement.
  void on_advertisement(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);

 protected:
  // One raw advertisement buffered from the BLE task for the main loop.
  struct QueuedAdv {
    uint64_t address;
    int32_t rssi;
    uint8_t address_type;
    uint16_t data_len;
    uint8_t data[62];  // legacy adv (31) + scan response (31)
  };
  // Matches ESP32's MAX_BLE_QUEUE_SIZE without PSRAM.
  static constexpr size_t MAX_INGEST_QUEUE_SIZE = 88;

  BleProxyHub *hub_{nullptr};
  // Filled by the BLE task (on_advertisement), drained by loop(). Guarded by a FreeRTOS
  // critical section; swap buffer avoids holding it during dispatch.
  std::vector<QueuedAdv> ingest_queue_{};
  std::vector<QueuedAdv> ingest_drain_{};
  uint16_t ingest_dropped_{0};
};

// Looked up by api_connection.cpp to route BLE subscription/GATT messages.
extern BluetoothProxy *global_bluetooth_proxy;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

}  // namespace esphome::bluetooth_proxy

#endif  // !USE_ESP32
