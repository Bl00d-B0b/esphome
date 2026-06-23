// bluetooth_proxy_compat.h
//
// Compatibility shim required by ESPHome's api_connection.cpp on non-ESP32 targets.
// Part of the esphome::bluetooth_proxy_base component (shared by the LibreTiny BLE
// hubs: bk72xx_ble_tracker, ln882h_ble_tracker, …).
//
// api_connection.cpp calls into bluetooth_proxy::BluetoothProxy (and reads
// global_bluetooth_proxy) whenever USE_BLUETOOTH_PROXY is defined.  On ESP32 the
// real bluetooth_proxy.h provides these declarations; on LibreTiny that header is
// guarded by #ifdef USE_ESP32 and compiles away to nothing.
//
// This file fills that gap: it declares the abstract interface and the global
// pointer so api_connection.cpp links correctly off-ESP32.  The shared concrete
// implementation is bluetooth_proxy_base::BluetoothProxyBase; per-hub thin
// subclasses (BK72xxBluetoothProxy, Ln882hBluetoothProxy) extend it.
//
// On ESP32 this file is a no-op — the real bluetooth_proxy.h takes precedence.

#pragma once

#ifndef USE_ESP32

#include <cstdint>

// Forward-declare only what api_connection.cpp uses through global_bluetooth_proxy.
// Full definitions come from api_pb2.h, which api_connection.cpp includes itself.
namespace esphome {
namespace api {
class APIConnection;
struct BluetoothDeviceRequest;
struct BluetoothGATTReadRequest;
struct BluetoothGATTWriteRequest;
struct BluetoothGATTReadDescriptorRequest;
struct BluetoothGATTWriteDescriptorRequest;
struct BluetoothGATTGetServicesRequest;
struct BluetoothGATTNotifyRequest;
struct BluetoothSetConnectionParamsRequest;
}  // namespace api

namespace bluetooth_proxy {

// Minimal abstract interface that satisfies all call-sites in api_connection.cpp.
// bluetooth_proxy_base::BluetoothProxyBase implements it; per-hub proxies extend that.
class BluetoothProxy {
 public:
  virtual ~BluetoothProxy() = default;

  // Called by api_connection.cpp → DeviceInfoResponse
  virtual uint32_t get_feature_flags() const = 0;
  virtual void get_bluetooth_mac_address_pretty(char output[18]) = 0;

  // Returns the currently-subscribed connection (nullptr if none).
  // Called by api_connection.cpp's destructor to detect stale references.
  virtual api::APIConnection *get_api_connection() const { return nullptr; }

  // Called when HA subscribes / unsubscribes to BLE advertisements
  virtual void subscribe_api_connection(api::APIConnection *conn, uint32_t flags) = 0;
  virtual void unsubscribe_api_connection(api::APIConnection *conn) = 0;

  // GATT / active-connection stubs — NOT implemented on LibreTiny (advert proxy only).
  // api_connection.cpp declares these on the interface; they are never called when
  // FEATURE_ACTIVE_CONNECTIONS is absent from get_feature_flags().
  virtual void bluetooth_device_request(const api::BluetoothDeviceRequest &) {}
  virtual void bluetooth_gatt_read(const api::BluetoothGATTReadRequest &) {}
  virtual void bluetooth_gatt_write(const api::BluetoothGATTWriteRequest &) {}
  virtual void bluetooth_gatt_read_descriptor(const api::BluetoothGATTReadDescriptorRequest &) {}
  virtual void bluetooth_gatt_write_descriptor(const api::BluetoothGATTWriteDescriptorRequest &) {}
  virtual void bluetooth_gatt_send_services(const api::BluetoothGATTGetServicesRequest &) {}
  virtual void bluetooth_gatt_notify(const api::BluetoothGATTNotifyRequest &) {}
  virtual void bluetooth_set_connection_params(const api::BluetoothSetConnectionParamsRequest &) {}
  virtual void send_connections_free() {}
  virtual void send_connections_free(api::APIConnection *) {}

  // Scanner mode toggle (passive ↔ active)
  virtual void bluetooth_scanner_set_mode(bool /*active*/) {}
};

// Defined in bluetooth_proxy_base.cpp; assigned by each hub proxy's setup().
extern BluetoothProxy *global_bluetooth_proxy;  // NOLINT

}  // namespace bluetooth_proxy

namespace bluetooth_proxy_base {
// The interface above is declared in namespace esphome::bluetooth_proxy — the name
// api_connection.cpp expects on ESP32 — so it cannot live in this component's own
// namespace. Expose it here under the component name as well: BluetoothProxyBase and
// the per-hub proxies in this component derive from this type.
using BluetoothProxy = bluetooth_proxy::BluetoothProxy;
}  // namespace bluetooth_proxy_base
}  // namespace esphome

#endif  // !USE_ESP32
