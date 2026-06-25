#pragma once

// bluetooth_proxy_hub.h
//
// Neutral hub interface that a LibreTiny BLE tracker (bk72xx_ble_tracker,
// ln882h_ble_tracker, …) implements so the generic, chip-agnostic BluetoothProxy can
// forward its advertisements without knowing the chip. Lightweight and NOT gated on
// USE_BLUETOOTH_PROXY, so a tracker can implement it whether or not a proxy is configured.
//
// On ESP32 the proxy gets its advertisements from esp32_ble_tracker directly, so this
// interface is only used off-ESP32.

#ifndef USE_ESP32

#include <cstdint>
#include <functional>

namespace esphome::bluetooth_proxy {

// Raw BLE advertisement callback — invoked by the hub from the BLE RTOS task.
using ScanResultCallback =
    std::function<void(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len)>;

// Implemented by each LibreTiny tracker hub (BK72xxBLETracker, LN882HBLETracker, …).
class BleProxyHub {
 public:
  virtual ~BleProxyHub() = default;

  // Write the adapter MAC in printable (MSB-first) order into out[0..5].
  virtual void get_proxy_mac(uint8_t out[6]) = 0;

  // Whether the hub's scanner is currently running.
  virtual bool proxy_scan_running() = 0;

  // Whether the hub is scanning actively (false on passive-only hubs, e.g. BK7231N).
  virtual bool proxy_scan_active() = 0;

  // Register the callback the hub invokes for every raw advertisement.
  virtual void set_proxy_scan_result_callback(ScanResultCallback cb) = 0;
};

}  // namespace esphome::bluetooth_proxy

#endif  // !USE_ESP32
