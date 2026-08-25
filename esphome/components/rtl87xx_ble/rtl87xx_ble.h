// Realtek BLE controller (RTL8720C/D): drives the vendor GAP stack. LibreTiny
// links the SDK's BT libraries and this component drives them - the bk72xx_ble
// arrangement. Hardware-verified on AmebaD; AmebaZ2 is compile-only so far.
//
// Bring-up is async and follows the WiFi STA (coexistence needs it), and there
// is no teardown - bte_deinit() crashes - so the stack stays resident.

#pragma once

#ifdef USE_LIBRETINY

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include <cstdint>

namespace esphome::rtl87xx_ble {

// Realtek T_GAP_ADV_EVT_TYPE values, pinned to the SDK enum by static_asserts
// in rtl87xx_ble.cpp. ADV_IND and ADV_SCAN_IND are the scannable types.
static constexpr uint8_t ADV_EVENT_TYPE_ADV_IND = 0;
static constexpr uint8_t ADV_EVENT_TYPE_ADV_SCAN_IND = 2;
static constexpr uint8_t ADV_EVENT_TYPE_SCAN_RSP = 4;

// Raw scan-report callback, invoked from the GAP task:
// (bd_addr LSB-first, addr_type, adv_type, rssi, data, len).
using raw_adv_callback_t = void (*)(const uint8_t bd_addr[6], uint8_t addr_type, uint8_t adv_type, int8_t rssi,
                                    const uint8_t *data, uint8_t len);

class RTL87xxBLE : public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  bool stack_ready();
  /// True while bring-up is still blocked on the WiFi STA, which coexistence
  /// needs running before the stack can start.
  bool waiting_for_network();
  /// nullptr clears the callback.
  void set_adv_callback(raw_adv_callback_t cb);
  /// Scan timing in milliseconds and mode; a running scan keeps its
  /// parameters until restarted.
  void set_scan_params(uint16_t interval_ms, uint16_t window_ms, bool active);
  /// Start scanning. False when the stack is not ready or the start failed.
  bool scan_start();
  void scan_stop();
  /// True while the stack reports a starting or active scan, read from its own
  /// device state so a controller-side drop shows here.
  bool scan_running();
  /// Bluetooth adapter MAC, printable order (out[0] = MSB); falls back to the
  /// WiFi MAC when the BT eFuse slot is unprogrammed.
  void get_mac(uint8_t out[6]);
};

}  // namespace esphome::rtl87xx_ble

#endif  // USE_LIBRETINY
