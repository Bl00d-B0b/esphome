// ln882h_ble_tracker.h
//
// ESPHome BLE scanner for LN882H (LibreTiny lightning family).
// Scans for BLE advertisements using the LN882H BLE 5.1 SDK and delivers
// parsed ESPBTDevice objects to registered listeners (bthome_mithermometer,
// ble_presence, …) and raw results to the optional bluetooth proxy.
//
// YAML config (values shown are the defaults; interval/window are the LN882H
// SDK's recommended scan parameters — 50 % duty cycle):
//
//   ln882h_ble_tracker:
//     scan_parameters:
//       interval:   100ms
//       window:      50ms
//       active:      true
//       duration:    5min
//       continuous:  true
//     bluetooth_proxy: false

#pragma once

#ifdef USE_LIBRETINY

#include "esphome/components/ble_device_base/ble_device.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <functional>
#include <stdint.h>
#include <utility>
#include <vector>

// component.h (above) pulls in esphome/core/defines.h, which defines USE_BLUETOOTH_PROXY
// when the separate bluetooth_proxy component is configured. The hub interface — and the
// BleProxyHub base below — are compiled only then, so the scanner builds stand-alone
// (BLE sensors, no proxy) without needing the bluetooth_proxy component present.
#ifdef USE_BLUETOOTH_PROXY
#include "esphome/components/bluetooth_proxy/bluetooth_proxy_hub.h"
#endif

namespace esphome::ln882h_ble_tracker {

// Callback invoked on every raw BLE advertisement (from BLE RTOS task context).
using ScanResultCallback =
    std::function<void(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len)>;

// ---------------------------------------------------------------------------
// QueuedAdv — a single BLE advertisement buffered from rw_task for the main loop
// ---------------------------------------------------------------------------
struct QueuedAdv {
  uint8_t mac[6];
  int rssi;
  uint8_t addr_type;
  uint16_t data_len;
  uint8_t data[64];  // max BLE adv payload is 31 bytes (legacy) or 255 (extended); 64 covers all legacy cases
  // Unmatched scan-response frame: forward to the Bluetooth proxy only, never to local
  // sensors/triggers (HA merges per address). Set by on_scan_rsp(), honored in loop().
  bool proxy_only{false};
};

// Maximum number of advertisements buffered between rw_task and loop().
// Matches ESP32's MAX_BLE_QUEUE_SIZE (88 without PSRAM, 100 with PSRAM).
// LN882H has no PSRAM, so we use the same value as ESP32 without PSRAM.
static constexpr size_t MAX_ADV_QUEUE_SIZE = 88;

// ---------------------------------------------------------------------------
// Trigger class forward declarations
// ---------------------------------------------------------------------------

class ESPBTAdvertiseTrigger;
class BLEServiceDataAdvertiseTrigger;
class BLEManufacturerDataAdvertiseTrigger;
class BLEEndOfScanTrigger;

// ---------------------------------------------------------------------------
// LN882HBLETracker
// ---------------------------------------------------------------------------

#ifdef USE_BLUETOOTH_PROXY
class LN882HBLETracker : public Component, public bluetooth_proxy::BleProxyHub {
#else
class LN882HBLETracker : public Component {
#endif
 public:
  // ---- ESPHome Component ----
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ---- YAML configuration setters ----
  void set_scan_active(bool scan_active) { this->scan_active_ = scan_active; }
  void set_scan_interval(uint32_t scan_interval) { this->scan_interval_ = scan_interval; }
  void set_scan_window(uint32_t scan_window) { this->scan_window_ = scan_window; }
  void set_scan_duration(uint32_t scan_duration) { this->scan_duration_ = scan_duration; }
  void set_scan_continuous(bool scan_continuous) { this->scan_continuous_ = scan_continuous; }

  // ---- Public actions (callable from YAML automations) ----
  // Mirrors esp32_ble_tracker: set_scan_continuous() + start_scan() / stop_scan().
  void start_scan();
  void stop_scan();

  // ---- Public accessors (used by the optional, separate bluetooth_proxy proxy) ----
  // ble_mac_ is stored MSB-first (printable order).
  void get_mac(uint8_t out[6]) const { memcpy(out, this->ble_mac_, 6); }
  bool is_scan_active() const { return this->scan_active_; }
  bool is_scan_running() const { return this->scan_running_; }
  void set_scan_result_callback(ScanResultCallback cb) { this->scan_result_callback_ = std::move(cb); }

#ifdef USE_BLUETOOTH_PROXY
  // ---- bluetooth_proxy::BleProxyHub interface ----
  // The proxy is hub-agnostic; these forward the LN scanner's state/MAC/callback.
  // Compiled only when the separate bluetooth_proxy component is configured.
  void get_proxy_mac(uint8_t out[6]) override { memcpy(out, this->ble_mac_, 6); }  // already MSB-first
  bool proxy_scan_running() override { return this->scan_running_; }
  bool proxy_scan_active() override { return this->scan_active_; }
  void set_proxy_scan_result_callback(bluetooth_proxy::ScanResultCallback cb) override {
    this->scan_result_callback_ = std::move(cb);
  }
#endif  // USE_BLUETOOTH_PROXY

  // Register a parsed-advertisement listener (bthome_mithermometer, ble_presence, …).
  void register_listener(esp32_ble_tracker::ESPBTDeviceListener *listener) { this->listeners_.push_back(listener); }

  // Register automation triggers (called from each trigger's constructor).
  void register_adv_trigger(ESPBTAdvertiseTrigger *trigger) { this->adv_triggers_.push_back(trigger); }
  void register_svc_data_trigger(BLEServiceDataAdvertiseTrigger *trigger) {
    this->svc_data_triggers_.push_back(trigger);
  }
  void register_mfr_data_trigger(BLEManufacturerDataAdvertiseTrigger *trigger) {
    this->mfr_data_triggers_.push_back(trigger);
  }
  void register_scan_end_trigger(BLEEndOfScanTrigger *trigger) { this->scan_end_triggers_.push_back(trigger); }

  // ---- Called from static BLE scan callback (BLE RTOS task context) ----
  void on_scan_result(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);
  // Unmatched scan-response frames (no pending advertisement to merge with):
  // forwarded to the Bluetooth proxy only (HA merges per address).
  void on_scan_rsp(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);
  // Bluedroid-style adv + scan-response merging (ESP-IDF concatenates both into
  // one result before ESPHome sees it; the LN controller reports them separately):
  // a scannable advertisement is held here briefly, its scan response is appended
  // on arrival and the pair is delivered as ONE merged frame. Held entries whose
  // scan response never arrives are flushed by loop() after PENDING_ADV_TIMEOUT_MS.
  void stash_adv(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);
  void deliver_scan_rsp(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);

  // Log a newly-seen device once (deduplicated via already_discovered_), at DEBUG.
  // Identical behaviour and format to esp32_ble_tracker::ESP32BLETracker.
  void print_bt_device_info(const esp32_ble_tracker::ESPBTDevice &device);

 protected:
  // Copy one advertisement into adv_queue_ under the scheduler lock (rw_task or main task).
  // proxy_only frames are forwarded to the Bluetooth proxy but never parsed for local sensors.
  void enqueue_adv_(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len,
                    bool proxy_only);
  void ble_stack_init_();
  void start_scan_();
  void restart_scan_hw_();
  void stop_scan_();
  void resolve_mac_();

  bool ble_stack_ready_{false};
  bool scan_running_{false};
  bool scan_active_{false};
  // Defaults are the LN882H SDK's recommended scan parameters
  // (ln_ble_scan.h: SCAN_INTERVAL_DEF 0xA0, SCAN_WINDOW_DEF 0x50 → 50 % duty).
  uint32_t scan_interval_{160};  // 160 × 0.625 ms = 100 ms (SDK SCAN_INTERVAL_DEF)
  uint32_t scan_window_{80};     // 80 × 0.625 ms = 50 ms (SDK SCAN_WINDOW_DEF; 50/100 = 50 %)
  uint32_t scan_duration_{300000};
  bool scan_continuous_{true};
  uint32_t scan_start_time_{0};
  uint8_t ble_mac_[6]{0};

  // Pending scannable advertisements awaiting their scan response (active scan).
  // 62 bytes = legacy adv (31) + scan response (31), the same merged maximum as
  // ESP-IDF delivers on ESP32.
  struct PendingAdv {
    bool used{false};
    uint8_t mac[6];
    uint8_t addr_type;
    int rssi;
    uint16_t data_len;
    uint8_t data[62];
    uint32_t stored_ms;
  };
  static constexpr size_t MAX_PENDING_ADV = 4;
  // On air a scan response follows its advertisement by T_IFS (150 µs) — the
  // timeout only covers HOST-side report queuing in rw_task under WiFi/BLE
  // coexistence, measured on-device at up to ~136 ms. 300 ms = >2x that margin,
  // while staying below any device's re-advertising period.
  static constexpr uint32_t PENDING_ADV_TIMEOUT_MS = 300;
  PendingAdv pending_adv_[MAX_PENDING_ADV];

  uint32_t last_scan_result_time_{0};  // millis() of most-recent advertisement; 0 = none yet
  uint8_t watchdog_restart_count_{0};  // consecutive watchdog restarts with no results (capped, no reboot)
  uint32_t scan_period_start_{0};      // millis() at start of current scan period; used to rate-limit on_scan_end()
  bool scan_started_once_{false};      // true after first successful scan start; suppresses repeated LOGI noise

  ScanResultCallback scan_result_callback_{nullptr};
  std::vector<esp32_ble_tracker::ESPBTDeviceListener *> listeners_{};

  // Addresses already printed by print_bt_device_info() — mirrors ESP32's dedup list.
  std::vector<uint64_t> already_discovered_{};

  // Advertisement queue: filled by rw_task (on_scan_result), drained by loop().
  // Protected by a FreeRTOS critical section (taskENTER_CRITICAL / EXIT_CRITICAL).
  // Capped at MAX_ADV_QUEUE_SIZE; overflow increments adv_dropped_count_ (same as ESP32).
  std::vector<QueuedAdv> adv_queue_{};
  std::vector<QueuedAdv> adv_drain_{};  // swap buffer — avoids holding critical section during dispatch
  uint16_t adv_dropped_count_{0};       // incremented under critical section; logged and reset in loop()

  std::vector<ESPBTAdvertiseTrigger *> adv_triggers_{};
  std::vector<BLEServiceDataAdvertiseTrigger *> svc_data_triggers_{};
  std::vector<BLEManufacturerDataAdvertiseTrigger *> mfr_data_triggers_{};
  std::vector<BLEEndOfScanTrigger *> scan_end_triggers_{};
};

// ---------------------------------------------------------------------------
// YAML automation actions: ln882h_ble_tracker.start_scan / .stop_scan
// ---------------------------------------------------------------------------

template<typename... Ts> class StartScanAction : public Action<Ts...>, public Parented<LN882HBLETracker> {
 public:
  void set_continuous(bool continuous) { this->continuous_ = continuous; }
  void play(const Ts &...x) override {
    // Mirrors esp32_ble_tracker: set flag first, then start if idle.
    this->parent_->set_scan_continuous(this->continuous_);
    if (!this->parent_->is_scan_running())
      this->parent_->start_scan();
  }

 protected:
  bool continuous_{true};
};

template<typename... Ts> class StopScanAction : public Action<Ts...>, public Parented<LN882HBLETracker> {
 public:
  void play(const Ts &...x) override { this->parent_->stop_scan(); }
};

// ---------------------------------------------------------------------------
// on_ble_advertise trigger
//
// Fires on every BLE advertisement, optionally filtered to one or more MACs.
// ---------------------------------------------------------------------------

class ESPBTAdvertiseTrigger : public Trigger<const esp32_ble_tracker::ESPBTDevice &> {
 public:
  explicit ESPBTAdvertiseTrigger(LN882HBLETracker *parent) { parent->register_adv_trigger(this); }

  void set_addresses(std::vector<uint64_t> addresses) { this->addresses_ = std::move(addresses); }

  // Returns true if the device was claimed (matched + fired), mirroring
  // esp32_ble_tracker::ESPBTAdvertiseTrigger::parse_device — the tracker uses this to
  // set `found`, suppressing the "Found device" log exactly like ESP32.
  bool process(const esp32_ble_tracker::ESPBTDevice &device) {
    if (!this->addresses_.empty()) {
      uint64_t addr = device.address_uint64();
      bool match = false;
      for (auto a : this->addresses_) {
        if (a == addr) {
          match = true;
          break;
        }
      }
      if (!match) {
        return false;
      }
    }
    this->trigger(device);
    return true;
  }

 protected:
  std::vector<uint64_t> addresses_{};
};

// ---------------------------------------------------------------------------
// on_ble_service_data_advertise trigger
//
// Fires when a BLE advertisement contains service data for the given UUID.
// Optional single-MAC filter.
// ---------------------------------------------------------------------------

class BLEServiceDataAdvertiseTrigger : public Trigger<const esp32_ble_tracker::adv_data_t &> {
 public:
  explicit BLEServiceDataAdvertiseTrigger(LN882HBLETracker *parent) { parent->register_svc_data_trigger(this); }

  void set_service_uuid16(uint64_t uuid) {
    this->uuid_ = esp32_ble_tracker::ESPBTUUID::from_uint16(static_cast<uint16_t>(uuid));
  }
  void set_service_uuid32(uint64_t uuid) {
    this->uuid_ = esp32_ble_tracker::ESPBTUUID::from_uint32(static_cast<uint32_t>(uuid));
  }
  void set_service_uuid128(const uint8_t *uuid) { this->uuid_ = esp32_ble_tracker::ESPBTUUID::from_raw(uuid); }

  void set_address(uint64_t address) {
    this->address_ = address;
    this->has_address_ = true;
  }

  bool process(const esp32_ble_tracker::ESPBTDevice &device) {
    if (this->has_address_ && device.address_uint64() != this->address_) {
      return false;
    }
    for (const auto &sd : device.get_service_datas()) {
      if (sd.uuid == this->uuid_) {
        this->trigger(sd.data);
        return true;
      }
    }
    return false;
  }

 protected:
  esp32_ble_tracker::ESPBTUUID uuid_{};
  uint64_t address_{0};
  bool has_address_{false};
};

// ---------------------------------------------------------------------------
// on_ble_manufacturer_data_advertise trigger
//
// Fires when a BLE advertisement contains manufacturer data for the given ID.
// Optional single-MAC filter.
// ---------------------------------------------------------------------------

class BLEManufacturerDataAdvertiseTrigger : public Trigger<const esp32_ble_tracker::adv_data_t &> {
 public:
  explicit BLEManufacturerDataAdvertiseTrigger(LN882HBLETracker *parent) { parent->register_mfr_data_trigger(this); }

  void set_manufacturer_uuid16(uint64_t uuid) {
    this->uuid_ = esp32_ble_tracker::ESPBTUUID::from_uint16(static_cast<uint16_t>(uuid));
  }
  void set_manufacturer_uuid32(uint64_t uuid) {
    this->uuid_ = esp32_ble_tracker::ESPBTUUID::from_uint32(static_cast<uint32_t>(uuid));
  }
  void set_manufacturer_uuid128(const uint8_t *uuid) { this->uuid_ = esp32_ble_tracker::ESPBTUUID::from_raw(uuid); }

  void set_address(uint64_t address) {
    this->address_ = address;
    this->has_address_ = true;
  }

  bool process(const esp32_ble_tracker::ESPBTDevice &device) {
    if (this->has_address_ && device.address_uint64() != this->address_) {
      return false;
    }
    for (const auto &md : device.get_manufacturer_datas()) {
      if (md.uuid == this->uuid_) {
        this->trigger(md.data);
        return true;
      }
    }
    return false;
  }

 protected:
  esp32_ble_tracker::ESPBTUUID uuid_{};
  uint64_t address_{0};
  bool has_address_{false};
};

// ---------------------------------------------------------------------------
// on_scan_end trigger
//
// Fires whenever a scan period ends (duration elapsed or stop_scan called).
// ---------------------------------------------------------------------------

class BLEEndOfScanTrigger : public Trigger<> {
 public:
  explicit BLEEndOfScanTrigger(LN882HBLETracker *parent) { parent->register_scan_end_trigger(this); }
};

}  // namespace esphome::ln882h_ble_tracker

#endif  // USE_LIBRETINY
