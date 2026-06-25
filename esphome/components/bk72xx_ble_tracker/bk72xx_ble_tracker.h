// bk72xx_ble_tracker.h
//
// ESPHome BLE scanner for BK7231N (LibreTiny beken-72xx family).
// Scans for BLE 5.1 advertisements using the Beken BDK BLE SDK and delivers
// parsed ESPBTDevice objects to registered listeners (bthome_mithermometer,
// ble_presence, …) and raw results to the optional bluetooth proxy.
//
// The BLE 5.x stack is already compiled and linked by the LibreTiny beken-72xx
// builder (CFG_SUPPORT_BLE=1, CFG_BLE_VERSION=BLE_VERSION_5_x in the beken-7231n
// sys_config.h; prebuilt libble_bk7231n.a). This component only calls into it —
// no framework patch is required, unlike the LN882H sibling.
//
// YAML config (values shown are the defaults; interval/window are a 30 % duty
// cycle, the BK reference scan rate). The Bluetooth proxy is the separate
// bluetooth_proxy component.
//
//   bk72xx_ble_tracker:
//     scan_parameters:
//       interval:   100ms
//       window:      30ms
//       duration:    5min
//       continuous:  true

#pragma once

#ifndef USE_ESP32

#include "bk72xx_ble_device.h"
#include "esphome/components/bluetooth_proxy/bluetooth_proxy_hub.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

#include <functional>
#include <stdint.h>
#include <utility>
#include <vector>

namespace esphome::bk72xx_ble_tracker {

// Callback invoked on every raw BLE advertisement (from the BLE RTOS task context).
using ScanResultCallback =
    std::function<void(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len)>;

// ---------------------------------------------------------------------------
// QueuedAdv — a single BLE advertisement buffered from the BLE task for the main loop
// ---------------------------------------------------------------------------
struct QueuedAdv {
  uint8_t mac[6];
  int rssi;
  uint8_t addr_type;
  uint16_t data_len;
  uint8_t data[64];  // max legacy BLE adv payload is 31 bytes; 64 covers adv + scan-response
};

// Maximum number of advertisements buffered between the BLE task and loop().
// Matches ESP32's MAX_BLE_QUEUE_SIZE without PSRAM (BK7231N has no PSRAM).
static constexpr size_t MAX_ADV_QUEUE_SIZE = 88;

// ---------------------------------------------------------------------------
// Trigger class forward declarations
// ---------------------------------------------------------------------------

class ESPBTAdvertiseTrigger;
class BLEServiceDataAdvertiseTrigger;
class BLEManufacturerDataAdvertiseTrigger;
class BLEEndOfScanTrigger;

// ---------------------------------------------------------------------------
// BK72xxBLETracker
// ---------------------------------------------------------------------------

class BK72xxBLETracker : public Component, public bluetooth_proxy::BleProxyHub {
 public:
  // ---- ESPHome Component ----
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // ---- YAML configuration setters ----
  void set_scan_interval(uint32_t scan_interval) { this->scan_interval_ = scan_interval; }
  void set_scan_window(uint32_t scan_window) { this->scan_window_ = scan_window; }
  void set_scan_duration(uint32_t scan_duration) { this->scan_duration_ = scan_duration; }
  void set_scan_continuous(bool scan_continuous) { this->scan_continuous_ = scan_continuous; }

  // ---- Public actions (callable from YAML automations) ----
  // Mirrors esp32_ble_tracker: set_scan_continuous() + start_scan() / stop_scan().
  void start_scan();
  void stop_scan();

  // ---- Public accessors (used by the optional, separate bluetooth_proxy proxy) ----
  // ble_mac_ is stored LSB-first (from common_default_bdaddr).
  void get_mac(uint8_t out[6]) const { memcpy(out, this->ble_mac_, 6); }
  bool is_scan_running() const { return this->scan_running_; }
  void set_scan_result_callback(ScanResultCallback cb) { this->scan_result_callback_ = std::move(cb); }

  // ---- bluetooth_proxy::BleProxyHub interface ----
  // The proxy is hub-agnostic; these forward the BK scanner's state/MAC/callback.
  void get_proxy_mac(uint8_t out[6]) override {
    // ble_mac_ is LSB-first; the proxy wants printable MSB-first order.
    for (int i = 0; i < 6; i++)
      out[i] = this->ble_mac_[5 - i];
  }
  bool proxy_scan_running() override { return this->scan_running_; }
  bool proxy_scan_active() override { return false; }  // BK7231N scans passive-only
  void set_proxy_scan_result_callback(bluetooth_proxy::ScanResultCallback cb) override {
    this->scan_result_callback_ = std::move(cb);
  }

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

  // ---- Called from the static BLE notice callback (BLE RTOS task context) ----
  // The BK controller delivers every advertisement (and, in active mode, scan
  // responses) as a BLE_5_REPORT_ADV notice; each is forwarded here directly.
  // Home Assistant merges adv + scan-response per address, so — unlike the LN882H
  // sibling — no host-side merge machinery is needed.
  void on_scan_result(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);

  // Log a newly-seen device once (deduplicated via already_discovered_), at DEBUG.
  // Identical behaviour and format to esp32_ble_tracker::ESP32BLETracker.
  void print_bt_device_info(const esp32_ble_tracker::ESPBTDevice &device);

 protected:
  void ble_stack_init_();
  bool scan_hw_start_();  // start the controller scan; no timer side-effects. true on success.
  void stop_scan_hw_();   // stop the controller scan; does NOT fire on_scan_end().
  void start_scan_();
  void restart_scan_hw_();
  void stop_scan_();
  void resolve_mac_();

  bool ble_stack_ready_{false};
  bool scan_running_{false};
  // Defaults: the BK reference — 30 % duty cycle
  // (interval 100 ms / window 30 ms), in 0.625 ms BLE units.
  uint32_t scan_interval_{160};  // 160 × 0.625 ms = 100 ms
  uint32_t scan_window_{48};     // 48 × 0.625 ms = 30 ms (30/100 = 30 %)
  uint32_t scan_duration_{300000};
  bool scan_continuous_{true};
  uint32_t scan_start_time_{0};
  uint8_t ble_mac_[6]{0};
  // BK BLE activity index for the scan actv, returned by app_ble_get_idle_actv_idx_handle()
  // at scan start and passed to bk_ble_scan_stop(). 0xFF = no active scan.
  uint8_t scan_actv_idx_{0xFF};

  uint32_t last_scan_result_time_{0};    // millis() of most-recent advertisement; 0 = none yet
  uint32_t last_scan_start_attempt_{0};  // millis() of last start_scan_() attempt; rate-limits retries
  uint8_t watchdog_restart_count_{0};    // consecutive watchdog restarts with no results
  // True once any advertisement has ever been received. Gates the watchdog reboot:
  // a scanner that has never seen a single frame is most likely in a BLE-quiet
  // environment, not wedged, so it must not reboot-loop.
  bool ever_received_adv_{false};
  uint32_t scan_period_start_{0};  // millis() at start of current scan period; used to rate-limit on_scan_end()
  bool scan_started_once_{false};  // true after first successful scan start; suppresses repeated LOGI noise

  ScanResultCallback scan_result_callback_{nullptr};
  std::vector<esp32_ble_tracker::ESPBTDeviceListener *> listeners_{};

  // Addresses already printed by print_bt_device_info() — mirrors ESP32's dedup list.
  std::vector<uint64_t> already_discovered_{};

  // Advertisement queue: filled by the BLE task (on_scan_result), drained by loop().
  // Protected by a FreeRTOS critical section (taskENTER_CRITICAL / EXIT_CRITICAL).
  // Capped at MAX_ADV_QUEUE_SIZE; overflow increments adv_dropped_count_ (same as ESP32).
  std::vector<QueuedAdv> adv_queue_{};
  std::vector<QueuedAdv> adv_drain_{};  // swap buffer — avoids holding the critical section during dispatch
  uint16_t adv_dropped_count_{0};       // incremented under critical section; logged and reset in loop()

  std::vector<ESPBTAdvertiseTrigger *> adv_triggers_{};
  std::vector<BLEServiceDataAdvertiseTrigger *> svc_data_triggers_{};
  std::vector<BLEManufacturerDataAdvertiseTrigger *> mfr_data_triggers_{};
  std::vector<BLEEndOfScanTrigger *> scan_end_triggers_{};
};

// ---------------------------------------------------------------------------
// YAML automation actions: bk72xx_ble_tracker.start_scan / .stop_scan
// ---------------------------------------------------------------------------

template<typename... Ts> class StartScanAction : public Action<Ts...>, public Parented<BK72xxBLETracker> {
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

template<typename... Ts> class StopScanAction : public Action<Ts...>, public Parented<BK72xxBLETracker> {
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
  explicit ESPBTAdvertiseTrigger(BK72xxBLETracker *parent) { parent->register_adv_trigger(this); }

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
  explicit BLEServiceDataAdvertiseTrigger(BK72xxBLETracker *parent) { parent->register_svc_data_trigger(this); }

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
  explicit BLEManufacturerDataAdvertiseTrigger(BK72xxBLETracker *parent) { parent->register_mfr_data_trigger(this); }

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
  explicit BLEEndOfScanTrigger(BK72xxBLETracker *parent) { parent->register_scan_end_trigger(this); }
};

}  // namespace esphome::bk72xx_ble_tracker

#endif  // !USE_ESP32
