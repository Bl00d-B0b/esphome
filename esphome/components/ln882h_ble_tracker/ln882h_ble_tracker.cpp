// ln882h_ble_tracker.cpp
//
// BLE scanner implementation for LN882H.
// BLE stack init and scan lifecycle mirror OpenBeken's hal_bt_proxy_ln882h.c
// (openshwprojects/OpenBK7231T_App, src/hal/ln882h/hal_bt_proxy_ln882h.c,
//  lines 318-330 for the sentinel/TRNG MAC pattern, full BLE init from line 60).
//
// This file implements scanning only.  Advertisement forwarding to Home
// Assistant is handled by ln882h_bluetooth_proxy (see that component).

#ifdef USE_LIBRETINY

#include "ln882h_ble_tracker.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

// FreeRTOS critical section — used to protect the advertisement queue shared
// between rw_task (producer) and the ESPHome main loop (consumer).
#include "FreeRTOS.h"
#include "task.h"

// ---------------------------------------------------------------------------
// LN882H BLE SDK — forward declarations
// ---------------------------------------------------------------------------
extern "C" {

struct ln_bd_addr_v_t {
  uint8_t addr[6];
};  // ABI-identical to ln_bd_addr_t
void ln_kv_ble_app_init(void);
struct ln_bd_addr_v_t *ln_kv_ble_pub_addr_get(void);
int ln_kv_ble_addr_store(struct ln_bd_addr_v_t addr);

// Derives the BLE MAC from the WiFi STA MAC (= WiFi low-24 NIC + 1, MSB-first);
// returns false if the WiFi MAC is unavailable. Provided by the LibreTiny
// platform (cores/lightning-ln882h/base/api/lt_ble.c).
bool lt_ble_mac_get(uint8_t out[6]);
void soc_module_clk_gate_enable(uint32_t clk);

void rw_init(uint8_t mac[6]);
void ln_gap_app_init(void);
void ln_gatt_app_init(void);
void ln_ble_conn_mgr_init(void);
void ln_ble_evt_mgr_init(void);
void ln_ble_smp_init(void);
void ln_ble_scan_mgr_init(void);
void ln_rw_app_task_init(void);
void ln_gap_reset(void);

void ln_ble_scan_actv_creat(void);
void ln_ble_scan_start(void *scan_param);
void ln_ble_scan_stop(void);

typedef void (*ble_evt_cb_t)(void *param);
void ln_ble_evt_mgr_reg_evt(int evt_id, ble_evt_cb_t cb);

}  // extern "C"

// ---------------------------------------------------------------------------
// LN882H SDK constants
// ---------------------------------------------------------------------------

static constexpr uint32_t CLK_G_BLE = 1u << 8;
static constexpr int BLE_EVT_ID_SCAN_REPORT = 3;

static constexpr uint8_t GAPM_SCAN_TYPE_OBSERVER = 0;
static constexpr uint8_t GAPM_DUP_FILT_DIS = 0;
static constexpr uint8_t GAPM_SCAN_PROP_PHY_1M_BIT = 1 << 0;
static constexpr uint8_t GAPM_SCAN_PROP_ACTIVE_1M_BIT = 1 << 1;

// ---------------------------------------------------------------------------
// SDK struct layouts
// ---------------------------------------------------------------------------

struct le_scan_parameters_t {
  uint8_t type;
  uint8_t prop;
  uint8_t dup_filt_pol;
  uint8_t _pad;
  uint16_t scan_intv;
  uint16_t scan_wd;
};

struct ble_scan_report_t {
  uint8_t actv_idx;
  uint8_t info;
  uint8_t trans_addr_type;
  uint8_t trans_addr[6];
  uint8_t target_addr_type;
  uint8_t target_addr[6];
  int8_t tx_pwr;
  int8_t rssi;  // signed dBm, range -127..+20 (ble_evt_scan_report_t from ln_ble_event_manager.h)
  uint16_t length;
  uint8_t data[0];
};

// ---------------------------------------------------------------------------
// __sprintf weak stub
//
// The LN882H BLE SDK objects reference __sprintf (a Beken/LN libc alias) that
// LibreTiny's newlib does not provide. Supply a weak fallback so linking
// succeeds; a real definition, if one is ever provided, takes precedence.
// ---------------------------------------------------------------------------
#include <cstdarg>
extern "C" __attribute__((weak)) int __sprintf(char *str, const char *format, ...) {
  va_list args;
  va_start(args, format);
  int ret = vsprintf(str, format, args);  // NOLINT
  va_end(args);
  return ret;
}

// ---------------------------------------------------------------------------
namespace esphome::ln882h_ble_tracker {

static const char *const TAG = "ln882h_ble_tracker";
static LN882HBLETracker *s_tracker = nullptr;

// GAPM extended-advertising report types (bits 2:0 of ble_scan_report_t::info).
// 0 = ADV_EVT (non-connectable/non-scannable), 1 = ADV_EXT (extended),
// 2 = SCAN_RSP_EXT (scan response to extended adv), 3 = SCAN_RSP_LEG (scan response to legacy adv).
static constexpr uint8_t GAPM_REPORT_TYPE_SCAN_RSP_EXT = 2;
static constexpr uint8_t GAPM_REPORT_TYPE_SCAN_RSP_LEG = 3;
// Bit 5 of ble_scan_report_t::info: the advertisement is scannable, i.e. a scan
// response may follow (enum gapm_adv_report_info, GAPM_REPORT_INFO_SCAN_ADV_BIT).
static constexpr uint8_t GAPM_REPORT_INFO_SCAN_ADV_BIT = 1u << 5;

static void ble_scan_callback(void *param) {
  if (s_tracker == nullptr)
    return;
  const auto *info = reinterpret_cast<const ble_scan_report_t *>(param);

  uint8_t report_type = info->info & 0x07;

  // BLE RSSI sign fix. The LN882H controller intermittently reports the RSSI with
  // a flipped sign: a real -58 dBm arrives as +58, above the SDK's documented
  // -127..+20 dBm maximum. Recover it by negating any value above +20 (verified
  // on-device: the out-of-range positives cluster at the magnitude of each
  // device's real readings, e.g. a device at -54..-74 emits +54..+74). This is
  // the ONLY LN882H-specific RSSI handling — downstream the value is used exactly
  // like on ESP32 (forwarded as-is, no validity gating, never dropped).
  int8_t raw = info->rssi;
  int rssi = (raw > 20) ? -static_cast<int>(raw) : static_cast<int>(raw);

  // Scan responses: unlike ESP-IDF (which merges adv + scan response into one
  // result before ESPHome sees it), the LN controller delivers them as separate
  // reports. Reproduce the merge: a scannable advertisement is held briefly and
  // its scan response appended on arrival, so listeners AND the proxy receive
  // one merged frame — identical to ESP32. Scan responses with no pending
  // advertisement (overheard in passive mode, or the adv was missed) still go
  // to the proxy raw; HA merges per address.
  if (report_type == GAPM_REPORT_TYPE_SCAN_RSP_EXT || report_type == GAPM_REPORT_TYPE_SCAN_RSP_LEG) {
    s_tracker->deliver_scan_rsp(info->trans_addr, rssi, info->trans_addr_type, info->data, info->length);
    return;
  }

  if (s_tracker->is_scan_active() && (info->info & GAPM_REPORT_INFO_SCAN_ADV_BIT) != 0) {
    s_tracker->stash_adv(info->trans_addr, rssi, info->trans_addr_type, info->data, info->length);
    return;
  }

  s_tracker->on_scan_result(info->trans_addr, rssi, info->trans_addr_type, info->data, info->length);
}

// ---------------------------------------------------------------------------
// Component lifecycle
// ---------------------------------------------------------------------------

void LN882HBLETracker::setup() {
  s_tracker = this;
  // Reserve the queue buffers up front so on_scan_result()'s push_back never
  // allocates from inside the FreeRTOS critical section (rw_task context). The
  // size is bounded by MAX_ADV_QUEUE_SIZE, so a reserved buffer never reallocates.
  this->adv_queue_.reserve(MAX_ADV_QUEUE_SIZE);
  this->adv_drain_.reserve(MAX_ADV_QUEUE_SIZE);
  // Resolve MAC early so ln882h_bluetooth_proxy can report it to HA on
  // first connection (before the BLE stack is fully initialised).
  this->resolve_mac_();
  ESP_LOGI(TAG, "LN882H BLE tracker ready (scan=%s, window=%u BLE units, interval=%u BLE units)",
           this->scan_active_ ? "active" : "passive", this->scan_window_, this->scan_interval_);
}

// Watchdog: if the BLE SDK silently drops the scan (e.g. during WiFi coexistence
// arbitration), scan_running_ stays true but no advertisements arrive.  After
// SCAN_WATCHDOG_CYCLES scan intervals of silence we force-stop and restart.
//
// Continuous mode:  stop + restart via the idle branch → scan_start_time_ resets each cycle.
// Non-continuous mode: restart_scan_hw_() restarts only the hardware scan; scan_start_time_
//   is preserved so the duration window keeps running from the original start time.
//
// The threshold is derived from the configured scan interval so it scales with the
// scan rate: at the SDK-default 100 ms interval, 30 cycles = 3 s (30 windows ×
// 50 ms window = 1.5 s cumulative listen).  Because the SDK default runs a 50 %
// duty cycle, advertisements are caught frequently, so a few seconds of TOTAL
// silence is real evidence the scan died rather than a statistically-normal gap.
// A lower duty cycle (longer interval) automatically widens the threshold in
// proportion.  After WATCHDOG_RESTART_CAP consecutive silent restarts the counter is
// capped and a single warning is logged — the device is never rebooted on silence
// (silence is not proof the scan died; the per-cycle restart is the recovery).
static constexpr uint32_t SCAN_WATCHDOG_CYCLES = 30;
static constexpr uint8_t WATCHDOG_RESTART_CAP = 20;

void LN882HBLETracker::loop() {
  // Flush pending scannable advertisements whose scan response never arrived
  // (device didn't answer / frame lost) — delivered unmerged after the timeout.
  const uint32_t pending_now = millis();
  for (auto &p : this->pending_adv_) {
    bool deliver = false;
    PendingAdv copy;
    taskENTER_CRITICAL();
    if (p.used && pending_now - p.stored_ms > PENDING_ADV_TIMEOUT_MS) {
      copy = p;
      p.used = false;
      deliver = true;
    }
    taskEXIT_CRITICAL();
    if (deliver)
      this->on_scan_result(copy.mac, copy.rssi, copy.addr_type, copy.data, copy.data_len);
  }

  // Drain the advertisement queue populated by rw_task. Always swap under the critical
  // section (never read the queue unsynchronized): the swap is O(1), so rw_task is blocked
  // only momentarily, and all per-advertisement work then runs here on the main task.
  taskENTER_CRITICAL();
  this->adv_drain_.swap(this->adv_queue_);
  uint16_t dropped = this->adv_dropped_count_;
  this->adv_dropped_count_ = 0;
  taskEXIT_CRITICAL();

  if (!this->adv_drain_.empty()) {
    // Advertisements arrived: refresh the watchdog bookkeeping here on the main task — it is
    // also read by the watchdog below on this same task, so there is no cross-task access.
    this->last_scan_result_time_ = millis();
    this->watchdog_restart_count_ = 0;

    const bool needs_device = !this->listeners_.empty() || !this->adv_triggers_.empty() ||
                              !this->svc_data_triggers_.empty() || !this->mfr_data_triggers_.empty();

    for (const auto &qa : this->adv_drain_) {
      // Raw callback for the bluetooth proxy — invoked here on the main task, so the
      // std::function is read on the same task that assigns it (no data race). Both full
      // advertisements and unmatched scan responses (proxy_only) are forwarded.
      if (this->scan_result_callback_)
        this->scan_result_callback_(qa.mac, qa.rssi, qa.addr_type, qa.data, qa.data_len);

      // Scan-response-only frames are never parsed for local sensors/triggers.
      if (qa.proxy_only || !needs_device)
        continue;
      esp32_ble_tracker::ESPBTDevice device;
      device.from_scan_result(qa.mac, qa.rssi, qa.addr_type, qa.data, qa.data_len);
      bool found = false;
      for (auto *listener : this->listeners_)
        if (listener->parse_device(device))
          found = true;
      // Automation triggers also claim the device when they fire — exactly like
      // esp32_ble_tracker, where these triggers are listeners ORed into `found`.
      for (auto *t : this->adv_triggers_)
        if (t->process(device))
          found = true;
      for (auto *t : this->svc_data_triggers_)
        if (t->process(device))
          found = true;
      for (auto *t : this->mfr_data_triggers_)
        if (t->process(device))
          found = true;
      // Mirror esp32_ble_tracker: log a newly-seen device only when nothing claimed
      // it and the scan is one-shot (continuous scans would spam).
      if (!found && !this->scan_continuous_)
        this->print_bt_device_info(device);
    }
    this->adv_drain_.clear();
  }

  // Log dropped advertisements — mirrors ESP32's buffer-overflow warning.
  if (dropped > 0)
    ESP_LOGW(TAG, "Dropped %u BLE advertisements due to queue overflow", dropped);

  if (this->scan_continuous_) {
    if (this->scan_running_) {
      // Watchdog: if the BLE SDK silently drops the scan (WiFi/BLE coexistence
      // arbitration), force-stop so the idle branch below can restart it.
      // This is a recovery-only restart — on_scan_end() is NOT fired here; it is
      // handled independently by the period timer further below.
      // Watchdog threshold = 30 scan cycles (interval is in 0.625 ms BLE units).
      const uint32_t watchdog_ms = this->scan_interval_ * 5 / 8 * SCAN_WATCHDOG_CYCLES;
      if (this->last_scan_result_time_ != 0 && millis() - this->last_scan_result_time_ > watchdog_ms) {
        // WiFi/BLE coexistence restarts are normal; log only at VERBOSE.
        ESP_LOGV(TAG, "BLE scan silent >%lums; restarting (attempt %u)", static_cast<unsigned long>(watchdog_ms),
                 this->watchdog_restart_count_ + 1);
        ln_ble_scan_stop();
        this->scan_running_ = false;
        // Do NOT reboot the device on silence alone. A run of silent cycles is not proof
        // the scan died — it can simply mean nothing is in range (e.g. a single-source
        // deployment whose only tracked device left). The per-cycle stop + restart above
        // is the recovery; rebooting here would drop WiFi/API/logging on every quiet spell.
        // Cap the counter so it cannot overflow, and warn once when it first saturates.
        if (this->watchdog_restart_count_ < WATCHDOG_RESTART_CAP) {
          ++this->watchdog_restart_count_;
          if (this->watchdog_restart_count_ == WATCHDOG_RESTART_CAP)
            ESP_LOGW(TAG,
                     "BLE scan silent for %u consecutive restarts; still restarting each cycle (device not rebooted)",
                     this->watchdog_restart_count_);
        }
      }
    } else {
      this->start_scan_();
    }
    // Period timer: fire on_scan_end() once per scan_duration_ window,
    // mirroring esp32_ble_tracker::cleanup_scan_state_().
    // Intentionally decoupled from the watchdog: scan_period_start_ is NOT reset
    // on watchdog restarts, so frequent WiFi coexistence gaps cannot delay or
    // block the period.  Also fires even when the scan runs without any watchdog
    // events (e.g. dedicated BLE mode with no WiFi).
    if (millis() - this->scan_period_start_ >= this->scan_duration_) {
      for (auto *listener : this->listeners_)
        listener->on_scan_end();
      for (auto *t : this->scan_end_triggers_)
        t->trigger();
      this->already_discovered_.clear();  // reset per-scan "Found device" dedup (esp32_ble_tracker parity)
      this->scan_period_start_ = millis();
    }
    return;
  }

  // Non-continuous mode: run for scan_duration_ ms, then stop and fire on_scan_end.
  // Restart is driven externally (e.g. wifi: on_connect:).
  if (this->scan_running_) {
    if (millis() - this->scan_start_time_ >= this->scan_duration_) {
      this->stop_scan_();
    } else if (this->last_scan_result_time_ != 0 &&
               millis() - this->last_scan_result_time_ > this->scan_interval_ * 5 / 8 * SCAN_WATCHDOG_CYCLES) {
      // Watchdog: BLE SDK silently dropped the scan (WiFi/BLE coexistence).
      // Restart hardware scan but preserve scan_start_time_ so the duration
      // window keeps running from the original start — no timer reset.
      this->restart_scan_hw_();
    }
  }
}

void LN882HBLETracker::dump_config() {
  ESP_LOGCONFIG(TAG, "LN882H BLE Tracker:");
  ESP_LOGCONFIG(TAG, "  Scan Duration: %u s", this->scan_duration_ / 1000);
  ESP_LOGCONFIG(TAG, "  Scan Interval: %.0f ms (%u BLE units)", this->scan_interval_ * 0.625f, this->scan_interval_);
  ESP_LOGCONFIG(TAG, "  Scan Window: %.0f ms (%u BLE units)", this->scan_window_ * 0.625f, this->scan_window_);
  ESP_LOGCONFIG(TAG, "  Scan Type: %s", this->scan_active_ ? "ACTIVE" : "PASSIVE");
  ESP_LOGCONFIG(TAG, "  Continuous Scanning: %s", this->scan_continuous_ ? "YES" : "NO");
}

// ---------------------------------------------------------------------------
// Scan result — called from rw_task (BLE RTOS context).
// Raw proxy callback runs here directly (lightweight, no ESPHome state).
// Parsed-advertisement dispatch is deferred: advertisement is pushed to a
// queue here and drained in loop() on the ESPHome main task, matching the
// ESP32 pattern.  This ensures publish_state() and automation triggers run
// on the main loop and every advertisement is visible to sensors, not just
// the last one in a burst.
// ---------------------------------------------------------------------------

// Hold a scannable advertisement, waiting (≤ PENDING_ADV_TIMEOUT_MS) for its
// scan response. BLE RTOS task context.
void LN882HBLETracker::stash_adv(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                 uint16_t data_len) {
  uint8_t old_data[62];
  uint16_t old_len = 0;
  uint8_t old_at = 0;
  int old_rssi = 0;
  bool flush_old = false;

  taskENTER_CRITICAL();
  PendingAdv *slot = nullptr;
  for (auto &p : this->pending_adv_) {
    if (p.used && p.addr_type == addr_type && memcmp(p.mac, mac, 6) == 0) {
      // Same device advertised again before its scan response arrived — deliver
      // the previous advertisement (its scan response is not coming) and reuse
      // the slot, so no frame is ever lost.
      memcpy(old_data, p.data, p.data_len);
      old_len = p.data_len;
      old_at = p.addr_type;
      old_rssi = p.rssi;
      flush_old = true;
      slot = &p;
      break;
    }
  }
  if (slot == nullptr) {
    for (auto &p : this->pending_adv_) {
      if (!p.used) {
        slot = &p;
        break;
      }
    }
  }
  if (slot != nullptr) {
    slot->used = true;
    memcpy(slot->mac, mac, 6);
    slot->addr_type = addr_type;
    slot->rssi = rssi;
    slot->data_len = (data_len <= sizeof(slot->data)) ? data_len : static_cast<uint16_t>(sizeof(slot->data));
    memcpy(slot->data, data, slot->data_len);
    slot->stored_ms = millis();
  }
  taskEXIT_CRITICAL();

  if (flush_old)
    this->on_scan_result(mac, old_rssi, old_at, old_data, old_len);
  if (slot == nullptr) {
    // Table full — degrade gracefully: deliver the advertisement unmerged.
    this->on_scan_result(mac, rssi, addr_type, data, data_len);
  }
}

// Scan response arrived: merge it with the pending advertisement from the same
// device into ONE frame (ESP-IDF/Bluedroid semantics). BLE RTOS task context.
void LN882HBLETracker::deliver_scan_rsp(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                        uint16_t data_len) {
  uint8_t merged[62];
  uint16_t merged_len = 0;
  bool found = false;

  taskENTER_CRITICAL();
  for (auto &p : this->pending_adv_) {
    if (p.used && p.addr_type == addr_type && memcmp(p.mac, mac, 6) == 0) {
      merged_len = p.data_len;
      memcpy(merged, p.data, merged_len);
      uint16_t room = static_cast<uint16_t>(sizeof(merged)) - merged_len;
      uint16_t add = (data_len <= room) ? data_len : room;
      memcpy(merged + merged_len, data, add);
      merged_len += add;
      p.used = false;
      found = true;
      break;
    }
  }
  taskEXIT_CRITICAL();

  if (found) {
    this->on_scan_result(mac, rssi, addr_type, merged, merged_len);
  } else {
    this->on_scan_rsp(mac, rssi, addr_type, data, data_len);
  }
}

void LN882HBLETracker::on_scan_rsp(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                   uint16_t data_len) {
  // Unmatched scan-response: goes to the Bluetooth proxy only (HA merges per address);
  // local listeners/triggers receive each advertisement exactly once via on_scan_result.
  // Queue it (proxy_only) and let loop() forward it on the main task — the watchdog
  // bookkeeping and the proxy callback both run there.
  this->enqueue_adv_(mac, rssi, addr_type, data, data_len, /*proxy_only=*/true);
}

void LN882HBLETracker::on_scan_result(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                      uint16_t data_len) {
  // Queue the (possibly merged) advertisement for dispatch on the main loop. All consumer
  // work — watchdog bookkeeping, the bluetooth_proxy callback, device parsing,
  // listeners/triggers and logging — runs in loop() on the main task, so no main-task-owned
  // state (including the scan_result_callback_ std::function) is touched from rw_task. This
  // is also called from loop() itself for timed-out pending advertisements; queueing there
  // is harmless (drained later in the same loop pass).
  this->enqueue_adv_(mac, rssi, addr_type, data, data_len, /*proxy_only=*/false);
}

void LN882HBLETracker::enqueue_adv_(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                    uint16_t data_len, bool proxy_only) {
  QueuedAdv qa;
  memcpy(qa.mac, mac, 6);
  qa.rssi = rssi;
  qa.addr_type = addr_type;
  qa.data_len = (data_len <= sizeof(qa.data)) ? data_len : sizeof(qa.data);
  memcpy(qa.data, data, qa.data_len);
  qa.proxy_only = proxy_only;

  taskENTER_CRITICAL();
  if (this->adv_queue_.size() < MAX_ADV_QUEUE_SIZE) {
    this->adv_queue_.push_back(qa);
  } else {
    this->adv_dropped_count_++;
  }
  taskEXIT_CRITICAL();
}

// Log a newly-discovered device once, at DEBUG — copied verbatim from
// esp32_ble_tracker::ESP32BLETracker::print_bt_device_info so the output (message,
// level, and dedup) is identical on this backend.
void LN882HBLETracker::print_bt_device_info(const esp32_ble_tracker::ESPBTDevice &device) {
  const uint64_t address = device.address_uint64();
  for (auto &disc : this->already_discovered_) {
    if (disc == address)
      return;
  }
  this->already_discovered_.push_back(address);

  char addr_buf[esp32_ble_tracker::ESPBTDevice::MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  ESP_LOGD(TAG, "Found device %s RSSI=%d", device.address_str_to(addr_buf), device.get_rssi());

  const char *address_type_s;
  switch (device.get_address_type()) {
    case 0:
      address_type_s = "PUBLIC";
      break;
    case 1:
      address_type_s = "RANDOM";
      break;
    case 2:
      address_type_s = "RPA_PUBLIC";
      break;
    case 3:
      address_type_s = "RPA_RANDOM";
      break;
    default:
      address_type_s = "UNKNOWN";
      break;
  }
  ESP_LOGD(TAG, "  Address Type: %s", address_type_s);
  if (!device.get_name().empty()) {
    ESP_LOGD(TAG, "  Name: '%s'", device.get_name().c_str());
  }
  for (auto &tx_power : device.get_tx_powers()) {
    ESP_LOGD(TAG, "  TX Power: %d", tx_power);
  }
}

// ---------------------------------------------------------------------------
// MAC resolution
// ---------------------------------------------------------------------------

void LN882HBLETracker::resolve_mac_() {
  ln_kv_ble_app_init();

  // The BLE MAC is derived from the WiFi MAC by the LibreTiny platform
  // (lt_ble_mac_get = WiFi low-24-bit NIC + 1, MSB-first). On success, persist it
  // to the SDK BLE KV ("2_ble_addr"); on failure (no WiFi MAC — should not happen
  // after WiFi init) fall back to the SDK's current address, never a zero MAC.
  ln_bd_addr_v_t bt_addr{};
  if (lt_ble_mac_get(bt_addr.addr)) {
    ln_kv_ble_addr_store(bt_addr);
    ESP_LOGI(TAG, "BLE MAC (WiFi+1): %02X:%02X:%02X:%02X:%02X:%02X", bt_addr.addr[0], bt_addr.addr[1], bt_addr.addr[2],
             bt_addr.addr[3], bt_addr.addr[4], bt_addr.addr[5]);
  } else {
    bt_addr = *ln_kv_ble_pub_addr_get();
    ESP_LOGW(TAG, "BLE MAC derivation failed; using SDK address %02X:%02X:%02X:%02X:%02X:%02X", bt_addr.addr[0],
             bt_addr.addr[1], bt_addr.addr[2], bt_addr.addr[3], bt_addr.addr[4], bt_addr.addr[5]);
  }
  memcpy(this->ble_mac_, bt_addr.addr, 6);
}

// ---------------------------------------------------------------------------
// BLE stack init (called from start_scan_() on first use)
// ---------------------------------------------------------------------------

void LN882HBLETracker::ble_stack_init_() {
  if (this->ble_stack_ready_) {
    ln_ble_evt_mgr_reg_evt(BLE_EVT_ID_SCAN_REPORT, ble_scan_callback);
    return;
  }

  *reinterpret_cast<volatile uint32_t *>(0x400121F8) = 0x003F;  // WiFi/BLE PTI coexistence
  soc_module_clk_gate_enable(CLK_G_BLE);

  rw_init(this->ble_mac_);
  ln_gap_app_init();
  ln_gatt_app_init();
  ln_ble_conn_mgr_init();
  ln_ble_evt_mgr_init();
  ln_ble_smp_init();
  ln_ble_scan_mgr_init();
  ln_rw_app_task_init();
  ln_gap_reset();

  delay(100);  // NOLINT — one-time BLE stack init; SDK requires this settle time

  ln_ble_scan_actv_creat();
  delay(10);

  le_scan_parameters_t probe_p{};
  probe_p.type = GAPM_SCAN_TYPE_OBSERVER;
  probe_p.prop = GAPM_SCAN_PROP_PHY_1M_BIT;
  probe_p.dup_filt_pol = GAPM_DUP_FILT_DIS;
  probe_p.scan_intv = 160;
  probe_p.scan_wd = 16;
  ln_ble_scan_start(&probe_p);
  delay(10);
  ln_ble_scan_stop();

  ln_ble_evt_mgr_reg_evt(BLE_EVT_ID_SCAN_REPORT, ble_scan_callback);
  this->ble_stack_ready_ = true;
  ESP_LOGI(TAG, "BLE stack initialised");
}

// ---------------------------------------------------------------------------
// Public scan actions
// ---------------------------------------------------------------------------

void LN882HBLETracker::start_scan() {
  // Mirrors esp32_ble_tracker::start_scan(): caller sets scan_continuous_ via
  // set_scan_continuous() first, then calls start_scan() to begin scanning.
  if (!this->scan_running_) {
    // Re-anchor the on_scan_end period to this explicit (re)start, so starting after a
    // long stop (e.g. Home Assistant reconnecting after more than scan_duration) does not
    // fire on_scan_end immediately. Watchdog restarts call start_scan_() directly and
    // deliberately leave the period running.
    this->scan_period_start_ = millis();
    this->start_scan_();
  }
}

void LN882HBLETracker::stop_scan() {
  this->scan_continuous_ = false;
  this->stop_scan_();
}

// ---------------------------------------------------------------------------
// Internal scan start / stop
// ---------------------------------------------------------------------------

void LN882HBLETracker::start_scan_() {
  if (!this->ble_stack_ready_)
    this->ble_stack_init_();
  if (this->scan_running_)
    return;

  le_scan_parameters_t p{};
  p.dup_filt_pol = GAPM_DUP_FILT_DIS;
  p.type = GAPM_SCAN_TYPE_OBSERVER;
  p.scan_intv = static_cast<uint16_t>(this->scan_interval_);
  p.scan_wd = static_cast<uint16_t>(this->scan_window_);
  // Legacy 1M PHY only: the advertisement buffers (QueuedAdv/PendingAdv) are sized for
  // legacy advertisements (62 B), so coded/extended PHY (up to 255 B) would be silently
  // truncated. Scan 1M only to keep the advertised capability and the buffers consistent.
  p.prop = GAPM_SCAN_PROP_PHY_1M_BIT;
  if (this->scan_active_)
    p.prop |= GAPM_SCAN_PROP_ACTIVE_1M_BIT;

  ln_ble_scan_start(&p);
  this->scan_running_ = true;
  this->scan_start_time_ = millis();
  this->last_scan_result_time_ = millis();  // reset watchdog timer; fires after SCAN_WATCHDOG_CYCLES of silence
  // LOGI on the first start so the user can confirm the scanner came up.
  // Subsequent restarts (watchdog coexistence recovery) are LOGD to avoid noise.
  if (!this->scan_started_once_) {
    // Anchor the continuous-mode on_scan_end period to the first scan start (not
    // to boot), so a scan that starts later than scan_duration into uptime does
    // not fire on_scan_end immediately. Deliberately set only on the first start,
    // never on watchdog restarts, to keep the period decoupled from coexistence gaps.
    this->scan_period_start_ = millis();
    ESP_LOGI(TAG, "BLE scan started (%s, window=%.0fms, interval=%.0fms)", this->scan_active_ ? "active" : "passive",
             this->scan_window_ * 0.625f, this->scan_interval_ * 0.625f);
    this->scan_started_once_ = true;
  } else {
    ESP_LOGV(TAG, "BLE scan restarted (%s)", this->scan_active_ ? "active" : "passive");
  }
}

// Restart the hardware BLE scan without resetting scan_start_time_.
// Used by the non-continuous watchdog: recovers from WiFi/BLE coexistence gaps
// while keeping the original duration window intact.
void LN882HBLETracker::restart_scan_hw_() {
  ln_ble_scan_stop();

  le_scan_parameters_t p{};
  p.dup_filt_pol = GAPM_DUP_FILT_DIS;
  p.type = GAPM_SCAN_TYPE_OBSERVER;
  p.scan_intv = static_cast<uint16_t>(this->scan_interval_);
  p.scan_wd = static_cast<uint16_t>(this->scan_window_);
  // Legacy 1M PHY only: the advertisement buffers (QueuedAdv/PendingAdv) are sized for
  // legacy advertisements (62 B), so coded/extended PHY (up to 255 B) would be silently
  // truncated. Scan 1M only to keep the advertised capability and the buffers consistent.
  p.prop = GAPM_SCAN_PROP_PHY_1M_BIT;
  if (this->scan_active_)
    p.prop |= GAPM_SCAN_PROP_ACTIVE_1M_BIT;

  ln_ble_scan_start(&p);
  this->last_scan_result_time_ = millis();  // reset watchdog timer; scan_start_time_ unchanged
  ESP_LOGV(TAG, "BLE scan silent >%lums; restarting within non-continuous window",
           static_cast<unsigned long>(this->scan_interval_ * 5 / 8 * SCAN_WATCHDOG_CYCLES));
}

void LN882HBLETracker::stop_scan_() {
  if (!this->scan_running_)
    return;
  ln_ble_scan_stop();
  this->scan_running_ = false;
  ESP_LOGI(TAG, "BLE scan stopped");
  for (auto *listener : this->listeners_)
    listener->on_scan_end();
  for (auto *t : this->scan_end_triggers_)
    t->trigger();
  // reset per-scan "Found device" dedup (esp32_ble_tracker parity)
  this->already_discovered_.clear();
  this->scan_period_start_ = millis();  // reset period clock so watchdog does not double-fire
}

}  // namespace esphome::ln882h_ble_tracker

#endif  // USE_LIBRETINY
