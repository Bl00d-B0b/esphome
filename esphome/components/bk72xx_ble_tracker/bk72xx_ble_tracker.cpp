// bk72xx_ble_tracker.cpp
//
// BLE scanner implementation for the BK72xx BLE-5.x chips (LibreTiny beken-72xx family).
// BLE stack init and scan lifecycle use the public Beken BDK BLE API (ble_api.h):
//   - ble_set_notice_cb() + ble_entry() for one-time stack init,
//   - app_ble_get_idle_actv_idx_handle(SCAN_ACTV) + bk_ble_scan_start/stop()
//     for the scan lifecycle,
//   - BLE_5_REPORT_ADV notice carrying a recv_adv_t for each advertisement.
//
// This file implements scanning only.  Advertisement forwarding to Home
// Assistant is handled by the separate bluetooth_proxy component.
//
// NOTE: the Beken BDK BLE 5.x stack is compiled and linked by the LibreTiny
// beken-72xx builder itself (prebuilt libble_<chip>.a + ble_5_x sources, gated on
// CFG_SUPPORT_BLE / CFG_BLE_VERSION in sys_config.h). This component only calls into
// it via the public ble_api.h — no framework patch is required.

#ifdef USE_LIBRETINY

#include "bk72xx_ble_tracker.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"  // get_mac_address_raw()
#include "esphome/core/log.h"

// FreeRTOS critical section — protects the advertisement queue shared between
// the BLE task (producer) and the ESPHome main loop (consumer).
#include "FreeRTOS.h"
#include "task.h"

// ---------------------------------------------------------------------------
// SDK-capability gate (not a chip allowlist).
// This component drives the Beken BLE *5.x* controller via its public API,
// `ble_api.h`, which the LibreTiny beken-72xx builder ships only for the
// BLE-5.x SoCs (it selects the `ble_pub` 5.x stack from CFG_BLE_VERSION; the
// 4.2 SoCs build a different, older API with no ble_api.h). Gate on the header
// itself so any BLE-5.x Beken chip — present or future — is supported without a
// hard-coded list, and a non-5.x build fails here with a clear message instead
// of a cryptic "ble_api.h: No such file or directory".
// ---------------------------------------------------------------------------
#if !__has_include("ble_api.h")
#error \
    "bk72xx_ble_tracker requires a BLE 5.x Beken SDK (ble_api.h). Supported SoCs: BK7231N/BK7236 (BLE 5.1) and BK7238/BK7252N/BK7253 (BLE 5.2). BK7231T/BK7251/BK7271 (BLE 4.2) and BK7231Q (no BLE) are not supported."
#endif

// ---------------------------------------------------------------------------
// Beken BDK BLE 5.x SDK — public API.
// Exposed on the include path by the LibreTiny beken-72xx builder
// (cores/.../ble_5_x_rw + driver/include). Wrapped in extern "C" because these
// are C headers consumed from C++ (a standard C-header-from-C++ pattern).
// ---------------------------------------------------------------------------
extern "C" {
#include "ble_api.h"  // bk_ble_scan_start/stop, ble_entry, ble_set_notice_cb,
                      // app_ble_get_idle_actv_idx_handle, struct scan_param,
                      // recv_adv_t, ble_notice_t, BLE_5_REPORT_ADV, SCAN_ACTV
#ifdef BK72XX_BLE_HAS_COMMON_BDADDR
#include "common_bt_defines.h"  // struct bd_addr
// The controller's public BLE address, populated by the BDK during ble_entry().
// Present on BK7231N; the other BLE-5.x chips' stacks have no such symbol — there the
// address is derived from the WiFi MAC instead (matching OpenBeken's BK7238 path).
extern struct bd_addr common_default_bdaddr;
#endif
// ble_entry() brings up the BDK BLE stack; it is not declared in ble_api.h, so
// declare it here.
void ble_entry(void);
}

// ---------------------------------------------------------------------------
namespace esphome::bk72xx_ble_tracker {

static const char *const TAG = "bk72xx_ble_tracker";
static BK72xxBLETracker *s_tracker = nullptr;

// ---------------------------------------------------------------------------
// BLE notice callback — runs in the BDK BLE task context.
// The BK controller reports every advertisement (and, in active mode, scan
// responses) as a BLE_5_REPORT_ADV notice carrying a recv_adv_t. We forward
// each directly; Home Assistant merges adv + scan-response per address, so no
// host-side merge is needed (unlike the LN882H sibling).
// ---------------------------------------------------------------------------
static void ble_notice_callback(ble_notice_t notice, void *param) {
  if (s_tracker == nullptr || param == nullptr)
    return;
  if (notice != BLE_5_REPORT_ADV)
    return;

  const recv_adv_t *info = reinterpret_cast<const recv_adv_t *>(param);
  // rssi is a signed dBm carried in a uint8_t; cast through int8_t (standard for a signed dBm value packed in a
  // uint8_t).
  int rssi = static_cast<int>(static_cast<int8_t>(info->rssi));
  s_tracker->on_scan_result(info->adv_addr, rssi, info->adv_addr_type, info->data, info->data_len);
}

// ---------------------------------------------------------------------------
// Component lifecycle
// ---------------------------------------------------------------------------

void BK72xxBLETracker::setup() {
  s_tracker = this;
  // Reserve the queue buffers up front so on_scan_result()'s push_back never
  // allocates from inside the FreeRTOS critical section (BLE task context). The
  // size is bounded by MAX_ADV_QUEUE_SIZE, so a reserved buffer never reallocates.
  this->adv_queue_.reserve(MAX_ADV_QUEUE_SIZE);
  this->adv_drain_.reserve(MAX_ADV_QUEUE_SIZE);
  // Resolve MAC early so the bluetooth_proxy proxy can report it to HA on first
  // connection (before the BLE stack is fully initialised).
  this->resolve_mac_();
  ESP_LOGI(TAG, "BK72xx BLE tracker ready (scan=passive, window=%u BLE units, interval=%u BLE units)",
           this->scan_window_, this->scan_interval_);
}

// Watchdog: if the BLE SDK silently drops the scan (e.g. during WiFi coexistence
// arbitration), scan_running_ stays true but no advertisements arrive.  After
// SCAN_WATCHDOG_CYCLES scan intervals of silence we force-stop and restart.
//
// The threshold is derived from the configured scan interval so it scales with
// the scan rate: at the default 100 ms interval, 30 cycles = 3 s.  Because the
// default runs a 30 % duty cycle, advertisements are caught frequently, so a few
// seconds of TOTAL silence is real evidence the scan died rather than a normal
// gap.  After WATCHDOG_RESTART_CAP consecutive silent restarts the counter is capped
// and a single warning is logged — the device is never rebooted on silence (silence
// is not proof the scan died; the per-cycle restart is the recovery).
static constexpr uint32_t SCAN_WATCHDOG_CYCLES = 30;
static constexpr uint8_t WATCHDOG_RESTART_CAP = 20;
// Minimum interval between scan (re)start attempts, so a failing bk_ble_scan_start
// cannot be retried every main-loop iteration (single-core CPU starvation).
static constexpr uint32_t SCAN_START_RETRY_MS = 1000;

void BK72xxBLETracker::loop() {
  // Drain the advertisement queue populated by the BLE task. Always swap under the scheduler
  // lock (never read the queue unsynchronized): the swap is O(1), so the BLE task is blocked
  // only momentarily, and all per-advertisement work then runs here on the main task.
  vTaskSuspendAll();
  this->adv_drain_.swap(this->adv_queue_);
  uint16_t dropped = this->adv_dropped_count_;
  this->adv_dropped_count_ = 0;
  xTaskResumeAll();

  if (!this->adv_drain_.empty()) {
    // Advertisements arrived: refresh the watchdog bookkeeping here on the main task — it is
    // also read by the watchdog below on this same task, so there is no cross-task access.
    this->last_scan_result_time_ = millis();
    this->watchdog_restart_count_ = 0;

    const bool needs_device = !this->listeners_.empty() || !this->adv_triggers_.empty() ||
                              !this->svc_data_triggers_.empty() || !this->mfr_data_triggers_.empty();

    for (const auto &qa : this->adv_drain_) {
      // Raw callback for the bluetooth proxy — invoked here on the main task, so the
      // std::function is read on the same task that assigns it (no data race).
      if (this->scan_result_callback_)
        this->scan_result_callback_(qa.mac, qa.rssi, qa.addr_type, qa.data, qa.data_len);

      if (!needs_device)
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
      // Recovery-only restart — on_scan_end() is NOT fired here; the period timer
      // below handles that independently.  Threshold = 30 scan cycles (interval is
      // in 0.625 ms BLE units, hence the × 5 / 8 conversion to ms).
      const uint32_t watchdog_ms = this->scan_interval_ * 5 / 8 * SCAN_WATCHDOG_CYCLES;
      if (this->last_scan_result_time_ != 0 && millis() - this->last_scan_result_time_ > watchdog_ms) {
        ESP_LOGV(TAG, "BLE scan silent >%lums; restarting (attempt %u)", static_cast<unsigned long>(watchdog_ms),
                 this->watchdog_restart_count_ + 1);
        this->stop_scan_hw_();
        // Do NOT reboot the device on silence alone. A run of silent cycles is not proof
        // the scan died — it can simply mean nothing is in range (e.g. a single-source
        // deployment whose only tracked device left). The per-cycle stop + restart (idle
        // branch below) is the recovery; rebooting here would drop WiFi/API/logging on
        // every quiet spell. Cap the counter so it cannot overflow, and warn once when it
        // first saturates.
        if (this->watchdog_restart_count_ < WATCHDOG_RESTART_CAP) {
          ++this->watchdog_restart_count_;
          if (this->watchdog_restart_count_ == WATCHDOG_RESTART_CAP)
            ESP_LOGW(TAG,
                     "BLE scan silent for %u consecutive restarts; still restarting each cycle (device not rebooted)",
                     this->watchdog_restart_count_);
        }
      }
    } else {
      // Rate-limit (re)start attempts. bk_ble_scan_start can fail (no idle activity
      // handle, WiFi/BLE coexistence) and leave scan_running_ false; retrying every
      // main-loop iteration would spin the single-core CPU and starve WiFi (device
      // becomes unresponsive). Attempt at most once per SCAN_START_RETRY_MS; the scan is
      // (re)started on events, not polled every main-loop iteration.
      if (millis() - this->last_scan_start_attempt_ >= SCAN_START_RETRY_MS) {
        this->last_scan_start_attempt_ = millis();
        this->start_scan_();
      }
    }
    // Period timer: fire on_scan_end() once per scan_duration_ window, mirroring
    // esp32_ble_tracker::cleanup_scan_state_(). Decoupled from the watchdog:
    // scan_period_start_ is NOT reset on watchdog restarts, so frequent WiFi
    // coexistence gaps cannot delay the period.
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
  // Restart is driven externally (e.g. api: on_client_connected:).
  if (this->scan_running_) {
    if (millis() - this->scan_start_time_ >= this->scan_duration_) {
      this->stop_scan_();
    } else if (this->last_scan_result_time_ != 0 &&
               millis() - this->last_scan_result_time_ > this->scan_interval_ * 5 / 8 * SCAN_WATCHDOG_CYCLES) {
      // Watchdog: restart the hardware scan but preserve scan_start_time_ so the
      // duration window keeps running from the original start — no timer reset.
      this->restart_scan_hw_();
    }
  } else if (this->restart_pending_) {
    // A watchdog restart_scan_hw_() failed to bring the scan back up. Either the duration
    // window has now elapsed (give up cleanly) or retry the restart (rate-limited), so a
    // transient failure doesn't silently leave the scan off for the rest of the window.
    if (millis() - this->scan_start_time_ >= this->scan_duration_) {
      this->restart_pending_ = false;
    } else if (millis() - this->last_scan_start_attempt_ >= SCAN_START_RETRY_MS) {
      this->last_scan_start_attempt_ = millis();
      this->restart_scan_hw_();
    }
  }
}

void BK72xxBLETracker::dump_config() {
  ESP_LOGCONFIG(TAG, "BK72xx BLE Tracker:");
  ESP_LOGCONFIG(TAG, "  Scan Duration: %u s", this->scan_duration_ / 1000);
  ESP_LOGCONFIG(TAG, "  Scan Interval: %.0f ms (%u BLE units)", this->scan_interval_ * 0.625f, this->scan_interval_);
  ESP_LOGCONFIG(TAG, "  Scan Window: %.0f ms (%u BLE units)", this->scan_window_ * 0.625f, this->scan_window_);
  ESP_LOGCONFIG(TAG, "  Scan Type: PASSIVE");
  ESP_LOGCONFIG(TAG, "  Continuous Scanning: %s", this->scan_continuous_ ? "YES" : "NO");
}

// ---------------------------------------------------------------------------
// Scan result — called from the BLE task (BLE RTOS context).
// Raw proxy callback runs here directly (lightweight, no ESPHome state).
// Parsed-advertisement dispatch is deferred: the advertisement is pushed to a
// queue here and drained in loop() on the ESPHome main task, matching the ESP32
// pattern.  This ensures publish_state() and automation triggers run on the main
// loop and every advertisement is visible to sensors, not just the last in a burst.
// ---------------------------------------------------------------------------

void BK72xxBLETracker::on_scan_result(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                      uint16_t data_len) {
  // BLE RTOS task context: do the bare minimum — copy the advertisement into the queue under
  // the scheduler lock and return. Everything else (watchdog bookkeeping, the bluetooth_proxy
  // callback, device parsing, listeners/triggers and logging) runs in loop() on the main task,
  // so no main-task-owned state — including the scan_result_callback_ std::function — is ever
  // touched from this task.
  QueuedAdv qa;
  memcpy(qa.mac, mac, 6);
  qa.rssi = rssi;
  qa.addr_type = addr_type;
  qa.data_len = (data_len <= sizeof(qa.data)) ? data_len : sizeof(qa.data);
  memcpy(qa.data, data, qa.data_len);

  vTaskSuspendAll();
  if (this->adv_queue_.size() < MAX_ADV_QUEUE_SIZE) {
    this->adv_queue_.push_back(qa);
  } else {
    this->adv_dropped_count_++;
  }
  xTaskResumeAll();
}

// Log a newly-discovered device once, at DEBUG — copied verbatim from
// esp32_ble_tracker::ESP32BLETracker::print_bt_device_info so the output (message,
// level, and dedup) is identical on this backend.
void BK72xxBLETracker::print_bt_device_info(const esp32_ble_tracker::ESPBTDevice &device) {
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

void BK72xxBLETracker::resolve_mac_() {
  // NOTE: byte order is reported to HA by the bluetooth_proxy proxy — verify on
  // hardware that the address displayed matches the chip's BLE MAC.
#ifdef BK72XX_BLE_HAS_COMMON_BDADDR
  // BK7231N: the BDK populates common_default_bdaddr (LSB-first, BLE convention)
  // during ble_entry(). It may still be zero before the stack is up; if so, fall
  // through to the WiFi-derived MAC below.
  bool nonzero = false;
  for (uint8_t b : common_default_bdaddr.addr) {
    if (b != 0) {
      nonzero = true;
      break;
    }
  }
  if (nonzero) {
    memcpy(this->ble_mac_, common_default_bdaddr.addr, 6);
    ESP_LOGI(TAG, "BLE MAC: %02X:%02X:%02X:%02X:%02X:%02X", this->ble_mac_[5], this->ble_mac_[4], this->ble_mac_[3],
             this->ble_mac_[2], this->ble_mac_[1], this->ble_mac_[0]);
    return;
  }
#endif
  // Chips whose BLE stack has no common_default_bdaddr (BK7238 and the other BLE-5.x
  // SoCs), or BK7231N before the stack is up: derive the BLE MAC as WiFi + 1 — the low
  // 24-bit NIC incremented (with wraparound), OUI unchanged. This is the standard
  // factory pairing (same as the LN882H sibling's lt_ble_mac_get()).
  uint8_t wifi_mac[6];
  get_mac_address_raw(wifi_mac);  // MSB-first
  uint32_t nic = (static_cast<uint32_t>(wifi_mac[3]) << 16) | (static_cast<uint32_t>(wifi_mac[4]) << 8) | wifi_mac[5];
  nic = (nic + 1) & 0xFFFFFF;
  const uint8_t ble[6] = {wifi_mac[0],
                          wifi_mac[1],
                          wifi_mac[2],
                          static_cast<uint8_t>(nic >> 16),
                          static_cast<uint8_t>(nic >> 8),
                          static_cast<uint8_t>(nic)};
  // Store LSB-first to match recv_adv_t adv_addr ordering.
  for (int i = 0; i < 6; i++)
    this->ble_mac_[i] = ble[5 - i];
  ESP_LOGW(TAG, "BLE address derived as WiFi MAC + 1: %02X:%02X:%02X:%02X:%02X:%02X", ble[0], ble[1], ble[2], ble[3],
           ble[4], ble[5]);
}

// ---------------------------------------------------------------------------
// BLE stack init (called from start_scan_() on first use)
// ---------------------------------------------------------------------------

void BK72xxBLETracker::ble_stack_init_() {
  if (this->ble_stack_ready_)
    return;

  // Register the advertisement callback, then bring up the BDK BLE stack.
  // One-time BLE stack init: set_notice_cb() then ble_entry().
  ble_set_notice_cb(ble_notice_callback);
  ble_entry();

  delay(100);  // NOLINT — one-time BLE stack init; the SDK needs this settle time

  // Re-read the BLE MAC now that the controller is up (common_default_bdaddr is
  // populated by ble_entry()); resolve_mac_() may have fallen back earlier.
  this->resolve_mac_();

  this->ble_stack_ready_ = true;
  ESP_LOGI(TAG, "BLE stack initialised");
}

// ---------------------------------------------------------------------------
// Public scan actions
// ---------------------------------------------------------------------------

void BK72xxBLETracker::start_scan() {
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

void BK72xxBLETracker::stop_scan() {
  this->scan_continuous_ = false;
  this->restart_pending_ = false;  // cancel any pending non-continuous restart retry
  this->stop_scan_();
}

// ---------------------------------------------------------------------------
// Internal scan start / stop
// ---------------------------------------------------------------------------

// Start the controller scan with the configured parameters. Returns true on
// success. Does NOT touch the duration/period timers (callers own those).
bool BK72xxBLETracker::scan_hw_start_() {
  struct scan_param sp;
  memset(&sp, 0, sizeof(sp));
  sp.channel_map = 7;  // advertising channels 37/38/39
  sp.interval = static_cast<uint16_t>(this->scan_interval_);
  sp.window = static_cast<uint16_t>(this->scan_window_);

  this->scan_actv_idx_ = app_ble_get_idle_actv_idx_handle(SCAN_ACTV);
  if (this->scan_actv_idx_ == 0xFF) {
    ESP_LOGE(TAG, "BLE scan start failed: no idle activity handle");
    return false;
  }
  ble_err_t ret = bk_ble_scan_start(this->scan_actv_idx_, &sp, nullptr);
  if (ret != ERR_SUCCESS) {
    ESP_LOGE(TAG, "bk_ble_scan_start failed (err %d)", static_cast<int>(ret));
    this->scan_actv_idx_ = 0xFF;
    return false;
  }
  return true;
}

// Stop the controller scan (hardware only — does NOT fire on_scan_end()).
void BK72xxBLETracker::stop_scan_hw_() {
  if (this->scan_actv_idx_ != 0xFF) {
    bk_ble_scan_stop(this->scan_actv_idx_, nullptr);
    this->scan_actv_idx_ = 0xFF;
  }
  this->scan_running_ = false;
}

void BK72xxBLETracker::start_scan_() {
  if (!this->ble_stack_ready_)
    this->ble_stack_init_();
  if (this->scan_running_)
    return;

  if (!this->scan_hw_start_())
    return;

  this->scan_running_ = true;
  this->restart_pending_ = false;
  this->scan_start_time_ = millis();
  this->last_scan_result_time_ = millis();  // reset watchdog timer
  if (!this->scan_started_once_) {
    // Anchor the continuous-mode on_scan_end period to the first scan start (not
    // to boot), so a scan that starts later than scan_duration into uptime does
    // not fire on_scan_end immediately. Set only on the first start, never on
    // watchdog restarts, to keep the period decoupled from coexistence gaps.
    this->scan_period_start_ = millis();
    ESP_LOGI(TAG, "BLE scan started (passive, window=%.0fms, interval=%.0fms)", this->scan_window_ * 0.625f,
             this->scan_interval_ * 0.625f);
    this->scan_started_once_ = true;
  } else {
    ESP_LOGV(TAG, "BLE scan restarted (passive)");
  }
}

// Restart the hardware BLE scan without resetting scan_start_time_.
// Used by the non-continuous watchdog: recovers from WiFi/BLE coexistence gaps
// while keeping the original duration window intact.
void BK72xxBLETracker::restart_scan_hw_() {
  this->stop_scan_hw_();
  if (this->scan_hw_start_()) {
    this->scan_running_ = true;
    this->last_scan_result_time_ = millis();  // reset watchdog timer; scan_start_time_ unchanged
    this->restart_pending_ = false;
    ESP_LOGV(TAG, "BLE scan silent; restarted within non-continuous window");
  } else {
    // Could not bring the controller scan back up (transient bk_ble_scan_start failure):
    // leave scan_running_ false but mark the restart pending so loop() retries it
    // (rate-limited) instead of silently leaving the scan off for the rest of the window.
    this->restart_pending_ = true;
  }
}

void BK72xxBLETracker::stop_scan_() {
  if (!this->scan_running_)
    return;
  this->stop_scan_hw_();
  ESP_LOGI(TAG, "BLE scan stopped");
  for (auto *listener : this->listeners_)
    listener->on_scan_end();
  for (auto *t : this->scan_end_triggers_)
    t->trigger();
  // reset per-scan "Found device" dedup (esp32_ble_tracker parity)
  this->already_discovered_.clear();
  this->scan_period_start_ = millis();  // reset period clock so the watchdog does not double-fire
}

}  // namespace esphome::bk72xx_ble_tracker

#endif  // USE_LIBRETINY
