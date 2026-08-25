#include "rtl87xx_ble.h"

// Only compiled when the component is configured: the vendor SDK headers exist
// only when LibreTiny's CONFIG_BT option pulled them in.
#if defined(USE_LIBRETINY) && defined(USE_RTL87XX_BLE)

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

#include <atomic>
#include <cstring>

#include <libretiny.h>

// Vendor SDK headers: on the include path via the family builder's CONFIG_BT
// block on AmebaD, unconditionally on AmebaZ2.
extern "C" {
#include <FreeRTOS.h>
#include <task.h>

#include <gap.h>
#include <gap_callback_le.h>
#include <gap_config.h>
#include <gap_le.h>
#include <gap_le_types.h>
#include <gap_msg.h>
#include <gap_scan.h>

#include <app_msg.h>
#include <bte.h>
#include <os_msg.h>
#include <os_sched.h>
#include <os_task.h>
#include <rtk_coex.h>
#include <trace_app.h>
#include <wifi_conf.h>

int bt_get_mac_address(uint8_t *mac);
#ifdef USE_LIBRETINY_VARIANT_RTL8720D
// AmebaD only: AmebaZ2's coexistence is driven entirely through bt_coex_init()
// and the mailbox; its SDK has no wifi-side switch.
void wifi_btcoex_set_bt_on(void);
#endif

// bt_uart_tx: HCI debug-bridge TX, referenced by hci_uart.c but only used by
// the AT-command bridge that is not built. Weak stub keeps the link resolved.
__attribute__((weak)) void bt_uart_tx(uint8_t rc) { (void) rc; }
}

namespace esphome::rtl87xx_ble {

static const char *const TAG = "rtl87xx_ble";

// Only this TU sees the vendor headers, so the demux values are pinned here:
// a renumbered SDK fails the build instead of misrouting scan responses.
static_assert(ADV_EVENT_TYPE_ADV_IND == GAP_ADV_EVT_TYPE_UNDIRECTED, "GAP adv event type renumbered");
static_assert(ADV_EVENT_TYPE_ADV_SCAN_IND == GAP_ADV_EVT_TYPE_SCANNABLE, "GAP adv event type renumbered");
static_assert(ADV_EVENT_TYPE_SCAN_RSP == GAP_ADV_EVT_TYPE_SCAN_RSP, "GAP adv event type renumbered");

static constexpr size_t EVT_QUEUE_LEN = 0x40;
static constexpr size_t IO_QUEUE_LEN = 0x20;

// Bring-up order follows OpenBeken's hal_bt_proxy_rtl8720d.c: stack after the
// WiFi STA, coexistence enabled (bt_coex_init, plus wifi_btcoex_set_bt_on on
// AmebaD), GAP events pumped by a dedicated task. The same sequence compiles
// for AmebaZ2 (same Realtek GAP SDK); hardware-verified on AmebaD only.
//
// The GAP callbacks are plain C with no user argument, so state lives in
// file statics (one stack instance per SoC).
// NOLINTBEGIN(cppcoreguidelines-avoid-non-const-global-variables)
static volatile bool s_ble_ready = false;
static volatile bool s_ble_starting = false;
static volatile bool s_scan_running = false;
// Written from the main loop, read from the GAP task; atomic so a clear
// (nullptr) can never be torn.
static std::atomic<raw_adv_callback_t> s_adv_cb{nullptr};

static uint16_t s_scan_interval_units = 0x520;  // 820 ms, the GAP reference default
static uint16_t s_scan_window_units = 0x520;
static uint8_t s_scan_mode = GAP_SCAN_MODE_PASSIVE;

static void *s_evt_queue = nullptr;
static void *s_io_queue = nullptr;
// NOLINTEND(cppcoreguidelines-avoid-non-const-global-variables)

static T_APP_RESULT gap_callback(uint8_t cb_type, void *p_cb_data) {
  auto *p_data = static_cast<T_LE_CB_DATA *>(p_cb_data);
  if (p_data == nullptr || cb_type != GAP_MSG_LE_SCAN_INFO || p_data->p_le_scan_info == nullptr)
    return APP_RESULT_SUCCESS;
  T_LE_SCAN_INFO *info = p_data->p_le_scan_info;
  raw_adv_callback_t cb = s_adv_cb.load(std::memory_order_acquire);
  if (cb != nullptr) {
    cb(info->bd_addr, (uint8_t) info->remote_addr_type, (uint8_t) info->adv_type, (int8_t) info->rssi, info->data,
       info->data_len);
  }
  return APP_RESULT_SUCCESS;
}

static void pump_task(void *param) {
  (void) param;
  uint8_t event;
  os_msg_queue_create(&s_io_queue, IO_QUEUE_LEN, sizeof(T_IO_MSG));
  os_msg_queue_create(&s_evt_queue, EVT_QUEUE_LEN, sizeof(uint8_t));
  gap_start_bt_stack(s_evt_queue, s_io_queue, IO_QUEUE_LEN);
  while (true) {
    if (os_msg_recv(s_evt_queue, &event, 0xFFFFFFFF)) {
      if (event == EVENT_IO_TO_APP) {
        T_IO_MSG io_msg;
        os_msg_recv(s_io_queue, &io_msg, 0);
      } else {
        gap_handle_msg(event);
      }
    }
  }
}

static void init_task(void *param) {
  (void) param;

  // coex bring-up needs a running STA interface
  while (!wifi_is_up(RTW_STA_INTERFACE)) {
    os_delay(100);
  }

  T_GAP_DEV_STATE state;
  bt_trace_init();
  le_get_gap_param(GAP_PARAM_DEV_STATE, &state);
  if (state.gap_init_state != GAP_INIT_STATE_STACK_READY) {
    bte_init();
    gap_config_max_le_link_num(2);
    le_gap_init(2);
  }

  uint8_t filter_policy = GAP_SCAN_FILTER_ANY;
  uint8_t filter_duplicate = GAP_SCAN_FILTER_DUPLICATE_DISABLE;
  le_scan_set_param(GAP_PARAM_SCAN_INTERVAL, sizeof(s_scan_interval_units), &s_scan_interval_units);
  le_scan_set_param(GAP_PARAM_SCAN_WINDOW, sizeof(s_scan_window_units), &s_scan_window_units);
  le_scan_set_param(GAP_PARAM_SCAN_MODE, sizeof(s_scan_mode), &s_scan_mode);
  le_scan_set_param(GAP_PARAM_SCAN_FILTER_POLICY, sizeof(filter_policy), &filter_policy);
  le_scan_set_param(GAP_PARAM_SCAN_FILTER_DUPLICATES, sizeof(filter_duplicate), &filter_duplicate);

  le_register_app_cb(gap_callback);

  xTaskCreate(pump_task, "rtl_ble_pump", 1024, nullptr, 1, nullptr);

  bt_coex_init();
  do {
    os_delay(100);
    le_get_gap_param(GAP_PARAM_DEV_STATE, &state);
  } while (state.gap_init_state != GAP_INIT_STATE_STACK_READY);
#ifdef USE_LIBRETINY_VARIANT_RTL8720D
  wifi_btcoex_set_bt_on();
#endif
  os_delay(50);

  s_ble_ready = true;
  ESP_LOGI(TAG, "BLE stack ready");
  vTaskDelete(nullptr);
}

void RTL87xxBLE::setup() {
  if (s_ble_ready || s_ble_starting)
    return;
  s_ble_starting = true;
  xTaskCreate(init_task, "rtl_ble_init", 1024, nullptr, 1, nullptr);
}

bool RTL87xxBLE::stack_ready() { return s_ble_ready; }

// Same predicate init_task() blocks on, so callers see the wait it is in.
bool RTL87xxBLE::waiting_for_network() { return !s_ble_ready && !wifi_is_up(RTW_STA_INTERFACE); }

void RTL87xxBLE::set_adv_callback(raw_adv_callback_t cb) { s_adv_cb.store(cb, std::memory_order_release); }

void RTL87xxBLE::set_scan_params(uint16_t interval_ms, uint16_t window_ms, bool active) {
  // GAP units are 0.625 ms
  s_scan_interval_units = (uint16_t) ((interval_ms * 16) / 10);
  s_scan_window_units = (uint16_t) ((window_ms * 16) / 10);
  s_scan_mode = active ? GAP_SCAN_MODE_ACTIVE : GAP_SCAN_MODE_PASSIVE;
  if (s_ble_ready) {
    le_scan_set_param(GAP_PARAM_SCAN_INTERVAL, sizeof(s_scan_interval_units), &s_scan_interval_units);
    le_scan_set_param(GAP_PARAM_SCAN_WINDOW, sizeof(s_scan_window_units), &s_scan_window_units);
    le_scan_set_param(GAP_PARAM_SCAN_MODE, sizeof(s_scan_mode), &s_scan_mode);
  }
}

bool RTL87xxBLE::scan_start() {
  if (!s_ble_ready || s_scan_running)
    return s_scan_running;
  if (le_scan_start() != GAP_CAUSE_SUCCESS) {
    ESP_LOGW(TAG, "le_scan_start failed");
    return false;
  }
  s_scan_running = true;
  return true;
}

void RTL87xxBLE::scan_stop() {
  if (!s_ble_ready || !s_scan_running)
    return;
  if (le_scan_stop() != GAP_CAUSE_SUCCESS) {
    // Leave the flag set: the tracker's reconciler must see that the radio is
    // still scanning rather than trust a stop that did not happen.
    ESP_LOGW(TAG, "le_scan_stop failed");
    return;
  }
  s_scan_running = false;
}

bool RTL87xxBLE::scan_running() {
  // The stack's own state, not a local flag, so a controller-side drop is
  // visible to the tracker's reconciler.
  if (!s_ble_ready)
    return false;
  T_GAP_DEV_STATE state;
  if (le_get_gap_param(GAP_PARAM_DEV_STATE, &state) != GAP_CAUSE_SUCCESS)
    return s_scan_running;
  bool running = state.gap_scan_state == GAP_SCAN_STATE_START || state.gap_scan_state == GAP_SCAN_STATE_SCANNING;
  s_scan_running = running;
  return running;
}

void RTL87xxBLE::get_mac(uint8_t out[6]) {
  uint8_t mac[6] = {0};
  bt_get_mac_address(mac);
  bool zero = true, ff = true;
  for (uint8_t b : mac) {
    zero &= b == 0x00;
    ff &= b == 0xFF;
  }
  if (zero || ff) {
    // No BT eFuse slot; use the running WiFi MAC. lt_get_device_mac() reads the
    // factory eFuse, unprogrammed on some modules and then all-zero.
    get_mac_address_raw(out);
    return;
  }
  // bt_get_mac_address already returns printable order (the WiFi MAC's OUI with
  // the last byte offset), so it is copied as-is.
  memcpy(out, mac, 6);
}

}  // namespace esphome::rtl87xx_ble

#endif  // USE_LIBRETINY && USE_RTL87XX_BLE
