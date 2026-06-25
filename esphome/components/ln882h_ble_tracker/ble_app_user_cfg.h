#pragma once

// BLE stack configuration for ln882h_ble_tracker.
// Observer role only — scans for advertisements, no advertising, no GATT server.

// Central role: enables scanning without advertising
#define BLE_DEFAULT_ROLE BLE_ROLE_CENTRAL

#define BLE_DEFAULT_DEVICE_NAME ("ln882h_ble_tracker")
#define BLE_DEV_NAME_MAX_LEN (40)

// Address is managed by ln882h_ble_tracker.cpp resolve_mac_(): the BLE MAC is
// derived from the WiFi MAC (low 24-bit NIC + 1) and pushed to the controller via
// rw_init(). This default is only a placeholder until that override runs.
//#define BLE_USE_STATIC_PUBLIC_ADDR
#define BLE_DEFAULT_PUBLIC_ADDR \
  { 0x00, 0x50, 0xC2, 0x00, 0x00, 0x00 }

// No auto-advertising, no auto-scanning — tracker controls scan lifecycle
#define BLE_CONFIG_AUTO_ADV (0)
#define BLE_CONFIG_AUTO_SCAN (0)

// No encryption needed for passive scan
//#define BLE_CONFIG_SECURITY       (1)

// No GATT profiles — observer only, no connections
// (all CFG_PRF_* left undefined)

#ifdef __cplusplus
// Component namespace marker required by ESPHome ci-custom.
namespace esphome::ln882h_ble_tracker {}  // namespace esphome::ln882h_ble_tracker
#endif
