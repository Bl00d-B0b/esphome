#pragma once

#include "esphome/core/defines.h"

#ifdef USE_ESP32
#ifdef USE_ESP32_BLE_UUID

// The BLE UUID type is owned by the platform-neutral ble_device_base layer;
// this header re-exports it under the historical esp32_ble name (esp32 only)
// and provides the ESP-IDF conversions, so ESP-IDF types never appear in the
// neutral type's own surface.

#include "esphome/components/ble_device_base/ble_device.h"

#include <esp_bt_defs.h>

namespace esphome::esp32_ble {

using ble_device_base::UUID_STR_LEN;
using ESPBTUUID = ble_device_base::ESPBTUUID;

/// Convert the neutral UUID to the ESP-IDF representation.
esp_bt_uuid_t uuid_to_idf(const ESPBTUUID &uuid);
/// Construct the neutral UUID from the ESP-IDF representation.
ESPBTUUID uuid_from_idf(const esp_bt_uuid_t &uuid);

}  // namespace esphome::esp32_ble

#endif  // USE_ESP32_BLE_UUID
#endif  // USE_ESP32
