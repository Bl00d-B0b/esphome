#include "ble_uuid.h"

#if defined(USE_ESP32) && defined(USE_ESP32_BLE_UUID)

#include <cstring>

namespace esphome::esp32_ble {

esp_bt_uuid_t uuid_to_idf(const ESPBTUUID &uuid) {
  esp_bt_uuid_t ret;
  switch (uuid.type()) {
    case ESPBTUUID::Type::UUID16:
      ret.len = ESP_UUID_LEN_16;
      ret.uuid.uuid16 = uuid.uuid16();
      break;
    case ESPBTUUID::Type::UUID32:
      ret.len = ESP_UUID_LEN_32;
      ret.uuid.uuid32 = uuid.uuid32();
      break;
    default:
    case ESPBTUUID::Type::UUID128:
      ret.len = ESP_UUID_LEN_128;
      memcpy(ret.uuid.uuid128, uuid.uuid128(), ESP_UUID_LEN_128);
      break;
  }
  return ret;
}

ESPBTUUID uuid_from_idf(const esp_bt_uuid_t &uuid) {
  if (uuid.len == ESP_UUID_LEN_16)
    return ESPBTUUID::from_uint16(uuid.uuid.uuid16);
  if (uuid.len == ESP_UUID_LEN_32)
    return ESPBTUUID::from_uint32(uuid.uuid.uuid32);
  return ESPBTUUID::from_raw(uuid.uuid.uuid128);
}

}  // namespace esphome::esp32_ble

#endif  // USE_ESP32 && USE_ESP32_BLE_UUID
