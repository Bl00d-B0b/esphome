// bk72xx_ble_device.cpp
//
// Implementation of the esp32_ble_tracker compatibility types for BK7231N.
// Parses raw BLE advertisement data into ESPBTDevice.

#ifndef USE_ESP32

#include "bk72xx_ble_device.h"

#include "esphome/core/log.h"

#include <mbedtls/aes.h>

#include <cstdio>
#include <cstring>

namespace esphome::bk72xx_ble_tracker {

#ifdef ESPHOME_LOG_HAS_VERBOSE
static const char *const TAG = "bk72xx_ble_tracker";
#endif

// ---------------------------------------------------------------------------
// ESPBTUUID
// ---------------------------------------------------------------------------

ESPBTUUID ESPBTUUID::from_uint16(uint16_t uuid) {
  ESPBTUUID ret;
  ret.type_ = Type::UUID16;
  ret.uuid_.uuid16 = uuid;
  return ret;
}

ESPBTUUID ESPBTUUID::from_uint32(uint32_t uuid) {
  ESPBTUUID ret;
  ret.type_ = Type::UUID32;
  ret.uuid_.uuid32 = uuid;
  return ret;
}

ESPBTUUID ESPBTUUID::from_raw(const uint8_t *data) {
  ESPBTUUID ret;
  ret.type_ = Type::UUID128;
  memcpy(ret.uuid_.uuid128, data, 16);
  return ret;
}

ESPBTUUID ESPBTUUID::from_raw_reversed(const uint8_t *data) {
  ESPBTUUID ret;
  ret.type_ = Type::UUID128;
  for (int i = 0; i < 16; i++)
    ret.uuid_.uuid128[i] = data[15 - i];
  return ret;
}

bool ESPBTUUID::contains(uint8_t b0, uint8_t b1) const {
  uint16_t target = (static_cast<uint16_t>(b1) << 8) | b0;
  switch (this->type_) {
    case Type::UUID16:
      return this->uuid_.uuid16 == target;
    case Type::UUID32:
      return (this->uuid_.uuid32 & 0xFFFF) == target;
    case Type::UUID128:
      return this->uuid_.uuid128[0] == b0 && this->uuid_.uuid128[1] == b1;
  }
  return false;
}

std::string ESPBTUUID::to_str() const {
  char buf[40];
  switch (this->type_) {
    case Type::UUID16:
      snprintf(buf, sizeof(buf), "0x%04X", this->uuid_.uuid16);
      break;
    case Type::UUID32:
      snprintf(buf, sizeof(buf), "0x%08X", this->uuid_.uuid32);
      break;
    case Type::UUID128:
      snprintf(buf, sizeof(buf), "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
               this->uuid_.uuid128[15], this->uuid_.uuid128[14], this->uuid_.uuid128[13], this->uuid_.uuid128[12],
               this->uuid_.uuid128[11], this->uuid_.uuid128[10], this->uuid_.uuid128[9], this->uuid_.uuid128[8],
               this->uuid_.uuid128[7], this->uuid_.uuid128[6], this->uuid_.uuid128[5], this->uuid_.uuid128[4],
               this->uuid_.uuid128[3], this->uuid_.uuid128[2], this->uuid_.uuid128[1], this->uuid_.uuid128[0]);
      break;
  }
  return std::string(buf);
}

bool ESPBTUUID::operator==(const ESPBTUUID &other) const {
  if (this->type_ != other.type_)
    return false;
  switch (this->type_) {
    case Type::UUID16:
      return this->uuid_.uuid16 == other.uuid_.uuid16;
    case Type::UUID32:
      return this->uuid_.uuid32 == other.uuid_.uuid32;
    case Type::UUID128:
      return memcmp(this->uuid_.uuid128, other.uuid_.uuid128, 16) == 0;
  }
  return false;
}

// ---------------------------------------------------------------------------
// ESPBLEiBeacon
// ---------------------------------------------------------------------------

ESPBLEiBeacon::ESPBLEiBeacon(const uint8_t *data) { memcpy(&this->beacon_data_, data, sizeof(this->beacon_data_)); }

optional<ESPBLEiBeacon> ESPBLEiBeacon::from_manufacturer_data(const ServiceData &data) {
  // Identical detection to esp32_ble_tracker: Apple company ID 0x004C and exactly 23 bytes.
  if (!data.uuid.contains(0x4C, 0x00))
    return {};
  if (data.data.size() != 23)
    return {};
  return ESPBLEiBeacon(data.data.data());
}

// ---------------------------------------------------------------------------
// ESPBTDevice
// ---------------------------------------------------------------------------

void ESPBTDevice::from_scan_result(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data,
                                   uint16_t data_len) {
  memcpy(this->address_, mac, 6);
  this->address_type_ = addr_type;
  this->rssi_ = rssi;
  this->name_.clear();
  this->service_uuids_.clear();
  this->manufacturer_datas_.clear();
  this->service_datas_.clear();
  this->tx_powers_.clear();
  this->appearance_.reset();
  this->ad_flag_.reset();
  this->parse_adv_(data, data_len);

#ifdef ESPHOME_LOG_HAS_VERY_VERBOSE
  ESP_LOGVV(TAG, "Parse Result:");
  char hex_buf[format_hex_pretty_size(62)];  // 62 = ESP32 BLE_ADV_MAX_LOG_BYTES; ':' sep, no length suffix
  const char *address_type;
  switch (this->address_type_) {
    case 0:
      address_type = "PUBLIC";
      break;
    case 1:
      address_type = "RANDOM";
      break;
    case 2:
      address_type = "RPA_PUBLIC";
      break;
    case 3:
      address_type = "RPA_RANDOM";
      break;
    default:
      address_type = "UNKNOWN";
      break;
  }
  ESP_LOGVV(TAG, "  Address: %02X:%02X:%02X:%02X:%02X:%02X (%s)", this->address_[5], this->address_[4],
            this->address_[3], this->address_[2], this->address_[1], this->address_[0], address_type);
  ESP_LOGVV(TAG, "  RSSI: %d", this->rssi_);
  ESP_LOGVV(TAG, "  Name: '%s'", this->name_.c_str());
  for (auto &it : this->tx_powers_) {
    ESP_LOGVV(TAG, "  TX Power: %d", it);
  }
  if (this->appearance_.has_value()) {
    ESP_LOGVV(TAG, "  Appearance: %u", *this->appearance_);
  }
  if (this->ad_flag_.has_value()) {
    ESP_LOGVV(TAG, "  Ad Flag: %u", *this->ad_flag_);
  }
  for (auto &uuid : this->service_uuids_) {
    ESP_LOGVV(TAG, "  Service UUID: %s", uuid.to_str().c_str());
  }
  for (auto &mfr : this->manufacturer_datas_) {
    auto ibeacon = ESPBLEiBeacon::from_manufacturer_data(mfr);
    if (ibeacon.has_value()) {
      ESP_LOGVV(TAG, "  Manufacturer iBeacon:");
      ESP_LOGVV(TAG, "    UUID: %s", ibeacon.value().get_uuid().to_str().c_str());
      ESP_LOGVV(TAG, "    Major: %u", ibeacon.value().get_major());
      ESP_LOGVV(TAG, "    Minor: %u", ibeacon.value().get_minor());
      ESP_LOGVV(TAG, "    TXPower: %d", ibeacon.value().get_signal_power());
    } else {
      ESP_LOGVV(TAG, "  Manufacturer ID: %s, data: %s", mfr.uuid.to_str().c_str(),
                format_hex_pretty_to(hex_buf, mfr.data.data(), mfr.data.size()));
    }
  }
  for (auto &svc : this->service_datas_) {
    ESP_LOGVV(TAG, "  Service data:");
    ESP_LOGVV(TAG, "    UUID: %s", svc.uuid.to_str().c_str());
    ESP_LOGVV(TAG, "    Data: %s", format_hex_pretty_to(hex_buf, svc.data.data(), svc.data.size()));
  }
  ESP_LOGVV(TAG, "  Adv data: %s", format_hex_pretty_to(hex_buf, data, data_len));
#endif
}

std::string ESPBTDevice::address_str() const {
  char buf[MAC_ADDRESS_PRETTY_BUFFER_SIZE];
  return std::string(this->address_str_to(buf));
}

const char *ESPBTDevice::address_str_to(char *buf) const {
  // BK7231N stores MAC bytes as [0]=LSB … [5]=MSB; display MSB-first.
  snprintf(buf, MAC_ADDRESS_PRETTY_BUFFER_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X", this->address_[5], this->address_[4],
           this->address_[3], this->address_[2], this->address_[1], this->address_[0]);
  return buf;
}

uint64_t ESPBTDevice::address_uint64() const {
  uint64_t addr = 0;
  for (int i = 0; i < 6; i++)
    addr |= static_cast<uint64_t>(this->address_[i]) << (i * 8);
  return addr;
}

bool ESPBTDevice::resolve_irk(const uint8_t *irk) const {
  // Bluetooth Core 5.x "ah" function: localHash = e(IRK, padding ‖ prand)[bottom 24 bits].
  // The resolvable private address (RPA) is prand (top 3 bytes) ‖ hash (bottom 3 bytes).
  // This mirrors esp32_ble_tracker::ESPBTDevice::resolve_irk exactly; address_uint64()
  // places the MSB at bit 40, identical to esp32_ble::ble_addr_to_uint64.
  uint8_t ecb_key[16];
  uint8_t ecb_plaintext[16] = {0};
  uint8_t ecb_ciphertext[16];
  const uint64_t addr64 = this->address_uint64();

  memcpy(ecb_key, irk, 16);
  ecb_plaintext[13] = (addr64 >> 40) & 0xff;  // prand (top 3 address bytes)
  ecb_plaintext[14] = (addr64 >> 32) & 0xff;
  ecb_plaintext[15] = (addr64 >> 24) & 0xff;

  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  if (mbedtls_aes_setkey_enc(&ctx, ecb_key, 128) != 0) {
    mbedtls_aes_free(&ctx);
    return false;
  }
  const int rc = mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, ecb_plaintext, ecb_ciphertext);
  mbedtls_aes_free(&ctx);
  if (rc != 0)
    return false;

  // Compare the AES output hash (bottom 24 bits) with the address hash bytes.
  return ecb_ciphertext[15] == (addr64 & 0xff) && ecb_ciphertext[14] == ((addr64 >> 8) & 0xff) &&
         ecb_ciphertext[13] == ((addr64 >> 16) & 0xff);
}

void ESPBTDevice::parse_adv_(const uint8_t *payload, uint16_t len) {
  // BLE AD structure TLV: [length][type][value...]
  // length includes the type byte.
  uint16_t offset = 0;
  while (offset + 2 < len) {
    uint8_t ad_len = payload[offset++];
    if (ad_len == 0)
      continue;  // possible zero-padded advertisement data (matches esp32_ble_tracker)
    if (offset + ad_len > len)
      break;
    uint8_t ad_type = payload[offset];
    const uint8_t *ad_data = &payload[offset + 1];
    uint8_t ad_data_len = ad_len - 1;
    offset += ad_len;

    switch (ad_type) {
      case 0x01:  // Flags
        if (ad_data_len >= 1)
          this->ad_flag_ = ad_data[0];
        break;

      case 0x08:  // Shortened Local Name
      case 0x09:  // Complete Local Name
        // Keep the longest name (matches esp32_ble_tracker): a Shortened Local Name
        // must not overwrite a longer Complete Local Name from the same payload.
        if (ad_data_len > this->name_.length())
          this->name_.assign(reinterpret_cast<const char *>(ad_data), ad_data_len);
        break;

      case 0x0A:  // TX Power Level
        if (ad_data_len >= 1)
          this->tx_powers_.push_back(static_cast<int8_t>(ad_data[0]));
        break;

      case 0x19:  // Appearance
        if (ad_data_len >= 2)
          this->appearance_ = static_cast<uint16_t>(ad_data[0]) | (static_cast<uint16_t>(ad_data[1]) << 8);
        break;

      case 0x02:  // Incomplete List of 16-bit Service UUIDs
      case 0x03:  // Complete List of 16-bit Service UUIDs
        for (uint8_t i = 0; (i + 1) < ad_data_len; i += 2) {
          uint16_t uuid = (static_cast<uint16_t>(ad_data[i + 1]) << 8) | ad_data[i];
          this->service_uuids_.push_back(ESPBTUUID::from_uint16(uuid));
        }
        break;

      case 0x04:  // Incomplete List of 32-bit Service UUIDs
      case 0x05:  // Complete List of 32-bit Service UUIDs
        for (uint8_t i = 0; (i + 3) < ad_data_len; i += 4) {
          uint32_t uuid = (static_cast<uint32_t>(ad_data[i + 3]) << 24) |
                          (static_cast<uint32_t>(ad_data[i + 2]) << 16) | (static_cast<uint32_t>(ad_data[i + 1]) << 8) |
                          ad_data[i];
          this->service_uuids_.push_back(ESPBTUUID::from_uint32(uuid));
        }
        break;

      case 0x06:  // Incomplete List of 128-bit Service UUIDs
      case 0x07:  // Complete List of 128-bit Service UUIDs
        // esp32_ble_tracker records only the first 128-bit UUID in the record.
        if (ad_data_len >= 16)
          this->service_uuids_.push_back(ESPBTUUID::from_raw(ad_data));
        break;

      case 0xFF:  // Manufacturer Specific Data
        if (ad_data_len >= 2) {
          uint16_t company_id = (static_cast<uint16_t>(ad_data[1]) << 8) | ad_data[0];
          ServiceData sd;
          sd.uuid = ESPBTUUID::from_uint16(company_id);
          sd.data.assign(ad_data + 2, ad_data + ad_data_len);
          this->manufacturer_datas_.push_back(std::move(sd));
        } else {
          ESP_LOGV(TAG, "Record length too small for ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE");
        }
        break;

      case 0x16:  // Service Data — 16-bit UUID
        if (ad_data_len >= 2) {
          uint16_t uuid = (static_cast<uint16_t>(ad_data[1]) << 8) | ad_data[0];
          ServiceData sd;
          sd.uuid = ESPBTUUID::from_uint16(uuid);
          sd.data.assign(ad_data + 2, ad_data + ad_data_len);
          this->service_datas_.push_back(std::move(sd));
        } else {
          ESP_LOGV(TAG, "Record length too small for ESP_BLE_AD_TYPE_SERVICE_DATA");
        }
        break;

      case 0x20:  // Service Data — 32-bit UUID
        if (ad_data_len >= 4) {
          uint32_t uuid = (static_cast<uint32_t>(ad_data[3]) << 24) | (static_cast<uint32_t>(ad_data[2]) << 16) |
                          (static_cast<uint32_t>(ad_data[1]) << 8) | ad_data[0];
          ServiceData sd;
          sd.uuid = ESPBTUUID::from_uint32(uuid);
          sd.data.assign(ad_data + 4, ad_data + ad_data_len);
          this->service_datas_.push_back(std::move(sd));
        } else {
          ESP_LOGV(TAG, "Record length too small for ESP_BLE_AD_TYPE_32SERVICE_DATA");
        }
        break;

      case 0x21:  // Service Data — 128-bit UUID
        if (ad_data_len >= 16) {
          ServiceData sd;
          sd.uuid = ESPBTUUID::from_raw(ad_data);
          sd.data.assign(ad_data + 16, ad_data + ad_data_len);
          this->service_datas_.push_back(std::move(sd));
        } else {
          ESP_LOGV(TAG, "Record length too small for ESP_BLE_AD_TYPE_128SERVICE_DATA");
        }
        break;

      case 0x12:  // Peripheral Connection Interval Range
        // Avoid logging this as it's very verbose (matches esp32_ble_tracker)
        break;

      default:
        ESP_LOGV(TAG, "Unhandled type: advType: 0x%02x", ad_type);
        break;
    }
  }
}

}  // namespace esphome::bk72xx_ble_tracker

#endif  // !USE_ESP32
