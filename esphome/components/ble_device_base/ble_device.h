// ble_device.h
//
// Universal BLE-device header. A BLE sensor includes ONLY this header and gets the
// `esp32_ble_tracker::` advertisement types (ESPBTUUID, ServiceData, ESPBLEiBeacon,
// ESPBTDevice, ESPBTDeviceListener) on EVERY platform:
//   - ESP32: forwards to esp32_ble_tracker.h (the canonical ESP-IDF-backed types).
//   - non-ESP32 (LibreTiny ln882x / bk72xx / …): provides neutral reimplementations
//     and aliases them into the esp32_ble_tracker namespace, so the same sensor C++
//     compiles unchanged.

#pragma once

#ifdef USE_ESP32

// On ESP32 the canonical types live in esp32_ble_tracker; forward to them.
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#else  // !USE_ESP32

#include "esphome/core/helpers.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace esphome::ble_device_base {

using adv_data_t = std::vector<uint8_t>;

// ---------------------------------------------------------------------------
// ESPBTUUID
// ---------------------------------------------------------------------------

class ESPBTUUID {
 public:
  static ESPBTUUID from_uint16(uint16_t uuid);
  static ESPBTUUID from_uint32(uint32_t uuid);
  /// Construct from raw 16-byte little-endian UUID.
  static ESPBTUUID from_raw(const uint8_t *data);
  /// Construct from raw 16-byte big-endian UUID (reversed on store).
  static ESPBTUUID from_raw_reversed(const uint8_t *data);

  /// True if the UUID encodes the 16-bit value [b1 << 8 | b0] (little-endian pair).
  /// Used by BTHome: contains(0xD2, 0xFC) matches UUID 0xFCD2.
  bool contains(uint8_t b0, uint8_t b1) const;

  bool operator==(const ESPBTUUID &other) const;
  bool operator!=(const ESPBTUUID &other) const { return !(*this == other); }

  std::string to_str() const;
  /// Buffer overload: matches ESP32's to_str(std::span<char,37>) signature.
  /// Writes into buf (must be ≥37 bytes) and returns buf.
  const char *to_str(char *buf) const {
    auto s = this->to_str();
    strncpy(buf, s.c_str(), 36);
    buf[36] = '\0';
    return buf;
  }

 protected:
  enum class Type : uint8_t { UUID16, UUID32, UUID128 };
  Type type_{Type::UUID16};
  union {
    uint16_t uuid16;
    uint32_t uuid32;
    uint8_t uuid128[16];
  } uuid_{};
};

// ---------------------------------------------------------------------------
// ServiceData — UUID-tagged advertisement payload (0x16 / 0xFF AD types)
// ---------------------------------------------------------------------------

struct ServiceData {
  ESPBTUUID uuid;
  adv_data_t data;
};

// ---------------------------------------------------------------------------
// ESPBLEiBeacon
// ---------------------------------------------------------------------------

class ESPBLEiBeacon {
 public:
  ESPBLEiBeacon() { memset(&this->beacon_data_, 0, sizeof(this->beacon_data_)); }
  explicit ESPBLEiBeacon(const uint8_t *data);
  static optional<ESPBLEiBeacon> from_manufacturer_data(const ServiceData &data);

  uint16_t get_major() const { return byteswap(this->beacon_data_.major); }
  uint16_t get_minor() const { return byteswap(this->beacon_data_.minor); }
  int8_t get_signal_power() const { return this->beacon_data_.signal_power; }
  ESPBTUUID get_uuid() const { return ESPBTUUID::from_raw_reversed(this->beacon_data_.proximity_uuid); }

 protected:
  struct PACKED BeaconData {
    uint8_t sub_type;
    uint8_t length;
    uint8_t proximity_uuid[16];
    uint16_t major;
    uint16_t minor;
    int8_t signal_power;
  } beacon_data_;
};

// ---------------------------------------------------------------------------
// ESPBTDevice — parsed BLE advertisement
// ---------------------------------------------------------------------------

class ESPBTDevice {
 public:
  /// Populate from a raw any non-ESP32 BLE platform scan result.
  void from_scan_result(const uint8_t *mac, int rssi, uint8_t addr_type, const uint8_t *data, uint16_t data_len);

  static constexpr size_t MAC_ADDRESS_PRETTY_BUFFER_SIZE = 18;

  /// Return MAC as "XX:XX:XX:XX:XX:XX" string.
  std::string address_str() const;
  /// Buffer overload: writes "XX:XX:XX:XX:XX:XX\0" into buf (must be ≥18 bytes), returns buf.
  /// Matches ESP32's address_str_to(char *buf) signature.
  const char *address_str_to(char *buf) const;
  /// Return MAC as packed uint64 (byte 0 in LSB, matching ESP32 address_uint64).
  uint64_t address_uint64() const;
  const uint8_t *address() const { return address_; }

  int get_rssi() const { return rssi_; }
  const std::string &get_name() const { return name_; }

  const std::vector<ESPBTUUID> &get_service_uuids() const { return service_uuids_; }
  const std::vector<ServiceData> &get_manufacturer_datas() const { return manufacturer_datas_; }
  const std::vector<ServiceData> &get_service_datas() const { return service_datas_; }
  const std::vector<int8_t> &get_tx_powers() const { return tx_powers_; }
  const optional<uint16_t> &get_appearance() const { return appearance_; }
  const optional<uint8_t> &get_ad_flag() const { return ad_flag_; }

  /// Resolve a Resolvable Private Address against a 16-byte IRK using the
  /// Bluetooth Core "ah" function (AES-128-ECB). Identical algorithm to
  /// esp32_ble_tracker::ESPBTDevice::resolve_irk. Implemented in the .cpp via
  /// mbedtls (LibreTiny links mbedtls for TLS, so AES-128 is always available).
  bool resolve_irk(const uint8_t *irk) const;

  optional<ESPBLEiBeacon> get_ibeacon() const {
    for (const auto &it : this->manufacturer_datas_) {
      auto res = ESPBLEiBeacon::from_manufacturer_data(it);
      if (res.has_value())
        return res;
    }
    return {};
  }

 protected:
  void parse_adv_(const uint8_t *payload, uint16_t len);

  uint8_t address_[6]{0};
  uint8_t address_type_{0};
  int rssi_{0};
  std::string name_{};
  std::vector<ESPBTUUID> service_uuids_{};
  std::vector<ServiceData> manufacturer_datas_{};
  std::vector<ServiceData> service_datas_{};
  std::vector<int8_t> tx_powers_{};
  optional<uint16_t> appearance_{};
  optional<uint8_t> ad_flag_{};
};

// ---------------------------------------------------------------------------
// ESPBTDeviceListener — base class for BLE sensor components
// ---------------------------------------------------------------------------

class ESPBTDeviceListener {
 public:
  virtual ~ESPBTDeviceListener() = default;
  /// Called at the end of each scan duration period.
  virtual void on_scan_end() {}
  virtual bool parse_device(const ESPBTDevice &device) = 0;
};

}  // namespace esphome::ble_device_base

// ---------------------------------------------------------------------------
// Compatibility aliases — expose the types in the esp32_ble_tracker namespace
// so that sensor components compiled with the any non-ESP32 BLE platform backend can include this
// header and use esp32_ble_tracker:: qualified names without modification.
// ---------------------------------------------------------------------------
namespace esphome::esp32_ble_tracker {
using adv_data_t = esphome::ble_device_base::adv_data_t;
using ESPBTUUID = esphome::ble_device_base::ESPBTUUID;
using ServiceData = esphome::ble_device_base::ServiceData;
using ESPBLEiBeacon = esphome::ble_device_base::ESPBLEiBeacon;
using ESPBTDevice = esphome::ble_device_base::ESPBTDevice;
using ESPBTDeviceListener = esphome::ble_device_base::ESPBTDeviceListener;
}  // namespace esphome::esp32_ble_tracker

#endif  // !USE_ESP32
