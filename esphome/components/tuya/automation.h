#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "tuya.h"

#include <vector>

namespace esphome::tuya {

class TuyaDatapointUpdateTrigger final : public Trigger<TuyaDatapoint> {
 public:
  explicit TuyaDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id) {
    parent->register_listener(sensor_id, [this](const TuyaDatapoint &dp) { this->trigger(dp); });
  }
};

class TuyaRawDatapointUpdateTrigger final : public Trigger<std::vector<uint8_t>> {
 public:
  explicit TuyaRawDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

class TuyaBoolDatapointUpdateTrigger final : public Trigger<bool> {
 public:
  explicit TuyaBoolDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

class TuyaIntDatapointUpdateTrigger final : public Trigger<int> {
 public:
  explicit TuyaIntDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

class TuyaUIntDatapointUpdateTrigger final : public Trigger<uint32_t> {
 public:
  explicit TuyaUIntDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

class TuyaStringDatapointUpdateTrigger final : public Trigger<std::string> {
 public:
  explicit TuyaStringDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

class TuyaEnumDatapointUpdateTrigger final : public Trigger<uint8_t> {
 public:
  explicit TuyaEnumDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

class TuyaBitmaskDatapointUpdateTrigger final : public Trigger<uint32_t> {
 public:
  explicit TuyaBitmaskDatapointUpdateTrigger(Tuya *parent, uint8_t sensor_id);
};

/// Fires with the keys the MCU asked for when it enables the weather service, asks for data, and every update
/// interval while enabled.
class TuyaWeatherRequestTrigger final : public Trigger<std::vector<std::string>> {
 public:
  explicit TuyaWeatherRequestTrigger(Tuya *parent) {
    parent->add_on_weather_request_callback([this](const std::vector<std::string> &keys) { this->trigger(keys); });
  }
};

template<typename... Ts> class TuyaSendWeatherAction final : public Action<Ts...> {
 public:
  explicit TuyaSendWeatherAction(Tuya *parent) : parent_(parent) {}
  void add_int(const std::string &key, TemplatableValue<int32_t, Ts...> value) {
    this->entries_.push_back(Entry{key, false, std::move(value), {}});
  }
  void add_string(const std::string &key, TemplatableValue<std::string, Ts...> value) {
    this->entries_.push_back(Entry{key, true, {}, std::move(value)});
  }
  void play(const Ts &...x) override {
    std::vector<TuyaWeatherValue> values;
    values.reserve(this->entries_.size());
    for (auto &e : this->entries_) {
      if (e.is_string) {
        values.push_back(TuyaWeatherValue{e.key, true, 0, e.string_value.value(x...)});
      } else {
        values.push_back(TuyaWeatherValue{e.key, false, e.int_value.value(x...), {}});
      }
    }
    this->parent_->send_weather(values);
  }

 protected:
  struct Entry {
    std::string key;
    bool is_string;
    TemplatableValue<int32_t, Ts...> int_value;
    TemplatableValue<std::string, Ts...> string_value;
  };
  Tuya *parent_;
  std::vector<Entry> entries_;
};

}  // namespace esphome::tuya
