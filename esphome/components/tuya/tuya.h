#pragma once

#include <cinttypes>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/time.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::tuya {

enum class TuyaDatapointType : uint8_t {
  RAW = 0x00,      // variable length
  BOOLEAN = 0x01,  // 1 byte (0/1)
  INTEGER = 0x02,  // 4 byte
  STRING = 0x03,   // variable length
  ENUM = 0x04,     // 1 byte
  BITMASK = 0x05,  // 1/2/4 bytes
};

struct TuyaDatapoint {
  uint8_t id;
  TuyaDatapointType type;
  size_t len;
  union {
    bool value_bool;
    int value_int;
    uint32_t value_uint;
    uint8_t value_enum;
    uint32_t value_bitmask;
  };
  std::string value_string;
  std::vector<uint8_t> value_raw;
};

struct TuyaDatapointListener {
  uint8_t datapoint_id;
  std::function<void(TuyaDatapoint)> on_datapoint;
};

enum class TuyaCommandType : uint8_t {
  HEARTBEAT = 0x00,
  PRODUCT_QUERY = 0x01,
  CONF_QUERY = 0x02,
  WIFI_STATE = 0x03,
  WIFI_RESET = 0x04,
  WIFI_SELECT = 0x05,
  DATAPOINT_DELIVER = 0x06,
  DATAPOINT_REPORT_ASYNC = 0x07,
  DATAPOINT_QUERY = 0x08,
  GMT_TIME_QUERY = 0x0C,
  WIFI_TEST = 0x0E,
  LOCAL_TIME_QUERY = 0x1C,
  WEATHER_ENABLE = 0x20,  // MCU asks for the weather service with a list of keys
  WEATHER_DATA = 0x21,    // module sends weather values, MCU acks with an empty 0x21
  DATAPOINT_REPORT_SYNC = 0x22,
  DATAPOINT_REPORT_ACK = 0x23,
  WIFI_RSSI = 0x24,
  VACUUM_MAP_UPLOAD = 0x28,
  GET_NETWORK_STATUS = 0x2B,
  EXTENDED_SERVICES = 0x34,
};

enum class TuyaExtendedServicesCommandType : uint8_t {
  REQUEST_WEATHER = 0x03,  // MCU asks for weather data now (at most once per 20 min)
  RESET_NOTIFICATION = 0x04,
  MODULE_RESET = 0x05,
  UPDATE_IN_PROGRESS = 0x0A,
};

enum class TuyaInitState : uint8_t {
  INIT_HEARTBEAT = 0x00,
  INIT_PRODUCT,
  INIT_CONF,
  INIT_WIFI,
  INIT_DATAPOINT,
  INIT_DONE,
};

struct TuyaCommand {
  TuyaCommandType cmd;
  std::vector<uint8_t> payload;
};

/// One weather value for the MCU: Tuya key (w.temp, w.conditionNum, ...), int32 or string.
struct TuyaWeatherValue {
  std::string key;
  bool is_string;
  int32_t value_int;
  std::string value_string;
};

class Tuya final : public Component, public uart::UARTDevice {
 public:
  float get_setup_priority() const override { return setup_priority::LATE; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void register_listener(uint8_t datapoint_id, const std::function<void(TuyaDatapoint)> &func);
  void set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value);
  void set_boolean_datapoint_value(uint8_t datapoint_id, bool value);
  void set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value);
  void set_status_pin(InternalGPIOPin *status_pin) { this->status_pin_ = status_pin; }
  void set_string_datapoint_value(uint8_t datapoint_id, const std::string &value);
  void set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value);
  void set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length);
  void force_set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value);
  void force_set_boolean_datapoint_value(uint8_t datapoint_id, bool value);
  void force_set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value);
  void force_set_string_datapoint_value(uint8_t datapoint_id, const std::string &value);
  void force_set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value);
  void force_set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length);
  TuyaInitState get_init_state();
#ifdef USE_TIME
  void set_time_id(time::RealTimeClock *time_id) { this->time_id_ = time_id; }
#endif
  void add_ignore_mcu_update_on_datapoints(uint8_t ignore_mcu_update_on_datapoints) {
    this->ignore_mcu_update_on_datapoints_.push_back(ignore_mcu_update_on_datapoints);
  }
  template<typename F> void add_on_initialized_callback(F &&callback) {
    this->initialized_callback_.add(std::forward<F>(callback));
  }
#ifdef USE_TUYA_MCU_WATCHDOG
  void set_mcu_watchdog(uint8_t missed_heartbeats) { this->watchdog_missed_heartbeats_ = missed_heartbeats; }
#endif
#ifdef USE_TUYA_MCU_WATCHDOG_RESET
  void set_mcu_watchdog_reset(InternalGPIOPin *tx_pin, uint32_t reset_time) {
    this->tx_pin_ = tx_pin;
    this->watchdog_reset_time_ = reset_time;
  }
  /// Number of MCU resets since boot.
  uint32_t get_mcu_resets() const { return this->mcu_resets_; }
#endif

  /// Weather service: the MCU enables it with 0x20 (list of keys) and asks for data with 0x34/0x03. The
  /// component acks both and answers with one 0x21 frame: values from send_weather() calls made by the request
  /// callback's automations first, then the configured sources (add_weather_sensor / add_weather_text_sensor)
  /// for the remaining keys. With `interval_ms` != 0 the same happens every interval while the service is
  /// enabled (Tuya's module pushes every 30 min); 0 = only on the MCU's requests.
  void set_weather_service(uint32_t interval_ms) {
    this->weather_configured_ = true;
    this->weather_update_interval_ = interval_ms;
  }
#ifdef USE_SENSOR
  /// Numeric source for a key: int keys send lroundf(state), w.windSpeed sends "%.1f"; NaN = key not sent.
  void add_weather_sensor(const std::string &key, sensor::Sensor *source) {
    this->weather_sensors_.emplace_back(key, source);
  }
#endif
#ifdef USE_TEXT_SENSOR
  /// String source for a key (w.conditionNum, w.windDir, ...); no state = key not sent.
  void add_weather_text_sensor(const std::string &key, text_sensor::TextSensor *source) {
    this->weather_text_sensors_.emplace_back(key, source);
  }
#endif
  template<typename F> void add_on_weather_request_callback(F &&callback) {
    this->weather_request_callback_.add(std::forward<F>(callback));
  }
  /// Send weather values to the MCU (0x21). Inside the request callback the values are collected into the
  /// answer frame; outside it a frame goes out at once. Keys the MCU did not ask for are skipped. Returns false
  /// when nothing was queued (service not enabled, MCU not initialised, no values).
  bool send_weather(const std::vector<TuyaWeatherValue> &values);

 protected:
  void handle_char_(uint8_t c);
  void handle_datapoints_(const uint8_t *buffer, size_t len);
  optional<TuyaDatapoint> get_datapoint_(uint8_t datapoint_id);
  bool validate_message_();

  void handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len);
  void send_raw_command_(TuyaCommand command);
  void process_command_queue_();
  void send_command_(const TuyaCommand &command);
  void send_empty_command_(TuyaCommandType command);
  void set_numeric_datapoint_value_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, uint32_t value,
                                    uint8_t length, bool forced);
  void set_string_datapoint_value_(uint8_t datapoint_id, const std::string &value, bool forced);
  void set_raw_datapoint_value_(uint8_t datapoint_id, const std::vector<uint8_t> &value, bool forced);
  void send_datapoint_command_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, std::vector<uint8_t> data);
  void set_status_pin_();
  void set_heartbeat_interval_(uint32_t interval);
  void send_wifi_status_();
  uint8_t get_wifi_status_code_();
  uint8_t get_wifi_rssi_();

#ifdef USE_TUYA_MCU_WATCHDOG
  void note_missed_heartbeat_();
  bool watchdog_reboot_() const;
  uint8_t watchdog_missed_heartbeats_{0};
  uint8_t heartbeat_misses_{0};
#endif
#ifdef USE_TUYA_MCU_WATCHDOG_RESET
  void start_tx_break_();
  void end_tx_break_();
  InternalGPIOPin *tx_pin_{nullptr};
  uint32_t watchdog_reset_time_{500};
  uint32_t tx_break_start_{0};
  uint32_t mcu_resets_{0};
  bool tx_break_active_{false};
  bool tx_break_unconfirmed_{false};
#endif
#ifdef USE_TIME
  void send_local_time_();
  void send_gmt_time_();
  time::RealTimeClock *time_id_{nullptr};
  bool time_sync_callback_registered_{false};
  bool gmt_time_sync_callback_registered_{false};
#endif
  TuyaInitState init_state_ = TuyaInitState::INIT_HEARTBEAT;
  bool init_failed_{false};
  uint8_t protocol_version_ = -1;
  InternalGPIOPin *status_pin_{nullptr};
  int status_pin_reported_ = -1;
  int reset_pin_reported_ = -1;
  uint32_t last_command_timestamp_ = 0;
  uint32_t last_rx_char_timestamp_ = 0;
  bool working_mode_received_{false};
  bool resent_{false};
  std::string product_;
  std::vector<TuyaDatapointListener> listeners_;
  std::vector<TuyaDatapoint> datapoints_;
  std::vector<uint8_t> rx_message_;
  std::vector<uint8_t> ignore_mcu_update_on_datapoints_{};
  std::vector<TuyaCommand> command_queue_;
  optional<TuyaCommandType> expected_response_{};
  uint8_t wifi_status_ = -1;
  CallbackManager<void()> initialized_callback_{};

  // Weather service
  void handle_weather_enable_(const uint8_t *buffer, size_t len);
  bool request_weather_(uint32_t now);
  bool send_weather_frame_(std::vector<TuyaWeatherValue> values);
  bool weather_frame_queued_() const;
  bool weather_key_requested_(const std::string &key) const;
  bool weather_configured_ = false;       // weather: block present
  uint32_t weather_update_interval_ = 0;  // 0 = only on the MCU's requests
  bool weather_enabled_ = false;          // MCU sent 0x20 (or 0x34/0x03)
  bool collecting_weather_ = false;       // inside request_weather_: send_weather() collects into pending_weather_
  std::vector<std::string> weather_keys_;
  uint8_t weather_forecast_days_ = 0;  // w.date.N in the enable list: values w.<key>.<day> for day < N
  std::vector<TuyaWeatherValue> pending_weather_;
  uint32_t last_weather_request_ = 0;  // last time a request was answered (or attempted)
  uint32_t last_weather_demand_ = 0;   // last 0x34/0x03 that counted against the 20 min limit
  CallbackManager<void(std::vector<std::string>)> weather_request_callback_{};
#ifdef USE_SENSOR
  std::vector<std::pair<std::string, sensor::Sensor *>> weather_sensors_;
#endif
#ifdef USE_TEXT_SENSOR
  std::vector<std::pair<std::string, text_sensor::TextSensor *>> weather_text_sensors_;
#endif
};

}  // namespace esphome::tuya
