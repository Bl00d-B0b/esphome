#include "tuya.h"
#include <algorithm>
#include <cmath>
#include "esphome/core/application.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"

#ifdef USE_NETWORK
#include "esphome/components/network/util.h"
#endif

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#ifdef USE_CAPTIVE_PORTAL
#include "esphome/components/captive_portal/captive_portal.h"
#endif

#if defined(USE_TUYA_MCU_RECOVERY_BREAK) && defined(USE_LIBRETINY)
#include "esphome/components/uart/uart_component_libretiny.h"
#endif

namespace esphome::tuya {

static const char *const TAG = "tuya";
static const int COMMAND_DELAY = 10;
static const int RECEIVE_TIMEOUT = 300;
static const int MAX_RETRIES = 5;
// Tuya: an on-demand weather request (0x34/0x03) is served at most once per 20 min.
static const uint32_t WEATHER_DEMAND_MIN_INTERVAL = 20 * 60 * 1000;
// The MCU sends 0x20 and 0x34/0x03 back to back; one request callback answers both.
static const uint32_t WEATHER_REQUEST_DEBOUNCE = 5000;
// Max bytes to log for datapoint values (larger values are truncated)
static constexpr size_t MAX_DATAPOINT_LOG_BYTES = 16;
#ifdef USE_TUYA_MCU_RECOVERY_BREAK
// Shortest time between two TX breaks
static constexpr uint32_t RECOVERY_MIN_INTERVAL = 60000;
#endif

static bool network_is_connected() {
#ifdef USE_NETWORK
  return network::is_connected();
#else
  return false;
#endif
}

void Tuya::setup() {
  this->set_heartbeat_interval_(this->init_heartbeat_interval_);
  if (this->status_pin_ != nullptr) {
    this->status_pin_->digital_write(false);
  }
}

void Tuya::loop() {
#ifdef USE_TUYA_MCU_RECOVERY_BREAK
  if (this->tx_break_active_) {
    if (millis() - this->tx_break_start_ < this->recovery_break_time_)
      return;
    this->end_tx_break_();
  }
#endif
  if (this->weather_enabled_ && this->weather_update_interval_ != 0 &&
      millis() - this->last_weather_request_ >= this->weather_update_interval_ && !this->weather_frame_queued_()) {
    this->request_weather_(millis());
  }
  // Read all available bytes in batches to reduce UART call overhead.
  size_t avail = this->available();
  uint8_t buf[64];
  while (avail > 0) {
    size_t to_read = std::min(avail, sizeof(buf));
    if (!this->read_array(buf, to_read)) {
      break;
    }
    avail -= to_read;

    for (size_t i = 0; i < to_read; i++) {
      this->handle_char_(buf[i]);
    }
  }
  process_command_queue_();
}

void Tuya::set_heartbeat_interval_(uint32_t interval) {
  this->set_interval("heartbeat", interval, [this] { this->send_empty_command_(TuyaCommandType::HEARTBEAT); });
}

void Tuya::dump_config() {
  ESP_LOGCONFIG(TAG, "Tuya:");
  if (this->init_state_ != TuyaInitState::INIT_DONE) {
    if (this->init_failed_) {
      ESP_LOGCONFIG(TAG, "  Initialization failed. Current init_state: %u", static_cast<uint8_t>(this->init_state_));
    } else {
      ESP_LOGCONFIG(TAG, "  Configuration will be reported when setup is complete. Current init_state: %u",
                    static_cast<uint8_t>(this->init_state_));
    }
    ESP_LOGCONFIG(TAG, "  If no further output is received, confirm that this is a supported Tuya device.");
    return;
  }
  for (auto &info : this->datapoints_) {
    if (info.type == TuyaDatapointType::RAW) {
      char hex_buf[format_hex_pretty_size(MAX_DATAPOINT_LOG_BYTES)];
      ESP_LOGCONFIG(TAG, "  Datapoint %u: raw (value: %s)", info.id,
                    format_hex_pretty_to(hex_buf, info.value_raw.data(), info.value_raw.size()));
    } else if (info.type == TuyaDatapointType::BOOLEAN) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: switch (value: %s)", info.id, ONOFF(info.value_bool));
    } else if (info.type == TuyaDatapointType::INTEGER) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: int value (value: %d)", info.id, info.value_int);
    } else if (info.type == TuyaDatapointType::STRING) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: string value (value: %s)", info.id, info.value_string.c_str());
    } else if (info.type == TuyaDatapointType::ENUM) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: enum (value: %d)", info.id, info.value_enum);
    } else if (info.type == TuyaDatapointType::BITMASK) {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: bitmask (value: %" PRIx32 ")", info.id, info.value_bitmask);
    } else {
      ESP_LOGCONFIG(TAG, "  Datapoint %u: unknown", info.id);
    }
  }
  if ((this->status_pin_reported_ != -1) || (this->reset_pin_reported_ != -1)) {
    ESP_LOGCONFIG(TAG, "  GPIO Configuration: status: pin %d, reset: pin %d", this->status_pin_reported_,
                  this->reset_pin_reported_);
  }
  LOG_PIN("  Status Pin: ", this->status_pin_);
  ESP_LOGCONFIG(TAG, "  Heartbeat: every %" PRIu32 " ms, every %" PRIu32 " ms until the working mode reply",
                this->heartbeat_interval_, this->init_heartbeat_interval_);
#ifdef USE_TUYA_MCU_RECOVERY
  if (this->recovery_missed_heartbeats_ != 0) {
    ESP_LOGCONFIG(TAG, "  MCU Recovery: %s after %u unanswered heartbeats",
                  this->recovery_reboot_() ? "reboot" : "TX break", this->recovery_missed_heartbeats_);
  }
#endif
#ifdef USE_TUYA_MCU_RECOVERY_BREAK
  if (this->tx_pin_ != nullptr) {
    ESP_LOGCONFIG(TAG,
                  "  Break Time: %" PRIu32 " ms\n"
                  "  MCU Recoveries: %" PRIu32,
                  this->recovery_break_time_, this->mcu_recoveries_);
  }
#endif
  ESP_LOGCONFIG(TAG, "  Product: '%s'", this->product_.c_str());
  if (this->weather_configured_) {
    if (this->weather_update_interval_ != 0) {
      ESP_LOGCONFIG(TAG, "  Weather service: on the MCU's requests and every %" PRIu32 " s; %s by the MCU, %u keys",
                    this->weather_update_interval_ / 1000, this->weather_enabled_ ? "enabled" : "not yet enabled",
                    (unsigned) this->weather_keys_.size());
    } else {
      ESP_LOGCONFIG(TAG, "  Weather service: on the MCU's requests only; %s by the MCU, %u keys",
                    this->weather_enabled_ ? "enabled" : "not yet enabled", (unsigned) this->weather_keys_.size());
    }
#ifdef USE_SENSOR
    for (auto &src : this->weather_sensors_)
      ESP_LOGCONFIG(TAG, "    %s from sensor '%s'", src.first.c_str(), src.second->get_name().c_str());
#endif
#ifdef USE_TEXT_SENSOR
    for (auto &src : this->weather_text_sensors_)
      ESP_LOGCONFIG(TAG, "    %s from text sensor '%s'", src.first.c_str(), src.second->get_name().c_str());
#endif
  }
}

bool Tuya::validate_message_() {
  uint32_t at = this->rx_message_.size() - 1;
  auto *data = &this->rx_message_[0];
  uint8_t new_byte = data[at];

  // Byte 0: HEADER1 (always 0x55)
  if (at == 0)
    return new_byte == 0x55;
  // Byte 1: HEADER2 (always 0xAA)
  if (at == 1)
    return new_byte == 0xAA;

  // Byte 2: VERSION
  // no validation for the following fields:
  uint8_t version = data[2];
  if (at == 2)
    return true;
  // Byte 3: COMMAND
  uint8_t command = data[3];
  if (at == 3)
    return true;

  // Byte 4: LENGTH1
  // Byte 5: LENGTH2
  if (at <= 5) {
    // no validation for these fields
    return true;
  }

  uint16_t length = (uint16_t(data[4]) << 8) | (uint16_t(data[5]));

  // wait until all data is read
  if (at - 6 < length)
    return true;

  // Byte 6+LEN: CHECKSUM - sum of all bytes (including header) modulo 256
  uint8_t rx_checksum = new_byte;
  uint8_t calc_checksum = 0;
  for (uint32_t i = 0; i < 6 + length; i++)
    calc_checksum += data[i];

  if (rx_checksum != calc_checksum) {
    ESP_LOGW(TAG, "Tuya Received invalid message checksum %02X!=%02X", rx_checksum, calc_checksum);
    return false;
  }

  // valid message
  const uint8_t *message_data = data + 6;
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  char hex_buf[format_hex_pretty_size(MAX_DATAPOINT_LOG_BYTES)];
  ESP_LOGV(TAG, "Received Tuya: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u", command, version,
           format_hex_pretty_to(hex_buf, message_data, length), static_cast<uint8_t>(this->init_state_));
#endif
  this->handle_command_(command, version, message_data, length);

  // return false to reset rx buffer
  return false;
}

void Tuya::handle_char_(uint8_t c) {
  this->rx_message_.push_back(c);
  if (!this->validate_message_()) {
    this->rx_message_.clear();
  } else {
    this->last_rx_char_timestamp_ = millis();
  }
}

void Tuya::handle_command_(uint8_t command, uint8_t version, const uint8_t *buffer, size_t len) {
  TuyaCommandType command_type = (TuyaCommandType) command;

  if (this->expected_response_.has_value() && this->expected_response_ == command_type) {
    this->expected_response_.reset();
    this->command_queue_.erase(command_queue_.begin());
    this->init_retries_ = 0;
#ifdef USE_TUYA_MCU_RECOVERY
    if (command_type == TuyaCommandType::HEARTBEAT) {
      this->heartbeat_misses_ = 0;
#ifdef USE_TUYA_MCU_RECOVERY_BREAK
      if (this->tx_break_unconfirmed_) {
        this->tx_break_unconfirmed_ = false;
        ESP_LOGI(TAG, "MCU answers heartbeats again after the TX break");
      }
#endif
    }
#endif
  }

  switch (command_type) {
    case TuyaCommandType::HEARTBEAT:
      ESP_LOGV(TAG, "MCU Heartbeat (0x%02X)", buffer[0]);
      this->protocol_version_ = version;
      if (buffer[0] == 0) {
        ESP_LOGI(TAG, "MCU restarted");
        this->init_state_ = TuyaInitState::INIT_HEARTBEAT;
        if (!this->init_heartbeat_) {
          this->init_heartbeat_ = true;
          this->set_heartbeat_interval_(this->init_heartbeat_interval_);
        }
        // the MCU enables the weather service again after a restart
        this->weather_enabled_ = false;
        this->weather_keys_.clear();
        this->weather_forecast_days_ = 0;
      }
      if (this->init_state_ == TuyaInitState::INIT_HEARTBEAT) {
        this->init_state_ = TuyaInitState::INIT_PRODUCT;
        this->send_empty_command_(TuyaCommandType::PRODUCT_QUERY);
      }
      break;
    case TuyaCommandType::PRODUCT_QUERY: {
      // check it is a valid string made up of printable characters
      bool valid = true;
      for (size_t i = 0; i < len; i++) {
        if (!std::isprint(buffer[i])) {
          valid = false;
          break;
        }
      }
      if (valid) {
        this->product_ = std::string(reinterpret_cast<const char *>(buffer), len);
      } else {
        this->product_ = R"({"p":"INVALID"})";
      }
      if (this->init_state_ == TuyaInitState::INIT_PRODUCT) {
        this->init_state_ = TuyaInitState::INIT_CONF;
        this->send_empty_command_(TuyaCommandType::CONF_QUERY);
      }
      break;
    }
    case TuyaCommandType::CONF_QUERY: {
      if (len >= 2) {
        this->status_pin_reported_ = buffer[0];
        this->reset_pin_reported_ = buffer[1];
      }
      // Tuya's module firmware goes to the normal heartbeat interval with this reply
      if (this->init_heartbeat_) {
        this->init_heartbeat_ = false;
        this->set_heartbeat_interval_(this->heartbeat_interval_);
      }
      if (this->init_state_ == TuyaInitState::INIT_CONF) {
        // If mcu returned status gpio, then we can omit sending wifi state
        if (this->status_pin_reported_ != -1) {
          this->init_state_ = TuyaInitState::INIT_DATAPOINT;
          this->send_empty_command_(TuyaCommandType::DATAPOINT_QUERY);
          if (this->status_pin_ != nullptr) {
            if (this->status_pin_->get_pin() != this->status_pin_reported_) {
              ESP_LOGW(TAG, "Supplied status_pin does not equal the reported pin %i. Using supplied pin anyway.",
                       this->status_pin_reported_);
            }
            ESP_LOGV(TAG, "Configured status pin %i", this->status_pin_->get_pin());
            this->set_interval("wifi", 1000, [this] { this->set_status_pin_(); });
          } else {
            ESP_LOGW(TAG, "MCU reported status_pin %i but no status_pin was configured; running in limited mode.",
                     this->status_pin_reported_);
          }
        } else {
          this->init_state_ = TuyaInitState::INIT_WIFI;
          ESP_LOGV(TAG, "Configured WIFI_STATE periodic send");
          this->set_interval("wifi", 1000, [this] { this->send_wifi_status_(); });
        }
      }
      break;
    }
    case TuyaCommandType::WIFI_STATE:
      if (this->init_state_ == TuyaInitState::INIT_WIFI) {
        this->init_state_ = TuyaInitState::INIT_DATAPOINT;
        this->send_empty_command_(TuyaCommandType::DATAPOINT_QUERY);
      }
      break;
    case TuyaCommandType::WIFI_SELECT:
    case TuyaCommandType::WIFI_RESET: {
      const bool is_select = (len >= 1);
      // Send WIFI_SELECT ACK
      TuyaCommand ack;
      ack.cmd = is_select ? TuyaCommandType::WIFI_SELECT : TuyaCommandType::WIFI_RESET;
      ack.payload.clear();
      this->send_command_(ack);
      // Establish pairing mode for correct first WIFI_STATE byte, EZ (0x00) default
      uint8_t first = 0x00;
      const char *mode_str = "EZ";
      if (is_select && buffer[0] == 0x01) {
        first = 0x01;
        mode_str = "AP";
      }
      // Send WIFI_STATE response, MCU exits pairing mode
      TuyaCommand st;
      st.cmd = TuyaCommandType::WIFI_STATE;
      st.payload.resize(1);
      st.payload[0] = first;
      this->send_command_(st);
      st.payload[0] = 0x02;
      this->send_command_(st);
      st.payload[0] = 0x03;
      this->send_command_(st);
      st.payload[0] = 0x04;
      this->send_command_(st);
      ESP_LOGI(TAG, "%s received (%s), replied with WIFI_STATE confirming connection established",
               is_select ? LOG_STR_LITERAL("WIFI_SELECT") : LOG_STR_LITERAL("WIFI_RESET"), mode_str);
      break;
    }
    case TuyaCommandType::DATAPOINT_DELIVER:
      break;
    case TuyaCommandType::DATAPOINT_REPORT_ASYNC:
    case TuyaCommandType::DATAPOINT_REPORT_SYNC:
      if (this->init_state_ == TuyaInitState::INIT_DATAPOINT) {
        this->init_state_ = TuyaInitState::INIT_DONE;
        this->set_timeout("datapoint_dump", 1000, [this] { this->dump_config(); });
        this->initialized_callback_.call();
      }
      this->handle_datapoints_(buffer, len);

      if (command_type == TuyaCommandType::DATAPOINT_REPORT_SYNC) {
        this->send_command_(
            TuyaCommand{.cmd = TuyaCommandType::DATAPOINT_REPORT_ACK, .payload = std::vector<uint8_t>{0x01}});
      }
      break;
    case TuyaCommandType::DATAPOINT_QUERY:
      break;
    case TuyaCommandType::WIFI_TEST:
      this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_TEST, .payload = std::vector<uint8_t>{0x00, 0x00}});
      break;
    case TuyaCommandType::WIFI_RSSI:
      this->send_command_(
          TuyaCommand{.cmd = TuyaCommandType::WIFI_RSSI, .payload = std::vector<uint8_t>{get_wifi_rssi_()}});
      break;
    case TuyaCommandType::LOCAL_TIME_QUERY:
#ifdef USE_TIME
      if (this->time_id_ != nullptr) {
        this->send_local_time_();

        if (!this->time_sync_callback_registered_) {
          // tuya mcu supports time, so we let them know when our time changed
          this->time_id_->add_on_time_sync_callback([this] { this->send_local_time_(); });
          this->time_sync_callback_registered_ = true;
        }
      } else
#endif
      {
        ESP_LOGW(TAG, "LOCAL_TIME_QUERY is not handled because time is not configured");
      }
      break;
    case TuyaCommandType::GMT_TIME_QUERY:
#ifdef USE_TIME
      if (this->time_id_ != nullptr) {
        this->send_gmt_time_();

        if (!this->gmt_time_sync_callback_registered_) {
          // tuya mcu supports time, so we let them know when our time changed
          this->time_id_->add_on_time_sync_callback([this] { this->send_gmt_time_(); });
          this->gmt_time_sync_callback_registered_ = true;
        }
      } else
#endif
      {
        ESP_LOGW(TAG, "GMT_TIME_QUERY is not handled because time is not configured");
      }
      break;
    case TuyaCommandType::VACUUM_MAP_UPLOAD:
      this->send_command_(
          TuyaCommand{.cmd = TuyaCommandType::VACUUM_MAP_UPLOAD, .payload = std::vector<uint8_t>{0x01}});
      ESP_LOGW(TAG, "Vacuum map upload requested, responding that it is not enabled.");
      break;
    case TuyaCommandType::GET_NETWORK_STATUS: {
      uint8_t wifi_status = this->get_wifi_status_code_();

      this->send_command_(
          TuyaCommand{.cmd = TuyaCommandType::GET_NETWORK_STATUS, .payload = std::vector<uint8_t>{wifi_status}});
      ESP_LOGV(TAG, "Network status requested, reported as %i", wifi_status);
      break;
    }
    case TuyaCommandType::WEATHER_ENABLE:
      this->handle_weather_enable_(buffer, len);
      break;
    case TuyaCommandType::WEATHER_DATA:
      ESP_LOGV(TAG, "Weather data acknowledged by the MCU");
      break;
    case TuyaCommandType::EXTENDED_SERVICES: {
      uint8_t subcommand = buffer[0];
      switch ((TuyaExtendedServicesCommandType) subcommand) {
        case TuyaExtendedServicesCommandType::REQUEST_WEATHER: {
          if (!this->weather_configured_) {
            ESP_LOGD(TAG, "MCU asks for weather data, weather service not configured");
            break;
          }
          this->send_command_(TuyaCommand{
              .cmd = TuyaCommandType::EXTENDED_SERVICES,
              .payload =
                  std::vector<uint8_t>{static_cast<uint8_t>(TuyaExtendedServicesCommandType::REQUEST_WEATHER), 0x00}});
          uint32_t now = millis();
          // a 0x20 a moment ago already answered this request (the MCU sends both back to back)
          bool answered =
              this->last_weather_request_ != 0 && now - this->last_weather_request_ < WEATHER_REQUEST_DEBOUNCE;
          if (!answered && this->last_weather_demand_ != 0 &&
              now - this->last_weather_demand_ < WEATHER_DEMAND_MIN_INTERVAL) {
            ESP_LOGD(TAG, "MCU asks for weather data, last request %" PRIu32 " s ago, not repeated",
                     (now - this->last_weather_demand_) / 1000);
            break;
          }
          this->last_weather_demand_ = now;  // every honoured request counts against the 20 min limit
          if (answered)
            break;
          this->weather_enabled_ = true;
          this->request_weather_(now);
          break;
        }
        case TuyaExtendedServicesCommandType::RESET_NOTIFICATION: {
          this->send_command_(
              TuyaCommand{.cmd = TuyaCommandType::EXTENDED_SERVICES,
                          .payload = std::vector<uint8_t>{
                              static_cast<uint8_t>(TuyaExtendedServicesCommandType::RESET_NOTIFICATION), 0x00}});
          ESP_LOGV(TAG, "Reset status notification enabled");
          break;
        }
        case TuyaExtendedServicesCommandType::MODULE_RESET: {
          ESP_LOGE(TAG, "EXTENDED_SERVICES::MODULE_RESET is not handled");
          break;
        }
        case TuyaExtendedServicesCommandType::UPDATE_IN_PROGRESS: {
          ESP_LOGE(TAG, "EXTENDED_SERVICES::UPDATE_IN_PROGRESS is not handled");
          break;
        }
        default:
          ESP_LOGE(TAG, "Invalid extended services subcommand (0x%02X) received", subcommand);
      }
      break;
    }
    default:
      ESP_LOGE(TAG, "Invalid command (0x%02X) received", command);
  }
}

void Tuya::handle_datapoints_(const uint8_t *buffer, size_t len) {
  while (len >= 4) {
    TuyaDatapoint datapoint{};
    datapoint.id = buffer[0];
    datapoint.type = (TuyaDatapointType) buffer[1];
    datapoint.value_uint = 0;

    size_t data_size = (buffer[2] << 8) + buffer[3];
    const uint8_t *data = buffer + 4;
    size_t data_len = len - 4;
    if (data_size > data_len) {
      ESP_LOGW(TAG, "Datapoint %u is truncated and cannot be parsed (%zu > %zu)", datapoint.id, data_size, data_len);
      return;
    }

    datapoint.len = data_size;

    switch (datapoint.type) {
      case TuyaDatapointType::RAW:
        datapoint.value_raw = std::vector<uint8_t>(data, data + data_size);
        {
          char hex_buf[format_hex_pretty_size(MAX_DATAPOINT_LOG_BYTES)];
          ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id,
                   format_hex_pretty_to(hex_buf, datapoint.value_raw.data(), datapoint.value_raw.size()));
        }
        break;
      case TuyaDatapointType::BOOLEAN:
        if (data_size != 1) {
          ESP_LOGW(TAG, "Datapoint %u has bad boolean len %zu", datapoint.id, data_size);
          return;
        }
        datapoint.value_bool = data[0];
        ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id, ONOFF(datapoint.value_bool));
        break;
      case TuyaDatapointType::INTEGER:
        if (data_size != 4) {
          ESP_LOGW(TAG, "Datapoint %u has bad integer len %zu", datapoint.id, data_size);
          return;
        }
        datapoint.value_uint = encode_uint32(data[0], data[1], data[2], data[3]);
        ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id, datapoint.value_int);
        break;
      case TuyaDatapointType::STRING:
        datapoint.value_string = std::string(reinterpret_cast<const char *>(data), data_size);
        ESP_LOGD(TAG, "Datapoint %u update to %s", datapoint.id, datapoint.value_string.c_str());
        break;
      case TuyaDatapointType::ENUM:
        if (data_size != 1) {
          ESP_LOGW(TAG, "Datapoint %u has bad enum len %zu", datapoint.id, data_size);
          return;
        }
        datapoint.value_enum = data[0];
        ESP_LOGD(TAG, "Datapoint %u update to %d", datapoint.id, datapoint.value_enum);
        break;
      case TuyaDatapointType::BITMASK:
        switch (data_size) {
          case 1:
            datapoint.value_bitmask = encode_uint32(0, 0, 0, data[0]);
            break;
          case 2:
            datapoint.value_bitmask = encode_uint32(0, 0, data[0], data[1]);
            break;
          case 4:
            datapoint.value_bitmask = encode_uint32(data[0], data[1], data[2], data[3]);
            break;
          default:
            ESP_LOGW(TAG, "Datapoint %u has bad bitmask len %zu", datapoint.id, data_size);
            return;
        }
        ESP_LOGD(TAG, "Datapoint %u update to %#08" PRIX32, datapoint.id, datapoint.value_bitmask);
        break;
      default:
        ESP_LOGW(TAG, "Datapoint %u has unknown type %#02hhX", datapoint.id, static_cast<uint8_t>(datapoint.type));
        return;
    }

    len -= data_size + 4;
    buffer = data + data_size;

    // drop update if datapoint is in ignore_mcu_datapoint_update list
    bool skip = false;
    for (auto i : this->ignore_mcu_update_on_datapoints_) {
      if (datapoint.id == i) {
        ESP_LOGV(TAG, "Datapoint %u found in ignore_mcu_update_on_datapoints list, dropping MCU update", datapoint.id);
        skip = true;
        break;
      }
    }
    if (skip)
      continue;

    // Update internal datapoints
    bool found = false;
    for (auto &other : this->datapoints_) {
      if (other.id == datapoint.id) {
        other = datapoint;
        found = true;
      }
    }
    if (!found) {
      this->datapoints_.push_back(datapoint);
    }

    // Run through listeners
    for (auto &listener : this->listeners_) {
      if (listener.datapoint_id == datapoint.id)
        listener.on_datapoint(datapoint);
    }
  }
}

void Tuya::send_raw_command_(TuyaCommand command) {
  uint8_t len_hi = (uint8_t) (command.payload.size() >> 8);
  uint8_t len_lo = (uint8_t) (command.payload.size() & 0xFF);
  uint8_t version = 0;

  this->last_command_timestamp_ = millis();
  switch (command.cmd) {
    case TuyaCommandType::HEARTBEAT:
      this->expected_response_ = TuyaCommandType::HEARTBEAT;
      break;
    case TuyaCommandType::PRODUCT_QUERY:
      this->expected_response_ = TuyaCommandType::PRODUCT_QUERY;
      break;
    case TuyaCommandType::CONF_QUERY:
      this->expected_response_ = TuyaCommandType::CONF_QUERY;
      break;
    case TuyaCommandType::DATAPOINT_DELIVER:
    case TuyaCommandType::DATAPOINT_QUERY:
      this->expected_response_ = TuyaCommandType::DATAPOINT_REPORT_ASYNC;
      break;
    case TuyaCommandType::WEATHER_DATA:
      this->expected_response_ = TuyaCommandType::WEATHER_DATA;
      break;
    default:
      break;
  }

#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  char hex_buf[format_hex_pretty_size(MAX_DATAPOINT_LOG_BYTES)];
  ESP_LOGV(TAG, "Sending Tuya: CMD=0x%02X VERSION=%u DATA=[%s] INIT_STATE=%u", static_cast<uint8_t>(command.cmd),
           version, format_hex_pretty_to(hex_buf, command.payload.data(), command.payload.size()),
           static_cast<uint8_t>(this->init_state_));
#endif

  this->write_array({0x55, 0xAA, version, (uint8_t) command.cmd, len_hi, len_lo});
  if (!command.payload.empty())
    this->write_array(command.payload.data(), command.payload.size());

  uint8_t checksum = 0x55 + 0xAA + (uint8_t) command.cmd + len_hi + len_lo;
  for (auto &data : command.payload)
    checksum += data;
  this->write_byte(checksum);
}

void Tuya::process_command_queue_() {
  uint32_t now = millis();
  uint32_t delay = now - this->last_command_timestamp_;

  if (now - this->last_rx_char_timestamp_ > RECEIVE_TIMEOUT) {
    this->rx_message_.clear();
  }

  if (this->expected_response_.has_value() && delay > RECEIVE_TIMEOUT) {
#ifdef USE_TUYA_MCU_RECOVERY
    const bool heartbeat = this->expected_response_ == TuyaCommandType::HEARTBEAT;
#endif
    this->expected_response_.reset();
    if (init_state_ != TuyaInitState::INIT_DONE) {
      if (++this->init_retries_ >= MAX_RETRIES) {
        this->init_failed_ = true;
        ESP_LOGE(TAG, "Initialization failed at init_state %u", static_cast<uint8_t>(this->init_state_));
        this->command_queue_.erase(command_queue_.begin());
        this->init_retries_ = 0;
      }
    } else {
      this->command_queue_.erase(command_queue_.begin());
#ifdef USE_TUYA_MCU_RECOVERY
      if (heartbeat)
        this->note_missed_heartbeat_();
#endif
    }
  }

#ifdef USE_TUYA_MCU_RECOVERY_BREAK
  if (this->tx_break_active_)
    return;
#endif

  // Left check of delay since last command in case there's ever a command sent by calling send_raw_command_ directly
  if (delay > COMMAND_DELAY && !this->command_queue_.empty() && this->rx_message_.empty() &&
      !this->expected_response_.has_value()) {
    this->send_raw_command_(command_queue_.front());
    if (!this->expected_response_.has_value())
      this->command_queue_.erase(command_queue_.begin());
  }
}

#ifdef USE_TUYA_MCU_RECOVERY
bool Tuya::recovery_reboot_() const {
#ifdef USE_TUYA_MCU_RECOVERY_BREAK
  return this->tx_pin_ == nullptr;
#else
  return true;
#endif
}

void Tuya::note_missed_heartbeat_() {
  if (this->recovery_missed_heartbeats_ == 0 || ++this->heartbeat_misses_ < this->recovery_missed_heartbeats_)
    return;
#ifdef USE_TUYA_MCU_RECOVERY_BREAK
  if (!this->recovery_reboot_()) {
    if (this->mcu_recoveries_ != 0 && millis() - this->tx_break_start_ < RECOVERY_MIN_INTERVAL)
      return;
    ESP_LOGW(TAG, "MCU left %u heartbeats unanswered, holding TX low for %" PRIu32 " ms", this->heartbeat_misses_,
             this->recovery_break_time_);
    this->heartbeat_misses_ = 0;
    this->start_tx_break_();
    return;
  }
#endif
  ESP_LOGE(TAG, "MCU left %u heartbeats unanswered, rebooting", this->heartbeat_misses_);
  App.safe_reboot();
}

#ifdef USE_TUYA_MCU_RECOVERY_BREAK
void Tuya::start_tx_break_() {
  this->flush();
#ifdef USE_LIBRETINY
  static_cast<uart::LibreTinyUARTComponent *>(this->parent_)->get_hw_serial()->end();
#endif
  this->tx_pin_->setup();
  this->tx_pin_->digital_write(false);
  this->tx_break_start_ = millis();
  this->tx_break_active_ = true;
  this->tx_break_unconfirmed_ = true;
  this->mcu_recoveries_++;
}

void Tuya::end_tx_break_() {
  this->tx_break_active_ = false;
  this->tx_pin_->digital_write(true);
#ifdef USE_LIBRETINY
  auto *bus = static_cast<uart::LibreTinyUARTComponent *>(this->parent_);
  bus->get_hw_serial()->begin(bus->get_baud_rate(), bus->get_config());
#elif defined(USE_ESP8266) || defined(USE_ESP32)
  this->parent_->load_settings(false);
#endif
  this->rx_message_.clear();
  this->send_empty_command_(TuyaCommandType::DATAPOINT_QUERY);
}
#endif  // USE_TUYA_MCU_RECOVERY_BREAK
#endif  // USE_TUYA_MCU_RECOVERY

void Tuya::send_command_(const TuyaCommand &command) {
  command_queue_.push_back(command);
  process_command_queue_();
}

void Tuya::send_empty_command_(TuyaCommandType command) {
  send_command_(TuyaCommand{.cmd = command, .payload = std::vector<uint8_t>{}});
}

void Tuya::set_status_pin_() {
  bool is_network_ready = network_is_connected() && remote_is_connected();
  this->status_pin_->digital_write(is_network_ready);
}

uint8_t Tuya::get_wifi_status_code_() {
  uint8_t status = 0x02;

  if (network_is_connected()) {
    status = 0x03;

    // Protocol version 3 also supports specifying when connected to "the cloud"
    if (this->protocol_version_ >= 0x03 && remote_is_connected()) {
      status = 0x04;
    }
  } else {
#ifdef USE_CAPTIVE_PORTAL
    if (captive_portal::global_captive_portal != nullptr && captive_portal::global_captive_portal->is_active()) {
      status = 0x01;
    }
#endif
  };

  return status;
}

uint8_t Tuya::get_wifi_rssi_() {
#ifdef USE_WIFI
  if (wifi::global_wifi_component != nullptr)
    return wifi::global_wifi_component->wifi_rssi();
#endif

  return 0;
}

void Tuya::send_wifi_status_() {
  uint8_t status = this->get_wifi_status_code_();

  if (status == this->wifi_status_) {
    return;
  }

  ESP_LOGD(TAG, "Sending WiFi Status");
  this->wifi_status_ = status;
  this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WIFI_STATE, .payload = std::vector<uint8_t>{status}});
}

#ifdef USE_TIME
void Tuya::send_local_time_() {
  std::vector<uint8_t> payload;
  ESPTime now = this->time_id_->now();
  if (now.is_valid()) {
    uint8_t year = now.year - 2000;
    uint8_t month = now.month;
    uint8_t day_of_month = now.day_of_month;
    uint8_t hour = now.hour;
    uint8_t minute = now.minute;
    uint8_t second = now.second;
    // Tuya days starts from Monday, esphome uses Sunday as day 1
    uint8_t day_of_week = now.day_of_week - 1;
    if (day_of_week == 0) {
      day_of_week = 7;
    }
    ESP_LOGD(TAG, "Sending local time");
    payload = std::vector<uint8_t>{0x01, year, month, day_of_month, hour, minute, second, day_of_week};
  } else {
    // By spec we need to notify MCU that the time was not obtained if this is a response to a query
    ESP_LOGW(TAG, "Sending missing local time");
    payload = std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  }
  this->send_command_(TuyaCommand{.cmd = TuyaCommandType::LOCAL_TIME_QUERY, .payload = payload});
}
void Tuya::send_gmt_time_() {
  std::vector<uint8_t> payload;
  ESPTime now = this->time_id_->utcnow();
  if (now.is_valid()) {
    uint8_t year = now.year - 2000;
    uint8_t month = now.month;
    uint8_t day_of_month = now.day_of_month;
    uint8_t hour = now.hour;
    uint8_t minute = now.minute;
    uint8_t second = now.second;
    ESP_LOGD(TAG, "Sending gmt time");
    payload = std::vector<uint8_t>{0x01, year, month, day_of_month, hour, minute, second};
  } else {
    // By spec we need to notify MCU that the time was not obtained if this is a response to a query
    ESP_LOGW(TAG, "Sending missing gmt time");
    payload = std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  }
  this->send_command_(TuyaCommand{.cmd = TuyaCommandType::GMT_TIME_QUERY, .payload = payload});
}
#endif

void Tuya::set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value) {
  this->set_raw_datapoint_value_(datapoint_id, value, false);
}

void Tuya::set_boolean_datapoint_value(uint8_t datapoint_id, bool value) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::BOOLEAN, value, 1, false);
}

void Tuya::set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::INTEGER, value, 4, false);
}

void Tuya::set_string_datapoint_value(uint8_t datapoint_id, const std::string &value) {
  this->set_string_datapoint_value_(datapoint_id, value, false);
}

void Tuya::set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::ENUM, value, 1, false);
}

void Tuya::set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::BITMASK, value, length, false);
}

void Tuya::force_set_raw_datapoint_value(uint8_t datapoint_id, const std::vector<uint8_t> &value) {
  this->set_raw_datapoint_value_(datapoint_id, value, true);
}

void Tuya::force_set_boolean_datapoint_value(uint8_t datapoint_id, bool value) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::BOOLEAN, value, 1, true);
}

void Tuya::force_set_integer_datapoint_value(uint8_t datapoint_id, uint32_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::INTEGER, value, 4, true);
}

void Tuya::force_set_string_datapoint_value(uint8_t datapoint_id, const std::string &value) {
  this->set_string_datapoint_value_(datapoint_id, value, true);
}

void Tuya::force_set_enum_datapoint_value(uint8_t datapoint_id, uint8_t value) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::ENUM, value, 1, true);
}

void Tuya::force_set_bitmask_datapoint_value(uint8_t datapoint_id, uint32_t value, uint8_t length) {
  this->set_numeric_datapoint_value_(datapoint_id, TuyaDatapointType::BITMASK, value, length, true);
}

optional<TuyaDatapoint> Tuya::get_datapoint_(uint8_t datapoint_id) {
  for (auto &datapoint : this->datapoints_) {
    if (datapoint.id == datapoint_id)
      return datapoint;
  }
  return {};
}

void Tuya::set_numeric_datapoint_value_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, const uint32_t value,
                                        uint8_t length, bool forced) {
  ESP_LOGD(TAG, "Setting datapoint %u to %" PRIu32, datapoint_id, value);
  optional<TuyaDatapoint> datapoint = this->get_datapoint_(datapoint_id);
  if (!datapoint.has_value()) {
    ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
  } else if (datapoint->type != datapoint_type) {
    ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type", datapoint_id);
    return;
  } else if (!forced && datapoint->value_uint == value) {
    ESP_LOGV(TAG, "Not sending unchanged value");
    return;
  }

  std::vector<uint8_t> data;
  switch (length) {
    case 4:
      data.push_back(value >> 24);
      data.push_back(value >> 16);
      [[fallthrough]];
    case 2:
      data.push_back(value >> 8);
      [[fallthrough]];
    case 1:
      data.push_back(value >> 0);
      break;
    default:
      ESP_LOGE(TAG, "Unexpected datapoint length %u", length);
      return;
  }
  this->send_datapoint_command_(datapoint_id, datapoint_type, data);
}

void Tuya::set_raw_datapoint_value_(uint8_t datapoint_id, const std::vector<uint8_t> &value, bool forced) {
  char hex_buf[format_hex_pretty_size(MAX_DATAPOINT_LOG_BYTES)];
  ESP_LOGD(TAG, "Setting datapoint %u to %s", datapoint_id, format_hex_pretty_to(hex_buf, value.data(), value.size()));
  optional<TuyaDatapoint> datapoint = this->get_datapoint_(datapoint_id);
  if (!datapoint.has_value()) {
    ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
  } else if (datapoint->type != TuyaDatapointType::RAW) {
    ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type", datapoint_id);
    return;
  } else if (!forced && datapoint->value_raw == value) {
    ESP_LOGV(TAG, "Not sending unchanged value");
    return;
  }
  this->send_datapoint_command_(datapoint_id, TuyaDatapointType::RAW, value);
}

void Tuya::set_string_datapoint_value_(uint8_t datapoint_id, const std::string &value, bool forced) {
  ESP_LOGD(TAG, "Setting datapoint %u to %s", datapoint_id, value.c_str());
  optional<TuyaDatapoint> datapoint = this->get_datapoint_(datapoint_id);
  if (!datapoint.has_value()) {
    ESP_LOGW(TAG, "Setting unknown datapoint %u", datapoint_id);
  } else if (datapoint->type != TuyaDatapointType::STRING) {
    ESP_LOGE(TAG, "Attempt to set datapoint %u with incorrect type", datapoint_id);
    return;
  } else if (!forced && datapoint->value_string == value) {
    ESP_LOGV(TAG, "Not sending unchanged value");
    return;
  }
  std::vector<uint8_t> data;
  for (char const &c : value) {
    data.push_back(c);
  }
  this->send_datapoint_command_(datapoint_id, TuyaDatapointType::STRING, data);
}

void Tuya::send_datapoint_command_(uint8_t datapoint_id, TuyaDatapointType datapoint_type, std::vector<uint8_t> data) {
  std::vector<uint8_t> buffer;
  buffer.push_back(datapoint_id);
  buffer.push_back(static_cast<uint8_t>(datapoint_type));
  buffer.push_back(data.size() >> 8);
  buffer.push_back(data.size() >> 0);
  buffer.insert(buffer.end(), data.begin(), data.end());

  this->send_command_(TuyaCommand{.cmd = TuyaCommandType::DATAPOINT_DELIVER, .payload = buffer});
}

// Weather service (Tuya MCU SDK "Weather service"): 0x20 from the MCU carries the keys it wants as a list of
// (length, key); the module answers 0x20 [01 00] and then sends 0x21 [01, (len key type len value)...] at enable
// and every 30 min, the MCU acks with an empty 0x21. 0x34/0x03 asks for data now, answered with 0x34 [03 00].
// An empty key list is taken as "everything configured". Without a weather: block the requests are only logged.
static const size_t WEATHER_MAX_PAYLOAD = 255;

static bool weather_has_key(const std::vector<TuyaWeatherValue> &values, const std::string &key) {
  for (const auto &v : values) {
    if (v.key == key)
      return true;
  }
  return false;
}

void Tuya::handle_weather_enable_(const uint8_t *buffer, size_t len) {
  if (!this->weather_configured_) {
    ESP_LOGD(TAG, "MCU asks for the weather service, not configured");
    return;
  }
  std::vector<std::string> keys;
  size_t i = 0;
  while (i < len) {
    size_t key_len = buffer[i];
    if (key_len == 0 || i + 1 + key_len > len) {
      ESP_LOGW(TAG, "Weather enable request malformed at byte %u", (unsigned) i);
      this->send_command_(
          TuyaCommand{.cmd = TuyaCommandType::WEATHER_ENABLE, .payload = std::vector<uint8_t>{0x00, 0x01}});
      return;
    }
    keys.emplace_back(reinterpret_cast<const char *>(buffer + i + 1), key_len);
    i += 1 + key_len;
  }
  // w.date.N asks for the forecast too: every value key then also comes as w.<key>.<day>, day 0 = today .. N-1
  this->weather_forecast_days_ = 0;
  for (const auto &key : keys) {
    if (key.size() == 8 && key.compare(0, 7, "w.date.") == 0 && key[7] >= '1' && key[7] <= '7')
      this->weather_forecast_days_ = key[7] - '0';
  }
  this->weather_keys_ = std::move(keys);
  this->weather_enabled_ = true;
  this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WEATHER_ENABLE, .payload = std::vector<uint8_t>{0x01, 0x00}});
  ESP_LOGI(TAG, "Weather service enabled by the MCU, %u keys", (unsigned) this->weather_keys_.size());
  this->request_weather_(millis());
}

bool Tuya::weather_frame_queued_() const {
  for (const auto &cmd : this->command_queue_) {
    if (cmd.cmd == TuyaCommandType::WEATHER_DATA)
      return true;
  }
  return false;
}

// An empty key list means "everything configured". A forecast key w.<key>.<day> counts as requested when the
// MCU asked for w.<key> and for at least day + 1 forecast days (w.date.N).
bool Tuya::weather_key_requested_(const std::string &key) const {
  if (this->weather_keys_.empty())
    return true;
  if (std::find(this->weather_keys_.begin(), this->weather_keys_.end(), key) != this->weather_keys_.end())
    return true;
  size_t n = key.size();
  if (n < 3 || key[n - 2] != '.' || key[n - 1] < '0' || key[n - 1] > '6')
    return false;
  if (key[n - 1] - '0' >= this->weather_forecast_days_)
    return false;
  std::string base = key.substr(0, n - 2);
  return std::find(this->weather_keys_.begin(), this->weather_keys_.end(), base) != this->weather_keys_.end();
}

// Answer a request: automations first (their send_weather() calls are collected), then the configured sources
// for the keys they left out, one 0x21 frame in all.
bool Tuya::request_weather_(uint32_t now) {
  this->last_weather_request_ = now;
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  std::string list;
  for (const auto &key : this->weather_keys_) {
    if (!list.empty())
      list += ", ";
    list += key;
  }
  ESP_LOGD(TAG, "Weather data requested: %s", list.c_str());
#endif
  this->pending_weather_.clear();
  this->collecting_weather_ = true;
  this->weather_request_callback_.call(this->weather_keys_);
  this->collecting_weather_ = false;
  std::vector<TuyaWeatherValue> values = std::move(this->pending_weather_);
  this->pending_weather_.clear();

#ifdef USE_SENSOR
  for (auto &src : this->weather_sensors_) {
    if (weather_has_key(values, src.first))
      continue;  // answered by an automation
    float state = src.second->state;
    if (!std::isfinite(state)) {
      ESP_LOGD(TAG, "Weather %s: '%s' has no value, not sent", src.first.c_str(), src.second->get_name().c_str());
      continue;
    }
    if (src.first.rfind("w.windSpeed", 0) == 0) {  // also the forecast variants w.windSpeed1 / w.windSpeed.1
      char buf[16];
      snprintf(buf, sizeof(buf), "%.1f", state);
      values.push_back(TuyaWeatherValue{src.first, true, 0, buf});
    } else {
      float rounded = roundf(state);
      if (rounded > 2147483647.0f || rounded < -2147483648.0f) {
        ESP_LOGW(TAG, "Weather %s: %.0f is out of the int32 range, not sent", src.first.c_str(), state);
        continue;
      }
      values.push_back(TuyaWeatherValue{src.first, false, static_cast<int32_t>(rounded), {}});
    }
  }
#endif
#ifdef USE_TEXT_SENSOR
  for (auto &src : this->weather_text_sensors_) {
    if (weather_has_key(values, src.first))
      continue;
    if (!src.second->has_state() || src.second->state.empty()) {
      ESP_LOGD(TAG, "Weather %s: '%s' has no value, not sent", src.first.c_str(), src.second->get_name().c_str());
      continue;
    }
    values.push_back(TuyaWeatherValue{src.first, true, 0, src.second->state});
  }
#endif
  if (values.empty()) {
    ESP_LOGD(TAG, "Weather data requested, nothing to send");
    return false;
  }
  return this->send_weather_frame_(std::move(values));
}

bool Tuya::send_weather(const std::vector<TuyaWeatherValue> &values) {
  if (this->collecting_weather_) {
    for (const auto &v : values) {
      if (!weather_has_key(this->pending_weather_, v.key))
        this->pending_weather_.push_back(v);
    }
    return true;
  }
  return this->send_weather_frame_(values);
}

bool Tuya::send_weather_frame_(std::vector<TuyaWeatherValue> values) {
  if (!this->weather_enabled_) {
    ESP_LOGD(TAG, "Weather data not sent, the MCU has not enabled the weather service");
    return false;
  }
  if (this->init_state_ != TuyaInitState::INIT_DONE) {
    ESP_LOGD(TAG, "Weather data not sent, MCU not initialised yet");
    return false;
  }
  std::vector<uint8_t> payload{0x01};
  unsigned sent = 0;
  for (const auto &v : values) {
    if (!this->weather_key_requested_(v.key)) {
      ESP_LOGV(TAG, "Weather key %s not requested by the MCU, skipped", v.key.c_str());
      continue;
    }
    size_t entry = 1 + v.key.size() + 2 + (v.is_string ? v.value_string.size() : 4);
    if (v.key.size() > 255 || v.value_string.size() > 255 || payload.size() + entry > WEATHER_MAX_PAYLOAD) {
      ESP_LOGW(TAG, "Weather %s: entry too long or frame full, not sent", v.key.c_str());
      continue;
    }
    if (v.is_string) {
      ESP_LOGD(TAG, "Weather %s: '%s'", v.key.c_str(), v.value_string.c_str());
    } else {
      ESP_LOGD(TAG, "Weather %s: %" PRId32, v.key.c_str(), v.value_int);
    }
    payload.push_back(static_cast<uint8_t>(v.key.size()));
    payload.insert(payload.end(), v.key.begin(), v.key.end());
    if (v.is_string) {
      payload.push_back(0x01);
      payload.push_back(static_cast<uint8_t>(v.value_string.size()));
      payload.insert(payload.end(), v.value_string.begin(), v.value_string.end());
    } else {
      uint32_t u = static_cast<uint32_t>(v.value_int);
      payload.push_back(0x00);
      payload.push_back(0x04);
      payload.push_back(u >> 24);
      payload.push_back(u >> 16);
      payload.push_back(u >> 8);
      payload.push_back(u);
    }
    sent++;
  }
  if (sent == 0) {
    ESP_LOGD(TAG, "Weather data not sent, no value matches the keys the MCU asked for");
    return false;
  }
  ESP_LOGD(TAG, "Sending weather data: %u values, %u bytes", sent, (unsigned) payload.size());
  this->send_command_(TuyaCommand{.cmd = TuyaCommandType::WEATHER_DATA, .payload = std::move(payload)});
  return true;
}

void Tuya::register_listener(uint8_t datapoint_id, const std::function<void(TuyaDatapoint)> &func) {
  auto listener = TuyaDatapointListener{
      .datapoint_id = datapoint_id,
      .on_datapoint = func,
  };
  this->listeners_.push_back(listener);

  // Run through existing datapoints
  for (auto &datapoint : this->datapoints_) {
    if (datapoint.id == datapoint_id)
      func(datapoint);
  }
}

TuyaInitState Tuya::get_init_state() { return this->init_state_; }

}  // namespace esphome::tuya
