#pragma once

#include <Arduino.h>
#include <osdp.hpp>

#include <cstring>
#include <string>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"

namespace esphome::osdp_bridge {

static const char *const TAG = "osdp_bridge";

class OsdpControllerBridge {
 public:
  OsdpControllerBridge(int uart_bus, int rx_pin, int tx_pin, int direction_pin,
                       int baud_rate, int reader_address,
                       const std::string &scbk_hex = "")
      : uart_bus_(uart_bus),
        rx_pin_(rx_pin),
        tx_pin_(tx_pin),
        direction_pin_(direction_pin),
        baud_rate_(baud_rate),
        reader_address_(reader_address),
        scbk_hex_(scbk_hex),
        serial_(uart_bus),
        channel_state_{&serial_, direction_pin},
        channel_{.data = &channel_state_,
                 .recv = &OsdpControllerBridge::osdp_read_,
                 .send = &OsdpControllerBridge::osdp_write_,
                 .flush = &OsdpControllerBridge::osdp_flush_,
                 .close = nullptr} {
    instance_ = this;
  }

  void begin() {
    if (started_) {
      return;
    }

    pinMode(direction_pin_, OUTPUT);
    digitalWrite(direction_pin_, LOW);
    serial_.begin(baud_rate_, SERIAL_8N1, rx_pin_, tx_pin_);

    scbk_present_ = parse_scbk_();

    std::memset(&pd_info_, 0, sizeof(pd_info_));
    pd_info_.name = "reader-0";
    pd_info_.baud_rate = baud_rate_;
    pd_info_.address = reader_address_;
    pd_info_.flags = OSDP_FLAG_ENABLE_NOTIFICATION;
    pd_info_.cap = nullptr;
    pd_info_.scbk = scbk_present_ ? scbk_ : nullptr;

    if (!control_panel_.setup(&channel_, 1, &pd_info_)) {
      ESP_LOGE(TAG, "Failed to initialise LibOSDP control panel");
      return;
    }

    control_panel_.set_event_callback(&OsdpControllerBridge::osdp_event_, this);
    control_panel_.set_command_completion_callback(
        &OsdpControllerBridge::osdp_command_complete_, this);

    started_ = true;
    last_status_log_ms_ = millis();

    ESP_LOGI(TAG, "LibOSDP bridge started on UART%d addr=%d scbk=%s", uart_bus_,
             reader_address_, scbk_present_ ? "configured" : "none");
  }

  void loop() {
    if (!started_) {
      return;
    }

    control_panel_.refresh();

    const uint32_t now = millis();
    if (now - last_status_log_ms_ >= 5000) {
      uint8_t online_mask[1] = {0};
      uint8_t secure_mask[1] = {0};
      control_panel_.get_status_mask(online_mask);
      control_panel_.get_sc_status_mask(secure_mask);
      ESP_LOGD(TAG, "online_mask=0x%02X sc_mask=0x%02X", online_mask[0],
               secure_mask[0]);
      last_status_log_ms_ = now;
    }
  }

  void set_online_binary_sensor(binary_sensor::BinarySensor *sensor) {
    online_binary_sensor_ = sensor;
  }

  void set_tamper_binary_sensor(binary_sensor::BinarySensor *sensor) {
    tamper_binary_sensor_ = sensor;
  }

  void set_door_binary_sensor(binary_sensor::BinarySensor *sensor) {
    door_binary_sensor_ = sensor;
  }

  void set_secure_channel_binary_sensor(binary_sensor::BinarySensor *sensor) {
    secure_channel_binary_sensor_ = sensor;
  }

  void set_last_card_text_sensor(text_sensor::TextSensor *sensor) {
    last_card_text_sensor_ = sensor;
  }

  void set_last_keypad_text_sensor(text_sensor::TextSensor *sensor) {
    last_keypad_text_sensor_ = sensor;
  }

  void set_last_event_text_sensor(text_sensor::TextSensor *sensor) {
    last_event_text_sensor_ = sensor;
  }

  void request_poll() { request_status_(); }

  void grant_access(const std::string &token) {
    publish_event_("grant_not_implemented:" + std::string(token));
    ESP_LOGW(TAG,
             "grant_access requested, but access-grant semantics are reader-specific");
  }

  void deny_access() { publish_event_("deny_not_implemented"); }

  void set_reader_led_blue() { submit_led_command_(OSDP_LED_COLOR_BLUE); }

  void set_reader_led_green() { submit_led_command_(OSDP_LED_COLOR_GREEN); }

  void set_reader_led_red() { submit_led_command_(OSDP_LED_COLOR_RED); }

  void set_reader_led_off() { submit_led_command_(OSDP_LED_COLOR_NONE); }

  void set_reader_led(bool enabled) {
    if (enabled) {
      set_reader_led_blue();
    } else {
      set_reader_led_off();
    }
  }

  void show_access_granted() {
    struct osdp_cmd led_cmd = {};
    led_cmd.id = OSDP_CMD_LED;
    led_cmd.led.reader = 0;
    led_cmd.led.led_number = 0;
    led_cmd.led.temporary.control_code = 2;
    led_cmd.led.temporary.on_count = 10;
    led_cmd.led.temporary.off_count = 0;
    led_cmd.led.temporary.on_color = OSDP_LED_COLOR_GREEN;
    led_cmd.led.temporary.off_color = OSDP_LED_COLOR_NONE;
    led_cmd.led.temporary.timer_count = 30;
    led_cmd.led.permanent.control_code = 1;
    led_cmd.led.permanent.on_count = 1;
    led_cmd.led.permanent.off_count = 0;
    led_cmd.led.permanent.on_color = OSDP_LED_COLOR_BLUE;
    led_cmd.led.permanent.off_color = OSDP_LED_COLOR_NONE;
    led_cmd.led.permanent.timer_count = 0;
    submit_command_(led_cmd);

    struct osdp_cmd buzzer_cmd = {};
    buzzer_cmd.id = OSDP_CMD_BUZZER;
    buzzer_cmd.buzzer.reader = 0;
    buzzer_cmd.buzzer.control_code = 2;
    buzzer_cmd.buzzer.on_count = 2;
    buzzer_cmd.buzzer.off_count = 0;
    buzzer_cmd.buzzer.rep_count = 1;
    submit_command_(buzzer_cmd);

    publish_event_("access_granted");
  }

  void show_access_denied() {
    struct osdp_cmd led_cmd = {};
    led_cmd.id = OSDP_CMD_LED;
    led_cmd.led.reader = 0;
    led_cmd.led.led_number = 0;
    led_cmd.led.temporary.control_code = 2;
    led_cmd.led.temporary.on_count = 5;
    led_cmd.led.temporary.off_count = 5;
    led_cmd.led.temporary.on_color = OSDP_LED_COLOR_RED;
    led_cmd.led.temporary.off_color = OSDP_LED_COLOR_NONE;
    led_cmd.led.temporary.timer_count = 30;
    led_cmd.led.permanent.control_code = 1;
    led_cmd.led.permanent.on_count = 1;
    led_cmd.led.permanent.off_count = 0;
    led_cmd.led.permanent.on_color = OSDP_LED_COLOR_BLUE;
    led_cmd.led.permanent.off_color = OSDP_LED_COLOR_NONE;
    led_cmd.led.permanent.timer_count = 0;
    submit_command_(led_cmd);

    struct osdp_cmd buzzer_cmd = {};
    buzzer_cmd.id = OSDP_CMD_BUZZER;
    buzzer_cmd.buzzer.reader = 0;
    buzzer_cmd.buzzer.control_code = 2;
    buzzer_cmd.buzzer.on_count = 1;
    buzzer_cmd.buzzer.off_count = 1;
    buzzer_cmd.buzzer.rep_count = 2;
    submit_command_(buzzer_cmd);

    publish_event_("access_denied");
  }

  void restore_idle_led() {
    set_reader_led_blue();
  }

  void set_output(uint8_t output, bool enabled) {
    struct osdp_cmd cmd = {};
    cmd.id = OSDP_CMD_OUTPUT;
    cmd.output.output_no = output;
    cmd.output.control_code = enabled ? 2 : 1;
    cmd.output.timer_count = 0;
    submit_command_(cmd);
  }

  void buzz(uint16_t on_ms, uint16_t off_ms, uint8_t repeats) {
    struct osdp_cmd cmd = {};
    cmd.id = OSDP_CMD_BUZZER;
    cmd.buzzer.reader = 0;
    cmd.buzzer.control_code = repeats == 0 ? 1 : 2;
    cmd.buzzer.on_count = on_ms / 100;
    cmd.buzzer.off_count = off_ms / 100;
    cmd.buzzer.rep_count = repeats;
    submit_command_(cmd);
  }

 protected:
  void submit_led_command_(uint8_t color) {
    struct osdp_cmd cmd = {};
    cmd.id = OSDP_CMD_LED;
    cmd.led.reader = 0;
    cmd.led.led_number = 0;
    cmd.led.temporary.control_code = 0;
    cmd.led.permanent.control_code = 1;
    cmd.led.permanent.on_count = color == OSDP_LED_COLOR_NONE ? 0 : 1;
    cmd.led.permanent.off_count = 0;
    cmd.led.permanent.on_color = color;
    cmd.led.permanent.off_color = OSDP_LED_COLOR_NONE;
    cmd.led.permanent.timer_count = 0;
    submit_command_(cmd);
  }
  struct OsdpSerialChannel {
    HardwareSerial *port;
    int direction_pin;
  };

  static int osdp_read_(void *data, uint8_t *buf, int maxlen) {
    auto *channel = static_cast<OsdpSerialChannel *>(data);
    int count = 0;

    while (count < maxlen && channel->port->available()) {
      buf[count++] = static_cast<uint8_t>(channel->port->read());
    }

    return count;
  }

  static int osdp_write_(void *data, uint8_t *buf, int len) {
    auto *channel = static_cast<OsdpSerialChannel *>(data);
    digitalWrite(channel->direction_pin, HIGH);
    delayMicroseconds(50);
    const int written = channel->port->write(buf, len);
    channel->port->flush();
    delayMicroseconds(50);
    digitalWrite(channel->direction_pin, LOW);
    return written == len ? written : -1;
  }

  static void osdp_flush_(void *data) {
    auto *channel = static_cast<OsdpSerialChannel *>(data);
    channel->port->flush();
    while (channel->port->available()) {
      channel->port->read();
    }
  }

  static int osdp_event_(void *arg, int pd, struct osdp_event *event) {
    return static_cast<OsdpControllerBridge *>(arg)->handle_event_(pd, event);
  }

  static void osdp_command_complete_(void *arg, int pd,
                                     const struct osdp_cmd *cmd,
                                     enum osdp_completion_status status) {
    static_cast<OsdpControllerBridge *>(arg)->handle_command_complete_(pd, cmd,
                                                                       status);
  }

  int handle_event_(int pd, struct osdp_event *event) {
    switch (event->type) {
      case OSDP_EVENT_CARDREAD: {
        const auto bytes = (event->cardread.length + 7) / 8;
        const auto card = bytes_to_hex_(event->cardread.data, bytes);
        ESP_LOGI(TAG, "PD %d card read: %s", pd, card.c_str());
        if (last_card_text_sensor_ != nullptr) {
          last_card_text_sensor_->publish_state(card.c_str());
        }
        publish_event_(card);
        break;
      }
      case OSDP_EVENT_KEYPRESS: {
        std::string keypad;
        keypad.reserve(event->keypress.length);
        for (int i = 0; i < event->keypress.length; ++i) {
          keypad.push_back(static_cast<char>(event->keypress.data[i]));
        }
        ESP_LOGI(TAG, "PD %d keypad: %s", pd, keypad.c_str());
        if (last_keypad_text_sensor_ != nullptr) {
          last_keypad_text_sensor_->publish_state(keypad.c_str());
        }
        publish_event_(keypad);
        break;
      }
      case OSDP_EVENT_STATUS:
        handle_status_event_(pd, event->status);
        break;
      case OSDP_EVENT_NOTIFICATION:
        handle_notification_event_(pd, event->notif);
        break;
      case OSDP_EVENT_MFGREP:
        ESP_LOGD(TAG, "PD %d manufacturer reply len=%d", pd, event->mfgrep.length);
        break;
      default:
        break;
    }

    return 0;
  }

  void handle_command_complete_(int pd, const struct osdp_cmd *cmd,
                                enum osdp_completion_status status) {
    ESP_LOGD(TAG, "PD %d command %d completion %d", pd, cmd->id, status);
    release_command_slot_(cmd);
  }

  void handle_status_event_(int pd, const struct osdp_status_report &status) {
    if (status.type == OSDP_STATUS_REPORT_REMOTE && status.nr_entries > 0) {
      const bool tamper = (status.report[0] & 0x01) != 0;
      const bool door_power = (status.report[0] & 0x02) != 0;
      ESP_LOGD(TAG, "PD %d remote status tamper=%d power=%d", pd, tamper,
               door_power);
      if (tamper_binary_sensor_ != nullptr) {
        tamper_binary_sensor_->publish_state(tamper);
      }
      if (door_binary_sensor_ != nullptr) {
        door_binary_sensor_->publish_state(door_power);
      }
      publish_event_(tamper ? "tamper" : "secure");
    }
  }

  void handle_notification_event_(
      int pd, const struct osdp_event_notification &notification) {
    if (notification.type == OSDP_EVENT_NOTIFICATION_PD_STATUS) {
      const bool online = notification.arg0 != 0;
      ESP_LOGI(TAG, "PD %d online=%d", pd, online);
      if (online_binary_sensor_ != nullptr) {
        online_binary_sensor_->publish_state(online);
      }
      if (online) {
        restore_idle_led();
      }
      publish_event_(online ? "online" : "offline");
    } else if (notification.type == OSDP_EVENT_NOTIFICATION_SC_STATUS) {
      const bool secure = notification.arg0 != 0;
      ESP_LOGI(TAG, "PD %d secure_channel=%d", pd, secure);
      if (secure_channel_binary_sensor_ != nullptr) {
        secure_channel_binary_sensor_->publish_state(secure);
      }
      publish_event_(secure ? "secure_channel_on" : "secure_channel_off");
    }
  }

  void publish_event_(const std::string &value) {
    if (last_event_text_sensor_ != nullptr) {
      last_event_text_sensor_->publish_state(value.c_str());
    }
  }

  void request_status_() {
    struct osdp_cmd cmd = {};
    cmd.id = OSDP_CMD_STATUS;
    cmd.status.type = OSDP_STATUS_REPORT_REMOTE;
    cmd.status.nr_entries = 0;
    submit_command_(cmd);
  }

  bool submit_command_(struct osdp_cmd &cmd) {
    if (!started_) {
      ESP_LOGW(TAG, "Ignoring command %d before bridge start", cmd.id);
      return false;
    }
    struct osdp_cmd *queued_cmd = reserve_command_slot_(cmd);
    if (queued_cmd == nullptr) {
      ESP_LOGW(TAG, "No free OSDP command slots for command %d", cmd.id);
      return false;
    }
    if (control_panel_.submit_command(0, queued_cmd) == 0) {
      return true;
    }
    release_command_slot_(queued_cmd);
    ESP_LOGW(TAG, "Failed to queue OSDP command %d", cmd.id);
    return false;
  }

  struct osdp_cmd *reserve_command_slot_(const struct osdp_cmd &cmd) {
    for (size_t i = 0; i < kCommandPoolSize; ++i) {
      if (!command_slot_in_use_[i]) {
        command_slot_in_use_[i] = true;
        command_pool_[i] = cmd;
        return &command_pool_[i];
      }
    }
    return nullptr;
  }

  void release_command_slot_(const struct osdp_cmd *cmd) {
    for (size_t i = 0; i < kCommandPoolSize; ++i) {
      if (&command_pool_[i] == cmd) {
        command_slot_in_use_[i] = false;
        std::memset(&command_pool_[i], 0, sizeof(command_pool_[i]));
        return;
      }
    }
  }

  bool parse_scbk_() {
    std::string filtered;
    filtered.reserve(scbk_hex_.size());
    for (char c : scbk_hex_) {
      if (c != ' ' && c != ':' && c != '-') {
        filtered.push_back(c);
      }
    }

    if (filtered.empty()) {
      return false;
    }

    if (filtered.size() != 32) {
      ESP_LOGW(TAG, "Ignoring SCBK because it is not 32 hex chars");
      return false;
    }

    for (size_t i = 0; i < 16; ++i) {
      const int hi = hex_nibble_(filtered[i * 2]);
      const int lo = hex_nibble_(filtered[i * 2 + 1]);
      if (hi < 0 || lo < 0) {
        ESP_LOGW(TAG, "Ignoring SCBK because it contains invalid hex");
        return false;
      }
      scbk_[i] = static_cast<uint8_t>((hi << 4) | lo);
    }

    return true;
  }

  static int hex_nibble_(char c) {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
      return 10 + c - 'a';
    }
    if (c >= 'A' && c <= 'F') {
      return 10 + c - 'A';
    }
    return -1;
  }

  static std::string bytes_to_hex_(const uint8_t *data, int length) {
    static const char *hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(length * 2);
    for (int i = 0; i < length; ++i) {
      out.push_back(hex[(data[i] >> 4) & 0x0F]);
      out.push_back(hex[data[i] & 0x0F]);
    }
    return out;
  }

  int uart_bus_;
  int rx_pin_;
  int tx_pin_;
  int direction_pin_;
  int baud_rate_;
  int reader_address_;
  std::string scbk_hex_;
  static constexpr size_t kCommandPoolSize = 4;
  HardwareSerial serial_;
  OsdpSerialChannel channel_state_;
  struct osdp_channel channel_;
  OSDP::ControlPanel control_panel_;
  osdp_pd_info_t pd_info_{};
  struct osdp_cmd command_pool_[kCommandPoolSize]{};
  bool command_slot_in_use_[kCommandPoolSize]{};
  uint8_t scbk_[16]{};
  bool scbk_present_{false};
  bool started_{false};
  uint32_t last_status_log_ms_{0};
  binary_sensor::BinarySensor *online_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *tamper_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *door_binary_sensor_{nullptr};
  binary_sensor::BinarySensor *secure_channel_binary_sensor_{nullptr};
  text_sensor::TextSensor *last_card_text_sensor_{nullptr};
  text_sensor::TextSensor *last_keypad_text_sensor_{nullptr};
  text_sensor::TextSensor *last_event_text_sensor_{nullptr};
  inline static OsdpControllerBridge *instance_{nullptr};
};

}  // namespace esphome::osdp_bridge
