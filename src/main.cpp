#include <Arduino.h>
#include <cstring>

#include <osdp.hpp>

namespace {

constexpr uint32_t kConsoleBaudRate = 115200;
constexpr uint32_t kReaderBaudRate = 9600;
constexpr int kReaderRxPin = 16;
constexpr int kReaderTxPin = 17;
constexpr int kRs485DirectionPin = 4;
constexpr int kReaderAddress = 1 ;
constexpr uint8_t kStatusMaskSize = 1;

HardwareSerial reader_bus(2);
OSDP::ControlPanel control_panel;
String console_line;

struct OsdpSerialChannel {
  HardwareSerial *port;
  int direction_pin;
};

OsdpSerialChannel osdp_channel_state{&reader_bus, kRs485DirectionPin};

osdp_pd_info_t pd_info = {
    .name = "reader-0",
    .baud_rate = static_cast<int>(kReaderBaudRate),
    .address = kReaderAddress,
    .flags = OSDP_FLAG_ENABLE_NOTIFICATION,
    .id = {},
    .cap = nullptr,
    .scbk = nullptr,
};

struct ControllerState {
  bool reader_online{false};
  bool secure_channel_active{false};
} controller_state;

String bytes_to_hex(const uint8_t *data, int length) {
  String hex;
  for (int i = 0; i < length; ++i) {
    if (data[i] < 0x10) {
      hex += '0';
    }
    hex += String(data[i], HEX);
  }
  hex.toUpperCase();
  return hex;
}

int osdp_read(void *data, uint8_t *buf, int maxlen) {
  auto *channel = static_cast<OsdpSerialChannel *>(data);
  int count = 0;

  while (count < maxlen && channel->port->available()) {
    buf[count++] = static_cast<uint8_t>(channel->port->read());
  }

  return count;
}

int osdp_write(void *data, uint8_t *buf, int len) {
  auto *channel = static_cast<OsdpSerialChannel *>(data);
  digitalWrite(kRs485DirectionPin, HIGH);
  delayMicroseconds(50);
  const int written = channel->port->write(buf, len);
  channel->port->flush();
  delayMicroseconds(50);
  digitalWrite(channel->direction_pin, LOW);
  return written == len ? written : -1;
}

void osdp_flush(void *data) {
  auto *channel = static_cast<OsdpSerialChannel *>(data);
  channel->port->flush();
  while (channel->port->available()) {
    channel->port->read();
  }
}

osdp_channel channel = {
    .data = &osdp_channel_state,
    .recv = osdp_read,
    .send = osdp_write,
    .flush = osdp_flush,
    .close = nullptr,
};

int osdp_event_callback(void *arg, int pd, struct osdp_event *event) {
  auto *state = static_cast<ControllerState *>(arg);

  switch (event->type) {
    case OSDP_EVENT_CARDREAD:
      Serial.printf("EVENT pd=%d card=%s bits=%d\n", pd,
                    bytes_to_hex(event->cardread.data,
                                 (event->cardread.length + 7) / 8)
                        .c_str(),
                    event->cardread.length);
      break;
    case OSDP_EVENT_KEYPRESS: {
      String keypad;
      for (int i = 0; i < event->keypress.length; ++i) {
        keypad += static_cast<char>(event->keypress.data[i]);
      }
      Serial.printf("EVENT pd=%d keypad=%s\n", pd, keypad.c_str());
      break;
    }
    case OSDP_EVENT_STATUS:
      if (event->status.type == OSDP_STATUS_REPORT_REMOTE &&
          event->status.nr_entries > 0) {
        const bool tamper = (event->status.report[0] & 0x01) != 0;
        const bool power = (event->status.report[0] & 0x02) != 0;
        Serial.printf("EVENT pd=%d remote_tamper=%d remote_power=%d\n", pd,
                      tamper, power);
      }
      break;
    case OSDP_EVENT_NOTIFICATION:
      if (event->notif.type == OSDP_EVENT_NOTIFICATION_PD_STATUS) {
        state->reader_online = event->notif.arg0 != 0;
        Serial.printf("EVENT pd=%d online=%d\n", pd, state->reader_online);
      } else if (event->notif.type == OSDP_EVENT_NOTIFICATION_SC_STATUS) {
        state->secure_channel_active = event->notif.arg0 != 0;
        Serial.printf("EVENT pd=%d secure_channel=%d\n", pd,
                      state->secure_channel_active);
      }
      break;
    case OSDP_EVENT_MFGREP:
      Serial.printf("EVENT pd=%d manufacturer_reply_len=%d\n", pd,
                    event->mfgrep.length);
      break;
    default:
      break;
  }

  return 0;
}

void osdp_command_complete_callback(void *arg, int pd,
                                    const struct osdp_cmd *cmd,
                                    enum osdp_completion_status status) {
  (void)arg;
  Serial.printf("CMD pd=%d id=%d status=%d\n", pd, cmd->id, status);
}

void print_help() {
  Serial.println();
  Serial.println("OSDP commands:");
  Serial.println("  status");
  Serial.println("  led on|off");
  Serial.println("  buzzer <on_ms> <off_ms> <repeats>");
  Serial.println("  output <channel> on|off");
  Serial.println();
}

bool submit_command(struct osdp_cmd &cmd) {
  if (control_panel.submit_command(0, &cmd) == 0) {
    return true;
  }

  Serial.println("Failed to queue OSDP command");
  return false;
}

void submit_status_query() {
  struct osdp_cmd cmd = {};
  cmd.id = OSDP_CMD_STATUS;
  cmd.status.type = OSDP_STATUS_REPORT_REMOTE;
  cmd.status.nr_entries = 0;
  submit_command(cmd);
}

void submit_led_command(bool enabled) {
  struct osdp_cmd cmd = {};
  cmd.id = OSDP_CMD_LED;
  cmd.led.reader = 0;
  cmd.led.led_number = 0;
  cmd.led.temporary.control_code = 0;
  cmd.led.permanent.control_code = 1;
  cmd.led.permanent.on_count = enabled ? 1 : 0;
  cmd.led.permanent.off_count = 0;
  cmd.led.permanent.on_color =
      enabled ? OSDP_LED_COLOR_GREEN : OSDP_LED_COLOR_NONE;
  cmd.led.permanent.off_color = OSDP_LED_COLOR_NONE;
  cmd.led.permanent.timer_count = 0;
  submit_command(cmd);
}

void submit_buzzer_command(uint16_t on_ms, uint16_t off_ms, uint8_t repeats) {
  struct osdp_cmd cmd = {};
  cmd.id = OSDP_CMD_BUZZER;
  cmd.buzzer.reader = 0;
  cmd.buzzer.control_code = repeats == 0 ? 1 : 2;
  cmd.buzzer.on_count = on_ms / 100;
  cmd.buzzer.off_count = off_ms / 100;
  cmd.buzzer.rep_count = repeats;
  submit_command(cmd);
}

void submit_output_command(uint8_t output, bool enabled) {
  struct osdp_cmd cmd = {};
  cmd.id = OSDP_CMD_OUTPUT;
  cmd.output.output_no = output;
  cmd.output.control_code = enabled ? 2 : 1;
  cmd.output.timer_count = 0;
  submit_command(cmd);
}

void handle_console_command(const String &line) {
  if (line == "status") {
    submit_status_query();
    return;
  }

  if (line == "led on") {
    submit_led_command(true);
    return;
  }

  if (line == "led off") {
    submit_led_command(false);
    return;
  }

  if (line.startsWith("buzzer ")) {
    uint16_t on_ms = 0;
    uint16_t off_ms = 0;
    uint8_t repeats = 0;
    if (sscanf(line.c_str(), "buzzer %hu %hu %hhu", &on_ms, &off_ms,
               &repeats) == 3) {
      submit_buzzer_command(on_ms, off_ms, repeats);
      return;
    }
  }

  if (line.startsWith("output ")) {
    uint8_t channel = 0;
    char state[4] = {0};
    if (sscanf(line.c_str(), "output %hhu %3s", &channel, state) == 2) {
      submit_output_command(channel, String(state) == "on");
      return;
    }
  }

  print_help();
}

}  // namespace

void setup() {
  Serial.begin(kConsoleBaudRate);
  pinMode(kRs485DirectionPin, OUTPUT);
  digitalWrite(kRs485DirectionPin, LOW);
  reader_bus.begin(kReaderBaudRate, SERIAL_8N1, kReaderRxPin, kReaderTxPin);

  delay(250);
  Serial.println();
  Serial.printf("LibOSDP version: %s\n", osdp_get_version());

  if (!control_panel.setup(&channel, 1, &pd_info)) {
    Serial.println("Failed to initialise LibOSDP control panel");
    return;
  }

  control_panel.set_event_callback(osdp_event_callback, &controller_state);
  control_panel.set_command_completion_callback(
      osdp_command_complete_callback, nullptr);

  Serial.println("OSDP control panel started");
  print_help();
}

void loop() {
  static uint32_t last_status_log_ms = 0;

  control_panel.refresh();

  if (millis() - last_status_log_ms >= 5000) {
    uint8_t online_mask[kStatusMaskSize] = {0};
    uint8_t secure_mask[kStatusMaskSize] = {0};
    control_panel.get_status_mask(online_mask);
    control_panel.get_sc_status_mask(secure_mask);
    Serial.printf("STATE online_mask=0x%02X sc_mask=0x%02X\n", online_mask[0],
                  secure_mask[0]);
    last_status_log_ms = millis();
  }

  while (Serial.available()) {
    const char byte = static_cast<char>(Serial.read());

    if (byte == '\r') {
      continue;
    }

    if (byte == '\n') {
      if (!console_line.isEmpty()) {
        handle_console_command(console_line);
        console_line.clear();
      }
      continue;
    }

    if (console_line.length() < 120) {
      console_line += byte;
    }
  }

  delay(10);
}
