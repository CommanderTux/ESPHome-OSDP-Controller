# ESPHome OSDP Controller

ESPHome-based OSDP control panel for ESP32, intended for Home Assistant use.

This project uses:

- ESPHome for Wi-Fi, OTA, API, and entity exposure
- LibOSDP for real OSDP communication
- RS485 half-duplex direction control on GPIO4

## Features

- OSDP reader communication over RS485
- Home Assistant entities for reader online state, secure channel state, tamper, and events
- Card and keypad event reporting
- Reader LED control
- Reader output control
- Access granted / denied helper actions with LED and buzzer feedback

## Wiring

Default ESP32 wiring in the included YAML:

- `GPIO16` -> RS485 RX
- `GPIO17` -> RS485 TX
- `GPIO4` -> RS485 DE/RE direction control

Adjust these if your board or transceiver uses different pins.

## Files

- `src/osdp-controller.yaml`: ESPHome configuration
- `src/osdp_api.h`: Custom LibOSDP bridge used by ESPHome
- `include/osdp.h` and `include/osdp.hpp`: local headers used for PlatformIO compatibility/reference

## Setup

1. Copy `src/osdp-controller.yaml` and `src/osdp_api.h` into your ESPHome config directory.
2. Update Wi-Fi, OTA, and API settings.
3. Set the correct OSDP reader address in the bridge constructor.
4. Set the SCBK string in the bridge constructor if your reader expects secure channel.

Current constructor example:

```cpp
auto *bridge = new esphome::osdp_bridge::OsdpControllerBridge(
    2, 16, 17, 4, 9600, 1, "YOUR_32_HEX_CHAR_SCBK");
```

If your reader allows plain mode, the SCBK can be left empty. If your reader only works with a configured key, use the same SCBK that worked in the standalone build.

## Home Assistant Controls

The ESPHome config exposes:

- `Reader Online`
- `Reader Secure Channel`
- `Reader Tamper`
- `Reader Door`
- `Last Card`
- `Last Keypad`
- `Last OSDP Event`
- `Reader LED`
- `Reader Output 1`
- `Reader LED Blue`
- `Reader LED Green`
- `Reader LED Red`
- `Access Granted`
- `Access Denied`

It also exposes API services for access granted, access denied, buzzer control, and other bridge actions.

## Notes

- The standalone `src/main.cpp` firmware is intentionally not tracked in this GitHub project.
- ESPHome builds its own generated `main.cpp`; the custom bridge logic lives in `src/osdp_api.h`.
