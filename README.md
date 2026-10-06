# ❄️ ESP32 AC Remote: control your air conditioner from anywhere

Turn an ordinary infrared-remote air conditioner into a smart one, for about $10 in parts.
An ESP32-C6 sits next to the AC, receives commands from a phone app over the internet, and "presses the buttons" by sending the same infrared signals as the original remote.

**[Open the phone app →](https://amitsaviro.github.io/esp32-ac-remote/)** (needs your own broker credentials)

## How it works

```
📱 Phone app ──wss──► ☁️ MQTT broker (EMQX Cloud) ◄──mqtts── 🔌 ESP32-C6 ──infrared──► ❄️ AC
                                                                 ▲
                                               original remote ──┘ (IR receiver keeps the app in sync)
```

- **Phone app**: a web app (PWA) you can add to the home screen. It publishes commands like `{"power":"on","temp":23}`.
- **MQTT broker**: a cloud "message board". Both the phone and the ESP32 connect *out* to it, so there are no open ports on the home router. Everything is encrypted (TLS) and password-protected.
- **ESP32-C6**: subscribes to commands, encodes them in the AC's infrared protocol, and publishes the current state back (retained, so the app shows it immediately). A "last will" message tells the app if the board goes offline.
- **IR receiver**: listens to the original remote, so presses on it show up in the app too.

## Hardware

| Part | Purpose |
|---|---|
| ESP32-C6 dev board | Wi-Fi + the brains |
| IR LED (940nm) | Sends commands to the AC (salvaged from an old remote) |
| IR receiver (VS1838B) | Learns and follows the original remote |
| Jumper wires | No soldering needed |

| Component | ESP32 pin |
|---|---|
| IR LED + (long leg) | GPIO 3 |
| IR LED − | GND |
| IR receiver OUT / GND / VCC | GPIO 2 / GND / 3V3 |

## What I learned building it

1. **Reverse engineering the remote.** Recording the remote with the IR receiver showed my Tornado AC speaks the **Whirlpool** protocol: 168 bits that carry the *entire* state (power, temperature, mode, fan, even the clock) on every button press.
2. **Power is a toggle.** The remote has no separate "on" and "off" buttons, so the ESP32 tracks the state (saved to flash) and the app can correct it.
3. **Measure, don't guess.** Three bugs looked like "the LED is broken", and none of them were:
   - a loose connection (found with an electrical pin test),
   - a 32 kHz carrier instead of 38 kHz, because software pin toggling was too slow (found by counting pulses),
   - bits garbled by Wi-Fi interrupts, fixed by moving transmission to the ESP32's **RMT** hardware peripheral. Its first version had an inverted carrier, found by comparing timings with the original remote.

## Setup

1. Install [PlatformIO](https://platformio.org/).
2. Copy `include/secrets.example.h` to `include/secrets.h` and fill in your Wi-Fi and MQTT broker details.
3. Create a free MQTT broker (e.g. EMQX Cloud Serverless) with two users: one for the board, one for the phone.
4. Flash: `pio run -t upload`
5. Open the web app, enter the phone user's credentials, and add it to your home screen.

To adapt it to another AC brand, record your remote first: [IRremoteESP8266](https://github.com/crankyoldgit/IRremoteESP8266) supports dozens of AC protocols.

## Project layout

```
src/main.cpp         ESP32 firmware (commented for beginners)
include/root_ca.h    broker's root TLS certificate
web/                 phone app (single HTML file + manifest + icons)
.github/workflows/   publishes web/ to GitHub Pages
```
