// =============================================================================
//  AC remote over the internet
// =============================================================================
//
//  THE BIG PICTURE
//  ---------------
//    Phone app  --(internet)-->  MQTT broker in the cloud  --(internet)-->
//    this ESP32 at home  --(infrared light)-->  air conditioner
//
//  The ESP32 is a tiny computer with Wi-Fi. It:
//    1. joins the home Wi-Fi,
//    2. connects to a cloud "message board" (an MQTT broker),
//    3. waits for commands from the phone, like {"power":"on","temp":23},
//    4. turns each command into the same infrared flashes the AC's remote sends,
//    5. reports the AC's current state back so the phone can show it.
//  It also listens to the ORIGINAL remote, so if someone uses it, the app
//  still shows the right state.
//
//  WHY A BROKER IN THE MIDDLE?
//  The home router blocks connections coming in from the internet (good for
//  security). So neither side connects to the other directly: both the phone
//  and the ESP32 connect OUT to the broker, and the broker passes messages.
//
//  MQTT TOPICS (like named mailboxes; TOPIC_BASE = "ac/livingroom")
//    <base>/set     phone -> ESP32  JSON, any subset of:
//                   {"power":"on"|"off"|"toggle", "temp":24,
//                    "mode":"cool"|"heat"|"dry"|"fan"|"auto",
//                    "fan":"auto"|"low"|"medium"|"high"}
//                   or {"assume":"on"|"off"} to fix our power state without sending
//    <base>/state   ESP32 -> phone  JSON with the full current state (retained)
//    <base>/status  "online" / "offline" (retained, offline set by the broker
//                   automatically if the ESP32 disappears)
//
//  Serial debug commands (type in the serial monitor): p = power,
//    + / - = temp, m = mode, s = state, l = loopback, d = drive strength
//
//  Onboard RGB LED: yellow = connecting to Wi-Fi, cyan = Wi-Fi but no MQTT,
//  green = fully connected, blue flash = IR received, purple flash = IR sent.
// =============================================================================

#include <Arduino.h>
#include <WiFi.h>              // join the home Wi-Fi
#include <WiFiClientSecure.h>  // encrypted (TLS) internet connections
#include <PubSubClient.h>      // MQTT: talk to the cloud broker
#include <ArduinoJson.h>       // read/write JSON messages like {"temp":23}
#include <Preferences.h>       // save small values that survive a reboot
#include <time.h>              // real clock from the internet
#include <IRrecv.h>            // decode infrared signals from the original remote
#include <IRsend.h>
#include <IRac.h>
#include <IRutils.h>
#include <ir_Whirlpool.h>      // knows the AC's "language" (Whirlpool protocol)
#include <driver/gpio.h>       // low-level pin settings (drive strength)
#include "secrets.h"           // Wi-Fi and broker passwords, NOT on GitHub
#include "root_ca.h"           // certificate to verify the broker is genuine

// ---------------------------------------------------------------- settings

const uint8_t BRIGHTNESS = 40;  // 0-255; the LED is very bright at full power
const unsigned long CONNECT_TIMEOUT_MS = 20000;  // give up on Wi-Fi after 20s
const unsigned long MQTT_RETRY_MS = 5000;        // retry the broker every 5s

// Israel time zone, including daylight saving rules. Used to set the AC clock.
const char *TIME_ZONE = "IST-2IDT,M3.4.4/26,M10.5.0";

const char *TOPIC_BASE = "ac/livingroom";
String topicSet, topicState, topicStatus;

// IR RECEIVER: the black 3-leg part. It turns infrared flashes into a clean
// on/off signal on this pin, so we can "hear" the original remote.
const uint16_t IR_RECV_PIN = 2;
// AC remotes send the whole state in one long message, so we need a big
// buffer (default is 100) and a timeout long enough to keep multi-part
// messages together but short enough to separate two button presses.
const uint16_t CAPTURE_BUFFER_SIZE = 1024;
const uint8_t RECV_TIMEOUT_MS = 50;
// Ignore short noise bursts (sunlight, lamps) that aren't real remote signals.
const uint16_t MIN_UNKNOWN_SIZE = 12;

// IR LED: the clear LED that "talks" to the AC with invisible light.
// It's wired straight to the pin with no current-limiting resistor,
// so we lower the pin's drive strength (~10mA instead of the default ~20mA)
// to protect both the LED and the ESP32. Range is about a meter.
const uint16_t IR_SEND_PIN = 3;

// HOW THE AC'S LANGUAGE LOOKS (Whirlpool protocol)
// Every message is a pattern of light flashes ("marks") and pauses ("spaces"),
// measured in microseconds (millionths of a second):
//   - a long "attention" flash + pause (the header),
//   - then 168 bits: each bit is a short flash, followed by
//     a SHORT pause for a 0 or a LONG pause for a 1.
// These numbers come from IRremoteESP8266's ir_Whirlpool.cpp and match what
// we recorded from the real remote.
const uint16_t WP_HDR_MARK = 8950;
const uint16_t WP_HDR_SPACE = 4484;
const uint16_t WP_BIT_MARK = 597;
const uint16_t WP_ONE_SPACE = 1649;
const uint16_t WP_ZERO_SPACE = 533;
const uint16_t WP_GAP = 7920;
// The 21-byte message is sent in three sections separated by gaps.
const uint8_t WP_SECTION_SIZES[] = {6, 8, 7};

IRrecv irrecv(IR_RECV_PIN, CAPTURE_BUFFER_SIZE, RECV_TIMEOUT_MS, true);
// Holds the AC settings (temp, mode, fan...) and packs them into the 21 bytes
// the AC understands. Sending goes through the RMT hardware instead
// (see sendIrState), so begin() is never called on it.
IRWhirlpoolAc ac(IR_SEND_PIN);
decode_results results;

WiFiClientSecure tlsClient;  // encrypted connection, so nobody can read or fake our messages
PubSubClient mqtt(tlsClient);
Preferences prefs;           // small key-value store in flash that survives reboots

// The AC's power button is a TOGGLE ("flip on/off"), not separate on and off
// buttons. So the ESP32 must remember whether it thinks the AC is on,
// otherwise "turn on" could accidentally turn it off.
bool acIsOn = false;

// ---------------------------------------------------------------- LED

void setLed(uint8_t r, uint8_t g, uint8_t b) {
  rgbLedWrite(RGB_BUILTIN, r, g, b);
}

// Shows the connection status at a glance, without a computer attached.
void restoreStatusLed() {
  if (WiFi.status() != WL_CONNECTED) setLed(0, 0, 0);
  else if (!mqtt.connected()) setLed(0, BRIGHTNESS, BRIGHTNESS);  // cyan
  else setLed(0, BRIGHTNESS, 0);                                   // green
}

// ---------------------------------------------------------------- names

// Mode and fan codes are numbers inside the IR protocol; the app uses words.
// These tables translate between the two, e.g. "cool" <-> 2.
struct Named { const char *name; uint8_t value; };
const Named MODES[] = {{"cool", kWhirlpoolAcCool}, {"dry", kWhirlpoolAcDry},
                       {"fan", kWhirlpoolAcFan},   {"heat", kWhirlpoolAcHeat},
                       {"auto", kWhirlpoolAcAuto}};
const Named FANS[] = {{"auto", kWhirlpoolAcFanAuto}, {"low", kWhirlpoolAcFanLow},
                      {"medium", kWhirlpoolAcFanMedium}, {"high", kWhirlpoolAcFanHigh}};
const size_t MODE_COUNT = sizeof(MODES) / sizeof(MODES[0]);
const size_t FAN_COUNT = sizeof(FANS) / sizeof(FANS[0]);

// Number -> word (for messages we send to the phone).
const char *nameOf(const Named *table, size_t count, uint8_t value) {
  for (size_t i = 0; i < count; i++)
    if (table[i].value == value) return table[i].name;
  return "unknown";
}

// Word -> number (for commands we get from the phone).
bool valueOf(const Named *table, size_t count, const char *name, uint8_t &value) {
  for (size_t i = 0; i < count; i++) {
    if (strcmp(table[i].name, name) == 0) {
      value = table[i].value;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------- state

// Saves the AC settings to flash memory, so a power cut or reboot doesn't
// make the ESP32 forget that the AC is on.
void saveState() {
  prefs.putBool("on", acIsOn);
  prefs.putUChar("temp", ac.getTemp());
  prefs.putUChar("mode", ac.getMode());
  prefs.putUChar("fan", ac.getFan());
}

// Reads the saved settings back after a reboot (24C cool is the default).
void loadState() {
  acIsOn = prefs.getBool("on", false);
  ac.setTemp(prefs.getUChar("temp", 24));
  ac.setMode(prefs.getUChar("mode", kWhirlpoolAcCool));
  ac.setFan(prefs.getUChar("fan", kWhirlpoolAcFanAuto));
}

void printState() {
  Serial.printf("AC is %s | %s\n", acIsOn ? "ON" : "OFF", ac.toString().c_str());
}

// Tells the phone (and anyone else subscribed) what the AC is doing now.
// `source` says who caused the change: "app", "remote", "serial" or "boot".
void publishState(const char *source) {
  saveState();
  printState();
  if (!mqtt.connected()) return;

  // Build a JSON message like {"power":true,"temp":23,"mode":"cool",...}
  JsonDocument doc;
  doc["power"] = acIsOn;
  doc["temp"] = ac.getTemp();
  doc["mode"] = nameOf(MODES, MODE_COUNT, ac.getMode());
  doc["fan"] = nameOf(FANS, FAN_COUNT, ac.getFan());
  doc["source"] = source;
  doc["rssi"] = WiFi.RSSI();  // Wi-Fi signal strength, handy for debugging

  char payload[256];
  serializeJson(doc, payload);
  // Retained: the broker keeps the last state, so a phone that opens the app
  // later gets it immediately instead of waiting for the next change.
  mqtt.publish(topicState.c_str(), payload, true);
}

// ---------------------------------------------------------------- IR send

// RMT = "Remote Control Transceiver", a piece of hardware inside the ESP32
// built for exactly this job. We hand it a list of "on for X us, off for Y us"
// steps and it plays them back with perfect timing by itself.
// Why not just switch the pin on/off in code? We tried: the processor also
// handles Wi-Fi, and every Wi-Fi interruption stretched a flash and garbled
// a bit, so the AC rejected the message. Hardware timing fixed it.
void setupIrSender() {
  rmtInit(IR_SEND_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 1000000);  // 1 tick = 1us
  // CARRIER: during every "mark" the LED doesn't just stay on, it blinks
  // 38,000 times per second. The AC's receiver only reacts to light blinking
  // at that rate, which is how it ignores sunlight and lamps.
  // 50% duty = on half of each blink.
  // Note carrier_level = false: with true the carrier ran during the LOW parts,
  // which inverted the whole signal (long marks, short spaces).
  rmtSetCarrier(IR_SEND_PIN, true, false, 38000, 0.5);
  // Limit the pin's current, since the LED has no resistor (see IR_SEND_PIN).
  gpio_set_drive_capability((gpio_num_t)IR_SEND_PIN, GPIO_DRIVE_CAP_1);
}

// Turns the 21 bytes of AC settings into light flashes and sends them.
// Each RMT "symbol" is one pair: LED on for X us, then off for Y us.
void sendIrState(const uint8_t *state) {
  rmt_data_t symbols[1 + kWhirlpoolAcBits + sizeof(WP_SECTION_SIZES)];
  size_t n = 0;
  // Small helper: append one "flash, then pause" step to the list.
  auto add = [&](uint16_t mark, uint16_t space) {
    symbols[n++] = {.duration0 = mark, .level0 = 1, .duration1 = space, .level1 = 0};
  };

  add(WP_HDR_MARK, WP_HDR_SPACE);  // the "attention!" header
  size_t byteIndex = 0;
  for (uint8_t section = 0; section < sizeof(WP_SECTION_SIZES); section++) {
    for (uint8_t i = 0; i < WP_SECTION_SIZES[section]; i++, byteIndex++) {
      for (uint8_t bit = 0; bit < 8; bit++) {  // least significant bit first
        bool one = state[byteIndex] & (1 << bit);
        // Every bit starts with the same short flash; the pause length
        // after it is what tells the AC whether it's a 1 or a 0.
        add(WP_BIT_MARK, one ? WP_ONE_SPACE : WP_ZERO_SPACE);
      }
    }
    add(WP_BIT_MARK, WP_GAP);  // section footer
  }
  rmtWrite(IR_SEND_PIN, symbols, n, 1000);  // send it all (wait up to 1s)
}

// Keeps the AC's own clock correct, the way the original remote does.
void updateAcClock() {
  struct tm now;
  if (getLocalTime(&now, 0)) ac.setClock(now.tm_hour * 60 + now.tm_min);
}

// Sends the current settings to the AC.
// `command` says which "button" was pressed (power, temp, mode...). The AC
// uses it to know what changed, e.g. to beep or show the new temperature.
void sendToAc(uint8_t command) {
  updateAcClock();
  ac.setCommand(command);
  // Our own LED would bounce back into the receiver and look like a remote
  // press, so stop listening while we transmit.
  irrecv.disableIRIn();
  setLed(BRIGHTNESS, 0, BRIGHTNESS);  // purple
  sendIrState(ac.getRaw());  // getRaw() also calculates the checksum bytes
  ac.setPowerToggle(false);  // toggle is a one-shot flag, never leave it set
  ac.setCommand(command);    // setPowerToggle() overwrites the command; restore it
  delay(100);
  irrecv.enableIRIn();
  restoreStatusLed();
}

// Presses the power button (on <-> off) and remembers the new state.
void togglePower() {
  ac.setPowerToggle(true);
  acIsOn = !acIsOn;
  Serial.printf(">> Power toggle (AC should now be %s)\n", acIsOn ? "ON" : "OFF");
  sendToAc(kWhirlpoolAcCommandPower);
}

// ---------------------------------------------------------------- commands from the app

// Runs a command that arrived from the phone, e.g. {"power":"on","temp":23}.
// One IR message always carries the WHOLE state, so even if several settings
// change we send only once, tagged with the most important "button" that changed.
void handleSetCommand(const char *json) {
  JsonDocument doc;
  if (deserializeJson(doc, json)) {
    Serial.printf("Ignoring bad JSON: %s\n", json);
    return;
  }

  // {"assume":"on"|"off"} corrects our idea of the power state without sending
  // anything, for when the AC was switched with the original remote out of
  // the receiver's sight.
  if (doc["assume"].is<const char *>()) {
    acIsOn = strcmp(doc["assume"], "on") == 0;
    Serial.printf(">> Assuming the AC is %s\n", acIsOn ? "ON" : "OFF");
    publishState("app");
    return;
  }

  // Power is a toggle, so "on" only presses the button if the AC is off,
  // and "off" only if it's on.
  bool wantPowerChange = false;
  if (doc["power"].is<const char *>()) {
    const char *p = doc["power"];
    if (strcmp(p, "toggle") == 0) wantPowerChange = true;
    else if (strcmp(p, "on") == 0) wantPowerChange = !acIsOn;
    else if (strcmp(p, "off") == 0) wantPowerChange = acIsOn;
  }

  // Apply any new fan / temperature / mode to our settings.
  int command = -1;
  uint8_t value;
  if (doc["fan"].is<const char *>() && valueOf(FANS, FAN_COUNT, doc["fan"], value) &&
      value != ac.getFan()) {
    ac.setFan(value);
    command = kWhirlpoolAcCommandFanSpeed;
  }
  if (doc["temp"].is<int>() && doc["temp"].as<int>() != ac.getTemp()) {
    ac.setTemp(doc["temp"].as<int>());  // the library clamps to the valid range
    command = kWhirlpoolAcCommandTemp;
  }
  if (doc["mode"].is<const char *>() && valueOf(MODES, MODE_COUNT, doc["mode"], value) &&
      value != ac.getMode()) {
    ac.setMode(value);
    command = kWhirlpoolAcCommandMode;
  }

  if (wantPowerChange) {
    togglePower();  // carries the new temp/mode/fan along with it
  } else {
    // Send even if nothing changed: re-sending the same settings is harmless,
    // the AC beeps as feedback, and it fixes things if it missed a command.
    Serial.println(">> Settings from app");
    sendToAc(command >= 0 ? command : kWhirlpoolAcCommandTemp);
  }
  // Publish even when nothing changed, so the app always gets a reply.
  publishState("app");
}

// Called automatically by the MQTT library whenever a message arrives.
void onMqttMessage(char *topic, byte *payload, unsigned int length) {
  // The payload isn't a C string yet: copy it and add the end marker '\0'.
  char json[256];
  length = min(length, (unsigned int)sizeof(json) - 1);
  memcpy(json, payload, length);
  json[length] = '\0';
  Serial.printf("<< MQTT %s: %s\n", topic, json);
  if (topicSet == topic) handleSetCommand(json);
}

// ---------------------------------------------------------------- IR receive

// Someone pressed the ORIGINAL remote: copy its settings so we stay in sync,
// and tell the phone, so the app shows the change too.
void handleRemotePress() {
  setLed(0, 0, BRIGHTNESS);  // blue flash
  if (results.decode_type == WHIRLPOOL_AC) {
    ac.setRaw(results.state);                  // take all settings from the remote
    if (ac.getPowerToggle()) acIsOn = !acIsOn; // the power button was pressed
    ac.setPowerToggle(false);
    Serial.print("<< Original remote: ");
    publishState("remote");
  } else {
    // Some other remote, like a TV. Just log it.
    Serial.print("<< Other IR signal: ");
    Serial.print(resultToHumanReadableBasic(&results));
  }
  delay(100);
  restoreStatusLed();
}

// ---------------------------------------------------------------- diagnostics
// Tools we used while building and debugging. Triggered from the serial monitor.

// Sends a real AC command while the receiver keeps listening. If the IR LED
// points at the receiver and the receiver decodes it, the LED works.
void loopbackTest() {
  Serial.println(">> Loopback test: point the IR LED at the receiver");
  irrecv.resume();  // drop anything already captured
  setLed(BRIGHTNESS, 0, BRIGHTNESS);  // purple
  ac.setCommand(kWhirlpoolAcCommandTemp);
  sendIrState(ac.getRaw());
  restoreStatusLed();

  unsigned long start = millis();
  while (millis() - start < 500) {
    if (irrecv.decode(&results)) {
      Serial.print(">> Loopback received: ");
      Serial.print(resultToHumanReadableBasic(&results));
      Serial.println(results.decode_type == WHIRLPOOL_AC
                         ? ">> LOOPBACK OK: the IR LED works"
                         : ">> LOOPBACK PARTIAL: got a signal but it didn't decode");
      // First timings (microseconds), to compare with the original remote:
      // 9000, 4500 header, then ~600 marks with ~550 (0) or ~1650 (1) spaces.
      Serial.printf(">> Raw (%d entries):", results.rawlen - 1);
      for (uint16_t i = 1; i < results.rawlen && i <= 40; i++)
        Serial.printf(" %u", results.rawbuf[i] * kRawTick);
      Serial.println();
      irrecv.resume();
      return;
    }
    delay(10);
  }
  Serial.println(">> LOOPBACK FAILED: the receiver saw nothing");
}

// Steps the IR pin's drive strength up (1 -> 2 -> 3 -> 1) for testing.
// Roughly: 1 = ~10mA, 2 = ~20mA, 3 = ~40mA. More current = longer range.
void cycleDriveStrength() {
  gpio_drive_cap_t cap;
  gpio_get_drive_capability((gpio_num_t)IR_SEND_PIN, &cap);
  cap = (cap >= GPIO_DRIVE_CAP_3) ? GPIO_DRIVE_CAP_1 : (gpio_drive_cap_t)(cap + 1);
  gpio_set_drive_capability((gpio_num_t)IR_SEND_PIN, cap);
  Serial.printf(">> IR pin drive strength is now %d\n", cap);
}

void changeTemp(int delta) {
  ac.setTemp(ac.getTemp() + delta);  // the library clamps to the valid range
  Serial.printf(">> Temperature %dC\n", ac.getTemp());
  sendToAc(kWhirlpoolAcCommandTemp);
}

// Cycles to the next mode: cool -> dry -> fan -> heat -> auto -> cool...
void nextMode() {
  size_t i = 0;
  while (i < MODE_COUNT && MODES[i].value != ac.getMode()) i++;
  ac.setMode(MODES[(i + 1) % MODE_COUNT].value);
  Serial.printf(">> Mode %s\n", nameOf(MODES, MODE_COUNT, ac.getMode()));
  sendToAc(kWhirlpoolAcCommandMode);
}

// Single-key commands typed in the serial monitor (USB cable to a computer).
void handleSerialCommand(char c) {
  switch (c) {
    case 'p': togglePower(); publishState("serial"); break;
    case '+': changeTemp(+1); publishState("serial"); break;
    case '-': changeTemp(-1); publishState("serial"); break;
    case 'm': nextMode(); publishState("serial"); break;
    case 's': printState(); break;
    case 'l': loopbackTest(); break;
    case 'd': cycleDriveStrength(); break;
    case '\n': case '\r': case ' ': break;
    default:
      Serial.println("Commands: p = power, + / - = temp, m = mode, s = state, "
                     "l = loopback, d = drive strength");
  }
}

// ---------------------------------------------------------------- connections

// Joins the home Wi-Fi. Waits up to 20 seconds, printing dots meanwhile.
bool connectWifi() {
  setLed(BRIGHTNESS, BRIGHTNESS / 2, 0);  // yellow
  Serial.printf("Connecting to \"%s\"", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > CONNECT_TIMEOUT_MS) {
      Serial.printf("\nFailed (status %d)\n", WiFi.status());
      setLed(BRIGHTNESS, 0, 0);  // red
      return false;
    }
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nConnected!");
  Serial.print("  IP address: ");
  Serial.println(WiFi.localIP());
  Serial.printf("  Signal:     %d dBm\n", WiFi.RSSI());
  restoreStatusLed();
  return true;
}

// Logs in to the cloud broker and starts listening for commands.
bool connectMqtt() {
  Serial.printf("Connecting to MQTT broker %s:%d... ", MQTT_HOST, MQTT_PORT);
  // Last Will: if we vanish without saying goodbye (power cut, crash), the
  // broker publishes "offline" for us, so the app knows the ESP32 is down.
  String clientId = "esp32-ac-" + WiFi.macAddress();  // unique name per board
  if (!mqtt.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD,
                    topicStatus.c_str(), 1, true, "offline")) {
    Serial.printf("failed (state %d)\n", mqtt.state());
    restoreStatusLed();
    return false;
  }
  Serial.println("connected!");
  mqtt.publish(topicStatus.c_str(), "online", true);  // "I'm here!"
  mqtt.subscribe(topicSet.c_str(), 1);  // "send me every message on /set"
  restoreStatusLed();
  publishState("boot");
  return true;
}

// ---------------------------------------------------------------- main
// Every Arduino program has two functions:
//   setup() runs ONCE when the board powers on,
//   loop()  runs AGAIN AND AGAIN forever after that.

void setup() {
  Serial.begin(115200);  // text output to the computer, for debugging
  delay(500);

  topicSet = String(TOPIC_BASE) + "/set";
  topicState = String(TOPIC_BASE) + "/state";
  topicStatus = String(TOPIC_BASE) + "/status";

  prefs.begin("ac", false);

  WiFi.mode(WIFI_STA);  // station mode: join an existing network, don't create one
  // With a mesh network several access points share one name. By default the
  // ESP32 joins the first one it finds; scan everything and pick the strongest.
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  connectWifi();

  // Real clock from the internet: TLS needs it to check certificate dates,
  // and the AC remote protocol includes the time of day.
  configTzTime(TIME_ZONE, "pool.ntp.org", "time.google.com");

  // Only trust a broker whose certificate is signed by this root authority.
  tlsClient.setCACert(ROOT_CA);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);  // "call this function when a message arrives"
  mqtt.setBufferSize(512);

  setupIrSender();
  ac.setModel(DG11J191);  // the exact remote model we recorded
  ac.setLight(true);
  loadState();  // restore what we knew before the last reboot

  irrecv.setUnknownThreshold(MIN_UNKNOWN_SIZE);
  irrecv.enableIRIn();  // start listening to the original remote

  Serial.println("Ready. Commands: p = power, + / - = temp, m = mode, s = state, "
                 "l = loopback, d = drive strength");
  printState();
}

void loop() {
  // 1. Did the original remote send something?
  if (irrecv.decode(&results)) {
    handleRemotePress();
    irrecv.resume();  // get ready for the next signal
  }

  // 2. Did someone type a command in the serial monitor?
  while (Serial.available()) {
    handleSerialCommand(Serial.read());
  }

  // 3. Still on Wi-Fi? Reconnect automatically if the router dropped us.
  static unsigned long lastWifiCheck = 0;
  if (millis() - lastWifiCheck > 5000) {
    lastWifiCheck = millis();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("Wi-Fi lost, reconnecting...");
      WiFi.disconnect();
      connectWifi();
    }
  }

  // 4. Stay connected to the broker and handle incoming phone commands.
  // Skip MQTT until secrets.h has a real broker, so IR testing isn't slowed
  // down by connection attempts that can't succeed.
  static const bool mqttConfigured = strstr(MQTT_HOST, "your-cluster") == nullptr;
  if (mqttConfigured && WiFi.status() == WL_CONNECTED) {
    static unsigned long lastMqttAttempt = 0;
    if (!mqtt.connected() && (lastMqttAttempt == 0 || millis() - lastMqttAttempt > MQTT_RETRY_MS)) {
      lastMqttAttempt = millis();
      connectMqtt();
    }
    mqtt.loop();  // processes incoming messages and keeps the connection alive
  }
}
