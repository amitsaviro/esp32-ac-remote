// Template for secrets.h. Copy it to include/secrets.h and fill in real values.
// secrets.h is gitignored so passwords never reach GitHub.
#pragma once

#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"

// MQTT broker (HiveMQ Cloud). Host looks like xxxxxxxx.s1.eu.hivemq.cloud
#define MQTT_HOST     "your-cluster.s1.eu.hivemq.cloud"
#define MQTT_PORT     8883
#define MQTT_USER     "esp32-ac"
#define MQTT_PASSWORD "your-mqtt-password"
