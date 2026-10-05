// Template for secrets.h. Copy it to include/secrets.h and fill in real values.
// secrets.h is gitignored so passwords never reach GitHub.
#pragma once

#define WIFI_SSID     "your-network"
#define WIFI_PASSWORD "your-password"

// MQTT broker (EMQX Cloud Serverless). Host looks like xxxxxxxx.ala.eu-central-1.emqxsl.com
#define MQTT_HOST     "your-cluster.ala.eu-central-1.emqxsl.com"
#define MQTT_PORT     8883
#define MQTT_USER     "esp32-ac"
#define MQTT_PASSWORD "your-mqtt-password"
