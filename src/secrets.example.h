#pragma once
// WZOR. Skopiuj jako secrets.h i uzupelnij. secrets.h NIE trafia na GitHuba.

// Siec WiFi 2.4 GHz (ESP32 nie widzi 5 GHz). Pusta nazwa = praca bez sieci.
#define WIFI_SSID      ""
#define WIFI_PASSWORD  ""

// Broker MQTT: adres IP (np. "192.168.1.50") albo  nazwa hosta.
#define MQTT_HOST      "test.mosquitto.org"
#define MQTT_PORT      1883
#define MQTT_USER      ""
#define MQTT_PASSWORD  ""
