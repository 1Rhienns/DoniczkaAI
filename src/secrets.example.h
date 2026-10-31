#pragma once
// WZOR. Skopiuj jako secrets.h i uzupelnij. secrets.h NIE trafia na GitHuba.

// Siec WiFi 2.4 GHz (ESP32 nie widzi 5 GHz). Pusta nazwa = praca bez sieci.
#define WIFI_SSID      ""
#define WIFI_PASSWORD  ""

// Broker MQTT: adres IP (np. "192.168.1.50") albo nazwa hosta.
// test.mosquitto.org to PUBLICZNY broker testowy - dane widzi kazdy. Tylko do testow!
#define MQTT_HOST      "test.mosquitto.org"
#define MQTT_PORT      1883      // 1883 = bez szyfrowania, 8883 = TLS (wymaga zmian w kodzie)
#define MQTT_USER      ""        // pusty = broker bez logowania
#define MQTT_PASSWORD  ""
