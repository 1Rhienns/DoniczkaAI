#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <LittleFS.h>
#include <time.h>
#include "secrets.h"

constexpr bool HAS_FIRE  = false;
constexpr bool HAS_AIR   = true;
constexpr bool HAS_LIGHT = false;
constexpr bool HAS_UV    = false;

constexpr uint8_t PIN_WATER_LEVEL = 25;
constexpr uint8_t PIN_PUMP        = 26;
constexpr uint8_t PIN_FIRE        = 27;
constexpr uint8_t PIN_SOIL[3]     = {34, 35, 32};
constexpr uint8_t PIN_LIGHT       = 33;
constexpr uint8_t PIN_UV          = 36;

constexpr int  WATER_PRESENT_LEVEL = LOW;

constexpr uint32_t WATER_DEBOUNCE_MS = 2000;

constexpr int  FIRE_ACTIVE_LEVEL = LOW;
constexpr bool PUMP_ACTIVE_LOW   = true;

constexpr float SOIL_MV_DRY[3] = {2500.0f, 2500.0f, 2500.0f};
constexpr float SOIL_MV_WET[3] = {1100.0f, 1100.0f, 1100.0f};

constexpr float SOIL_PUMP_ON_PCT  = 30.0f;
constexpr float SOIL_PUMP_OFF_PCT = 60.0f;

constexpr uint32_t PUMP_MAX_RUN_MS  = 60UL * 1000UL;
constexpr uint32_t PUMP_COOLDOWN_MS = 5UL * 60UL * 1000UL;

constexpr uint32_t AIR_INTERVAL_MS = 1000;

constexpr float LIGHT_LUX_PER_MV  = 0.2f;

constexpr float LIGHT_MV_SATURATE = 3000.0f;

constexpr float UV_INDEX_PER_MV = 0.01f;

constexpr uint32_t READ_INTERVAL_MS = 60000;
constexpr uint32_t PAGE_INTERVAL_MS = 5000;
constexpr uint8_t  ADC_SAMPLES      = 16;

constexpr uint32_t SCREEN_REFRESH_MS = 1000;

constexpr uint32_t REPORT_INTERVAL_MS    = 6UL * 60UL * 60UL * 1000UL;
constexpr uint32_t FIRST_REPORT_DELAY_MS = 2UL * 60UL * 1000UL;
constexpr uint16_t QUEUE_MAX             = 40;
constexpr uint32_t MQTT_RETRY_MS         = 30000;
constexpr bool     SEND_ALARMS_NOW       = true;

const char* const  TIMEZONE              = "CET-1CEST,M3.5.0,M10.5.0/3";

enum class PumpBlock { None, NoWater, Fire, Cooldown };

struct Readings {
  bool  fire = false;
  bool  water = true;
  bool  waterRaw = true;
  float soilMv[3] = {0, 0, 0};
  float soilPct[3] = {0, 0, 0};
  float soilAvgPct = 0;
  bool  airOk = false;
  float airHumPct = 0;
  float airTempC = 0;
  float airPressHpa = 0;
  float lightLux = 0;
  bool  lightSaturated = false;
  float uvIndex = 0;
};

struct PumpState {
  bool      on = false;
  uint32_t  startedAt = 0;
  uint32_t  cooldownUntil = 0;
  bool      inCooldown = false;
  PumpBlock block = PumpBlock::None;
};

U8G2_SSD1306_64X48_ER_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Adafruit_BME280 bme;
bool     bmeFound = false;
uint32_t lastAirRead = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
bool     netEnabled = false;
bool     fsOk = false;
uint16_t queueCount = 0;
uint32_t lastMqttTry = 0;
bool     mqttTried = false;
uint32_t nextReportAt = 0;
uint32_t lastScreen = 0;
bool     haveReading = false;
char     clientId[24] = "doniczka";
char     topicData[48], topicStatus[48], topicAlarm[48];
const char* QUEUE_FILE = "/queue.jsonl";

Readings  rd;
PumpState pump;
uint32_t  lastRead = 0;
uint32_t  waterChangedAt = 0;
uint32_t  pageStartedAt = 0;
uint8_t   page = 0;

float mapf(float x, float inMin, float inMax, float outMin, float outMax);
float clampf(float x, float lo, float hi);
float readMilliVolts(uint8_t pin);
void setPumpOutput(bool on);
const char* uvLabel(float uvi);
void readSensors(uint32_t now);
void updatePump(uint32_t now);
void drawAlarm(const char* l1, const char* l2);
void drawScreen(uint32_t now);
void logSerial();
int readChipId(uint8_t addr);
void initBme();
const char* jnum(char* buf, size_t n, bool valid, float v, int decimals);
const char* pumpBlockName(PumpBlock b);
uint32_t unixTime();
void buildReport(char* out, size_t n, uint32_t now);
uint16_t queueReadAll(String* lines, uint16_t maxLines);
void queueWriteAll(String* lines, uint16_t from, uint16_t to);
void queueAppend(const char* json);
void queueFlush(uint32_t now);
void reportTask(uint32_t now);
void networkSetup();
void networkTask(uint32_t now);
void alarmTask();

void setup() {

  pinMode(PIN_PUMP, OUTPUT);
  setPumpOutput(false);

  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);
  if (HAS_FIRE) pinMode(PIN_FIRE, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);

  Serial.begin(115200);

  rd.waterRaw = rd.water = digitalRead(PIN_WATER_LEVEL) == WATER_PRESENT_LEVEL;

  Wire.begin();
  display.begin();
  if (HAS_AIR) initBme();

  fsOk = LittleFS.begin(true);
  if (fsOk) {
    static String lines[QUEUE_MAX + 1];
    queueCount = queueReadAll(lines, QUEUE_MAX + 1);
    Serial.printf("Kolejka: %u raport(y) czeka na wyslanie\n", queueCount);
  } else {
    Serial.println("LittleFS: blad - raporty nie beda przechowywane");
  }

  networkSetup();
  nextReportAt = millis() + FIRST_REPORT_DELAY_MS;
}

void loop() {
  uint32_t now = millis();

  if (!haveReading || now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;
    readSensors(now);
    updatePump(now);
    logSerial();
    haveReading = true;
  }

  if (now - lastScreen >= SCREEN_REFRESH_MS) {
    lastScreen = now;
    drawScreen(now);
  }

  reportTask(now);
  networkTask(now);
  alarmTask();
}

float mapf(float x, float inMin, float inMax, float outMin, float outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

float readMilliVolts(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < ADC_SAMPLES; i++) sum += analogReadMilliVolts(pin);
  return static_cast<float>(sum) / ADC_SAMPLES;
}

void setPumpOutput(bool on) {
  bool level = PUMP_ACTIVE_LOW ? !on : on;
  digitalWrite(PIN_PUMP, level ? HIGH : LOW);
}

const char* uvLabel(float uvi) {
  if (uvi < 3)  return "niski";
  if (uvi < 6)  return "umiark.";
  if (uvi < 8)  return "wysoki";
  if (uvi < 11) return "b.wysoki";
  return "ekstrem.";
}

void readSensors(uint32_t now) {
  rd.fire = HAS_FIRE && digitalRead(PIN_FIRE) == FIRE_ACTIVE_LEVEL;

  bool raw = digitalRead(PIN_WATER_LEVEL) == WATER_PRESENT_LEVEL;
  if (raw != rd.waterRaw) {
    rd.waterRaw = raw;
    waterChangedAt = now;
  }
  if (rd.water != rd.waterRaw && (now - waterChangedAt) >= WATER_DEBOUNCE_MS) {
    rd.water = rd.waterRaw;
  }

  float sum = 0;
  for (uint8_t i = 0; i < 3; i++) {
    rd.soilMv[i]  = readMilliVolts(PIN_SOIL[i]);
    rd.soilPct[i] = clampf(mapf(rd.soilMv[i], SOIL_MV_DRY[i], SOIL_MV_WET[i], 0, 100),
                           0, 100);
    sum += rd.soilPct[i];
  }
  rd.soilAvgPct = sum / 3.0f;

  if (HAS_AIR && bmeFound &&
      (lastAirRead == 0 || now - lastAirRead >= AIR_INTERVAL_MS)) {
    lastAirRead = now;
    if (bme.takeForcedMeasurement()) {
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      float p = bme.readPressure() / 100.0f;
      rd.airOk = !std::isnan(t) && !std::isnan(h) && !std::isnan(p);
      if (rd.airOk) {
        rd.airTempC = t;
        rd.airHumPct = h;
        rd.airPressHpa = p;
      }
    } else {
      rd.airOk = false;
    }
  }

  if (HAS_LIGHT) {
    float mv = readMilliVolts(PIN_LIGHT);
    rd.lightSaturated = mv >= LIGHT_MV_SATURATE;
    rd.lightLux = mv * LIGHT_LUX_PER_MV;
  }

  if (HAS_UV) {
    rd.uvIndex = readMilliVolts(PIN_UV) * UV_INDEX_PER_MV;
  }
}

void updatePump(uint32_t now) {
  if (pump.inCooldown && (int32_t)(now - pump.cooldownUntil) >= 0) {
    pump.inCooldown = false;
  }

  if (rd.fire)               pump.block = PumpBlock::Fire;
  else if (!rd.water)        pump.block = PumpBlock::NoWater;
  else if (pump.inCooldown)  pump.block = PumpBlock::Cooldown;
  else                       pump.block = PumpBlock::None;

  bool want = pump.on;
  if (rd.soilAvgPct < SOIL_PUMP_ON_PCT)        want = true;
  else if (rd.soilAvgPct >= SOIL_PUMP_OFF_PCT) want = false;

  if (pump.block != PumpBlock::None) want = false;

  if (pump.on && want && (now - pump.startedAt) >= PUMP_MAX_RUN_MS) {
    want = false;
    pump.inCooldown = true;
    pump.cooldownUntil = now + PUMP_COOLDOWN_MS;
    pump.block = PumpBlock::Cooldown;
  }

  if (want && !pump.on) pump.startedAt = now;
  pump.on = want;
  setPumpOutput(pump.on);
}

void drawAlarm(const char* l1, const char* l2) {
  display.setFont(u8g2_font_7x14B_tf);
  display.drawStr(0, 16, l1);
  display.drawStr(0, 34, l2);
  display.setFont(u8g2_font_5x8_tf);
  display.drawStr(0, 46, "Pompa STOP");
}

void drawScreen(uint32_t now) {
  display.clearBuffer();

  if (rd.fire) {
    drawAlarm("OGIEN!", "OGIEN!");
  } else if (pump.block == PumpBlock::NoWater) {
    drawAlarm("BRAK", "WODY!");
  } else {

    uint8_t kinds[6];
    uint8_t pages = 0;
    kinds[pages++] = 0;
    kinds[pages++] = 1;
    if (HAS_AIR)   kinds[pages++] = 2;
    if (HAS_LIGHT) kinds[pages++] = 3;
    if (HAS_UV)    kinds[pages++] = 4;
    kinds[pages++] = 5;

    if (now - pageStartedAt >= PAGE_INTERVAL_MS) {
      pageStartedAt = now;
      page = (page + 1) % pages;
    }
    if (page >= pages) page = 0;
    const uint8_t kind = kinds[page];

    char buf[16];
    display.setFont(u8g2_font_5x8_tf);
    switch (kind) {
      case 0:
        display.drawStr(0, 8, "Gleba sr.");
        display.setFont(u8g2_font_logisoso16_tf);
        snprintf(buf, sizeof(buf), "%.0f%%", rd.soilAvgPct);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        snprintf(buf, sizeof(buf), "%.0f %.0f %.0f",
                 rd.soilPct[0], rd.soilPct[1], rd.soilPct[2]);
        display.drawStr(0, 44, buf);
        break;
      case 1:
        display.drawStr(0, 8, "Woda: OK");
        display.drawStr(0, 20, "Pompa:");
        display.setFont(u8g2_font_7x14B_tf);
        display.drawStr(0, 36, pump.on ? "WLACZ." : "wylacz.");
        display.setFont(u8g2_font_5x8_tf);
        if (pump.block == PumpBlock::Cooldown) display.drawStr(0, 46, "(przerwa)");
        break;
      case 2:
        display.drawStr(0, 8, "Powietrze");
        if (!rd.airOk) {
          display.drawStr(0, 26, "BME280:");
          display.drawStr(0, 38, "brak odczytu");
          break;
        }
        snprintf(buf, sizeof(buf), "T: %.1fC", rd.airTempC);
        display.drawStr(0, 22, buf);
        snprintf(buf, sizeof(buf), "H: %.0f%%", rd.airHumPct);
        display.drawStr(0, 34, buf);
        snprintf(buf, sizeof(buf), "P: %.0fhPa", rd.airPressHpa);
        display.drawStr(0, 46, buf);
        break;
      case 5:
        display.drawStr(0, 8, "Siec");
        display.drawStr(0, 22, !netEnabled ? "WiFi: wyl."
                              : WiFi.status() == WL_CONNECTED ? "WiFi: ok" : "WiFi: brak");
        display.drawStr(0, 34, mqtt.connected() ? "MQTT: ok" : "MQTT: brak");
        snprintf(buf, sizeof(buf), "Kolejka: %u", queueCount);
        display.drawStr(0, 46, buf);
        break;
      case 3:
        display.drawStr(0, 8, "Swiatlo");
        display.setFont(u8g2_font_7x14B_tf);
        snprintf(buf, sizeof(buf), rd.lightSaturated ? ">%.0f" : "%.0f", rd.lightLux);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        display.drawStr(0, 44, "lux");
        break;
      case 4:
        display.drawStr(0, 8, "UV index");
        display.setFont(u8g2_font_7x14B_tf);
        snprintf(buf, sizeof(buf), "%.1f", rd.uvIndex);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        display.drawStr(0, 44, uvLabel(rd.uvIndex));
        break;
    }
  }
  display.sendBuffer();
}

void logSerial() {

  Serial.printf("soil mV=%.0f/%.0f/%.0f  %%=%.0f/%.0f/%.0f avg=%.0f | lvl=%d water=%d | "
                "pump=%d block=%d",
                rd.soilMv[0], rd.soilMv[1], rd.soilMv[2],
                rd.soilPct[0], rd.soilPct[1], rd.soilPct[2], rd.soilAvgPct,
                digitalRead(PIN_WATER_LEVEL), rd.water, pump.on,
                static_cast<int>(pump.block));
  if (HAS_FIRE)  Serial.printf(" fire=%d", rd.fire);
  if (HAS_AIR)   Serial.printf(" air_ok=%d t=%.1fC h=%.0f%% p=%.0fhPa",
                               rd.airOk, rd.airTempC, rd.airHumPct, rd.airPressHpa);
  if (HAS_LIGHT) Serial.printf(" lux=%.0f%s", rd.lightLux, rd.lightSaturated ? "(max)" : "");
  if (HAS_UV)    Serial.printf(" uvi=%.1f", rd.uvIndex);
  Serial.println();
}

int readChipId(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom((int)addr, 1) != 1) return -1;
  return Wire.read();
}

void initBme() {
  const uint8_t addrs[2] = {0x76, 0x77};
  for (uint8_t a : addrs) {
    int id = readChipId(a);
    if (id < 0) continue;
    if (id == 0x58) {
      Serial.printf("0x%02X: to BMP280, nie BME280 - nie mierzy wilgotnosci!\n", a);
      continue;
    }
    if (bme.begin(a, &Wire)) {

      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::FILTER_OFF);
      bmeFound = true;
      Serial.printf("BME280 znaleziony pod 0x%02X\n", a);
      return;
    }
    Serial.printf("0x%02X: uklad ID=0x%02X, nie udalo sie uruchomic\n", a, id);
  }
  Serial.println("BME280 nie znaleziony - sprawdz kable SDA/SCL i zasilanie 3.3V");
}

const char* jnum(char* buf, size_t n, bool valid, float v, int decimals) {
  if (!valid || std::isnan(v) || std::isinf(v)) snprintf(buf, n, "null");
  else snprintf(buf, n, "%.*f", decimals, v);
  return buf;
}

const char* pumpBlockName(PumpBlock b) {
  switch (b) {
    case PumpBlock::NoWater:  return "brak_wody";
    case PumpBlock::Fire:     return "ogien";
    case PumpBlock::Cooldown: return "przerwa";
    default:                  return "brak";
  }
}

uint32_t unixTime() {
  time_t t = time(nullptr);
  return t > 1700000000 ? static_cast<uint32_t>(t) : 0;
}

void buildReport(char* out, size_t n, uint32_t now) {
  char s0[12], s1[12], s2[12], avg[12], t[12], h[12], p[12], lux[12], uvi[12], ts[16];
  uint32_t ut = unixTime();
  if (ut) snprintf(ts, sizeof(ts), "%lu", (unsigned long)ut);
  else    snprintf(ts, sizeof(ts), "null");
  bool air = HAS_AIR && rd.airOk;
  snprintf(out, n,
    "{\"ts\":%s,\"uptime_s\":%lu,"
    "\"soil\":[%s,%s,%s],\"soil_avg\":%s,"
    "\"water\":%s,\"pump\":%s,\"pump_block\":\"%s\","
    "\"air_t\":%s,\"air_h\":%s,\"air_p\":%s,"
    "\"lux\":%s,\"lux_max\":%s,\"uvi\":%s,\"fire\":%s}",
    ts, (unsigned long)(now / 1000),
    jnum(s0, 12, true, rd.soilPct[0], 0), jnum(s1, 12, true, rd.soilPct[1], 0),
    jnum(s2, 12, true, rd.soilPct[2], 0), jnum(avg, 12, true, rd.soilAvgPct, 1),
    rd.water ? "true" : "false", pump.on ? "true" : "false", pumpBlockName(pump.block),
    jnum(t, 12, air, rd.airTempC, 1), jnum(h, 12, air, rd.airHumPct, 0),
    jnum(p, 12, air, rd.airPressHpa, 0),
    jnum(lux, 12, HAS_LIGHT, rd.lightLux, 0),
    HAS_LIGHT ? (rd.lightSaturated ? "true" : "false") : "null",
    jnum(uvi, 12, HAS_UV, rd.uvIndex, 1),
    HAS_FIRE ? (rd.fire ? "true" : "false") : "null");
}

uint16_t queueReadAll(String* lines, uint16_t maxLines) {
  uint16_t n = 0;
  File f = LittleFS.open(QUEUE_FILE, "r");
  if (!f) return 0;
  while (f.available() && n < maxLines) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) lines[n++] = line;
  }
  f.close();
  return n;
}

void queueWriteAll(String* lines, uint16_t from, uint16_t to) {
  if (from >= to) {
    LittleFS.remove(QUEUE_FILE);
    queueCount = 0;
    return;
  }
  File f = LittleFS.open(QUEUE_FILE, "w");
  if (!f) return;
  for (uint16_t i = from; i < to; i++) {
    f.print(lines[i]);
    f.print('\n');
  }
  f.close();
  queueCount = to - from;
}

void queueAppend(const char* json) {
  File f = LittleFS.open(QUEUE_FILE, "a");
  if (!f) {
    Serial.println("Kolejka: blad zapisu do flash");
    return;
  }
  f.print(json);
  f.print('\n');
  f.close();
  queueCount++;
  if (queueCount > QUEUE_MAX) {
    static String lines[QUEUE_MAX + 1];
    uint16_t n = queueReadAll(lines, QUEUE_MAX + 1);
    queueWriteAll(lines, n > QUEUE_MAX ? n - QUEUE_MAX : 0, n);
  }
}

void queueFlush(uint32_t now) {
  static uint32_t lastFail = 0;
  static bool failed = false;
  if (!fsOk || queueCount == 0 || !mqtt.connected()) return;
  if (failed && now - lastFail < MQTT_RETRY_MS) return;
  static String lines[QUEUE_MAX + 1];
  uint16_t n = queueReadAll(lines, QUEUE_MAX + 1);
  uint16_t sent = 0;
  while (sent < n && mqtt.publish(topicData, lines[sent].c_str(), true)) sent++;
  failed = sent < n;
  if (failed) lastFail = now;
  if (sent < n) Serial.printf("MQTT: wyslano %u z %u, reszta pozniej\n", sent, n);
  else          Serial.printf("MQTT: wyslano %u raport(y)\n", sent);
  queueWriteAll(lines, sent, n);
}

void reportTask(uint32_t now) {
  if (!haveReading || (int32_t)(now - nextReportAt) < 0) return;
  nextReportAt = now + REPORT_INTERVAL_MS;

  char json[400];
  buildReport(json, sizeof(json), now);
  Serial.printf("Raport: %s\n", json);

  if (fsOk) {
    queueAppend(json);
  } else if (mqtt.connected()) {
    mqtt.publish(topicData, json, true);
  } else {
    Serial.println("Raport utracony: brak flash i brak sieci");
  }
}

void networkSetup() {
  if (strlen(WIFI_SSID) == 0) {
    Serial.println("WiFi wylaczone (pusty WIFI_SSID w secrets.h)");
    return;
  }
  netEnabled = true;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(clientId, sizeof(clientId), "doniczka-%02x%02x%02x", mac[3], mac[4], mac[5]);
  snprintf(topicData,   sizeof(topicData),   "doniczkaAI/%02x%02x%02x/dane",   mac[3], mac[4], mac[5]);
  snprintf(topicStatus, sizeof(topicStatus), "doniczkaAI/%02x%02x%02x/status", mac[3], mac[4], mac[5]);
  snprintf(topicAlarm,  sizeof(topicAlarm),  "doniczkaAI/%02x%02x%02x/alarm",  mac[3], mac[4], mac[5]);
  Serial.printf("MQTT: klient %s, temat %s\n", clientId, topicData);

  configTzTime(TIMEZONE, "pool.ntp.org", "time.google.com");
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(30);
  mqtt.setSocketTimeout(3);
}

void networkTask(uint32_t now) {
  if (!netEnabled || WiFi.status() != WL_CONNECTED) return;

  if (!mqtt.connected()) {
    if (mqttTried && now - lastMqttTry < MQTT_RETRY_MS) return;
    mqttTried = true;
    lastMqttTry = now;
    const char* user = strlen(MQTT_USER) ? MQTT_USER : nullptr;
    const char* pass = strlen(MQTT_USER) ? MQTT_PASSWORD : nullptr;

    if (!mqtt.connect(clientId, user, pass, topicStatus, 0, true, "offline")) {
      Serial.printf("MQTT: brak polaczenia z %s:%d (kod %d)\n",
                    MQTT_HOST, MQTT_PORT, mqtt.state());
      return;
    }
    Serial.println("MQTT: polaczono");
    mqtt.publish(topicStatus, "online", true);
  }

  mqtt.loop();
  queueFlush(now);
}

void alarmTask() {
  static PumpBlock lastSent = PumpBlock::None;
  static bool sentOnce = false;
  if (!SEND_ALARMS_NOW || !mqtt.connected()) return;
  PumpBlock cur = (pump.block == PumpBlock::Cooldown) ? PumpBlock::None : pump.block;
  if (sentOnce && cur == lastSent) return;
  const char* msg = cur == PumpBlock::NoWater ? "brak_wody"
                  : cur == PumpBlock::Fire    ? "ogien" : "ok";
  if (mqtt.publish(topicAlarm, msg, true)) {
    lastSent = cur;
    sentOnce = true;
  }
}
