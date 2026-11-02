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

constexpr float SOIL_MV_VALID_MIN = 300.0f;
constexpr float SOIL_MV_VALID_MAX = 3100.0f;

constexpr float SOIL_PUMP_ON_PCT  = 30.0f;
constexpr float SOIL_PUMP_OFF_PCT = 60.0f;

constexpr uint32_t PUMP_MAX_RUN_MS  = 60UL * 1000UL;
constexpr uint32_t PUMP_COOLDOWN_MS = 5UL * 60UL * 1000UL;

constexpr uint32_t AIR_INTERVAL_MS = 1000;
constexpr uint32_t BME_RETRY_MS    = 60000;

constexpr float LIGHT_LUX_PER_MV  = 0.2f;

constexpr float LIGHT_MV_SATURATE = 3000.0f;

constexpr float UV_INDEX_PER_MV = 0.01f;
constexpr float UV_MV_SATURATE  = 3000.0f;

constexpr uint32_t READ_INTERVAL_MS = 60000;
constexpr uint32_t PAGE_INTERVAL_MS = 5000;
constexpr uint8_t  ADC_SAMPLES      = 16;

constexpr uint32_t SCREEN_REFRESH_MS = 1000;

constexpr uint32_t REPORT_INTERVAL_MS    = 6UL * 60UL * 60UL * 1000UL;
constexpr uint32_t FIRST_REPORT_DELAY_MS = 2UL * 60UL * 1000UL;
constexpr uint16_t QUEUE_MAX             = 40;
constexpr uint16_t EVENT_QUEUE_MAX       = 100;
constexpr uint32_t MQTT_RETRY_MS         = 30000;
constexpr bool     SEND_ALARMS_NOW       = true;

constexpr uint32_t SENSOR_STABLE_MS    = 30000;
constexpr uint32_t SENSOR_EVENT_GAP_MS = 5UL * 60UL * 1000UL;

const char* const  TIMEZONE              = "CET-1CEST,M3.5.0,M10.5.0/3";

enum class PumpBlock { None, NoWater, Fire, Cooldown, SensorError };

enum class SensorState : uint8_t { Ok, Disabled, NotFound, ReadFailed, OutOfRange, WireBroken };

enum SensorId : uint8_t { S_SOIL_TOP, S_SOIL_MIDDLE, S_SOIL_BOTTOM, S_AIR, S_LIGHT, S_UV, SENSOR_COUNT };

const char* const SENSOR_NAMES[SENSOR_COUNT] = {
  "soil_top", "soil_middle", "soil_bottom", "air", "lux", "uvi"
};

struct SensorHealth {
  SensorState state = SensorState::Ok;
  SensorState reported = SensorState::Ok;
  uint32_t    changedAt = 0;
  uint32_t    lastEventAt = 0;
  bool        evented = false;
};

struct Readings {
  bool  fire = false;
  bool  water = true;
  bool  waterRaw = true;
  float soilMv[3] = {0, 0, 0};
  float soilPct[3] = {0, 0, 0};
  float soilAvgPct = 0;
  uint8_t soilValid = 0;
  float airHumPct = 0;
  float airTempC = 0;
  float airPressHpa = 0;
  float lightLux = 0;
  float uvIndex = 0;
};

struct PumpState {
  bool      on = false;
  uint32_t  startedAt = 0;
  uint32_t  cooldownUntil = 0;
  bool      inCooldown = false;
  PumpBlock block = PumpBlock::None;
};

struct Queue {
  const char* file;
  uint16_t    max;
  const char* topic;
  const char* label;
  bool        retain;
  uint16_t    count = 0;
  bool        failed = false;
  uint32_t    lastFail = 0;
};

U8G2_SSD1306_64X48_ER_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Adafruit_BME280 bme;
bool     bmeFound = false;
uint32_t lastAirRead = 0;
uint32_t lastBmeTry = 0;

WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
bool     netEnabled = false;
bool     fsOk = false;
uint32_t lastMqttTry = 0;
bool     mqttTried = false;
uint32_t nextReportAt = 0;
uint32_t lastScreen = 0;
bool     haveReading = false;
char     clientId[24] = "doniczka";
char     topicData[48], topicStatus[48], topicAlarm[48], topicEvents[48];

Queue dataQ  {"/queue.jsonl",  QUEUE_MAX,       topicData,   "raport",    true};
Queue eventQ {"/events.jsonl", EVENT_QUEUE_MAX, topicEvents, "zdarzenie", false};

constexpr uint16_t QUEUE_LINES = (QUEUE_MAX > EVENT_QUEUE_MAX ? QUEUE_MAX : EVENT_QUEUE_MAX) + 1;
String queueLines[QUEUE_LINES];

Readings     rd;
PumpState    pump;
SensorHealth health[SENSOR_COUNT];
uint32_t     lastRead = 0;
uint32_t     waterChangedAt = 0;
uint32_t     pageStartedAt = 0;
uint8_t      page = 0;

float mapf(float x, float inMin, float inMax, float outMin, float outMax);
float clampf(float x, float lo, float hi);
float readMilliVolts(uint8_t pin);
void setPumpOutput(bool on);
const char* uvLabel(float uvi);
bool sensorOk(SensorId id);
void setSensorState(SensorId id, SensorState s, uint32_t now);
void initHealth();
void readSensors(uint32_t now);
void readAir(uint32_t now);
void updatePump(uint32_t now);
void drawAlarm(const char* l1, const char* l2);
void drawScreen(uint32_t now);
void logSerial();
int readChipId(uint8_t addr);
bool initBme(bool verbose);
const char* jnum(char* buf, size_t n, bool valid, float v, int decimals);
const char* pumpBlockName(PumpBlock b);
const char* sensorStateName(SensorState s);
uint32_t unixTime();
void jsonTime(char* ts, size_t n);
void buildReport(char* out, size_t n, uint32_t now);
void buildSensorEvent(char* out, size_t n, uint32_t now, SensorId id, SensorState s);
uint16_t queueReadAll(Queue& q, String* lines, uint16_t maxLines);
void queueWriteAll(Queue& q, String* lines, uint16_t from, uint16_t to);
void queueAppend(Queue& q, const char* json);
void queueFlush(Queue& q, uint32_t now);
void storeOrSend(Queue& q, const char* json);
void reportTask(uint32_t now);
void sensorEventsTask(uint32_t now);
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

  initHealth();

  Wire.begin();
  display.begin();
  if (HAS_AIR) initBme(true);

  fsOk = LittleFS.begin(true);
  if (fsOk) {
    dataQ.count  = queueReadAll(dataQ,  queueLines, QUEUE_LINES);
    eventQ.count = queueReadAll(eventQ, queueLines, QUEUE_LINES);
    Serial.printf("Kolejka: %u raport(y), %u zdarzen(ia) czeka na wyslanie\n",
                  dataQ.count, eventQ.count);
  } else {
    Serial.println("LittleFS: blad - raporty i zdarzenia nie beda przechowywane");
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

  sensorEventsTask(now);
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

bool sensorOk(SensorId id) {
  return health[id].state == SensorState::Ok;
}

void setSensorState(SensorId id, SensorState s, uint32_t now) {
  if (health[id].state != s) {
    health[id].state = s;
    health[id].changedAt = now;
  }
}

void initHealth() {
  const bool enabled[SENSOR_COUNT] = {true, true, true, HAS_AIR, HAS_LIGHT, HAS_UV};
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SensorState s = enabled[i] ? SensorState::Ok : SensorState::Disabled;
    health[i].state = s;
    health[i].reported = s;
  }
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
  rd.soilValid = 0;
  for (uint8_t i = 0; i < 3; i++) {
    SensorId id = static_cast<SensorId>(S_SOIL_TOP + i);
    rd.soilMv[i] = readMilliVolts(PIN_SOIL[i]);
    if (rd.soilMv[i] < SOIL_MV_VALID_MIN || rd.soilMv[i] > SOIL_MV_VALID_MAX) {
      setSensorState(id, SensorState::WireBroken, now);
      continue;
    }
    setSensorState(id, SensorState::Ok, now);
    rd.soilPct[i] = clampf(mapf(rd.soilMv[i], SOIL_MV_DRY[i], SOIL_MV_WET[i], 0, 100),
                           0, 100);
    sum += rd.soilPct[i];
    rd.soilValid++;
  }
  rd.soilAvgPct = rd.soilValid ? sum / rd.soilValid : 0;

  if (HAS_AIR) readAir(now);

  if (HAS_LIGHT) {
    float mv = readMilliVolts(PIN_LIGHT);
    if (mv >= LIGHT_MV_SATURATE) {
      setSensorState(S_LIGHT, SensorState::OutOfRange, now);
    } else {
      setSensorState(S_LIGHT, SensorState::Ok, now);
      rd.lightLux = mv * LIGHT_LUX_PER_MV;
    }
  }

  if (HAS_UV) {
    float mv = readMilliVolts(PIN_UV);
    if (mv >= UV_MV_SATURATE) {
      setSensorState(S_UV, SensorState::OutOfRange, now);
    } else {
      setSensorState(S_UV, SensorState::Ok, now);
      rd.uvIndex = mv * UV_INDEX_PER_MV;
    }
  }
}

void readAir(uint32_t now) {
  if (!bmeFound) {
    if (now - lastBmeTry >= BME_RETRY_MS) {
      lastBmeTry = now;
      initBme(false);
    }
    if (!bmeFound) {
      setSensorState(S_AIR, SensorState::NotFound, now);
      return;
    }
  }

  if (lastAirRead != 0 && now - lastAirRead < AIR_INTERVAL_MS) return;
  lastAirRead = now;

  if (!bme.takeForcedMeasurement()) {
    setSensorState(S_AIR, SensorState::ReadFailed, now);
    return;
  }
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  if (std::isnan(t) || std::isnan(h) || std::isnan(p)) {
    setSensorState(S_AIR, SensorState::ReadFailed, now);
    return;
  }
  if (t < -40 || t > 85 || h < 0 || h > 100 || p < 300 || p > 1100) {
    setSensorState(S_AIR, SensorState::OutOfRange, now);
    return;
  }
  setSensorState(S_AIR, SensorState::Ok, now);
  rd.airTempC = t;
  rd.airHumPct = h;
  rd.airPressHpa = p;
}

void updatePump(uint32_t now) {
  if (pump.inCooldown && (int32_t)(now - pump.cooldownUntil) >= 0) {
    pump.inCooldown = false;
  }

  if (rd.fire)                pump.block = PumpBlock::Fire;
  else if (!rd.water)         pump.block = PumpBlock::NoWater;
  else if (rd.soilValid == 0) pump.block = PumpBlock::SensorError;
  else if (pump.inCooldown)   pump.block = PumpBlock::Cooldown;
  else                        pump.block = PumpBlock::None;

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
  } else if (pump.block == PumpBlock::SensorError) {
    drawAlarm("BLAD", "GLEBY!");
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

    char buf[24];
    char v[3][8];
    display.setFont(u8g2_font_5x8_tf);
    switch (kind) {
      case 0:
        display.drawStr(0, 8, "Gleba sr.");
        display.setFont(u8g2_font_logisoso16_tf);
        snprintf(buf, sizeof(buf), "%.0f%%", rd.soilAvgPct);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        for (uint8_t i = 0; i < 3; i++) {
          if (sensorOk(static_cast<SensorId>(S_SOIL_TOP + i)))
            snprintf(v[i], sizeof(v[i]), "%.0f", rd.soilPct[i]);
          else
            snprintf(v[i], sizeof(v[i]), "--");
        }
        snprintf(buf, sizeof(buf), "%s %s %s", v[0], v[1], v[2]);
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
        if (!sensorOk(S_AIR)) {
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
        snprintf(buf, sizeof(buf), "Kol: %u/%u", dataQ.count, eventQ.count);
        display.drawStr(0, 46, buf);
        break;
      case 3:
        display.drawStr(0, 8, "Swiatlo");
        display.setFont(u8g2_font_7x14B_tf);
        if (sensorOk(S_LIGHT)) snprintf(buf, sizeof(buf), "%.0f", rd.lightLux);
        else snprintf(buf, sizeof(buf), ">%.0f", LIGHT_MV_SATURATE * LIGHT_LUX_PER_MV);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        display.drawStr(0, 44, "lux");
        break;
      case 4:
        display.drawStr(0, 8, "UV index");
        display.setFont(u8g2_font_7x14B_tf);
        if (sensorOk(S_UV)) snprintf(buf, sizeof(buf), "%.1f", rd.uvIndex);
        else snprintf(buf, sizeof(buf), "--");
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        if (sensorOk(S_UV)) display.drawStr(0, 44, uvLabel(rd.uvIndex));
        break;
    }
  }
  display.sendBuffer();
}

void logSerial() {

  Serial.printf("soil mV=%.0f/%.0f/%.0f  %%=%.0f/%.0f/%.0f avg=%.0f (%u ok) | lvl=%d water=%d | "
                "pump=%d block=%s",
                rd.soilMv[0], rd.soilMv[1], rd.soilMv[2],
                rd.soilPct[0], rd.soilPct[1], rd.soilPct[2], rd.soilAvgPct, rd.soilValid,
                digitalRead(PIN_WATER_LEVEL), rd.water, pump.on,
                pumpBlockName(pump.block));
  if (HAS_FIRE)  Serial.printf(" fire=%d", rd.fire);
  if (HAS_AIR)   Serial.printf(" air=%s t=%.1fC h=%.0f%% p=%.0fhPa",
                               sensorStateName(health[S_AIR].state),
                               rd.airTempC, rd.airHumPct, rd.airPressHpa);
  if (HAS_LIGHT) Serial.printf(" lux=%s/%.0f", sensorStateName(health[S_LIGHT].state), rd.lightLux);
  if (HAS_UV)    Serial.printf(" uvi=%s/%.1f", sensorStateName(health[S_UV].state), rd.uvIndex);
  Serial.println();
}

int readChipId(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom((int)addr, 1) != 1) return -1;
  return Wire.read();
}

bool initBme(bool verbose) {
  const uint8_t addrs[2] = {0x76, 0x77};
  for (uint8_t a : addrs) {
    int id = readChipId(a);
    if (id < 0) continue;
    if (id == 0x58) {
      if (verbose) Serial.printf("0x%02X: to BMP280, nie BME280 - nie mierzy wilgotnosci!\n", a);
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
      return true;
    }
    if (verbose) Serial.printf("0x%02X: uklad ID=0x%02X, nie udalo sie uruchomic\n", a, id);
  }
  if (verbose) Serial.println("BME280 nie znaleziony - sprawdz kable SDA/SCL i zasilanie 3.3V");
  return false;
}

const char* jnum(char* buf, size_t n, bool valid, float v, int decimals) {
  if (!valid || std::isnan(v) || std::isinf(v)) snprintf(buf, n, "null");
  else snprintf(buf, n, "%.*f", decimals, v);
  return buf;
}

const char* pumpBlockName(PumpBlock b) {
  switch (b) {
    case PumpBlock::NoWater:     return "NO_WATER";
    case PumpBlock::Fire:        return "FIRE";
    case PumpBlock::Cooldown:    return "COOLDOWN";
    case PumpBlock::SensorError: return "SENSOR_ERROR";
    default:                     return "NONE";
  }
}

const char* sensorStateName(SensorState s) {
  switch (s) {
    case SensorState::Ok:         return "OK";
    case SensorState::Disabled:   return "DISABLED";
    case SensorState::NotFound:   return "NOT_FOUND";
    case SensorState::ReadFailed: return "READ_FAILED";
    case SensorState::OutOfRange: return "OUT_OF_RANGE";
    case SensorState::WireBroken: return "WIRE_BROKEN";
  }
  return "UNKNOWN";
}

uint32_t unixTime() {
  time_t t = time(nullptr);
  return t > 1700000000 ? static_cast<uint32_t>(t) : 0;
}

void jsonTime(char* ts, size_t n) {
  uint32_t ut = unixTime();
  if (ut) snprintf(ts, n, "%lu", (unsigned long)ut);
  else    snprintf(ts, n, "null");
}

void buildReport(char* out, size_t n, uint32_t now) {
  char s0[12], s1[12], s2[12], t[12], h[12], p[12], lux[12], uvi[12], ts[16];
  jsonTime(ts, sizeof(ts));
  bool air = sensorOk(S_AIR);
  snprintf(out, n,
    "{\"ts\":%s,\"uptime_s\":%lu,"
    "\"soil\":{\"top\":%s,\"middle\":%s,\"bottom\":%s},"
    "\"pump_block\":\"%s\","
    "\"air_t\":%s,\"air_h\":%s,\"air_p\":%s,"
    "\"lux\":%s,\"uvi\":%s,\"fire\":%s}",
    ts, (unsigned long)(now / 1000),
    jnum(s0, 12, sensorOk(S_SOIL_TOP),    rd.soilPct[0], 0),
    jnum(s1, 12, sensorOk(S_SOIL_MIDDLE), rd.soilPct[1], 0),
    jnum(s2, 12, sensorOk(S_SOIL_BOTTOM), rd.soilPct[2], 0),
    pumpBlockName(pump.block),
    jnum(t, 12, air, rd.airTempC, 1), jnum(h, 12, air, rd.airHumPct, 0),
    jnum(p, 12, air, rd.airPressHpa, 0),
    jnum(lux, 12, sensorOk(S_LIGHT), rd.lightLux, 0),
    jnum(uvi, 12, sensorOk(S_UV), rd.uvIndex, 1),
    HAS_FIRE ? (rd.fire ? "true" : "false") : "null");
}

void buildSensorEvent(char* out, size_t n, uint32_t now, SensorId id, SensorState s) {
  char ts[16];
  jsonTime(ts, sizeof(ts));
  snprintf(out, n, "{\"ts\":%s,\"uptime_s\":%lu,\"sensor\":\"%s\",\"state\":\"%s\"}",
           ts, (unsigned long)(now / 1000), SENSOR_NAMES[id], sensorStateName(s));
}

uint16_t queueReadAll(Queue& q, String* lines, uint16_t maxLines) {
  uint16_t n = 0;
  File f = LittleFS.open(q.file, "r");
  if (!f) return 0;
  while (f.available() && n < maxLines) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) lines[n++] = line;
  }
  f.close();
  return n;
}

void queueWriteAll(Queue& q, String* lines, uint16_t from, uint16_t to) {
  if (from >= to) {
    LittleFS.remove(q.file);
    q.count = 0;
    return;
  }
  File f = LittleFS.open(q.file, "w");
  if (!f) return;
  for (uint16_t i = from; i < to; i++) {
    f.print(lines[i]);
    f.print('\n');
  }
  f.close();
  q.count = to - from;
}

void queueAppend(Queue& q, const char* json) {
  File f = LittleFS.open(q.file, "a");
  if (!f) {
    Serial.printf("Kolejka %s: blad zapisu do flash\n", q.file);
    return;
  }
  f.print(json);
  f.print('\n');
  f.close();
  q.count++;
  if (q.count > q.max) {
    uint16_t n = queueReadAll(q, queueLines, QUEUE_LINES);
    queueWriteAll(q, queueLines, n > q.max ? n - q.max : 0, n);
  }
}

void queueFlush(Queue& q, uint32_t now) {
  if (!fsOk || q.count == 0 || !mqtt.connected()) return;
  if (q.failed && now - q.lastFail < MQTT_RETRY_MS) return;
  uint16_t n = queueReadAll(q, queueLines, QUEUE_LINES);
  uint16_t sent = 0;
  while (sent < n && mqtt.publish(q.topic, queueLines[sent].c_str(), q.retain)) sent++;
  q.failed = sent < n;
  if (q.failed) q.lastFail = now;
  if (sent < n) Serial.printf("MQTT: %s - wyslano %u z %u, reszta pozniej\n", q.label, sent, n);
  else          Serial.printf("MQTT: %s - wyslano %u\n", q.label, sent);
  queueWriteAll(q, queueLines, sent, n);
}

void storeOrSend(Queue& q, const char* json) {
  if (fsOk) {
    queueAppend(q, json);
  } else if (mqtt.connected()) {
    mqtt.publish(q.topic, json, q.retain);
  } else {
    Serial.printf("Utracony %s: brak flash i brak sieci\n", q.label);
  }
}

void reportTask(uint32_t now) {
  if (!haveReading || (int32_t)(now - nextReportAt) < 0) return;
  nextReportAt = now + REPORT_INTERVAL_MS;

  char json[320];
  buildReport(json, sizeof(json), now);
  Serial.printf("Raport: %s\n", json);
  storeOrSend(dataQ, json);
}

void sensorEventsTask(uint32_t now) {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SensorHealth& h = health[i];
    if (h.state == h.reported) continue;
    if (now - h.changedAt < SENSOR_STABLE_MS) continue;
    if (h.evented && now - h.lastEventAt < SENSOR_EVENT_GAP_MS) continue;

    char json[160];
    buildSensorEvent(json, sizeof(json), now, static_cast<SensorId>(i), h.state);
    Serial.printf("Zdarzenie: %s\n", json);
    storeOrSend(eventQ, json);
    h.reported = h.state;
    h.lastEventAt = now;
    h.evented = true;
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
  snprintf(topicEvents, sizeof(topicEvents), "doniczkaAI/%02x%02x%02x/events", mac[3], mac[4], mac[5]);
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
  queueFlush(eventQ, now);
  queueFlush(dataQ, now);
}

void alarmTask() {
  static PumpBlock lastSent = PumpBlock::None;
  static bool sentOnce = false;
  if (!SEND_ALARMS_NOW || !mqtt.connected()) return;
  PumpBlock cur = (pump.block == PumpBlock::NoWater || pump.block == PumpBlock::Fire)
                  ? pump.block : PumpBlock::None;
  if (sentOnce && cur == lastSent) return;
  if (mqtt.publish(topicAlarm, pumpBlockName(cur), true)) {
    lastSent = cur;
    sentOnce = true;
  }
}
