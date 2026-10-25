/*
 * doniczkaAI - sterownik nawadniania na ESP32
 *
 * POSIADANY SPRZET:
 *   - 3x pojemnosciowy czujnik wilgotnosci gleby HW-390 (analog)
 *   - plywakowy czujnik poziomu wody (kontaktron, Botland SES-07244) - CYFROWY
 *   - Grove OLED 0.66" SSD1306, 64x48, I2C
 *   - analogowy czujnik swiatla (zalozony TEMT6000) -> luksy
 *   - analogowy czujnik UV (zalozony GUVA-S12SD)   -> UV index
 *   - BME280 (I2C): temperatura, wilgotnosc, cisnienie atmosferyczne
 *   - przekaznik pompy
 *
 * CZUJNIKI NA PRZYSZLOSC (wylaczone flagami HAS_*, wlaczysz gdy kupisz):
 *   - plomien (cyfrowy)
 *
 * PODLACZENIE:
 *   Plywak:  jeden przewod -> GPIO25, drugi -> GND (polaryzacja bez znaczenia).
 *   HW-390:  VCC -> 3.3V, GND -> GND, AOUT -> GPIO34 / 35 / 32.
 *   Swiatlo: VCC -> 3.3V, GND -> GND, SIG/OUT -> GPIO33.
 *   UV:      VCC -> 3.3V, GND -> GND, SIG/OUT -> GPIO36 (VP).
 *   OLED:    Grove: czerwony VCC -> 3.3V, czarny GND, zolty SCL -> GPIO22,
 *            bialy SDA -> GPIO21.
 *   BME280:  VCC/VIN -> 3.3V, GND -> GND, SCL -> GPIO22, SDA -> GPIO21
 *            (ta sama magistrala I2C co OLED - rownolegle, adresy sie nie gryza).
 *            Pinow CSB i SDO nie podlaczaj (SDO decyduje o adresie 0x76/0x77,
 *            kod sprawdza oba).
 *   Pompa:   Nie zadziala bezposrednio z pinu ESP tylko przez przekaznik/tranzystor,
 *            zasilanie pompy z osobnego zrodla, wspolna masa z ESP.
 *
 * Wszystkie progi sa w sekcji KONFIGURACJA.
 */

#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// ======================= KONFIGURACJA =======================

constexpr bool HAS_FIRE  = false;
constexpr bool HAS_AIR   = true;    // BME280
constexpr bool HAS_LIGHT = false;
constexpr bool HAS_UV    = false;

// --- Piny ---
// Wszystkie analogowe na ADC1 (GPIO32-39), dzialaja tez z wlaczonym WiFi.
// I2C (OLED + BME280): SDA=21, SCL=22.
constexpr uint8_t PIN_WATER_LEVEL = 25;  // kontaktron
constexpr uint8_t PIN_PUMP        = 26;  // przekaznik
constexpr uint8_t PIN_FIRE        = 27;
constexpr uint8_t PIN_SOIL[3]     = {34, 35, 32};
constexpr uint8_t PIN_LIGHT       = 33;
constexpr uint8_t PIN_UV          = 36;

// --- Kontaktron ---
// Pin z INPUT_PULLUP: kontaktron zwarty = LOW, rozwarty = HIGH.
// Czy "zwarty" oznacza "jest woda"  zalezy od tego, jak zostanie zamontowany plywak
constexpr int  WATER_PRESENT_LEVEL = LOW;
// Plywak na fali potrafi drgac - stan musi byc stabilny tyle ms.
constexpr uint32_t WATER_DEBOUNCE_MS = 2000;

// --- Plomien / przekaznik pomy ---
constexpr int  FIRE_ACTIVE_LEVEL = LOW;
constexpr bool PUMP_ACTIVE_LOW   = true;

// --- Gleba: kalibracja kazdego czujnika osobno (mV z Serial Monitor) ---
// HW-390 daje WYZSZE napiecie, gdy jest SUCHO.
// DRY = czujnik w powietrzu, WET = czujnik w szklance wody (do linii na plytce).
constexpr float SOIL_MV_DRY[3] = {2500.0f, 2500.0f, 2500.0f};  // TODO: zmierz
constexpr float SOIL_MV_WET[3] = {1100.0f, 1100.0f, 1100.0f};  // TODO: zmierz
// Histereza: pompa startuje ponizej ON, wylacza sie po osiagnieciu OFF.
constexpr float SOIL_PUMP_ON_PCT  = 30.0f;  // TODO: ustal
constexpr float SOIL_PUMP_OFF_PCT = 60.0f;  // TODO: ustal
// Zabezpieczenie przed zalaniem (czujnik wypadl z ziemi, padl itp.)
constexpr uint32_t PUMP_MAX_RUN_MS  = 60UL * 1000UL;
constexpr uint32_t PUMP_COOLDOWN_MS = 5UL * 60UL * 1000UL;

// --- BME280 ---
// Czytamy rzadziej niz reszte, bo czujnik sie nagrzewa przy czestych pomiarach
// i moze zawyzac temperature.
constexpr uint32_t AIR_INTERVAL_MS = 60000;

// --- Swiatlo -> luksy ---
// TEMT6000: rezystor 10k na module, 2 uA na 1 lx -> 200 lx na 1 V = 0.2 lx/mV.
// Inny czujnik = inny wspolczynnik. Zwykly fotorezystor (LDR) NIE da
// wiarygodnych luksow - tylko orientacyjnie.
constexpr float LIGHT_LUX_PER_MV  = 0.2f;
// Powyzej tego napiecia ADC ESP32 sie nasyca -> pokazujemy ">xxx lx".
// Przy 3.3 V TEMT6000 konczy sie ok. 600 lx (sloneczny dzien to 10 000+ lx).
constexpr float LIGHT_MV_SATURATE = 3000.0f;

// --- UV -> UV index ---
// GUVA-S12SD (typowy modul): UV index ~= Vout[mV] / 100 (przyblizenie).
constexpr float UV_INDEX_PER_MV = 0.01f;

// --- Czasy ---
constexpr uint32_t READ_INTERVAL_MS = 60000;
constexpr uint32_t PAGE_INTERVAL_MS = 5000;  // ekran jest maly - strony sie zmieniaja
constexpr uint8_t  ADC_SAMPLES      = 16;

// ===================== KONIEC KONFIGURACJI ===================

// Ekran 0.66" 64x48.

U8G2_SSD1306_64X48_ER_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);
Adafruit_BME280 bme;
bool     bmeFound = false;
uint32_t lastAirRead = 0;

struct Readings {
  bool  fire = false;
  bool  water = true;     // po debounce
  bool  waterRaw = true;  // surowy odczyt
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

enum class PumpBlock { None, NoWater, Fire, Cooldown };

struct PumpState {
  bool      on = false;
  uint32_t  startedAt = 0;
  uint32_t  cooldownUntil = 0;
  bool      inCooldown = false;
  PumpBlock block = PumpBlock::None;
};

Readings  rd;
PumpState pump;
uint32_t  lastRead = 0;
uint32_t  waterChangedAt = 0;
uint32_t  pageStartedAt = 0;
uint8_t   page = 0;

// ---------- pomocnicze ----------

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

// Skala UV
const char* uvLabel(float uvi) {
  if (uvi < 3)  return "niski";
  if (uvi < 6)  return "umiark.";
  if (uvi < 8)  return "wysoki";
  if (uvi < 11) return "b.wysoki";
  return "ekstrem.";
}

// ---------- odczyty ----------

void readSensors(uint32_t now) {
  rd.fire = HAS_FIRE && digitalRead(PIN_FIRE) == FIRE_ACTIVE_LEVEL;

  // Plywak z debounce, zmiana stanu przyjeta dopiero gdy trwa WATER_DEBOUNCE_MS.
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
      float p = bme.readPressure() / 100.0f;  // Pa -> hPa
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

// ---------- pompa ----------

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
  // pomiedzy progami: bez zmian (histereza)

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

// ---------- ekran 64x48 ----------

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
    // lista stron: gleba, pompa, [powietrze], [swiatlo], [UV]
    uint8_t kinds[5];
    uint8_t pages = 0;
    kinds[pages++] = 0;
    kinds[pages++] = 1;
    if (HAS_AIR)   kinds[pages++] = 2;
    if (HAS_LIGHT) kinds[pages++] = 3;
    if (HAS_UV)    kinds[pages++] = 4;

    if (now - pageStartedAt >= PAGE_INTERVAL_MS) {
      pageStartedAt = now;
      page = (page + 1) % pages;
    }
    if (page >= pages) page = 0;
    const uint8_t kind = kinds[page];

    char buf[16];
    display.setFont(u8g2_font_5x8_tf);
    switch (kind) {
      case 0:  // gleba
        display.drawStr(0, 8, "Gleba sr.");
        display.setFont(u8g2_font_logisoso16_tf);
        snprintf(buf, sizeof(buf), "%.0f%%", rd.soilAvgPct);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        snprintf(buf, sizeof(buf), "%.0f %.0f %.0f",
                 rd.soilPct[0], rd.soilPct[1], rd.soilPct[2]);
        display.drawStr(0, 44, buf);
        break;
      case 1:  // pompa / woda
        display.drawStr(0, 8, "Woda: OK");
        display.drawStr(0, 20, "Pompa:");
        display.setFont(u8g2_font_7x14B_tf);
        display.drawStr(0, 36, pump.on ? "WLACZ." : "wylacz.");
        display.setFont(u8g2_font_5x8_tf);
        if (pump.block == PumpBlock::Cooldown) display.drawStr(0, 46, "(przerwa)");
        break;
      case 2:  // powietrze
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
      case 3:  // swiatlo
        display.drawStr(0, 8, "Swiatlo");
        display.setFont(u8g2_font_7x14B_tf);
        snprintf(buf, sizeof(buf), rd.lightSaturated ? ">%.0f" : "%.0f", rd.lightLux);
        display.drawStr(0, 30, buf);
        display.setFont(u8g2_font_5x8_tf);
        display.drawStr(0, 44, "lux");
        break;
      case 4:  // UV
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
  // Surowe mV gleby sa potrzebne do kalibracji SOIL_MV_DRY / SOIL_MV_WET.
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

// ---------- BME280 ----------

// Czyta ID ukladu: 0x60 = BME280, 0x58 = BMP280 (bez wilgotnosci!).
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
      // Tryb wymuszony: 1 pomiar na zadanie, potem sen -> mniej samonagrzewania.
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X1,   // temperatura
                      Adafruit_BME280::SAMPLING_X1,   // cisnienie
                      Adafruit_BME280::SAMPLING_X1,   // wilgotnosc
                      Adafruit_BME280::FILTER_OFF);
      bmeFound = true;
      Serial.printf("BME280 znaleziony pod 0x%02X\n", a);
      return;
    }
    Serial.printf("0x%02X: uklad ID=0x%02X, nie udalo sie uruchomic\n", a, id);
  }
  Serial.println("BME280 nie znaleziony - sprawdz kable SDA/SCL i zasilanie 3.3V");
}

// ---------- setup / loop ----------

void setup() {
  // Pompa wylaczona od razu po ustawieniu pinu (rdzen ESP32 3.x ignoruje
  // digitalWrite przed pinMode, wiec kolejnosc musi byc taka).
  pinMode(PIN_PUMP, OUTPUT);
  setPumpOutput(false);

  pinMode(PIN_WATER_LEVEL, INPUT_PULLUP);
  if (HAS_FIRE) pinMode(PIN_FIRE, INPUT_PULLUP);
  analogSetAttenuation(ADC_11db);  // zakres ~0-3.1 V

  Serial.begin(115200);

  // Startujemy z odczytu plywaka, zeby nie czekac na debounce po wlaczeniu.
  rd.waterRaw = rd.water = digitalRead(PIN_WATER_LEVEL) == WATER_PRESENT_LEVEL;

  Wire.begin();  // SDA=21, SCL=22
  display.begin();
  if (HAS_AIR) initBme();
}

void loop() {
  uint32_t now = millis();
  if (now - lastRead < READ_INTERVAL_MS) return;
  lastRead = now;

  readSensors(now);
  updatePump(now);
  drawScreen(now);
  logSerial();
}
