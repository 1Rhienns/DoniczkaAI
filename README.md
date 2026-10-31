# doniczkaAI

Sterownik nawadniania na ESP32: 3 czujniki wilgotności gleby, pływak w zbiorniku, BME280, ekran OLED 64×48, pompa przez przekaźnik, raporty przez MQTT 4 razy na dobę.

## Sprzęt

- 3× pojemnościowy czujnik wilgotności gleby HW-390 (analogowy)
- pływakowy czujnik poziomu wody z kontaktronem, Botland SES-07244 (cyfrowy)
- Grove OLED 0.66" SSD1306, 64×48, I2C
- BME280: temperatura, wilgotność, ciśnienie (I2C)
- przekaźnik pompy
- opcjonalnie (flagi `HAS_*`): czujnik światła (założony TEMT6000), UV (założony GUVA-S12SD), płomień

## Podłączenie

| Element | Połączenie |
|---|---|
| Pływak | jeden przewód → GPIO25, drugi → GND (polaryzacja bez znaczenia) |
| HW-390 ×3 | VCC → 3.3V, GND → GND, AOUT → GPIO34 / GPIO35 / GPIO32 |
| OLED (Grove) | czerwony → 3.3V, czarny → GND, żółty SCL → GPIO22, biały SDA → GPIO21 |
| BME280 | VIN → 3.3V, GND → GND, SCL → GPIO22, SDA → GPIO21 (równolegle z OLED). CSB i SDO niepodłączone |
| Przekaźnik pompy | IN → GPIO26. Pompa **nigdy** bezpośrednio z pinu ESP; osobne zasilanie pompy, wspólna masa z ESP |
| Światło (opcja) | SIG → GPIO33 |
| UV (opcja) | SIG → GPIO36 (VP) |
| Płomień (opcja) | DO → GPIO27 |

Wszystkie wejścia analogowe są na ADC1 (GPIO32–39), więc działają przy włączonym WiFi.

## Pierwsze uruchomienie

1. Skopiuj `src/secrets.example.h` jako `src/secrets.h` i uzupełnij WiFi (tylko 2.4 GHz) oraz broker. `secrets.h` jest w `.gitignore`.
2. Kalibracja gleby: otwórz Serial Monitor (115200), odczytaj `soil mV` dla czujnika w powietrzu (`SOIL_MV_DRY`) i w szklance wody do linii na płytce (`SOIL_MV_WET`). HW-390 daje **wyższe** napięcie, gdy jest **sucho**.
3. Pływak: podnieś go palcem i sprawdź `lvl=`. Jeśli przy pełnym zbiorniku pokazuje „BRAK WODY”, odwróć pływak na trzpieniu albo zmień `WATER_PRESENT_LEVEL`.

## Ustawienia (sekcja konfiguracji w `main.cpp`)

| Ustawienie | Znaczenie |
|---|---|
| `HAS_FIRE`, `HAS_AIR`, `HAS_LIGHT`, `HAS_UV` | które czujniki są podłączone |
| `WATER_PRESENT_LEVEL` | stan pinu pływaka oznaczający „jest woda” (pin z INPUT_PULLUP: zwarty = LOW) |
| `WATER_DEBOUNCE_MS` | jak długo stan pływaka musi być stały (fala od pompy) |
| `PUMP_ACTIVE_LOW` | większość modułów przekaźników włącza się stanem LOW |
| `SOIL_PUMP_ON_PCT` / `SOIL_PUMP_OFF_PCT` | histereza: pompa startuje poniżej ON, staje po osiągnięciu OFF |
| `PUMP_MAX_RUN_MS` / `PUMP_COOLDOWN_MS` | maks. ciągła praca pompy i przerwa po niej (ochrona przed zalaniem) |
| `READ_INTERVAL_MS` | co ile czytane są czujniki i sterowana pompa (zalecane 500–1000 ms) |
| `AIR_INTERVAL_MS` | co ile pomiar BME280 (zalecane 60000 – częste pomiary nagrzewają czujnik) |
| `LIGHT_LUX_PER_MV` | TEMT6000: 0.2 lx/mV; zasięg przy 3.3 V ok. 600 lx |
| `UV_INDEX_PER_MV` | GUVA-S12SD: UV index ≈ mV / 100 |
| `REPORT_INTERVAL_MS` | raport co 6 h = 4 razy na dobę |
| `QUEUE_MAX` | ile raportów trzymać bez sieci (40 = 10 dni) |
| `MQTT_RETRY_MS` | co ile próba połączenia z brokerem |

## MQTT

`XXXXXX` = końcówka adresu MAC płytki (wypisywana na Serial Monitor przy starcie).

| Temat | Treść |
|---|---|
| `doniczkaAI/XXXXXX/dane` | raport JSON co 6 h (retain) |
| `doniczkaAI/XXXXXX/status` | `online` / `offline` (offline publikuje broker, gdy ESP zniknie) |
| `doniczkaAI/XXXXXX/alarm` | `brak_wody` / `ogien` / `ok`, od razu przy zmianie |

Bez sieci raporty czekają w pamięci flash i wychodzą, gdy połączenie wróci. Sterowanie pompą nie zależy od sieci.
