// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DHT.h>
#include <Adafruit_SCD30.h>
#include <DS1302.h>
#include <Adafruit_SSD1306.h>
#include <SD.h>
#include <SPI.h>
#include <ModbusMaster.h>
#include <ArduinoJson.h>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Определения пинов
#define DHTPIN 25            // DHT11: температура и влажность воздуха
#define DHTTYPE DHT11
#define LDR_PIN 36           // Фоторезистор: измерение освещённости
#define SOIL_MOISTURE_PIN 39 // Аналоговый датчик влажности почвы
#define WATER_LEVEL_PIN 34   // Датчик уровня воды в резервуаре
#define RELAY_LIGHT 26       // Реле для освещения
#define RELAY_PUMP 27        // Реле для насоса (полив)
#define RELAY_FAN 14         // Реле для вентилятора
#define RELAY_CO2 13         // Реле для подачи CO2
#define SD_CS_PIN 5          // CS пин для SD-карты
#define RS485_TX 17          // TX для RS485
#define RS485_RX 16          // RX для RS485
#define RS485_DE_RE 4        // Управление направлением RS485 (DE/RE)

// Объявление функций для RS485
void preTransmission();
void postTransmission();

// Структура зависимости между реле
struct Dependency {
  uint8_t sourcePin;   // пин-источник
  bool sourceState;    // состояние, при котором срабатывает зависимость
  uint8_t targetPin;   // пин-цель
  bool targetState;    // состояние, которое устанавливается на целевом пине
};

// Класс управления теплицей
class GreenhouseManager {
public:
  // Параметры сенсоров
  float temp = 0.0;            // температура воздуха
  float hum = 0.0;             // влажность воздуха
  float co2 = 0.0;             // концентрация CO2
  int lightVal = 0;            // значение освещённости
  int soilMoisture = 0;        // влажность почвы
  int waterLevel = 0;          // уровень воды
  float soil_pH = 0.0;         // pH почвы
  int soil_EC = 0, soil_N = 0, soil_P = 0, soil_K = 0; // EC и NPK

  // Пороговые значения и режим работы
  float tempThreshold = 30.0;      // порог температуры
  float humidityThreshold = 80.0;  // порог влажности воздуха
  int lightThreshold = 1000;       // порог освещённости
  int soilMoistureLow = 1500;      // порог влажности почвы
  int co2Threshold = 800;          // порог CO2
  bool autoMode = true;            // автоматический режим

  // Список зависимостей между реле
  std::vector<Dependency> deps;

  // Объекты для работы с аппаратурой
  DHT dht{DHTPIN, DHTTYPE};                // DHT11
  Adafruit_SCD30 scd30;                    // CO2 сенсор
  DS1302 rtc{21, 19, 18};               // часы реального времени
  Adafruit_SSD1306 display{128, 32, &Wire, -1}; // OLED дисплей
  ModbusMaster node;                       // Modbus RTU
  AsyncWebServer server{80};               // Веб-сервер

  // Инициализация всех компонентов
  void init() {
    Serial.begin(115200);
    dht.begin();                   // инициализация DHT11
    Wire.begin();                  // I2C шина

    // Настройка RS485
    pinMode(RS485_DE_RE, OUTPUT);
    Serial2.begin(4800, SERIAL_8N1, RS485_RX, RS485_TX);
    node.begin(1, Serial2);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // Часы RTC
    rtc.halt(false);
    rtc.writeProtect(false);

    // CO2 сенсор
    scd30.begin();
    scd30.setMeasurementInterval(2);

    // OLED дисплей
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Greenhouse Init");
    display.display();

    // Настройка реле
    pinMode(RELAY_LIGHT, OUTPUT);
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_FAN, OUTPUT);
    pinMode(RELAY_CO2, OUTPUT);
    digitalWrite(RELAY_LIGHT, LOW);
    digitalWrite(RELAY_PUMP, LOW);
    digitalWrite(RELAY_FAN, LOW);
    digitalWrite(RELAY_CO2, LOW);

    // Инициализация SD-карты
    if (!SD.begin(SD_CS_PIN)) {
      Serial.println("SD init failed");
    } else {
      // Создание заголовка CSV-файла
      File f = SD.open("log.csv", FILE_WRITE);
      if (f) {
        f.println("Time,Temp,Hum,CO2,Light,Soil,Water,pH,EC,N,P,K");
        f.close();
      }
    }

    // Точка доступа WiFi
    WiFi.softAP("Greenhouse_AP", "12345678");

    // Инициализация зависимостей между реле
    initDependencies();

    // Настройка веб-сервера
    setupWebServer();

    // Создание FreeRTOS задач для чтения и управления
    xTaskCreatePinnedToCore(taskReadSensors, "ReadTask", 4096, this, 1, NULL, 1);
    xTaskCreatePinnedToCore(taskControlLoop, "CtrlTask", 4096, this, 1, NULL, 1);
  }

private:
  // Задание зависимостей: если насос включён, включить свет
  void initDependencies() {
    deps.push_back({RELAY_PUMP, HIGH, RELAY_LIGHT, HIGH});
    // Можете добавить ещё правил здесь
  }

  // Настройка маршрутов веб-сервера
  void setupWebServer() {
    // Маршрут для получения данных в JSON
    server.on("/data", HTTP_GET, [this](AsyncWebServerRequest *req) {
      DynamicJsonDocument d(512);
      d["auto"] = autoMode;
      auto t = d.createNestedObject("thresholds");
      t["temp"] = tempThreshold;
      t["hum"] = humidityThreshold;
      t["light"] = lightThreshold;
      t["soil"] = soilMoistureLow;
      t["co2"] = co2Threshold;
      d["temp"] = temp;
      d["hum"] = hum;
      d["co2"] = co2;
      d["light"] = lightVal;
      d["soil"] = soilMoisture;
      d["water"] = waterLevel;
      d["pH"] = soil_pH;
      d["EC"] = soil_EC;
      d["N"] = soil_N;
      d["P"] = soil_P;
      d["K"] = soil_K;
      String out;
      serializeJson(d, out);
      req->send(200, "application/json", out);
    });
    // TODO: добавить /toggle, /set, /relay
    server.begin();
  }

  // Чтение всех сенсоров
  void readAll() {
    temp = dht.readTemperature();
    hum = dht.readHumidity();
    co2 = scd30.CO2;
    lightVal = analogRead(LDR_PIN);
    soilMoisture = analogRead(SOIL_MOISTURE_PIN);
    waterLevel = analogRead(WATER_LEVEL_PIN);
    if (node.readInputRegisters(0x05, 1) == node.ku8MBSuccess)
      soil_pH = node.getResponseBuffer(0) / 100.0;
    if (node.readInputRegisters(0x06, 1) == node.ku8MBSuccess)
      soil_EC = node.getResponseBuffer(0);
    if (node.readInputRegisters(0x07, 3) == node.ku8MBSuccess) {
      soil_N = node.getResponseBuffer(0);
      soil_P = node.getResponseBuffer(1);
      soil_K = node.getResponseBuffer(2);
    }
  }

  // Применение логики управления и зависимостей
  void applyControl() {
    if (autoMode) {
      digitalWrite(RELAY_FAN,
        (temp > tempThreshold || hum > humidityThreshold) ? HIGH : LOW);
      digitalWrite(RELAY_PUMP,
        (soilMoisture < soilMoistureLow) ? HIGH : LOW);
      digitalWrite(RELAY_CO2,
        (co2 < co2Threshold) ? HIGH : LOW);
      digitalWrite(RELAY_LIGHT,
        (lightVal < lightThreshold) ? HIGH : LOW);
    }
    // Обработка зависимостей
    for (auto &d : deps) {
      if (digitalRead(d.sourcePin) ==
          (d.sourceState ? HIGH : LOW)) {
        digitalWrite(d.targetPin,
          d.targetState ? HIGH : LOW);
      }
    }
  }

  // Логирование данных на SD-карту
  void logData() {
    File f = SD.open("log.csv", FILE_APPEND);
    if (f) {
      f.printf("%lu,%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%d,%d,%d,%d\n",
        millis(), temp, hum, co2,
        lightVal, soilMoisture, waterLevel,
        soil_pH, soil_EC, soil_N, soil_P, soil_K);
      f.close();
    }
  }

  // Задача FreeRTOS для чтения сенсоров
  static void taskReadSensors(void *pv) {
    auto *mgr = static_cast<GreenhouseManager*>(pv);
    for (;;) {
      mgr->readAll();
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }

  // Задача FreeRTOS для управления и логирования
  static void taskControlLoop(void *pv) {
    auto *mgr = static_cast<GreenhouseManager*>(pv);
    for (;;) {
      mgr->applyControl();
      mgr->logData();
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
};

// Глобальный экземпляр менеджера теплицы
GreenhouseManager GH;

// Определения для RS485
void preTransmission() { digitalWrite(RS485_DE_RE, HIGH); }
void postTransmission() { digitalWrite(RS485_DE_RE, LOW); }

void setup() {
  // Инициализация всей системы
  GH.init();
}

void loop() {
  // Пусто: задачи FreRTOS обрабатывают все события
}
