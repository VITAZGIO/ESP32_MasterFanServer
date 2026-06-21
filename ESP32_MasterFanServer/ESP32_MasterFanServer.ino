#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ============================================================
//  ESP32-C3 Server Fan Controller v2
//  - расширенная диагностика в монитор порта (раз в 10 сек)
//  - детальный лог подключения WiFi / MQTT по этапам
//  - раздельное + общее управление кулерами
//  - Power LED detect, RPM2, RPM CPU, температура HDD
// ============================================================

static const char* WIFI_SSID = "PONT228";
static const char* WIFI_PASS = "2284201337";

static const char* MQTT_HOST = "192.168.1.195";
static const uint16_t MQTT_PORT = 1883;

// ---------------- ПИНЫ ----------------
// --- Кулер 1 (впускной, снизу) ---
static const int FAN1_PWM_PIN   = 4;
static const int FAN1_TACH_PIN  = 2;
// --- Кулер 2 (выпускной, сзади) ---
static const int FAN2_PWM_PIN   = 3;
static const int FAN2_TACH_PIN  = 5;
// --- Тахометр башенного CPU-кулера ---
// >>> ПИН ТАХОМЕТРА БАШЕННОГО КУЛЕРА <<<
// Сейчас стоит GPIO1. Если CPU RPM показывает 0 (а кулер крутится),
// поменяй здесь цифру на 0 или 9 (другие свободные пины) и перезалей.
// НЕ ставь сюда 20 и 21 — это UART (монитор порта), будет мешать счёту.
static const int CPU_TACH_PIN   = 1;

static const int FAN_POWER_PIN  = 10;   // MOSFET -> реле (питание обоих кулеров)
static const int POWER_SW_PIN   = 7;    // кнопка Power через оптопару
static const int POWER_LED_PIN  = 8;    // вход: статус сервера (делитель 10k/10k)

static const int TEMP_SENSOR_PIN = 6;   // DS18B20 у HDD

// Отдельные PWM-каналы для каждого кулера
static const int PWM1_CHANNEL = 0;
static const int PWM2_CHANNEL = 1;
static const int PWM_FREQ = 25000;
static const int PWM_RESOLUTION = 8;

// Уровень 1 = тихо (минимум), уровень 10 = максимум.
// Значение = скважность ШИМ (duty cycle): доля времени, когда сигнал включён.
static const uint8_t FAN_SPEED_PWM[10] = {
  50, 80, 100, 128, 140, 160, 180, 200, 225, 255
};

// Кулеры РАЗНЫЕ модели и реагируют на ШИМ по-разному:
// FAN1 — прямой (уровень 1 = pwm минимальный),
// FAN2 — инверсный (нужно слать 255 - pwm, иначе крутит наоборот).
// Если вдруг какой-то кулер крутит наоборот — поменяй его флаг.
static const bool FAN1_INVERTED = false;
static const bool FAN2_INVERTED = true;

static int speedLevel1 = 1;   // уровень кулера 1
static int speedLevel2 = 1;   // уровень кулера 2
static bool fanEnabled = false;

// MOSFET/реле: включение = LOW, выключение = HIGH
static const int FAN_ON_LEVEL  = LOW;
static const int FAN_OFF_LEVEL = HIGH;

static const int POWER_SW_ACTIVE_LEVEL = HIGH;
static const int POWER_SW_IDLE_LEVEL   = LOW;
static const uint32_t POWER_SW_PULSE_MS = 500;

// Power LED detect: какой уровень на пине = сервер ВКЛЮЧЕН.
// Если делитель даёт высокий уровень когда сервер включён -> HIGH.
static const int SERVER_ON_LEVEL = HIGH;

// ---------------- ТОПИКИ ----------------
static const char* T_STATUS = "serverfan/status";

static const char* T_FAN_POWER_SET   = "serverfan/fan/power/set";
static const char* T_FAN_POWER_STATE = "serverfan/fan/power/state";

// общий топик скорости (ставит обоим)
static const char* T_FAN_SPEED_SET   = "serverfan/fan/speed/set";
static const char* T_FAN_SPEED_STATE = "serverfan/fan/speed/state";

// индивидуальные топики скорости
static const char* T_FAN1_SPEED_SET   = "serverfan/fan1/speed/set";
static const char* T_FAN1_SPEED_STATE = "serverfan/fan1/speed/state";
static const char* T_FAN2_SPEED_SET   = "serverfan/fan2/speed/set";
static const char* T_FAN2_SPEED_STATE = "serverfan/fan2/speed/state";

static const char* T_FAN1_PWM_STATE = "serverfan/fan1/pwm/state";
static const char* T_FAN2_PWM_STATE = "serverfan/fan2/pwm/state";

// обороты
static const char* T_FAN1_RPM_STATE = "serverfan/fan1/rpm/state";
static const char* T_FAN2_RPM_STATE = "serverfan/fan2/rpm/state";
static const char* T_CPU_RPM_STATE  = "serverfan/cpu/rpm/state";

// температура у HDD
static const char* T_TEMP_HDD_STATE = "serverfan/temp/hdd/state";

// статус сервера по Power LED
static const char* T_SERVER_STATE = "serverfan/server/state";

// кнопка Power
static const char* T_POWER_BUTTON_PRESS = "serverfan/server/power_button/press";

WiFiClient espClient;
PubSubClient mqtt(espClient);

OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);

// Тахометры
volatile uint32_t tach1Pulses = 0;
volatile uint32_t lastTach1Us = 0;
volatile uint32_t tach2Pulses = 0;
volatile uint32_t lastTach2Us = 0;
volatile uint32_t tachCpuPulses = 0;
volatile uint32_t lastTachCpuUs = 0;

static uint32_t lastRpmCalcMs = 0;
static uint32_t lastPublishMs = 0;
static uint32_t lastTempReadMs = 0;
static uint32_t lastDiagMs = 0;

static bool powerPulseActive = false;
static uint32_t powerPulseOffAt = 0;

static int currentRpm1 = 0;
static int currentRpm2 = 0;
static int currentRpmCpu = 0;
static float currentTemp = 0.0f;
static bool serverOn = false;

// ============================================================
//  ISR тахометров
// ============================================================
void IRAM_ATTR tach1ISR() {
  uint32_t now = micros();
  if (now - lastTach1Us > 1500) {
    tach1Pulses++;
    lastTach1Us = now;
  }
}
void IRAM_ATTR tach2ISR() {
  uint32_t now = micros();
  if (now - lastTach2Us > 1500) {
    tach2Pulses++;
    lastTach2Us = now;
  }
}
void IRAM_ATTR tachCpuISR() {
  uint32_t now = micros();
  if (now - lastTachCpuUs > 1500) {
    tachCpuPulses++;
    lastTachCpuUs = now;
  }
}

static int parseIntSafe(const char* s, int def = 0) {
  if (!s || !*s) return def;
  return atoi(s);
}

static void publishState();

// ============================================================
//  ВСПОМОГАТЕЛЬНОЕ: расшифровка состояния MQTT
// ============================================================
static const char* mqttStateStr(int s) {
  switch (s) {
    case -4: return "TIMEOUT (сервер не ответил вовремя)";
    case -3: return "CONNECTION_LOST (связь оборвалась)";
    case -2: return "CONNECT_FAILED (не удалось подключиться к брокеру)";
    case -1: return "DISCONNECTED (отключён)";
    case  0: return "CONNECTED (всё ок)";
    case  1: return "BAD_PROTOCOL (брокер не принял версию)";
    case  2: return "BAD_CLIENT_ID (неверный client id)";
    case  3: return "UNAVAILABLE (брокер недоступен)";
    case  4: return "BAD_CREDENTIALS (логин/пароль)";
    case  5: return "UNAUTHORIZED (нет прав)";
    default: return "UNKNOWN";
  }
}

static const char* wifiStatusStr(int s) {
  switch (s) {
    case WL_IDLE_STATUS:     return "IDLE (простой)";
    case WL_NO_SSID_AVAIL:   return "NO_SSID (сеть не найдена)";
    case WL_SCAN_COMPLETED:  return "SCAN_DONE";
    case WL_CONNECTED:       return "CONNECTED (подключено)";
    case WL_CONNECT_FAILED:  return "CONNECT_FAILED (ошибка, проверь пароль)";
    case WL_CONNECTION_LOST: return "CONNECTION_LOST (потеряно)";
    case WL_DISCONNECTED:    return "DISCONNECTED (отключено)";
    default:                 return "UNKNOWN";
  }
}

// ============================================================
//  ПРИМЕНЕНИЕ СОСТОЯНИЙ
// ============================================================
static void applyFanPower(bool on) {
  fanEnabled = on;
  digitalWrite(FAN_POWER_PIN, fanEnabled ? FAN_ON_LEVEL : FAN_OFF_LEVEL);
  Serial.print("[FAN] Питание кулеров (реле) = ");
  Serial.println(fanEnabled ? "ON" : "OFF");
  publishState();
}

static uint8_t levelToPwm1(int level) {
  level = constrain(level, 1, 10);
  uint8_t pwm = FAN_SPEED_PWM[level - 1];
  return FAN1_INVERTED ? (255 - pwm) : pwm;
}

static uint8_t levelToPwm2(int level) {
  level = constrain(level, 1, 10);
  uint8_t pwm = FAN_SPEED_PWM[level - 1];
  return FAN2_INVERTED ? (255 - pwm) : pwm;
}

static void applySpeed1(int level) {
  speedLevel1 = constrain(level, 1, 10);
  ledcWrite(PWM1_CHANNEL, levelToPwm1(speedLevel1));
  Serial.print("[FAN1] Уровень = ");
  Serial.print(speedLevel1);
  Serial.print("/10 | PWM = ");
  Serial.println(levelToPwm1(speedLevel1));
  publishState();
}

static void applySpeed2(int level) {
  speedLevel2 = constrain(level, 1, 10);
  ledcWrite(PWM2_CHANNEL, levelToPwm2(speedLevel2));
  Serial.print("[FAN2] Уровень = ");
  Serial.print(speedLevel2);
  Serial.print("/10 | PWM = ");
  Serial.println(levelToPwm2(speedLevel2));
  publishState();
}

// общий: ставит ОБОИМ кулерам один уровень
static void applySpeedBoth(int level) {
  level = constrain(level, 1, 10);
  Serial.print("[FAN ALL] Общий уровень для обоих = ");
  Serial.println(level);
  speedLevel1 = level;
  speedLevel2 = level;
  ledcWrite(PWM1_CHANNEL, levelToPwm1(level));
  ledcWrite(PWM2_CHANNEL, levelToPwm2(level));
  publishState();
}

static void pressPowerButton(uint32_t pulseMs = POWER_SW_PULSE_MS) {
  if (powerPulseActive) return;
  pulseMs = constrain(pulseMs, 100, 10000);
  Serial.print("[POWER] Нажатие кнопки, длительность мс = ");
  Serial.println(pulseMs);
  digitalWrite(POWER_SW_PIN, POWER_SW_ACTIVE_LEVEL);
  powerPulseActive = true;
  powerPulseOffAt = millis() + pulseMs;
}

// ============================================================
//  ПУБЛИКАЦИЯ СОСТОЯНИЯ В MQTT
// ============================================================
static void publishState() {
  if (!mqtt.connected()) return;
  char buf[32];

  mqtt.publish(T_FAN_POWER_STATE, fanEnabled ? "ON" : "OFF", true);

  // общий топик скорости показывает уровень кулера 1 (как опорный)
  snprintf(buf, sizeof(buf), "%d", speedLevel1);
  mqtt.publish(T_FAN_SPEED_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%d", speedLevel1);
  mqtt.publish(T_FAN1_SPEED_STATE, buf, true);
  snprintf(buf, sizeof(buf), "%d", speedLevel2);
  mqtt.publish(T_FAN2_SPEED_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%d", levelToPwm1(speedLevel1));
  mqtt.publish(T_FAN1_PWM_STATE, buf, true);
  snprintf(buf, sizeof(buf), "%d", levelToPwm2(speedLevel2));
  mqtt.publish(T_FAN2_PWM_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%d", currentRpm1);
  mqtt.publish(T_FAN1_RPM_STATE, buf, true);
  snprintf(buf, sizeof(buf), "%d", currentRpm2);
  mqtt.publish(T_FAN2_RPM_STATE, buf, true);
  snprintf(buf, sizeof(buf), "%d", currentRpmCpu);
  mqtt.publish(T_CPU_RPM_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%.1f", currentTemp);
  mqtt.publish(T_TEMP_HDD_STATE, buf, true);

  mqtt.publish(T_SERVER_STATE, serverOn ? "ON" : "OFF", true);
}

// ============================================================
//  ОБРАБОТКА ВХОДЯЩИХ MQTT
// ============================================================
static void mqttCallback(char* topic, byte* payload, unsigned int len) {
  static char msg[128];
  unsigned int n = min(len, sizeof(msg) - 1);
  memcpy(msg, payload, n);
  msg[n] = 0;

  Serial.print("[MQTT IN] ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(msg);

  if (!strcmp(topic, T_FAN_POWER_SET)) {
    if (!strcasecmp(msg, "ON")) applyFanPower(true);
    else if (!strcasecmp(msg, "OFF")) applyFanPower(false);
    else if (!strcasecmp(msg, "TOGGLE")) applyFanPower(!fanEnabled);
  }
  else if (!strcmp(topic, T_FAN_SPEED_SET)) {
    applySpeedBoth(parseIntSafe(msg, speedLevel1));
  }
  else if (!strcmp(topic, T_FAN1_SPEED_SET)) {
    applySpeed1(parseIntSafe(msg, speedLevel1));
  }
  else if (!strcmp(topic, T_FAN2_SPEED_SET)) {
    applySpeed2(parseIntSafe(msg, speedLevel2));
  }
  else if (!strcmp(topic, T_POWER_BUTTON_PRESS)) {
    if (!strcasecmp(msg, "PRESS") || !strcasecmp(msg, "ON") || !strcasecmp(msg, "PULSE")) {
      pressPowerButton(500);
    }
    else if (!strcasecmp(msg, "HOLD_5000")) {
      pressPowerButton(5000);
    }
    else if (!strncasecmp(msg, "HOLD_", 5)) {
      pressPowerButton(atoi(msg + 5));
    }
  }
}

// ============================================================
//  WIFI с подробным логом
// ============================================================
static uint32_t wifiConnectStartMs = 0;
static bool wifiWasConnected = false;

static void connectWifi() {
  Serial.println("------------------------------------------");
  Serial.print("[WiFi] Подключение к сети: ");
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiConnectStartMs = millis();
}

// ============================================================
//  MQTT с подробным логом
// ============================================================
static bool connectMqtt() {
  if (mqtt.connected()) return true;

  Serial.println("------------------------------------------");
  Serial.print("[MQTT] Подключение к брокеру ");
  Serial.print(MQTT_HOST);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);

  String cid = "serverfan-esp32c3-";
  cid += String((uint32_t)ESP.getEfuseMac(), HEX);
  Serial.print("[MQTT] Client ID = ");
  Serial.println(cid);

  if (!mqtt.connect(cid.c_str(), T_STATUS, 0, true, "offline")) {
    Serial.print("[MQTT] ОШИБКА подключения, state = ");
    Serial.print(mqtt.state());
    Serial.print(" -> ");
    Serial.println(mqttStateStr(mqtt.state()));
    return false;
  }

  Serial.println("[MQTT] Подключено успешно!");
  mqtt.publish(T_STATUS, "online", true);

  mqtt.subscribe(T_FAN_POWER_SET);
  mqtt.subscribe(T_FAN_SPEED_SET);
  mqtt.subscribe(T_FAN1_SPEED_SET);
  mqtt.subscribe(T_FAN2_SPEED_SET);
  mqtt.subscribe(T_POWER_BUTTON_PRESS);
  Serial.println("[MQTT] Подписка на топики управления выполнена");

  publishState();
  return true;
}

// ============================================================
//  ДИАГНОСТИКА (раз в 10 сек)
// ============================================================
static void printDiagnostics() {
  Serial.println();
  Serial.println("========== ДИАГНОСТИКА ==========");

  // WiFi
  int ws = WiFi.status();
  Serial.print("[WiFi] Статус: ");
  Serial.println(wifiStatusStr(ws));
  if (ws == WL_CONNECTED) {
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    Serial.print("[WiFi] Сигнал RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm  (ближе к 0 = лучше; -50 отлично, -80 слабо)");
  }

  // MQTT
  Serial.print("[MQTT] Связь: ");
  if (mqtt.connected()) {
    Serial.println("ПОДКЛЮЧЕНО");
  } else {
    Serial.print("НЕТ, state = ");
    Serial.print(mqtt.state());
    Serial.print(" -> ");
    Serial.println(mqttStateStr(mqtt.state()));
  }

  // Сервер
  Serial.print("[СЕРВЕР] Power LED: ");
  Serial.println(serverOn ? "ВКЛЮЧЕН (есть напруга)" : "ВЫКЛЮЧЕН (нет напруги)");

  // Кулеры
  Serial.print("[КУЛЕРА] Реле питания: ");
  Serial.println(fanEnabled ? "ON" : "OFF");
  Serial.print("[FAN1] уровень ");
  Serial.print(speedLevel1);
  Serial.print("/10, RPM = ");
  Serial.println(currentRpm1);
  Serial.print("[FAN2] уровень ");
  Serial.print(speedLevel2);
  Serial.print("/10, RPM = ");
  Serial.println(currentRpm2);
  Serial.print("[CPU FAN] RPM = ");
  Serial.println(currentRpmCpu);

  // Температура
  Serial.print("[ТЕМП HDD] ");
  if (currentTemp <= -100) Serial.println("ДАТЧИК НЕ НАЙДЕН!");
  else { Serial.print(currentTemp, 1); Serial.println(" C"); }

  Serial.print("[СИСТЕМА] Аптайм: ");
  Serial.print(millis() / 1000);
  Serial.print(" сек | Свободная память: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" байт");
  Serial.println("=================================");
  Serial.println();
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==================================================");
  Serial.println("  ESP32-C3 Server Fan Controller v2");
  Serial.println("==================================================");

  // Сразу безопасно выключаем реле кулеров
  digitalWrite(FAN_POWER_PIN, FAN_OFF_LEVEL);
  pinMode(FAN_POWER_PIN, OUTPUT);
  digitalWrite(FAN_POWER_PIN, FAN_OFF_LEVEL);

  pinMode(POWER_SW_PIN, OUTPUT);
  digitalWrite(POWER_SW_PIN, POWER_SW_IDLE_LEVEL);

  pinMode(POWER_LED_PIN, INPUT);

  pinMode(FAN1_TACH_PIN, INPUT_PULLUP);
  pinMode(FAN2_TACH_PIN, INPUT_PULLUP);
  pinMode(CPU_TACH_PIN, INPUT_PULLUP);

  // ШИМ кулеров
  ledcSetup(PWM1_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN1_PWM_PIN, PWM1_CHANNEL);
  ledcSetup(PWM2_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN2_PWM_PIN, PWM2_CHANNEL);
  Serial.println("[INIT] ШИМ кулеров настроен");

  attachInterrupt(digitalPinToInterrupt(FAN1_TACH_PIN), tach1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(FAN2_TACH_PIN), tach2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(CPU_TACH_PIN), tachCpuISR, FALLING);
  Serial.println("[INIT] Тахометры (3 шт) подключены");

  tempSensor.begin();
  Serial.println("[INIT] DS18B20 инициализирован");

  // Старт: оба кулера на уровень 1, питание включено
  applySpeedBoth(1);
  applyFanPower(true);

  connectWifi();

  lastRpmCalcMs = millis();
  lastDiagMs = millis();

  Serial.println("[INIT] ГОТОВ");
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  uint32_t now = millis();

  // --- WiFi ---
  if (WiFi.status() != WL_CONNECTED) {
    if (wifiWasConnected) {
      Serial.println("[WiFi] СВЯЗЬ ПОТЕРЯНА, переподключение...");
      wifiWasConnected = false;
    }
    static uint32_t lastWifiTry = 0;
    if (now - lastWifiTry > 5000) {
      lastWifiTry = now;
      Serial.print("[WiFi] Ещё не подключено (");
      Serial.print(wifiStatusStr(WiFi.status()));
      Serial.println("), пробую снова...");
      connectWifi();
    }
  } else {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.println("------------------------------------------");
      Serial.print("[WiFi] ПОДКЛЮЧЕНО! IP = ");
      Serial.print(WiFi.localIP());
      Serial.print(" | RSSI = ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");
    }
    connectMqtt();
    mqtt.loop();
  }

  // --- завершение импульса кнопки Power ---
  if (powerPulseActive && (int32_t)(now - powerPulseOffAt) >= 0) {
    powerPulseActive = false;
    digitalWrite(POWER_SW_PIN, POWER_SW_IDLE_LEVEL);
    Serial.println("[POWER] Кнопка отпущена");
  }

  // --- чтение Power LED (статус сервера) ---
  bool srv = (digitalRead(POWER_LED_PIN) == SERVER_ON_LEVEL);
  if (srv != serverOn) {
    serverOn = srv;
    Serial.print("[СЕРВЕР] Статус изменился: ");
    Serial.println(serverOn ? "ВКЛЮЧЕН" : "ВЫКЛЮЧЕН");
    publishState();
  }

  // --- температура ---
  if (now - lastTempReadMs >= 2000) {
    lastTempReadMs = now;
    tempSensor.requestTemperatures();
    float t = tempSensor.getTempCByIndex(0);
    if (t == DEVICE_DISCONNECTED_C) {
      currentTemp = -127.0f;
    } else {
      currentTemp = t;
    }
  }

  // --- расчёт RPM (раз в 2 сек) ---
  if (now - lastRpmCalcMs >= 2000) {
    uint32_t interval = now - lastRpmCalcMs;
    lastRpmCalcMs = now;

    noInterrupts();
    uint32_t p1 = tach1Pulses;   tach1Pulses = 0;
    uint32_t p2 = tach2Pulses;   tach2Pulses = 0;
    uint32_t pc = tachCpuPulses; tachCpuPulses = 0;
    interrupts();

    // 2 импульса на оборот -> делим на 2
    currentRpm1   = (int)((p1 * 60000UL) / (interval * 2UL));
    currentRpm2   = (int)((p2 * 60000UL) / (interval * 2UL));
    currentRpmCpu = (int)((pc * 60000UL) / (interval * 2UL));
  }

  // --- диагностика в монитор (раз в 10 сек) ---
  if (now - lastDiagMs >= 10000) {
    lastDiagMs = now;
    printDiagnostics();
  }

  // --- публикация состояния в MQTT (раз в 5 сек) ---
  if (mqtt.connected() && now - lastPublishMs >= 5000) {
    lastPublishMs = now;
    publishState();
  }

  delay(5);
}
