#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

static const char* WIFI_SSID = "PONT228";
static const char* WIFI_PASS = "2284201337";

static const char* MQTT_HOST = "192.168.1.195";
static const uint16_t MQTT_PORT = 1883;

static const int FAN_PWM_PIN   = 4;
static const int FAN_TACH_PIN  = 2;
static const int FAN_POWER_PIN = 10;
static const int POWER_SW_PIN  = 8;

static const int PWM_CHANNEL = 0;
static const int PWM_FREQ = 25000;
static const int PWM_RESOLUTION = 8;

static const uint8_t FAN_SPEED_PWM[10] = {
  225, 200, 180, 160, 140, 128, 100, 80, 50, 0
};

static int currentSpeedLevel = 1;
static bool fanEnabled = false;

// ТВОЙ MOSFET: включение = LOW, выключение = HIGH
static const int FAN_ON_LEVEL  = HIGH;
static const int FAN_OFF_LEVEL = LOW;

static const int POWER_SW_ACTIVE_LEVEL = HIGH;
static const int POWER_SW_IDLE_LEVEL   = LOW;
static const uint32_t POWER_SW_PULSE_MS = 500;

static const char* T_STATUS = "serverfan/status";

static const char* T_FAN_POWER_SET   = "serverfan/fan/power/set";
static const char* T_FAN_POWER_STATE = "serverfan/fan/power/state";

static const char* T_FAN_SPEED_SET   = "serverfan/fan/speed/set";
static const char* T_FAN_SPEED_STATE = "serverfan/fan/speed/state";

static const char* T_FAN_PWM_STATE = "serverfan/fan/pwm/state";
static const char* T_FAN_RPM_STATE = "serverfan/fan/rpm/state";

static const char* T_POWER_BUTTON_PRESS = "serverfan/server/power_button/press";

WiFiClient espClient;
PubSubClient mqtt(espClient);

volatile uint32_t tachPulses = 0;
volatile uint32_t lastTachUs = 0;

static uint32_t lastRpmCalcMs = 0;
static uint32_t lastPublishMs = 0;

static bool powerPulseActive = false;
static uint32_t powerPulseOffAt = 0;

static int currentRpm = 0;

void IRAM_ATTR tachISR() {
  uint32_t now = micros();

  // фильтр помех тахометра
  if (now - lastTachUs > 1500) {
    tachPulses++;
    lastTachUs = now;
  }
}

static int parseIntSafe(const char* s, int def = 0) {
  if (!s || !*s) return def;
  return atoi(s);
}

static void publishState();

static void applyFanPower(bool on) {
  fanEnabled = on;
  digitalWrite(FAN_POWER_PIN, fanEnabled ? FAN_ON_LEVEL : FAN_OFF_LEVEL);

  Serial.print("Fan power = ");
  Serial.println(fanEnabled ? "ON" : "OFF");

  publishState();
}

static void applySpeedLevel(int level) {
  level = constrain(level, 1, 10);
  currentSpeedLevel = level;

  uint8_t pwm = FAN_SPEED_PWM[currentSpeedLevel - 1];
  ledcWrite(PWM_CHANNEL, pwm);

  Serial.print("Fan speed level = ");
  Serial.print(currentSpeedLevel);
  Serial.print(" / 10 | PWM = ");
  Serial.println(pwm);

  publishState();
}

static void pressPowerButton(uint32_t pulseMs = POWER_SW_PULSE_MS) {
  if (powerPulseActive) return;

  pulseMs = constrain(pulseMs, 100, 10000);

  Serial.print("Power SW pulse ms = ");
  Serial.println(pulseMs);

  digitalWrite(POWER_SW_PIN, POWER_SW_ACTIVE_LEVEL);
  powerPulseActive = true;
  powerPulseOffAt = millis() + pulseMs;
}

static void publishState() {
  if (!mqtt.connected()) return;

  char buf[32];

  mqtt.publish(T_FAN_POWER_STATE, fanEnabled ? "ON" : "OFF", true);

  snprintf(buf, sizeof(buf), "%d", currentSpeedLevel);
  mqtt.publish(T_FAN_SPEED_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%d", FAN_SPEED_PWM[currentSpeedLevel - 1]);
  mqtt.publish(T_FAN_PWM_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%d", currentRpm);
  mqtt.publish(T_FAN_RPM_STATE, buf, true);
}

static void mqttCallback(char* topic, byte* payload, unsigned int len) {
  static char msg[128];
  unsigned int n = min(len, sizeof(msg) - 1);
  memcpy(msg, payload, n);
  msg[n] = 0;

  Serial.print("MQTT ");
  Serial.print(topic);
  Serial.print(" = ");
  Serial.println(msg);

  if (!strcmp(topic, T_FAN_POWER_SET)) {
    if (!strcasecmp(msg, "ON")) applyFanPower(true);
    else if (!strcasecmp(msg, "OFF")) applyFanPower(false);
    else if (!strcasecmp(msg, "TOGGLE")) applyFanPower(!fanEnabled);
  }

  else if (!strcmp(topic, T_FAN_SPEED_SET)) {
    int level = parseIntSafe(msg, currentSpeedLevel);
    applySpeedLevel(level);
  }

  else if (!strcmp(topic, T_POWER_BUTTON_PRESS)) {
    if (!strcasecmp(msg, "PRESS") || !strcasecmp(msg, "ON") || !strcasecmp(msg, "PULSE")) {
      pressPowerButton(500);
    }
    else if (!strcasecmp(msg, "HOLD_5000")) {
      pressPowerButton(5000);
    }
    else if (!strncasecmp(msg, "HOLD_", 5)) {
      uint32_t ms = atoi(msg + 5);
      pressPowerButton(ms);
    }
  }
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static bool connectMqtt() {
  if (mqtt.connected()) return true;

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setBufferSize(1024);

  String cid = "serverfan-esp32c3-";
  cid += String((uint32_t)ESP.getEfuseMac(), HEX);

  if (!mqtt.connect(cid.c_str(), T_STATUS, 0, true, "offline")) {
    return false;
  }

  mqtt.publish(T_STATUS, "online", true);

  mqtt.subscribe(T_FAN_POWER_SET);
  mqtt.subscribe(T_FAN_SPEED_SET);
  mqtt.subscribe(T_POWER_BUTTON_PRESS);

  publishState();

  Serial.println("MQTT connected");
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("ESP32-C3 Server Fan Controller");

  // Сразу безопасно выключаем MOSFET
  digitalWrite(FAN_POWER_PIN, FAN_OFF_LEVEL);
  pinMode(FAN_POWER_PIN, OUTPUT);
  digitalWrite(FAN_POWER_PIN, FAN_OFF_LEVEL);

  pinMode(POWER_SW_PIN, OUTPUT);
  digitalWrite(POWER_SW_PIN, POWER_SW_IDLE_LEVEL);

  pinMode(FAN_TACH_PIN, INPUT_PULLUP);

  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FAN_PWM_PIN, PWM_CHANNEL);
  Serial.println("PWM attach OK");

  attachInterrupt(digitalPinToInterrupt(FAN_TACH_PIN), tachISR, FALLING);

  // Старт: минимальная скорость, кулер выключен
  applySpeedLevel(1);
  applyFanPower(false);

  connectWifi();

  lastRpmCalcMs = millis();

  Serial.println("READY");
}

void loop() {
  uint32_t now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    static uint32_t lastWifiTry = 0;
    if (now - lastWifiTry > 3000) {
      lastWifiTry = now;
      Serial.println("WiFi reconnect...");
      connectWifi();
    }
  } else {
    connectMqtt();
    mqtt.loop();
  }

  if (powerPulseActive && (int32_t)(now - powerPulseOffAt) >= 0) {
    powerPulseActive = false;
    digitalWrite(POWER_SW_PIN, POWER_SW_IDLE_LEVEL);
    Serial.println("Power SW released");
  }

  if (now - lastRpmCalcMs >= 2000) {
    uint32_t interval = now - lastRpmCalcMs;
    lastRpmCalcMs = now;

    noInterrupts();
    uint32_t pulses = tachPulses;
    tachPulses = 0;
    interrupts();

    currentRpm = (int)((pulses * 60000UL) / (interval * 2UL));

    Serial.print("RPM = ");
    Serial.print(currentRpm);
    Serial.print(" | speed = ");
    Serial.print(currentSpeedLevel);
    Serial.print(" | fan = ");
    Serial.println(fanEnabled ? "ON" : "OFF");
  }

  if (mqtt.connected() && now - lastPublishMs >= 5000) {
    lastPublishMs = now;
    publishState();
  }

  delay(5);
}


