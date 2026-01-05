/*********************************************************************
 * VanFox – Portable Air Quality Monitor for Camper Vans
 * -----------------------------------------------------
 * Reads data from:
 *  - SCD41  (CO₂, Temperature, Humidity)
 *  - SGP41  (VOC, NOx)
 *  - BMI270 (Accelerometer, Gyroscope)
 *  - SSD1306 128x32 OLED Display
 *  - Battery voltage via ADC0 (LaskaKit MicroESP-C3 mini)
 *
 * Features:
 *  - Auto display sleep after inactivity
 *  - Triple-shake gesture to toggle LEVEL MODE
 *  - Level mode (digital spirit level)
 *  - Adaptive sensor/display update rates
 *********************************************************************/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SensirionI2cScd4x.h>
#include <SensirionI2CSgp41.h>
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "secrets.h"

// ===================================================================
// ---------------------- PINS CONFIG --------------------------------
#define SWITCH_PIN 2
#define BUZZ_PIN 3

// ===================================================================
// ---------------------- OLED CONFIG --------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_ADDR 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ===================================================================
// ---------------------- SENSOR OBJECTS -----------------------------
SensirionI2cScd4x scd4x;
SensirionI2CSgp41 sgp41;
VOCGasIndexAlgorithm vocAlgorithm;
NOxGasIndexAlgorithm noxAlgorithm;
BMI270 imu;

// ===================================================================
// ---------------------- WIFI & MQTT OBJECTS ------------------------
enum OperatingMode { MODE_TRAVELING, MODE_HOME };
OperatingMode currentMode = MODE_TRAVELING;
WiFiClient espClient;
PubSubClient mqtt(espClient);

// ===================================================================
// ---------------------- BATTERY CONFIG ------------------------------
#define PIN_BAT_ADC 0
#define BAT_DIVIDER_RATIO 1.7693877551  // (1M + 1.3M)

float readBatteryVoltage() {
  return analogReadMilliVolts(PIN_BAT_ADC) * BAT_DIVIDER_RATIO / 1000.0;
}

int voltageToPercent(float voltage) {
  voltage = constrain(voltage, 3.0, 4.0);
  return (int)((voltage - 3.0) * 100 / (4.0 - 3.0));
}

// ===================================================================
// ---------------------- LEVEL OFFSETS -------------------------------
const float ROLL_OFFSET  = 0.0;  // Calibration offset (adjust on level surface)

// ===================================================================
// ---------------------- GLOBAL VARIABLES ---------------------------
float co2_ppm = NAN, temperature_c = NAN, humidity_rh = NAN;
float voc_index = NAN, nox_index = NAN;
float battery_v = NAN;
int battery_p = 0;
bool charging = false;

bool displayOn = true;
bool gyroMode = false;
unsigned long lastModeSwitch = 0;

// ---------------------- TIMERS & PERIODS ---------------------------
const uint32_t MOTION_PERIOD_MS   = 100;    // IMU polling (shake + motion)
const uint32_t AIR_DISPLAY_MS     = 1000;   // redraw air screen
const uint32_t LEVEL_DISPLAY_MS   = 200;    // redraw level screen
const uint32_t AIR_SENSOR_READ_MS = 2000;   // SCD41 & SGP41 read interval
const uint32_t HOME_DISPLAY_TIMEOUT = 30000;       // 30s
const uint32_t TRAVELING_DISPLAY_TIMEOUT = 10000;  // 10s
const uint32_t HA_SEND_INTERVAL_MS = 300000;       // 5min
const uint32_t MQTT_RECONNECT_INTERVAL_MS = 60000; // 60s
const uint32_t DEEP_SLEEP_WAKEUP_SEC = 60;         // Wake every 60s to check switch

uint32_t t_motion = 0;
uint32_t t_air    = 0;
uint32_t t_level  = 0;
uint32_t t_air_sensor = 0;  // Replaces t_scd and t_sgp
uint32_t t_ha_send = 0;
uint32_t t_mqtt_reconnect = 0;

// ---------------------- ROBUST SHAKE DETECTION  ---------------------------
bool detectShake(float shakeIntensity) {
  static bool wasAbove = false;
  static unsigned long lastShake = 0;
  const float UPPER_THRESHOLD = 1.2;
  const float LOWER_THRESHOLD = 0.5;
  const int MIN_INTERVAL = 250;

  bool shakeDetected = false;

  if (!wasAbove && shakeIntensity > UPPER_THRESHOLD) {
    wasAbove = true;
  }

  if (wasAbove && shakeIntensity < LOWER_THRESHOLD) {
    wasAbove = false;
    unsigned long now = millis();
    if (now - lastShake > MIN_INTERVAL) {
      shakeDetected = true;
      lastShake = now;
    }
  }

  return shakeDetected;
}

// ===================================================================
// ---------------------- DEEP SLEEP FUNCTION ------------------------
void enterDeepSleep() {
  Serial.println("Switch OFF - entering deep sleep...");

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.println("Powering OFF...");
  display.println("Flip switch to wake");
  display.display();
  delay(2000);

  display.clearDisplay();
  display.display();

  // Configure timer wakeup: wake periodically to check switch state
  esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKEUP_SEC * 1000000ULL);

  Serial.printf("Entering deep sleep (%ds wakeup)...\n", DEEP_SLEEP_WAKEUP_SEC);
  delay(100);
  esp_deep_sleep_start();
}

// ===================================================================
// ---------------------- SETUP --------------------------------------
void setup() {
  // --- Configure mode switch FIRST (before any other init) ---
  pinMode(SWITCH_PIN, INPUT_PULLUP);

  // Check if we woke from deep sleep with timer wakeup
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    if (digitalRead(SWITCH_PIN) == LOW) {
      esp_sleep_enable_timer_wakeup(DEEP_SLEEP_WAKEUP_SEC * 1000000ULL);
      esp_deep_sleep_start();
    }
  }

  Serial.begin(115200);
  delay(1000);

  // --- Disable WiFi immediately to prevent auto-connect ---
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);

  // --- I2C Init ---
  Wire.begin(8, 10);   // SDA=8, SCL=10
  Wire.setClock(50000);

  // --- OLED Init ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found!");
    while (true) delay(100);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.println("VanFox Air Monitor");
  display.println("Initializing...");
  display.display();
  delay(1000);

  // ---------------- MODE SELECTION BASED ON SWITCH ----------------
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("VanFox Air Monitor");

  // Give WiFi hardware time to stabilize after boot
  delay(2000);

  bool switchOn = digitalRead(SWITCH_PIN) == HIGH;

  // If switch is OFF, enter deep sleep immediately (no sensor init needed)
  if (!switchOn) {
    display.println("Switch OFF - Power");
    display.println("saving mode");
    display.display();
    delay(1000);
    enterDeepSleep();
  }

  if (switchOn) {
    // Switch ON - Try HOME mode with WiFi
    Serial.println("Switch ON - attempting HOME mode");
    display.println("Switch: HOME Mode");
    display.println("Waiting 10s...");
    display.println("(router rate limit)");
    display.display();

    // Wait to let router rate-limit expire
    delay(10000);  // 10 second delay before WiFi attempt

    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Checking WiFi...");
    display.display();

    if (initWiFi()) {
      currentMode = MODE_HOME;
      display.println("WiFi Connected!");
      display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
      display.display();
      delay(1000);

      if (initHomeAssistantAPI()) {
        display.println("HA: Connected");
      } else {
        display.println("HA: Failed (retry)");
      }
      display.display();
      delay(2000);
    } else {
      currentMode = MODE_TRAVELING;
      display.println("WiFi Failed!");
      display.println("Running offline");
      display.display();
      delay(2000);
    }
  }

  // ---------------- SENSOR INITIALIZATION ----------------
  bool scd_ok = false, sgp_ok = false, imu_ok = false;

  // --- SCD41 Init ---
  Serial.println("Initializing SCD41...");
  scd4x.begin(Wire, 0x62);
  delay(500);
  scd4x.stopPeriodicMeasurement();
  if (scd4x.reinit() == 0) {
    scd4x.startPeriodicMeasurement();
    scd_ok = true;
    Serial.println("SCD41 ready.");
  } else {
    Serial.println("SCD41 not responding!");
  }

  // --- SGP41 Init ---
  Serial.println("Initializing SGP41...");
  sgp41.begin(Wire);
  delay(200);
  uint16_t rawVoc, rawNox;
  uint16_t dummyH = 0, dummyT = 0;
  if (sgp41.measureRawSignals(dummyH, dummyT, rawVoc, rawNox) == 0) {
    sgp_ok = true;
    Serial.println("SGP41 ready.");
  } else {
    Serial.println("SGP41 not responding!");
  }

  // --- BMI270 Init ---
  Serial.println("Initializing BMI270...");
  if (imu.beginI2C(0x68) == BMI2_OK) {
    imu_ok = true;
    Serial.println("BMI270 ready.");
  } else {
    Serial.println("BMI270 not detected!");
  }

  // ---------------- DISPLAY SENSOR STATUS ----------------
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Sensor check:");
  display.printf("SCD41: %s\n", scd_ok ? "OK" : "FAIL");
  display.printf("SGP41: %s\n", sgp_ok ? "OK" : "FAIL");
  display.printf("BMI270: %s\n", imu_ok ? "OK" : "FAIL");

  if (scd_ok && sgp_ok && imu_ok)
    display.println("All sensors ready!");
  else
    display.println("Some sensors missing!");

  display.display();
  delay(2000);
}

// ===================================================================
// ---------------------- LOOP --------------------------------------
void loop() {
  uint32_t now = millis();

  // ============================================================
  // ------------------- MODE SWITCH CHECK ----------------------
  static uint32_t lastSwitchCheck = 0;
  static bool lastSwitchState = HIGH;

  if (now - lastSwitchCheck >= 2000) {
    lastSwitchCheck = now;
    bool currentSwitchState = digitalRead(SWITCH_PIN) == HIGH;

    // Detect switch change
    if (currentSwitchState != lastSwitchState) {
      lastSwitchState = currentSwitchState;

      if (currentSwitchState) {
        Serial.println("Switch ON - switching to HOME mode");

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.println("Switching to");
        display.println("HOME MODE...");
        display.display();
        delay(1000);

        if (initWiFi()) {
          currentMode = MODE_HOME;
          initHomeAssistantAPI();
          Serial.println("HOME mode activated");
        } else {
          Serial.println("WiFi failed, staying TRAVELING");
        }

      } else {
        enterDeepSleep();
      }
    }
  }

  // --- Air Quality Sensors (SCD41 + SGP41) ---
  if (now - t_air_sensor >= AIR_SENSOR_READ_MS) {
    t_air_sensor = now;

    // Read SCD41 (CO2, Temperature, Humidity)
    bool dataReady = false;
    if (!scd4x.getDataReadyStatus(dataReady) && dataReady) {
      uint16_t co2Raw;
      float temp, hum;
      if (scd4x.readMeasurement(co2Raw, temp, hum) == 0) {
        co2_ppm = co2Raw;
        temperature_c = temp;
        humidity_rh = hum;
      }
    }

    // Read SGP41 (VOC, NOx) - uses temp/humidity from SCD41 for compensation
    uint16_t compHumidity = (uint16_t)((humidity_rh * 65535) / 100);
    uint16_t compTemperature = (uint16_t)(((temperature_c + 45) * 65535) / 175);
    uint16_t rawVoc = 0, rawNox = 0;
    if (sgp41.measureRawSignals(compHumidity, compTemperature, rawVoc, rawNox) == 0) {
      voc_index = vocAlgorithm.process(rawVoc);
      nox_index = noxAlgorithm.process(rawNox);
    }
  }

  // --- Battery ---
  static float smoothBatteryV = 0.0;
  battery_v = readBatteryVoltage();
  smoothBatteryV = 0.9f * smoothBatteryV + 0.1f * battery_v;
  battery_p = voltageToPercent(smoothBatteryV);

  // ============================================================
  // ------------------- MQTT HANDLING (HOME MODE) --------------
  if (currentMode == MODE_HOME) {
    if (mqtt.connected()) {
      mqtt.loop();
      if (now - t_ha_send >= HA_SEND_INTERVAL_MS) {
        t_ha_send = now;
        sendDataToHomeAssistant();
      }
    } else {
      if (now - t_mqtt_reconnect >= MQTT_RECONNECT_INTERVAL_MS) {
        t_mqtt_reconnect = now;
        Serial.println("Reconnecting to MQTT...");
        if (WiFi.status() != WL_CONNECTED) {
          Serial.println("WiFi disconnected - reconnecting...");
          WiFi.disconnect();
          WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

          unsigned long startAttempt = millis();
          while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 10000) {
            delay(500);
          }

          if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi reconnect failed");
            return;
          }
          Serial.println("WiFi reconnected!");
        }
        if (mqtt.connect("vanfox", "vanfox", "vanfox97411")) {
          Serial.println("MQTT reconnected");
          publishDiscoveryConfig();
        } else {
          Serial.print("MQTT reconnect failed, rc=");
          Serial.println(mqtt.state());
        }
      }
    }
  }

  // ============================================================
  // ------------------- Motion / Shake polling -----------------
  if (now - t_motion >= MOTION_PERIOD_MS) {
    t_motion = now;

    if (imu.getSensorData() == BMI2_OK) {
      float accMagnitude = sqrt(
        imu.data.accelX * imu.data.accelX +
        imu.data.accelY * imu.data.accelY +
        imu.data.accelZ * imu.data.accelZ
      );

        float gyroMagnitude = sqrt(
        imu.data.gyroX * imu.data.gyroX +
        imu.data.gyroY * imu.data.gyroY +
        imu.data.gyroZ * imu.data.gyroZ
      );

      float shakeIntensity = (accMagnitude - 1.0f) * 2.0f + (gyroMagnitude / 500.0f);
      Serial.printf("acc=%.2f g  gyro=%.1f °/s  shake=%.2f\n",
                accMagnitude, gyroMagnitude, shakeIntensity);

      // --- Display move detection ---
      static unsigned long lastMotionTime = millis();
      if (accMagnitude > 1.05 && accMagnitude < 3.0) {
        lastMotionTime = millis();
        if (!displayOn && !gyroMode) {
          displayOn = true;
          Serial.println("Motion detected → display ON");
        }
      }

      // Display timeout based on mode
      uint32_t displayTimeout = (currentMode == MODE_HOME) ?
                                HOME_DISPLAY_TIMEOUT : TRAVELING_DISPLAY_TIMEOUT;

      if (!gyroMode && displayOn && millis() - lastMotionTime > displayTimeout) {
        displayOn = false;
        display.clearDisplay();
        display.display();
        Serial.printf("No motion → display OFF (%s mode)\n",
                     currentMode == MODE_HOME ? "HOME" : "TRAVELING");
      }

      // --- Triple shake ---
      static int shakeCount = 0;
      static unsigned long lastShakeTime = 0;

      if (detectShake(shakeIntensity)) {
        shakeCount++;
        lastShakeTime = millis();
        Serial.printf("Shake %d detected\n", shakeCount);
      }

      if (millis() - lastShakeTime > 2000) shakeCount = 0;  // reset after 2 sec

      if (shakeCount >= 3) {
        shakeCount = 0;
        gyroMode = !gyroMode;
        displayOn = true;
        lastModeSwitch = millis();

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        if (gyroMode) {
          display.println("Switching to");
          display.println("LEVEL MODE...");
          Serial.println("Triple shake → LEVEL MODE ON");
        } else {
          display.println("Switching to");
          display.println("AIR MODE...");
          Serial.println("Triple shake → AIR MODE OFF");
        }
        display.display();
        delay(1500);
      }
    }
  }
  
  // ============================================================
  // --- Automatic exit from LEVEL MODE after 2 minutes ---
  if (gyroMode && (millis() - lastModeSwitch > 120000)) {  // 2 min = 120000 ms
    gyroMode = false;
    displayOn = true;
    Serial.println("LEVEL MODE timeout → back to AIR MODE");

    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.println("Timeout expired");
    display.println("--> AIR MODE");
    display.display();
    delay(1000);
  }

  // ============================================================
  // ------------------- DISPLAY UPDATE -------------------------
  if (displayOn) {
    if (!gyroMode && (now - t_air >= AIR_DISPLAY_MS)) {
      t_air = now;
      display.clearDisplay();
      display.setTextSize(1);
      display.setCursor(0, 0);
      display.printf("CO2: %.0f ppm\n", co2_ppm);
      display.printf("T: %.1fC  H: %.1f%%\n", temperature_c, humidity_rh);
      display.printf("VOC: %.0f  NOx: %.0f\n", voc_index, nox_index);

      // Status line shows mode and connection status
      if (currentMode == MODE_HOME) {
        if (mqtt.connected()) {
          display.print("Connected to HA");
        } else if (WiFi.status() == WL_CONNECTED) {
          display.print("Home MQTT ERR");
        } else {
          display.print("Home WiFi LOST");
        }
      } else {
        display.printf("Bat: %d%% %.2fV", battery_p, battery_v);
      }
      display.display();
    }

    if (gyroMode && (now - t_level >= LEVEL_DISPLAY_MS)) {
      t_level = now;
      imu.getSensorData();
      float roll = atan2(imu.data.accelY,
                         sqrt(imu.data.accelX * imu.data.accelX + imu.data.accelZ * imu.data.accelZ))
                   * 180.0 / PI;

      // Apply calibration offset
      roll += ROLL_OFFSET;
      static float smoothRoll = roll;
      smoothRoll = 0.7f * smoothRoll + 0.3f * roll;

      display.clearDisplay();
      display.setTextSize(2);
      display.setCursor(20, 10);
      display.printf("%.1f", smoothRoll);
      display.display();
    }
  }

  // ============================================================
  // ------------------- POWER MANAGEMENT ------------------------
  if (currentMode == MODE_HOME) {
    // HOME mode: No light sleep (WiFi needs to stay active)
    delay(50);
  } else {  // TRAVELING mode
    // Enable light sleep with 50ms timer wakeup for power savings
    esp_sleep_enable_timer_wakeup(50 * 1000);
    esp_light_sleep_start();
  }
}

// ===================================================================
// ---------------------- WIFI FUNCTIONS -----------------------------
bool initWiFi() {
  Serial.println("Attempting WiFi connection...");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(3000);  // Wait 1 second for router rate-limit to clear

  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);

  // Now start fresh connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(WIFI_PS_MIN_MODEM);
    Serial.println("\nWiFi connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    return true;
  }

  Serial.println("\nWiFi connection failed!");
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  return false;
}

// ===================================================================
// ---------------------- MQTT FUNCTIONS -----------------------------
bool initHomeAssistantAPI() {
  Serial.println("Connecting to MQTT broker...");
  Serial.print("Server: ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.println(MQTT_PORT);

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);

  int retries = 3;
  while (retries > 0 && !mqtt.connected()) {
    Serial.print("MQTT attempt ");
    Serial.print(4 - retries);
    Serial.print("... ");

    if (mqtt.connect("vanfox", MQTT_NAME, MQTT_PASSWORD)) {
      Serial.println("connected!");
      publishDiscoveryConfig();
      return true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.print(" - ");
      // Decode error code
      switch(mqtt.state()) {
        case -4: Serial.println("Connection timeout"); break;
        case -3: Serial.println("Connection lost"); break;
        case -2: Serial.println("Connect failed"); break;
        case -1: Serial.println("Disconnected"); break;
        case 1: Serial.println("Bad protocol"); break;
        case 2: Serial.println("Bad client ID"); break;
        case 3: Serial.println("Server unavailable"); break;
        case 4: Serial.println("Bad credentials"); break;
        case 5: Serial.println("Unauthorized"); break;
        default: Serial.println("Unknown error"); break;
      }
      retries--;
      delay(2000);
    }
  }

  Serial.println("MQTT connection failed - will retry in loop");
  return false;
}

void publishDiscoveryConfig() {
  Serial.println("Publishing MQTT discovery configs...");

  // CO2 sensor
  mqtt.publish("homeassistant/sensor/vanfox_co2/config",
    "{\"name\":\"VanFox CO2\","
    "\"stat_t\":\"homeassistant/sensor/vanfox/state\","
    "\"unit_of_meas\":\"ppm\","
    "\"val_tpl\":\"{{value_json.co2}}\","
    "\"dev_cla\":\"carbon_dioxide\","
    "\"dev\":{\"ids\":[\"vanfox_001\"],\"name\":\"VanFox\",\"mf\":\"DIY\",\"mdl\":\"Air Monitor\"}}", true);

  // Temperature
  mqtt.publish("homeassistant/sensor/vanfox_temp/config",
    "{\"name\":\"VanFox Temperature\","
    "\"stat_t\":\"homeassistant/sensor/vanfox/state\","
    "\"unit_of_meas\":\"°C\","
    "\"val_tpl\":\"{{value_json.temperature}}\","
    "\"dev_cla\":\"temperature\","
    "\"dev\":{\"ids\":[\"vanfox_001\"]}}", true);

  // Humidity
  mqtt.publish("homeassistant/sensor/vanfox_humidity/config",
    "{\"name\":\"VanFox Humidity\","
    "\"stat_t\":\"homeassistant/sensor/vanfox/state\","
    "\"unit_of_meas\":\"%\","
    "\"val_tpl\":\"{{value_json.humidity}}\","
    "\"dev_cla\":\"humidity\","
    "\"dev\":{\"ids\":[\"vanfox_001\"]}}", true);

  // VOC
  mqtt.publish("homeassistant/sensor/vanfox_voc/config",
    "{\"name\":\"VanFox VOC\","
    "\"stat_t\":\"homeassistant/sensor/vanfox/state\","
    "\"val_tpl\":\"{{value_json.voc}}\","
    "\"dev_cla\":\"aqi\","
    "\"dev\":{\"ids\":[\"vanfox_001\"]}}", true);

  // NOx
  mqtt.publish("homeassistant/sensor/vanfox_nox/config",
    "{\"name\":\"VanFox NOx\","
    "\"stat_t\":\"homeassistant/sensor/vanfox/state\","
    "\"val_tpl\":\"{{value_json.nox}}\","
    "\"dev_cla\":\"aqi\","
    "\"dev\":{\"ids\":[\"vanfox_001\"]}}", true);

  Serial.println("MQTT discovery configs published!");
}

void sendDataToHomeAssistant() {
  if (isnan(co2_ppm) || isnan(temperature_c)) {
    Serial.println("Sensor data not ready - skipping HA send");
    return;
  }

  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"co2\":%.0f,"
    "\"temperature\":%.1f,"
    "\"humidity\":%.1f,"
    "\"voc\":%.0f,"
    "\"nox\":%.0f}",
    co2_ppm, temperature_c, humidity_rh,
    voc_index, nox_index);

  if (mqtt.publish("homeassistant/sensor/vanfox/state", payload)) {
    Serial.print("Sent to HA: ");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish to MQTT");
  }
}