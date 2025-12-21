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

// ===================================================================
// ---------------------- WIFI & MQTT CONFIG -------------------------
#define WIFI_SSID "id"
#define WIFI_PASSWORD "pass"
#define MQTT_SERVER "192.168.1.x"
#define MQTT_PORT 1883
#define RESTART_PIN 2
#define MQTT_NAME "vanfox"
#define MQTT_PASSWORD "vanfox97411"

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
  return (int)((voltage - 3.0) * 100 / (4.2 - 3.0));
}

// ===================================================================
// ---------------------- LEVEL OFFSETS -------------------------------
const float ROLL_OFFSET  = 0.5;  // Calibration offset for roll (adjust on level surface)

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
const uint32_t AIR_DISPLAY_MS     = 1000;  // redraw air screen
const uint32_t LEVEL_DISPLAY_MS   = 200;   // redraw level screen
const uint32_t SCD_READ_MS        = 2000;  // SCD41 read attempt
const uint32_t SGP_READ_MS        = 2000;  // SGP41 read attempt
const uint32_t HOME_DISPLAY_TIMEOUT = 30000;      // 30s
const uint32_t TRAVELING_DISPLAY_TIMEOUT = 10000;  // 10s
const uint32_t HA_SEND_INTERVAL_MS = 60000;  // 10 minutes
const uint32_t MQTT_RECONNECT_INTERVAL_MS = 60000;  // 60s

uint32_t t_motion = 0;
uint32_t t_air    = 0;
uint32_t t_level  = 0;
uint32_t t_scd    = 0;
uint32_t t_sgp    = 0;
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
// ---------------------- SETUP --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // --- Disable WiFi immediately to prevent auto-connect ---
  WiFi.persistent(false);  // Don't save credentials
  WiFi.mode(WIFI_OFF);     // Turn off WiFi completely
  delay(100);              // Let it fully shut down

  // --- Configure mode switch ---
  pinMode(RESTART_PIN, INPUT_PULLUP);

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

  // Check switch position
  bool switchOn = digitalRead(RESTART_PIN) == HIGH;

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
      display.printf("IP: %s", WiFi.localIP().toString().c_str());
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
      // WiFi failed, but switch says HOME - show error
      currentMode = MODE_TRAVELING;
      display.println("WiFi Failed!");
      display.println("Running offline");
      display.display();
      delay(2000);
    }
  } else {
    // Switch OFF - TRAVELING mode (no WiFi)
    Serial.println("Switch OFF - TRAVELING mode");
    currentMode = MODE_TRAVELING;
    display.println("Switch: TRAVEL Mode");
    display.println("WiFi disabled");
    display.display();
    delay(2000);

    // Disable WiFi/BT for power saving
    esp_wifi_stop();
    esp_wifi_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
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
  // jednoduchý test čítania
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

  if (now - lastSwitchCheck >= 2000) {  // Check every 2 seconds
    lastSwitchCheck = now;
    bool currentSwitchState = digitalRead(RESTART_PIN) == HIGH;

    // Detect switch change
    if (currentSwitchState != lastSwitchState) {
      lastSwitchState = currentSwitchState;

      if (currentSwitchState) {
        // Switch turned ON - switch to HOME mode
        Serial.println("Switch ON - switching to HOME mode");

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.println("Switching to");
        display.println("HOME MODE...");
        display.display();
        delay(1000);

        // Try to connect WiFi
        if (initWiFi()) {
          currentMode = MODE_HOME;
          initHomeAssistantAPI();
          Serial.println("HOME mode activated");
        } else {
          Serial.println("WiFi failed, staying TRAVELING");
        }

      } else {
        // Switch turned OFF - switch to TRAVELING mode
        Serial.println("Switch OFF - switching to TRAVELING mode");

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 10);
        display.println("Switching to");
        display.println("TRAVELING MODE...");
        display.display();
        delay(1000);

        // Disconnect WiFi and MQTT
        if (currentMode == MODE_HOME) {
          mqtt.disconnect();
          WiFi.disconnect(true);
          WiFi.mode(WIFI_OFF);
          esp_wifi_stop();
          esp_wifi_deinit();
        }

        currentMode = MODE_TRAVELING;
        Serial.println("TRAVELING mode activated");
      }
    }
  }

  // --- SCD41 (CO2, T, RH) ---
  if (now - t_scd >= SCD_READ_MS) {
    t_scd = now;
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
  }

  // --- SGP41 (VOC, NOx) ---
  if (now - t_sgp >= SGP_READ_MS) {
    t_sgp = now;
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

      // Send data periodically
      if (now - t_ha_send >= HA_SEND_INTERVAL_MS) {
        t_ha_send = now;
        sendDataToHomeAssistant();
      }
    } else {
      // Reconnect every 60s
      if (now - t_mqtt_reconnect >= MQTT_RECONNECT_INTERVAL_MS) {
        t_mqtt_reconnect = now;
        Serial.println("Reconnecting to MQTT...");

        // Check WiFi first
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
            return;  // Skip rest of loop
          }
          Serial.println("WiFi reconnected!");
        }

        // Then reconnect MQTT
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
      display.setTextSize(1);  // Reset text size for air quality display
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

      // Calculate roll (side-to-side tilt) only
      float roll = atan2(imu.data.accelY,
                         sqrt(imu.data.accelX * imu.data.accelX + imu.data.accelZ * imu.data.accelZ))
                   * 180.0 / PI;

      // Apply calibration offset
      roll += ROLL_OFFSET;

      // Smooth the reading to reduce jitter
      static float smoothRoll = roll;
      smoothRoll = 0.7f * smoothRoll + 0.3f * roll;

      // Display - just the roll angle
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
    if (!displayOn) {
      // Enable aggressive light sleep in TRAVELING mode
      //esp_sleep_enable_timer_wakeup(50000);  // 50ms
      //esp_light_sleep_start();
      delay(50);
    } else {
      delay(50);
    }
  }
}

// ===================================================================
// ---------------------- WIFI FUNCTIONS -----------------------------
bool initWiFi() {
  Serial.println("Attempting WiFi connection...");

  // Clean WiFi state before connecting (prevents stale connection issues)
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(3000);  // Wait 1 second for router rate-limit to clear

  // Disable auto-reconnect to prevent interference
  WiFi.setAutoReconnect(false);
  WiFi.persistent(false);  // Don't save credentials to flash

  // Now start fresh connection
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.setSleep(WIFI_PS_MIN_MODEM);  // Enable modem-sleep for power saving
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
  mqtt.setKeepAlive(60);  // Increased from 15 to 60 to prevent timeout with 10-min send interval

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
    "\"nox\":%.0f}",  // Fixed: removed trailing comma, added closing brace
    co2_ppm, temperature_c, humidity_rh,
    voc_index, nox_index);

  if (mqtt.publish("homeassistant/sensor/vanfox/state", payload)) {
    Serial.print("Sent to HA: ");
    Serial.println(payload);
  } else {
    Serial.println("Failed to publish to MQTT");
  }
}