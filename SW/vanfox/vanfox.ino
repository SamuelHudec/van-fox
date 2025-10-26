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
#include "esp_wifi.h"
#include "esp_bt.h"

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
// ---------------------- BATTERY CONFIG ------------------------------
#define PIN_BAT_ADC 0
#define BAT_DIVIDER_RATIO 1.7693877551  // (1M + 1.3M)

float readBatteryVoltage() {
  return analogReadMilliVolts(PIN_BAT_ADC) * BAT_DIVIDER_RATIO / 1000.0;
}

int voltageToPercent(float voltage) {
  voltage = constrain(voltage, 3.0, 4.2);
  return (int)((voltage - 3.0) * 100 / (4.2 - 3.0));
}

// ===================================================================
// ---------------------- LEVEL OFFSETS -------------------------------
const float PITCH_OFFSET = -87.9;
const float ROLL_OFFSET  = 2;

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

uint32_t t_motion = 0;
uint32_t t_air    = 0;
uint32_t t_level  = 0;
uint32_t t_scd    = 0;
uint32_t t_sgp    = 0;

// ===================================================================
// ---------------------- SETUP --------------------------------------
void setup() {
  // --- Hard disable of WiFi and Bluetooth ---
  esp_wifi_stop();
  esp_wifi_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
  
  Serial.begin(115200);
  delay(1000);

  // --- I2C Init ---
  Wire.begin(8, 10);   // SDA=8, SCL=10
  Wire.setClock(50000);

  // --- OLED Init ---
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("❌ OLED not found!");
    while (true) delay(100);
  }
  display.setRotation(2);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.println("VanFox Air Monitor");
  display.println("Initializing...");
  display.display();

  // --- SCD41 Init ---
  scd4x.begin(Wire, 0x62);
  delay(500);
  scd4x.stopPeriodicMeasurement();
  scd4x.reinit();
  scd4x.startPeriodicMeasurement();

  // --- SGP41 Init ---
  sgp41.begin(Wire);
  delay(200);

  // --- BMI270 Init ---
  Serial.println("Initializing BMI270...");
  if (imu.beginI2C(0x68) != BMI2_OK)
    Serial.println("⚠️ BMI270 not detected!");
  else
    Serial.println("✅ BMI270 ready.");

  display.clearDisplay();
  display.println("Sensors ready!");
  display.display();
  delay(1000);
}

// ===================================================================
// ---------------------- LOOP --------------------------------------
void loop() {
  uint32_t now = millis();

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
  battery_v = readBatteryVoltage();
  battery_p = voltageToPercent(battery_v);

  // ============================================================
  // ------------------- Motion / Shake polling -----------------
  if (now - t_motion >= MOTION_PERIOD_MS) {
    t_motion = now;

    if (imu.getSensorData() == BMI2_OK) {
      static unsigned long lastShakeTime = 0;
      static int shakeCount = 0;
      static unsigned long lastMotionTime = millis();

      float accMag = sqrt(
        imu.data.accelX * imu.data.accelX +
        imu.data.accelY * imu.data.accelY +
        imu.data.accelZ * imu.data.accelZ
      );

      // --- Motion wakes display ---
      if (accMag > 1.05 && accMag < 3.0) {
        lastMotionTime = now;
        if (!displayOn && !gyroMode) {
          displayOn = true;
          Serial.println("💡 Motion → display ON");
        }
      }

      // --- Auto display off (only outside LEVEL MODE) ---
      if (!gyroMode && displayOn && (now - lastMotionTime > 10000)) {
        displayOn = false;
        display.clearDisplay();
        display.display();
        Serial.println("🌙 No motion → display OFF");
      }

      // --- TRIPLE SHAKE TOGGLE ---
      if (accMag > 1.30 && accMag < 3.0) {
        if (now - lastShakeTime < 800) shakeCount++;
        else shakeCount = 1;
        lastShakeTime = now;

        static uint32_t modeDebounce = 0;
        if (shakeCount >= 3 && (now - modeDebounce > 1200)) {
          shakeCount = 0;
          modeDebounce = now;

          gyroMode = !gyroMode;
          displayOn = true;
          lastModeSwitch = now;
          lastMotionTime = now;

          display.clearDisplay();
          display.setCursor(0, 10);
          display.println("Switching to");
          display.println(gyroMode ? "LEVEL MODE..." : "AIR MODE...");
          display.display();

          Serial.println(gyroMode ? "🎯 Triple shake → LEVEL MODE ON"
                                  : "📊 Triple shake → AIR MODE OFF");
          delay(1000);
        }
      }
    }
  }

  // ============================================================
  // ------------------- DISPLAY UPDATE -------------------------
  if (displayOn) {
    if (!gyroMode && (now - t_air >= AIR_DISPLAY_MS)) {
      t_air = now;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.printf("CO2: %.0f ppm\n", co2_ppm);
      display.printf("T: %.1fC  H: %.1f%%\n", temperature_c, humidity_rh);
      display.printf("VOC: %.0f  NOx: %.0f\n", voc_index, nox_index);
      display.printf("Bat: %d%% (%.2fV)", battery_p, battery_v);
      display.display();
    }

    if (gyroMode && (now - t_level >= LEVEL_DISPLAY_MS)) {
      t_level = now;
      imu.getSensorData();
      float pitch = atan2(imu.data.accelX,
                          sqrt(imu.data.accelY * imu.data.accelY + imu.data.accelZ * imu.data.accelZ))
                          * 180.0 / PI;
      float roll  = atan2(imu.data.accelY,
                          sqrt(imu.data.accelX * imu.data.accelX + imu.data.accelZ * imu.data.accelZ))
                          * 180.0 / PI;

      // --- Apply calibration offsets ---
      pitch += PITCH_OFFSET;
      roll  += ROLL_OFFSET;

      static float smoothPitch = pitch, smoothRoll = roll;
      smoothPitch = 0.7f * smoothPitch + 0.3f * pitch;
      smoothRoll  = 0.7f * smoothRoll  + 0.3f * roll;

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("LEVEL MODE");
      display.printf("Pitch: %.1f°\n", smoothPitch);
      display.printf("Roll : %.1f°\n", smoothRoll);
      display.println((abs(smoothPitch) < 1.0 && abs(smoothRoll) < 1.0)
                        ? "Level" : "Adjust");
      display.display();
    }
  }

  delay(100)
  //esp_sleep_enable_timer_wakeup(50 * 1000); // 5 ms
  //esp_light_sleep_start();
}