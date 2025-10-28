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
const float PITCH_OFFSET = -88.2;
const float ROLL_OFFSET  = 1.7;

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
    Serial.println("OLED not found!");
    while (true) delay(100);
  }
  display.setRotation(2);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.println("VanFox Air Monitor");
  display.println("Initializing...");
  display.display();

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
      if (!gyroMode && displayOn && millis() - lastMotionTime > 10000) {
        displayOn = false;
        display.clearDisplay();
        display.display();
        Serial.println("No motion → display OFF");
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
                        ? "--> Level <---" : "??? Adjust ???");
      display.display();
    }
  }

  // ============================================================
  // ------------------- POWER MANAGEMENT ------------------------
  if (!gyroMode) {
    esp_sleep_enable_timer_wakeup(50 * 1000); // 50 ms
    esp_light_sleep_start();
  } else {
    delay(200);
  }
}