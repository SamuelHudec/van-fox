# VanFox 🦊  – Portable Air Quality Monitor for Camper Vans

**VanFox** is a fully self-contained ESP32-C3-based air quality monitor designed for camper vans and mobile setups.
It combines multiple environmental sensors, a motion-activated OLED display, and gesture-based control — all running on battery power.

## 🧩 Components can be found in one store
- ~298czk [Board](https://www.laskakit.cz/laskakit-microesp/)
- ~768czk [CO2, temerature and humidity senzor](https://www.laskakit.cz/laskakit-scd41-senzor-co2--teploty-a-vlhkosti-vzduchu)
- ~368czk [VOx and NOx senzor](https://www.laskakit.cz/laskakit-sgp41-voc-a-nox-senzor-kvality-ovzdusi/)
- ~118czk [Battery](https://www.laskakit.cz/ehao-lipol-baterie-603048-900mah-3-7v/)
- ~128czk [Oled Display](https://www.laskakit.cz/laskakit-oled-displej-128x32-0-91--i--c/)
- ~198czk [Gyroscop](https://www.laskakit.cz/laskakit-bmi270-6-osy-gyroskop-a-akcelerometr/)
- ~6czk [Buzz](https://www.laskakit.cz/aktivni-bzucak-3-3v/)
- ~2czk [Switch](https://www.laskakit.cz/posuvny-prepinac-0-3a-50vdc/)
- 4x~12czk [uŠup](https://www.laskakit.cz/--sup--stemma-qt--qwiic-jst-sh-4-pin-kabel-5cm/)

## ⚙️ Operating Modes

### 🌿 Air Quality Mode (default)
Displays current environmental readings:
```
CO2: 412 ppm
T: 21.3°C  H: 45.7%
VOC: 102  NOx: 34
Bat: 87% (4.05V)
```
- OLED automatically turns off after 10 s of no motion.
- Device enters **light sleep** between sensor reads for power saving.

---

### 📏 Level Mode (Digital Spirit Level)
Activated by **triple shake gesture**:
```
LEVEL MODE
Pitch: 0.8°
Roll : -0.2°
-->Level<--
```
- Shows inclination (pitch & roll) in degrees.
- Automatically returns to **Air Mode** after 2 minutes.
- Uses continuous polling (no sleep) — ideal for calibration or debugging.
- USB remains active → can be flashed without reset.


## 🧭 Gesture Controls

| Gesture | Function |
|----------|-----------|
| **Triple shake** | Toggle between Air Mode ↔ Level Mode |
| **No motion for 10 s** | Display turns off (Air Mode only) |
| **Level Mode active for > 2 min** | Automatically returns to Air Mode |
