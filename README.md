# VanFox 🦊  – Portable Air Quality Monitor for Camper Vans

![preview](doc/3.jpeg)

**VanFox** is a fully self-contained ESP32-C3-based air quality monitor designed for camper vans and Home Assistant.
It combines multiple environmental sensors, a motion-activated OLED display and gesture-based control.

VanFox is designed to be open, hackable and power-efficient, with a strong focus on real-world camper van usage. Thanks to the built-in battery, it can operate without external power — during off-grid camping, power outages, or when taken outside.

As a camper, I like when one device serves more than one purpose. VanFox can travel with you in your van, and when you are home, it can also be used as a stationary air quality monitor for your household.

## Assembly Guide

1. Purchase the components
	- ~298czk [Board](https://www.laskakit.cz/laskakit-microesp/)
	- ~768czk [CO2, temerature and humidity senzor](https://www.laskakit.cz/laskakit-scd41-senzor-co2--teploty-a-vlhkosti-vzduchu)
	- ~368czk [VOx and NOx senzor](https://www.laskakit.cz/laskakit-sgp41-voc-a-nox-senzor-kvality-ovzdusi/)
	- ~118czk [Battery](https://www.laskakit.cz/ehao-lipol-baterie-603048-900mah-3-7v/)
	- ~128czk [Oled Display](https://www.laskakit.cz/laskakit-oled-displej-128x32-0-91--i--c/)
	- ~198czk [Gyroscop](https://www.laskakit.cz/laskakit-bmi270-6-osy-gyroskop-a-akcelerometr/)
	- ~6czk [Buzz](https://www.laskakit.cz/aktivni-bzucak-3-3v/)
	- ~4czk [Switch](https://www.laskakit.cz/posuvny-prepinac-0-5a-50vdc/)
	- 4x~12czk [uŠup](https://www.laskakit.cz/--sup--stemma-qt--qwiic-jst-sh-4-pin-kabel-5cm/)
	- M2x10 screws, I bougth a box and trimmed as needed

2. 3D print the enclosure from [3D](3D).
3. Solder the buzzer, switch, I²C cables, and battery to the ESP board. For I²C and battery, the pin placement is fixed by the board design. GPIO pins for the buzzer and switch were intentionally placed as far as possible from the USB-C connector, so the board fits properly into the enclosure.
4. Create a copy of `secrets_example.h` and rename it to `secrets.h`, then fill in all required values. 
5. Use Arduino IDE to flash the firmware to the device.
6. Home Assistant integration (optional)
	- Install the MQTT Broker add-on.
	- In the add-on configuration, add the device credentials `MQTT_NAME` and `MQTT_PASSWORD`.

There is also an option to use [ESPhome](SW/vanfox-esphome.yaml). However, gyroscope support is currently missing, which means, the display cannot be turned off automatically and will remain on continuously.

![preview2](doc/2.jpeg)

## Operating Modes

### Power Modes (Switch-Controlled)

**Switch ON - Device Active:**
- **Home Mode** (WiFi available)
  - Connects to Home Assistant via MQTT
  - Sends sensor data
  - Display timeout: 30 seconds
- **Traveling Mode** (No WiFi)
  - Runs offline, no cloud connectivity
  - Light sleep between sensor reads (~90% power savings)
  - Display timeout: 10 seconds

**Switch OFF - Deep Sleep:**
- Ultra-low power (~10µA draw)
- Wakes every 60 seconds to check switch state
- Device boots when switch turned ON

### Display Modes (Available in Both Home & Traveling)

#### Air Quality Mode (Default)
Shows real-time environmental readings (Motion detection wakes display):
```
CO2: 412 ppm
T: 21.3°C  H: 45.7%
VOC: 102  NOx: 34
Bat: 100% (4.05V)
```

#### Level Mode (Digital Spirit Level)
Activated by **triple shake gesture**. Shows roll angle in degrees for vehicle leveling. Auto-returns to Air Quality Mode after 2 minutes

## TODO
- add boudaries for buzz