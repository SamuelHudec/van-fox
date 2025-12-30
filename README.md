# VanFox 🦊  – Portable Air Quality Monitor for Camper Vans

![preview](doc/3.jpeg)

**VanFox** is a fully self-contained ESP32-C3-based air quality monitor designed for camper vans and Home Assistant together.
It combines multiple environmental sensors, a motion-activated OLED display and gesture-based control. The secondary goal was to make parts available in one place.
The battery gives it the ability to operate without power, in case of a power outage or to take it outside with you.

## Postup

1. Kupte si componenty:
- ~298czk [Board](https://www.laskakit.cz/laskakit-microesp/)
- ~768czk [CO2, temerature and humidity senzor](https://www.laskakit.cz/laskakit-scd41-senzor-co2--teploty-a-vlhkosti-vzduchu)
- ~368czk [VOx and NOx senzor](https://www.laskakit.cz/laskakit-sgp41-voc-a-nox-senzor-kvality-ovzdusi/)
- ~118czk [Battery](https://www.laskakit.cz/ehao-lipol-baterie-603048-900mah-3-7v/)
- ~128czk [Oled Display](https://www.laskakit.cz/laskakit-oled-displej-128x32-0-91--i--c/)
- ~198czk [Gyroscop](https://www.laskakit.cz/laskakit-bmi270-6-osy-gyroskop-a-akcelerometr/)
- ~6czk [Buzz](https://www.laskakit.cz/aktivni-bzucak-3-3v/)
- ~4czk [Switch](https://www.laskakit.cz/posuvny-prepinac-0-5a-50vdc/)
- 4x~12czk [uŠup](https://www.laskakit.cz/--sup--stemma-qt--qwiic-jst-sh-4-pin-kabel-5cm/)

2. Vytlačte si [krabičku](3D) pomocou 3D tlačiarne,
3. Pripajkujte buzz, switch, I2C a bateriu k ESP doske. Pri I2C a baterii som nemal na výber, ale GPIO piny som umiestinl čo najdalej od usb-c portu. Len preto aby mi doska dobre sadla do krabičky.
4. Urob kopiu `secrets.h` z `secrets_example.h` a vypln. 
5. Pomocou Arduino IDE flash
6. Ak chceš využívať Home Assistante, stiahni si MQTT broker addon a cez config v addone pridaj zariadenie `MQTT_NAME` a `MQTT_PASSWORD`, ktoré si vyplnil.

Je tu možnosť využiť aj [ESPhome](SW/vanfox-esphome.yaml), ale tu nie je podpora pre gyrskop, takže display bude svietiť nonstop.

![preview2](doc/2.heic)

## Operating Modes

### Air Quality Mode (default)
Displays current environmental readings:
```
CO2: 412 ppm
T: 21.3°C  H: 45.7%
VOC: 102  NOx: 34
Bat: 100% (4.05V)
```
- OLED automatically turns off after 10 s of no motion.
- Device enters **light sleep** between sensor reads for power saving. (this feature is in experimental phase)

---

### Level Mode (Digital Spirit Level)
Activated by **triple shake gesture**:
- Shows inclination (roll) in degrees.
- Automatically returns to **Air Mode** after 2 minutes.
- Uses continuous polling (no sleep) — ideal for calibration or debugging.
- USB remains active → can be flashed without reset.


## Gesture Controls

| Gesture | Function |
|----------|-----------|
| **Triple shake** | Toggle between Air Mode ↔ Level Mode |
| **No motion for 10 s** | Display turns off (Air Mode only) |
| **Level Mode active for > 2 min** | Automatically returns to Air Mode |
