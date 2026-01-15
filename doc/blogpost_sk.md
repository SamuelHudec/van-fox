VanFox vznikol z mojej zvedavosti. S manželkou trávime veľa času v camper van a zaujímalo ma, ako sa v malom uzavretom priestore postupne zhoršuje kvalita vzduchu. Počas spánku stúpa CO₂ a pri varení sa zas hromadí vlhkosť a výpary oveľa skôr než doma v byte. Hotové riešenia možno existujú, ale ja som chcel niečo, na čom sa niečo nové naučím a bude to zábava. Navyše som chcel, aby zariadenie vedelo slúžiť nielen vo van-e, ale aj doma. Na moje prekvapenie aj manželka je nadšená.

Ako základ som si zvolil LaskaKit ESP32 mini, pretože mi v jednom riešení ponúka I²C konektivitu, nabíjanie batérie, jednoduché flashovanie kódu a možnosť reťaziť senzory pomocou uŠup konektorov. Postup zostavenia, zoznam komponentov aj 3D model krabičky som popísal na GitHube (odkaz nižšie).

Mašinka meria CO₂, teplotu, vlhkosť a kvalitu ovzdušia pomocou VOC a NOx indexov. Údaje zobrazuje na OLED displeji, ktorý sa rozsvieti len pri pohybe zariadenia. Vďaka integrovanej batérii ho možno používať ako prenosné zariadenie pri kempovaní v aute, stane atď. Zatiaľ čo doma je určený na trvalé napájanie, na cestách experimentujem s úspornými režimami a light sleep.

Gyroskop a akcelerometer som pôvodne pridal len kvôli tomu, aby sa displej rozsvietil pohybom bez potreby tlačidiel. Až neskôr som si uvedomil, že viem vyriešiť aj ďalší môj problém. Preto som pridal LEVEL MODE, digitálnu vodováhu aktivovanú trojitým zatrasením zariadenia.

Čo budeš potrebovať:
- [ESP32 dev board](https://www.laskakit.cz/laskakit-microesp/)
- [CO2, temperature and humidity senzor](https://www.laskakit.cz/laskakit-stcc4-senzor-co2--teploty-a-vlhkosti-vzduchu/)
- [VOx and NOx senzor](https://www.laskakit.cz/laskakit-sgp41-voc-a-nox-senzor-kvality-ovzdusi/)
- [Battery](https://www.laskakit.cz/ehao-lipol-baterie-603048-900mah-3-7v/)
- [Oled Display](https://www.laskakit.cz/laskakit-oled-displej-128x32-0-91--i--c/)
- [Gyroscop](https://www.laskakit.cz/laskakit-bmi270-6-osy-gyroskop-a-akcelerometr/)
- [Buzz](https://www.laskakit.cz/aktivni-bzucak-3-3v/)
- [Switch](https://www.laskakit.cz/posuvny-prepinac-0-5a-50vdc/)
- 4x [μŠup](https://www.laskakit.cz/--sup--stemma-qt--qwiic-jst-sh-4-pin-kabel-5cm/)
- 10x M2 šroub, ja som kupil celu krabičku a odštipal poldla potreby

Ako na to:
1. Vytlačte krabičku z [3D](https://github.com/SamuelHudec/van-fox/tree/main/3D).
2. Pripájkujte bzučiak, spínač, káble I²C a batériu k doske ESP. Pre I²C a batériu je umiestnenie pinov pevne stanovené dizajnom dosky. Piny GPIO pre bzučiak (GPIO 3) a spínač (GPIO 2) boli zámerne vybrané čo najďalej od konektora USB-C, aby doska správne zapadla do krytu.
3. Vytvorte kópiu súboru `secrets_example.h` a premenujte ho na `secrets.h`, potom vyplňte všetky požadované hodnoty. Nahrajte [kod](https://github.com/SamuelHudec/van-fox/blob/main/SW/vanfox/vanfox.ino).
4. Integrácia s Home Assistant (voliteľné)
- Nainštalujte doplnok MQTT Broker.
- V konfigurácii doplnku pridajte prihlasovacie údaje zariadenia `MQTT_NAME` a `MQTT_PASSWORD`.

Projekt je stále vo vývoji. Napríklad bzučiak je zapojený, ale zatiaľ sa v kóde nepoužíva. Pôvodný zámer bol zvukové upozornenie pri zhoršení kvality ovzdušia, no správanie ešte nemám úplne navrhnuté. Rovnako aj gestá, najmä triple-shake, by si zaslúžili ďalšie doladenie. V kóde je aj integrácia do Home Assistantu cez MQTT.

Ak máš nápady, chuť prispieť alebo niečo vylepšiť, som otvorený spolupráci. Napríklad, by sa prepínač dal napojiť priamo na prúd a namiesto deepsleep režimu, natvrdo vypnuť modul. Rád uvítam akékoľvek rady, či už po SW alebo HW stránke. Neboj sa rovno otvoriť PR s vylepšeniami alebo pokojne pridať aj vlastný file či rozšírenie.

https://github.com/SamuelHudec/van-fox