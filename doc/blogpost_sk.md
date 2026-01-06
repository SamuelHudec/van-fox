VanFox vznikol z mojej praktickej potreby. S manželkou trávime veľa času v camper vane a zaujímalo ma, ako sa v malom uzavretom priestore postupne zhoršuje kvalita vzduchu. Počas spánku stúpa CO₂, pri varení sa hromadí vlhkosť a rôzne výpary sú cítiť oveľa skôr než doma v byte. Hotové riešenia možno existujú, ale ja som chcel niečo, na čom sa niečo nové naučím a bude to zábava. Navyše som chcel, aby zariadenie vedelo slúžiť nielen vo van-e, ale aj doma.

Ako základ som si zvolil LaskaKit ESP32 mini, pretože mi v jednom riešení ponúka I²C konektivitu, nabíjanie batérie, jednoduché flashovanie kódu a možnosť reťaziť senzory pomocou uŠup konektorov. Postup zostavenia, zoznam komponentov aj 3D model krabičky som popísal na GitHube (odkaz nižšie).

VanFox meria CO₂, teplotu, vlhkosť a kvalitu ovzdušia pomocou VOC a NOx indexov. Údaje zobrazuje na OLED displeji, ktorý sa rozsvieti len pri pohybe zariadenia. Vďaka integrovanej batérii ho možno používať ako prenosné zariadenie pri kempovaní v aute, stane atď. Zatiaľ čo doma je určený na trvalé napájanie, na cestách experimentujem s úspornými režimami a light sleep.

Gyroskop a akcelerometer som pôvodne pridal len kvôli tomu, aby sa displej rozsvietil pohybom bez potreby tlačidiel. Až neskôr som si uvedomil, že viem vyriešiť aj ďalší môj problém. Preto som pridal LEVEL MODE, digitálnu vodováhu aktivovanú trojitým zatrasením zariadenia.

Projekt je stále vo vývoji. Napríklad bzučiak je zapojený, ale zatiaľ sa v kóde nepoužíva. Pôvodný zámer bol zvukové upozornenie pri zhoršení kvality ovzdušia, no správanie ešte nemám úplne navrhnuté. Rovnako aj gestá, najmä triple-shake, by si zaslúžili ďalšie doladenie.

VanFox podporuje integráciu do Home Assistantu cez MQTT, ale zámerne nie je viazaný na jednu platformu.

Ak máš nápady, chuť prispieť alebo niečo vylepšiť, som otvorený spolupráci. Rád uvítam akékoľvek rady, či už po SW alebo HW stránke. Neboj sa rovno otvoriť PR s vylepšeniami alebo pokojne pridať aj vlastný file či rozšírenie.

https://github.com/SamuelHudec/van-fox