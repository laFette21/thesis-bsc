# SLAM algoritmus implementációja

## Tartalomjegyzék

- Abstract / Témabejelentő?
- Köszönetnyilvánítás
- Bevezetés
  - Motiváció
    - Helymeghatározás problémája a robotikában
  - Irodalmi áttekintés
    - Különböző SLAM módszerek rövid bemutatása
- Felhasználói dokumentáció
  - Szimulációs környezet
    - Függőségek felsorolása
    - Telepítési útmutató?
    - Használati útmutató
    - Input/Output adatok ismertetése?
  - SLAM
    - Függőségek felsorolása
    - Telepítési útmutató?
    - Használati útmutató
    - Input/Output adatok ismertetése?
- Elméleti alapok
  - Szimulációs környezet
    - Mindenféle matematikai okosság bemutatása
  - SLAM
    - Mindenféle matematikai okosság bemutatása
- Fejlesztői dokumentáció
  - Szimulációs környezet
    - UML diagram
    - Use case diagram
    - Forráskód?
    - Továbbfejlesztési lehetőségek
  - SLAM
    - UML diagram
    - Use case diagram
    - Forráskód?
    - Továbbfejlesztési lehetőségek
- Tesztelés
  - Szimulációs környezet? (Csak ha lesz idő)
  - SLAM
    - Unit tesztelés?
    - Szimulátorból generált adatokkal való tesztelés
- Alkalmazás
  - Önvezető járművek úgy általában?
  - BME Formula Racing Team
- Konklúzió
  - Leszögezni, hogy miért olyan szuper a gráf alapú SLAM és hogy mi a legnagyobb probléma vele (Adatasszociáció hiánya)
- Forrásjegyzék

## Abstract / Témabejelentő

A járműipar egyre korszerűbb vezetőtámogató rendszereket fejleszt, ezzel folyamatos haladást mutatva a teljesen önvezető járművek felé. Az önvezető rendszerek egyik, ha nem a legfontosabb része a megbízható helyzetbecslés és a környezet feltérképezése.

A feladat a mobil robotika egy közismert problémája, melynek elterjedt megoldása az úgynevezett „Egyidejű lokalizáció és helymeghatározás” (Simultaneous Localization and Mapping, SLAM) módszer. A módszer nehézsége, hogy a térképkészítéshez szükségünk van egy precíz helyzetbecslésre, viszont a robot helyzetének pontos meghatározásához elengedhetetlen egy térkép. A SLAM során a robot iteratívan becsli a mozgását, eközben leképezi a környezetérzékelő szenzorból származó objektumokat, amelyek az algoritmus bemenetéül szolgálnak. A bemenet alapján a robot térképet készít a környezetéről és ezzel párhuzamosan elhelyezi magát a folyamatosan készülő térképen.

A dolgozatban egy SLAM algoritmus kerül kifejlesztésre az alábbi lépésekkel:
- SLAM irodalmi áttekintése;
- Szimulációs környezet kialakítása, amelyből egy adott pályaíven haladva az aktuális pillanatban érzékelt objektumok távolságadatai lekérdezhetőek lokális koordináta-rendszerben, és a jármű sebessége is rendelkezésre áll;
- SLAM algoritmus megvalósítása, mely az aktuálisan beérkező adatok alapján térképet készít, és becslést ad a jármű mozgására;
- Az algoritmus szimulált környezetben való demonstrációja és kiértékelése.

## Köszönetnyilvánítás

Ezúton szeretném megköszönni mindazoknak, akik nélkül ez a dolgozat nem jöhetett volna létre. Először is hatalmas köszönet mindkét konzulensemnek, *Fazekas Máténak* és *Eichhardt Ivánnak*, akik rendelkezésemre bocsátották tudásukat, hasznos tanácsokkal és rengeteg segítséggel láttak el a kutatás során.

Szintén hálával tartozom az Eötvös Loránd Tudományegyetem Informatikai Karának, hogy az itt eltöltött félévek alatt elsajátíthattam a kutatáshoz elengedhetetlenül szükséges matematikai és informatikai alapokat.

Továbbá szeretném megköszönni a *BME Formula Racing Team*-nek, hogy különböző erőforrásokat biztosítottak számomra, illetve hogy a köreikben végezhettem a kutatásom zömét.

Végül, de nem utolsósorban szeretnék köszönetet mondani a családomnak és barátaimnak, hogy minden támogatást megadtak, amire a tanulmányaim során szükségem volt.

## Bevezetés

### Motiváció

### Irodalmi áttekintés

## Felhasználói dokumentáció

### Szimulációs környezet

### SLAM

## Elméleti alapok / alapismeretek

### Szimulációs környezet

### SLAM

## Fejlesztői dokumentáció

### Szimulációs környezet

### SLAM

## Tesztelés

### Szimulációs környezet

### SLAM

## Alkalmazás

### Önvezető járművek általában



### BME Formula Racing Team

A *BME Formula Racing Team* 2007-ben alakult, körülbelül 10-15 közlekedésmérnök hallgatóból, amely azóta kinőtte magát, és már az egyetem szinte összes karáról csatlakoztak hallgatók a csapathoz.

Fennállásuk során már 16 autót építettek, ebből 5 belsőégésű 11 pedig elektromos motorral hajtott. 3 elektromos autót később önvezetővé alakítottak át. A csapat mérnökei és menedzserei különböző területeken dolgoznak ugyanazért a célért, a leendő mérnökök tudásának gyarapításáért és a minél jobb eredmények eléréséért, valamint hogy megteremtsék a magas szintű anyagi, szervezeti és tárgyi feltételeket.

Eddig több mint 300 hallgató tevékenykedett a tervezés, a beszerzés, a gyártás, a menedzsment, a logisztika és a gazdasági ügyek intézése területén. Jelenleg több mint 60 csapattaggal működnek, akik közt található gépész-, közlekedés-, villamosmérnök és közgazdaságtan hallgató is.

2011-ben mutatkozott be a Formula Student mezőnyében a BME FRT - és Magyarország - első elektromos hajtású Formula Student autója, a 2021-es évben pedig már a tizedik ilyen autójuk készült el.

Szerencsésnek mondhatom magam, hogy 2021 októberétől én is a csapat tagja lehetek, és első kézből tapasztalhatom meg, hogy milyen komplex feladatokkal kell szembesülnie egy autonóm rendszerekkel foglalkozó informatikusnak a csapat önvezető csoportjának tagjaként.

Mondhatjuk, hogy a dolgozatom termékgazdájának szerepét a csapat töltötte be, ugyanis ebben az évben a jelenleg is az autón működő részecskeszűrős SLAM mellé készült el a gráf alapú implementációm, hogy az előbb említett algoritmussal fúzionálva pontosabb működést érhessünk el.

## Konklúzió

A helymeghatározás és térképezés két elengehetetlen kritériuma a robotika azon részének, ahol az eszközünk autonóm módon kényszerül feladatok megoldására.

(Nem szükséges messzire mennünk, ha önvezető eszközöket szeretnénk csodálni. Egy remek példa - a manapság egyre elterjedtebb háztartási eszköz - a robotporszívó.)

A modern gráf alapú SLAM algoritmusok számítási költségüket tekintve nagyon hatékony megoldást kínálnak a probléma megoldására abban az esetben, ha a robotunk képes tökéletesen felismerni a környezetében jelenlévő tereptárgyakat.

A már korábban meglátogatott helyek felismerése borzalmasan nehéz feladat egy számítógép számára. Vegyünk példának egy hosszú, egyenes és referenciapontokban szegényes folyosót. Robotunk folyamatos bizonytalanságban mozogna, hiszen bár tudná, hogy történt elmozdulás, az érzékelésből nagyon kevés megerősítést kapna efelől. A hasonló helyszínek tetejébe ráadásul minden valós esetben társul valamilyen zaj a szenzorok pontatlanságából adódóan.

A dolgozatomnak egy ilyen gráf alapú SLAM algoritmus, illetve ezen algoritmus tesztelését elősegítő szimulációs környezet megvalósítása volt a célja. A szakdolgozat megírása közben a legnagyobb nehézséget nem maga az implementáció okozta, hanem megérteni, hogy mi is történik a színfalak mögött eme komplex probléma megoldása folyamán.
