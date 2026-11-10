# 4-DOF Karton Robot Kol (Arduino Uno, 6× SG90)

Dönme matrislerinden (Rx/Ry/Rz → atan2) servo açısı hesaplayan, butonlu ve otomatik görevli robot kol.

![Robot kol](gorseller/robot_kol.jpg)

- **Servolar:** taban (pin 3), omuz 2 adet ters (5, 6), dirsek (9), gripper 2 çene ters (10, 11)
- **Butonlar:** taban 2/4, omuz 7/8, dirsek 12/13, gripper A0/A1, otomatik görev A2 (`INPUT_PULLUP`, diğer bacak GND'ye)
- **Görev:** 3 küpü (kırmızı, yeşil, mavi) alıp kendi hedeflerine (A, B, C) taşır.
- **Seri monitör (9600):** `j <base|shoulder|elbow|grip> <derece>` · `fk` · `home` · `task` · `stop` · `setstart` (mevcut pozu açılış pozu olarak kaydeder) · `resetstart`

## Buton testi ve breadboard bağlantısı

**Durum:** Buton testi ve servolu genel test, kabloların ve butonların breadboard'a tam oturmaması nedeniyle tamamlanamadı. Buton test kodu Arduino'ya yüklendi ve çalıştı (`Button test started` yazısı geliyor), ana kod (`src/main.cpp`) derlendi. Breadboard bağlantısı aşağıda anlatıldığı gibi kuruldu, ancak butonların tamamı doğrulanamadı.

Butonlar `test_donanim/buton_testi.ino` ile tek başına denenir. Bir butona basınca adı seri monitöre (9600 baud) yazılır, örneğin `Base right pressed`. Servolar bu aşamada takılı olmaz. Servolu genel test için `src/main.cpp` kullanılır.

### Butonlar

Her butonun iki bağlantısı var:
- bir bacak → Arduino'daki pin (sinyal teli)
- karşı bacak → mavi (−) ray, yani GND

| Buton | Pin | Buton | Pin |
|---|---|---|---|
| Taban sol | 2 | Dirsek aşağı | 12 |
| Taban sağ | 4 | Dirsek yukarı | 13 |
| Omuz ileri | 7 | Gripper aç | A0 |
| Omuz geri | 8 | Gripper kapat | A1 |
| Otomatik görev | A2 | | |

Butonlar breadboard'un ortasındaki oluğu köprüler: iki bacak üst yarıdaki, iki bacak alt yarıdaki deliklere girer.

![Butonların breadboard'daki yerleşimi](gorseller/breadboard_butonlar.jpg)

### Güç rayları

- Arduino **GND** → mavi (−) ray
- Arduino **5V** → kırmızı (+) ray
- Servo kırmızı kablo → kırmızı (+) ray, kahverengi kablo → mavi (−) ray, turuncu kablo → servonun pini

Bir ray sırasındaki bütün delikler birbirine bağlıdır. Kabloyu sıranın herhangi bir boş deliğine takabilirsin.

![Breadboard bağlantı şeması](gorseller/breadboard_baglanti_semasi.png)

![Arduino Uno ve breadboard](gorseller/uno_ve_breadboard.jpg)

### Test sırasında karşılaşılan sorun

Bazı butonlarda basmadan seri monitöre sürekli `... pressed` yazıldı. Olası sebep: butonun içinde birbirine hep bağlı olan bacaklar ortadaki oluğun iki yanına denk geliyor, yani devre basmadan kapalı kalıyor. Çözüm olarak **butonu 90° çevirip tekrar takmak** gerekiyor. Ayrıca bacakların deliklere sıkıca oturması ve kabloların gevşek olmaması gerekiyor. Bu düzeltmeler kablo ve buton oturma sorunları yüzünden uçtan uca doğrulanamadı.

Doğru bağlantıda beklenen davranış: basmadan monitör sessiz kalır, basınca butonun adı yazılır.

## Klasörler
- `src/main.cpp` – Arduino/PlatformIO kodu
- `diagram.json`, `wokwi.toml` – Wokwi devresi (PlatformIO derlemesi sonrası çalışır)
- `test_donanim/buton_testi.ino` – butonları seri monitörden test eden küçük kod (kendi klasöründe aç)
- `karton_sablonlar.pdf` – 1:1 karton kesim şablonları
- `gorseller/` – README fotoğrafları
- `arduino-denemeler/` – ilk basit denemeler (LED yakma gibi) ve ekran kayıtları

> Konum sabitleri (omuz yüksekliği, tutma noktası, küp/hedef açıları) kendi kolunun ölçüsüne göre kalibre edilmelidir.
