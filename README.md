# 868-Funkempfänger (LilyGo LoRa32 V2.1)

ESP32-Firmware für ein **LilyGo / TTGO LoRa32 V2.1 (SX1276, 868 MHz)**. Empfängt per
[rtl_433_ESP](https://github.com/NorthernMan54/rtl_433_ESP) Ecowitt-/Fine-Offset-Sensoren
auf **868,35 MHz (FSK)** und macht daraus Webseite, OLED-Anzeige, LED und ntfy-Push.

## Funktionen

- **Temperatur/Feuchte** (z. B. Ecowitt WH31E): Kacheln mit relativer und absoluter Feuchte,
  Verlauf, RSSI, Schwellenwerte mit Hysterese, Signalverlust- und Batteriemeldung.
- **Referenzvergleich + Lüftungsempfehlung:** Differenz der absoluten Feuchte gegen einen
  Referenzsensor (z. B. außen), Anzeige auf Web, OLED und LED.
- **Wasserleck-Melder** (Ecowitt WH55, Seite `/wasser`): Alarm als Banner, OLED-Vollbild-
  Warnung, blinkende LED und ntfy; Entwarnung, Batterie schwach/leer (tägliche Erinnerung),
  „meldet sich nicht mehr" nach 15 min, Namen je Sensor, Testknopf.
- **WLAN-Portal** (`/wifi`, zwei Netze, Setup-Hotspot), **OTA-Updates**, mehrere
  OLED-Seiten (`/display`), Diagnoseseite `/andere` für Telegramme ohne Temperatur/Feuchte.
- Optional: MQTT (`-DUSE_MQTT=1`) und ThingSpeak.

## Wichtig: FSK, nicht OOK

rtl_433_ESP kann pro Build nur OOK **oder** FSK. Dieses Projekt baut mit
`-DOOK_MODULATION=false` (FSK) – ein OOK-Build sieht die 868-MHz-Ecowitt-Sensoren nie.
SX1262-Boards (z. B. Heltec V3) liefern keinen rohen Bitstrom und taugen nicht.

## Bauen und flashen

```
cp include/secrets.h.example include/secrets.h     # und anpassen
pio run -e lilygo868_dauer -t upload --upload-port COMxx   # per USB
pio run -e ota -t upload --upload-port <IP>                 # drahtlos
```

Das OTA-Passwort steht in `platformio.ini` (`--auth=DEIN_OTA_PASSWORT`) und muss mit
`OTA_PASSWORD` in `secrets.h` übereinstimmen. `include/secrets.h` gehört nicht ins Repo.

## Weboberfläche

`/` Übersicht · `/alle` Sensoren verwalten · `/settings` Schwellenwerte/ntfy · `/wasser`
Wassersensoren · `/wifi` WLAN · `/display` OLED-Seiten · `/andere` übrige Telegramme · `/json`

## Tools

`tools/sendezyklen-log-seriell.ps1` und `tools/sendezyklen-log-http.ps1` protokollieren
jedes Telegramm (Zeit, Sensor, RSSI) in eine CSV-Datei.
