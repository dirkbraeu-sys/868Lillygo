/* ===========================================================================
 *  LilyGo 868-Empfaenger
 *  ---------------------------------------------------------------------------
 *  Hardware : LilyGo / TTGO LoRa32 V2.1 (ESP32 + SX1276 868 MHz + SSD1306 OLED)
 *
 *  Aufgabe  : 868-MHz-Sensoren empfangen und dekodieren (rtl_433-Protokolle
 *             ueber die Bibliothek rtl_433_ESP, SX1276 im OOK-Dauerempfang).
 *             Reiner, ungefilterter Scanner: ALLE erkannten Temperatur-/
 *             Feuchte-Sensoren (Schluessel Modell+Kanal+ID) werden gelistet,
 *             keine feste Zuordnung/Rolle, keine Abweichungs-/Alarmlogik.
 *
 *  Anzeige  : OLED (Liste, seitenweise) und Status-Webseite unter
 *             http://empfaenger.local/ und /json. WLAN optional mit OTA.
 *             Bewusst eigenstaendiges Projekt - KEINE Kopplung an das
 *             Schwesterprojekt "Feuchtewaechter 433" (Desktop\feuchtewaechter-433\)
 *             und KEIN MQTT/Home-Assistant-Push (Code dazu ist unten komplett
 *             per USE_MQTT 0 abgeschaltet, nicht nur ausgeblendet).
 *
 *  Schwellenwerte: je Sensor (Modell+Kanal+ID) optional Ober-/Untergrenze
 *             fuer Temperatur und rel. Feuchte, mit Hysterese gegen
 *             Meldungs-Flattern. Eingabe/Aenderung auf
 *             http://empfaenger.local/settings, persistiert im NVS-Flash
 *             (uebersteht Neustart/Stromausfall). Bei Ueber-/Unterschreiten
 *             bzw. Rueckkehr in den Normalbereich geht eine ntfy-Push-
 *             Meldung raus (Topic in include/secrets.h, NTFY_TOPIC).
 * ===========================================================================
 */

#include <Arduino.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <ctype.h>
#include <Preferences.h>

// --- Bausteine einzeln zuschaltbar (per -D ueberschreibbar). Standard: an. ---
#ifndef USE_OLED
#define USE_OLED  1
#endif
#ifndef USE_RADIO
#define USE_RADIO 1
#endif
#ifndef USE_WIFI
#define USE_WIFI  1
#endif
#ifndef USE_MQTT
#define USE_MQTT  0              // aus: eigenstaendiges Projekt, keine HA-Kopplung (per -D wieder anschaltbar)
#endif
#ifndef USE_THINGSPEAK
#define USE_THINGSPEAK  0        // aus: braucht eigenen ThingSpeak-Account/API-Key (per -D wieder anschaltbar)
#endif
#include <Wire.h>
#include <ArduinoJson.h>
#include <U8g2lib.h>
#include <rtl_433_ESP.h>

#if USE_WIFI
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "secrets.h"                      // WIFI_SSID / WIFI_PASS / OTA_PASSWORD / MQTT_* / NTFY_TOPIC / AP_SETUP_PASSWORD
#ifndef NET_HOSTNAME                      // fuer ein zweites Board per -DNET_HOSTNAME=... ueberschreibbar
#define NET_HOSTNAME  "lilygo868"          // erreichbar als http://lilygo868.local/
#endif
#define AP_SETUP_SSID "LilyGo868-Setup"   // Hotspot-Name (je Board eindeutig), wenn kein WLAN erreichbar ist
#endif
#if USE_MQTT
#include <PubSubClient.h>
#endif

/* ------------------------------------------------------------------ *
 *  1) BOARD-PINS  (TTGO / LilyGo LoRa32 V2.1) - siehe Feuchtewaechter-433:
 *     der OLED-Reset-Pin (GPIO16) darf auf diesem PICO-D4-Board NICHT
 *     bedient werden (Boot-Crash) -> U8X8_PIN_NONE im Konstruktor unten.
 * ------------------------------------------------------------------ */
#define OLED_SDA          21
#define OLED_SCL          22
#define LED_PIN           25          // On-Board-LED: blinkt kurz bei jedem Empfang

#ifndef RF_MODULE_RECEIVER_GPIO
#define RF_MODULE_RECEIVER_GPIO 32
#endif
#ifndef RF_MODULE_FREQUENCY
#define RF_MODULE_FREQUENCY 868.3
#endif

/* ------------------------------------------------------------------ *
 *  2) INTERN                                                        *
 * ------------------------------------------------------------------ */
#define JSON_MSG_BUFFER 512
static char  messageBuffer[JSON_MSG_BUFFER];
static rtl_433_ESP rf;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /*reset=*/ U8X8_PIN_NONE);

// Ein gesehener Sensor, Schluessel = Modell + Kanal + ID
struct Sensor {
  bool     valid = false;
  char     model[32] = {0};
  int      channel   = 0;
  long     id        = -1;
  float    t         = NAN;    // C
  float    rh        = NAN;    // %
  float    rssi      = NAN;    // dBm, Empfangsstaerke des letzten Telegramms (NAN = nicht geliefert)
  bool     batteryOk = true;   // aus dem Funktelegramm (battery_ok), Default ok
  uint32_t stamp     = 0;      // millis() des letzten Empfangs
  uint32_t firstSeen = 0;
  uint32_t count     = 0;      // wie oft schon empfangen
#if USE_MQTT
  bool     discovered = false; // MQTT-Discovery fuer diesen Sensor schon gesendet? (aktuell ungenutzt, USE_MQTT=0)
#endif
};

static const int MAX_SENSORS = 32;
static Sensor sensors[MAX_SENSORS];
static int    sensorCount = 0;
static uint32_t lastRxMs  = 0;   // fuer den kurzen LED-Blitz bei Empfang

// true, wenn das Board mangels bekanntem WLAN gerade selbst einen Hotspot
// aufgemacht hat (siehe Abschnitt 6, netSetup()). Frueh deklariert, damit
// das OLED (Abschnitt 4) das ohne Vorwaertsdeklaration abfragen kann.
static bool apMode = false;
// Hotspot laeuft (softAP) - entweder als Boot-Fallback (apMode) oder zusaetzlich im Betrieb,
// wenn das WLAN laenger ausfaellt bzw. von /wifi aus von Hand eingeschaltet wurde (apManual).
static bool apActive = false;
static bool apManual = false;

static uint32_t ageSec(const Sensor &s) { return (millis() - s.stamp) / 1000; }

// Absolute Feuchte (g Wasserdampf je m3 Luft) nach der Magnus-Formel - der Sensor
// liefert nur die relative Feuchte, die absolute ist daraus berechenbar und
// vergleichbarer zwischen unterschiedlich temperierten Orten.
static float saturationVaporPressureHpa(float tC) {
  return 6.112f * expf((17.62f * tC) / (243.12f + tC));
}
static float absoluteHumidity(float tC, float rh) {
  if (isnan(tC) || isnan(rh)) return NAN;
  float es = saturationVaporPressureHpa(tC);
  float e  = es * (rh / 100.0f);
  return 216.7f * e / (273.15f + tC);
}

// Taupunkt (Magnus, gleiche Konstanten wie die Taupunkt-Sensoren in Home Assistant).
static float dewPointC(float tC, float rh) {
  if (isnan(tC) || isnan(rh) || rh <= 0.0f) return NAN;
  float g = logf(rh / 100.0f) + 17.62f * tC / (243.12f + tC);
  return 243.12f * g / (17.62f - g);
}

// Messgroessen, die sich fuer die Display-Seiten waehlen lassen (Startseite, "Angezeigte Werte").
enum { Q_T = 0, Q_RH = 1, Q_AH = 2, Q_TD = 3, Q_SPREAD = 4, Q_RSSI = 5, Q_COUNT = 6 };
static float qValue(int q, float t, float rh, float rssi) {
  switch (q) {
    case Q_T:      return t;
    case Q_RH:     return rh;
    case Q_AH:     return absoluteHumidity(t, rh);
    case Q_TD:     return dewPointC(t, rh);
    case Q_SPREAD: { float td = dewPointC(t, rh); return isnan(td) ? NAN : t - td; }   // Abstand zum Taupunkt
    case Q_RSSI:   return rssi;
  }
  return NAN;
}
static const char *Q_CARD_LABEL[Q_COUNT] = {"TEMP.", "LUFTFEUCHTE", "ABS.FEUCHTE", "TAUPUNKT", "TP-ABSTAND", "EMPFANG"};
static const char *Q_SHORT[Q_COUNT]      = {"TEMP.", "REL.F.", "ABS.F.", "TAUPKT", "ABST.", "EMPF."};
static const char *Q_UNIT[Q_COUNT]       = {"\xb0" "C", "%", "g/m3", "\xb0" "C", "K", "dBm"};
static int qDecimals(int q) { return (q == Q_RH || q == Q_RSSI) ? 0 : 1; }

// Sensorschluessel (Modell+Kanal+ID) fuer Weboberflaeche/MQTT/Schwellenwerte:
// alles ausser [A-Za-z0-9_] wird zu '_' (rtl_433-Modellnamen wie "Nexus-TH"
// enthalten Bindestriche).
static void makeSensorKey(const char *model, int channel, long id, char *out, size_t outLen) {
  char m[32];
  strncpy(m, model, sizeof(m) - 1);
  m[sizeof(m) - 1] = 0;
  for (char *p = m; *p; p++)
    if (!isalnum((unsigned char)*p)) *p = '_';
  snprintf(out, outLen, "%s_ch%d_id%ld", m, channel, id);
}

#if USE_WIFI
// Definition weiter unten in Abschnitt 6 (braucht HTTPClient/WiFi).
static void notify(const char *msg, bool force = false);
#endif
// Definitionen weiter unten in Abschnitt 2d (Eigene Sensoren) - hier schon
// gebraucht, um Signalverlust-Meldungen auf "eigene" Sensoren zu begrenzen
// bzw. Anzeigenamen statt Herstellername in ntfy-Meldungen zu nutzen.
static bool isOwned(const char *model, int channel, long id);
static bool anyOwned();
static bool getDisplayName(const char *model, int channel, long id, char *out, size_t outLen);

// Liefert den Anzeigenamen falls vergeben, sonst den Herstellernamen -
// fuer alle Stellen, an denen bisher makeSensorKey() allein genutzt wurde.
static void makeDisplayLabel(const char *model, int channel, long id, char *out, size_t outLen) {
  if (getDisplayName(model, channel, id, out, outLen)) return;
  makeSensorKey(model, channel, id, out, outLen);
}

/* ------------------------------------------------------------------ *
 *  2b) SCHWELLENWERTE + NTFY-PUSH                                    *
 *      Je Sensor (Modell+Kanal+ID) optional Ober-/Untergrenze fuer   *
 *      Temperatur und rel. Feuchte, mit Hysterese gegen Flattern.    *
 *      Einstellbar auf http://empfaenger.local/settings, persistiert *
 *      im NVS-Flash (uebersteht Neustart/Stromausfall). "kein Limit" *
 *      wird intern als Sentinel -999/+999 gespeichert.               *
 * ------------------------------------------------------------------ */
struct Threshold {
  bool  used      = false;   // Slot belegt (Sensor wurde konfiguriert)?
  char  model[32] = {0};
  int   channel   = 0;
  long  id        = -1;
  float tMin  = -999, tMax  = 999;   // Grad C, Sentinel = kein Limit
  float rhMin = -999, rhMax = 999;   // %, Sentinel = kein Limit
  float hystT  = 0.5f;               // Hysterese Temperatur (Grad C)
  float hystRH = 3.0f;               // Hysterese Feuchte (%-Punkte)
  // Je Schwellenwert-Typ einzeln abschaltbar, ob dafuer eine ntfy-Meldung
  // rausgeht - die Auswertung (Alarmzustand, Faerbung auf der Startseite)
  // laeuft davon UNABHAENGIG immer weiter, nur der Push ist optional.
  bool  notifyT  = true;
  bool  notifyRH = true;
  // Vergleich mit dem Referenzsensor (siehe RefSensor/checkComparisons):
  bool  cmpEnabled = false;
  float cmpDT  = 999;   // max. erlaubte |Temp-Differenz| zur Referenz (Grad C), Sentinel = aus
  float cmpDRH = 999;   // max. erlaubte |rF-Differenz| (%-Punkte), Sentinel = aus
  float cmpDAH = 999;   // max. erlaubte |abs.Feuchte-Differenz| (g/m3), Sentinel = aus
  float hystAH = 0.5f;  // Hysterese fuer die AH-Differenz (g/m3)
  bool  notifyCmpT = true, notifyCmpRH = true, notifyCmpAH = true;
  // ThingSpeak: je Wert einzeln abschaltbar, ob und auf welches Feld (1-8)
  // er gesendet wird (siehe sendThingSpeak()) - frei waehlbar, damit mehrere
  // Sensoren sich nicht versehentlich denselben Kanal teilen und man nicht
  // zwangslaeufig alle drei Werte eines Sensors mitsenden muss.
  bool  tsEnT  = false; int tsFieldT  = 1;   // Temperatur
  bool  tsEnRH = false; int tsFieldRH = 2;   // rel. Feuchte
  bool  tsEnAH = false; int tsFieldAH = 3;   // abs. Feuchte
  // ...und dieselben drei nochmal als Abweichung (Delta) zum Referenzsensor -
  // fuer den aktuell markierten Referenzsensor selbst ergeben diese natuerlich
  // keinen Sinn (Delta zu sich selbst), siehe Filterung in handleSettingsGet().
  bool  tsEnDT  = false; int tsFieldDT  = 4;   // Delta-Temperatur
  bool  tsEnDRH = false; int tsFieldDRH = 5;   // Delta-rel.Feuchte
  bool  tsEnDAH = false; int tsFieldDAH = 6;   // Delta-abs.Feuchte
  // Laufzeit-Alarmzustand (nicht persistiert, nach Neustart neu bewertet):
  bool  alarmTHigh = false, alarmTLow = false;
  bool  alarmRHHigh = false, alarmRHLow = false;
  bool  alarmSignalLost = false;   // >SIGNAL_LOST_SEC ohne Empfang
  bool  alarmBatteryLow = false;   // battery_ok=0 im letzten Telegramm
  bool  alarmCmpT = false, alarmCmpRH = false, alarmCmpAH = false;   // Vergleich mit Referenz
};

static const int MAX_THRESH = 8;   // max. gleichzeitig konfigurierte Sensoren
static Threshold thresholds[MAX_THRESH];
static Preferences prefs;

// Wird in checkComparisons() aktualisiert: true, sobald FUER MINDESTENS EINEN
// Sensor die Luftungsempfehlung aktiv ist (siehe dort). Steuert die Onboard-LED
// (siehe updateLed()) - so ist die Empfehlung auch ganz ohne WLAN/Handy direkt
// am Board sichtbar.
static bool ventilationRecommended = false;

static Threshold *findThreshold(const char *model, int channel, long id) {
  for (int i = 0; i < MAX_THRESH; i++) {
    Threshold &c = thresholds[i];
    if (c.used && c.channel == channel && c.id == id && strcmp(c.model, model) == 0)
      return &c;
  }
  return nullptr;
}

// Fuer den Speichern-Handler: vorhandenen Slot fuer diesen Sensor wiederverwenden
// oder einen freien belegen.
static Threshold *getOrCreateThreshold(const char *model, int channel, long id) {
  Threshold *c = findThreshold(model, channel, id);
  if (c) return c;
  for (int i = 0; i < MAX_THRESH; i++) {
    if (!thresholds[i].used) {
      strncpy(thresholds[i].model, model, sizeof(thresholds[i].model) - 1);
      thresholds[i].channel = channel;
      thresholds[i].id = id;
      return &thresholds[i];
    }
  }
  return nullptr;   // alle Slots belegt (mehr als MAX_THRESH konfigurierte Sensoren)
}

// Persistierte Felder als kompaktes, reines POD-Struct - wird als EIN einziger
// Byte-Blob fuer alle MAX_THRESH Slots gespeichert (Key "all"), statt wie
// frueher ~19 einzelne Preferences-Keys PRO Slot (bis zu 150+ Keys insgesamt).
// Grund: NVS reserviert pro Key einen eigenen Eintrag und der Flash-Platz dafuer
// ist auf diesem Board knapp bemessen - mit vielen Slots und Schluessel-Kirchen
// (Sensor wechselt/kommt/geht) lief das NVS irgendwann komplett voll
// ("NOT_ENOUGH_SPACE" in nvs_set_*), wonach JEDES Speichern - inklusive neuer
// Grenzwerte - still fehlschlug (Preferences loggt den Fehler nur auf die
// Serial-Konsole, die Weboberflaeche zeigte trotzdem "Gespeichert."). Ein Blob
// pro Namespace braucht dagegen nur EINEN NVS-Eintrag insgesamt.
struct ThreshPersist {
  bool  used = false;
  char  model[32] = {0};
  int   channel = 0;
  long  id = -1;
  float tMin = -999, tMax = 999, rhMin = -999, rhMax = 999;
  float hystT = 0.5f, hystRH = 3.0f;
  bool  notifyT = true, notifyRH = true;
  bool  cmpEnabled = false;
  float cmpDT = 999, cmpDRH = 999, cmpDAH = 999, hystAH = 0.5f;
  bool  notifyCmpT = true, notifyCmpRH = true, notifyCmpAH = true;
  bool  tsEnT = false;  int tsFieldT  = 1;
  bool  tsEnRH = false; int tsFieldRH = 2;
  bool  tsEnAH = false; int tsFieldAH = 3;
  bool  tsEnDT = false;  int tsFieldDT  = 4;
  bool  tsEnDRH = false; int tsFieldDRH = 5;
  bool  tsEnDAH = false; int tsFieldDAH = 6;
};

// Aeltere Blob-Layouts - nur fuer die Migration beim ersten Boot nach einem
// Update, damit ein reiner Struct-Groessenwechsel nicht die gerade erst neu
// eingegebenen Schwellenwerte/Referenz-Einstellungen des Nutzers loescht
// (siehe die Fallback-Kette weiter unten in loadThresholds()).
// V3: wie das aktuelle Layout, aber noch OHNE die drei Delta-ThingSpeak-Felder
// (tsEnDT/tsEnDRH/tsEnDAH) - das vorherige "aktuelle" Struct vor dieser Aenderung.
struct ThreshPersistV3 {
  bool  used = false;
  char  model[32] = {0};
  int   channel = 0;
  long  id = -1;
  float tMin = -999, tMax = 999, rhMin = -999, rhMax = 999;
  float hystT = 0.5f, hystRH = 3.0f;
  bool  notifyT = true, notifyRH = true;
  bool  cmpEnabled = false;
  float cmpDT = 999, cmpDRH = 999, cmpDAH = 999, hystAH = 0.5f;
  bool  notifyCmpT = true, notifyCmpRH = true, notifyCmpAH = true;
  bool  tsEnT = false;  int tsFieldT  = 1;
  bool  tsEnRH = false; int tsFieldRH = 2;
  bool  tsEnAH = false; int tsFieldAH = 3;
};
// V2: ein gemeinsamer ThingSpeak-Schalter fuer Temperatur+rel.Feuchte, keine
// separate abs.-Feuchte-Auswahl.
struct ThreshPersistV2 {
  bool  used = false;
  char  model[32] = {0};
  int   channel = 0;
  long  id = -1;
  float tMin = -999, tMax = 999, rhMin = -999, rhMax = 999;
  float hystT = 0.5f, hystRH = 3.0f;
  bool  notifyT = true, notifyRH = true;
  bool  cmpEnabled = false;
  float cmpDT = 999, cmpDRH = 999, cmpDAH = 999, hystAH = 0.5f;
  bool  notifyCmpT = true, notifyCmpRH = true, notifyCmpAH = true;
  bool  tsEnabled = false;
  int   tsFieldT = 1, tsFieldRH = 2;
};
// V1: noch ganz ohne ThingSpeak-Felder.
struct ThreshPersistV1 {
  bool  used = false;
  char  model[32] = {0};
  int   channel = 0;
  long  id = -1;
  float tMin = -999, tMax = 999, rhMin = -999, rhMax = 999;
  float hystT = 0.5f, hystRH = 3.0f;
  bool  notifyT = true, notifyRH = true;
  bool  cmpEnabled = false;
  float cmpDT = 999, cmpDRH = 999, cmpDAH = 999, hystAH = 0.5f;
  bool  notifyCmpT = true, notifyCmpRH = true, notifyCmpAH = true;
};

static void saveThresholds();   // Vorwaertsdeklaration fuer die Migration unten

static void loadThresholds() {
  prefs.begin("thresh", true);
  ThreshPersist buf[MAX_THRESH];
  size_t got = prefs.getBytes("all", buf, sizeof(buf));
  int n = 0;
  bool migrated = false;
  if (got == sizeof(buf)) {
    for (int i = 0; i < MAX_THRESH; i++) {
      ThreshPersist &p = buf[i];
      Threshold &t = thresholds[i];
      t.used = p.used;
      if (!t.used) continue;
      n++;
      strncpy(t.model, p.model, sizeof(t.model) - 1);
      t.channel = p.channel; t.id = p.id;
      t.tMin = p.tMin; t.tMax = p.tMax; t.rhMin = p.rhMin; t.rhMax = p.rhMax;
      t.hystT = p.hystT; t.hystRH = p.hystRH;
      t.notifyT = p.notifyT; t.notifyRH = p.notifyRH;
      t.cmpEnabled = p.cmpEnabled;
      t.cmpDT = p.cmpDT; t.cmpDRH = p.cmpDRH; t.cmpDAH = p.cmpDAH; t.hystAH = p.hystAH;
      t.notifyCmpT = p.notifyCmpT; t.notifyCmpRH = p.notifyCmpRH; t.notifyCmpAH = p.notifyCmpAH;
      t.tsEnT = p.tsEnT; t.tsFieldT = p.tsFieldT;
      t.tsEnRH = p.tsEnRH; t.tsFieldRH = p.tsFieldRH;
      t.tsEnAH = p.tsEnAH; t.tsFieldAH = p.tsFieldAH;
      t.tsEnDT = p.tsEnDT; t.tsFieldDT = p.tsFieldDT;
      t.tsEnDRH = p.tsEnDRH; t.tsFieldDRH = p.tsFieldDRH;
      t.tsEnDAH = p.tsEnDAH; t.tsFieldDAH = p.tsFieldDAH;
    }
  } else if (got == sizeof(ThreshPersistV3) * MAX_THRESH) {
    // Vorheriges Format: wie aktuell, aber noch ohne die drei Delta-Felder
    // (ThingSpeak fuer Delta-T/-rF/-AH). Werte uebernehmen, Delta-Felder auf
    // Default (aus) setzen, dann im neuen Format zurueckschreiben.
    ThreshPersistV3 bufV3[MAX_THRESH];
    prefs.getBytes("all", bufV3, sizeof(bufV3));
    for (int i = 0; i < MAX_THRESH; i++) {
      ThreshPersistV3 &p = bufV3[i];
      Threshold &t = thresholds[i];
      t.used = p.used;
      if (!t.used) continue;
      n++;
      strncpy(t.model, p.model, sizeof(t.model) - 1);
      t.channel = p.channel; t.id = p.id;
      t.tMin = p.tMin; t.tMax = p.tMax; t.rhMin = p.rhMin; t.rhMax = p.rhMax;
      t.hystT = p.hystT; t.hystRH = p.hystRH;
      t.notifyT = p.notifyT; t.notifyRH = p.notifyRH;
      t.cmpEnabled = p.cmpEnabled;
      t.cmpDT = p.cmpDT; t.cmpDRH = p.cmpDRH; t.cmpDAH = p.cmpDAH; t.hystAH = p.hystAH;
      t.notifyCmpT = p.notifyCmpT; t.notifyCmpRH = p.notifyCmpRH; t.notifyCmpAH = p.notifyCmpAH;
      t.tsEnT = p.tsEnT; t.tsFieldT = p.tsFieldT;
      t.tsEnRH = p.tsEnRH; t.tsFieldRH = p.tsFieldRH;
      t.tsEnAH = p.tsEnAH; t.tsFieldAH = p.tsFieldAH;
      // t.tsEnDT/tsEnDRH/tsEnDAH bleiben bei den Struct-Defaults (aus).
      migrated = true;
    }
    if (migrated) Serial.println("Schwellenwerte: von Blob-Format v3 auf v4 migriert (Delta-Werte fuer ThingSpeak ergaenzt).");
  } else if (got == sizeof(ThreshPersistV2) * MAX_THRESH) {
    // Vorheriges Format: EIN gemeinsamer ThingSpeak-Schalter fuer Temperatur+
    // rel.Feuchte, keine abs.-Feuchte-Auswahl. Werte uebernehmen, den alten
    // Schalter auf die beiden neuen Einzel-Schalter aufteilen, abs. Feuchte
    // auf Default (aus) setzen, dann im neuen Format zurueckschreiben.
    ThreshPersistV2 bufV2[MAX_THRESH];
    prefs.getBytes("all", bufV2, sizeof(bufV2));
    for (int i = 0; i < MAX_THRESH; i++) {
      ThreshPersistV2 &p = bufV2[i];
      Threshold &t = thresholds[i];
      t.used = p.used;
      if (!t.used) continue;
      n++;
      strncpy(t.model, p.model, sizeof(t.model) - 1);
      t.channel = p.channel; t.id = p.id;
      t.tMin = p.tMin; t.tMax = p.tMax; t.rhMin = p.rhMin; t.rhMax = p.rhMax;
      t.hystT = p.hystT; t.hystRH = p.hystRH;
      t.notifyT = p.notifyT; t.notifyRH = p.notifyRH;
      t.cmpEnabled = p.cmpEnabled;
      t.cmpDT = p.cmpDT; t.cmpDRH = p.cmpDRH; t.cmpDAH = p.cmpDAH; t.hystAH = p.hystAH;
      t.notifyCmpT = p.notifyCmpT; t.notifyCmpRH = p.notifyCmpRH; t.notifyCmpAH = p.notifyCmpAH;
      t.tsEnT = p.tsEnabled; t.tsFieldT = p.tsFieldT;
      t.tsEnRH = p.tsEnabled; t.tsFieldRH = p.tsFieldRH;
      // t.tsEnAH/tsFieldAH bleiben bei den Struct-Defaults (aus).
      migrated = true;
    }
    if (migrated) Serial.println("Schwellenwerte: von Blob-Format v2 auf v3 migriert (abs. Feuchte fuer ThingSpeak ergaenzt).");
  } else if (got == sizeof(ThreshPersistV1) * MAX_THRESH) {
    // Vorheriges Blob-Format (vor den ThingSpeak-Feldern) - Werte uebernehmen,
    // ThingSpeak-Felder auf Default setzen, dann im neuen Format zurueckschreiben.
    ThreshPersistV1 bufV1[MAX_THRESH];
    prefs.getBytes("all", bufV1, sizeof(bufV1));
    for (int i = 0; i < MAX_THRESH; i++) {
      ThreshPersistV1 &p = bufV1[i];
      Threshold &t = thresholds[i];
      t.used = p.used;
      if (!t.used) continue;
      n++;
      strncpy(t.model, p.model, sizeof(t.model) - 1);
      t.channel = p.channel; t.id = p.id;
      t.tMin = p.tMin; t.tMax = p.tMax; t.rhMin = p.rhMin; t.rhMax = p.rhMax;
      t.hystT = p.hystT; t.hystRH = p.hystRH;
      t.notifyT = p.notifyT; t.notifyRH = p.notifyRH;
      t.cmpEnabled = p.cmpEnabled;
      t.cmpDT = p.cmpDT; t.cmpDRH = p.cmpDRH; t.cmpDAH = p.cmpDAH; t.hystAH = p.hystAH;
      t.notifyCmpT = p.notifyCmpT; t.notifyCmpRH = p.notifyCmpRH; t.notifyCmpAH = p.notifyCmpAH;
      // t.tsEnabled/tsFieldT/tsFieldRH bleiben bei den Struct-Defaults.
      migrated = true;
    }
    if (migrated) Serial.println("Schwellenwerte: von Blob-Format v1 auf v2 migriert (ThingSpeak-Felder ergaenzt).");
  } else if (got == 0) {
    // Neuer Blob-Key ("all") existiert noch nicht - entweder Erststart, oder
    // Umstieg von der alten Einzel-Key-Version (s0u, s0m, s0tn, ...). Alte
    // Keys versuchsweise lesen, damit vorhandene Grenzwerte aus fruehreren
    // Firmware-Versionen nicht verloren gehen; anschliessend sofort im neuen,
    // kompakten Format zurueckschreiben (raeumt die alten Keys mit auf).
    for (int i = 0; i < MAX_THRESH; i++) {
      char k[8];
      snprintf(k, sizeof(k), "s%du", i);
      if (!prefs.getBool(k, false)) continue;
      Threshold &t = thresholds[i];
      t.used = true;
      snprintf(k, sizeof(k), "s%dm", i);
      String m = prefs.getString(k, "");
      strncpy(t.model, m.c_str(), sizeof(t.model) - 1);
      snprintf(k, sizeof(k), "s%dc", i);  t.channel = prefs.getInt(k, 0);
      snprintf(k, sizeof(k), "s%di", i);  t.id      = prefs.getLong(k, -1);
      snprintf(k, sizeof(k), "s%dtn", i); t.tMin    = prefs.getFloat(k, -999);
      snprintf(k, sizeof(k), "s%dtx", i); t.tMax    = prefs.getFloat(k, 999);
      snprintf(k, sizeof(k), "s%drn", i); t.rhMin   = prefs.getFloat(k, -999);
      snprintf(k, sizeof(k), "s%drx", i); t.rhMax   = prefs.getFloat(k, 999);
      snprintf(k, sizeof(k), "s%dht", i); t.hystT   = prefs.getFloat(k, 0.5f);
      snprintf(k, sizeof(k), "s%dhr", i); t.hystRH  = prefs.getFloat(k, 3.0f);
      snprintf(k, sizeof(k), "s%dnt", i); t.notifyT  = prefs.getBool(k, true);
      snprintf(k, sizeof(k), "s%dnr", i); t.notifyRH = prefs.getBool(k, true);
      snprintf(k, sizeof(k), "s%dce", i); t.cmpEnabled = prefs.getBool(k, false);
      snprintf(k, sizeof(k), "s%dct", i); t.cmpDT   = prefs.getFloat(k, 999);
      snprintf(k, sizeof(k), "s%dcr", i); t.cmpDRH  = prefs.getFloat(k, 999);
      snprintf(k, sizeof(k), "s%dca", i); t.cmpDAH  = prefs.getFloat(k, 999);
      snprintf(k, sizeof(k), "s%dha", i); t.hystAH  = prefs.getFloat(k, 0.5f);
      snprintf(k, sizeof(k), "s%dnct", i); t.notifyCmpT  = prefs.getBool(k, true);
      snprintf(k, sizeof(k), "s%dncr", i); t.notifyCmpRH = prefs.getBool(k, true);
      snprintf(k, sizeof(k), "s%dnca", i); t.notifyCmpAH = prefs.getBool(k, true);
      n++;
      migrated = true;
    }
    if (migrated) Serial.println("Schwellenwerte: aus altem Format migriert.");
  } else {
    // Blob mit falscher Groesse (z.B. Struct-Layout hat sich geaendert) -
    // nicht vertrauenswuerdig uebernehmen, sauber bei "nichts konfiguriert"
    // neu anfangen statt mit halb kaputten Werten weiterzulaufen.
    Serial.println("Schwellenwerte: gespeicherter Blob passt nicht (Format geaendert?) - ignoriert.");
  }
  prefs.end();
  if (migrated) saveThresholds();
  Serial.printf("Schwellenwerte geladen: %d konfigurierte Sensor(en)\n", n);
}

static void saveThresholds() {
  ThreshPersist buf[MAX_THRESH];
  for (int i = 0; i < MAX_THRESH; i++) {
    Threshold &t = thresholds[i];
    ThreshPersist &p = buf[i];
    p.used = t.used;
    strncpy(p.model, t.model, sizeof(p.model) - 1);
    p.channel = t.channel; p.id = t.id;
    p.tMin = t.tMin; p.tMax = t.tMax; p.rhMin = t.rhMin; p.rhMax = t.rhMax;
    p.hystT = t.hystT; p.hystRH = t.hystRH;
    p.notifyT = t.notifyT; p.notifyRH = t.notifyRH;
    p.cmpEnabled = t.cmpEnabled;
    p.cmpDT = t.cmpDT; p.cmpDRH = t.cmpDRH; p.cmpDAH = t.cmpDAH; p.hystAH = t.hystAH;
    p.notifyCmpT = t.notifyCmpT; p.notifyCmpRH = t.notifyCmpRH; p.notifyCmpAH = t.notifyCmpAH;
    p.tsEnT = t.tsEnT; p.tsFieldT = t.tsFieldT;
    p.tsEnRH = t.tsEnRH; p.tsFieldRH = t.tsFieldRH;
    p.tsEnAH = t.tsEnAH; p.tsFieldAH = t.tsFieldAH;
    p.tsEnDT = t.tsEnDT; p.tsFieldDT = t.tsFieldDT;
    p.tsEnDRH = t.tsEnDRH; p.tsFieldDRH = t.tsFieldDRH;
    p.tsEnDAH = t.tsEnDAH; p.tsFieldDAH = t.tsFieldDAH;
  }
  prefs.begin("thresh", false);
  // clear() raeumt alte, fragmentierte Einzel-Keys aus fruehren Firmware-
  // Versionen (s0tx, s0ce, ...) mit auf - sonst bliebe der bereits belegte
  // Flash-Platz dauerhaft verschwendet, obwohl die Keys nie wieder gelesen
  // werden.
  prefs.clear();
  size_t wrote = prefs.putBytes("all", buf, sizeof(buf));
  prefs.end();
  if (wrote != sizeof(buf))
    Serial.println("FEHLER: Schwellenwerte konnten nicht gespeichert werden (NVS voll?)");
}

// Prueft einen frisch aktualisierten Sensor gegen seine (falls vorhandene)
// Schwellenwerte und stoesst bei Ueber-/Unterschreiten bzw. Rueckkehr in den
// Normalbereich (jeweils mit Hysterese) eine ntfy-Meldung an. Ohne USE_WIFI
// no-op (Funktion bleibt bestehen, damit der Aufrufer nicht bedingt kompiliert
// werden muss).
static void checkThresholds(const Sensor &s) {
#if USE_WIFI
  Threshold *c = findThreshold(s.model, s.channel, s.id);
  if (!c) return;
  char key[40];
  makeDisplayLabel(s.model, s.channel, s.id, key, sizeof(key));
  char msg[110];

  // notifyT/notifyRH schalten NUR den Push ab - der Alarmzustand (fuer die
  // Faerbung auf der Startseite) wird immer aktualisiert, unabhaengig davon.
  if (c->tMax < 900) {
    if (!c->alarmTHigh && s.t > c->tMax) {
      c->alarmTHigh = true;
      snprintf(msg, sizeof(msg), "%s: Temperatur %.1f C ueber Grenzwert %.1f C", key, s.t, c->tMax);
      if (c->notifyT) notify(msg);
    } else if (c->alarmTHigh && s.t < c->tMax - c->hystT) {
      c->alarmTHigh = false;
      snprintf(msg, sizeof(msg), "%s: Temperatur wieder unter %.1f C (jetzt %.1f C)", key, c->tMax, s.t);
      if (c->notifyT) notify(msg);
    }
  }
  if (c->tMin > -900) {
    if (!c->alarmTLow && s.t < c->tMin) {
      c->alarmTLow = true;
      snprintf(msg, sizeof(msg), "%s: Temperatur %.1f C unter Grenzwert %.1f C", key, s.t, c->tMin);
      if (c->notifyT) notify(msg);
    } else if (c->alarmTLow && s.t > c->tMin + c->hystT) {
      c->alarmTLow = false;
      snprintf(msg, sizeof(msg), "%s: Temperatur wieder ueber %.1f C (jetzt %.1f C)", key, c->tMin, s.t);
      if (c->notifyT) notify(msg);
    }
  }
  if (c->rhMax < 900) {
    if (!c->alarmRHHigh && s.rh > c->rhMax) {
      c->alarmRHHigh = true;
      snprintf(msg, sizeof(msg), "%s: Luftfeuchte %.0f%% ueber Grenzwert %.0f%%", key, s.rh, c->rhMax);
      if (c->notifyRH) notify(msg);
    } else if (c->alarmRHHigh && s.rh < c->rhMax - c->hystRH) {
      c->alarmRHHigh = false;
      snprintf(msg, sizeof(msg), "%s: Luftfeuchte wieder unter %.0f%% (jetzt %.0f%%)", key, c->rhMax, s.rh);
      if (c->notifyRH) notify(msg);
    }
  }
  if (c->rhMin > -900) {
    if (!c->alarmRHLow && s.rh < c->rhMin) {
      c->alarmRHLow = true;
      snprintf(msg, sizeof(msg), "%s: Luftfeuchte %.0f%% unter Grenzwert %.0f%%", key, s.rh, c->rhMin);
      if (c->notifyRH) notify(msg);
    } else if (c->alarmRHLow && s.rh > c->rhMin + c->hystRH) {
      c->alarmRHLow = false;
      snprintf(msg, sizeof(msg), "%s: Luftfeuchte wieder ueber %.0f%% (jetzt %.0f%%)", key, c->rhMin, s.rh);
      if (c->notifyRH) notify(msg);
    }
  }

  // Batteriezustand - unabhaengig von Temp/Feuchte-Grenzen, keine Hysterese noetig
  // (ein Bit, kein Messwert der knapp an einer Schwelle schwankt).
  if (!c->alarmBatteryLow && !s.batteryOk) {
    c->alarmBatteryLow = true;
    snprintf(msg, sizeof(msg), "%s: Batterie schwach", key);
    notify(msg);
  } else if (c->alarmBatteryLow && s.batteryOk) {
    c->alarmBatteryLow = false;
    snprintf(msg, sizeof(msg), "%s: Batterie wieder ok (getauscht?)", key);
    notify(msg);
  }
#endif
}

/* ------------------------------------------------------------------ *
 *  2c) REFERENZSENSOR-VERGLEICH                                     *
 *      Ein Sensor gilt als Referenz (z.B. "Außen"); beliebige andere *
 *      Sensoren koennen unter /settings pro Sensor einen Vergleich   *
 *      dazu aktivieren: |Temp-Differenz|, |rF-Differenz| und/oder    *
 *      |abs.Feuchte-Differenz| zur Referenz, mit eigener Hysterese.  *
 *      Ueberschreitet die Differenz den eingetragenen Grenzwert,     *
 *      geht wie bei den normalen Schwellenwerten eine ntfy-Meldung   *
 *      raus (und eine Entwarnung, sobald sie wieder drunter faellt). *
 * ------------------------------------------------------------------ */
struct RefSensor {
  bool used    = false;
  char model[32] = {0};
  int  channel = 0;
  long id      = -1;
};
static RefSensor refSensor;
static Preferences prefsRef;

static void loadRefSensor() {
  prefsRef.begin("refsensor", true);
  refSensor.used = prefsRef.getBool("u", false);
  if (refSensor.used) {
    String m = prefsRef.getString("m", "");
    strncpy(refSensor.model, m.c_str(), sizeof(refSensor.model) - 1);
    refSensor.channel = prefsRef.getInt("c", 0);
    refSensor.id      = prefsRef.getLong("i", -1);
  }
  prefsRef.end();
  if (refSensor.used) {
    char label[40];
    makeSensorKey(refSensor.model, refSensor.channel, refSensor.id, label, sizeof(label));
    Serial.printf("Referenzsensor geladen: %s\n", label);
  }
}

static void saveRefSensor() {
  prefsRef.begin("refsensor", false);
  prefsRef.putBool("u", refSensor.used);
  if (refSensor.used) {
    prefsRef.putString("m", refSensor.model);
    prefsRef.putInt("c", refSensor.channel);
    prefsRef.putLong("i", refSensor.id);
  }
  prefsRef.end();
}

// Aktuell bekannter Live-Sensor mit dieser Identitaet, oder nullptr.
static const Sensor *findLiveSensor(const char *model, int channel, long id) {
  for (int i = 0; i < sensorCount; i++) {
    Sensor &s = sensors[i];
    if (s.channel == channel && s.id == id && strcmp(s.model, model) == 0) return &s;
  }
  return nullptr;
}

// "changed" ist der Sensor, der gerade einen frischen Wert gemeldet hat - kann
// sowohl die Referenz selbst sein als auch einer der Datensensoren. Geprueft
// wird jeder Threshold-Slot mit aktivem Vergleich, der entweder "changed" IST
// oder dessen Gegenstueck (die Referenz) sich gerade geaendert hat, damit der
// Vergleich unabhaengig davon aktuell bleibt, welcher der beiden Sensoren
// zuletzt gesendet hat.
static void checkComparisons(const Sensor &changed) {
#if USE_WIFI
  if (!refSensor.used) return;
  bool changedIsRef = changed.channel == refSensor.channel && changed.id == refSensor.id &&
                       strcmp(changed.model, refSensor.model) == 0;

  for (int i = 0; i < MAX_THRESH; i++) {
    Threshold &c = thresholds[i];
    if (!c.used || !c.cmpEnabled) continue;
    bool cIsRef = c.channel == refSensor.channel && c.id == refSensor.id &&
                  strcmp(c.model, refSensor.model) == 0;
    if (cIsRef) continue;   // Referenz mit sich selbst vergleichen ist sinnlos
    bool cIsChanged = c.channel == changed.channel && c.id == changed.id &&
                       strcmp(c.model, changed.model) == 0;
    if (!changedIsRef && !cIsChanged) continue;

    const Sensor *dataS = changedIsRef ? findLiveSensor(c.model, c.channel, c.id) : &changed;
    const Sensor *refS  = changedIsRef ? &changed
                                        : findLiveSensor(refSensor.model, refSensor.channel, refSensor.id);
    if (!dataS || !refS) continue;

    char dataLabel[40], refLabel[40];
    makeDisplayLabel(dataS->model, dataS->channel, dataS->id, dataLabel, sizeof(dataLabel));
    makeDisplayLabel(refS->model, refS->channel, refS->id, refLabel, sizeof(refLabel));
    char msg[140];

    // notifyCmpT/RH/AH schalten NUR den Push ab - der Alarmzustand (fuer die
    // Faerbung der Vergleichszeile auf der Startseite) laeuft immer weiter.
    if (c.cmpDT < 900) {
      float d = fabsf(dataS->t - refS->t);
      if (!c.alarmCmpT && d > c.cmpDT) {
        c.alarmCmpT = true;
        snprintf(msg, sizeof(msg), "%s vs %s: Temp.-Differenz %.1f C ueber Grenzwert %.1f C",
                 dataLabel, refLabel, d, c.cmpDT);
        if (c.notifyCmpT) notify(msg);
      } else if (c.alarmCmpT && d < c.cmpDT - c.hystT) {
        c.alarmCmpT = false;
        snprintf(msg, sizeof(msg), "%s vs %s: Temp.-Differenz wieder unter %.1f C (jetzt %.1f C)",
                 dataLabel, refLabel, c.cmpDT, d);
        if (c.notifyCmpT) notify(msg);
      }
    }
    if (c.cmpDRH < 900) {
      float d = fabsf(dataS->rh - refS->rh);
      if (!c.alarmCmpRH && d > c.cmpDRH) {
        c.alarmCmpRH = true;
        snprintf(msg, sizeof(msg), "%s vs %s: Feuchte-Differenz %.0f%% ueber Grenzwert %.0f%%",
                 dataLabel, refLabel, d, c.cmpDRH);
        if (c.notifyCmpRH) notify(msg);
      } else if (c.alarmCmpRH && d < c.cmpDRH - c.hystRH) {
        c.alarmCmpRH = false;
        snprintf(msg, sizeof(msg), "%s vs %s: Feuchte-Differenz wieder unter %.0f%% (jetzt %.0f%%)",
                 dataLabel, refLabel, c.cmpDRH, d);
        if (c.notifyCmpRH) notify(msg);
      }
    }
    if (c.cmpDAH < 900) {
      float d = fabsf(absoluteHumidity(dataS->t, dataS->rh) - absoluteHumidity(refS->t, refS->rh));
      if (!c.alarmCmpAH && d > c.cmpDAH) {
        c.alarmCmpAH = true;
        snprintf(msg, sizeof(msg), "%s vs %s: abs.Feuchte-Differenz %.1fg/m3 ueber Grenzwert %.1fg/m3",
                 dataLabel, refLabel, d, c.cmpDAH);
        if (c.notifyCmpAH) notify(msg);
      } else if (c.alarmCmpAH && d < c.cmpDAH - c.hystAH) {
        c.alarmCmpAH = false;
        snprintf(msg, sizeof(msg), "%s vs %s: abs.Feuchte-Differenz wieder unter %.1fg/m3 (jetzt %.1fg/m3)",
                 dataLabel, refLabel, c.cmpDAH, d);
        if (c.notifyCmpAH) notify(msg);
      }
    }
  }

  // Globale Luftungsempfehlung neu ermitteln: ueber ALLE Slots pruefen, ob
  // irgendein Sensor gerade "feuchter als die Referenz und ueber dem ΔAH-
  // Grenzwert" ist (gleiche Bedingung wie fuer den gruenen Hinweis auf der
  // Startseite) - nur dann hilft Lueften tatsaechlich, nur dann soll die LED
  // leuchten. Bewusst am Ende einmal ueber alles gescannt statt inkrementell
  // gepflegt, weil ein einzelner Funkempfang mehrere Slots betreffen kann
  // (Referenzsensor selbst gesendet -> alle verglichenen Sensoren aendern
  // sich gleichzeitig) und ein voller Scan bei MAX_THRESH=8 trivial billig ist.
  ventilationRecommended = false;
  for (int i = 0; i < MAX_THRESH; i++) {
    Threshold &c = thresholds[i];
    if (!c.used || !c.cmpEnabled || !c.alarmCmpAH) continue;
    const Sensor *dataS = findLiveSensor(c.model, c.channel, c.id);
    const Sensor *refS2 = findLiveSensor(refSensor.model, refSensor.channel, refSensor.id);
    if (!dataS || !refS2) continue;
    if (absoluteHumidity(dataS->t, dataS->rh) > absoluteHumidity(refS2->t, refS2->rh)) {
      ventilationRecommended = true;
      break;
    }
  }
#endif
}

// Wie lange ohne Empfang, bevor ein konfigurierter Sensor als "verstummt" gilt.
static const uint32_t SIGNAL_LOST_SEC = 15 * 60;

// Laeuft periodisch (nicht bei jedem Funkempfang, sondern zeitgesteuert aus
// loop()) durch alle Sensoren MIT Schwellenwert-Konfiguration und meldet per
// ntfy, wenn seit SIGNAL_LOST_SEC nichts mehr von ihnen kam - bzw. wenn der
// Empfang danach wieder da ist. Sensoren ohne konfigurierte Schwellenwerte
// werden bewusst nicht ueberwacht (nur "der Sender mit Schwellenwerten").
static void checkSignalLoss() {
#if USE_WIFI
  for (int i = 0; i < MAX_THRESH; i++) {
    Threshold &c = thresholds[i];
    if (!c.used) continue;
    // Nur fuer Sensoren, die unter /alle als "eigen" markiert sind - sonst
    // wuerde z.B. eine versehentlich konfigurierte Fehldekodierung ebenfalls
    // eine "kein Empfang"-Meldung ausloesen, obwohl sie nie wiederkommt.
    // Ausnahme: solange gar nichts markiert ist (Erstbenutzung, visibleSensors()
    // zeigt dann uebergangsweise alles), gilt auch hier weiter alles als relevant.
    if (anyOwned() && !isOwned(c.model, c.channel, c.id)) continue;

    const Sensor *sp = nullptr;
    for (int j = 0; j < sensorCount; j++) {
      if (sensors[j].channel == c.channel && sensors[j].id == c.id &&
          strcmp(sensors[j].model, c.model) == 0) { sp = &sensors[j]; break; }
    }
    if (!sp) continue;   // Sensor noch nie live gesehen (sollte nicht vorkommen)

    uint32_t age = ageSec(*sp);
    char key[40];
    makeDisplayLabel(c.model, c.channel, c.id, key, sizeof(key));
    char msg[100];

    if (!c.alarmSignalLost && age >= SIGNAL_LOST_SEC) {
      c.alarmSignalLost = true;
      snprintf(msg, sizeof(msg), "%s: kein Empfang seit %lu min", key, (unsigned long)(age / 60));
      notify(msg);
    } else if (c.alarmSignalLost && age < SIGNAL_LOST_SEC) {
      c.alarmSignalLost = false;
      snprintf(msg, sizeof(msg), "%s: Empfang wieder da", key);
      notify(msg);
    }
  }
#endif
}

/* ------------------------------------------------------------------ *
 *  2c) HISTORIE                                                     *
 *      Ringpuffer je Sensor mit den letzten Messwerten, rein im RAM  *
 *      (kein Flash-Verschleiss) - geht bei Neustart/Stromausfall     *
 *      verloren, reicht aber fuer einen kurzen Verlauf auf der       *
 *      Weboberfläche (ein Punkt pro Empfang, alle ~70-90s -> HIST_LEN*
 *      Punkte decken je nach Sensor grob 1-1,5 Stunden ab).          *
 * ------------------------------------------------------------------ */
struct HistPoint {
  uint32_t t    = 0;     // Sekunden seit Boot
  float    temp = NAN;
  float    rh   = NAN;
};

static const int HIST_LEN = 60;   // Punkte je Sensor im Ringpuffer
static const int MAX_HIST = 8;    // max. gleichzeitig mit Historie gefuehrte Sensoren

struct History {
  bool      used    = false;
  char      model[32] = {0};
  int       channel = 0;
  long      id      = -1;
  HistPoint pts[HIST_LEN];
  int       head  = 0;   // naechster Schreibindex
  int       count = 0;   // gueltige Punkte (<= HIST_LEN)
};

static History histories[MAX_HIST];

static History *findHistory(const char *model, int channel, long id) {
  for (int i = 0; i < MAX_HIST; i++) {
    History &h = histories[i];
    if (h.used && h.channel == channel && h.id == id && strcmp(h.model, model) == 0)
      return &h;
  }
  return nullptr;
}

static History *getOrCreateHistory(const char *model, int channel, long id) {
  History *h = findHistory(model, channel, id);
  if (h) return h;
  for (int i = 0; i < MAX_HIST; i++) {
    if (!histories[i].used) {
      strncpy(histories[i].model, model, sizeof(histories[i].model) - 1);
      histories[i].channel = channel;
      histories[i].id = id;
      histories[i].used = true;
      histories[i].head = 0;
      histories[i].count = 0;
      return &histories[i];
    }
  }
  return nullptr;   // alle Slots belegt
}

static void recordHistory(const Sensor &s) {
  History *h = getOrCreateHistory(s.model, s.channel, s.id);
  if (!h) return;
  HistPoint &p = h->pts[h->head];
  p.t = millis() / 1000;
  p.temp = s.t;
  p.rh = s.rh;
  h->head = (h->head + 1) % HIST_LEN;
  if (h->count < HIST_LEN) h->count++;
}

/* ------------------------------------------------------------------ *
 *  2d) EIGENE SENSOREN + ANZEIGENAMEN                                *
 *      Ohne Whitelist landet frueher oder spaeter auch jeder          *
 *      Nachbarsensor und jede Fehldekodierung (billige TH-Sensoren,   *
 *      OOK-Rauschen) mit eigener Karte auf der Startseite - das       *
 *      macht die Seite voll und langsam. Diese Liste (persistiert,    *
 *      analog zu Threshold/History) merkt sich, welche der je         *
 *      gesehenen Sensoren tatsaechlich "meine" sind; Startseite und   *
 *      OLED zeigen dann nur noch diese. Verwaltet unter /alle - dort  *
 *      laesst sich pro Sensor auch ein Anzeigename vergeben (z.B.     *
 *      "Garten" statt "Bresser-3CH"), der ueberall dort erscheint,    *
 *      wo sonst der Herstellername stuende (Web, OLED, ntfy).         *
 *      Ist die Liste leer (Erstbenutzung), wird uebergangsweise       *
 *      wieder alles gezeigt, statt einer leeren Seite.                *
 * ------------------------------------------------------------------ */
struct OwnedSensor {
  bool used    = false;   // Slot belegt (Name und/oder "eigen" gesetzt)?
  bool eigen   = false;   // Haekchen "eigen" - steuert Sichtbarkeit/Signalverlust
  char model[32] = {0};
  int  channel = 0;
  long id      = -1;
  char name[21] = {0};    // Anzeigename, leer = Herstellername verwenden
};

static const int MAX_OWNED = 8;
static OwnedSensor owned[MAX_OWNED];
static Preferences prefsOwned;

static bool isOwned(const char *model, int channel, long id) {
  for (int i = 0; i < MAX_OWNED; i++) {
    OwnedSensor &o = owned[i];
    if (o.used && o.eigen && o.channel == channel && o.id == id && strcmp(o.model, model) == 0)
      return true;
  }
  return false;
}

static bool anyOwned() {
  for (int i = 0; i < MAX_OWNED; i++) if (owned[i].used && owned[i].eigen) return true;
  return false;
}

// Anzeigename fuer diesen Sensor, falls einer vergeben ist. Liefert false
// (und laesst out unangetastet), wenn keiner gesetzt ist - Aufrufer soll
// dann auf den Herstellernamen zurueckfallen.
static bool getDisplayName(const char *model, int channel, long id, char *out, size_t outLen) {
  for (int i = 0; i < MAX_OWNED; i++) {
    OwnedSensor &o = owned[i];
    if (o.used && o.name[0] && o.channel == channel && o.id == id && strcmp(o.model, model) == 0) {
      strncpy(out, o.name, outLen - 1);
      out[outLen - 1] = 0;
      return true;
    }
  }
  return false;
}

// Aus /alle: "eigen"-Haekchen und/oder Anzeigename fuer einen Sensor setzen.
// name==nullptr oder leer -> Name loeschen (Herstellername wieder anzeigen).
// Ist am Ende weder eigen=true noch ein Name gesetzt, wird der Slot wieder
// freigegeben (wie vorher bei setOwned()).
static void setSensorMeta(const char *model, int channel, long id, bool eigen, const char *name) {
  int slot = -1, freeSlot = -1, staleSlot = -1;
  for (int i = 0; i < MAX_OWNED; i++) {
    OwnedSensor &o = owned[i];
    if (o.used && o.channel == channel && o.id == id && strcmp(o.model, model) == 0) { slot = i; break; }
    if (freeSlot < 0 && !o.used) freeSlot = i;
    // Eintrag eines Sensors, der gerade gar nicht empfangen wird (z.B. alte ID nach Batteriewechsel)
    if (staleSlot < 0 && o.used && !findLiveSensor(o.model, o.channel, o.id)) staleSlot = i;
  }
  bool wantsSlot = eigen || (name && name[0]);
  if (slot < 0) {
    if (!wantsSlot) return;                   // nichts zu speichern
    if (freeSlot < 0) freeSlot = staleSlot;   // alle belegt: veralteten Eintrag ueberschreiben
    if (freeSlot < 0) return;                 // wirklich alle 8 Slots mit aktiven Sensoren belegt
    slot = freeSlot;
    OwnedSensor &o = owned[slot];
    strncpy(o.model, model, sizeof(o.model) - 1);
    o.channel = channel;
    o.id = id;
    o.used = true;
  }
  OwnedSensor &o = owned[slot];
  o.eigen = eigen;
  strncpy(o.name, name ? name : "", sizeof(o.name) - 1);
  o.name[sizeof(o.name) - 1] = 0;
  if (!o.eigen && !o.name[0]) o.used = false;   // weder eigen noch benannt -> Slot freigeben
}

// Wie ThreshPersist bei den Schwellenwerten: ein einziger Byte-Blob fuer alle
// MAX_OWNED Slots statt ~6 Einzel-Keys pro Slot, um das NVS nicht unnoetig
// mit vielen kleinen Eintraegen zu fuellen.
struct OwnedPersist {
  bool used = false, eigen = false;
  char model[32] = {0};
  int  channel = 0;
  long id = -1;
  char name[21] = {0};
};

static void saveOwned();   // Vorwaertsdeklaration fuer die Migration unten

static void loadOwned() {
  prefsOwned.begin("owned", true);
  OwnedPersist buf[MAX_OWNED];
  size_t got = prefsOwned.getBytes("all", buf, sizeof(buf));
  int n = 0;
  bool migrated = false;
  if (got == sizeof(buf)) {
    for (int i = 0; i < MAX_OWNED; i++) {
      OwnedPersist &p = buf[i];
      OwnedSensor &o = owned[i];
      o.used = p.used;
      if (!o.used) continue;
      n++;
      strncpy(o.model, p.model, sizeof(o.model) - 1);
      o.channel = p.channel; o.id = p.id; o.eigen = p.eigen;
      strncpy(o.name, p.name, sizeof(o.name) - 1);
    }
  } else if (got == 0) {
    // Neuer Blob-Key ("all") existiert noch nicht - entweder Erststart, oder
    // Umstieg von der alten Einzel-Key-Version (o0u, o0m, ...). Alte Keys
    // versuchsweise lesen, damit Namen/"eigen"-Haekchen aus fruehreren
    // Firmware-Versionen nicht verloren gehen; anschliessend sofort im neuen,
    // kompakten Format zurueckschreiben (raeumt die alten Keys mit auf).
    for (int i = 0; i < MAX_OWNED; i++) {
      char k[8];
      snprintf(k, sizeof(k), "o%du", i);
      if (!prefsOwned.getBool(k, false)) continue;
      OwnedSensor &o = owned[i];
      o.used = true;
      snprintf(k, sizeof(k), "o%dm", i);
      String m = prefsOwned.getString(k, "");
      strncpy(o.model, m.c_str(), sizeof(o.model) - 1);
      snprintf(k, sizeof(k), "o%dc", i); o.channel = prefsOwned.getInt(k, 0);
      snprintf(k, sizeof(k), "o%di", i); o.id      = prefsOwned.getLong(k, -1);
      snprintf(k, sizeof(k), "o%de", i); o.eigen   = prefsOwned.getBool(k, false);
      snprintf(k, sizeof(k), "o%dn", i);
      String nm = prefsOwned.getString(k, "");
      strncpy(o.name, nm.c_str(), sizeof(o.name) - 1);
      n++;
      migrated = true;
    }
    if (migrated) Serial.println("Eigene Sensoren: aus altem Format migriert.");
  } else {
    Serial.println("Eigene Sensoren: gespeicherter Blob passt nicht (Format geaendert?) - ignoriert.");
  }
  prefsOwned.end();
  if (migrated) saveOwned();
  Serial.printf("Eigene Sensoren geladen: %d\n", n);
}

static void saveOwned() {
  OwnedPersist buf[MAX_OWNED];
  for (int i = 0; i < MAX_OWNED; i++) {
    OwnedSensor &o = owned[i];
    OwnedPersist &p = buf[i];
    p.used = o.used; p.eigen = o.eigen;
    strncpy(p.model, o.model, sizeof(p.model) - 1);
    p.channel = o.channel; p.id = o.id;
    strncpy(p.name, o.name, sizeof(p.name) - 1);
  }
  prefsOwned.begin("owned", false);
  prefsOwned.clear();   // alte Einzel-Keys aus fruehren Versionen mit aufraeumen
  size_t wrote = prefsOwned.putBytes("all", buf, sizeof(buf));
  prefsOwned.end();
  if (wrote != sizeof(buf))
    Serial.println("FEHLER: Eigene Sensoren konnten nicht gespeichert werden (NVS voll?)");
}

/* ------------------------------------------------------------------ *
 *  2e) EMPFANGSEMPFINDLICHKEIT                                      *
 *      rtl_433_ESP erkennt ein Signal, wenn der Pegel um            *
 *      RSSI_THRESHOLD dB ueber dem laufend nachkalibrierten          *
 *      Rausch-Grundpegel liegt (Default 9). Kleinerer Wert = mehr    *
 *      Reichweite, aber auch mehr Fehldekodierungen aus dem          *
 *      Rauschen selbst - deshalb hier einstellbar statt fest, per    *
 *      /settings live per rf.setRSSIThreshold() nachjustierbar,      *
 *      ohne neu zu flashen. Persistiert im NVS (Namespace "radio").  *
 * ------------------------------------------------------------------ */
static const int RSSI_SENS_DEFAULT = 9;
static const int RSSI_SENS_MIN = 3, RSSI_SENS_MAX = 20;
static int rssiSensitivity = RSSI_SENS_DEFAULT;
static Preferences prefsRadio;

// Display-Anzeige (waehlbar im Untermenue /display, im selben NVS-Namespace "radio").
// Welche Anzeigen sich auf dem OLED abwechseln, steht als Bitmaske in "dmask":
//   DM_CARDS = Sensorkarten (je Sensor eine Seite), DM_GAUGE = Tacho (momentane
//   Differenz Innen - Aussen), DM_GRAPH = Verlauf (Kurven Innen/Aussen/Differenz).
// Jede Anzeige hat ihre eigene Anzeigedauer in Zehntelsekunden ("dtC"/"dtG"/"dtV";
// bei den Sensorkarten gilt sie je Karte). Startwert, solange nichts gewaehlt wurde:
// Umgebung empfaenger2 (-DOLED_GRAPH_ONLY=1) startet mit dem Verlauf, alle anderen
// mit Sensorkarten + Verlauf. Frueher gab es nur eine Einzelauswahl ("disp"), die beim
// ersten Start mit dieser Version in die Bitmaske uebernommen wird.
#ifndef OLED_GRAPH_ONLY
#define OLED_GRAPH_ONLY 0
#endif
enum { DM_CARDS = 1, DM_GAUGE = 2, DM_GRAPH = 4, DM_COMPARE = 8,
       DM_AMPEL = 16, DM_MINMAX = 32, DM_MOLD = 64, DM_RX = 128, DM_SYS = 256, DM_CLOCK = 512 };
static const int DM_ALL = 1023;
static const int DM_DEFAULT = OLED_GRAPH_ONLY ? DM_GRAPH : (DM_CARDS | DM_GRAPH);
static int dispMask = DM_DEFAULT;
static const int DISP_DUR_DEFAULT = 40, DISP_DUR_MIN = 5, DISP_DUR_MAX = 600;   // Zehntelsekunden: 4,0 s, 0,5 ... 60,0 s
static int dispDurCards = DISP_DUR_DEFAULT, dispDurGauge = DISP_DUR_DEFAULT, dispDurGraph = DISP_DUR_DEFAULT;
static int dispDurCmp = DISP_DUR_DEFAULT;
static int dispDurAmpel = DISP_DUR_DEFAULT, dispDurMm = DISP_DUR_DEFAULT, dispDurMold = DISP_DUR_DEFAULT;
static int dispDurRx = DISP_DUR_DEFAULT, dispDurSys = DISP_DUR_DEFAULT, dispDurClock = DISP_DUR_DEFAULT;

// Sensor-Vergleich (DM_COMPARE): dieselbe Groesse von zwei Sensoren nebeneinander, optional mit
// Differenz. Jede Seite ist entweder "Innen" (Sensor mit aktivem Vergleich), "Aussen"
// (Referenzsensor) oder ein bestimmter Sensor (Modell+Kanal+ID). Schluessel kA/kB/qK/kD/dtK.
struct SensorSel { uint8_t kind; char model[32]; int32_t channel; int32_t id; };   // kind: 0 Innen, 1 Aussen, 2 bestimmt
static SensorSel cmpA = {0, {0}, 0, 0}, cmpB = {1, {0}, 0, 0};
static int  dispQK = Q_AH;         // Groesse des Vergleichs
static bool cmpDiff = true;        // Differenz A - B als dritte Spalte

// Welche Werte die Display-Seiten zeigen (Q_*-Konstanten): Sensorkarte Wert 1/2/3
// (qL/qR/qX; Wert 3 = Q_COUNT heisst "nicht anzeigen" -> zwei grosse Spalten statt drei),
// Tacho und Verlauf (qG/qV; nur Groessen, bei denen eine Differenz Innen-Aussen
// sinnvoll ist: T, rF, aF, Taupunkt).
static int dispQL = Q_T, dispQR = Q_RH, dispQX = Q_TD, dispQG = Q_AH, dispQV = Q_AH;
// Weitere Anzeigen (Schluessel in Klammern):
//  Lueften-Ampel (qA, adt): Groesse der Differenz Innen-Aussen (nur mit Urteil: abs. Feuchte oder
//    Taupunkt), optional mit den Einzelwerten Innen/Aussen darunter.
//  Min/Max (qM, mW): Groesse und welche Seite (0 beide, 1 Innen, 2 Aussen).
//  Schimmelrisiko (hS, qH1, qH2): Sensor und die beiden gezeigten Werte, Urteil aus rel. Feuchte/Taupunktabstand.
//  Empfangs-Uebersicht (rC): rechte Spalte 0 Alter, 1 dBm, 2 Anzahl.
//  Systemseite (sM): Bitmaske der Zeilen (1 WLAN, 2 IP, 4 Signal, 8 Laufzeit, 16 Speicher, 32 Telegramme).
//  Uhr (qC): Wert von Innen/Aussen unter der Uhrzeit (Q_COUNT = keiner).
//  Trend (tC, tG, tW): Pfeile auf Sensorkarten / im Tacho und in der Ampel, Zeitraum in Minuten.
static int  dispQA = Q_AH;  static bool ampelDetail = true;
static int  dispQM = Q_AH, mmWho = 0;
static SensorSel moldSel = {0, {0}, 0, 0};
static int  dispQH1 = Q_RH, dispQH2 = Q_SPREAD;
static int  rxCol = 0;
static int  sysMask = 2 | 4 | 8 | 16;
static int  dispQC = Q_T;
static int  trendCards = 1, trendGauge = 1, trendMin = 30;

static bool qAllowedForDiff(int q) { return q == Q_T || q == Q_RH || q == Q_AH || q == Q_TD; }
static int clampDur(int tenths) {
  return tenths < DISP_DUR_MIN ? DISP_DUR_MIN : (tenths > DISP_DUR_MAX ? DISP_DUR_MAX : tenths);
}

static void loadRadioSettings() {
  prefsRadio.begin("radio", true);
  rssiSensitivity = prefsRadio.getInt("rssi", RSSI_SENS_DEFAULT);
  dispMask        = prefsRadio.getInt("dmask", -1);
  int legacy      = prefsRadio.getInt("disp", -1);      // alte Einzelauswahl (0 Karten, 1 Verlauf, 2 beides, 3 Tacho, 4 alles)
  dispDurCards    = prefsRadio.getInt("dtC", DISP_DUR_DEFAULT);
  dispDurGauge    = prefsRadio.getInt("dtG", DISP_DUR_DEFAULT);
  dispDurGraph    = prefsRadio.getInt("dtV", DISP_DUR_DEFAULT);
  dispDurCmp      = prefsRadio.getInt("dtK", DISP_DUR_DEFAULT);
  dispQK          = prefsRadio.getInt("qK", Q_AH);
  cmpDiff         = prefsRadio.getBool("kD", true);
  if (prefsRadio.getBytes("kA", &cmpA, sizeof(cmpA)) != sizeof(cmpA)) { cmpA = {0, {0}, 0, 0}; }
  if (prefsRadio.getBytes("kB", &cmpB, sizeof(cmpB)) != sizeof(cmpB)) { cmpB = {1, {0}, 0, 0}; }
  dispDurAmpel    = prefsRadio.getInt("dtA", DISP_DUR_DEFAULT);
  dispDurMm       = prefsRadio.getInt("dtM", DISP_DUR_DEFAULT);
  dispDurMold     = prefsRadio.getInt("dtH", DISP_DUR_DEFAULT);
  dispDurRx       = prefsRadio.getInt("dtR", DISP_DUR_DEFAULT);
  dispDurSys      = prefsRadio.getInt("dtY", DISP_DUR_DEFAULT);
  dispDurClock    = prefsRadio.getInt("dtU", DISP_DUR_DEFAULT);
  dispQA          = prefsRadio.getInt("qA", Q_AH);
  ampelDetail     = prefsRadio.getBool("adt", true);
  dispQM          = prefsRadio.getInt("qM", Q_AH);
  mmWho           = prefsRadio.getInt("mW", 0);
  dispQH1         = prefsRadio.getInt("qH1", Q_RH);
  dispQH2         = prefsRadio.getInt("qH2", Q_SPREAD);
  rxCol           = prefsRadio.getInt("rC", 0);
  sysMask         = prefsRadio.getInt("sM", 2 | 4 | 8 | 16);
  dispQC          = prefsRadio.getInt("qC", Q_T);
  trendCards      = prefsRadio.getInt("tC", 1);
  trendGauge      = prefsRadio.getInt("tG", 1);
  trendMin        = prefsRadio.getInt("tW", 30);
  if (prefsRadio.getBytes("hS", &moldSel, sizeof(moldSel)) != sizeof(moldSel)) { moldSel = {0, {0}, 0, 0}; }
  dispQL          = prefsRadio.getInt("qL", Q_T);
  dispQR          = prefsRadio.getInt("qR", Q_RH);
  dispQX          = prefsRadio.getInt("qX", Q_TD);
  dispQG          = prefsRadio.getInt("qG", Q_AH);
  dispQV          = prefsRadio.getInt("qV", Q_AH);
  prefsRadio.end();
  if (rssiSensitivity < RSSI_SENS_MIN) rssiSensitivity = RSSI_SENS_MIN;
  if (rssiSensitivity > RSSI_SENS_MAX) rssiSensitivity = RSSI_SENS_MAX;
  if (dispMask < 0) {                                    // noch keine Bitmaske gespeichert
    static const int fromLegacy[5] = {DM_CARDS, DM_GRAPH, DM_CARDS | DM_GRAPH, DM_GAUGE, DM_CARDS | DM_GAUGE | DM_GRAPH};
    dispMask = (legacy >= 0 && legacy <= 4) ? fromLegacy[legacy] : DM_DEFAULT;
  }
  dispMask &= DM_ALL;
  if (dispMask == 0) dispMask = DM_CARDS;
  dispDurCards = clampDur(dispDurCards);
  dispDurGauge = clampDur(dispDurGauge);
  dispDurGraph = clampDur(dispDurGraph);
  dispDurCmp = clampDur(dispDurCmp);
  if (dispQK < 0 || dispQK >= Q_COUNT) dispQK = Q_AH;
  dispDurAmpel = clampDur(dispDurAmpel);
  dispDurMm    = clampDur(dispDurMm);
  dispDurMold  = clampDur(dispDurMold);
  dispDurRx    = clampDur(dispDurRx);
  dispDurSys   = clampDur(dispDurSys);
  dispDurClock = clampDur(dispDurClock);
  if (dispQA != Q_AH && dispQA != Q_TD) dispQA = Q_AH;
  if (!qAllowedForDiff(dispQM)) dispQM = Q_AH;
  if (mmWho < 0 || mmWho > 2) mmWho = 0;
  if (dispQH1 < 0 || dispQH1 >= Q_COUNT) dispQH1 = Q_RH;
  if (dispQH2 < 0 || dispQH2 >= Q_COUNT) dispQH2 = Q_SPREAD;
  if (rxCol < 0 || rxCol > 2) rxCol = 0;
  sysMask &= 63;
  if (dispQC < 0 || dispQC > Q_COUNT) dispQC = Q_T;
  trendCards = trendCards ? 1 : 0;
  trendGauge = trendGauge ? 1 : 0;
  if (trendMin != 15 && trendMin != 30 && trendMin != 60 && trendMin != 120) trendMin = 30;
  if (moldSel.kind > 2) moldSel = {0, {0}, 0, 0};
  moldSel.model[sizeof(moldSel.model) - 1] = 0;
  if (cmpA.kind > 2) cmpA = {0, {0}, 0, 0};
  if (cmpB.kind > 2) cmpB = {1, {0}, 0, 0};
  cmpA.model[sizeof(cmpA.model) - 1] = 0;
  cmpB.model[sizeof(cmpB.model) - 1] = 0;
  if (dispQL < 0 || dispQL >= Q_COUNT) dispQL = Q_T;
  if (dispQR < 0 || dispQR >= Q_COUNT) dispQR = Q_RH;
  if (dispQX < 0 || dispQX > Q_COUNT) dispQX = Q_TD;
  if (!qAllowedForDiff(dispQG)) dispQG = Q_AH;
  if (!qAllowedForDiff(dispQV)) dispQV = Q_AH;
}

static void saveRadioSettings() {
  prefsRadio.begin("radio", false);
  prefsRadio.putInt("rssi", rssiSensitivity);
  prefsRadio.putInt("dmask", dispMask);
  prefsRadio.putInt("dtC", dispDurCards);
  prefsRadio.putInt("dtG", dispDurGauge);
  prefsRadio.putInt("dtV", dispDurGraph);
  prefsRadio.putInt("dtK", dispDurCmp);
  prefsRadio.putInt("qK", dispQK);
  prefsRadio.putBool("kD", cmpDiff);
  prefsRadio.putBytes("kA", &cmpA, sizeof(cmpA));
  prefsRadio.putBytes("kB", &cmpB, sizeof(cmpB));
  prefsRadio.putInt("dtA", dispDurAmpel);
  prefsRadio.putInt("dtM", dispDurMm);
  prefsRadio.putInt("dtH", dispDurMold);
  prefsRadio.putInt("dtR", dispDurRx);
  prefsRadio.putInt("dtY", dispDurSys);
  prefsRadio.putInt("dtU", dispDurClock);
  prefsRadio.putInt("qA", dispQA);
  prefsRadio.putBool("adt", ampelDetail);
  prefsRadio.putInt("qM", dispQM);
  prefsRadio.putInt("mW", mmWho);
  prefsRadio.putInt("qH1", dispQH1);
  prefsRadio.putInt("qH2", dispQH2);
  prefsRadio.putInt("rC", rxCol);
  prefsRadio.putInt("sM", sysMask);
  prefsRadio.putInt("qC", dispQC);
  prefsRadio.putInt("tC", trendCards);
  prefsRadio.putInt("tG", trendGauge);
  prefsRadio.putInt("tW", trendMin);
  prefsRadio.putBytes("hS", &moldSel, sizeof(moldSel));
  prefsRadio.putInt("qL", dispQL);
  prefsRadio.putInt("qR", dispQR);
  prefsRadio.putInt("qX", dispQX);
  prefsRadio.putInt("qG", dispQG);
  prefsRadio.putInt("qV", dispQV);
  prefsRadio.end();
}

#if USE_WIFI
/* ------------------------------------------------------------------ *
 *  2f) WLAN-ZUGANGSDATEN                                             *
 *      SSID/Passwort aus include/secrets.h sind nur der Erststart-    *
 *      Fallback. Sobald einmal ueber /wifi gespeichert wurde,         *
 *      ueberschreibt der NVS-Wert das fest einkompilierte Passwort -  *
 *      so laesst sich das WLAN wechseln, ohne neu zu flashen.         *
 *      Optional ein zweites, Backup-WLAN (z.B. Handy-Hotspot) - wird  *
 *      versucht, wenn das Haupt-WLAN beim Start nicht erreichbar ist. *
 *      Findet netSetup() am Ende keins von beiden, macht das Board    *
 *      stattdessen selbst einen Hotspot auf (siehe startApFallback()  *
 *      unten), ueber den /wifi ebenfalls erreichbar ist.              *
 * ------------------------------------------------------------------ */
static char wifiSsid[33]  = WIFI_SSID;
static char wifiPass[65]  = WIFI_PASS;
static char wifiSsid2[33] = WIFI_SSID2;   // Backup-WLAN, optional - leer = keins hinterlegt
static char wifiPass2[65] = WIFI_PASS2;
static Preferences prefsWifi;

static void loadWifiCreds() {
  prefsWifi.begin("wifi", true);
  String s = prefsWifi.getString("ssid", "");
  String p = prefsWifi.getString("pass", "");
  String s2 = prefsWifi.getString("ssid2", "");
  String p2 = prefsWifi.getString("pass2", "");
  prefsWifi.end();
  if (s.length()) {
    strncpy(wifiSsid, s.c_str(), sizeof(wifiSsid) - 1);
    wifiSsid[sizeof(wifiSsid) - 1] = 0;
    strncpy(wifiPass, p.c_str(), sizeof(wifiPass) - 1);
    wifiPass[sizeof(wifiPass) - 1] = 0;
    Serial.println(F("WLAN-Zugangsdaten aus NVS geladen (ueberschreibt secrets.h)"));
  }
  if (s2.length()) {   // nur ueberschreiben, wenn im NVS ein Backup-WLAN gespeichert ist (sonst gilt secrets.h)
    strncpy(wifiSsid2, s2.c_str(), sizeof(wifiSsid2) - 1);
    wifiSsid2[sizeof(wifiSsid2) - 1] = 0;
    strncpy(wifiPass2, p2.c_str(), sizeof(wifiPass2) - 1);
    wifiPass2[sizeof(wifiPass2) - 1] = 0;
  }
}

static void saveWifiCreds(const char *ssid, const char *pass, const char *ssid2, const char *pass2) {
  prefsWifi.begin("wifi", false);
  prefsWifi.putString("ssid", ssid);
  prefsWifi.putString("pass", pass);
  prefsWifi.putString("ssid2", ssid2);
  prefsWifi.putString("pass2", pass2);
  prefsWifi.end();
}

#endif

/* ------------------------------------------------------------------ *
 *  3) SENSOR-EMPFANG                                                *
 * ------------------------------------------------------------------ */
static Sensor *findOrAddSensor(const char *model, int channel, long id) {
  for (int i = 0; i < sensorCount; i++) {
    Sensor &s = sensors[i];
    if (s.channel == channel && s.id == id && strcmp(s.model, model) == 0)
      return &s;
  }
  if (sensorCount < MAX_SENSORS) {
    Sensor &s = sensors[sensorCount++];
    strncpy(s.model, model, sizeof(s.model) - 1);
    s.channel = channel;
    s.id = id;
    s.firstSeen = millis();
    return &s;
  }
  return nullptr;   // Liste voll - neue, bisher ungesehene Sensoren werden dann ignoriert
}

// Dekodierte Meldungen OHNE Temperatur/Feuchte (z.B. Wasserleck-, Regen-, Bodenfeuchte-Sensoren)
// werden nicht als Sensorkarte gefuehrt. Damit man trotzdem sieht, ob sie ankommen, merkt sich
// die Firmware die letzten paar Meldungen und zeigt sie unter /andere an.
#define OTHER_KEEP 6
#define OTHER_LEN 256
static char     otherMsg[OTHER_KEEP][OTHER_LEN];
static uint32_t otherMs[OTHER_KEEP];
static uint32_t otherTotal = 0;
static void rememberOther(const char *m) {
  int i = otherTotal % OTHER_KEEP;
  strncpy(otherMsg[i], m, sizeof(otherMsg[i]) - 1);
  otherMsg[i][sizeof(otherMsg[i]) - 1] = 0;
  otherMs[i] = millis();
  otherTotal++;
  Serial.printf("RX-ANDERE %s\n", otherMsg[i]);
}

/* ------------------------------------------------------------------ *
 *  Wasserleck-Sensoren (Ecowitt WH55, Bresser Leakage)                *
 *  Telegramm: alarm (0/1), battery_ok (WH55: 0.2 .. 1.0), raw_value.  *
 *  Meldungen laufen ueber ntfy (force = auch waehrend der Boot-Ruhe). *
 * ------------------------------------------------------------------ */
#define LEAK_MAX      4
#define LEAK_LOST_MS  900000UL      // 15 min ohne Telegramm (WH55 sendet etwa alle 62 s)

struct LeakSensor {
  bool     used;
  char     model[32];
  int32_t  channel;
  int32_t  id;
  char     name[21];
  bool     seen;        // seit dem Start mindestens ein Telegramm empfangen
  bool     alarm;
  bool     battLow;
  bool     battEmpty;
  uint32_t battNote;    // millis() der letzten Batterie-Meldung
  bool     lost;
  int      battPct;     // -1 = unbekannt
  int      raw;         // -1 = unbekannt
  float    rssi;
  uint32_t stamp;       // millis() des letzten Telegramms
  uint32_t alarmSince;
  uint32_t count;
};
static LeakSensor leaks[LEAK_MAX];
static Preferences prefsLeak;

struct LeakPersist { char model[32]; int32_t channel; int32_t id; char name[21]; uint8_t used; };

static String leakBannerHtml();   // Definition bei den Web-Handlern

static void leakResetSlot(LeakSensor &l) {
  memset(&l, 0, sizeof(l));
  l.battPct = -1;
  l.raw = -1;
  l.rssi = NAN;
}

static void saveLeaks() {
  LeakPersist p[LEAK_MAX];
  memset(p, 0, sizeof(p));
  for (int i = 0; i < LEAK_MAX; i++) {
    p[i].used = leaks[i].used ? 1 : 0;
    strncpy(p[i].model, leaks[i].model, sizeof(p[i].model) - 1);
    p[i].channel = leaks[i].channel;
    p[i].id = leaks[i].id;
    strncpy(p[i].name, leaks[i].name, sizeof(p[i].name) - 1);
  }
  prefsLeak.begin("leak", false);
  prefsLeak.clear();
  size_t n = prefsLeak.putBytes("all", p, sizeof(p));
  prefsLeak.end();
  if (n != sizeof(p)) Serial.println(F("FEHLER: Wassersensor-Namen konnten nicht gespeichert werden (NVS voll?)"));
}

static void loadLeaks() {
  for (int i = 0; i < LEAK_MAX; i++) leakResetSlot(leaks[i]);
  LeakPersist p[LEAK_MAX];
  prefsLeak.begin("leak", true);
  size_t n = prefsLeak.getBytes("all", p, sizeof(p));
  prefsLeak.end();
  if (n != sizeof(p)) return;
  for (int i = 0; i < LEAK_MAX; i++) {
    if (!p[i].used) continue;
    leaks[i].used = true;
    strncpy(leaks[i].model, p[i].model, sizeof(leaks[i].model) - 1);
    leaks[i].channel = p[i].channel;
    leaks[i].id = p[i].id;
    strncpy(leaks[i].name, p[i].name, sizeof(leaks[i].name) - 1);
  }
}

static bool isLeakModel(const char *m) {
  return strstr(m, "WH55") || strstr(m, "Leak") || strstr(m, "leak");
}

static void leakLabel(const LeakSensor &l, char *out, size_t n) {
  if (l.name[0]) snprintf(out, n, "%s", l.name);
  else snprintf(out, n, "Wassersensor Kanal %d", (int)l.channel);
}

static bool leakAnyAlarm() {
  for (int i = 0; i < LEAK_MAX; i++)
    if (leaks[i].used && leaks[i].alarm) return true;
  return false;
}

static LeakSensor *leakFind(const char *model, int channel, long id, bool add) {
  for (int i = 0; i < LEAK_MAX; i++)
    if (leaks[i].used && leaks[i].channel == channel && leaks[i].id == id && !strcmp(leaks[i].model, model))
      return &leaks[i];
  if (!add) return nullptr;
  for (int i = 0; i < LEAK_MAX; i++) {
    if (leaks[i].used) continue;
    leakResetSlot(leaks[i]);
    leaks[i].used = true;
    strncpy(leaks[i].model, model, sizeof(leaks[i].model) - 1);
    leaks[i].channel = channel;
    leaks[i].id = id;
    saveLeaks();            // neuer Sensor: Identitaet merken
    return &leaks[i];
  }
  return nullptr;
}

static void leakNotify(const char *msg) {
#if USE_WIFI
  notify(msg, true);
#else
  Serial.println(msg);
#endif
}

static void leakUpdate(JsonDocument &doc, const char *model) {
  int  ch = doc["channel"] | 0;
  long id = doc["id"] | -1L;
  LeakSensor *l = leakFind(model, ch, id, true);
  if (!l) { Serial.println(F("Wassersensor: Liste voll - ignoriert")); return; }

  bool  alarm = (doc["alarm"] | 0) != 0;
  float bf    = doc["battery_ok"] | 1.0f;
  int   pct   = (int)(bf * 100.0f + 0.5f);
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  bool low = bf < 0.5f;
  bool empty = bf <= 0.21f;          // WH55: 0.2 = letzter Balken
  l->raw  = doc.containsKey("raw_value") ? (int)(doc["raw_value"] | -1) : -1;
  l->rssi = doc["rssi"] | NAN;
  l->battPct = pct;

  uint32_t now = millis();
  bool wasSeen = l->seen, wasAlarm = l->alarm, wasLost = l->lost, wasLow = l->battLow, wasEmpty = l->battEmpty;
  l->seen = true;
  l->lost = false;
  l->stamp = now;
  l->count++;
  l->alarm = alarm;
  l->battLow = low;
  l->battEmpty = empty;

  char lab[32];
  leakLabel(*l, lab, sizeof(lab));
  char msg[112];
  Serial.printf("WASSER  %s ch=%d id=%ld alarm=%d raw=%d batt=%d%% RSSI=%.0fdBm\n",
                model, ch, id, alarm ? 1 : 0, l->raw, pct, l->rssi);

  if (alarm && (!wasSeen || !wasAlarm)) {
    l->alarmSince = now;
    snprintf(msg, sizeof(msg), "WASSER ERKANNT: %s", lab);
    leakNotify(msg);
  } else if (!alarm && wasSeen && wasAlarm) {
    snprintf(msg, sizeof(msg), "Wasseralarm beendet, wieder trocken: %s", lab);
    leakNotify(msg);
  }
  if (empty && !wasEmpty) {
    snprintf(msg, sizeof(msg), "BATTERIE LEER (%d %%) - bitte wechseln: %s", pct, lab);
    leakNotify(msg);
    l->battNote = now;
  } else if (low && !wasLow) {
    snprintf(msg, sizeof(msg), "Batterie schwach (%d %%): %s", pct, lab);
    leakNotify(msg);
    l->battNote = now;
  } else if (low && now - l->battNote > 86400000UL) {   // taeglich erinnern, solange nicht getauscht
    snprintf(msg, sizeof(msg), "Erinnerung: Batterie %s (%d %%): %s", empty ? "LEER" : "schwach", pct, lab);
    leakNotify(msg);
    l->battNote = now;
  }
  if (wasLost) {
    snprintf(msg, sizeof(msg), "Wassersensor meldet sich wieder: %s", lab);
    leakNotify(msg);
  }
}

static void leakCheckLost() {
  uint32_t now = millis();
  for (int i = 0; i < LEAK_MAX; i++) {
    LeakSensor &l = leaks[i];
    if (!l.used || !l.seen || l.lost) continue;
    if (now - l.stamp <= LEAK_LOST_MS) continue;
    l.lost = true;
    char lab[32], msg[112];
    leakLabel(l, lab, sizeof(lab));
    snprintf(msg, sizeof(msg), "Wassersensor meldet sich nicht mehr (seit 15 min): %s", lab);
    leakNotify(msg);
  }
}

static void rtl_433_Callback(char *message) {
  // deserializeJson(char*) zerlegt den Text an Ort und Stelle - fuer /andere vorher kopieren.
  char copy[OTHER_LEN];
  strncpy(copy, message, sizeof(copy) - 1);
  copy[sizeof(copy) - 1] = 0;
  StaticJsonDocument<JSON_MSG_BUFFER> doc;
  if (deserializeJson(doc, message)) return;
  {
    const char *lm = doc["model"] | "";
    if (isLeakModel(lm) && doc.containsKey("alarm")) { leakUpdate(doc, lm); return; }
  }
  if (!doc.containsKey("temperature_C") || !doc.containsKey("humidity")) { rememberOther(copy); return; }

  float t  = doc["temperature_C"] | NAN;
  float rh = doc["humidity"]      | NAN;
  if (isnan(t) || isnan(rh) || rh <= 0.0f || rh > 100.0f) return;

  const char *model = doc["model"]   | "?";
  int   channel     = doc["channel"] | 0;
  long  id          = doc["id"]      | -1L;
  // battery_ok liefern die meisten TH-Decoder (Nexus-TH, Bresser-3CH, ...) mit;
  // fehlt das Feld (anderes Modell), wird "ok" angenommen statt faelschlich zu warnen.
  bool  batteryOk   = (doc["battery_ok"] | 1) != 0;
  float rssi        = doc["rssi"] | NAN;   // dBm, vom Funkmodul pro Telegramm mitgeliefert

  lastRxMs = millis();
  Sensor *s = findOrAddSensor(model, channel, id);
  Serial.printf("RX  model=%-14s ch=%d id=%-5ld T=%.1fC  RH=%.0f%%  RSSI=%.0fdBm  Batt=%s%s\n",
                model, channel, id, t, rh, rssi, batteryOk ? "ok" : "SCHWACH",
                s ? "" : "   (Liste voll - ignoriert)");
  if (!s) return;

  s->t = t; s->rh = rh; s->batteryOk = batteryOk; s->stamp = millis(); s->count++;
  if (!isnan(rssi)) s->rssi = rssi;
  s->valid = true;
  recordHistory(*s);
  checkThresholds(*s);
  checkComparisons(*s);
}

/* ------------------------------------------------------------------ *
 *  4) OLED: eine Sensor-"Karte" im Vollbild, seitenweise gewechselt. *
 *     Grosse Temperaturziffern (wie ein kleines Thermometer-Display) *
 *     statt einer dichten Tabelle - auf 128x64 ist das besser        *
 *     ablesbar als 6 winzige Textzeilen.                             *
 * ------------------------------------------------------------------ */
#if USE_OLED
static bool g_oledOk = false;

// Frische-Symbol oben rechts: gefuellter Punkt = frisch, Ring = traege,
// kleines Kreuz = abgelaufen (gleiche Schwellen wie auf der Webseite).
static void drawFreshIcon(int cx, int cy, uint32_t age) {
  if (age < 120) {
    u8g2.drawDisc(cx, cy, 3);
  } else if (age < 900) {
    u8g2.drawCircle(cx, cy, 3);
  } else {
    u8g2.drawLine(cx - 3, cy - 3, cx + 3, cy + 3);
    u8g2.drawLine(cx - 3, cy + 3, cx + 3, cy - 3);
  }
}

// Kompakte Textform der konfigurierten Schwellenwerte (leer, wenn keine
// Grenze gesetzt ist) fuer die Zeile zwischen grosser Zahl und Fusszeile.
static void formatLimits(const Threshold *c, char *out, size_t outLen) {
  out[0] = 0;
  if (!c || !c->used) return;
  char tpart[16] = "", rpart[16] = "";
  if (c->tMin > -900 && c->tMax < 900)
    snprintf(tpart, sizeof(tpart), "T%.0f..%.0f", c->tMin, c->tMax);
  else if (c->tMax < 900)
    snprintf(tpart, sizeof(tpart), "T<%.0f", c->tMax);
  else if (c->tMin > -900)
    snprintf(tpart, sizeof(tpart), "T>%.0f", c->tMin);

  if (c->rhMin > -900 && c->rhMax < 900)
    snprintf(rpart, sizeof(rpart), "H%.0f..%.0f", c->rhMin, c->rhMax);
  else if (c->rhMax < 900)
    snprintf(rpart, sizeof(rpart), "H<%.0f", c->rhMax);
  else if (c->rhMin > -900)
    snprintf(rpart, sizeof(rpart), "H>%.0f", c->rhMin);

  if (tpart[0] && rpart[0]) snprintf(out, outLen, "%s  %s", tpart, rpart);
  else if (tpart[0])        snprintf(out, outLen, "%s", tpart);
  else if (rpart[0])        snprintf(out, outLen, "%s", rpart);
}

// WLAN-Empfangsstaerke, mittig unten (dBm, negative Zahl - je naeher an 0,
// desto besser). Zeichnet nichts, solange kein WLAN verbunden ist.
#if USE_WIFI
static void drawWifiStrength(int cy) {
  if (WiFi.status() != WL_CONNECTED) return;
  u8g2.setFont(u8g2_font_5x8_tf);
  char b[10];
  snprintf(b, sizeof(b), "%ddBm", (int)WiFi.RSSI());
  int w = u8g2.getStrWidth(b);
  u8g2.drawStr((128 - w) / 2, cy, b);
}
#endif

static bool sensorAlarming(const Threshold *c) {
  return c && (c->alarmTHigh || c->alarmTLow || c->alarmRHHigh || c->alarmRHLow ||
               c->alarmSignalLost || c->alarmBatteryLow);
}

// ---- Trend-Pfeile ------------------------------------------------------------------------
//  Pfeil hoch/runter/gleich plus Aenderung gegenueber vor trendMin Minuten (Sensorkarten, Tacho,
//  Ampel). Aenderungen unter trendEps() gelten als "gleich".
static float trendEps(int q) {
  switch (q) {
    case Q_RH:   return 2.0f;
    case Q_RSSI: return 4.0f;
    default:     return 0.3f;
  }
}

// Pfeil (7x7) und Aenderung, zusammen mittig bei cx, Oberkante top (insgesamt 8 px hoch).
static void drawTrendMark(int cx, int top, float delta, int q) {
  float eps = trendEps(q);
  int dir = delta > eps ? 1 : (delta < -eps ? -1 : 0);
  char b[8];
  snprintf(b, sizeof(b), qDecimals(q) ? "%+.1f" : "%+.0f", delta);
  u8g2.setFont(u8g2_font_5x8_tf);
  int x = cx - (10 + u8g2.getStrWidth(b)) / 2;
  int my = top + 3;
  if (dir > 0)      u8g2.drawTriangle(x + 3, top, x, top + 6, x + 6, top + 6);
  else if (dir < 0) u8g2.drawTriangle(x, top, x + 6, top, x + 3, top + 6);
  else { u8g2.drawHLine(x, my, 5); u8g2.drawTriangle(x + 6, my, x + 3, my - 3, x + 3, my + 3); }
  u8g2.drawStr(x + 10, top + 7, b);
}

// Aenderung der Groesse q eines Sensors gegenueber vor trendMin Minuten, aus seiner Historie
// (Ringpuffer, ein Punkt je Empfang). Ist die Historie kuerzer als der Zeitraum, zaehlt der aelteste
// Punkt, sofern er mindestens den halben Zeitraum zurueckliegt. false = noch nicht berechenbar.
static bool cardTrend(const Sensor &s, int q, float &delta) {
  if (q == Q_RSSI) return false;
  History *h = findHistory(s.model, s.channel, s.id);
  if (!h || h->count < 2) return false;
  uint32_t nowS = millis() / 1000, win = (uint32_t)trendMin * 60UL;
  int ref = -1;
  for (int i = 0; i < h->count; i++) {                         // vom neuesten rueckwaerts
    int idx = (((h->head - 1 - i) % HIST_LEN) + HIST_LEN) % HIST_LEN;
    if (nowS - h->pts[idx].t >= win) { ref = idx; break; }
  }
  if (ref < 0) {
    int old = (h->head - h->count + HIST_LEN) % HIST_LEN;
    if (nowS - h->pts[old].t < win / 2) return false;
    ref = old;
  }
  float then = qValue(q, h->pts[ref].temp, h->pts[ref].rh, NAN);
  float cur  = qValue(q, s.t, s.rh, NAN);
  if (isnan(then) || isnan(cur)) return false;
  delta = cur - then;
  return true;
}

static bool gaugeTrend(int q, float dNow, float &delta);

static void drawSensorCard(const Sensor &s, int page, int pages) {
  Threshold *c = findThreshold(s.model, s.channel, s.id);
  // Bei Grenzwert-Alarm blinkt die Kopfzeile invertiert mit Warntext.
  bool blinkOn = sensorAlarming(c) && ((millis() / 600) % 2 == 0);

  // Kopfzeile: Modell + Kanal/ID links, Seitenzaehler rechts, Frische-Icon ganz rechts
  // (im Alarm-Blink stattdessen invertierter Warnbalken).
  if (blinkOn) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 11);
    u8g2.setDrawColor(0);
  }
  u8g2.setFont(u8g2_font_6x10_tf);
  char head[24];
  if (blinkOn && c->alarmSignalLost)      snprintf(head, sizeof(head), "!  KEIN SIGNAL  !");
  else if (blinkOn && c->alarmBatteryLow) snprintf(head, sizeof(head), "! BATTERIE SCHWACH !");
  else if (blinkOn)                       snprintf(head, sizeof(head), "!  GRENZWERT  !");
  else {
    char dispName[21];
    bool named = getDisplayName(s.model, s.channel, s.id, dispName, sizeof(dispName));
    snprintf(head, sizeof(head), "%-10.10s ch%d", named ? dispName : s.model, s.channel);
  }
  u8g2.drawStr(0, 8, head);
  if (!blinkOn) {
    if (pages > 1) {
      char pg[8];
      snprintf(pg, sizeof(pg), "%d/%d", page + 1, pages);
      u8g2.drawStr(128 - u8g2.getStrWidth(pg) - 11, 8, pg);
    }
    drawFreshIcon(122, 4, ageSec(s));
  }
  if (blinkOn) u8g2.setDrawColor(1);
  u8g2.drawHLine(0, 11, 128);

  // Spalten der Sensorkarte: Wert 1 und 2 (Standard Temperatur, rel. Feuchte) und
  // optional Wert 3 (Standard Taupunkt, im Untermenue /display abwaehlbar). Mit drei Werten
  // stehen drei schmale Spalten nebeneinander (Name oben, Zahl, Einheit darunter), mit
  // zwei Werten wie urspruenglich zwei grosse Spalten mit der Einheit neben der Zahl.
  const bool three = dispQX < Q_COUNT;
  if (three) {
    static const int DIV1 = 43, DIV2 = 85, MID3[3] = {21, 64, 106};
    u8g2.drawVLine(DIV1, 12, 34);
    u8g2.drawVLine(DIV2, 12, 34);
    static const char *LBL3[Q_COUNT] = {"TEMP.", "REL.F.", "ABS.F.", "TAUPKT", "ABST.", "EMPF."};
    const int q3[3] = {dispQL, dispQR, dispQX};
    for (int k = 0; k < 3; k++) {
      int q = q3[k];
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(MID3[k] - u8g2.getStrWidth(LBL3[q]) / 2, 19, LBL3[q]);
      float v = qValue(q, s.t, s.rh, s.rssi);
      char num[10];
      if (isnan(v)) strcpy(num, "--");
      else snprintf(num, sizeof(num), qDecimals(q) ? "%.1f" : "%.0f", v);
      u8g2.setFont(u8g2_font_logisoso16_tn);
      int nw = u8g2.getStrWidth(num);
      u8g2.drawStr(MID3[k] - nw / 2, 37, num);
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(MID3[k] - u8g2.getStrWidth(Q_UNIT[q]) / 2, 45, Q_UNIT[q]);
      float tr;
      if (trendCards && s.batteryOk && cardTrend(s, q, tr)) drawTrendMark(MID3[k], 47, tr, q);
    }
  } else {
    static const int DIVIDER_X = 64, COL_L_MID = 32, COL_R_MID = 96;
    u8g2.drawVLine(DIVIDER_X, 12, 33);

    const int colQ[2]   = {dispQL, dispQR};
    const int colMid[2] = {COL_L_MID, COL_R_MID};
    for (int k = 0; k < 2; k++) {
      int q = colQ[k];
      u8g2.setFont(u8g2_font_5x8_tf);
      const char *lbl = Q_CARD_LABEL[q];
      u8g2.drawStr(colMid[k] - u8g2.getStrWidth(lbl) / 2, 20, lbl);

      float v = qValue(q, s.t, s.rh, s.rssi);
      char num[10];
      if (isnan(v)) strcpy(num, "--");
      else snprintf(num, sizeof(num), qDecimals(q) ? "%.1f" : "%.0f", v);
      u8g2.setFont(u8g2_font_logisoso16_tn);
      int nw = u8g2.getStrWidth(num);
      // Einheit klein neben der Zahl; die lange Einheit "g/m3" in der kleineren Schrift
      bool smallUnit = (q == Q_AH);
      u8g2.setFont(smallUnit ? u8g2_font_5x8_tf : u8g2_font_6x10_tf);
      int uw = u8g2.getStrWidth(Q_UNIT[q]);
      int x = colMid[k] - (nw + 2 + uw) / 2;
      if (x < 0) x = 0;
      u8g2.drawStr(x + nw + 2, 34, Q_UNIT[q]);
      u8g2.setFont(u8g2_font_logisoso16_tn);
      u8g2.drawStr(x, 43, num);
      float tr;
      if (trendCards && s.batteryOk && cardTrend(s, q, tr)) drawTrendMark(colMid[k], 47, tr, q);
    }
  }

  // Zusatzzeile: nur noch ein Batteriehinweis, falls schwach (absolute Feuchte
  // und Schwellenwerte bewusst nicht mehr auf dem OLED - Platzgrund, stehen
  // weiter auf der Weboberfläche).
  if (!s.batteryOk) {
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(0, 53, "BATT SCHWACH");
  }

  // Fusszeile: Alter des letzten Empfangs links, Sensor-ID rechts.
  u8g2.setFont(u8g2_font_6x10_tf);
  char foot[20];
  snprintf(foot, sizeof(foot), "vor %lus", (unsigned long)ageSec(s));
  u8g2.drawStr(0, 62, foot);
  char idbuf[16];
  snprintf(idbuf, sizeof(idbuf), "ID %ld", s.id);
  u8g2.drawStr(128 - u8g2.getStrWidth(idbuf), 62, idbuf);
#if USE_WIFI
  drawWifiStrength(62);
#endif
}

// Welche Sensoren sollen angezeigt werden: nur die als "eigen" markierten -
// solange noch keiner markiert ist (Erstbenutzung), uebergangsweise alle.
// Fuellt idxOut mit Indizes in sensors[], liefert die Anzahl.
static int visibleSensors(int *idxOut, int maxOut) {
  bool filter = anyOwned();
  int n = 0;
  for (int i = 0; i < sensorCount && n < maxOut; i++) {
    if (filter && !isOwned(sensors[i].model, sensors[i].channel, sensors[i].id)) continue;
    idxOut[n++] = i;
  }
  return n;
}

/* ---- 4b) Verlauf: Innen / Aussen / Differenz (Groesse waehlbar) ----
 *  Entspricht dem 24-h-Verlauf "Feuchte-Differenz, Innen und Aussen" im Home-
 *  Assistant-Dashboard. Aussen = Referenzsensor, Innen = erster Sensor mit
 *  aktivem "Mit Referenzsensor vergleichen" (beides unter /settings). Alle
 *  GRAPH_STEP_MS wird ein Punkt abgelegt (nur im RAM, nach einem Neustart
 *  beginnt der Verlauf neu); solange weniger als 24 h vorliegen, wird die
 *  vorhandene Zeitspanne auf die ganze Breite gestreckt. Abgelegt werden die
 *  Rohwerte Temperatur und rel. Feuchte - welche Groesse gezeichnet wird
 *  (abs. Feuchte, Temperatur, rel. Feuchte, Taupunkt), steht in dispQV und
 *  laesst sich auf der Startseite jederzeit umstellen, auch rueckwirkend. */

static const int      GRAPH_N       = 360;           // Punkte im Ringpuffer
static const uint32_t GRAPH_STEP_MS = 240000UL;      // 4 min -> 360 Punkte = 24 h
static const uint32_t GRAPH_MAX_AGE = 1800;          // s - aelterer Messwert zaehlt als "fehlt"
static const int16_t  GRAPH_NONE    = -32768;        // kein gueltiger Wert
static const int      GRAPH_X0 = 21, GRAPH_W = 107;  // Plotbereich (Spalten)
static const int      GRAPH_Y0 = 10, GRAPH_Y1 = 52;  // Plotbereich (Zeilen)

static int16_t graphTin[GRAPH_N], graphRin[GRAPH_N], graphTout[GRAPH_N], graphRout[GRAPH_N];  // je *100
static int     graphHead = 0, graphCount = 0;

static const Sensor *graphOutdoor() {
  return refSensor.used ? findLiveSensor(refSensor.model, refSensor.channel, refSensor.id) : nullptr;
}

static const Sensor *graphIndoor() {
  for (int i = 0; i < MAX_THRESH; i++) {
    const Threshold &c = thresholds[i];
    if (!c.used || !c.cmpEnabled) continue;
    if (refSensor.used && c.channel == refSensor.channel && c.id == refSensor.id &&
        strcmp(c.model, refSensor.model) == 0) continue;
    const Sensor *s = findLiveSensor(c.model, c.channel, c.id);
    if (s) return s;
  }
  return nullptr;
}

static bool graphFresh(const Sensor *s) {
  if (!s || !s->valid || ageSec(*s) > GRAPH_MAX_AGE) return false;
  float ah = absoluteHumidity(s->t, s->rh);
  return !isnan(ah) && ah >= 0 && ah <= 60;   // Fehldekodierungen ausfiltern
}

// Aktueller Wert der Groesse q (NAN = Sensor fehlt oder ist veraltet).
static float graphNow(const Sensor *s, int q) {
  return graphFresh(s) ? qValue(q, s->t, s->rh, NAN) : NAN;
}

static void graphSample() {
  static uint32_t last = 0;
  static bool started = false;
  if (started && millis() - last < GRAPH_STEP_MS) return;
  const Sensor *si = graphIndoor(), *so = graphOutdoor();
  bool fi = graphFresh(si), fo = graphFresh(so);
  if (!started && !fi && !fo) return;                           // erst starten, wenn es Daten gibt
  started = true;
  last = millis();
  graphTin[graphHead]  = fi ? (int16_t)lroundf(si->t  * 100.0f) : GRAPH_NONE;
  graphRin[graphHead]  = fi ? (int16_t)lroundf(si->rh * 100.0f) : GRAPH_NONE;
  graphTout[graphHead] = fo ? (int16_t)lroundf(so->t  * 100.0f) : GRAPH_NONE;
  graphRout[graphHead] = fo ? (int16_t)lroundf(so->rh * 100.0f) : GRAPH_NONE;
  graphHead = (graphHead + 1) % GRAPH_N;
  if (graphCount < GRAPH_N) graphCount++;
}

// Wert der Groesse q fuer Messpunkt i (0 = aeltester), Innen oder Aussen.
static float graphValAt(bool indoor, int i, int q) {
  int idx = (graphHead - graphCount + i + GRAPH_N) % GRAPH_N;
  int16_t t = indoor ? graphTin[idx] : graphTout[idx];
  int16_t r = indoor ? graphRin[idx] : graphRout[idx];
  if (t == GRAPH_NONE || r == GRAPH_NONE) return NAN;
  return qValue(q, t / 100.0f, r / 100.0f, NAN);
}

// Wert fuer Plot-Spalte c: bei mehr Punkten als Spalten gemittelt, bei weniger linear
// interpoliert (Luecken bleiben Luecken).
static float graphColVal(bool indoor, int c, int q) {
  if (graphCount < GRAPH_W) {
    float pos = (float)c * (graphCount - 1) / (GRAPH_W - 1);
    int i0 = (int)pos, i1 = (i0 + 1 < graphCount) ? i0 + 1 : i0;
    float f = pos - i0, v0 = graphValAt(indoor, i0, q), v1 = graphValAt(indoor, i1, q);
    if (!isnan(v0) && !isnan(v1)) return v0 + (v1 - v0) * f;
    return f < 0.001f ? v0 : NAN;
  }
  int i0 = (int)((long)c * graphCount / GRAPH_W);
  int i1 = (int)((long)(c + 1) * graphCount / GRAPH_W) - 1;
  if (i1 < i0) i1 = i0;
  float sum = 0;
  int n = 0;
  for (int i = i0; i <= i1 && i < graphCount; i++) {
    float v = graphValAt(indoor, i, q);
    if (!isnan(v)) { sum += v; n++; }
  }
  return n ? sum / n : NAN;
}

static int graphY(float v, float lo, float span) {
  return GRAPH_Y1 - (int)lroundf((v - lo) / span * (GRAPH_Y1 - GRAPH_Y0));
}

// Rand, Rundungsschritt und Mindestspanne der y-Achse je Groesse.
struct GraphScale { float pad, step, minSpan; };
static GraphScale graphScale(int q) {
  switch (q) {
    case Q_RH: return {2.0f, 5.0f, 10.0f};
    case Q_T:
    case Q_TD: return {0.5f, 1.0f, 4.0f};
    default:   return {0.3f, 0.5f, 2.0f};
  }
}

static void drawGraphPage() {
  const int q = dispQV, dq = qDecimals(q);
  const Sensor *sIn = graphIndoor(), *sOut = graphOutdoor();
  float nowIn = graphNow(sIn, q), nowOut = graphNow(sOut, q);

  // Kopfzeile: Linienmuster als Legende + aktuelle Werte der gewaehlten Groesse
  u8g2.setFont(u8g2_font_5x8_tf);
  int x = 0;
  for (int k = 0; k < 3; k++) {
    char b[14];
    if (k == 0) { u8g2.drawHLine(x, 3, 7); u8g2.drawHLine(x, 4, 7); }          // Innen: dick
    else if (k == 1) u8g2.drawHLine(x, 4, 7);                                  // Aussen: duenn
    else for (int dx = 0; dx < 7; dx += 2) u8g2.drawPixel(x + dx, 4);          // Differenz: gepunktet
    x += 9;
    if (k == 0)      { if (!isnan(nowIn))  snprintf(b, sizeof(b), "In%.*f", dq, nowIn);   else strcpy(b, "In--"); }
    else if (k == 1) { if (!isnan(nowOut)) snprintf(b, sizeof(b), "Au%.*f", dq, nowOut);  else strcpy(b, "Au--"); }
    else             { if (!isnan(nowIn) && !isnan(nowOut)) snprintf(b, sizeof(b), "D%+.*f", dq, nowIn - nowOut); else strcpy(b, "D--"); }
    u8g2.drawStr(x, 7, b);
    x += u8g2.getStrWidth(b) + 5;
  }

  if (graphCount < 2) {
    u8g2.drawStr(0, 32, "sammle Verlauf ...");
    if (!sOut) u8g2.drawStr(0, 44, "Aussen = Referenzsensor");
    if (!sIn)  u8g2.drawStr(0, 54, "Innen = Vergleich aktiv");
    return;
  }

  // Wertebereich aus Innen, Aussen und Differenz (wie im gemeinsamen HA-Diagramm)
  float lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < graphCount; i++) {
    float a = graphValAt(true, i, q), o = graphValAt(false, i, q);
    if (!isnan(a)) { lo = fminf(lo, a); hi = fmaxf(hi, a); }
    if (!isnan(o)) { lo = fminf(lo, o); hi = fmaxf(hi, o); }
    if (!isnan(a) && !isnan(o)) { lo = fminf(lo, a - o); hi = fmaxf(hi, a - o); }
  }
  if (lo > hi) { u8g2.drawStr(0, 36, "keine Messwerte"); return; }
  GraphScale sc = graphScale(q);
  lo = floorf((lo - sc.pad) / sc.step) * sc.step;
  hi = ceilf((hi + sc.pad) / sc.step) * sc.step;
  if (hi - lo < sc.minSpan) { float mid = (hi + lo) / 2; lo = mid - sc.minSpan / 2; hi = mid + sc.minSpan / 2; }
  float span = hi - lo;

  // Achsen, gepunktete Nulllinie
  u8g2.drawVLine(GRAPH_X0 - 1, GRAPH_Y0, GRAPH_Y1 - GRAPH_Y0 + 2);
  u8g2.drawHLine(GRAPH_X0 - 1, GRAPH_Y1 + 1, GRAPH_W + 1);
  if (lo < 0 && hi > 0) {
    int y0 = graphY(0, lo, span);
    for (int px = GRAPH_X0; px < GRAPH_X0 + GRAPH_W; px += 4) u8g2.drawPixel(px, y0);
  }

  // Kurven
  int pxI = -1, pyI = 0, pxO = -1, pyO = 0;
  for (int c = 0; c < GRAPH_W; c++) {
    int px = GRAPH_X0 + c;
    float a = graphColVal(true, c, q), o = graphColVal(false, c, q);
    if (!isnan(a)) {
      int py = graphY(a, lo, span);
      if (pxI >= 0) { u8g2.drawLine(pxI, pyI, px, py); u8g2.drawLine(pxI, pyI + 1, px, py + 1); }
      else { u8g2.drawPixel(px, py); u8g2.drawPixel(px, py + 1); }
      pxI = px; pyI = py;
    } else pxI = -1;
    if (!isnan(o)) {
      int py = graphY(o, lo, span);
      if (pxO >= 0) u8g2.drawLine(pxO, pyO, px, py); else u8g2.drawPixel(px, py);
      pxO = px; pyO = py;
    } else pxO = -1;
    if (!isnan(a) && !isnan(o) && (c % 2 == 0)) u8g2.drawPixel(px, graphY(a - o, lo, span));
  }

  // Beschriftung: Wertebereich links, Einheit, Zeitspanne unten
  char t[10];
  snprintf(t, sizeof(t), "%.*f", dq, hi);  u8g2.drawStr(0, GRAPH_Y0 + 7, t);
  snprintf(t, sizeof(t), "%.*f", dq, lo);  u8g2.drawStr(0, GRAPH_Y1, t);
  u8g2.drawStr(0, 63, q == Q_TD || q == Q_T ? "\xb0" "C" : Q_UNIT[q]);
  int mins = (graphCount - 1) * (int)(GRAPH_STEP_MS / 60000UL);
  if (mins >= 120) snprintf(t, sizeof(t), "-%dh", (mins + 30) / 60);
  else             snprintf(t, sizeof(t), "-%dmin", mins);
  u8g2.drawStr(GRAPH_X0, 63, t);
  u8g2.drawStr(128 - u8g2.getStrWidth("jetzt"), 63, "jetzt");
}

/* ---- 4c) Tacho: momentane Differenz Innen - Aussen (Groesse waehlbar) ----
 *  Halbkreis mit Zeiger, wie die Gauge-Karte im Home-Assistant-Dashboard. Fuer die
 *  absolute Feuchte (g/m3) gilt dieselbe Skala und Zonen wie dort (-3 ... 6, gruen
 *  bis 0,5, gelb bis 1,0, darueber rot = Lueften); der Taupunkt hat eine eigene
 *  Skala mit Zonen, Temperatur und rel. Feuchte nur eine Skala ohne Zonen/Urteil.
 *  Das OLED ist einfarbig, deshalb unterscheidet das Muster die Zonen: gruen =
 *  nur Umriss, gelb = schraffiert, rot = voll ausgefuellt. Innen/Aussen wie beim
 *  Verlauf (graphIndoor()/graphOutdoor()), die Groesse steht in dispQG. */
struct GaugeCfg { float mn, mx, yellow, red; int dec; const char *unit; bool verdict; };
static GaugeCfg gaugeCfg(int q) {
  switch (q) {
    case Q_TD: return {-4.0f, 8.0f, 1.0f, 2.0f, 1, "K", true};
    case Q_T:  return {-10.0f, 20.0f, 0.0f, 0.0f, 1, "K", false};
    case Q_RH: return {-40.0f, 60.0f, 0.0f, 0.0f, 0, "%", false};
    default:   return {-3.0f, 6.0f, 0.5f, 1.0f, 1, "g/m3", true};
  }
}
static const int   GAUGE_CX = 64, GAUGE_CY = 47, GAUGE_R_OUT = 43, GAUGE_R_IN = 34;
static const float PI_F = 3.14159265f;

static float gaugeAngle(float v, const GaugeCfg &g) {   // Minimum -> 180 Grad (links), Maximum -> 0 Grad (rechts)
  float f = (v - g.mn) / (g.mx - g.mn);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  return (1.0f - f) * PI_F;
}

static void gaugeLine(float ang, float r0, float r1) {
  u8g2.drawLine(GAUGE_CX + (int)lroundf(r0 * cosf(ang)), GAUGE_CY - (int)lroundf(r0 * sinf(ang)),
                GAUGE_CX + (int)lroundf(r1 * cosf(ang)), GAUGE_CY - (int)lroundf(r1 * sinf(ang)));
}

// style 0 = Umriss (gruen), 1 = schraffiert (gelb), 2 = voll (rot)
static void gaugeBand(float v0, float v1, int style, const GaugeCfg &g) {
  float a0 = gaugeAngle(v0, g), a1 = gaugeAngle(v1, g);   // a0 >= a1
  if (style == 2) {                                       // fein genug, damit keine Luecken bleiben
    for (float a = a1; a <= a0 + 0.001f; a += 0.006f) gaugeLine(a, GAUGE_R_IN, GAUGE_R_OUT);
  } else {
    for (float a = a1; a <= a0 + 0.001f; a += 0.008f) {   // Bogenkanten als Punktreihe
      u8g2.drawPixel(GAUGE_CX + (int)lroundf(GAUGE_R_OUT * cosf(a)), GAUGE_CY - (int)lroundf(GAUGE_R_OUT * sinf(a)));
      u8g2.drawPixel(GAUGE_CX + (int)lroundf(GAUGE_R_IN * cosf(a)),  GAUGE_CY - (int)lroundf(GAUGE_R_IN * sinf(a)));
    }
    if (style == 1)
      for (float a = a1; a <= a0; a += 0.06f) gaugeLine(a, GAUGE_R_IN, GAUGE_R_OUT);
  }
  gaugeLine(a0, GAUGE_R_IN, GAUGE_R_OUT);                // Zonengrenzen
  gaugeLine(a1, GAUGE_R_IN, GAUGE_R_OUT);
}

static void drawGaugePage() {
  const int q = dispQG;
  const GaugeCfg g = gaugeCfg(q);
  float vi = graphNow(graphIndoor(), q), vo = graphNow(graphOutdoor(), q);
  bool ok = !isnan(vi) && !isnan(vo);
  float d = ok ? vi - vo : 0;

  // Skala: Zonen (oder ein einziger Umriss), Marke bei 0, Endwerte
  if (g.verdict) {
    gaugeBand(g.mn, g.yellow, 0, g);
    gaugeBand(g.yellow, g.red, 1, g);
    gaugeBand(g.red, g.mx, 2, g);
  } else {
    gaugeBand(g.mn, g.mx, 0, g);
  }
  gaugeLine(gaugeAngle(0, g), GAUGE_R_OUT + 1, GAUGE_R_OUT + 4);
  u8g2.setFont(u8g2_font_5x8_tf);
  char lim[8];
  snprintf(lim, sizeof(lim), "%.0f", g.mn);
  u8g2.drawStr(26 - u8g2.getStrWidth(lim) / 2, 58, lim);
  snprintf(lim, sizeof(lim), "%.0f", g.mx);
  u8g2.drawStr(102 - u8g2.getStrWidth(lim) / 2, 58, lim);

  // Kopf: was gemessen wird (links) und das Urteil (rechts)
  u8g2.drawStr(0, 7, "Innen-");
  u8g2.drawStr(0, 16, "Aussen");
  const char *verdict = !ok ? (graphIndoor() ? "Aussen?" : "Innen?")
                            : (!g.verdict ? "" : (d >= g.red ? "LUEFTEN" : (d >= g.yellow ? "Grenze" : "ok")));
  u8g2.drawStr(128 - u8g2.getStrWidth(verdict), 7, verdict);

  // Zahlenwert unter dem Drehpunkt
  char val[10];
  if (ok) snprintf(val, sizeof(val), "%+.*f", g.dec, d); else strcpy(val, "--");
  u8g2.setFont(u8g2_font_8x13B_tf);
  int vw = u8g2.getStrWidth(val);
  u8g2.setFont(u8g2_font_5x8_tf);
  int uw = u8g2.getStrWidth(g.unit);
  int x0 = (128 - (vw + 3 + uw)) / 2;
  u8g2.setFont(u8g2_font_8x13B_tf);
  u8g2.drawStr(x0, 62, val);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(x0 + vw + 3, 61, g.unit);
  float tr;
  if (ok && trendGauge && gaugeTrend(q, d, tr)) drawTrendMark(111, 54, tr, q);

  // Zeiger als richtiger Pfeil: 2 px dicker Schaft vom Drehpunkt bis zur Spitze und eine lange,
  // spitze Pfeilspitze; schwarzer Rand um die Spitze, damit sie auf der gefuellten Zone sichtbar bleibt.
  if (ok) {
    const float TIP = 42, BASE = 27, HW = 5;                   // Spitze/Kopf-Basis (Radius) und halbe Kopfbreite
    float th = gaugeAngle(d, g), c = cosf(th), s = sinf(th);
    float bx = GAUGE_CX + BASE * c, by = GAUGE_CY - BASE * s;  // Mitte der Kopf-Basis
    int ox = (int)lroundf(s), oy = (int)lroundf(c);            // 1 px seitlich fuer einen dickeren Schaft
    u8g2.setDrawColor(0);
    u8g2.drawTriangle((int)lroundf(GAUGE_CX + (TIP + 2.5f) * c), (int)lroundf(GAUGE_CY - (TIP + 2.5f) * s),
                      (int)lroundf(bx - 1.2f * c + 7.5f * s), (int)lroundf(by + 1.2f * s + 7.5f * c),
                      (int)lroundf(bx - 1.2f * c - 7.5f * s), (int)lroundf(by + 1.2f * s - 7.5f * c));
    u8g2.setDrawColor(1);
    u8g2.drawLine(GAUGE_CX, GAUGE_CY, (int)lroundf(bx), (int)lroundf(by));
    u8g2.drawLine(GAUGE_CX + ox, GAUGE_CY + oy, (int)lroundf(bx) + ox, (int)lroundf(by) + oy);
    u8g2.drawTriangle((int)lroundf(GAUGE_CX + TIP * c), (int)lroundf(GAUGE_CY - TIP * s),
                      (int)lroundf(bx + HW * s), (int)lroundf(by + HW * c),
                      (int)lroundf(bx - HW * s), (int)lroundf(by - HW * c));
  }
  u8g2.drawDisc(GAUGE_CX, GAUGE_CY, 3);
}

/* ---- 4d) Sensor-Vergleich: dieselbe Groesse von zwei Sensoren nebeneinander ----
 *  Die Groesse (dispQK: Temperatur, rel./abs. Feuchte, Taupunkt, Abstand zum Taupunkt oder
 *  Empfangsstaerke) steht fuer Sensor A (links) und Sensor B (Mitte) nebeneinander, optional
 *  mit der Differenz A - B als dritter Spalte. A und B sind "Innen", "Aussen" oder ein
 *  bestimmter Sensor (cmpA/cmpB, einstellbar im Untermenue /display). */
static const Sensor *resolveSel(const SensorSel &sel) {
  if (sel.kind == 0) return graphIndoor();
  if (sel.kind == 1) return graphOutdoor();
  return findLiveSensor(sel.model, (int)sel.channel, (long)sel.id);
}

static float cmpValue(const Sensor *s, int q) {
  return graphFresh(s) ? qValue(q, s->t, s->rh, s->rssi) : NAN;
}

// Spaltenbeschriftung: "INNEN"/"AUSSEN" oder der Name des Sensors, auf die Spaltenbreite gekuerzt
// (die Schrift 5x8 muss schon gesetzt sein).
static void selLabel(const SensorSel &sel, const Sensor *sen, int maxPx, char *out, size_t n) {
  if (sel.kind == 0)      strncpy(out, "INNEN", n);
  else if (sel.kind == 1) strncpy(out, "AUSSEN", n);
  else {
    char nm[21];
    bool named = sen ? getDisplayName(sen->model, sen->channel, sen->id, nm, sizeof(nm))
                     : getDisplayName(sel.model, (int)sel.channel, (long)sel.id, nm, sizeof(nm));
    strncpy(out, named ? nm : sel.model, n);
  }
  out[n - 1] = 0;
  while (out[0] && u8g2.getStrWidth(out) > maxPx) out[strlen(out) - 1] = 0;
}

static void drawComparePage(int pos, int total) {
  const int q = dispQK, dec = qDecimals(q);
  const Sensor *sens[2] = {resolveSel(cmpA), resolveSel(cmpB)};
  const SensorSel *sels[2] = {&cmpA, &cmpB};
  float vals[3];
  vals[0] = cmpValue(sens[0], q);
  vals[1] = cmpValue(sens[1], q);
  vals[2] = (isnan(vals[0]) || isnan(vals[1])) ? NAN : vals[0] - vals[1];
  const int cols = cmpDiff ? 3 : 2;
  const int colW = 128 / cols;

  u8g2.setFont(u8g2_font_6x10_tf);
  char head[32];
  snprintf(head, sizeof(head), "%s %s", Q_CARD_LABEL[q], Q_UNIT[q]);
  u8g2.drawStr(0, 8, head);
  if (total > 1) {
    char pg[8];
    snprintf(pg, sizeof(pg), "%d/%d", pos + 1, total);
    u8g2.drawStr(128 - u8g2.getStrWidth(pg), 8, pg);
  }
  u8g2.drawHLine(0, 11, 128);

  for (int k = 0; k < cols; k++) {
    int mid = colW * k + colW / 2;
    if (k > 0) u8g2.drawVLine(colW * k, 12, 42);
    char lbl[22];
    u8g2.setFont(u8g2_font_5x8_tf);
    if (k < 2) selLabel(*sels[k], sens[k], colW - 4, lbl, sizeof(lbl)); else strcpy(lbl, "DIFF.");
    u8g2.drawStr(mid - u8g2.getStrWidth(lbl) / 2, 21, lbl);

    char num[10];
    if (isnan(vals[k])) strcpy(num, "--"); else snprintf(num, sizeof(num), "%.*f", dec, vals[k]);
    u8g2.setFont(cmpDiff ? u8g2_font_logisoso16_tn : u8g2_font_logisoso24_tn);
    u8g2.drawStr(mid - u8g2.getStrWidth(num) / 2, cmpDiff ? 43 : 48, num);

    if (k < 2 && sens[k]) {                       // Alter des letzten Empfangs unter der Zahl
      char age[14];
      snprintf(age, sizeof(age), "vor %lus", (unsigned long)ageSec(*sens[k]));
      u8g2.setFont(u8g2_font_5x8_tf);
      u8g2.drawStr(mid - u8g2.getStrWidth(age) / 2, 61, age);
    }
  }
}

#if USE_WIFI
// Hinweisseite zum Hotspot: wie man sich verbindet und wohin man dann geht.
static void drawHotspotInfo() {
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(0, 14, WiFi.status() == WL_CONNECTED ? "Hotspot an" : "Kein WLAN!");
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 27, "Hotspot offen:");
  u8g2.drawStr(0, 38, AP_SETUP_SSID);
  u8g2.setFont(u8g2_font_5x8_tf);
  String ip = "http://" + WiFi.softAPIP().toString() + "/";
  u8g2.drawStr(0, 50, ip.c_str());
  u8g2.drawStr(0, 61, "Passwort: siehe secrets.h");
}
#endif

/* ---- 4e) Weitere Anzeigen: Lueften-Ampel, Min/Max, Schimmelrisiko, Empfangs-Uebersicht,
 *      Systemseite, Uhr. Jede mit eigener Wertauswahl im Untermenue /display. ---- */

// Aenderung der Differenz Innen - Aussen (Groesse q) gegenueber vor trendMin Minuten aus dem
// 24-h-Verlauf (ein Punkt alle 4 min). dNow = aktuelle Differenz.
static bool gaugeTrend(int q, float dNow, float &delta) {
  if (graphCount < 2) return false;
  int steps = (int)((uint32_t)trendMin * 60000UL / GRAPH_STEP_MS);
  int i = graphCount - 1 - steps;
  if (i < 0) {
    if (graphCount - 1 < (steps + 1) / 2) return false;      // weniger als der halbe Zeitraum
    i = 0;
  }
  float a = graphValAt(true, i, q), o = graphValAt(false, i, q);
  if (isnan(a) || isnan(o)) return false;
  delta = dNow - (a - o);
  return true;
}

static const char *const DAY_DE[7] = {"So", "Mo", "Di", "Mi", "Do", "Fr", "Sa"};

// ---- Lueften-Ampel: grosses Urteil + Differenz Innen - Aussen
static void drawAmpelPage() {
  const int q = dispQA;
  const GaugeCfg g = gaugeCfg(q);
  const Sensor *si = graphIndoor(), *so = graphOutdoor();
  float vi = graphNow(si, q), vo = graphNow(so, q);
  bool ok = !isnan(vi) && !isnan(vo);
  float d = ok ? vi - vo : 0;
  int lvl = !ok ? -1 : (d >= g.red ? 2 : (d >= g.yellow ? 1 : 0));
  const char *txt = lvl < 0 ? (si ? "Aussen?" : "Innen?") : (lvl == 2 ? "LUEFTEN" : (lvl == 1 ? "GRENZE" : "OK"));

  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(0, 7, "Lueften-Ampel");
  const char *ql = Q_SHORT[q];
  u8g2.drawStr(128 - u8g2.getStrWidth(ql), 7, ql);

  u8g2.setFont(u8g2_font_logisoso16_tf);
  if (lvl == 2) { u8g2.drawBox(0, 11, 128, 26); u8g2.setDrawColor(0); }
  else if (lvl == 1) { u8g2.drawFrame(0, 11, 128, 26); u8g2.drawFrame(1, 12, 126, 24); }
  u8g2.drawStr((128 - u8g2.getStrWidth(txt)) / 2, 32, txt);
  u8g2.setDrawColor(1);

  char val[10];
  if (ok) snprintf(val, sizeof(val), "%+.*f", g.dec, d); else strcpy(val, "--");
  u8g2.setFont(u8g2_font_8x13B_tf);
  int vw = u8g2.getStrWidth(val);
  u8g2.drawStr(2, 51, val);
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.drawStr(2 + vw + 3, 50, g.unit);
  float tr;
  if (ok && trendGauge && gaugeTrend(q, d, tr)) drawTrendMark(100, 43, tr, q);

  if (ampelDetail) {
    char b[28];
    int dec = qDecimals(q);
    char a[10], c[10];
    if (isnan(vi)) strcpy(a, "--"); else snprintf(a, sizeof(a), "%.*f", dec, vi);
    if (isnan(vo)) strcpy(c, "--"); else snprintf(c, sizeof(c), "%.*f", dec, vo);
    snprintf(b, sizeof(b), "Innen %s  Aussen %s", a, c);
    u8g2.drawStr(0, 62, b);
  }
}

// ---- Min/Max aus dem Verlauf (Innen/Aussen)
static void drawMinMaxPage() {
  const int q = dispQM, dec = qDecimals(q);
  u8g2.setFont(u8g2_font_6x10_tf);
  char head[32];
  snprintf(head, sizeof(head), "MIN/MAX %s", Q_UNIT[q]);
  u8g2.drawStr(0, 8, head);
  int mins = graphCount > 1 ? (graphCount - 1) * (int)(GRAPH_STEP_MS / 60000UL) : 0;
  char span[10];
  if (mins >= 120) snprintf(span, sizeof(span), "%dh", (mins + 30) / 60); else snprintf(span, sizeof(span), "%dmin", mins);
  if (mins > 0) u8g2.drawStr(128 - u8g2.getStrWidth(span), 8, span);
  u8g2.drawHLine(0, 11, 128);

  const int cols = mmWho == 0 ? 2 : 1, colW = 128 / cols;
  for (int c = 0; c < cols; c++) {
    bool indoor = mmWho == 1 || (mmWho == 0 && c == 0);
    float lo = 1e9f, hi = -1e9f;
    for (int i = 0; i < graphCount; i++) {
      float v = graphValAt(indoor, i, q);
      if (isnan(v)) continue;
      lo = fminf(lo, v); hi = fmaxf(hi, v);
    }
    int mid = colW * c + colW / 2;
    if (c > 0) u8g2.drawVLine(colW * c, 12, 40);
    u8g2.setFont(u8g2_font_5x8_tf);
    const char *lbl = indoor ? "INNEN" : "AUSSEN";
    u8g2.drawStr(mid - u8g2.getStrWidth(lbl) / 2, 21, lbl);
    char b[10];
    u8g2.setFont(u8g2_font_8x13B_tf);
    for (int k = 0; k < 2; k++) {
      float v = k == 0 ? hi : lo;
      if (lo > hi) strcpy(b, "--"); else snprintf(b, sizeof(b), "%.*f", dec, v);
      int y = k == 0 ? 38 : 56;
      int x = mid - (u8g2.getStrWidth(b) + 10) / 2;
      if (k == 0) u8g2.drawTriangle(x + 3, y - 8, x, y - 1, x + 6, y - 1);          // Hoechstwert: Pfeil hoch
      else        u8g2.drawTriangle(x, y - 8, x + 6, y - 8, x + 3, y - 1);          // Tiefstwert: Pfeil runter
      u8g2.drawStr(x + 10, y, b);
    }
  }
}

// ---- Schimmelrisiko: Sensor + zwei Werte + Urteil
static void drawMoldPage() {
  const Sensor *s = resolveSel(moldSel);
  u8g2.setFont(u8g2_font_5x8_tf);
  char lbl[22];
  selLabel(moldSel, s, 64, lbl, sizeof(lbl));
  char head[40];
  snprintf(head, sizeof(head), "Schimmel %s", lbl);
  u8g2.drawStr(0, 7, head);
  u8g2.drawHLine(0, 11, 128);
  if (!graphFresh(s)) {
    u8g2.setFont(u8g2_font_7x14B_tf);
    u8g2.drawStr(0, 38, "kein Wert");
    return;
  }
  float td = dewPointC(s->t, s->rh);
  float spread = isnan(td) ? NAN : s->t - td;
  int lvl = (s->rh >= 80 || (!isnan(spread) && spread < 2)) ? 2 : ((s->rh >= 70 || (!isnan(spread) && spread < 4)) ? 1 : 0);

  const int qs[2] = {dispQH1, dispQH2};
  u8g2.drawVLine(64, 12, 34);
  for (int k = 0; k < 2; k++) {
    int q = qs[k], mid = k == 0 ? 32 : 96;
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(mid - u8g2.getStrWidth(Q_CARD_LABEL[q]) / 2, 20, Q_CARD_LABEL[q]);
    float v = qValue(q, s->t, s->rh, s->rssi);
    char num[10];
    if (isnan(v)) strcpy(num, "--"); else snprintf(num, sizeof(num), "%.*f", qDecimals(q), v);
    u8g2.setFont(u8g2_font_logisoso16_tn);
    u8g2.drawStr(mid - u8g2.getStrWidth(num) / 2, 40, num);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(mid - u8g2.getStrWidth(Q_UNIT[q]) / 2, 47, Q_UNIT[q]);
  }
  const char *txt = lvl == 2 ? "SCHIMMELGEFAHR" : (lvl == 1 ? "Achtung" : "kein Risiko");
  u8g2.setFont(u8g2_font_7x14B_tf);
  if (lvl == 2) { u8g2.drawBox(0, 48, 128, 16); u8g2.setDrawColor(0); }
  else if (lvl == 1) u8g2.drawFrame(0, 48, 128, 16);
  u8g2.drawStr((128 - u8g2.getStrWidth(txt)) / 2, 61, txt);
  u8g2.setDrawColor(1);
}

// ---- Empfangs-Uebersicht: je Sensor Name, Balken der Empfangsstaerke, Alter/dBm/Anzahl
static const int RX_ROWS = 5;
static void drawRxPage(int page, int pages) {
  int vis[MAX_SENSORS];
  int n = visibleSensors(vis, MAX_SENSORS);
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 8, "EMPFANG");
  static const char *const COLN[3] = {"Alter", "dBm", "Anzahl"};
  u8g2.drawStr(128 - u8g2.getStrWidth(COLN[rxCol]), 8, COLN[rxCol]);
  if (pages > 1) {
    char pg[8];
    snprintf(pg, sizeof(pg), "%d/%d", page + 1, pages);
    u8g2.drawStr(64 - u8g2.getStrWidth(pg) / 2, 8, pg);
  }
  u8g2.drawHLine(0, 11, 128);
  for (int r = 0; r < RX_ROWS; r++) {
    int i = page * RX_ROWS + r;
    if (i >= n) break;
    const Sensor &sn = sensors[vis[i]];
    int y = 21 + r * 10;
    char nm[21], shown[9];
    bool named = getDisplayName(sn.model, sn.channel, sn.id, nm, sizeof(nm));
    snprintf(shown, sizeof(shown), "%.8s", named ? nm : sn.model);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, y, shown);
    int len = 0;
    if (!isnan(sn.rssi)) {
      float f = (sn.rssi + 100.0f) / 70.0f;
      len = (int)lroundf((f < 0 ? 0 : (f > 1 ? 1 : f)) * 38.0f);
    }
    u8g2.drawFrame(51, y - 7, 40, 8);
    if (len > 0) u8g2.drawBox(52, y - 6, len, 6);
    char b[12];
    if (rxCol == 0)      snprintf(b, sizeof(b), "%lus", (unsigned long)ageSec(sn));
    else if (rxCol == 1) { if (isnan(sn.rssi)) strcpy(b, "--"); else snprintf(b, sizeof(b), "%.0f", sn.rssi); }
    else                 snprintf(b, sizeof(b), "%lux", (unsigned long)sn.count);
    u8g2.drawStr(128 - u8g2.getStrWidth(b), y, b);
  }
  if (n == 0) u8g2.drawStr(0, 36, "keine Sensoren");
}

// ---- Systemseite: waehlbare Zeilen
static void drawSysPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(0, 8, "SYSTEM");
  u8g2.drawHLine(0, 11, 128);
  char line[40];
  int row = 0;
  auto put = [&](const char *t) { if (row < 5) { u8g2.drawStr(0, 21 + row * 10, t); row++; } };
#if USE_WIFI
  if (sysMask & 1) { snprintf(line, sizeof(line), "WLAN %.20s", apMode ? AP_SETUP_SSID : WiFi.SSID().c_str()); put(line); }
  if (sysMask & 2) { snprintf(line, sizeof(line), "IP %s", (apMode ? WiFi.softAPIP() : WiFi.localIP()).toString().c_str()); put(line); }
  if (sysMask & 4) { snprintf(line, sizeof(line), "Signal %d dBm", (int)WiFi.RSSI()); put(line); }
#endif
  if (sysMask & 8) {
    uint32_t sec = millis() / 1000;
    if (sec >= 86400) snprintf(line, sizeof(line), "Laufzeit %lud %luh", (unsigned long)(sec / 86400), (unsigned long)(sec % 86400 / 3600));
    else              snprintf(line, sizeof(line), "Laufzeit %luh %lumin", (unsigned long)(sec / 3600), (unsigned long)(sec % 3600 / 60));
    put(line);
  }
  if (sysMask & 16) {
    snprintf(line, sizeof(line), "Heap %u/%u kB", (unsigned)(ESP.getFreeHeap() / 1024),
             (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024));
    put(line);
  }
  if (sysMask & 32) {
    unsigned long total = 0;
    for (int i = 0; i < sensorCount; i++) total += sensors[i].count;
    snprintf(line, sizeof(line), "Telegramme %lu", total);
    put(line);
  }
  if (row == 0) u8g2.drawStr(0, 36, "keine Zeile gewaehlt");
}

// ---- Uhr mit Werten von Innen/Aussen
static void drawClockPage() {
  time_t now = time(nullptr);
  bool ok = now > 1700000000;
  struct tm tm;
  if (ok) localtime_r(&now, &tm);
  char b[24];
  u8g2.setFont(u8g2_font_logisoso24_tn);
  if (ok) snprintf(b, sizeof(b), "%02d:%02d", tm.tm_hour, tm.tm_min); else strcpy(b, "--:--");
  int w = u8g2.getStrWidth(b);
  u8g2.drawStr((128 - w) / 2 - 8, 30, b);
  u8g2.setFont(u8g2_font_6x10_tf);
  if (ok) { snprintf(b, sizeof(b), "%02d", tm.tm_sec); u8g2.drawStr((128 - w) / 2 - 8 + w + 3, 30, b); }
  if (ok) snprintf(b, sizeof(b), "%s %02d.%02d.%04d", DAY_DE[tm.tm_wday], tm.tm_mday, tm.tm_mon + 1, tm.tm_year + 1900);
  else    snprintf(b, sizeof(b), "keine Zeit (Internet?)");
  u8g2.drawStr((128 - u8g2.getStrWidth(b)) / 2, 44, b);
  if (dispQC < Q_COUNT) {
    const Sensor *si = graphIndoor(), *so = graphOutdoor();
    float vi = graphNow(si, dispQC), vo = graphNow(so, dispQC);
    char a[10], c[10];
    if (isnan(vi)) strcpy(a, "--"); else snprintf(a, sizeof(a), "%.*f", qDecimals(dispQC), vi);
    if (isnan(vo)) strcpy(c, "--"); else snprintf(c, sizeof(c), "%.*f", qDecimals(dispQC), vo);
    snprintf(b, sizeof(b), "In %s  Au %s %s", a, c, Q_UNIT[dispQC]);
    u8g2.drawStr((128 - u8g2.getStrWidth(b)) / 2, 62, b);
  }
}

// ---- Anzeige-Wechsel: welche Seite gerade auf dem OLED steht --------------------------------
//  Die angehakten Anzeigen (dispMask) laufen der Reihe nach, jede mit ihrer eigenen Dauer:
//  erst alle Sensorkarten (je Karte dispDurCards), dann Tacho, dann Verlauf. Der Verlauf
//  kommt erst dazu, wenn es mindestens zwei Messpunkte gibt (ist er die einzige Anzeige,
//  steht er trotzdem da und zeigt "sammle Verlauf"). Ein Hotspot-Hinweis hat Vorrang:
//  im Boot-Fallback dauerhaft, sonst 4 s von je 16 s.
enum { S_HOTSPOT = 0, S_CARD = 1, S_GAUGE = 2, S_GRAPH = 3, S_WAIT = 4, S_COMPARE = 5,
       S_AMPEL = 6, S_MINMAX = 7, S_MOLD = 8, S_RX = 9, S_SYS = 10, S_CLOCK = 11 };
struct DispSlot { uint8_t type; uint8_t idx; uint32_t ms; };
struct DispChoice {
  uint8_t type = S_WAIT;
  uint8_t idx  = 0;          // Index in vis[] bei Sensorkarten
  int     pos  = 0;          // Position in der Seitenliste (fuer "2/5")
  int     total = 1;         // Anzahl der Seiten im Wechsel
  int     n    = 0;          // sichtbare Sensoren
  int     vis[MAX_SENSORS];
};

static void chooseDisplay(DispChoice &c) {
#if USE_WIFI
  if (apMode || (apActive && ((millis() / 4000) % 4) == 3)) { c.type = S_HOTSPOT; c.idx = 0; return; }
#endif
  c.n = visibleSensors(c.vis, MAX_SENSORS);
  DispSlot sl[MAX_SENSORS + 16];
  int k = 0;
  if ((dispMask & DM_CARDS) && c.n > 0)
    for (int i = 0; i < c.n; i++) sl[k++] = {S_CARD, (uint8_t)i, (uint32_t)dispDurCards * 100UL};
  if (dispMask & DM_GAUGE) sl[k++] = {S_GAUGE, 0, (uint32_t)dispDurGauge * 100UL};
  if (dispMask & DM_COMPARE) sl[k++] = {S_COMPARE, 0, (uint32_t)dispDurCmp * 100UL};
  if (dispMask & DM_AMPEL)   sl[k++] = {S_AMPEL,   0, (uint32_t)dispDurAmpel * 100UL};
  if (dispMask & DM_MINMAX)  sl[k++] = {S_MINMAX,  0, (uint32_t)dispDurMm * 100UL};
  if (dispMask & DM_MOLD)    sl[k++] = {S_MOLD,    0, (uint32_t)dispDurMold * 100UL};
  if (dispMask & DM_RX) {
    int pages = c.n > 0 ? (c.n + RX_ROWS - 1) / RX_ROWS : 1;
    for (int i = 0; i < pages; i++) sl[k++] = {S_RX, (uint8_t)i, (uint32_t)dispDurRx * 100UL};
  }
  if (dispMask & DM_SYS)     sl[k++] = {S_SYS,     0, (uint32_t)dispDurSys * 100UL};
  if (dispMask & DM_CLOCK)   sl[k++] = {S_CLOCK,   0, (uint32_t)dispDurClock * 100UL};
  if ((dispMask & DM_GRAPH) && (graphCount >= 2 || k == 0)) sl[k++] = {S_GRAPH, 0, (uint32_t)dispDurGraph * 100UL};
  if (k == 0) sl[k++] = {S_WAIT, 0, 1000UL};
  uint32_t total = 0;
  for (int i = 0; i < k; i++) total += sl[i].ms;
  uint32_t t = millis() % total;
  int pos = 0;
  for (int i = 0; i < k; i++) {
    if (t < sl[i].ms) { pos = i; break; }
    t -= sl[i].ms;
  }
  c.type = sl[pos].type;
  c.idx = sl[pos].idx;
  c.pos = pos;
  c.total = k;
}

// Vollbild-Warnung bei Wasseralarm: blinkt (invertiert) und nennt die betroffenen Sensoren.
static void drawLeakAlarm() {
  bool inv = ((millis() / 400UL) % 2UL) == 0UL;
  if (inv) {
    u8g2.setDrawColor(1);
    u8g2.drawBox(0, 0, 128, 64);
    u8g2.setDrawColor(0);
  } else {
    u8g2.setDrawColor(1);
  }
  u8g2.setFont(u8g2_font_logisoso16_tf);
  u8g2.drawStr(6, 24, "WASSER!");
  u8g2.setFont(u8g2_font_7x14B_tf);
  int y = 42;
  for (int i = 0; i < LEAK_MAX && y < 64; i++) {
    if (!(leaks[i].used && leaks[i].alarm)) continue;
    char lab[32];
    leakLabel(leaks[i], lab, sizeof(lab));
    lab[18] = 0;                       // 18 Zeichen x 7 px = 126 px
    u8g2.drawStr(2, y, lab);
    y += 15;
  }
  u8g2.setDrawColor(1);
}

static void draw(const DispChoice &c) {
  if (!g_oledOk) return;
  u8g2.clearBuffer();
  if (leakAnyAlarm()) { drawLeakAlarm(); u8g2.sendBuffer(); return; }
  switch (c.type) {
#if USE_WIFI
    case S_HOTSPOT: drawHotspotInfo(); break;
#endif
    case S_CARD:    drawSensorCard(sensors[c.vis[c.idx]], c.pos, c.total); break;
    case S_GAUGE:   drawGaugePage(); break;
    case S_GRAPH:   drawGraphPage(); break;
    case S_COMPARE: drawComparePage(c.pos, c.total); break;
    case S_AMPEL:   drawAmpelPage(); break;
    case S_MINMAX:  drawMinMaxPage(); break;
    case S_MOLD:    drawMoldPage(); break;
    case S_RX:      drawRxPage(c.idx, c.n > 0 ? (c.n + RX_ROWS - 1) / RX_ROWS : 1); break;
    case S_SYS:     drawSysPage(); break;
    case S_CLOCK:   drawClockPage(); break;
    default:
      u8g2.setFont(u8g2_font_7x14B_tf);
      u8g2.drawStr(0, 26, "868-Empfaenger");
      u8g2.drawHLine(0, 30, 128);
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.drawStr(0, 48, "warte auf Sensoren...");
#if USE_WIFI
      drawWifiStrength(62);
#endif
  }
  u8g2.sendBuffer();
}

// Wird in jedem Schleifendurchlauf aufgerufen: zeichnet neu, sobald die Seite wechselt
// (Zehntelsekunden genau), sonst hoechstens einmal pro Sekunde (Werte, Alter ...).
static void displayTick() {
  static uint32_t lastDraw = 0, lastCheck = 0;
  static int lastId = -1;
  if (millis() - lastCheck < 50) return;       // alle 50 ms genuegt (Wechsel in Zehntelsekunden)
  lastCheck = millis();
  DispChoice c;
  chooseDisplay(c);
  int id = c.type * 100 + c.idx;
  if (id != lastId || millis() - lastDraw >= (leakAnyAlarm() ? 200UL : 1000UL)) {
    lastId = id;
    lastDraw = millis();
    draw(c);
  }
}

static void splash() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_logisoso24_tn);
  u8g2.drawStr(4, 34, "868");
  u8g2.setFont(u8g2_font_7x14B_tf);
  u8g2.drawStr(4, 52, "Empfaenger");
  u8g2.sendBuffer();
}
#endif

/* ------------------------------------------------------------------ *
 *  5) LED: kurzer Blitz bei jedem Empfang, dauerhaft an bei aktiver  *
 *     Lueftungsempfehlung (nicht blockierend, funktioniert komplett  *
 *     ohne WLAN/ntfy - reine Lokalauswertung am Board selbst)        *
 * ------------------------------------------------------------------ */
#if USE_OLED
// Zeigt der Tacho gerade "LUEFTEN" (Differenz Innen - Aussen im roten Bereich)?
static bool gaugeSaysVentilate() {
  const GaugeCfg g = gaugeCfg(dispQG);
  if (!g.verdict) return false;
  float vi = graphNow(graphIndoor(), dispQG), vo = graphNow(graphOutdoor(), dispQG);
  return !isnan(vi) && !isnan(vo) && (vi - vo) >= g.red;
}
#endif

static void updateLed() {
  if (leakAnyAlarm()) {                 // Wasseralarm: schnelles Blinken, hat Vorrang
    digitalWrite(LED_PIN, ((millis() / 150UL) % 2UL) ? HIGH : LOW);
    return;
  }
  bool vent = ventilationRecommended;
#if USE_OLED
  static uint32_t lastGauge = 0;
  static bool gaugeVent = false;
  if (millis() - lastGauge > 500) { lastGauge = millis(); gaugeVent = gaugeSaysVentilate(); }
  vent = vent || gaugeVent;
#endif
  if (vent) {
    digitalWrite(LED_PIN, HIGH);   // dauerhaft an, solange Lueften empfohlen ist
    return;
  }
  digitalWrite(LED_PIN, (lastRxMs && millis() - lastRxMs < 80) ? HIGH : LOW);
}

/* ------------------------------------------------------------------ *
 *  6) WLAN: Status-Webseite + OTA-Updates                            *
 * ------------------------------------------------------------------ */
#if USE_WIFI
static WebServer server(80);
static bool      netServicesUp = false;

static void handleAndere() {
  String o = "Meldungen ohne Temperatur/Feuchte (neueste zuerst), insgesamt seit Start: " + String(otherTotal) + "\n\n";
  int n = otherTotal < OTHER_KEEP ? (int)otherTotal : OTHER_KEEP;
  for (int k = 0; k < n; k++) {
    int i = (otherTotal - 1 - k) % OTHER_KEEP;
    o += "vor " + String((millis() - otherMs[i]) / 1000) + " s: " + otherMsg[i] + "\n";
  }
  if (!n) o += "(noch keine)\n";
  server.send(200, "text/plain; charset=utf-8", o);
}

static void handleJson() {
  DynamicJsonDocument doc(4096);
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < sensorCount; i++) {
    Sensor &s = sensors[i];
    JsonObject o = arr.createNestedObject();
    o["model"]   = s.model;
    char dispName[21];
    if (getDisplayName(s.model, s.channel, s.id, dispName, sizeof(dispName)))
      o["name"] = String(dispName);   // eigener Anzeigename (siehe /alle), falls vergeben
    o["channel"] = s.channel;
    o["id"]      = s.id;
    o["t"]       = s.t;
    o["rh"]      = s.rh;
    o["ah"]      = absoluteHumidity(s.t, s.rh);   // absolute Feuchte, g/m3, aus t+rh berechnet
    o["dew"]     = dewPointC(s.t, s.rh);          // Taupunkt, Grad C, aus t+rh berechnet
    o["battery_ok"] = s.batteryOk;
    if (!isnan(s.rssi)) o["rssi"] = (int)lroundf(s.rssi);   // Empfangsstaerke in dBm, falls bekannt
    o["age_s"]   = ageSec(s);
    o["count"]   = s.count;
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

// Frischeampel fuer die letzte Empfangszeit: frisch/traege/abgelaufen.
static const char *freshClass(uint32_t age) {
  if (age < 120)  return "fresh";
  if (age < 900)  return "stale";
  return "old";
}

// String(float,decimals) von Arduino kann fuehrende Leerzeichen einfuegen
// (dtostrf-Padding) - das macht z.B. das value-Attribut eines <input type=number>
// fuer den Browser ungueltig (Feld erscheint leer). snprintf ist sauber.
static String numStr(float v, int decimals) {
  char b[12];
  snprintf(b, sizeof(b), "%.*f", decimals, v);
  return String(b);
}

// SVG-Sparkline (Polyline) aus dem Ringpuffer eines Sensors - useTemp=false
// zeichnet die Luftfeuchte. Skaliert automatisch auf min/max im Puffer.
static String sparklineSvg(const History *h, bool useTemp, const char *stroke) {
  const int W = 220, H = 34;
  if (!h || h->count < 2)
    return String("<div class=spark-empty>noch zu wenig Verlauf</div>");

  int n = h->count;
  int startIdx = (h->head - n + HIST_LEN) % HIST_LEN;
  float vmin = 1e9f, vmax = -1e9f;
  for (int i = 0; i < n; i++) {
    int idx = (startIdx + i) % HIST_LEN;
    float v = useTemp ? h->pts[idx].temp : h->pts[idx].rh;
    if (v < vmin) vmin = v;
    if (v > vmax) vmax = v;
  }
  if (vmax - vmin < 0.5f) { vmax += 0.5f; vmin -= 0.5f; }   // sonst kollabiert die Skalierung bei fast konstanten Werten

  String pts;
  const int step = (n > 30) ? (n + 29) / 30 : 1;       // Seite schlank halten: max. ~30 Punkte je Kurve
  pts.reserve((size_t)(n / step + 1) * 9);
  for (int i = 0; i < n; i += step) {
    int idx = (startIdx + i) % HIST_LEN;
    float v = useTemp ? h->pts[idx].temp : h->pts[idx].rh;
    float x = (n > 1) ? (float)i / (n - 1) * W : 0;
    float y = H - ((v - vmin) / (vmax - vmin)) * H;
    char b[16];
    snprintf(b, sizeof(b), "%s%.0f,%.1f", i ? " " : "", x, y);
    pts += b;
  }

  char range[28];
  snprintf(range, sizeof(range), useTemp ? "%.1f..%.1f&deg;C" : "%.0f..%.0f%%", vmin, vmax);

  return String("<svg class=spark viewBox='0 0 ") + W + " " + H +
         "' preserveAspectRatio='none'><polyline points='" + pts +
         "' fill='none' stroke='" + stroke + "' stroke-width='1.6' "
         "stroke-linejoin='round' stroke-linecap='round'/></svg>"
         "<span class=spark-range>" + range + "</span>";
}

// Zeitspanne, die der Ringpuffer gerade abdeckt (Text fuer die Verlaufs-Ueberschrift).
static String historySpanText(const History *h) {
  if (!h || h->count < 2) return String("");
  int newest = (h->head - 1 + HIST_LEN) % HIST_LEN;
  int oldest = (h->head - h->count + HIST_LEN) % HIST_LEN;
  uint32_t spanSec = h->pts[newest].t - h->pts[oldest].t;
  uint32_t m = spanSec / 60;
  char b[24];
  if (m < 60) snprintf(b, sizeof(b), "%lu min", (unsigned long)m);
  else        snprintf(b, sizeof(b), "%lu:%02lu h", (unsigned long)(m / 60), (unsigned long)(m % 60));
  return String(b);
}

// Gepufferte Ausgabe fuer chunked Seiten: kleine Haeppchen werden gesammelt und erst ab ~1,2 KB
// gesendet. Jeder einzelne sendContent() ist sonst ein eigenes TCP-Paket, und mit Nagle/Delayed-ACK
// dauert eine Seite aus vielen Dutzend Mini-Chunks mehrere Sekunden.
// Fester Puffer statt String: nach laengerer Laufzeit ist der Heap zerstueckelt, ein reserve() von
// 1,5 KB schlug dann still fehl und Teile der Seite gingen verloren.
static char   txBuf[1700];
static size_t txLen = 0;
static void txFlush() {
  if (txLen) { server.sendContent(txBuf, txLen); txLen = 0; }
}
static void scBegin() { txLen = 0; }
static void sc(const char *p) {
  size_t n = strlen(p);
  while (n) {
    size_t room = sizeof(txBuf) - txLen;
    size_t k = n < room ? n : room;
    memcpy(txBuf + txLen, p, k);
    txLen += k; p += k; n -= k;
    if (txLen >= sizeof(txBuf) - 1) txFlush();
  }
}
static void sc(const String &t) { sc(t.c_str()); }
static void scEnd() {
  txFlush();
  server.sendContent("");   // beendet den chunked-Transfer
}

static void handleRoot() {
  int idx[MAX_SENSORS];
  int n = visibleSensors(idx, MAX_SENSORS);

  // Wie schon bei /settings: die Seite wird nicht mehr in einem grossen String
  // zusammengebaut, sondern haeppchenweise per sendContent() direkt raus-
  // geschrieben (HTTP chunked transfer). Seit die Karten hier durch die
  // groesser dargestellte abs. Feuchte und den eigenen Abweichungs-Block
  // deutlich mehr Inhalt haben, waere sonst bei vielen Sensoren + aktivem
  // Vergleich derselbe Heap-Fragmentierungs-Absturz wie frueher bei /settings
  // moeglich. Kleine, einzeln gesendete Haeppchen brauchen nie einen grossen
  // zusammenhaengenden Speicherblock.
  scBegin();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");   // nur Header - Inhalt kommt gepuffert in kleinen Stuecken
  sc(
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<meta http-equiv=refresh content=30>"
    "<title>868-Empfaenger</title><style>"
    ":root{color-scheme:light dark}"
    "*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;"
    "padding:1.4em 1em 2.4em;background:#0f172a;color:#e6ebf5}"
    "header{max-width:46em;margin:0 auto 1.2em}"
    "h1{font-size:1.4em;margin:.1em 0;display:flex;align-items:center;gap:.4em}"
    "header p{color:#93a0b8;margin:.2em 0;font-size:.92em}"
    ".grid{max-width:46em;margin:0 auto;display:grid;"
    "grid-template-columns:1fr;gap:.9em}"
    ".card{position:relative;background:linear-gradient(160deg,#1b2740,#141d31);"
    "border:1px solid #263352;border-radius:14px;padding:1em 1.1em;"
    "box-shadow:0 4px 14px rgba(0,0,0,.25)}"
    ".card-head{font-weight:600;font-size:.98em;margin-bottom:.3em}"
    ".chan{display:block;font-weight:400;color:#8291ad;font-size:.78em;margin-top:.15em}"
    ".bigrow{display:flex;gap:1.4em;flex-wrap:wrap;margin:.35em 0}"
    ".bigcap{display:block;font-size:.68em;color:#8291ad;margin-bottom:.1em}"
    ".temp,.hum,.ah,.td{font-size:2em;font-weight:700;line-height:1;margin:0}"
    ".temp{color:#7dd3fc}"
    ".hum{color:#34d399}"
    ".ah{color:#c4b5fd}"
    ".td{color:#fbbf24}"
    ".temp .unit,.hum .unit,.ah .unit,.td .unit{font-size:.48em;font-weight:400;color:#9fb4d8;margin-left:.1em}"
    ".refbadge{background:#0369a1;color:#fff;font-size:.62em;font-weight:600;"
    "border-radius:999px;padding:.15em .55em;vertical-align:middle;margin-left:.3em}"
    ".cmpblock{margin:.8em 0 0;padding-top:.7em;border-top:1px dashed #263352}"
    ".cmpblock-head{font-size:.72em;color:#8291ad;margin-bottom:.4em;font-weight:600}"
    ".cmpblock-head b{color:#c3cee3;font-weight:600}"
    ".delta{font-size:1.6em;font-weight:700;line-height:1;margin:0;color:#8291ad}"
    ".delta.warn{color:#e2934f}"
    ".delta .unit{font-size:.5em;font-weight:400;margin-left:.1em}"
    ".ventrec{margin-top:.7em;background:#0c2f1a;border:1px solid #22c55e;border-radius:10px;"
    "padding:.5em .8em;font-size:.85em;font-weight:600;color:#86efac}"
    ".stats{display:flex;flex-wrap:wrap;align-items:center;gap:.4em;margin-top:.5em}"
    ".limcap{font-size:.78em;color:#8291ad}"
    ".pill{background:#0f1a30;border:1px solid #263352;border-radius:999px;"
    "padding:.22em .7em;font-size:.78em;color:#c3cee3}"
    ".pill-warn{background:#2c2013;border:1px solid #7a3f16;border-radius:999px;"
    "padding:.22em .7em;font-size:.78em;color:#f0c090}"
    ".fresh-fresh,.fresh-stale,.fresh-old{position:absolute;top:.9em;right:.9em;"
    "width:.6em;height:.6em;border-radius:50%}"
    ".fresh-fresh{background:#22c55e;box-shadow:0 0 .5em #22c55e}"
    ".fresh-stale{background:#eab308}"
    ".fresh-old{background:#64748b}"
    ".spark-block{margin-top:.7em}"
    ".spark-cap{font-size:.72em;color:#8291ad;margin-bottom:.15em}"
    ".spark{width:100%;height:34px;display:block}"
    ".spark-range{display:block;text-align:right;font-size:.68em;color:#6b7690;margin-top:-2px}"
    ".spark-empty{font-size:.75em;color:#5f6d87;padding:.3em 0}"
    ".empty{max-width:46em;margin:2em auto;text-align:center;color:#93a0b8}"
    "footer{max-width:46em;margin:1.6em auto 0;color:#7c8aa5;font-size:.82em;"
    "border-top:1px solid #263352;padding-top:.8em}"
    "footer a{color:#7dd3fc}"
    ".navlink{color:#7dd3fc;text-decoration:none;font-size:.88em}"
    ".howto{max-width:46em;margin:1em auto 0;background:#141d31;border:1px solid #263352;"
    "border-radius:12px;padding:.7em 1.1em;font-size:.86em;color:#c3cee3}"
    ".howto summary{cursor:pointer;font-weight:600;color:#e6ebf5;list-style:revert}"
    ".howto ul{margin:.7em 0 .1em;padding-left:1.2em}"
    ".howto li{margin:.45em 0;line-height:1.45}"
    ".howto a{color:#7dd3fc}"
    ".dispbox{max-width:46em;margin:1em auto 0;background:#141d31;border:1px solid #263352;"
    "border-radius:12px;padding:.8em 1.1em;font-size:.86em;color:#c3cee3}"
    ".dispbox b{color:#e6ebf5}"
    ".oledprev{display:block;margin-top:.7em;width:256px;max-width:100%;height:auto;"
    "image-rendering:pixelated;background:#000;border:1px solid #263352;border-radius:6px}"
    "@media(prefers-color-scheme:light){body{background:#f3f5fb;color:#1c2333}"
    "header p{color:#5b6785}.card{background:#fff;border-color:#e1e6f0;"
    "box-shadow:0 2px 10px rgba(30,41,70,.08)}"
    ".chan,.bigcap{color:#6b7690}.temp{color:#0369a1}.hum{color:#059669}.ah{color:#7c3aed}.td{color:#b45309}"
    ".temp .unit,.hum .unit,.ah .unit,.td .unit{color:#3b4863}"
    ".pill{background:#eef2fa;border-color:#dde3f0;color:#3b4863}"
    ".pill-warn{background:#fbe9dc;border-color:#e7b98c;color:#7a3f16}"
    ".limcap{color:#6b7690}"
    ".spark-cap,.spark-range,.spark-empty{color:#5b6785}"
    ".cmpblock{border-color:#e1e6f0}.cmpblock-head{color:#6b7690}.cmpblock-head b{color:#1c2333}"
    ".delta{color:#5b6785}.delta.warn{color:#b45309}"
    ".ventrec{background:#dcfce7;border-color:#86efac;color:#14532d}"
    ".empty{color:#5b6785}footer{color:#5b6785;border-color:#e1e6f0}"
    ".navlink{color:#0369a1}"
    ".howto{background:#fff;border-color:#e1e6f0;color:#334}"
    ".howto summary{color:#1c2333}.howto a{color:#0369a1}"
    ".dispbox{background:#fff;border-color:#e1e6f0;color:#334}.dispbox b{color:#1c2333}"
    ".oledprev{border-color:#e1e6f0}}"
    "</style></head><body>");
  sc(
    "<header><h1>&#128225; 868-Empf&auml;nger</h1>"
    "<p><strong>" + String(n) + "</strong> von <strong>" + String(sensorCount) +
    "</strong> gesehenen Sensor(en) angezeigt &middot; "
    "<code>" + String(NET_HOSTNAME) + ".local</code></p>"
    "<p><a class=navlink href='/alle'>&#128203;&nbsp;Alle Sensoren verwalten</a>"
    " &middot; <a class=navlink href='/settings'>&#9881;&nbsp;Schwellenwerte / ntfy einrichten</a>"
    " &middot; <a class=navlink href='/wifi'>&#128246;&nbsp;WLAN wechseln</a>"
#if USE_OLED
    " &middot; <a class=navlink href='/display'>&#128250;&nbsp;Display-Anzeige</a>"
#endif
    " &middot; <a class=navlink href='/wasser'>&#128167;&nbsp;Wassersensoren</a>"
    "</p>");
  if (leakAnyAlarm()) sc(leakBannerHtml());
  sc("<div class=grid>");

  if (n == 0 && sensorCount == 0) {
    sc("<div class=empty>&#128225; warte auf Sensoren &hellip;</div>");
  } else if (n == 0 && !anyOwned()) {
    sc("<div class=empty>&#128225; " + String(sensorCount) +
            " Sensor(en) empfangen, aber keiner als eigen markiert &ndash; "
            "<a class=navlink href='/alle'>unter /alle ausw&auml;hlen</a>.</div>");
  } else if (n == 0) {
    sc("<div class=empty>&#128225; eigene Sensoren markiert, aber seit dem "
            "letzten Neustart noch keiner davon empfangen &hellip;</div>");
  }

  for (int k = 0; k < n; k++) {
    Sensor &s = sensors[idx[k]];
    uint32_t age = ageSec(s);
    History *h = findHistory(s.model, s.channel, s.id);
    String span = historySpanText(h);

    char dispName[21];
    bool named = getDisplayName(s.model, s.channel, s.id, dispName, sizeof(dispName));
    bool isRef = refSensor.used && refSensor.channel == s.channel && refSensor.id == s.id &&
                 strcmp(refSensor.model, s.model) == 0;

    sc("<div class=card><div class=fresh-" + String(freshClass(age)) + "></div>");
    sc("<div class=card-head>" + String(named ? dispName : s.model) +
            (isRef ? String(" <span class=refbadge>Referenz</span>") : String("")) +
            " <span class=chan>" +
            (named ? String(s.model) + " &middot; " : String("")) + "Kanal " +
            String(s.channel) + " &middot; ID " + String(s.id) + "</span></div>");
    float ah = absoluteHumidity(s.t, s.rh);
    sc("<div class=bigrow>"
            "<div class=bigcol><span class=bigcap>Temperatur</span>"
            "<div class=temp>" + numStr(s.t, 1) + "<span class=unit>&deg;C</span></div></div>"
            "<div class=bigcol><span class=bigcap>Luftfeuchte</span>"
            "<div class=hum>" + numStr(s.rh, 0) + "<span class=unit>%</span></div></div>"
            "<div class=bigcol><span class=bigcap>Abs. Feuchte</span>"
            "<div class=ah>" + numStr(ah, 1) + "<span class=unit>g/m&sup3;</span></div></div>"
            "<div class=bigcol><span class=bigcap>Taupunkt</span>"
            "<div class=td>" + numStr(dewPointC(s.t, s.rh), 1) + "<span class=unit>&deg;C</span></div></div>"
            "</div>");
    sc("<div class=stats><span class=pill>vor " + String((unsigned long)age) + "s</span>"
            "<span class=pill>" + String((unsigned long)s.count) + "&times; empfangen</span>" +
            (s.batteryOk ? String("") : "<span class=pill-warn>&#128267; Batterie schwach</span>") +
            "</div>");

    // Nur die tatsaechlich eingetragenen Grenzwerte kompakt anzeigen (nichts,
    // wenn fuer diesen Sensor gar keine gesetzt sind) - auf einen Blick sehen,
    // was bereits konfiguriert ist, ohne dafuer extra auf /settings wechseln
    // zu muessen.
    {
      Threshold *lc = findThreshold(s.model, s.channel, s.id);
      if (lc) {
        String limPills;
        if (lc->tMin > -900)  limPills += "<span class=pill>T &ge; " + numStr(lc->tMin, 1) + "&deg;C</span>";
        if (lc->tMax < 900)   limPills += "<span class=pill>T &le; " + numStr(lc->tMax, 1) + "&deg;C</span>";
        if (lc->rhMin > -900) limPills += "<span class=pill>rF &ge; " + numStr(lc->rhMin, 0) + "%</span>";
        if (lc->rhMax < 900)  limPills += "<span class=pill>rF &le; " + numStr(lc->rhMax, 0) + "%</span>";
        if (lc->cmpEnabled) {
          if (lc->cmpDT < 900)  limPills += "<span class=pill>&Delta;T &le; " + numStr(lc->cmpDT, 1) + "&deg;C</span>";
          if (lc->cmpDRH < 900) limPills += "<span class=pill>&Delta;rF &le; " + numStr(lc->cmpDRH, 0) + "%</span>";
          if (lc->cmpDAH < 900) limPills += "<span class=pill>&Delta;AH &le; " + numStr(lc->cmpDAH, 1) + "&nbsp;g/m&sup3;</span>";
        }
        if (limPills.length())
          sc("<div class=stats><span class=limcap>Limits:</span>" + limPills + "</div>");
      }
    }

    // Vergleich mit dem Referenzsensor (falls fuer diesen Sensor aktiviert) -
    // live berechnete Differenzen, nicht nur der Alarmzustand. Genauso gross/
    // deutlich dargestellt wie Temperatur/Luftfeuchte oben (eigener Block mit
    // Ueberschrift statt einer kleinen Zeile), damit man die Abweichung auf
    // einen Blick erfasst. Jede Differenz wird amber/fett hervorgehoben, wenn
    // sie aktuell den jeweiligen Grenzwert ueberschreitet (gleicher
    // Alarmzustand wie fuer die ntfy-Meldung).
    {
      Threshold *tc = findThreshold(s.model, s.channel, s.id);
      bool isRefSelf = refSensor.used && refSensor.channel == s.channel && refSensor.id == s.id &&
                        strcmp(refSensor.model, s.model) == 0;
      if (refSensor.used && tc && tc->cmpEnabled && !isRefSelf) {
        const Sensor *refS = findLiveSensor(refSensor.model, refSensor.channel, refSensor.id);
        if (refS) {
          char refLbl[21];
          bool refNamed = getDisplayName(refS->model, refS->channel, refS->id, refLbl, sizeof(refLbl));
          String refName = refNamed ? String(refLbl) : String(refS->model);
          float refAH = absoluteHumidity(refS->t, refS->rh);
          float dT  = fabsf(s.t - refS->t);
          float dRH = fabsf(s.rh - refS->rh);
          float dAH = fabsf(ah - refAH);
          sc("<div class=cmpblock>"
                  "<div class=cmpblock-head>Abweichung zu <b>" + refName + "</b></div>"
                  "<div class=bigrow>"
                  "<div class=bigcol><span class=bigcap>&Delta; Temperatur</span>"
                  "<div class='delta" + String(tc->alarmCmpT ? " warn" : "") + "'>" + numStr(dT, 1) +
                  "<span class=unit>&deg;C</span></div></div>"
                  "<div class=bigcol><span class=bigcap>&Delta; Luftfeuchte</span>"
                  "<div class='delta" + String(tc->alarmCmpRH ? " warn" : "") + "'>" + numStr(dRH, 0) +
                  "<span class=unit>%</span></div></div>"
                  "<div class=bigcol><span class=bigcap>&Delta; Abs. Feuchte</span>"
                  "<div class='delta" + String(tc->alarmCmpAH ? " warn" : "") + "'>" + numStr(dAH, 1) +
                  "<span class=unit>g/m&sup3;</span></div></div>"
                  "</div>");
          // Luefungsempfehlung: nur sinnvoll, wenn DIESER Sensor feuchter ist als
          // die Referenz (Luften bringt dann trockenere Referenzluft rein) - bei
          // umgekehrtem Vorzeichen wuerde Lueften die Feuchte hier nur erhoehen.
          // Nutzt denselben Schwellenwert+Hysterese wie der Alarm (alarmCmpAH),
          // die Empfehlung verschwindet also automatisch wieder, sobald die
          // Differenz unter die Hysterese-Schwelle faellt.
          if (tc->alarmCmpAH && ah > refAH) {
            sc("<div class=ventrec>&#128168; L&uuml;ftungsempfehlung: deutlich "
                    "feuchter als " + refName + " &ndash; jetzt l&uuml;ften</div>");
          }
          sc("</div>");
        }
      }
    }

    sc("<div class=spark-block><div class=spark-cap>Temp-Verlauf" +
            (span.length() ? String(" &middot; ") + span : String("")) + "</div>" +
            sparklineSvg(h, true, "#38bdf8") + "</div>");
    sc("<div class=spark-block><div class=spark-cap>rF-Verlauf</div>" +
            sparklineSvg(h, false, "#34d399") + "</div>");
    sc("</div>");
  }

  sc("</div>");

#if USE_OLED
  // Live-Vorschau des Displays (wie auf dem Geraet). Das Bild wird erst nach dem vorherigen Bild und
  // nur bei sichtbarem Tab neu geholt (alle 4 s), damit der Webserver nicht dauernd beschaeftigt ist.
  // Welche Anzeigen laufen, wird im Untermenue /display eingestellt.
  sc("<div class=dispbox><b>&#128250; Display</b> "
     "<a class=navlink href='/display'>&middot; Anzeige &auml;ndern &rarr;</a>"
     "<img id=oledprev class=oledprev src='/oled.bmp' alt='Display-Vorschau' width=256 height=128>"
     "<script>var p=document.getElementById('oledprev');function n(){setTimeout(function(){"
     "if(!document.hidden)p.src='/oled.bmp?'+Date.now();else n()},4000)}p.onload=p.onerror=n;n()</script></div>");
#endif

  // Die Erklaerung steht jetzt UNTER den Sensor-Karten statt darueber - so sieht
  // man die eigentlichen Messwerte sofort beim Laden, ohne erst daran vorbei-
  // scrollen zu muessen.
  sc(
    "<details class=howto><summary>Wie funktioniert das?</summary><ul>"
    "<li>Das Board h&ouml;rt auf 868,35&nbsp;MHz. Unter <a href='/alle'>/alle</a> w&auml;hlst du "
    "die eigenen Sensoren und vergibst Namen, unter <a href='/display'>/display</a> das OLED.</li>"
    "<li>Unter <a href='/settings'>/settings</a>: Grenzwerte, ntfy-Alarm, Referenzsensor-Vergleich; "
    "unter <a href='/wifi'>/wifi</a> das WLAN (ohne WLAN macht das Board einen Hotspot auf).</li>"
    "<li>Die Verlaufs-Grafiken zeigen die letzten ~60 Empf&auml;nge, nur im RAM.</li>"
    "</ul></details>");

  sc(
    "<footer>Uptime " + String((unsigned long)(millis() / 1000)) + "&nbsp;s &middot; WLAN " +
    String((int)WiFi.RSSI()) + "&nbsp;dBm &middot; Heap " + String((unsigned)(ESP.getFreeHeap() / 1024)) +
    "/" + String((unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024)) +
    "&nbsp;kB &middot; <a href='/json'>/json</a> &middot; "
    "Auto-Refresh alle 30&nbsp;s</footer></body></html>");
  scEnd();
}

// ------------------------------------------------------------------------
//  ntfy.sh-Push. Einfacher HTTP-POST (kein TLS noetig, ntfy.sh nimmt Klartext-
//  HTTP an) - baugleich zum bewaehrten Muster aus feuchtewaechter-433, dort
//  laeuft HTTPS wegen zu wenig zusammenhaengendem Heap nicht zuverlaessig.
//  force=true (z.B. Testmeldung) ignoriert die Boot-Ruhe.
// ------------------------------------------------------------------------
static const uint32_t NTFY_BOOT_QUIET_MS = 30000;   // erste 30s nach Boot: keine Push-Meldungen

static void notify(const char *msg, bool force) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.printf("ntfy uebersprungen (kein WLAN): %s\n", msg);
    return;
  }
  if (!force && millis() < NTFY_BOOT_QUIET_MS) {
    Serial.printf("ntfy uebersprungen (Boot-Ruhe): %s\n", msg);
    return;
  }
  HTTPClient http;
  http.begin("http://ntfy.sh/" NTFY_TOPIC);
  http.addHeader("Content-Type", "text/plain; charset=utf-8");
  http.addHeader("Title", "868-Empfaenger");
  int code = http.POST((uint8_t *)msg, strlen(msg));
  Serial.printf("ntfy [%d]: %s\n", code, msg);
  http.end();
}

// ------------------------------------------------------------------------
//  ThingSpeak: dauerhafte Aufzeichnung/Verlauf ueber die Board-Laufzeit
//  hinaus (im Gegensatz zum Ringpuffer im RAM, der bei jedem Neustart weg
//  ist). Nur kompiliert, wenn USE_THINGSPEAK=1 (siehe platformio.ini).
//  API-Key + An/Aus sind zusaetzlich zur Compile-Zeit-Option ueber /settings
//  einstellbar (Namespace "thingspk", ueberschreibt THINGSPEAK_API_KEY aus
//  secrets.h - gleiches Muster wie die WLAN-Zugangsdaten). Zuordnung erfolgt
//  FELD-zentriert unten auf /settings ("ThingSpeak - Felder"): pro Kanal-Feld
//  (1-8) waehlt man aus, welcher Sensor-Wert dort hineingeht - Temperatur,
//  rel. Feuchte, abs. Feuchte, oder die jeweilige Abweichung (Delta) zum
//  Referenzsensor, siehe Threshold.tsEnT/tsEnRH/tsEnAH/tsEnDT/tsEnDRH/tsEnDAH +
//  die zugehoerigen tsField*. Einfacher HTTP-GET, kein TLS noetig (ThingSpeak
//  nimmt Klartext-HTTP an) - Free-Tier
//  erlaubt max. 1 Update alle 15s pro Channel, aufgerufen wird alle 60s aus
//  loop() (siehe dort), das reicht mit deutlichem Puffer.
// ------------------------------------------------------------------------
#if USE_THINGSPEAK
static char tsApiKeyOverride[41] = "";   // leer = THINGSPEAK_API_KEY aus secrets.h nutzen
static bool tsRuntimeOn = true;          // Laufzeit-Schalter, ohne Neu-Flash umschaltbar
static Preferences prefsTs;

static void loadThingSpeakSettings() {
  prefsTs.begin("thingspk", true);
  String key = prefsTs.getString("key", "");
  tsRuntimeOn = prefsTs.getBool("on", true);
  prefsTs.end();
  strncpy(tsApiKeyOverride, key.c_str(), sizeof(tsApiKeyOverride) - 1);
  tsApiKeyOverride[sizeof(tsApiKeyOverride) - 1] = 0;
}

static void saveThingSpeakSettings() {
  prefsTs.begin("thingspk", false);
  prefsTs.putString("key", tsApiKeyOverride);
  prefsTs.putBool("on", tsRuntimeOn);
  prefsTs.end();
}

// Effektiver API-Key: /settings-Eingabe hat Vorrang, sonst der aus secrets.h.
static const char *effectiveTsApiKey() {
  return tsApiKeyOverride[0] ? tsApiKeyOverride : THINGSPEAK_API_KEY;
}

static void sendThingSpeak() {
  if (!tsRuntimeOn) return;
  if (WiFi.status() != WL_CONNECTED) return;
  const char *key = effectiveTsApiKey();
  if (!key[0]) return;   // kein Key eingetragen -> nichts zu tun

  // Fuer die drei Delta-Werte (Abweichung zum Referenzsensor) wird der
  // Referenzsensor selbst gerade LIVE gebraucht - ohne gesetzten/aktuell
  // empfangenden Referenzsensor bleiben diese drei Felder schlicht unbelegt,
  // die T/rF/AH-Rohwerte werden davon nicht beeinflusst.
  const Sensor *refS = refSensor.used
    ? findLiveSensor(refSensor.model, refSensor.channel, refSensor.id) : nullptr;

  String url = "http://api.thingspeak.com/update?api_key=" + String(key);
  int sent = 0;    // Sensoren, die mindestens einen Wert beisteuern
  int fields = 0;  // tatsaechlich gesetzte Felder, fuer die Log-Zeile
  for (int i = 0; i < MAX_THRESH; i++) {
    Threshold &t = thresholds[i];
    bool anyDelta = t.tsEnDT || t.tsEnDRH || t.tsEnDAH;
    if (!t.used || (!t.tsEnT && !t.tsEnRH && !t.tsEnAH && !anyDelta)) continue;
    const Sensor *s = findLiveSensor(t.model, t.channel, t.id);
    if (!s) continue;
    bool contributed = false;
    if (t.tsEnT && t.tsFieldT >= 1 && t.tsFieldT <= 8) {
      url += "&field" + String(t.tsFieldT) + "=" + numStr(s->t, 1);
      contributed = true; fields++;
    }
    if (t.tsEnRH && t.tsFieldRH >= 1 && t.tsFieldRH <= 8) {
      url += "&field" + String(t.tsFieldRH) + "=" + numStr((float)s->rh, 0);
      contributed = true; fields++;
    }
    if (t.tsEnAH && t.tsFieldAH >= 1 && t.tsFieldAH <= 8) {
      url += "&field" + String(t.tsFieldAH) + "=" + numStr(absoluteHumidity(s->t, s->rh), 1);
      contributed = true; fields++;
    }
    if (refS && s != refS) {
      if (t.tsEnDT && t.tsFieldDT >= 1 && t.tsFieldDT <= 8) {
        url += "&field" + String(t.tsFieldDT) + "=" + numStr(s->t - refS->t, 1);
        contributed = true; fields++;
      }
      if (t.tsEnDRH && t.tsFieldDRH >= 1 && t.tsFieldDRH <= 8) {
        url += "&field" + String(t.tsFieldDRH) + "=" + numStr((float)(s->rh - refS->rh), 0);
        contributed = true; fields++;
      }
      if (t.tsEnDAH && t.tsFieldDAH >= 1 && t.tsFieldDAH <= 8) {
        url += "&field" + String(t.tsFieldDAH) + "=" +
               numStr(absoluteHumidity(s->t, s->rh) - absoluteHumidity(refS->t, refS->rh), 1);
        contributed = true; fields++;
      }
    }
    if (contributed) sent++;
  }
  if (fields == 0) return;   // nichts ausgewaehlt -> keinen leeren Request schicken

  HTTPClient http;
  http.begin(url);
  int code = http.GET();
  Serial.printf("ThingSpeak [%d]: %d Sensor(en), %d Feld(er) gesendet\n", code, sent, fields);
  http.end();
}
#endif

// ------------------------------------------------------------------------
//  /settings: Schwellenwerte je aktuell bekanntem Sensor eingeben/aendern.
// ------------------------------------------------------------------------
static String limitStr(float v, bool isMin) {
  if ((isMin && v <= -900) || (!isMin && v >= 900)) return String("");
  char b[10];
  snprintf(b, sizeof(b), "%.1f", v);
  return String(b);
}

#if USE_THINGSPEAK
// Feld-zentrierte ThingSpeak-Zuordnung: statt je Sensor "auf welches Feld
// senden" zu fragen, listet der eigene Bereich am Ende von /settings die 8
// Kanal-Felder auf und laesst je Feld waehlen, WELCHER Sensor-Wert (Temperatur/
// rel. Feuchte/abs. Feuchte/Delta-T/Delta-rF/Delta-AH) dort hineingeht. Die
// Optionsliste (alle moeglichen Sensor-Wert-Kombinationen) wird waehrend des
// Sensor-Kartendurchlaufs in handleSettingsGet() einmal befuellt
// (tsAddOption()) und hier fuer jedes der 8 Felder wiederverwendet - das
// erspart, sie 8x neu aufzubauen. tsFieldOwner[f] haelt fest, welche
// Kombination aktuell auf Feld f zeigt (Format "<Sensorindex>_<Typ>", z.B.
// "2_DAH"), damit das richtige <option> als "selected" markiert wird.
static const int TS_MAX_OPTIONS = MAX_THRESH * 6;   // 3 Rohwerte + 3 Delta-Werte je Sensor
static String tsOptVal[TS_MAX_OPTIONS];
static String tsOptLabel[TS_MAX_OPTIONS];
static int    tsOptCount = 0;
static String tsFieldOwner[9];   // Index 1..8, Wert "<i>_<Typ>" oder leer = unbelegt

static void tsAddOption(int idx, const char *suffix, const char *txt, const String &label,
                         bool enabled, int field) {
  if (tsOptCount >= TS_MAX_OPTIONS) return;
  String val = String(idx) + "_" + suffix;
  tsOptVal[tsOptCount] = val;
  tsOptLabel[tsOptCount] = label + " &ndash; " + txt;
  tsOptCount++;
  if (enabled && field >= 1 && field <= 8) tsFieldOwner[field] = val;
}

// Baut das <select> fuer EIN Kanal-Feld (1-8) aus der oben befuellten
// Optionsliste, inkl. "nicht belegt".
static String tsFieldAssignSelect(int fieldNum) {
  const String &owner = tsFieldOwner[fieldNum];
  String out = "<select name='tsf" + String(fieldNum) + "'>";
  out += "<option value='none'" + String(owner.length() == 0 ? " selected" : "") +
         ">&mdash; nicht belegt &mdash;</option>";
  for (int k = 0; k < tsOptCount; k++) {
    out += "<option value='" + tsOptVal[k] + "'" + (tsOptVal[k] == owner ? " selected" : "") +
           ">" + tsOptLabel[k] + "</option>";
  }
  out += "</select>";
  return out;
}
#endif

// Konsolidierte Ntfy-Auswahl: statt der ntfy-Checkboxen verteilt auf jeder
// einzelnen Sensor-Karte gibt es einen eigenen Bereich ("Unterpunkt"), der je
// Sensor eine Zeile mit allen ntfy-Checkboxen auf einen Blick zeigt. Jede
// Zeile wird waehrend des Sensor-Kartendurchlaufs in handleSettingsGet() als
// EIGENE, kleine Ntfy-Zeile gepuffert und danach einzeln per sendContent()
// gesendet - NIE zu einem grossen String zusammenhaengen (siehe der bereits
// behobene Truncation-Bug bei den ThingSpeak-Feldern weiter oben).
static String ntfyRow[MAX_THRESH];
static int    ntfyRowCount = 0;

static void handleSettingsGet() {
  // Nur die als "eigen" markierten Sender beruecksichtigen (gleicher Filter
  // wie schon auf / und dem OLED, siehe visibleSensors()) - Nachbarsensoren/
  // Fehldekodierungen, die man nie ueber /alle als eigen markiert hat, tauchen
  // dann in den Schwellenwerten, der Ntfy-Auswahl und den ThingSpeak-Feldern
  // gar nicht erst auf. Solange noch kein Sensor als eigen markiert ist
  // (Erstbenutzung), zeigt visibleSensors() uebergangsweise weiterhin alle.
  int visIdx[MAX_SENSORS];
  int visN = visibleSensors(visIdx, MAX_SENSORS);
  int n = visN < MAX_THRESH ? visN : MAX_THRESH;

  // Die Seite wird nicht mehr in einem einzigen grossen String zusammengebaut,
  // sondern haeppchenweise direkt per sendContent() rausgeschrieben (HTTP
  // chunked transfer). Grund: selbst mit body.reserve() blieb bei vielen
  // Sensor-Karten (>10 KB Gesamtseite) ein einzelner Realloc auf einen so
  // grossen zusammenhaengenden Speicherblock noetig, der bei fragmentiertem
  // Heap fehlschlagen und das Board abstuerzen lassen konnte. Kleine, einzeln
  // gesendete Haeppchen (wenige hundert Byte) brauchen nie einen grossen
  // zusammenhaengenden Block und sind damit robust.
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8",
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>868-Empfaenger &middot; Schwellenwerte</title><style>"
    ":root{color-scheme:light dark}*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;"
    "padding:1.4em 1em 2.4em;background:#0f172a;color:#e6ebf5}"
    "h1{max-width:46em;margin:0 auto .3em;font-size:1.3em;display:flex;align-items:center;gap:.4em}"
    ".navlink{color:#7dd3fc;text-decoration:none;font-size:.88em}"
    ".back{max-width:46em;margin:0 auto 1em}"
    ".banner{max-width:46em;margin:0 auto .8em;background:#14532d;border:1px solid #22c55e;"
    "border-radius:10px;padding:.5em .9em;font-size:.9em}"
    "form,.grid{max-width:46em;margin:0 auto}"
    ".grid{display:grid;grid-template-columns:1fr;gap:.9em}"
    ".card{background:linear-gradient(160deg,#1b2740,#141d31);border:1px solid #263352;"
    "border-radius:14px;padding:1em 1.1em;box-shadow:0 4px 14px rgba(0,0,0,.25)}"
    ".card-head{font-weight:600;font-size:.98em;margin-bottom:.5em}"
    ".refbadge{background:#0369a1;color:#fff;font-size:.68em;font-weight:600;"
    "border-radius:999px;padding:.15em .55em;vertical-align:middle;margin-left:.3em}"
    ".chan{display:block;font-weight:400;color:#8291ad;font-size:.78em;margin-top:.15em}"
    ".cardrule{border:0;border-top:1px solid #263352;margin:.9em 0}"
    ".row{display:flex;gap:.7em;margin:.5em 0}"
    ".row label{flex:1;min-width:0;display:flex;flex-direction:column;gap:.3em;"
    "font-size:.8em;color:#93a0b8}"
    "input[type=number]{background:#0f1a30;border:1px solid #263352;border-radius:8px;"
    "color:#e6ebf5;padding:.4em .5em;font-size:.95em;width:100%}"
    "input[type=checkbox],input[type=radio]{width:1.1em;height:1.1em;flex:none}"
    "button{margin-top:1.1em;max-width:46em;background:#0369a1;color:#fff;border:0;"
    "border-radius:10px;padding:.6em 1.4em;font-size:1em;cursor:pointer;display:block;"
    "margin-left:auto;margin-right:auto}"
    "button:hover{background:#0284c7}"
    ".empty,.hint{max-width:46em;margin:1em auto;color:#93a0b8;font-size:.85em}"
    ".sensblock{max-width:46em;margin:0 auto 1.2em}"
    ".fieldhint{font-size:.82em;color:#93a0b8;margin:.2em 0 .8em}"
    ".ntfytoggle{display:flex;align-items:center;gap:.4em;font-size:.78em;"
    "color:#93a0b8;margin:-.2em 0 .6em}"
    ".tsassign{display:flex;align-items:center;gap:.6em;margin:.5em 0}"
    ".tsassign label{flex:none;width:4.2em;font-size:.85em;color:#93a0b8}"
    ".tsassign select{flex:1;min-width:0;background:#0f1a30;border:1px solid #263352;"
    "border-radius:8px;color:#e6ebf5;padding:.45em .5em;font-size:.88em}"
    "details.card{padding:1em 1.1em}details.card>summary{font-weight:600;"
    "font-size:.98em;cursor:pointer}details.card[open]>summary{margin-bottom:.7em}"
    ".ntfyrow{border-top:1px solid #263352;padding:.55em 0}"
    ".ntfyrow:first-of-type{border-top:0;padding-top:0}"
    ".ntfyname{font-weight:600;font-size:.85em;margin-bottom:.35em}"
    ".ntfychecks{display:flex;flex-wrap:wrap;gap:.4em 1em;font-size:.78em;color:#93a0b8}"
    ".ntfychecks label{display:flex;align-items:center;gap:.3em}"
    ".cmprow{font-size:.78em;color:#8291ad;margin:-.1em 0 .8em}"
    ".cmpval{color:#8291ad}.cmpwarn{color:#e2934f;font-weight:600}"
    ".topicbox{font-family:ui-monospace,Consolas,monospace;font-size:1.05em;"
    "background:#0f1a30;border:1px solid #263352;border-radius:8px;padding:.6em .8em;"
    "word-break:break-all;user-select:all;color:#7dd3fc}"
    "@media(prefers-color-scheme:light){body{background:#f3f5fb;color:#1c2333}"
    ".navlink{color:#0369a1}.card{background:#fff;border-color:#e1e6f0;"
    "box-shadow:0 2px 10px rgba(30,41,70,.08)}.chan{color:#6b7690}"
    ".row label{color:#5b6785}input[type=number]{background:#eef2fa;border-color:#dde3f0;color:#1c2333}"
    ".empty,.hint,.fieldhint,.ntfytoggle{color:#5b6785}.banner{background:#dcfce7;border-color:#86efac;color:#14532d}"
    ".tsassign label{color:#5b6785}.tsassign select{background:#eef2fa;border-color:#dde3f0;color:#1c2333}"
    ".ntfyrow{border-color:#e1e6f0}.ntfychecks{color:#5b6785}"
    ".cmprow,.cmpval{color:#5b6785}"
    ".topicbox{background:#eef2fa;border-color:#dde3f0;color:#0369a1}.cardrule{border-color:#e1e6f0}}"
    "</style></head><body>"
    "<h1>&#9881; Schwellenwerte</h1>"
    "<p class=back><a class=navlink href='/'>&larr; zur&uuml;ck zur &Uuml;bersicht</a></p>");

  if (server.hasArg("saved"))
    server.sendContent("<div class=banner>Gespeichert.</div>");

  server.sendContent(
    "<div class=sensblock><div class=card><div class=card-head>"
    "&#128225; Empfangsempfindlichkeit</div>"
    "<p class=fieldhint>Ein Funktelegramm gilt ab so vielen dB &uuml;ber dem laufend "
    "gemessenen Rausch-Grundpegel als Signal. Kleinerer Wert = mehr Reichweite, aber "
    "auch mehr Fehldekodierungen aus dem Rauschen selbst. Wirkt sofort, kein Neustart "
    "n&ouml;tig.</p>"
    "<form method=POST action='/settings'><div class=row>"
    "<label>RSSI-Schwelle (dB &uuml;ber Rauschgrund, " + String(RSSI_SENS_MIN) + "-" +
    String(RSSI_SENS_MAX) + ")<input name=rssi type=number min=" + String(RSSI_SENS_MIN) +
    " max=" + String(RSSI_SENS_MAX) + " step=1 value='" + String(rssiSensitivity) +
    "'></label></div>"
    "<button type=submit>Speichern</button></form></div></div>");

  // Ntfy und ThingSpeak stehen jetzt als eigene Unterpunkte (<details>) weiter
  // unten, gemeinsam mit den dazugehoerigen je-Sensor-Einstellungen, statt in
  // getrennten Karten hier oben verteilt zu sein - siehe dort.
  server.sendContent("<form method=POST action='/settings'>");
#if USE_THINGSPEAK
  tsOptCount = 0;
  for (int k = 1; k <= 8; k++) tsFieldOwner[k] = "";
#endif
  ntfyRowCount = 0;
  if (n == 0) {
    server.sendContent(sensorCount == 0
      ? "<div class=empty>Noch keine Sensoren empfangen &ndash; warte, bis mindestens einer "
        "gesendet hat, und lade die Seite dann neu.</div>"
      : "<div class=empty>Kein empfangener Sensor ist als \"eigen\" markiert &ndash; unter "
        "<a class=navlink href='/alle'>/alle</a> mindestens einen ausw&auml;hlen, dann taucht "
        "er hier auf.</div>");
  } else {
    if (visN > MAX_THRESH)
      server.sendContent("<div class=empty>Nur die ersten " + String(MAX_THRESH) +
              " eigenen Sensoren sind hier konfigurierbar.</div>");
    server.sendContent(
      "<div class=row style='max-width:46em;margin:0 auto .8em'>"
      "<label style='flex:none'><input type=radio name=refidx value='-1'" +
      String(refSensor.used ? "" : " checked") + "> Kein Referenzsensor "
      "(Vergleich deaktiviert)</label></div>");
    server.sendContent("<div class=grid>");
    for (int i = 0; i < n; i++) {
      Sensor &s = sensors[visIdx[i]];
      Threshold *c = findThreshold(s.model, s.channel, s.id);
      float tMin  = c ? c->tMin  : -999, tMax  = c ? c->tMax  : 999;
      float rhMin = c ? c->rhMin : -999, rhMax = c ? c->rhMax : 999;
      float hT = c ? c->hystT : 0.5f, hR = c ? c->hystRH : 3.0f;
      bool  cmpOn = c ? c->cmpEnabled : false;
      float cdt = c ? c->cmpDT : 999, cdr = c ? c->cmpDRH : 999, cda = c ? c->cmpDAH : 999;
      float cha = c ? c->hystAH : 0.5f;
      bool nT = c ? c->notifyT : true, nR = c ? c->notifyRH : true;
      bool nCT = c ? c->notifyCmpT : true, nCR = c ? c->notifyCmpRH : true, nCA = c ? c->notifyCmpAH : true;
#if USE_THINGSPEAK
      bool tsEnT   = c ? c->tsEnT   : false; int tsFT  = c ? c->tsFieldT   : 1;
      bool tsEnRH  = c ? c->tsEnRH  : false; int tsFR  = c ? c->tsFieldRH  : 2;
      bool tsEnAH  = c ? c->tsEnAH  : false; int tsFA  = c ? c->tsFieldAH  : 3;
      bool tsEnDT  = c ? c->tsEnDT  : false; int tsFDT = c ? c->tsFieldDT  : 4;
      bool tsEnDRH = c ? c->tsEnDRH : false; int tsFDR = c ? c->tsFieldDRH : 5;
      bool tsEnDAH = c ? c->tsEnDAH : false; int tsFDA = c ? c->tsFieldDAH : 6;
#endif
      bool  isRef = refSensor.used && refSensor.channel == s.channel && refSensor.id == s.id &&
                    strcmp(refSensor.model, s.model) == 0;
      char dispName[21];
      bool named = getDisplayName(s.model, s.channel, s.id, dispName, sizeof(dispName));
      float ah = absoluteHumidity(s.t, s.rh);
      String pfx = "f" + String(i);
#if USE_THINGSPEAK
      {
        String label = named ? String(dispName) : String(s.model);
        tsAddOption(i, "T",  "Temperatur",   label, tsEnT,  tsFT);
        tsAddOption(i, "RH", "rel. Feuchte", label, tsEnRH, tsFR);
        tsAddOption(i, "AH", "abs. Feuchte", label, tsEnAH, tsFA);
        if (!isRef) {   // Delta zu sich selbst waere immer 0 - keinen Sinn anbieten
          tsAddOption(i, "DT",  "&Delta;Temp.",       label, tsEnDT,  tsFDT);
          tsAddOption(i, "DRH", "&Delta;rel.Feuchte",  label, tsEnDRH, tsFDR);
          tsAddOption(i, "DAH", "&Delta;abs.Feuchte", label, tsEnDAH, tsFDA);
        }
      }
#endif
      // Konsolidierte Ntfy-Zeile fuer diesen Sensor - Delta-Checkboxen nur,
      // wenn er nicht selbst der Referenzsensor ist (gleiche Begruendung wie
      // bei den ThingSpeak-Delta-Optionen oben: Delta zu sich selbst ist 0).
      {
        String label = named ? String(dispName) : String(s.model);
        String row = "<div class=ntfyrow><div class=ntfyname>" + label +
                "</div><div class=ntfychecks>"
                "<label><input type=checkbox name='" + pfx + "_nt'" + (nT ? " checked" : "") +
                "> Temp.-Grenzwert</label>"
                "<label><input type=checkbox name='" + pfx + "_nr'" + (nR ? " checked" : "") +
                "> rF-Grenzwert</label>";
        if (!isRef) {
          row += "<label><input type=checkbox name='" + pfx + "_nct'" + (nCT ? " checked" : "") +
                 "> &Delta;T</label>"
                 "<label><input type=checkbox name='" + pfx + "_ncr'" + (nCR ? " checked" : "") +
                 "> &Delta;rF</label>"
                 "<label><input type=checkbox name='" + pfx + "_nca'" + (nCA ? " checked" : "") +
                 "> &Delta;abs.Feuchte</label>";
        }
        row += "</div></div>";
        if (ntfyRowCount < MAX_THRESH) ntfyRow[ntfyRowCount++] = row;
      }

      // Referenzwerte einmal vorab holen - werden gleich direkt bei den
      // jeweiligen Limit-Feldern eingeblendet (Temp/rF: reiner Istwert;
      // Delta-Felder: Abweichung zum Referenzsensor), statt in einer
      // Sammelzeile weiter oben, wo der Bezug zum passenden Feld verloren geht.
      const Sensor *refS = (refSensor.used && !isRef)
        ? findLiveSensor(refSensor.model, refSensor.channel, refSensor.id) : nullptr;
      float dT = 0, dRH = 0, dAH = 0;
      char refLbl[21] = {0};
      bool refNamed = false;
      if (refS) {
        refNamed = getDisplayName(refS->model, refS->channel, refS->id, refLbl, sizeof(refLbl));
        dT  = fabsf(s.t - refS->t);
        dRH = fabsf(s.rh - refS->rh);
        dAH = fabsf(ah - absoluteHumidity(refS->t, refS->rh));
      }

      server.sendContent("<div class=card>");
      server.sendContent("<div class=card-head>" + String(named ? dispName : s.model) +
              (isRef ? String(" <span class=refbadge>Referenz</span>") : String("")) +
              " <span class=chan>" + (named ? String(s.model) + " &middot; " : String("")) +
              "Kanal " + String(s.channel) + " &middot; ID " + String(s.id) + "</span></div>");
      server.sendContent("<input type=hidden name='" + pfx + "_model' value='" + String(s.model) + "'>");
      server.sendContent("<input type=hidden name='" + pfx + "_channel' value='" + String(s.channel) + "'>");
      server.sendContent("<input type=hidden name='" + pfx + "_id' value='" + String(s.id) + "'>");

      server.sendContent("<p class=fieldhint style='margin:.1em 0 .3em'>aktuell: " +
              numStr(s.t, 1) + "&deg;C</p>");
      server.sendContent("<div class=row><label>Temp. min (&deg;C)<input name='" + pfx + "_tmin' type=number step=0.1 "
              "placeholder='kein Limit' value='" + limitStr(tMin, true) + "'></label>"
              "<label>Temp. max (&deg;C)<input name='" + pfx + "_tmax' type=number step=0.1 "
              "placeholder='kein Limit' value='" + limitStr(tMax, false) + "'></label></div>");
      server.sendContent("<p class=fieldhint style='margin:.1em 0 .3em'>aktuell: " +
              numStr(s.rh, 0) + "&nbsp;%</p>");
      server.sendContent("<div class=row><label>rF min (%)<input name='" + pfx + "_rhmin' type=number step=1 "
              "placeholder='kein Limit' value='" + limitStr(rhMin, true) + "'></label>"
              "<label>rF max (%)<input name='" + pfx + "_rhmax' type=number step=1 "
              "placeholder='kein Limit' value='" + limitStr(rhMax, false) + "'></label></div>");
      server.sendContent("<div class=row><label>Hysterese T (&deg;C)<input name='" + pfx + "_ht' type=number step=0.1 "
              "value='" + numStr(hT, 1) + "'></label>"
              "<label>Hysterese rF (%)<input name='" + pfx + "_hr' type=number step=1 "
              "value='" + numStr(hR, 0) + "'></label></div>");

      server.sendContent("<hr class=cardrule>"
              "<label class=row style='flex-direction:row;align-items:center;gap:.4em'>"
              "<input type=checkbox name='" + pfx + "_cmp'" + (cmpOn ? " checked" : "") +
              "> Mit Referenzsensor vergleichen</label>");
      if (refS) {
        server.sendContent("<p class=fieldhint style='margin:.1em 0 .3em'>aktuell: "
                "<span class='cmpval" + String(c && c->alarmCmpT ? " cmpwarn" : "") + "'>&Delta;T " +
                numStr(dT, 1) + "&deg;C</span> &middot; "
                "<span class='cmpval" + String(c && c->alarmCmpRH ? " cmpwarn" : "") + "'>&Delta;rF " +
                numStr(dRH, 0) + "%</span> vs " + String(refNamed ? refLbl : refS->model) + "</p>");
      }
      server.sendContent("<div class=row><label>&Delta;Temp. max (&deg;C)<input name='" + pfx + "_cdt' type=number "
              "step=0.1 placeholder='kein Limit' value='" + limitStr(cdt, false) + "'></label>"
              "<label>&Delta;rF max (%)<input name='" + pfx + "_cdr' type=number step=1 "
              "placeholder='kein Limit' value='" + limitStr(cdr, false) + "'></label></div>");
      if (refS) {
        server.sendContent("<p class=fieldhint style='margin:.1em 0 .3em'>aktuell: " + numStr(ah, 1) +
                "&nbsp;g/m&sup3; abs. &middot; <span class='cmpval" +
                String(c && c->alarmCmpAH ? " cmpwarn" : "") + "'>&Delta;AH " + numStr(dAH, 1) +
                "&nbsp;g/m&sup3;</span> vs " + String(refNamed ? refLbl : refS->model) + "</p>");
      } else {
        server.sendContent("<p class=fieldhint style='margin:.1em 0 .3em'>aktuell: " + numStr(ah, 1) +
                "&nbsp;g/m&sup3; abs.</p>");
      }
      server.sendContent("<div class=row><label>&Delta;abs.Feuchte max (g/m&sup3;)<input name='" + pfx + "_cda' "
              "type=number step=0.1 placeholder='kein Limit' value='" + limitStr(cda, false) + "'></label>"
              "<label>Hysterese &Delta;AH (g/m&sup3;)<input name='" + pfx + "_cha' type=number step=0.1 "
              "value='" + numStr(cha, 1) + "'></label></div>");
      server.sendContent("<label class=row style='flex-direction:row;align-items:center;gap:.4em;margin-top:.5em'>"
              "<input type=radio name=refidx value='" + String(i) + "'" + (isRef ? " checked" : "") +
              "> Dieser Sensor ist der Referenzsensor</label>");
      server.sendContent("</div>");
    }
    server.sendContent("</div>");
  }

  // Ntfy und ThingSpeak jeweils als eigener "Unterpunkt" (<details>, nativ
    // auf-/zuklappbar ohne JavaScript - gleiches Muster wie der bestehende
    // "Wie funktioniert das?"-Block auf der Startseite) mit den zugehoerigen
    // globalen Einstellungen UND der je-Sensor-Auswahl zusammen an einem Ort,
    // statt ueber mehrere Karten verteilt. Beide liegen bewusst noch INNERHALB
    // des grossen Formulars (nicht in eigenen <form>-Tags), weil die
    // je-Sensor-Auswahl auf die versteckten f{i}_model/_channel/_id-Felder des
    // Hauptformulars angewiesen ist.
    server.sendContent("<details class=card open style='max-width:46em;margin:1.2em auto 0'>"
            "<summary>&#128276; Ntfy</summary>"
            "<p class=fieldhint>Push-Meldungen (Grenzwerte, Signalverlust, Batterie) laufen "
            "&uuml;ber dieses Topic. In der ntfy-App (oder unter ntfy.sh) unter genau diesem "
            "Namen abonnieren &ndash; Topic antippen/markieren zum Kopieren:</p>"
            "<div class=topicbox>");
    server.sendContent(NTFY_TOPIC);
    server.sendContent(
            "</div>"
            "<p class=fieldhint style='margin:.6em 0 1em'>Web-Ansicht ohne App: "
            "<a href='https://ntfy.sh/");
    server.sendContent(NTFY_TOPIC);
    server.sendContent("' target=_blank>ntfy.sh/");
    server.sendContent(NTFY_TOPIC);
    server.sendContent(
            "</a> &middot; <a class=navlink href='/testpush'>&#128276; Testmeldung senden</a></p>"
            "<p class=fieldhint style='margin:0 0 .3em'>Je Sensor, welche Meldungen ankommen "
            "sollen:</p>");
    // Jede Zeile einzeln senden statt zu einem grossen String zusammenzuhaengen
    // (gleiche Regel wie beim ThingSpeak-Feldbereich unten - siehe dessen
    // Kommentar weiter unten fuer den Hintergrund dieser Regel).
    for (int k = 0; k < ntfyRowCount; k++) server.sendContent(ntfyRow[k]);
    server.sendContent("</details>");

#if USE_THINGSPEAK
    server.sendContent("<details class=card open style='max-width:46em;margin:1.2em auto 0'>"
            "<summary>&#128202; ThingSpeak</summary>"
            "<p class=fieldhint>Dauerhafte Aufzeichnung/Verlauf ueber die Board-Laufzeit "
            "hinaus (kostenloser Kanal auf <a href='https://thingspeak.com' target=_blank>"
            "thingspeak.com</a> noetig, dort unter \"API Keys\" den <em>Write-API-Key</em> "
            "kopieren).</p>"
            "<div class=row><label>Write-API-Key<input name=tsapikey type=text maxlength=40 "
            "placeholder='aus secrets.h uebernehmen' value='" + String(tsApiKeyOverride) + "'></label></div>"
            "<label class=ntfytoggle style='margin-top:.4em'><input type=checkbox name=tson" +
            String(tsRuntimeOn ? " checked" : "") + "> &#128202; ThingSpeak-Uebertragung aktiv</label>"
            "<p class=fieldhint style='margin:1em 0 .3em'>Jedem der 8 Kanal-Felder EINEN "
            "Sensor-Wert zuordnen (oder unbelegt lassen). &Delta;-Werte (Abweichung zum "
            "Referenzsensor) werden erst gesendet, sobald oben ein Referenzsensor gesetzt "
            "ist und live empfangen wird.</p>");
    // Jedes Feld einzeln per sendContent() raus, NICHT erst zu einem grossen
    // String zusammenbauen: bei vielen Sensoren (je bis zu 6 Optionen) wird
    // die Optionsliste pro Auswahlfeld gross, und 8 Felder in einem
    // zusammenhaengenden String wuerden denselben Heap-Fragmentierungs-Fehler
    // reproduzieren, der schon /settings und / insgesamt betraf (siehe
    // Kommentar am Anfang von handleSettingsGet()) - dort wurden bereits alle
    // anderen Abschnitte deshalb haeppchenweise gesendet, dieser Bereich muss
    // dem gleichen Muster folgen.
    for (int f = 1; f <= 8; f++) {
      server.sendContent("<div class=tsassign><label>Feld " + String(f) + "</label>" +
              tsFieldAssignSelect(f) + "</div>");
    }
    server.sendContent("</details>");
#endif
  server.sendContent("<button type=submit>Speichern</button>");
  server.sendContent("</form>");
  server.sendContent(
    "<p class=hint>Leeres Feld = kein Limit. Hysterese verhindert Melde-Flattern nahe der Grenze "
    "(Meldung bei &Uuml;berschreiten, Entwarnung erst wieder deutlich im Normalbereich). Der "
    "Vergleich pr&uuml;ft die Differenz zum Referenzsensor unabh&auml;ngig von den Grenzwerten "
    "oben &ndash; z.B. um bei zu gro&szlig;em Feuchte-Unterschied zwischen Keller und Au&szlig;en "
    "eine L&uuml;ftungsempfehlung per ntfy zu bekommen.</p>");
  server.sendContent("<p><a class=navlink href='/testpush'>&#128276;&nbsp;Testmeldung an ntfy senden</a></p>");
  server.sendContent("</body></html>");
  server.sendContent("");   // beendet den chunked-Transfer
}

static void handleSettingsPost() {
  if (server.hasArg("rssi")) {
    int v = server.arg("rssi").toInt();
    if (v < RSSI_SENS_MIN) v = RSSI_SENS_MIN;
    if (v > RSSI_SENS_MAX) v = RSSI_SENS_MAX;
    rssiSensitivity = v;
    saveRadioSettings();
#if USE_RADIO
    rf.setRSSIThreshold(rssiSensitivity);
#endif
    server.sendHeader("Location", "/settings?saved=1");
    server.send(303, "text/plain", "");
    return;
  }

#if USE_THINGSPEAK
  // Write-API-Key + Ein/Aus-Schalter sind jetzt Teil des grossen Formulars
  // (Unterpunkt "ThingSpeak" auf /settings, kein eigenes <form> mehr) - daher
  // hier unbedingt mitsichern statt in einem eigenen Zweig mit fruehem
  // return, der Referenzsensor und Schwellenwerte desselben Submits
  // uebersprungen haette.
  {
    String key = server.arg("tsapikey");
    key.trim();
    strncpy(tsApiKeyOverride, key.c_str(), sizeof(tsApiKeyOverride) - 1);
    tsApiKeyOverride[sizeof(tsApiKeyOverride) - 1] = 0;
    tsRuntimeOn = server.hasArg("tson");
    saveThingSpeakSettings();
  }
#endif

  // Referenzsensor: "-1" bzw. fehlend = keiner. Modell/Kanal/ID kommen aus den
  // gleichen f{i}_model/_channel/_id-Feldern wie die Schwellenwerte unten, per
  // Index aus dem Radio-Button "refidx" ausgewaehlt.
  int refIdx = server.hasArg("refidx") ? server.arg("refidx").toInt() : -1;
  if (refIdx < 0) {
    refSensor.used = false;
  } else {
    String rm = server.arg("f" + String(refIdx) + "_model");
    if (rm.length()) {
      int rc = server.arg("f" + String(refIdx) + "_channel").toInt();
      long rid = server.arg("f" + String(refIdx) + "_id").toInt();
      strncpy(refSensor.model, rm.c_str(), sizeof(refSensor.model) - 1);
      refSensor.model[sizeof(refSensor.model) - 1] = 0;
      refSensor.channel = rc;
      refSensor.id = rid;
      refSensor.used = true;
    }
  }
  saveRefSensor();

#if USE_THINGSPEAK
  // Feld-zentrierte ThingSpeak-Zuordnung (Formularfelder tsf1..tsf8, siehe
  // tsFieldAssignSelect() in handleSettingsGet()): jedes Feld enthaelt "none"
  // oder "<Sensorindex>_<Typ>" (Sensorindex = derselbe Index i wie unten im
  // Sensor-Durchlauf, Typ = T/RH/AH/DT/DRH/DAH). Erst hier in Want-Arrays je
  // Sensorindex aufloesen, dann unten beim jeweiligen Sensor auf die Threshold
  // uebertragen - dadurch bleibt "hoechstens ein Feld pro Wert" automatisch
  // erhalten, weil jedes Feld nur einmal in der Zuordnung vorkommen kann.
  bool wantTsEnT[MAX_THRESH]   = {false}; int wantTsFieldT[MAX_THRESH]   = {0};
  bool wantTsEnRH[MAX_THRESH]  = {false}; int wantTsFieldRH[MAX_THRESH]  = {0};
  bool wantTsEnAH[MAX_THRESH]  = {false}; int wantTsFieldAH[MAX_THRESH]  = {0};
  bool wantTsEnDT[MAX_THRESH]  = {false}; int wantTsFieldDT[MAX_THRESH]  = {0};
  bool wantTsEnDRH[MAX_THRESH] = {false}; int wantTsFieldDRH[MAX_THRESH] = {0};
  bool wantTsEnDAH[MAX_THRESH] = {false}; int wantTsFieldDAH[MAX_THRESH] = {0};
  for (int f = 1; f <= 8; f++) {
    String v = server.arg("tsf" + String(f));
    int us = v.indexOf('_');
    if (us <= 0) continue;   // "none" oder leer -> unbelegt
    int idx = v.substring(0, us).toInt();
    String type = v.substring(us + 1);
    if (idx < 0 || idx >= MAX_THRESH) continue;
    if (type == "T")        { wantTsEnT[idx]   = true; wantTsFieldT[idx]   = f; }
    else if (type == "RH")  { wantTsEnRH[idx]  = true; wantTsFieldRH[idx]  = f; }
    else if (type == "AH")  { wantTsEnAH[idx]  = true; wantTsFieldAH[idx]  = f; }
    else if (type == "DT")  { wantTsEnDT[idx]  = true; wantTsFieldDT[idx]  = f; }
    else if (type == "DRH") { wantTsEnDRH[idx] = true; wantTsFieldDRH[idx] = f; }
    else if (type == "DAH") { wantTsEnDAH[idx] = true; wantTsFieldDAH[idx] = f; }
  }
#endif

  for (int i = 0; i < MAX_THRESH; i++) {
    String mArg = "f" + String(i) + "_model";
    if (!server.hasArg(mArg)) break;
    String model = server.arg(mArg);
    if (model.length() == 0) continue;
    int  channel = server.arg("f" + String(i) + "_channel").toInt();
    long id      = server.arg("f" + String(i) + "_id").toInt();
    Threshold *c = getOrCreateThreshold(model.c_str(), channel, id);
    if (!c) continue;   // alle Slots belegt

    String tminS  = server.arg("f" + String(i) + "_tmin");
    String tmaxS  = server.arg("f" + String(i) + "_tmax");
    String rhminS = server.arg("f" + String(i) + "_rhmin");
    String rhmaxS = server.arg("f" + String(i) + "_rhmax");
    String htS    = server.arg("f" + String(i) + "_ht");
    String hrS    = server.arg("f" + String(i) + "_hr");
    String cdtS   = server.arg("f" + String(i) + "_cdt");
    String cdrS   = server.arg("f" + String(i) + "_cdr");
    String cdaS   = server.arg("f" + String(i) + "_cda");
    String chaS   = server.arg("f" + String(i) + "_cha");

    c->tMin   = tminS.length()  ? tminS.toFloat()  : -999;
    c->tMax   = tmaxS.length()  ? tmaxS.toFloat()  :  999;
    c->rhMin  = rhminS.length() ? rhminS.toFloat() : -999;
    c->rhMax  = rhmaxS.length() ? rhmaxS.toFloat() :  999;
    c->hystT  = htS.length()    ? htS.toFloat()    : 0.5f;
    c->hystRH = hrS.length()    ? hrS.toFloat()    : 3.0f;
    c->notifyT  = server.hasArg("f" + String(i) + "_nt");
    c->notifyRH = server.hasArg("f" + String(i) + "_nr");
    c->cmpEnabled = server.hasArg("f" + String(i) + "_cmp");
    c->cmpDT  = cdtS.length()   ? cdtS.toFloat()   :  999;
    c->cmpDRH = cdrS.length()   ? cdrS.toFloat()   :  999;
    c->cmpDAH = cdaS.length()   ? cdaS.toFloat()   :  999;
    c->hystAH = chaS.length()   ? chaS.toFloat()   : 0.5f;
    c->notifyCmpT  = server.hasArg("f" + String(i) + "_nct");
    c->notifyCmpRH = server.hasArg("f" + String(i) + "_ncr");
    c->notifyCmpAH = server.hasArg("f" + String(i) + "_nca");
#if USE_THINGSPEAK
    c->tsEnT = wantTsEnT[i];
    if (wantTsEnT[i]) c->tsFieldT = wantTsFieldT[i];
    c->tsEnRH = wantTsEnRH[i];
    if (wantTsEnRH[i]) c->tsFieldRH = wantTsFieldRH[i];
    c->tsEnAH = wantTsEnAH[i];
    if (wantTsEnAH[i]) c->tsFieldAH = wantTsFieldAH[i];
    c->tsEnDT = wantTsEnDT[i];
    if (wantTsEnDT[i]) c->tsFieldDT = wantTsFieldDT[i];
    c->tsEnDRH = wantTsEnDRH[i];
    if (wantTsEnDRH[i]) c->tsFieldDRH = wantTsFieldDRH[i];
    c->tsEnDAH = wantTsEnDAH[i];
    if (wantTsEnDAH[i]) c->tsFieldDAH = wantTsFieldDAH[i];
#endif
    c->used   = true;
    // Neue Grenzen -> Alarmzustand zuruecksetzen, sonst bleibt z.B. "alarmTHigh"
    // haengen, obwohl die neue Grenze laengst wieder passt. Das gilt genauso fuer
    // die Vergleichs-Alarme (alarmCmpT/RH/AH): checkComparisons() wertet einen
    // Grenzwert nur aus, wenn er nicht auf "kein Limit" (Sentinel >=900) steht -
    // wird ein Grenzwert also von "aktiv" auf "kein Limit" zurueckgesetzt, wuerde
    // die Entwarnung sonst NIE mehr laufen und der Alarm (samt z.B. der
    // Lueftungsempfehlung auf der Startseite) bliebe dauerhaft haengen. Deshalb
    // hier immer zuruecksetzen, nicht nur bei geaendertem Referenzsensor.
    c->alarmTHigh = c->alarmTLow = c->alarmRHHigh = c->alarmRHLow =
      c->alarmSignalLost = c->alarmBatteryLow = false;
    c->alarmCmpT = c->alarmCmpRH = c->alarmCmpAH = false;
  }
  ventilationRecommended = false;   // wird beim naechsten Empfang neu ermittelt
  saveThresholds();
  server.sendHeader("Location", "/settings?saved=1");
  server.send(303, "text/plain", "");
}

static void handleTestPush() {
  notify("Testmeldung vom 868-Empfaenger. Wenn du das liest, funktioniert ntfy.", true);
  server.send(200, "text/plain; charset=utf-8", "Testmeldung gesendet - bitte ntfy-App pruefen.");
}

#if USE_OLED
// Aktueller OLED-Inhalt als 1-Bit-BMP (128x64) - zum Ansehen des Displays im Browser
// (http://<host>/oled.bmp), z.B. zur Fernkontrolle. Liest den u8g2-Pufferinhalt
// (Seitenformat: je 128 Byte pro 8-Zeilen-Seite, LSB = oberste Zeile der Seite).
static void handleOledBmp() {
  if (!g_oledOk) { server.send(404, "text/plain", "kein OLED"); return; }
  static uint8_t bmp[62 + 64 * 16];
  memset(bmp, 0, sizeof(bmp));
  const uint32_t size = sizeof(bmp);
  bmp[0] = 'B'; bmp[1] = 'M';
  bmp[2] = size & 0xff; bmp[3] = (size >> 8) & 0xff;
  bmp[10] = 62;                                    // Offset der Pixeldaten
  bmp[14] = 40;                                    // Groesse des DIB-Headers
  bmp[18] = 128;                                   // Breite
  bmp[22] = 64;                                    // Hoehe (positiv = Zeilen von unten nach oben)
  bmp[26] = 1;                                     // Farbebenen
  bmp[28] = 1;                                     // 1 Bit pro Pixel
  bmp[46] = 2;                                     // Farbpalette: 2 Eintraege
  bmp[58] = bmp[59] = bmp[60] = 0xff;              // Palette[1] = weiss (Pixel an)
  const uint8_t *buf = u8g2.getBufferPtr();
  for (int y = 0; y < 64; y++) {
    uint8_t *row = bmp + 62 + (63 - y) * 16;
    for (int x = 0; x < 128; x++)
      if ((buf[(y >> 3) * 128 + x] >> (y & 7)) & 1) row[x >> 3] |= 0x80 >> (x & 7);
  }
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(sizeof(bmp));
  server.send(200, "image/bmp", "");
  server.sendContent((const char *)bmp, sizeof(bmp));
}

// Dauer-Eingabe "Sekunden.Zehntel" (auch mit Komma) in Zehntelsekunden umrechnen.
static int parseTenths(const String &in, int def) {
  String s = in;
  s.trim();
  s.replace(',', '.');
  if (s.length() == 0 || !(isdigit((unsigned char)s[0]) || s[0] == '.')) return def;   // kein Zahlenwert -> unveraendert
  return clampDur((int)lroundf(s.toFloat() * 10.0f));
}

// Untermenue "Display-Anzeige": je Anzeigeart ein Block mit Haekchen (laeuft im Wechsel mit),
// den Werten, die diese Anzeige verwendet, und ihrer Dauer (Sekunden mit Zehntelsekunden).
// Jede Aenderung wird sofort uebernommen (onchange), es gibt keinen Speichern-Knopf.
static String tenthsStr(int t) { return String(t / 10) + "." + String(t % 10); }

static String htmlEscStr(const char *in) {
  String o;
  for (; *in; in++) {
    switch (*in) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      case '\'': o += "&#39;"; break;
      default: o += *in;
    }
  }
  return o;
}

// Auswahlfeld fuer einen der beiden Vergleichssensoren: Innen, Aussen oder ein bestimmter Sensor.
static void sendSensorSelect(const char *name, const char *caption, const SensorSel &cur, const int *vis, int n) {
  sc(String("<label>") + caption + "<select name=" + name + " onchange='this.form.submit()'>");
  sc(String("<option value=i") + (cur.kind == 0 ? " selected" : "") + ">Innen (Vergleichssensor)</option>");
  sc(String("<option value=a") + (cur.kind == 1 ? " selected" : "") + ">Au&szlig;en (Referenzsensor)</option>");
  bool found = false;
  for (int i = 0; i < n; i++) {
    const Sensor &sn = sensors[vis[i]];
    bool sel = cur.kind == 2 && !strcmp(cur.model, sn.model) && cur.channel == sn.channel && cur.id == sn.id;
    found |= sel;
    char nm[21];
    bool named = getDisplayName(sn.model, sn.channel, sn.id, nm, sizeof(nm));
    String label = named ? String(nm) : String(sn.model) + " " + sn.channel + "/" + sn.id;
    sc(String("<option value=\"s|") + htmlEscStr(sn.model) + "|" + sn.channel + "|" + sn.id + "\"" +
                       (sel ? " selected" : "") + ">" + htmlEscStr(label.c_str()) + "</option>");
  }
  if (cur.kind == 2 && !found)                    // gespeicherter Sensor, gerade nicht in der Liste
    sc(String("<option value=\"s|") + htmlEscStr(cur.model) + "|" + cur.channel + "|" + cur.id +
                       "\" selected>" + htmlEscStr(cur.model) + " " + cur.channel + "/" + cur.id + " (nicht empfangen)</option>");
  sc("</select></label>");
}

static bool parseSensorSel(const String &v, SensorSel &out) {
  if (v == "i") { out = {0, {0}, 0, 0}; return true; }
  if (v == "a") { out = {1, {0}, 0, 0}; return true; }
  if (!v.startsWith("s|")) return false;
  int p1 = v.indexOf('|', 2), p2 = v.indexOf('|', p1 + 1);
  if (p1 < 0 || p2 < 0) return false;
  String m = v.substring(2, p1);
  if (m.length() == 0 || m.length() >= sizeof(out.model)) return false;
  SensorSel t = {2, {0}, (int32_t)v.substring(p1 + 1, p2).toInt(), (int32_t)v.substring(p2 + 1).toInt()};
  strncpy(t.model, m.c_str(), sizeof(t.model) - 1);
  out = t;
  return true;
}

static const char *const Q_NAME_WEB[Q_COUNT] = {"Temperatur", "rel. Feuchte", "abs. Feuchte", "Taupunkt",
                                                  "Abstand zum Taupunkt", "Empfangsst&auml;rke"};

// Auswahl einer Groesse. mode 0 alle, 1 nur Differenz-taugliche (T, rF, aF, Taupunkt),
// 2 nur mit Urteil (aF, Taupunkt), 3 alle plus "keine".
static void sendQSel(const char *name, const char *caption, int cur, int mode) {
  sc(String("<label>") + caption + "<select name=" + name + " onchange='this.form.submit()'>");
  for (int q = 0; q < Q_COUNT; q++) {
    if (mode == 1 && !qAllowedForDiff(q)) continue;
    if (mode == 2 && q != Q_AH && q != Q_TD) continue;
    sc(String("<option value=") + q + (q == cur ? " selected" : "") + ">" + Q_NAME_WEB[q] + "</option>");
  }
  if (mode == 3)
    sc(String("<option value=") + Q_COUNT + (cur == Q_COUNT ? " selected" : "") + ">&ndash; keine &ndash;</option>");
  sc("</select></label>");
}

// Auswahl aus einer kleinen Liste fester Werte.
static void sendEnumSel(const char *name, const char *caption, int cur, const char *const *labels, const int *vals, int n) {
  sc(String("<label>") + caption + "<select name=" + name + " onchange='this.form.submit()'>");
  for (int i = 0; i < n; i++)
    sc(String("<option value=") + vals[i] + (vals[i] == cur ? " selected" : "") + ">" + labels[i] + "</option>");
  sc("</select></label>");
}

static void sendBlockOpen(const char *chk, const char *id, const char *title, const char *sub, bool on) {
  sc(String("<div class=pg><input class=chk type=checkbox id=") + id + " name=" + chk + (on ? " checked" : "") +
     " onchange='this.form.submit()'><label class=pgtitle for=" + id + "><span>" + title + "<small>" + sub +
     "</small></span></label><div class=pgopts>");
}

static void sendDurInput(const char *name, int d) {
  sc(String("<label>Dauer (s)<input type=number name=") + name + " min=0.5 max=60 step=0.1 value='" + tenthsStr(d) +
     "' onchange='this.form.submit()'></label>");
}

// Die weiteren Anzeigen (Ampel, Min/Max, Schimmelrisiko, Empfang, System, Uhr) und die Trend-Einstellungen.
static void sendMoreDisplayBlocks(const int *vis, int n) {
  static const char *const ONOFF[2] = {"aus", "an"};
  static const int ONOFF_V[2] = {0, 1};

  sendBlockOpen("a", "pa", "L&uuml;ften-Ampel", "gro&szlig;es Urteil (L&Uuml;FTEN / GRENZE / OK) mit der Differenz Innen &minus; Au&szlig;en",
                (dispMask & DM_AMPEL) != 0);
  sendQSel("qa", "Differenz von", dispQA, 2);
  static const char *const AD[2] = {"nur Differenz", "mit Innen-/Au&szlig;enwert"};
  sendEnumSel("ad", "Zusatz", ampelDetail ? 1 : 0, AD, ONOFF_V, 2);
  sendDurInput("da", dispDurAmpel);
  sc("</div></div>");

  sendBlockOpen("m", "pm", "Min / Max", "Tiefst- und H&ouml;chstwerte aus dem Verlauf (bis 24 Stunden)", (dispMask & DM_MINMAX) != 0);
  sendQSel("qm", "Wert", dispQM, 1);
  static const char *const MW[3] = {"Innen und Au&szlig;en", "nur Innen", "nur Au&szlig;en"};
  static const int MW_V[3] = {0, 1, 2};
  sendEnumSel("mw", "Seite", mmWho, MW, MW_V, 3);
  sendDurInput("dm", dispDurMm);
  sc("</div></div>");

  sendBlockOpen("h", "ph", "Schimmelrisiko", "zwei Werte eines Sensors mit Urteil aus rel. Feuchte und Taupunktabstand",
                (dispMask & DM_MOLD) != 0);
  sendSensorSelect("hs", "Sensor", moldSel, vis, n);
  sendQSel("h1", "Wert links", dispQH1, 0);
  sendQSel("h2", "Wert rechts", dispQH2, 0);
  sendDurInput("dh", dispDurMold);
  sc("</div></div>");

  sendBlockOpen("r", "pr", "Empfangs-&Uuml;bersicht", "alle Sensoren mit Balken f&uuml;r die Empfangsst&auml;rke", (dispMask & DM_RX) != 0);
  static const char *const RC[3] = {"Alter des Empfangs", "Empfangsst&auml;rke (dBm)", "Anzahl Empf&auml;nge"};
  static const int RC_V[3] = {0, 1, 2};
  sendEnumSel("rc", "Rechte Spalte", rxCol, RC, RC_V, 3);
  sendDurInput("dr", dispDurRx);
  sc("</div></div>");

  sendBlockOpen("y", "py", "Systemseite", "Zeilen mit WLAN, IP, Laufzeit, Speicher &hellip;", (dispMask & DM_SYS) != 0);
  static const char *const SYS_L[6] = {"WLAN-Name", "IP-Adresse", "WLAN-Signal", "Laufzeit", "Speicher (Heap)", "Telegramme"};
  for (int i = 0; i < 6; i++)
    sc(String("<label style='flex-direction:row;align-items:center;gap:.5em'><input type=checkbox name=s") + i +
       ((sysMask & (1 << i)) ? " checked" : "") + " onchange='this.form.submit()'>" + SYS_L[i] + "</label>");
  sendDurInput("dy", dispDurSys);
  sc("</div></div>");

  sendBlockOpen("u", "pu", "Uhr", "Uhrzeit und Datum (per Internet) mit Werten von Innen und Au&szlig;en", (dispMask & DM_CLOCK) != 0);
  sendQSel("qc", "Werte darunter", dispQC, 3);
  sendDurInput("du", dispDurClock);
  sc("</div></div>");

  sc("<div class=pg><span></span><div class=pgtitle><span>Trend-Pfeile<small>Pfeil und &Auml;nderung auf den Sensorkarten, "
     "im Tacho und in der Ampel</small></span></div><div class=pgopts>");
  sendEnumSel("tc", "Auf Sensorkarten", trendCards, ONOFF, ONOFF_V, 2);
  sendEnumSel("tg", "Im Tacho / Ampel", trendGauge, ONOFF, ONOFF_V, 2);
  static const char *const TW[4] = {"15 Minuten", "30 Minuten", "1 Stunde", "2 Stunden"};
  static const int TW_V[4] = {15, 30, 60, 120};
  sendEnumSel("tw", "Vergleich mit vor", trendMin, TW, TW_V, 4);
  sc("</div></div>");
}

static void handleDisplayGet() {
  scBegin();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  sc(
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>868-Empfaenger &middot; Display-Anzeige</title><style>"
    ":root{color-scheme:light dark}*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;"
    "padding:1.4em 1em 2.4em;background:#0f172a;color:#e6ebf5}"
    "h1{max-width:46em;margin:0 auto .3em;font-size:1.3em}"
    ".navlink{color:#7dd3fc;text-decoration:none;font-size:.88em}"
    ".back{max-width:46em;margin:0 auto 1em}"
    "form{max-width:46em;margin:0 auto}"
    ".card{background:linear-gradient(160deg,#1b2740,#141d31);border:1px solid #263352;"
    "border-radius:14px;padding:1em 1.1em;box-shadow:0 4px 14px rgba(0,0,0,.25);margin-bottom:1em}"
    ".card-head{font-weight:600;font-size:.98em;margin-bottom:.3em}"
    ".hint{font-size:.85em;color:#93a0b8;margin:.2em 0 .8em}"
    ".flow{background:#0f1a30;border:1px solid #263352;border-radius:10px;padding:.55em .8em;"
    "font-size:.88em;margin:.2em 0 .4em}"
    ".top{display:flex;flex-wrap:wrap;gap:1em;align-items:flex-start}"
    ".top>div:first-child{flex:1 1 15em}"
    ".oledprev{display:block;width:256px;max-width:100%;height:auto;image-rendering:pixelated;"
    "background:#000;border:1px solid #263352;border-radius:6px}"
    ".pg{display:grid;grid-template-columns:auto 1fr;column-gap:.7em;row-gap:.6em;"
    "border-top:1px solid #263352;padding:.8em 0 .5em}"
    ".pg:first-of-type{border-top:0}"
    ".pgtitle{cursor:pointer;font-weight:600}"
    ".pgtitle small{display:block;color:#8291ad;font-weight:400;font-size:.82em}"
    ".chk{width:1.2em;height:1.2em;margin:.15em 0 0}"
    ".pgopts{grid-column:2;display:grid;grid-template-columns:repeat(auto-fit,minmax(9.5em,1fr));"
    "gap:.5em .8em;transition:opacity .15s}"
    ".chk:not(:checked)~.pgopts{opacity:.45}"
    ".pgopts label{display:flex;flex-direction:column;gap:.15em;font-size:.8em;color:#93a0b8}"
    ".pgopts select,.pgopts input[type=number]{width:100%;background:#0f1a30;border:1px solid #263352;"
    "border-radius:8px;color:#e6ebf5;padding:.4em .45em;font-size:1.1em}"
    "@media(prefers-color-scheme:light){body{background:#f3f5fb;color:#1c2333}"
    ".navlink{color:#0369a1}.card{background:#fff;border-color:#e1e6f0;"
    "box-shadow:0 2px 10px rgba(30,41,70,.08)}.hint,.pgopts label{color:#5b6785}"
    ".flow{background:#eef2fa;border-color:#dde3f0}.pg{border-color:#e1e6f0}.pgtitle small{color:#6b7690}"
    ".pgopts select,.pgopts input[type=number]{background:#eef2fa;border-color:#dde3f0;color:#1c2333}"
    ".oledprev{border-color:#e1e6f0}}"
    "</style></head><body>"
    "<h1>&#128250; Display-Anzeige</h1>"
    "<p class=back><a class=navlink href='/'>&larr; zur&uuml;ck zur &Uuml;bersicht</a></p>"
    "<form method=POST action='/display'><input type=hidden name=f value=1>");

  // Kopf: Zusammenfassung des Ablaufs + Live-Vorschau
  int vis[MAX_SENSORS];
  int n = visibleSensors(vis, MAX_SENSORS);
  String flow;
  int totalT = 0, parts = 0;
  if (dispMask & DM_CARDS) {
    flow += "Sensorkarten (" + String(n) + " &times; " + tenthsStr(dispDurCards) + "&nbsp;s)";
    totalT += n * dispDurCards; parts++;
  }
  if (dispMask & DM_GAUGE) {
    flow += String(parts ? " &rarr; " : "") + "Tacho (" + tenthsStr(dispDurGauge) + "&nbsp;s)";
    totalT += dispDurGauge; parts++;
  }
  if (dispMask & DM_COMPARE) {
    flow += String(parts ? " &rarr; " : "") + "Vergleich (" + tenthsStr(dispDurCmp) + "&nbsp;s)";
    totalT += dispDurCmp; parts++;
  }
  {
    struct FlowItem { int bit; const char *name; int dur; };
    const FlowItem more[6] = {{DM_AMPEL, "Ampel", dispDurAmpel}, {DM_MINMAX, "Min/Max", dispDurMm},
                              {DM_MOLD, "Schimmel", dispDurMold}, {DM_RX, "Empfang", dispDurRx},
                              {DM_SYS, "System", dispDurSys}, {DM_CLOCK, "Uhr", dispDurClock}};
    for (int i = 0; i < 6; i++)
      if (dispMask & more[i].bit) {
        flow += String(parts ? " &rarr; " : "") + more[i].name + " (" + tenthsStr(more[i].dur) + "&nbsp;s)";
        totalT += more[i].dur; parts++;
      }
  }
  if (dispMask & DM_GRAPH) {
    flow += String(parts ? " &rarr; " : "") + "Verlauf (" + tenthsStr(dispDurGraph) + "&nbsp;s" +
            (graphCount >= 2 ? "" : ", folgt sobald Messpunkte vorliegen") + ")";
    if (graphCount >= 2) totalT += dispDurGraph;
    parts++;
  }
  sc("<div class=card><div class=top><div><div class=card-head>Ablauf</div>"
          "<p class=hint>Alle angehakten Anzeigen wechseln sich der Reihe nach ab, ist nur eine angehakt, "
          "bleibt sie stehen. &Auml;nderungen gelten sofort.</p><div class=flow>" + flow +
          (parts > 1 ? "<br>ein Durchlauf: " + tenthsStr(totalT) + "&nbsp;s" : String("")) +
          "</div></div><div><div class=card-head>So sieht das Display gerade aus</div>"
          "<img id=oledprev class=oledprev src='/oled.bmp' alt='Display-Vorschau' width=256 height=128>"
          "</div></div></div>");

  static const char *qName[Q_COUNT] = {"Temperatur", "rel. Feuchte", "abs. Feuchte", "Taupunkt",
                                       "Abstand zum Taupunkt", "Empfangsst&auml;rke"};
  struct Sel { const char *name; const char *caption; int cur; bool diffOnly; bool allowNone; };
  struct Pg  { const char *chk; const char *id; const char *title; const char *sub; bool on;
               const Sel *sels; int nSel; const char *dur; const char *durCap; int d; };
  static const Sel selCards[3] = {{"ql", "Wert 1", 0, false, false}, {"qr", "Wert 2", 0, false, false},
                                  {"qx", "Wert 3", 0, false, true}};
  static const Sel selGauge[1] = {{"qg", "Differenz von", 0, true, false}};
  static const Sel selGraph[1] = {{"qv", "Kurven von", 0, true, false}};
  static const Sel selCmp[1]   = {{"qk", "Wert", 0, false, false}};
  const Pg pgs[4] = {
    {"c", "pc", "Sensorkarten", "je Sensor eine Seite mit bis zu drei Werten nebeneinander",
     (dispMask & DM_CARDS) != 0, selCards, 3, "dc", "Dauer je Karte (s)", dispDurCards},
    {"g", "pg", "Tacho", "momentane Differenz Innen &minus; Au&szlig;en als Halbkreis mit Zeiger",
     (dispMask & DM_GAUGE) != 0, selGauge, 1, "dg", "Dauer (s)", dispDurGauge},
    {"v", "pv", "Verlauf", "Kurven Innen / Au&szlig;en / Differenz &uuml;ber bis zu 24 Stunden",
     (dispMask & DM_GRAPH) != 0, selGraph, 1, "dv", "Dauer (s)", dispDurGraph},
    {"k", "pk", "Sensor-Vergleich", "derselbe Wert von zwei Sensoren nebeneinander, auf Wunsch mit Differenz",
     (dispMask & DM_COMPARE) != 0, selCmp, 1, "dk", "Dauer (s)", dispDurCmp}};
  const int cur[4][3] = {{dispQL, dispQR, dispQX}, {dispQG, 0, 0}, {dispQV, 0, 0}, {dispQK, 0, 0}};

  sc("<div class=card><div class=card-head>Anzeigen und Werte</div>"
          "<p class=hint>Je Anzeige: Haken setzen, Werte w&auml;hlen und die Dauer in Sekunden mit "
          "Zehntelsekunden eintragen (z.&nbsp;B. 2,5). Taupunkt und absolute Feuchte werden aus Temperatur und "
          "relativer Feuchte berechnet; mindestens eine Anzeige bleibt immer aktiv.</p>");
  for (int k = 0; k < 4; k++) {
    sc(String("<div class=pg><input class=chk type=checkbox id=") + pgs[k].id + " name=" + pgs[k].chk +
            (pgs[k].on ? " checked" : "") + " onchange='this.form.submit()'>"
            "<label class=pgtitle for=" + pgs[k].id + "><span>" + pgs[k].title + "<small>" + pgs[k].sub +
            "</small></span></label><div class=pgopts>");
    for (int j = 0; j < pgs[k].nSel; j++) {
      const Sel &sl = pgs[k].sels[j];
      sc(String("<label>") + sl.caption + "<select name=" + sl.name + " onchange='this.form.submit()'>");
      for (int q = 0; q < Q_COUNT; q++) {
        if (sl.diffOnly && !qAllowedForDiff(q)) continue;
        sc(String("<option value=") + q + (q == cur[k][j] ? " selected" : "") + ">" + qName[q] + "</option>");
      }
      if (sl.allowNone)
        sc(String("<option value=") + Q_COUNT + (cur[k][j] == Q_COUNT ? " selected" : "") +
                ">&ndash; nicht anzeigen &ndash;</option>");
      sc("</select></label>");
    }
    if (k == 3) {
      sendSensorSelect("ka", "Sensor links", cmpA, vis, n);
      sendSensorSelect("kb", "Sensor rechts", cmpB, vis, n);
      sc(String("<label>Differenz<select name=kd onchange='this.form.submit()'><option value=1") +
              (cmpDiff ? " selected" : "") + ">mit Differenz (links &minus; rechts)</option><option value=0" +
              (cmpDiff ? "" : " selected") + ">ohne Differenz</option></select></label>");
    }
    sc(String("<label>") + pgs[k].durCap + "<input type=number name=" + pgs[k].dur +
            " min=0.5 max=60 step=0.1 value='" + tenthsStr(pgs[k].d) + "' onchange='this.form.submit()'></label>"
            "</div></div>");
  }
  sendMoreDisplayBlocks(vis, n);
  sc("</div></form>"
          "<script>var p=document.getElementById('oledprev');function n(){setTimeout(function(){"
          "if(!document.hidden)p.src='/oled.bmp?'+Date.now();else n()},4000)}p.onload=p.onerror=n;n()</script></body></html>");
  scEnd();
}

// Speichern vom Formular auf /display (f=1: Haekchen, Dauern, Wertauswahl). Nicht
// angehakte Anzeigen werden vom Browser gar nicht erst mitgeschickt - deshalb steht das
// versteckte Feld f dabei, damit "fehlt" als "abgewaehlt" verstanden wird.
/* ------------------------------------------------------------------ *
 *  Wassersensoren: Banner auf der Startseite und Seite /wasser        *
 * ------------------------------------------------------------------ */
static String leakBannerHtml() {
  String b = "<div style='background:#c0392b;color:#fff;padding:1em;border-radius:10px;margin:.8em 0;font-size:1.15em'>"
             "<b>&#128167; WASSER ERKANNT</b>";
  for (int i = 0; i < LEAK_MAX; i++) {
    if (!(leaks[i].used && leaks[i].alarm)) continue;
    char lab[32];
    leakLabel(leaks[i], lab, sizeof(lab));
    b += "<br>" + htmlEscStr(lab) + " &ndash; seit " + String((unsigned long)((millis() - leaks[i].alarmSince) / 60000UL)) + " min";
  }
  b += " &middot; <a style='color:#fff' href='/wasser'>Details</a></div>";
  return b;
}

static void handleWasserGet() {
  String o;
  o.reserve(3200);
  o += "<!doctype html><html><head><meta charset=utf-8>"
       "<meta name=viewport content='width=device-width,initial-scale=1'>"
       "<meta http-equiv=refresh content=20>"
       "<title>868-Empfaenger &middot; Wassersensoren</title><style>"
       "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;max-width:46em;margin:auto;padding:1em}"
       ".card{border:1px solid #8886;border-left:6px solid #2e9d4f;border-radius:10px;padding:.8em 1em;margin:.8em 0}"
       ".alarm{background:#c0392b;color:#fff;border-color:#c0392b}"
       "input[type=text]{width:100%;padding:.4em;font-size:1em;margin-top:.3em}"
       "button{padding:.5em 1em;font-size:1em;margin-top:.6em}.s{opacity:.7;font-size:.9em}"
       "</style></head><body><h1>&#128167; Wassersensoren</h1>"
       "<p><a href='/'>&larr; zur &Uuml;bersicht</a></p>"
       "<form method=POST action='/wasser'>";
  int shown = 0;
  for (int i = 0; i < LEAK_MAX; i++) {
    const LeakSensor &l = leaks[i];
    if (!l.used) continue;
    shown++;
    char lab[32];
    leakLabel(l, lab, sizeof(lab));
    String state = !l.seen ? "noch nicht empfangen"
                 : l.alarm ? "WASSER ERKANNT"
                 : l.lost  ? "meldet sich nicht mehr"
                           : "trocken";
    o += String("<div class='card") + (l.alarm ? " alarm" : "") + "'><b>" + htmlEscStr(lab) + "</b> &middot; " + state +
         "<br><span class=s>" + htmlEscStr(l.model) + " &middot; Kanal " + String((int)l.channel) +
         " &middot; ID " + String((long)l.id);
    if (l.seen) {
      o += " &middot; vor " + String((unsigned long)((millis() - l.stamp) / 1000UL)) + " s";
      if (l.battPct >= 0) o += " &middot; Batterie " + String(l.battPct) + " %" + (l.battLow ? " (schwach)" : "");
      if (!isnan(l.rssi)) o += " &middot; " + String((int)l.rssi) + " dBm";
      if (l.raw >= 0) o += " &middot; Rohwert " + String(l.raw);
    }
    o += "</span><br><label>Name <input type=text name=n" + String(i) + " maxlength=20 value='" +
         htmlEscStr(l.name) + "' placeholder='z.B. Waschmaschine'></label>"
         "<br><label><input type=checkbox name=d" + String(i) + "> Eintrag entfernen</label></div>";
  }
  if (!shown) o += "<p>Noch kein Wassersensor empfangen. Batterie einsetzen &ndash; der Sensor meldet sich nach etwa einer Minute.</p>";
  else o += "<button type=submit>Speichern</button>";
  o += "</form><form method=POST action='/wasser'><input type=hidden name=test value=1>"
       "<button type=submit>ntfy-Test senden</button></form></body></html>";
  server.send(200, "text/html; charset=utf-8", o);
}

static void handleWasserPost() {
  if (server.hasArg("test")) {
    leakNotify("Testmeldung vom Wasserleck-Melder: wenn du das liest, funktioniert ntfy.");
  } else {
    bool changed = false;
    for (int i = 0; i < LEAK_MAX; i++) {
      if (!leaks[i].used) continue;
      if (server.hasArg("d" + String(i))) {
        leakResetSlot(leaks[i]);
        changed = true;
        continue;
      }
      if (server.hasArg("n" + String(i))) {
        String v = server.arg("n" + String(i));
        v.trim();
        if (strcmp(leaks[i].name, v.c_str()) != 0) {
          strncpy(leaks[i].name, v.c_str(), sizeof(leaks[i].name) - 1);
          leaks[i].name[sizeof(leaks[i].name) - 1] = 0;
          changed = true;
        }
      }
    }
    if (changed) saveLeaks();
  }
  server.sendHeader("Location", "/wasser");
  server.send(303, "text/plain", "");
}

static void handleDisplayPost() {
  bool changed = false;
  if (server.hasArg("f")) {
    int m = 0;
    if (server.hasArg("c")) m |= DM_CARDS;
    if (server.hasArg("g")) m |= DM_GAUGE;
    if (server.hasArg("v")) m |= DM_GRAPH;
    if (server.hasArg("k")) m |= DM_COMPARE;
    if (server.hasArg("a")) m |= DM_AMPEL;
    if (server.hasArg("m")) m |= DM_MINMAX;
    if (server.hasArg("h")) m |= DM_MOLD;
    if (server.hasArg("r")) m |= DM_RX;
    if (server.hasArg("y")) m |= DM_SYS;
    if (server.hasArg("u")) m |= DM_CLOCK;
    int sm = 0;
    for (int i = 0; i < 6; i++) if (server.hasArg(String("s") + i)) sm |= 1 << i;
    sysMask = sm;
    if (m == 0) m = DM_CARDS;                 // mindestens eine Anzeige bleibt aktiv
    dispMask = m;
    dispDurCards = parseTenths(server.arg("dc"), dispDurCards);
    dispDurGauge = parseTenths(server.arg("dg"), dispDurGauge);
    dispDurGraph = parseTenths(server.arg("dv"), dispDurGraph);
    dispDurCmp   = parseTenths(server.arg("dk"), dispDurCmp);
    dispDurAmpel = parseTenths(server.arg("da"), dispDurAmpel);
    dispDurMm    = parseTenths(server.arg("dm"), dispDurMm);
    dispDurMold  = parseTenths(server.arg("dh"), dispDurMold);
    dispDurRx    = parseTenths(server.arg("dr"), dispDurRx);
    dispDurSys   = parseTenths(server.arg("dy"), dispDurSys);
    dispDurClock = parseTenths(server.arg("du"), dispDurClock);
    changed = true;
  }
  if (server.hasArg("ql")) { int v = server.arg("ql").toInt(); if (v >= 0 && v < Q_COUNT)  { dispQL = v; changed = true; } }
  if (server.hasArg("qr")) { int v = server.arg("qr").toInt(); if (v >= 0 && v < Q_COUNT)  { dispQR = v; changed = true; } }
  if (server.hasArg("qx")) { int v = server.arg("qx").toInt(); if (v >= 0 && v <= Q_COUNT) { dispQX = v; changed = true; } }
  if (server.hasArg("qg")) { int v = server.arg("qg").toInt(); if (qAllowedForDiff(v)) { dispQG = v; changed = true; } }
  if (server.hasArg("qv")) { int v = server.arg("qv").toInt(); if (qAllowedForDiff(v)) { dispQV = v; changed = true; } }
  if (server.hasArg("qk")) { int v = server.arg("qk").toInt(); if (v >= 0 && v < Q_COUNT) { dispQK = v; changed = true; } }
  if (server.hasArg("ka") && parseSensorSel(server.arg("ka"), cmpA)) changed = true;
  if (server.hasArg("kb") && parseSensorSel(server.arg("kb"), cmpB)) changed = true;
  if (server.hasArg("kd")) { cmpDiff = server.arg("kd") == "1"; changed = true; }
  if (server.hasArg("qa")) { int v = server.arg("qa").toInt(); if (v == Q_AH || v == Q_TD) { dispQA = v; changed = true; } }
  if (server.hasArg("ad")) { ampelDetail = server.arg("ad") == "1"; changed = true; }
  if (server.hasArg("qm")) { int v = server.arg("qm").toInt(); if (qAllowedForDiff(v)) { dispQM = v; changed = true; } }
  if (server.hasArg("mw")) { int v = server.arg("mw").toInt(); if (v >= 0 && v <= 2) { mmWho = v; changed = true; } }
  if (server.hasArg("hs") && parseSensorSel(server.arg("hs"), moldSel)) changed = true;
  if (server.hasArg("h1")) { int v = server.arg("h1").toInt(); if (v >= 0 && v < Q_COUNT) { dispQH1 = v; changed = true; } }
  if (server.hasArg("h2")) { int v = server.arg("h2").toInt(); if (v >= 0 && v < Q_COUNT) { dispQH2 = v; changed = true; } }
  if (server.hasArg("rc")) { int v = server.arg("rc").toInt(); if (v >= 0 && v <= 2) { rxCol = v; changed = true; } }
  if (server.hasArg("qc")) { int v = server.arg("qc").toInt(); if (v >= 0 && v <= Q_COUNT) { dispQC = v; changed = true; } }
  if (server.hasArg("tc")) { trendCards = server.arg("tc") == "1"; changed = true; }
  if (server.hasArg("tg")) { trendGauge = server.arg("tg") == "1"; changed = true; }
  if (server.hasArg("tw")) { int v = server.arg("tw").toInt(); if (v == 15 || v == 30 || v == 60 || v == 120) { trendMin = v; changed = true; } }
  if (changed) saveRadioSettings();
  server.sendHeader("Location", "/display");
  server.send(303, "text/plain", "");
}
#endif

// ------------------------------------------------------------------------
//  /alle: bewusst schlanke Tabelle OHNE Sparklines/grosse Zahlen - zeigt
//  jeden je gesehenen Sensor (auch Nachbarn/Fehldekodierungen) und laesst
//  per Haekchen auswaehlen, welche als "eigen" gelten und damit auf der
//  Startseite/OLED auftauchen. Bleibt auch bei allen MAX_SENSORS=32
//  Eintraegen schnell, weil pro Zeile nur ein paar Textzellen entstehen.
// ------------------------------------------------------------------------
static void handleAlleGet() {
  String rows;
  rows.reserve((size_t)sensorCount * 260 + 64);
  for (int i = 0; i < sensorCount; i++) {
    Sensor &s = sensors[i];
    String pfx = "f" + String(i);
    bool own = isOwned(s.model, s.channel, s.id);
    char dispName[21];
    bool named = getDisplayName(s.model, s.channel, s.id, dispName, sizeof(dispName));
    rows += "<tr><td>"
            "<input type=hidden name='" + pfx + "_model' value='" + String(s.model) + "'>"
            "<input type=hidden name='" + pfx + "_channel' value='" + String(s.channel) + "'>"
            "<input type=hidden name='" + pfx + "_id' value='" + String(s.id) + "'>"
            "<input type=checkbox name='" + pfx + "_own'" + (own ? " checked" : "") + "></td>"
            "<td><input type=text class=nameinput name='" + pfx + "_name' maxlength=20 "
            "placeholder='" + String(s.model) + "' value='" + (named ? String(dispName) : String("")) +
            "'></td>"
            "<td>" + String(s.model) + "</td><td>" + String(s.channel) + "</td><td>" + String(s.id) + "</td>"
            "<td>" + numStr(s.t, 1) + "&nbsp;&deg;C</td><td>" + numStr(s.rh, 0) + "&nbsp;%</td>"
            "<td>" + numStr(absoluteHumidity(s.t, s.rh), 1) + "&nbsp;g/m&sup3;</td>"
            "<td>vor " + String((unsigned long)ageSec(s)) + "s</td>"
            "<td>" + String((unsigned long)s.count) + "&times;</td>"
            "<td>" + (s.batteryOk ? String("ok") : String("<span class=warn2>schwach</span>")) + "</td></tr>";
  }
  if (sensorCount == 0)
    rows = "<tr><td colspan=11 class=empty2>noch keine Sensoren empfangen</td></tr>";

  String page;
  page.reserve(rows.length() + 1900);
  page +=
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>868-Empfaenger &middot; Alle Sensoren</title><style>"
    ":root{color-scheme:light dark}*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;"
    "padding:1.4em 1em 2.4em;background:#0f172a;color:#e6ebf5}"
    "h1{max-width:54em;margin:0 auto .3em;font-size:1.3em;display:flex;align-items:center;gap:.4em}"
    ".navlink{color:#7dd3fc;text-decoration:none;font-size:.88em}"
    ".back,.hint,.banner{max-width:54em;margin:0 auto .8em;font-size:.86em}"
    ".hint{color:#93a0b8}"
    ".banner{background:#14532d;border:1px solid #22c55e;border-radius:10px;padding:.5em .9em}"
    "form{max-width:54em;margin:0 auto}"
    ".table-wrap{overflow-x:auto;border:1px solid #263352;border-radius:10px}"
    "table{width:100%;border-collapse:collapse;font-size:.86em}"
    "th{text-align:left;font-size:.7em;letter-spacing:.04em;text-transform:uppercase;color:#8291ad;"
    "padding:.55em .7em;border-bottom:1px solid #263352;background:#141d31}"
    "td{padding:.5em .7em;border-bottom:1px solid #1c2740}"
    "tr:last-child td{border-bottom:none}"
    ".empty2{text-align:center;color:#8291ad;padding:1.4em 0}"
    ".warn2{color:#e2934f;font-weight:600}"
    "input[type=checkbox]{width:1.15em;height:1.15em}"
    ".nameinput{background:#0f1a30;border:1px solid #263352;border-radius:6px;"
    "color:#e6ebf5;padding:.3em .5em;font-size:.95em;width:9em;max-width:100%}"
    "button{margin-top:1.1em;background:#0369a1;color:#fff;border:0;border-radius:10px;"
    "padding:.6em 1.4em;font-size:1em;cursor:pointer}"
    "button:hover{background:#0284c7}"
    "@media(prefers-color-scheme:light){body{background:#f3f5fb;color:#1c2333}"
    ".navlink{color:#0369a1}.hint{color:#5b6785}"
    ".table-wrap{border-color:#e1e6f0}th{background:#eef2fa;color:#6b7690;border-color:#e1e6f0}"
    "td{border-color:#eef1f6}.empty2{color:#5b6785}.warn2{color:#b5641c}"
    ".nameinput{background:#eef2fa;border-color:#dde3f0;color:#1c2333}"
    ".banner{background:#dcfce7;border-color:#86efac;color:#14532d}}"
    "</style></head><body>"
    "<h1>&#128203; Alle gesehenen Sensoren</h1>"
    "<p class=back><a class=navlink href='/'>&larr; zur&uuml;ck zur &Uuml;bersicht</a></p>";
  if (server.hasArg("saved")) page += "<div class=banner>Gespeichert.</div>";
  page +=
    "<p class=hint>Ohne Whitelist landet hier fr&uuml;her oder sp&auml;ter auch jeder "
    "Nachbarsensor und jede Fehldekodierung. H&auml;kchen setzen bei dem/den eigenen "
    "Sensor(en) &ndash; nur die markierten erscheinen dann auf der Startseite und dem "
    "Display. Ohne jede Markierung wird &uuml;bergangsweise weiter alles gezeigt. Im "
    "Namensfeld l&auml;sst sich au&szlig;erdem ein eigener Name (z.B. \"Garten\") statt "
    "des Herstellernamens vergeben &ndash; erscheint dann &uuml;berall, auch in "
    "ntfy-Meldungen. Leer lassen = Herstellername anzeigen.</p>"
    "<form method=POST action='/alle'><div class=table-wrap><table>"
    "<tr><th>Eigen</th><th>Name</th><th>Modell</th><th>Kanal</th><th>ID</th><th>Temp.</th>"
    "<th>rel. Feuchte</th><th>abs. Feuchte</th><th>Empfang</th><th>Anzahl</th><th>Batterie</th></tr>";
  // rows EINZELN anhaengen (siehe handleRoot/handleSettingsGet) - bei bis zu
  // 32 Sensoren kann das ein grosser Block werden, den man nicht mit dem
  // Kopf-Text in einem Rutsch zusammenbauen sollte.
  page += rows;
  page += "</table></div><button type=submit>Speichern</button></form>"
    "</body></html>";
  server.send(200, "text/html; charset=utf-8", page);
}

static void handleAllePost() {
  for (int i = 0; i < sensorCount; i++) {
    String pfx = "f" + String(i);
    if (!server.hasArg(pfx + "_model")) continue;
    String model = server.arg(pfx + "_model");
    if (model.length() == 0) continue;
    int  channel = server.arg(pfx + "_channel").toInt();
    long id      = server.arg(pfx + "_id").toInt();
    bool wanted  = server.hasArg(pfx + "_own");
    String name  = server.arg(pfx + "_name");
    name.trim();
    setSensorMeta(model.c_str(), channel, id, wanted, name.c_str());
  }
  saveOwned();
  server.sendHeader("Location", "/alle?saved=1");
  server.send(303, "text/plain", "");
}

// Definitionen weiter unten (gleich nach startNetServices) - hier schon
// gebraucht, um die Routen zu registrieren.
static void handleWifiGet();
static void handleWifiPost();
static void handleHotspotPost();

static void startNetServices() {
  if (netServicesUp || WiFi.status() != WL_CONNECTED) return;
  WiFi.setSleep(false);   // kein Modem-Sleep: sonst antwortet das Board nach Pausen traege
  configTzTime("CET-1CEST,M3.5.0,M10.5.0/3", "pool.ntp.org", "time.google.com");   // Uhrzeit fuer die Uhr-Anzeige
  if (MDNS.begin(NET_HOSTNAME)) MDNS.addService("http", "tcp", 80);
  ArduinoOTA.setHostname(NET_HOSTNAME);
#ifdef OTA_PASSWORD
  if (strlen(OTA_PASSWORD) > 0) ArduinoOTA.setPassword(OTA_PASSWORD);
#endif
  ArduinoOTA.onStart([]()   { Serial.println(F("OTA: Update startet")); });
  ArduinoOTA.onEnd([]()     { Serial.println(F("OTA: fertig, Neustart")); });
  ArduinoOTA.onError([](ota_error_t e){ Serial.printf("OTA-Fehler %u\n", e); });
  ArduinoOTA.begin();
  server.on("/", handleRoot);
  server.on("/json", handleJson);
  server.on("/andere", handleAndere);
  server.on("/wasser", HTTP_GET, handleWasserGet);
  server.on("/wasser", HTTP_POST, handleWasserPost);
  server.on("/settings", HTTP_GET, handleSettingsGet);
  server.on("/settings", HTTP_POST, handleSettingsPost);
  server.on("/testpush", handleTestPush);
  server.on("/alle", HTTP_GET, handleAlleGet);
  server.on("/alle", HTTP_POST, handleAllePost);
#if USE_OLED
  server.on("/oled.bmp", handleOledBmp);
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204); });   // sonst jedes Mal eine 404-Anfrage mehr
  server.on("/display", HTTP_GET, handleDisplayGet);
  server.on("/display", HTTP_POST, handleDisplayPost);
#endif
  server.on("/wifi", HTTP_GET, handleWifiGet);
  server.on("/wifi", HTTP_POST, handleWifiPost);
  server.on("/hotspot", HTTP_POST, handleHotspotPost);
  server.begin();
  netServicesUp = true;
  Serial.printf("WLAN ok: http://%s.local/  bzw. http://%s/   (OTA: pio run -e ota -t upload)\n",
                NET_HOSTNAME, WiFi.localIP().toString().c_str());
}

// ------------------------------------------------------------------------
//  /wifi: WLAN-Zugangsdaten eingeben/aendern - erreichbar sowohl normal
//  verbunden (http://empfaenger.local/wifi, zum Wechseln) als auch im
//  Hotspot-Fallback unten (http://192.168.4.1/, wenn kein bekanntes WLAN
//  gefunden wurde). In beiden Faellen: speichern -> Neustart -> verbindet
//  sich mit den neuen Daten (oder faellt zurueck in den Hotspot, falls die
//  falsch waren).
// ------------------------------------------------------------------------
static void handleWifiGet() {
  // Das Scannen blockiert einige Sekunden. Die Seite wird NICHT zu einem grossen String
  // zusammengebaut, sondern haeppchenweise gesendet (wie /settings und /): mit offenem
  // Hotspot ist der Heap knapper, und der grosse String scheiterte dann lautlos mit
  // einer leeren Seite.
  int nNets = WiFi.scanNetworks();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8",
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>868-Empfaenger &middot; WLAN</title><style>"
    ":root{color-scheme:light dark}*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;margin:0;"
    "padding:1.4em 1em 2.4em;background:#0f172a;color:#e6ebf5}"
    "h1{max-width:32em;margin:0 auto .3em;font-size:1.3em;display:flex;align-items:center;gap:.4em}"
    ".navlink{color:#7dd3fc;text-decoration:none;font-size:.88em}"
    ".back{max-width:32em;margin:0 auto 1em}"
    "form{max-width:32em;margin:0 auto}"
    ".card{background:linear-gradient(160deg,#1b2740,#141d31);border:1px solid #263352;"
    "border-radius:14px;padding:1em 1.1em;box-shadow:0 4px 14px rgba(0,0,0,.25);margin-bottom:1em}"
    ".card-head{font-weight:600;font-size:.98em;margin-bottom:.3em}"
    ".hint{font-size:.85em;color:#93a0b8;margin:0 0 .9em}"
    ".row{display:flex;flex-direction:column;gap:.3em;margin:.6em 0;font-size:.85em;color:#93a0b8}"
    "input[type=text],input[type=password]{background:#0f1a30;border:1px solid #263352;"
    "border-radius:8px;color:#e6ebf5;padding:.5em .6em;font-size:1em;width:100%}"
    "button{margin-top:1.1em;background:#0369a1;color:#fff;border:0;border-radius:10px;"
    "padding:.6em 1.4em;font-size:1em;cursor:pointer;display:block;margin-left:auto;margin-right:auto}"
    "button:hover{background:#0284c7}"
    "@media(prefers-color-scheme:light){body{background:#f3f5fb;color:#1c2333}"
    ".navlink{color:#0369a1}.card{background:#fff;border-color:#e1e6f0;"
    "box-shadow:0 2px 10px rgba(30,41,70,.08)}.row,.hint{color:#5b6785}"
    "input[type=text],input[type=password]{background:#eef2fa;border-color:#dde3f0;color:#1c2333}}"
    "</style></head><body>"
    "<h1>&#128246; WLAN einrichten</h1>");
  if (!apMode)
    server.sendContent("<p class=back><a class=navlink href='/'>&larr; zur&uuml;ck zur &Uuml;bersicht</a></p>");

  server.sendContent("<form method=POST action='/wifi'><datalist id=ssids>");
  for (int i = 0; i < nNets && i < 40; i++) {
    String ssid = WiFi.SSID(i);
    ssid.replace("&", "&amp;");
    ssid.replace("<", "&lt;");
    ssid.replace("'", "&#39;");
    server.sendContent("<option value='" + ssid + "'>" + ssid + " (" + String(WiFi.RSSI(i)) +
            "&nbsp;dBm)</option>");
  }
  WiFi.scanDelete();
  server.sendContent("</datalist>");

  server.sendContent("<div class=card><div class=card-head>Haupt-WLAN</div>");
  if (apMode) {
    server.sendContent(String("<p class=hint>Kein bekanntes WLAN gefunden &ndash; das Board hat diesen Hotspot "
            "selbst aufgemacht (<code>") + AP_SETUP_SSID + "</code>, Passwort <code>" + htmlEscStr(AP_SETUP_PASSWORD) +
            "</code>). Trag hier dein WLAN ein (aus der "
            "Liste w&auml;hlen oder eintippen); das Board startet danach neu und verbindet sich damit.</p>");
  } else {
    server.sendContent("<p class=hint>Aktuell verbunden mit <code>" + String(wifiSsid) + "</code>. Neues WLAN "
            "eintragen und speichern &ndash; das Board startet danach neu. Passwortfeld leer lassen, "
            "um bei gleicher SSID das aktuelle Passwort zu behalten.</p>");
  }
  server.sendContent("<label class=row>WLAN-Name (SSID)<input type=text name=ssid list=ssids maxlength=32 "
          "required value='" + String(apMode ? "" : wifiSsid) + "'></label>"
          "<label class=row>WLAN-Passwort<input type=password name=pass maxlength=64 placeholder='" +
          String(apMode ? "" : "aktuelles Passwort behalten") + "'></label></div>");

  server.sendContent("<div class=card><div class=card-head>Backup-WLAN <span class=hint style='margin:0'>"
          "(optional)</span></div>"
          "<p class=hint>Wird versucht, wenn das Haupt-WLAN beim Start nicht erreichbar ist "
          "&ndash; z.B. ein Handy-Hotspot. Leer lassen, um keins zu hinterlegen.</p>"
          "<label class=row>WLAN-Name (SSID)<input type=text name=ssid2 list=ssids maxlength=32 "
          "value='" + String(wifiSsid2) + "'></label>"
          "<label class=row>WLAN-Passwort<input type=password name=pass2 maxlength=64 placeholder='" +
          String(wifiSsid2[0] ? "aktuelles Passwort behalten" : "") + "'></label></div>");
  server.sendContent("<button type=submit>Speichern &amp; neu starten</button></form>");

  // Hotspot: Status und manuelles Ein-/Ausschalten (nur im normalen Betrieb sinnvoll;
  // im Boot-Fallback ist er ohnehin offen).
  if (!apMode) {
    server.sendContent(String("<form method=POST action='/hotspot' style='margin-top:1.4em'><div class=card>"
            "<div class=card-head>Hotspot</div><p class=hint>Notfall-Zugang, falls das WLAN ausf&auml;llt: "
            "&ouml;ffnet sich automatisch, wenn das WLAN l&auml;nger als 2 Minuten weg ist, und schlie&szlig;t sich "
            "wieder, sobald es stabil zur&uuml;ck ist. Name: <code>") + AP_SETUP_SSID +
            "</code> &middot; Adresse <code>http://192.168.4.1/</code> &middot; Passwort <code>" + htmlEscStr(AP_SETUP_PASSWORD) +
            "</code>."
            "<br>Aktuell: <b>" + (apActive ? "an" : "aus") + "</b></p>"
            "<input type=hidden name=on value='" + (apActive ? "0" : "1") + "'>"
            "<button type=submit>" + (apActive ? "Hotspot schlie&szlig;en" : "Hotspot jetzt &ouml;ffnen") +
            "</button></div></form>");
  }
  server.sendContent("</body></html>");
  server.sendContent("");   // beendet den chunked-Transfer
}

static void handleWifiPost() {
  String ssid  = server.arg("ssid");
  String pass  = server.arg("pass");
  String ssid2 = server.arg("ssid2");
  String pass2 = server.arg("pass2");
  ssid.trim();
  ssid2.trim();
  if (ssid.length() == 0) {
    server.send(400, "text/plain; charset=utf-8", "SSID darf nicht leer sein.");
    return;
  }
  if (pass.length() == 0 && !apMode) pass = String(wifiPass);     // leer = aktuelles Passwort behalten
  if (ssid2.length() == 0) pass2 = "";                            // kein Backup -> auch kein Passwort merken
  else if (pass2.length() == 0) pass2 = String(wifiPass2);        // leer = aktuelles Backup-Passwort behalten
  saveWifiCreds(ssid.c_str(), pass.c_str(), ssid2.c_str(), pass2.c_str());

  String page =
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>868-Empfaenger &middot; WLAN</title><style>"
    ":root{color-scheme:light dark}"
    "body{font-family:system-ui,sans-serif;margin:0;padding:2.4em 1em;background:#0f172a;"
    "color:#e6ebf5;text-align:center}"
    "@media(prefers-color-scheme:light){body{background:#f3f5fb;color:#1c2333}}"
    "</style></head><body><p>Gespeichert. Das Board startet neu und verbindet sich mit <b>" +
    ssid + "</b>" + (ssid2.length() ? " (Backup: <b>" + ssid2 + "</b>)" : "") +
    " &hellip;</p></body></html>";
  server.send(200, "text/html; charset=utf-8", page);
  delay(800);   // Antwort noch rausschreiben lassen, bevor der Neustart die Verbindung kappt
  ESP.restart();
}

// Kein bekanntes WLAN gefunden -> eigener Hotspot, damit man trotzdem an
// die Konfiguration kommt (sonst waere das Board bei falschem/geaendertem
// WLAN-Passwort nur noch per USB erreichbar). WIFI_AP_STA statt WIFI_AP,
// damit handleWifiGet() (fuer die Netzwerkauswahl auf /wifi) auch im
// Hotspot-Modus noch nach WLANs suchen kann.
//
// Hotspot-Faelle:
//  1) Boot-Fallback (apMode): beim Start war kein bekanntes WLAN erreichbar -> Hotspot
//     mit Setup-Seite. Im Hintergrund wird alle 60 s erneut versucht, sich mit dem Haupt-/
//     Backup-WLAN zu verbinden (apRetryConnect()), bei Erfolg Neustart - z.B. wenn der
//     Router nach einem Stromausfall erst NACH dem Board wieder da ist.
//  2) Laufzeit-Hotspot: faellt das WLAN im Betrieb laenger als HOTSPOT_AFTER_MS aus, wird der
//     Hotspot zusaetzlich geoeffnet (hotspotWatch()); die komplette Weboberflaeche ist dann
//     auch unter http://192.168.4.1/ erreichbar. Ist das WLAN wieder stabil da, schliesst er
//     sich. Auf /wifi laesst er sich ausserdem von Hand oeffnen/schliessen (apManual).
static const uint32_t HOTSPOT_AFTER_MS = 120000UL;

static void startHotspot() {
  if (apActive) return;
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SETUP_SSID, AP_SETUP_PASSWORD);
  apActive = true;
  Serial.printf("Hotspot '%s' aktiv, IP %s - dort http://%s/ oeffnen "
                "(Hotspot-Passwort siehe secrets.h)\n",
                AP_SETUP_SSID, WiFi.softAPIP().toString().c_str(),
                WiFi.softAPIP().toString().c_str());
}

static void stopHotspot() {
  if (!apActive || apMode) return;      // im Boot-Fallback bleibt er bis zum Neustart offen
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apActive = false;
  Serial.println(F("Hotspot geschlossen (WLAN wieder da)"));
}

static void startApFallback() {
  apMode = true;
  startHotspot();
  server.on("/", handleWifiGet);
  server.on("/wifi", HTTP_GET, handleWifiGet);
  server.on("/wifi", HTTP_POST, handleWifiPost);
  server.begin();
  Serial.println(F("Kein WLAN gefunden - dort neues WLAN eintragen; im Hintergrund wird es weiter versucht"));
}

// Im Boot-Fallback: nicht blockierend alle 60 s Haupt-, dann Backup-WLAN probieren.
static void apRetryConnect() {
  static uint32_t lastTry = 0, attemptStart = 0;
  static uint8_t stage = 0;    // 0 = Pause, 1 = Haupt-WLAN laeuft, 2 = Backup-WLAN laeuft
  uint32_t now = millis();
  if (stage == 0) {
    if (now - lastTry < 60000UL) return;
    if (!wifiSsid[0] && !wifiSsid2[0]) return;       // noch nie ein WLAN eingetragen
    lastTry = now;
    attemptStart = now;
    stage = 1;
    Serial.println(F("Hotspot: versuche erneut, mit dem WLAN zu verbinden ..."));
    WiFi.begin(wifiSsid, wifiPass);
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("Hotspot: WLAN wieder erreichbar - Neustart"));
    delay(300);
    ESP.restart();
  }
  if (now - attemptStart > 12000UL) {
    if (stage == 1 && wifiSsid2[0]) {
      stage = 2;
      attemptStart = now;
      WiFi.begin(wifiSsid2, wifiPass2);
    } else {
      stage = 0;
      WiFi.disconnect(false, false);                 // nur die Station trennen, Hotspot bleibt
    }
  }
}

// Im normalen Betrieb: WLAN-Ausfall beobachten, Hotspot oeffnen/schliessen, Wiederverbindung anstossen.
static void hotspotWatch() {
  static uint32_t downSince = 0, upSince = 0, lastKick = 0;
  uint32_t now = millis();
  if (WiFi.status() == WL_CONNECTED) {
    downSince = 0;
    if (!upSince) upSince = now;
    if (apActive && !apManual && now - upSince > 15000UL) stopHotspot();
  } else {
    upSince = 0;
    if (!downSince) downSince = now;
    if (!apActive && now - downSince > HOTSPOT_AFTER_MS) startHotspot();
    if (now - lastKick > 30000UL) { lastKick = now; WiFi.reconnect(); }
  }
}

// Hotspot von /wifi aus von Hand oeffnen (on=1) oder schliessen.
static void handleHotspotPost() {
  if (server.arg("on") == "1") { apManual = true; startHotspot(); }
  else { apManual = false; if (WiFi.status() == WL_CONNECTED) stopHotspot(); }
  server.sendHeader("Location", "/wifi");
  server.send(303, "text/plain", "");
}

// Verbindet mit der gegebenen SSID/Passwort, wartet bis zu 15s. true = verbunden.
static bool tryConnect(const char *ssid, const char *pass) {
  if (!ssid[0]) return false;
  // Der ESP32-Kern verlangt sonst mindestens WPA2 und lehnt offene Netze ab. Nur lockern,
  // wenn fuer dieses Netz wirklich kein Passwort hinterlegt ist.
  WiFi.setMinSecurity(pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN);
  WiFi.begin(ssid, pass);
  Serial.printf("WLAN: verbinde mit '%s' ", ssid);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) { delay(250); Serial.print('.'); }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

static void netSetup() {
  loadWifiCreds();
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(NET_HOSTNAME);
  WiFi.setAutoReconnect(true);
  if (tryConnect(wifiSsid, wifiPass)) { startNetServices(); return; }
  if (wifiSsid2[0]) {
    Serial.println(F("Haupt-WLAN nicht erreichbar - versuche Backup-WLAN..."));
    if (tryConnect(wifiSsid2, wifiPass2)) { startNetServices(); return; }
  }
  startApFallback();
}

// Der WebServer bedient nur EINE Verbindung auf einmal und wartet nach der Antwort bzw. bei leeren
// Vorab-Verbindungen der Browser (preconnect) jeweils bis zu 5 s - dann haengt die Seite. Eine
// Verbindung, ueber die nichts mehr kommt, wird deshalb nach 1,5 s selbst geschlossen.
static void serveWeb() {
  server.handleClient();
  static uint32_t idleSince = 0;
  WiFiClient cl = server.client();
  if (cl && cl.connected()) {
    if (!idleSince) idleSince = millis();
    else if (millis() - idleSince > 1500 && !cl.available()) { cl.stop(); idleSince = 0; }
  } else {
    idleSince = 0;
  }
}

static void netLoop() {
  if (apMode) {                                    // Boot-Fallback: nur die Setup-Seite bedienen
    serveWeb();
    apRetryConnect();
    return;
  }
  if (netServicesUp) {
    ArduinoOTA.handle();
    serveWeb();
    hotspotWatch();
    return;
  }
  if (WiFi.status() == WL_CONNECTED) { startNetServices(); return; }
  // Der ESP32 wiederholt einen fehlgeschlagenen Erstverbindungsversuch NICHT
  // von selbst -> alle 20 s neu anstossen.
  static uint32_t lastTry = 0;
  if (millis() - lastTry > 20000) {
    lastTry = millis();
    Serial.println(F("WLAN: neuer Verbindungsversuch..."));
    WiFi.disconnect();
    WiFi.begin(wifiSsid, wifiPass);
  }
}
#endif  // USE_WIFI

/* ------------------------------------------------------------------ *
 *  6b) MQTT / Home Assistant (Auto-Discovery, ein HA-Geraet je       *
 *      physischem 868-MHz-Sensor). Baugleich zum Discovery-Muster    *
 *      aus Feuchtewaechter-433, hier aber dynamisch fuer beliebig    *
 *      viele, zur Laufzeit erst entdeckte Sensoren statt fester      *
 *      innen/aussen-Rollen.                                          *
 * ------------------------------------------------------------------ */
#if USE_MQTT
static WiFiClient   mqttNet;
static PubSubClient mqtt(mqttNet);
static char         gBridgeId[20] = "empfaenger";      // eindeutig aus MAC, in netSetup()

// Sensorschluessel fuer Topics/IDs: Modell+Kanal+ID, alles ausser [A-Za-z0-9_]
// wird zu '_' (rtl_433-Modellnamen wie "Nexus-TH" enthalten Bindestriche).
static void sensorKey(const Sensor &s, char *out, size_t outLen) {
  char model[32];
  strncpy(model, s.model, sizeof(model) - 1);
  model[sizeof(model) - 1] = 0;
  for (char *p = model; *p; p++)
    if (!isalnum((unsigned char)*p)) *p = '_';
  snprintf(out, outLen, "%s_ch%d_id%ld", model, s.channel, s.id);
}

static void publishSensorDiscovery(const Sensor &s) {
  char key[40];
  sensorKey(s, key, sizeof(key));
  char devName[48], topic[110], pl[620];
  snprintf(devName, sizeof(devName), "868 %s Kanal %d ID %ld",
           s.model, s.channel, s.id);

  // Temperatur
  snprintf(pl, sizeof(pl),
    "{\"name\":\"Temperatur\",\"uniq_id\":\"%s_%s_t\",\"stat_t\":\"%s/%s/state\","
    "\"val_tpl\":\"{{ value_json.t }}\",\"unit_of_meas\":\"\\u00b0C\","
    "\"dev_cla\":\"temperature\",\"stat_cla\":\"measurement\",\"exp_aft\":3600,"
    "\"dev\":{\"ids\":[\"%s_%s\"],\"name\":\"%s\",\"mf\":\"868MHz\",\"mdl\":\"%s\","
    "\"via_device\":\"%s\"}}",
    gBridgeId, key, gBridgeId, key, gBridgeId, key, devName, s.model, gBridgeId);
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_%s/t/config", gBridgeId, key);
  mqtt.publish(topic, pl, true);

  // Rel. Luftfeuchte
  snprintf(pl, sizeof(pl),
    "{\"name\":\"Luftfeuchte\",\"uniq_id\":\"%s_%s_rh\",\"stat_t\":\"%s/%s/state\","
    "\"val_tpl\":\"{{ value_json.rh }}\",\"unit_of_meas\":\"%%\","
    "\"dev_cla\":\"humidity\",\"stat_cla\":\"measurement\",\"exp_aft\":3600,"
    "\"dev\":{\"ids\":[\"%s_%s\"],\"name\":\"%s\"}}",
    gBridgeId, key, gBridgeId, key, gBridgeId, key, devName);
  snprintf(topic, sizeof(topic), "homeassistant/sensor/%s_%s/rh/config", gBridgeId, key);
  mqtt.publish(topic, pl, true);
}

static void publishSensorState(const Sensor &s) {
  char key[40], topic[70], pl[96];
  sensorKey(s, key, sizeof(key));
  snprintf(pl, sizeof(pl), "{\"t\":%.1f,\"rh\":%.0f,\"age_s\":%lu}",
           s.t, s.rh, (unsigned long)ageSec(s));
  snprintf(topic, sizeof(topic), "%s/%s/state", gBridgeId, key);
  mqtt.publish(topic, pl, true);
}

static void mqttLoop() {
  if (WiFi.status() != WL_CONNECTED) return;
  if (mqtt.connected()) {
    mqtt.loop();
    // Neu entdeckte Sensoren einmalig anmelden.
    for (int i = 0; i < sensorCount; i++) {
      if (!sensors[i].discovered) {
        publishSensorDiscovery(sensors[i]);
        sensors[i].discovered = true;
      }
    }
    // Alle bekannten Sensoren regelmaessig aktualisieren (HA markiert
    // ueber "exp_aft" selbststaendig veraltete/verstummte Sensoren).
    static uint32_t lastPub = 0;
    if (millis() - lastPub > 30000) {
      lastPub = millis();
      for (int i = 0; i < sensorCount; i++)
        if (sensors[i].valid) publishSensorState(sensors[i]);
    }
    return;
  }
  static uint32_t lastTry = 0;
  if (millis() - lastTry < 5000) return;
  lastTry = millis();
  char avail[32];
  snprintf(avail, sizeof(avail), "%s/status", gBridgeId);
  if (mqtt.connect(gBridgeId, MQTT_USER, MQTT_PASS, avail, 0, true, "offline")) {
    mqtt.publish(avail, "online", true);
    for (int i = 0; i < sensorCount; i++) sensors[i].discovered = false;  // neu anmelden
    Serial.println(F("MQTT: verbunden"));
  } else {
    Serial.printf("MQTT: connect fehlgeschlagen (rc=%d) - neuer Versuch in 5 s\n", mqtt.state());
  }
}
#endif  // USE_MQTT

/* ------------------------------------------------------------------ *
 *  7) SETUP / LOOP                                                  *
 * ------------------------------------------------------------------ */
#ifdef HEAPTRACE
#include <esp_heap_caps.h>
static volatile size_t g_failSize = 0;
static volatile uint32_t g_failCaps = 0;
static volatile uint32_t g_failCount = 0;
static void onAllocFail(size_t size, uint32_t caps, const char *fn) {
  g_failSize = size;
  g_failCaps = caps;
  g_failCount = g_failCount + 1;
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);
  // Legt den (sonst erst beim ersten printf angelegten) Lock der Konsolen-Ausgabe
  // jetzt an, solange Speicher da ist. Sonst bricht lock_init_generic() mit abort()
  // ab, wenn der erste printf-Aufruf (mDNS-Fehlermeldung im tcpip-Task) in einen
  // Speicherengpass faellt.
  printf("\n");
#ifdef HEAPTRACE
  heap_caps_register_failed_alloc_callback(onAllocFail);
#endif
  Serial.println(F("\n== LilyGo 868-Empfaenger =="));

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  loadThresholds();
  loadOwned();
  loadRadioSettings();
  loadRefSensor();
  loadLeaks();
#if USE_THINGSPEAK
  loadThingSpeakSettings();
#endif

#if USE_OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(100000);
  u8g2.setI2CAddress(0x3C << 1);
  if (u8g2.begin()) {
    u8g2.setBusClock(100000);
    g_oledOk = true;
    splash();
  } else {
    Serial.println(F("Warnung: kein OLED gefunden - laeuft ohne Display weiter"));
  }
#endif

#if USE_RADIO
  rf.initReceiver(RF_MODULE_RECEIVER_GPIO, RF_MODULE_FREQUENCY);
  rf.setCallback(rtl_433_Callback, messageBuffer, JSON_MSG_BUFFER);
  rf.enableReceiver();
  rf.setRSSIThreshold(rssiSensitivity);
  Serial.printf("Empfaenger aktiv auf %.2f MHz (GPIO %d), RSSI-Schwelle %d dB\n",
                (double)RF_MODULE_FREQUENCY, RF_MODULE_RECEIVER_GPIO, rssiSensitivity);
#endif

#if USE_WIFI
  netSetup();                        // nach dem Funk-Init, damit der Empfang zuerst laeuft
#endif

#if USE_MQTT
  uint8_t mac[6];
  WiFi.macAddress(mac);
  snprintf(gBridgeId, sizeof(gBridgeId), "empfaenger_%02x%02x%02x", mac[3], mac[4], mac[5]);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(768);   // Discovery-Payloads passen nicht in die 256-B-Voreinstellung
#endif
}

void loop() {
#if USE_RADIO
  rf.loop();                         // Funk-Dekoder bedienen (nicht blockieren!)
#endif
#if USE_WIFI
  netLoop();                         // OTA + Webserver bedienen
#endif
#if USE_MQTT
  mqttLoop();                        // Discovery + State an Home Assistant
#endif
  updateLed();
#if USE_OLED
  displayTick();
#endif

  static uint32_t lastTick = 0;
  if (millis() - lastTick >= 1000) {
    lastTick = millis();
#if USE_OLED
    graphSample();
#endif
    static uint32_t lastLog = 0;
    if (millis() - lastLog >= 30000) {     // alle 30 s eine Statuszeile
      lastLog = millis();
      Serial.printf("STATUS  %d Sensor(en) bekannt, Heap frei %u | 8Bit frei %u (min %u, Block %u)\n",
                    sensorCount, (unsigned)ESP.getFreeHeap(), (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
#ifdef HEAPTRACE
    static uint32_t lastFailCount = 0;
    if (g_failCount != lastFailCount) {
      lastFailCount = g_failCount;
      Serial.printf("ALLOCFAIL Nr.%u: %u Byte, caps=0x%x, 8Bit frei=%u, Block=%u, min=%u\n",
                    (unsigned)lastFailCount, (unsigned)g_failSize, (unsigned)g_failCaps,
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (millis() < 90000) {
      Serial.printf("HEAP t=%us 8Bit frei=%u min=%u block=%u\n", (unsigned)(millis() / 1000),
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
#endif
    static uint32_t lastSignalCheck = 0;
    if (millis() - lastSignalCheck >= 15000) {   // alle 15 s auf Signalverlust pruefen
      lastSignalCheck = millis();
      checkSignalLoss();
      leakCheckLost();
    }
#if USE_THINGSPEAK
    static uint32_t lastThingSpeak = 0;
    if (millis() - lastThingSpeak >= 60000) {    // alle 60 s (ThingSpeak-Limit: min. 15s)
      lastThingSpeak = millis();
      sendThingSpeak();
    }
#endif
  }
}
