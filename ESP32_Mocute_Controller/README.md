# ESP32 Mocute 052 -> Hoverboard Controller

ESP32-basierter Bluetooth-Empfänger für den **Mocute 052** Gamepad-Controller.
Steuert das Hoverboard über die serielle UART-Schnittstelle mit dem
[hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC) Protokoll.

Unterstützt optional ein **Dual-ESP32 System** mit Richtungserkennung für
den Follow Me Modus.

## Funktionsübersicht

### Einzel-ESP32 (Basic)
```
Mocute 052  ──BT──>  ESP32  ──UART──>  Hoverboard Mainboard
(Gamepad)           (Master)           (FOC Firmware)
```

### Dual-ESP32 (mit Richtungserkennung)
```
                     ┌─────────────────────────────────────┐
                     │          Hoverboard                 │
Mocute 052  ──BT──>  │  Master ESP32 (rechtes Sideboard)   │
(Gamepad)            │      │ UART ──> Mainboard           │
    │                │      │ ESP-NOW                      │
    │  BT Signal     │      ▼                              │
    └─ ─ ─ ─ ─ ─ ─> │  Satellite ESP32 (linkes Sideboard) │
    (nicht verbunden)│                                     │
                     └─────────────────────────────────────┘

RSSI_Master > RSSI_Satellite → Nutzer ist rechts  → lenke rechts
RSSI_Satellite > RSSI_Master → Nutzer ist links   → lenke links
RSSI_Master ≈ RSSI_Satellite → Nutzer ist mittig  → geradeaus
```

## Hardware

### Benötigte Komponenten

**Einzel-ESP32 (Basic):**
- 1x ESP32 DevKit V1 (oder kompatibel)
- Mocute 052 Bluetooth Gamepad
- Hoverboard mit FOC Firmware
- 3 Kabel für die Verbindung (GND, TX, RX)

**Dual-ESP32 (mit Richtungserkennung):**
- 2x ESP32 DevKit V1 (oder kompatibel)
- Mocute 052 Bluetooth Gamepad
- Hoverboard mit FOC Firmware
- 3 Kabel für Master (GND, TX, RX)
- Stromversorgung für Satellite (USB / 3.3V von Sideboard)

### Verkabelung ESP32 → Hoverboard

| ESP32 Pin | Hoverboard Sensor-Kabel | Farbe   | Beschreibung |
|-----------|------------------------|---------|--------------|
| GPIO 16   | TX                     | Grün    | ESP32 RX ← Hoverboard TX |
| GPIO 17   | RX                     | Gelb    | ESP32 TX → Hoverboard RX |
| GND       | GND                    | Schwarz | Masse        |
| -         | -                      | **ROT** | **NICHT ANSCHLIESSEN! 15V!** |

> ⚠️ **WARNUNG**: Das rote Kabel führt 15V und zerstört den ESP32!

### Hoverboard Firmware Konfiguration

In der `config.h` des Hoverboards muss die serielle Steuerung aktiviert sein:

**Option 1: Rechtes Sensor-Kabel (USART3) - empfohlen** (5V-tolerant):
```c
#define VARIANT_USART
#define CONTROL_SERIAL_USART3  0
#define FEEDBACK_SERIAL_USART3
```

**Option 2: Linkes Sensor-Kabel (USART2)** (nur 3.3V!):
```c
#define VARIANT_USART
#define CONTROL_SERIAL_USART2  0
#define FEEDBACK_SERIAL_USART2
```

## Steuerung (Control Mapping)

| Eingabe              | Funktion                          |
|----------------------|-----------------------------------|
| Linker Joystick Y    | Geschwindigkeit (vor/zurück)      |
| Linker Joystick X    | Lenkung (links/rechts)            |
| D-Pad Hoch/Runter    | Langsam vor/zurück                |
| D-Pad Links/Rechts   | Langsam lenken                    |
| Taste A / R1         | Turbo-Modus (höhere Geschw.)      |
| Taste B / L1         | Notbremse (sofortiger Stopp)      |
| **Taste X**          | **Follow Me Modus ein/aus**       |
| **Taste Y**          | **RSSI-Kalibrierungswerte anzeigen** |

### Geschwindigkeitslimits (einstellbar in `main.cpp`)

| Parameter           | Wert | Beschreibung                   |
|---------------------|------|--------------------------------|
| SPEED_LIMIT_NORMAL  | 300  | Normale max. Geschwindigkeit   |
| SPEED_LIMIT_TURBO   | 600  | Turbo max. Geschwindigkeit     |
| STEER_LIMIT         | 400  | Max. Lenkausschlag             |
| JOYSTICK_DEADZONE   | 30   | Totzone des Joysticks          |
| GAMEPAD_TIMEOUT_MS  | 500  | Sicherheits-Timeout (ms)       |

> Der Hoverboard-Eingangsbereich ist [-1000, 1000]. Die Limits halten die Werte
> bewusst niedriger für sicheres Fahren. Werte können bei Bedarf angepasst werden.

## Sicherheitsfunktionen

1. **Notbremse**: Taste B / L1 setzt Geschwindigkeit und Lenkung sofort auf 0
2. **Verbindungs-Timeout**: Motoren stoppen wenn keine Gamepad-Daten für 500ms empfangen werden
3. **Disconnect-Stop**: Motoren stoppen sofort wenn der Controller die BT-Verbindung verliert
4. **Geschwindigkeitsbegrenzung**: Standard-Geschwindigkeit auf 300/1000 begrenzt
5. **Joystick-Totzone**: Verhindert unbeabsichtigtes Fahren bei nicht zentriertem Joystick
6. **Follow Me RSSI-Timeout**: Motoren stoppen wenn kein BT-Signal für 2s empfangen wird
7. **Notbremse deaktiviert Follow Me**: Emergency Stop beendet den Follow Me Modus sofort


## Follow Me Modus (RSSI-basiert)

### Funktionsprinzip

Der Follow Me Modus nutzt die **Bluetooth-Signalstärke (RSSI)** um die Entfernung
zum Gamepad-Controller abzuschätzen und das Hoverboard automatisch folgen zu lassen.

```
      Nutzer mit              ESP32 +
      Gamepad                 Hoverboard
        📱  ─── BT RSSI ───>  🛹
        
  Entfernung ↑  → RSSI ↓  → Geschwindigkeit ↑ (folgen)
  Entfernung ↓  → RSSI ↑  → Geschwindigkeit ↓ (stoppen)
```

### RSSI-Zonen

| Zone     | RSSI Bereich | Verhalten                        |
|----------|-------------|----------------------------------|
| CLOSE    | ≥ 200       | Langsam rückwärts (zu nah)       |
| IDEAL    | 180 - 199   | Anhalten (ideale Distanz)        |
| FOLLOW   | 155 - 179   | Vorwärts, proportional zur Dist. |
| FAR      | 130 - 154   | Vorwärts, maximale Follow-Geschw.|
| LOST     | < 130       | Notbremse (Signal verloren)      |

> ⚠️ **Wichtig**: Die RSSI-Werte müssen für eure Umgebung kalibriert werden!
> Nutzt Taste Y um die aktuellen Werte im Serial Monitor anzuzeigen.

### Kalibrierung

1. Gamepad verbinden und Follow Me **nicht** aktivieren
2. **Taste Y** drücken in verschiedenen Entfernungen:
   - Direkt neben dem Hoverboard (→ notieren als RSSI_ZONE_CLOSE)
   - In idealer Folge-Distanz ~1-2m (→ RSSI_ZONE_IDEAL)
   - In ~3-4m Entfernung (→ RSSI_ZONE_FOLLOW)
   - In ~6-8m Entfernung (→ RSSI_ZONE_FAR)
3. Werte in `include/follow_me.h` anpassen
4. Neu flashen

### Einschränkungen

- **Einzel-ESP32: Keine Richtungserkennung**: RSSI kann nur Entfernung schätzen, nicht die Richtung.
  Lenkung muss weiterhin manuell per Joystick erfolgen!
- **Dual-ESP32: Grobe Richtung**: Mit zwei ESP32 auf gegenüberliegenden Sideboards kann die
  grobe Richtung (links/rechts) erkannt werden. Automatische Lenkung möglich!
- **Signalschwankungen**: RSSI schwankt durch Körperhaltung, Reflexionen, Hindernisse
- **Umgebungsabhängig**: Werte variieren stark zwischen Innen/Außen und verschiedenen Räumen
- **Beste Ergebnisse**: Im Freien mit Sichtverbindung
- **Experiment**: Dieser Modus ist experimentell und sollte erstmalig nur mit niedrigen Geschwindigkeiten getestet werden!


## Dual-ESP32 Richtungserkennung

### Prinzip

Zwei ESP32-Module werden an gegenüberliegenden Seiten des Hoverboards montiert.
Beide messen die Bluetooth-Signalstärke (RSSI) des Gamepads. Aus der **Differenz
der RSSI-Werte** wird die Richtung des Benutzers berechnet:

| Situation | RSSI Master | RSSI Satellite | Ergebnis |
|-----------|-------------|---------------|----------|
| Nutzer rechts | Stark | Schwach | Lenke rechts |
| Nutzer links | Schwach | Stark | Lenke links |
| Nutzer mittig | ≈ gleich | ≈ gleich | Geradeaus |

### Konfiguration

#### 1. Master ESP32 MAC-Adresse ermitteln

Den Master flashen und im Serial Monitor die MAC-Adresse ablesen:
```
[ESPNOW] Master MAC: AA:BB:CC:DD:EE:FF
[ESPNOW] Set this MAC in the Satellite's masterMacAddress[]!
```

#### 2. Satellite konfigurieren

In `satellite/main.cpp` die MAC-Adressen eintragen:

```cpp
// Master ESP32 MAC address
uint8_t masterMacAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// Gamepad MAC address (aus Master Serial-Output beim Verbinden)
uint8_t targetGamepadMac[] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
```

#### 3. Build & Flash

```bash
# Master flashen (auf dem rechten Sideboard)
pio run -e esp32_master -t upload

# Satellite flashen (auf dem linken Sideboard)
pio run -e esp32_satellite -t upload
```

#### 4. Montage

- **Master ESP32**: Am rechten Sideboard, mit UART-Verbindung zum Mainboard
- **Satellite ESP32**: Am linken Sideboard, nur Stromversorgung nötig
- Möglichst symmetrisch montieren, gleiche Höhe und Ausrichtung
- Abstand zwischen Master und Satellite ≈ Hoverboard-Breite (~50cm)

### Richtungs-Kalibrierung

1. Follow Me aktivieren (Taste X)
2. Mit Gamepad auf verschiedene Positionen gehen:
   - Direkt mittig vor dem Hoverboard
   - Links neben dem Hoverboard
   - Rechts neben dem Hoverboard
3. **Taste Y** für Kalibrierungswerte
4. Schwellenwerte in `include/direction_detect.h` anpassen:

```cpp
#define DIR_RSSI_DEADZONE   5    // Totzone (keine Lenkung)
#define DIR_RSSI_STRONG    20    // Starke Richtungserkennung
#define DIR_STEER_MAX     300    // Max. Auto-Lenkwert
```

### Einschränkungen Dual-ESP32

- **Keine Vorne/Hinten-Unterscheidung**: Nur links/rechts erkennbar
- **BT Inquiry nötig**: Satellite scannt per BT Classic Inquiry nach dem Gamepad.
  Manche Gamepads sind im verbundenen Zustand nicht per Inquiry sichtbar!
- **Fallback**: Falls der Mocute nicht per Inquiry sichtbar ist, kann alternativ
  ein BLE Beacon-Tag verwendet werden (Konfiguration in `satellite/main.cpp`)
- **WiFi deaktiviert**: ESP-NOW nutzt den WiFi-Chip. Keine gleichzeitige WiFi-Verbindung möglich.
- **Latenz**: BT Inquiry dauert einige Sekunden → Reaktionszeit ist langsamer als RSSI über Verbindung

## Installation & Build

### Voraussetzungen
- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)

### Build & Flash

```bash
# In das Projektverzeichnis wechseln
cd ESP32_Mocute_Controller

# === Einzel-ESP32 (nur Master) ===
pio run -e esp32_master
pio run -e esp32_master -t upload
pio device monitor

# === Dual-ESP32 ===
# Master flashen
pio run -e esp32_master -t upload

# Satellite flashen (zweiten ESP32 anschließen!)
pio run -e esp32_satellite -t upload

# Beide gleichzeitig bauen
pio run
```

### Mocute 052 Pairing

1. ESP32 einschalten (flashen + mit Strom versorgen)
2. Mocute 052 einschalten (Power-Taste gedrückt halten)
3. Mocute in den **Game-Modus** setzen (Mode B / iCade)
4. ESP32 erkennt den Gamepad automatisch und verbindet sich
5. Blaue LED am ESP32 leuchtet bei erfolgreicher Verbindung

### Serial Monitor Ausgabe

```
============================================
  ESP32 Mocute 052 -> Hoverboard Controller
       + Follow Me Mode (RSSI-based)
============================================

[HOVER] Serial initialized at 115200 baud (RX=16, TX=17)
[BP32]  Initializing Bluepad32...
[BP32]  Ready! Waiting for Mocute 052 gamepad...

Controls:
  Joystick L  = Steer + Speed
  Button A/R1 = Turbo
  Button B/L1 = Emergency Stop
  Button X    = Toggle Follow Me mode
  Button Y    = Print RSSI calibration values

[BP32]  Controller connected, index=0
[CMD]   steer=   0 speed=   0 turbo=0 estop=0 connected=1

# After pressing Button X:
[FOLLOW] Follow Me mode: ENABLED
[FOLLOW] steer=   0 speed= 150 zone=FOLLOW rssi=170(168.5) connected=1

# After pressing Button Y:
[RSSI-CAL] Raw RSSI=172, Filtered=168.5, Zone=FOLLOW
[RSSI-CAL] Hold gamepad at desired distance and note these values.
```

## Projektstruktur

```
ESP32_Mocute_Controller/
├── platformio.ini              # PlatformIO Konfiguration (Master + Satellite)
├── README.md                   # Diese Datei
├── include/
│   ├── hoverboard_serial.h    # Hoverboard Protokoll-Implementierung
│   ├── follow_me.h            # Follow Me Modus (RSSI-basiert + Richtung)
│   ├── espnow_protocol.h     # ESP-NOW Nachrichtenformat (Master ↔ Satellite)
│   └── direction_detect.h    # Richtungserkennung aus Dual-RSSI
├── src/
│   └── main.cpp                # Master-Firmware (Bluepad32 + Steuerlogik)
└── satellite/
    └── main.cpp                # Satellite-Firmware (BT Scanner + ESP-NOW)
```

## Anpassungen

### Andere Gamepads
Bluepad32 unterstützt viele Bluetooth-Controller:
- PS4 / PS5 DualSense
- Xbox Wireless Controller
- Nintendo Switch Pro Controller
- 8BitDo Controller
- Generic BT/BLE Gamepads

Der Code funktioniert ohne Änderung mit allen von Bluepad32 unterstützten Controllern.

### Pin-Änderungen
Die UART-Pins können in `main.cpp` angepasst werden:
```cpp
#define HOVER_RX_PIN    4    // ESP32 RX
#define HOVER_TX_PIN    5    // ESP32 TX
```

## Lizenz

GPL v3 - Kompatibel mit der hoverboard-firmware-hack-FOC Lizenz.
