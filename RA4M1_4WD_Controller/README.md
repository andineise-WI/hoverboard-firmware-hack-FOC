# RA4M1 4WD Hoverboard Controller

Steuert ein **4WD-Fahrzeug** aus zwei Hoverboard-Mainboards (Front + Heck) über eine **HOTRC DS650** Fernsteuerung.

```
┌──────────────┐         PWM CH1-CH4         ┌─────────────────┐
│  HOTRC DS650 │  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─▶  │   RC-Empfänger  │
│  Fernsteuer. │          2.4 GHz             │   (6-Kanal)     │
└──────────────┘                              └──┬──┬──┬──┬─────┘
                                                 │  │  │  │  CH1..CH4 (PWM)
                                                 ▼  ▼  ▼  ▼
                                           ┌─────────────────────┐
                                           │   Arduino UNO R4    │
                                           │   Minima (RA4M1)    │
                                           │                     │
                                           │  4WD-Mischer        │
                                           │  Failsafe           │
                                           │  Expo / Rampe       │
                                           └──┬──────────────┬───┘
                                    UART TX   │              │  UART TX
                                   (Serial1)  │              │  (Board2Serial)
                                              ▼              ▼
                                   ┌──────────────┐  ┌──────────────┐
                                   │ Hoverboard 1 │  │ Hoverboard 2 │
                                   │ FRONT (Vorne)│  │ REAR (Hinten)│
                                   │  ◀L    R▶    │  │  ◀L    R▶    │
                                   └──────────────┘  └──────────────┘
```

## Features

- **4 RC-Kanäle**: Lenkung (CH1), Gas/Bremse (CH2), Speed-Limit-Regler (CH3), Fahrmodus-Schalter (CH4)
- **3 Fahrmodi**: Normal (70%), Sport (100%), Crawl (35%) – umschaltbar über CH4-Schalter
- **Stufenloser Speed-Limiter**: über CH3-Drehregler (VR-Knopf)
- **Expo-Kurven**: Feinfühliges Fahren um die Mittelstellung
- **Rampen**: Sanftes Beschleunigen und Bremsen (einstellbar)
- **Failsafe**: Automatischer Motorstopp bei Signal-Verlust
- **Feedback**: Batteriespannung, Temperatur und Geschwindigkeit von beiden Boards
- **Getrennte Lenk-Ratio**: Front-/Heck-Lenkanteil unabhängig einstellbar
- **Debug-Ausgabe**: Live-Daten über USB Serial Monitor

## Hardware

| Komponente | Beschreibung |
|---|---|
| Fernsteuerung | HOTRC DS650 (2.4 GHz, Pistolengriff) |
| Empfänger | HOTRC DS650 Empfänger (6 Kanäle, PWM-Ausgang) |
| Controller | Arduino UNO R4 Minima (Renesas RA4M1) |
| Front-Antrieb | Hoverboard Mainboard #1, Firmware: hoverboard-firmware-hack-FOC, VARIANT_USART |
| Heck-Antrieb | Hoverboard Mainboard #2, Firmware: hoverboard-firmware-hack-FOC, VARIANT_USART |
| Stromversorgung | Hoverboard-Akkus (36V), RA4M1 über USB oder 5V-Regler |

## Verdrahtung

### RC-Empfänger → RA4M1

| RC-Kanal | Funktion | RA4M1 Pin | Bemerkung |
|---|---|---|---|
| CH1 Signal | Lenkung | D2 | PWM-Eingang |
| CH2 Signal | Gas/Bremse | D4 | PWM-Eingang |
| CH3 Signal | Speed-Limit (VR) | D5 | PWM-Eingang (optional) |
| CH4 Signal | Modus-Schalter | D10 | PWM-Eingang (optional) |
| VCC | Stromversorgung | 3.3V | **Siehe Spannungshinweis!** |
| GND | Masse | GND | |

> **⚠ Spannungshinweis:** Die RA4M1-GPIO-Pins sind **3.3V**. Wenn der Empfänger mit 5V betrieben wird, unbedingt **Spannungsteiler** oder **Level-Shifter** an den Signal-Leitungen verwenden! Alternativ den Empfänger mit 3.3V versorgen (viele Empfänger funktionieren bei 3.3V).

### RA4M1 → Hoverboard 1 (Front)

Nutzt **Serial1** (Hardware-UART):

| RA4M1 Pin | Hoverboard-Kabel | Funktion |
|---|---|---|
| D1 (TX) | Rechtes Sensorkabel – Grün (RX) | Befehle senden |
| D0 (RX) | Rechtes Sensorkabel – Gelb (TX) | Feedback empfangen |
| GND | Rechtes Sensorkabel – Schwarz (GND) | Masse verbinden! |

### RA4M1 → Hoverboard 2 (Heck)

Nutzt **zweiten UART** auf D11/D12:

| RA4M1 Pin | Hoverboard-Kabel | Funktion |
|---|---|---|
| D11 (TX) | Rechtes Sensorkabel – Grün (RX) | Befehle senden |
| D12 (RX) | Rechtes Sensorkabel – Gelb (TX) | Feedback empfangen |
| GND | Rechtes Sensorkabel – Schwarz (GND) | Masse verbinden! |

> **Hinweis:** Die USART3-Pins des STM32F103 (rechtes Sensorkabel) sind 5V-tolerant. Die 3.3V-Signale des RA4M1 werden korrekt erkannt.

### Hoverboard Sensor-Kabel Pinout (rechts, 6-polig)

```
┌──────────────────────┐
│ 1 - GND  (Schwarz)   │
│ 2 - RX   (Grün)      │  ← Befehle vom RA4M1
│ 3 - TX   (Gelb)      │  → Feedback zum RA4M1
│ 4 - 3.3V (nicht ben.)│
│ 5 - Hall             │
│ 6 - Hall             │
└──────────────────────┘
```

## Hoverboard-Firmware Konfiguration

Beide Hoverboard-Mainboards müssen mit **VARIANT_USART** geflasht werden.

In der Hoverboard `config.h` müssen folgende Einstellungen aktiv sein:

```c
// Variante auswählen
#define VARIANT_USART

// Im VARIANT_USART Block:
#define CONTROL_SERIAL_USART3  0    // Steuerung über rechtes Sensorkabel
#define FEEDBACK_SERIAL_USART3      // Feedback über rechtes Sensorkabel
```

Beide Boards werden mit **identischer Firmware** geflasht. Das PlatformIO-Hauptprojekt (übergeordnetes Verzeichnis) ist bereits für `VARIANT_USART` konfiguriert.

## Software-Konfiguration

Alle Parameter befinden sich in [`include/config.h`](include/config.h):

### Wichtigste Parameter

| Parameter | Default | Beschreibung |
|---|---|---|
| `FRONT_STEER_RATIO` | 100% | Lenkanteil Vorderachse |
| `REAR_STEER_RATIO` | 100% | Lenkanteil Hinterachse |
| `EXPO_STEERING` | 30% | Expo-Kurve Lenkung (0=linear) |
| `EXPO_THROTTLE` | 20% | Expo-Kurve Gas (0=linear) |
| `RAMP_RATE` | 30 | Beschleunigungs-Rampe (pro 20ms) |
| `CMD_DEADBAND` | 30 | Totzone um Mittelstellung |
| `RC_FAILSAFE_TIMEOUT` | 500ms | Motorstopp bei Signalverlust |
| `NORMAL_SPEED_LIMIT` | 70% | Geschwindigkeitslimit Normal-Modus |
| `SPORT_SPEED_LIMIT` | 100% | Geschwindigkeitslimit Sport-Modus |
| `CRAWL_SPEED_LIMIT` | 35% | Geschwindigkeitslimit Crawl-Modus |

### Lenkverhalten anpassen

**Nur Vorderachs-Lenkung (Auto-artig):**
```c
#define FRONT_STEER_RATIO   100
#define REAR_STEER_RATIO    0
```

**Gleichmäßige 4WD-Lenkung:**
```c
#define FRONT_STEER_RATIO   100
#define REAR_STEER_RATIO    100
```

**Reduzierte Heck-Lenkung (stabiler bei hoher Geschwindigkeit):**
```c
#define FRONT_STEER_RATIO   100
#define REAR_STEER_RATIO    40
```

### Richtungsumkehr

Falls ein Board oder Motor in die falsche Richtung dreht:
```c
#define FRONT_SPEED_INVERT  true    // Front-Antrieb umkehren
#define REAR_STEER_INVERT   true    // Heck-Lenkrichtung umkehren
```

## Build & Upload

### Voraussetzungen

- [PlatformIO IDE](https://platformio.org/install/ide?install=vscode) (VS Code Extension)
- USB-Kabel für Arduino UNO R4 Minima

### Kompilieren und Flashen

```bash
cd RA4M1_4WD_Controller

# Kompilieren
pio run

# Kompilieren und Flashen
pio run --target upload

# Serial Monitor öffnen
pio device monitor
```

Oder in VS Code: PlatformIO Sidebar → Build / Upload / Monitor

## Debug-Ausgabe

Bei aktiviertem `DEBUG_ENABLE` (Standard: ein) werden über USB Serial (115200 Baud) folgende Daten ausgegeben:

```
RC: CH1=1500 CH2=1500 CH3=1500 CH4=1000 | Mode=NORM  Lim=70% | Front S=0 T=0 | Rear S=0 T=0
RC: CH1=1620 CH2=1700 CH3=1800 CH4=1000 | Mode=NORM  Lim=70% | Front S=85 T=140 | Rear S=85 T=140 | B1: V=38.5V T=28.3C sL=45 sR=47
```

## Fehlersuche

| Problem | Lösung |
|---|---|
| Kein RC-Signal | LED blinkt schnell (10 Hz). Empfänger-Binding prüfen, Verkabelung kontrollieren |
| Motoren reagieren nicht | Hoverboard `config.h` prüfen: `CONTROL_SERIAL_USART3` aktiv? GND verbunden? |
| Falsche Drehrichtung | `*_INVERT` Parameter in `config.h` anpassen |
| Lenkung vertauscht | `FRONT_STEER_INVERT` / `REAR_STEER_INVERT` umschalten |
| Board2 UART kompiliert nicht | Alternative: D11/D12 Pins ändern oder SoftwareSerial verwenden (siehe unten) |

### Alternative für zweiten UART

Falls `UART Board2Serial(D11, D12)` nicht mit Ihrer Core-Version funktioniert, können Sie stattdessen SoftwareSerial nutzen. In `main.cpp` ersetzen:

```cpp
// Statt:
UART Board2Serial(PIN_BOARD2_TX, PIN_BOARD2_RX);

// Nutzen:
#include <SoftwareSerial.h>
SoftwareSerial Board2Serial(PIN_BOARD2_RX, PIN_BOARD2_TX);  // RX, TX
```

## Lizenz

GPL-3.0 — passend zur hoverboard-firmware-hack-FOC Lizenz.
