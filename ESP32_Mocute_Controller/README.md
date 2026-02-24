# ESP32 Mocute 052 -> Hoverboard Controller

ESP32-basierter Bluetooth-Empfänger für den **Mocute 052** Gamepad-Controller.
Steuert das Hoverboard über die serielle UART-Schnittstelle mit dem
[hoverboard-firmware-hack-FOC](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC) Protokoll.

## Funktionsübersicht

```
Mocute 052  ──BT──>  ESP32  ──UART──>  Hoverboard Mainboard
(Gamepad)           (dieser Code)      (FOC Firmware)
```

## Hardware

### Benötigte Komponenten
- ESP32 DevKit V1 (oder kompatibel)
- Mocute 052 Bluetooth Gamepad
- Hoverboard mit FOC Firmware
- 3 Kabel für die Verbindung (GND, TX, RX)

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

## Installation & Build

### Voraussetzungen
- [PlatformIO](https://platformio.org/) (VS Code Extension oder CLI)

### Build & Flash

```bash
# In das Projektverzeichnis wechseln
cd ESP32_Mocute_Controller

# Build
pio run

# Flash auf ESP32
pio run --target upload

# Serial Monitor öffnen
pio device monitor
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
============================================

[HOVER] Serial initialized at 115200 baud (RX=16, TX=17)
[BP32]  Initializing Bluepad32...
[BP32]  Ready! Waiting for Mocute 052 gamepad...

[BP32]  Controller connected, index=0
[BP32]  Controller model: Mocute-052, VID=0x04e8, PID=0x04e8
[CMD]   steer=   0 speed=   0 turbo=0 estop=0 connected=1
[CMD]   steer= 150 speed= 200 turbo=0 estop=0 connected=1
[HOVER] Feedback: speedL=180 speedR=185 bat=36.5V temp=28°C
```

## Projektstruktur

```
ESP32_Mocute_Controller/
├── platformio.ini            # PlatformIO Konfiguration
├── README.md                 # Diese Datei
├── include/
│   └── hoverboard_serial.h  # Hoverboard Protokoll-Implementierung
└── src/
    └── main.cpp              # Hauptprogramm (Bluepad32 + Steuerlogik)
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
#define HOVER_RX_PIN    16    // ESP32 RX
#define HOVER_TX_PIN    17    // ESP32 TX
```

## Lizenz

GPL v3 - Kompatibel mit der hoverboard-firmware-hack-FOC Lizenz.
