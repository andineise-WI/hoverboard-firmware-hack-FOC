/**
 * @file main.cpp
 * @brief ESP32 Bluetooth Gamepad Receiver for Hoverboard Control
 *
 * Receives input from a Mocute 052 Bluetooth gamepad via Bluepad32
 * and sends steer/speed commands to the hoverboard mainboard via UART.
 *
 * Hardware Connections (ESP32 -> Hoverboard):
 *   ESP32 GPIO16 (RX2) -> Hoverboard TX (Green wire on sensor cable)
 *   ESP32 GPIO17 (TX2) -> Hoverboard RX (Yellow wire on sensor cable)
 *   ESP32 GND          -> Hoverboard GND (Black wire on sensor cable)
 *   WARNING: Do NOT connect the red 15V wire!
 *
 * Hoverboard config.h must have:
 *   #define CONTROL_SERIAL_USART3   (right sensor cable)
 *   #define FEEDBACK_SERIAL_USART3
 *   or
 *   #define CONTROL_SERIAL_USART2   (left sensor cable, 3.3V only!)
 *   #define FEEDBACK_SERIAL_USART2
 *
 * Mocute 052 Pairing:
 *   1. Turn on the Mocute 052 controller (hold power button)
 *   2. Set it to "Game" mode (Mode B / iCade mode depending on your model)
 *   3. The ESP32 will automatically discover and connect to the gamepad
 *   4. LED on ESP32 will be ON when a gamepad is connected
 *
 * Control Mapping:
 *   Left Joystick Y-axis  -> Speed (forward/backward)
 *   Left Joystick X-axis  -> Steering (left/right)
 *   Button A (or R1)      -> Turbo mode (increased speed limit)
 *   Button B (or L1)      -> Emergency stop (sets speed & steer to 0)
 *   Button X              -> Toggle Follow Me mode (RSSI-based)
 *   Button Y              -> RSSI/Direction calibration print
 *
 * Follow Me Mode:
 *   When enabled (Button X toggle), the hoverboard automatically
 *   adjusts speed based on BT signal strength (RSSI).
 *   - Closer signal = stop/reverse
 *   - Farther signal = drive forward
 *
 * Dual-ESP32 Direction Detection:
 *   When a Satellite ESP32 is present on the opposite sideboard,
 *   the direction to the user is estimated from RSSI difference.
 *   - Steering is automatic when direction data is available
 *   - Manual joystick steering overrides auto-steering
 *   - Without Satellite, steering remains manual-only
 */

#include <Arduino.h>
#include <Bluepad32.h>
// WiFi/ESP-NOW disabled: conflicts with Bluepad32's BTstack radio control
// #include <WiFi.h>
// #include <esp_now.h>
#include "hoverboard_serial.h"
#include "follow_me.h"
#include "espnow_protocol.h"
#include "direction_detect.h"

// Access Bluepad32 internals for RSSI reading
extern "C" {
    #include "uni_hid_device.h"
    #include "uni_bt.h"
}

// ========================== PIN Configuration ==========================
// UART2 pins for hoverboard communication
// NOTE: GPIO16/17 are used by PSRAM in the Bluepad32 framework!
//       Must use alternative pins.
#define HOVER_RX_PIN    25    // ESP32 RX <- Hoverboard TX
#define HOVER_TX_PIN    26    // ESP32 TX -> Hoverboard RX

// Status LED
#define LED_PIN         2     // Built-in LED on most ESP32 DevKit boards

// ========================== Gamepad Settings ==========================
// Joystick deadzone (Mocute 052 joysticks may not center perfectly)
#define JOYSTICK_DEADZONE   30    // Values within +/- deadzone are treated as 0

// Joystick input range (Bluepad32 reports -512 to 511 for axes)
#define JOYSTICK_INPUT_MIN  -512
#define JOYSTICK_INPUT_MAX   511

// Speed limits
#define SPEED_LIMIT_NORMAL  300   // Normal speed limit [-1000, 1000]
#define SPEED_LIMIT_TURBO   600   // Turbo speed limit (when turbo button held)
#define STEER_LIMIT         400   // Steering limit [-1000, 1000]

// Safety: timeout if no gamepad data received
#define GAMEPAD_TIMEOUT_MS  500   // [ms] Stop motors if no gamepad update within this time

// ========================== Global Variables ==========================
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Hoverboard serial interface on UART2
HoverboardSerial hoverSerial(Serial2, HOVER_RX_PIN, HOVER_TX_PIN);

// Current command values
int16_t cmdSteer = 0;
int16_t cmdSpeed = 0;
bool turboMode = false;
bool emergencyStop = false;

// Follow Me mode
FollowMe followMe;
DirectionDetector dirDetector;
bool followMeButtonPrev = false;   // For edge detection on toggle button
bool rssiCalibButtonPrev = false;  // For edge detection on calibration button
unsigned long lastRssiReadTime = 0;

// ESP-NOW / Satellite state
bool satelliteOnline = false;
unsigned long lastSatelliteMsg = 0;

// Timing
unsigned long lastSendTime = 0;
unsigned long lastGamepadTime = 0;
bool gamepadConnected = false;

// ========================== Helper Functions ==========================

/**
 * @brief Apply deadzone and map joystick value to output range
 * @param value Raw joystick value (typically -512 to 511)
 * @param deadzone Deadzone threshold
 * @param outMax Maximum output value
 * @return Mapped value in range [-outMax, outMax]
 */
int16_t mapJoystick(int32_t value, int16_t deadzone, int16_t outMax) {
    // Apply deadzone
    if (abs(value) < deadzone) {
        return 0;
    }

    // Remove deadzone offset
    if (value > 0) {
        value -= deadzone;
    } else {
        value += deadzone;
    }

    // Map to output range
    int32_t inputRange = JOYSTICK_INPUT_MAX - deadzone;
    if (inputRange <= 0) return 0;

    int32_t result = (value * (int32_t)outMax) / inputRange;

    // Clamp
    if (result > outMax) result = outMax;
    if (result < -outMax) result = -outMax;

    return (int16_t)result;
}

// ========================== Bluepad32 Callbacks ==========================

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("[BP32] Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            foundEmptySlot = true;

            // Print controller info
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("[BP32] Controller model: %s, VID=0x%04x, PID=0x%04x\n",
                          ctl->getModelName().c_str(),
                          properties.vendor_id,
                          properties.product_id);

            // Print BT protocol type
            if (ctl->isGamepad())        Serial.println("[BP32] Type: Gamepad");
            else if (ctl->isMouse())     Serial.println("[BP32] Type: Mouse");
            else if (ctl->isKeyboard())  Serial.println("[BP32] Type: Keyboard");
            else if (ctl->isBalanceBoard()) Serial.println("[BP32] Type: Balance Board");
            else                         Serial.println("[BP32] Type: Unknown");

            // Print BT transport from Bluepad32 internals
            uni_hid_device_t* dev = uni_hid_device_get_instance_for_idx(i);
            if (dev != nullptr) {
                const char* btProto = "Unknown";
                switch (dev->conn.protocol) {
                    case UNI_BT_CONN_PROTOCOL_BR_EDR: btProto = "BR/EDR (Classic BT)"; break;
                    case UNI_BT_CONN_PROTOCOL_BLE:    btProto = "BLE (Bluetooth Low Energy)"; break;
                    default: break;
                }
                Serial.printf("[BP32] BT Protocol: %s\n", btProto);
                char addr_str[18];
                snprintf(addr_str, sizeof(addr_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                         dev->conn.btaddr[0], dev->conn.btaddr[1], dev->conn.btaddr[2],
                         dev->conn.btaddr[3], dev->conn.btaddr[4], dev->conn.btaddr[5]);
                Serial.printf("[BP32] BT Address: %s\n", addr_str);
                Serial.printf("[BP32] Connection handle: 0x%04x\n", dev->conn.handle);
            }
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("[BP32] CALLBACK: Controller connected, but no empty slot found");
    }

    gamepadConnected = true;
    digitalWrite(LED_PIN, HIGH);
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("[BP32] Controller disconnected, index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController) {
        Serial.println("[BP32] CALLBACK: Controller disconnected, but not found in list");
    }

    // Check if any controller is still connected
    gamepadConnected = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr) {
            gamepadConnected = true;
            break;
        }
    }

    if (!gamepadConnected) {
        // Safety: stop motors when all controllers disconnect
        cmdSteer = 0;
        cmdSpeed = 0;
        turboMode = false;
        emergencyStop = false;
        digitalWrite(LED_PIN, LOW);
        Serial.println("[SAFETY] All controllers disconnected - motors stopped");
    }
}

// ========================== ESP-NOW (Satellite Communication) ==========================
// DISABLED: WiFi/ESP-NOW conflicts with Bluepad32's BTstack radio control
// causing WDT reset. All ESP-NOW code commented out until a compatible
// solution is found (e.g. using esp_wifi_start() directly instead of Arduino WiFi).

/*
void onEspNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    if (len < (int)sizeof(EspNowHeader)) return;

    const EspNowHeader* hdr = (const EspNowHeader*)data;
    if (!espnow_validate_header(hdr)) return;

    lastSatelliteMsg = millis();
    satelliteOnline = true;

    switch (hdr->msgType) {
        case MSG_TYPE_RSSI_REPORT: {
            if (len >= (int)sizeof(EspNowRssiReport)) {
                const EspNowRssiReport* report = (const EspNowRssiReport*)data;
                if (report->targetFound) {
                    dirDetector.updateSatelliteRssi(report->rssi_raw, report->confidence);
                }
            }
            break;
        }

        case MSG_TYPE_HEARTBEAT: {
            break;
        }

        default:
            break;
    }
}

bool setupEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    Serial.printf("[ESPNOW] Master MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("[ESPNOW] Set this MAC in the Satellite's masterMacAddress[]!");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] Init failed!");
        return false;
    }

    esp_now_register_recv_cb(onEspNowDataRecv);

    Serial.println("[ESPNOW] Ready, waiting for Satellite...");
    return true;
}
*/

// ========================== RSSI Reading ==========================

// Store last RSSI value received via HCI event
static int8_t lastRssiValue = 0;

/**
 * @brief Read RSSI from the last HCI measurement result
 * @param ctl Controller pointer
 * @return RSSI value (0-255, higher = closer) or 0 if not available
 *
 * Note: Bluepad32 v3.x does not store RSSI in the connection struct.
 * We request RSSI via gap_read_rssi() and the result arrives
 * asynchronously via HCI. We store it in lastRssiValue.
 * The value is converted: RSSI_dBm is negative (e.g. -60),
 * we convert to 0-255 range: mapped = constrain(100 + rssi_dbm, 0, 255)
 */
uint8_t readControllerRssi(ControllerPtr ctl) {
    if (ctl == nullptr) return 0;

    // Convert raw RSSI (dBm, typically -100 to 0) to 0-255 range
    // where higher = closer
    int mapped = 100 + (int)lastRssiValue;  // e.g. -60 dBm -> 40
    return (uint8_t)constrain(mapped * 2, 0, 255);
}

/**
 * @brief Request a fresh RSSI reading from the BT stack
 *        This triggers a GAP_EVENT_RSSI_MEASUREMENT which updates conn.rssi
 */
void requestRssiUpdate() {
    for (int i = 0; i < CONFIG_BLUEPAD32_MAX_DEVICES; i++) {
        uni_hid_device_t* dev = uni_hid_device_get_instance_for_idx(i);
        if (dev != nullptr && uni_bt_conn_is_connected(&dev->conn) &&
            dev->conn.handle != UNI_BT_CONN_HANDLE_INVALID) {
            // Request RSSI update from BT stack (result arrives async in packet handler)
            gap_read_rssi(dev->conn.handle);
        }
    }
}

// ========================== Process Gamepad ==========================

void processGamepad(ControllerPtr ctl) {
    if (ctl == nullptr || !ctl->isConnected()) {
        return;
    }

    // Accept any controller type - Mocute 052 emulates Xbox One via BLE
    // and may not be classified as "Gamepad" by Bluepad32 v3.x
    // if (!ctl->isGamepad()) { return; }

    lastGamepadTime = millis();

    // Read joystick axes
    // Bluepad32: axisX/axisY = left stick, axisRX/axisRY = right stick
    // Range: -512 to 511
    int32_t axisX = ctl->axisX();   // Left stick X (steering)
    int32_t axisY = ctl->axisY();   // Left stick Y (speed)

    // Check buttons
    uint16_t buttons = ctl->buttons();

    // Button A or R1 = Turbo mode
    turboMode = (buttons & BUTTON_A) || (buttons & BUTTON_SHOULDER_R);

    // Button B or L1 = Emergency stop
    emergencyStop = (buttons & BUTTON_B) || (buttons & BUTTON_SHOULDER_L);

    // Button X = Toggle Follow Me mode (edge detection: only on press, not hold)
    bool followMeButton = (buttons & BUTTON_X) != 0;
    if (followMeButton && !followMeButtonPrev) {
        followMe.toggle();
    }
    followMeButtonPrev = followMeButton;

    // Button Y = Print RSSI/Direction calibration info (edge detection)
    bool rssiCalibButton = (buttons & BUTTON_Y) != 0;
    if (rssiCalibButton && !rssiCalibButtonPrev) {
        uint8_t rssi = readControllerRssi(ctl);
        Serial.printf("[RSSI-CAL] Raw RSSI=%u, Filtered=%.1f, Zone=%s\n",
                      rssi, followMe.getFilteredRssi(), followMe.getZoneName());
        if (dirDetector.isAvailable()) {
            dirDetector.printCalibration();
        } else {
            Serial.println("[RSSI-CAL] Direction: not available (no Satellite data)");
        }
        Serial.println("[RSSI-CAL] Hold gamepad at desired distance and note these values.");
        Serial.println("[RSSI-CAL] Adjust RSSI_ZONE_* thresholds in follow_me.h accordingly.");
    }
    rssiCalibButtonPrev = rssiCalibButton;

    // Determine speed limit based on turbo mode
    int16_t speedLimit = turboMode ? SPEED_LIMIT_TURBO : SPEED_LIMIT_NORMAL;
    int16_t steerLimit = STEER_LIMIT;

    if (emergencyStop) {
        // Emergency stop: zero all outputs (overrides Follow Me)
        cmdSteer = 0;
        cmdSpeed = 0;
        if (followMe.isEnabled()) {
            followMe.disable();
            Serial.println("[FOLLOW] Emergency stop - Follow Me disabled");
        }
    } else if (followMe.isEnabled()) {
        // === FOLLOW ME MODE ===
        // Speed comes from RSSI-based distance estimation
        cmdSpeed = followMe.getFollowSpeed();

        // Steering: auto-direction from Dual-ESP32 if available,
        // otherwise manual via joystick
        int16_t manualSteer = mapJoystick(axisX, JOYSTICK_DEADZONE, steerLimit);

        if (manualSteer != 0) {
            // Manual joystick always overrides auto-steering
            cmdSteer = manualSteer;
        } else if (followMe.hasDirection()) {
            // Auto-steering from direction detection (Dual-ESP32)
            cmdSteer = followMe.getDirectionSteer();
        } else {
            cmdSteer = 0;
        }

        // D-Pad can override steering in Follow Me mode
        uint8_t dpad = ctl->dpad();
        if (dpad & DPAD_LEFT) {
            cmdSteer = -steerLimit / 2;
        }
        if (dpad & DPAD_RIGHT) {
            cmdSteer = steerLimit / 2;
        }
    } else {
        // === NORMAL MODE ===
        // Map joystick to commands
        // Y-axis: forward is typically negative on most gamepads, so invert
        cmdSpeed = mapJoystick(-axisY, JOYSTICK_DEADZONE, speedLimit);
        cmdSteer = mapJoystick(axisX, JOYSTICK_DEADZONE, steerLimit);

        // D-Pad for fine control (optional, overrides joystick if pressed)
        uint8_t dpad = ctl->dpad();
        if (dpad & DPAD_UP) {
            cmdSpeed = speedLimit / 3;    // Slow forward
        }
        if (dpad & DPAD_DOWN) {
            cmdSpeed = -speedLimit / 3;   // Slow backward
        }
        if (dpad & DPAD_LEFT) {
            cmdSteer = -steerLimit / 2;   // Turn left
        }
        if (dpad & DPAD_RIGHT) {
            cmdSteer = steerLimit / 2;    // Turn right
        }
    }

    // === RSSI Update ===
    // Read RSSI periodically for Follow Me mode + Direction detection
    unsigned long now = millis();
    if (now - lastRssiReadTime >= RSSI_READ_INTERVAL_MS) {
        lastRssiReadTime = now;

        // Request fresh RSSI from BT stack
        requestRssiUpdate();

        // Read current RSSI value
        uint8_t rssi = readControllerRssi(ctl);
        if (rssi > 0) {
            followMe.updateRssi(rssi);

            // Feed master RSSI to direction detector
            dirDetector.updateMasterRssi(rssi);

            // Update Follow Me with direction steering (if available)
            followMe.setDirectionSteer(dirDetector.getDirectionSteer());
        }
    }
}

// ========================== SETUP ==========================

void setup() {
    // Initialize debug serial
    Serial.begin(115200);
    delay(1000);  // Give BTstack time to fully initialize on Core 0
    Serial.println();
    Serial.println("============================================");
    Serial.println("  ESP32 Mocute 052 -> Hoverboard Controller");
    Serial.println("       + Follow Me Mode (RSSI-based)");
    Serial.println("       + Dual-ESP32 Direction Detection");
    Serial.println("============================================");
    Serial.println();
    Serial.flush();

    Serial.println("[DBG] Step 1: LED setup...");
    Serial.flush();
    // LED setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("[DBG] Step 2: Hoverboard serial...");
    Serial.flush();
    // Initialize hoverboard serial
    hoverSerial.begin();
    Serial.printf("[HOVER] Serial initialized at %d baud (RX=%d, TX=%d)\n",
                  HOVER_SERIAL_BAUD, HOVER_RX_PIN, HOVER_TX_PIN);
    Serial.flush();

    // Send initial stop command
    hoverSerial.send(0, 0);

    Serial.println("[DBG] Step 3: Bluepad32 setup...");
    Serial.flush();
    // Initialize Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    Serial.println("[DBG] Step 3: Bluepad32 setup DONE");
    Serial.flush();

    // ESP-NOW disabled: WiFi.mode(WIFI_STA) conflicts with Bluepad32's BTstack
    // which directly controls the radio. This causes WDT reset.
    // TODO: Investigate Bluepad32-compatible ESP-NOW initialization
    Serial.println("[ESPNOW] ESP-NOW disabled (incompatible with BTstack radio control)");
    Serial.println("[ESPNOW] Direction detection unavailable. Follow Me = distance-only.");
    /*
    Serial.println("[ESPNOW] Initializing ESP-NOW for Satellite communication...");
    if (setupEspNow()) {
        Serial.println("[ESPNOW] ESP-NOW ready. Satellite can connect for direction detection.");
    } else {
        Serial.println("[ESPNOW] ESP-NOW init failed. Direction detection unavailable.");
        Serial.println("[ESPNOW] Follow Me will work with distance-only (no direction).");
    }
    */

    // Optionally forget previously paired devices to allow new pairing
    // BP32.forgetBluetoothKeys();

    // Enable virtual devices (e.g., mouse, keyboard) - usually not needed for gamepad
    // BP32.enableVirtualDevice(false);

    Serial.println("[BP32]  Ready! Waiting for Mocute 052 gamepad...");
    Serial.println("[BP32]  Turn on the Mocute 052 and set it to Game mode.");
    Serial.println();
    Serial.println("Controls:");
    Serial.println("  Joystick L  = Steer + Speed");
    Serial.println("  Button A/R1 = Turbo");
    Serial.println("  Button B/L1 = Emergency Stop");
    Serial.println("  Button X    = Toggle Follow Me mode");
    Serial.println("  Button Y    = Print RSSI/Direction calibration values");
    Serial.println();
    Serial.println("Dual-ESP32:");
    Serial.println("  Satellite auto-detected via ESP-NOW");
    Serial.println("  When online: auto-steering in Follow Me mode");
    Serial.println();

    // Initialize controller slots
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        myControllers[i] = nullptr;
    }
}

// ========================== LOOP ==========================

void loop() {
    // Must call this to process Bluepad32 events
    BP32.update();

    // Process all connected controllers
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
            processGamepad(myControllers[i]);
        }
    }

    // Safety timeout: stop if no gamepad data for too long
    if (gamepadConnected && (millis() - lastGamepadTime > GAMEPAD_TIMEOUT_MS)) {
        cmdSteer = 0;
        cmdSpeed = 0;
        static unsigned long lastTimeoutMsg = 0;
        if (millis() - lastTimeoutMsg > 2000) {
            Serial.println("[SAFETY] Gamepad timeout - motors stopped");
            lastTimeoutMsg = millis();
        }
    }

    // Send commands to hoverboard at fixed interval
    unsigned long now = millis();
    if (now - lastSendTime >= HOVER_SEND_INTERVAL) {
        lastSendTime = now;

        // Send command
        hoverSerial.send(cmdSteer, cmdSpeed);

        // Try to receive feedback
        if (hoverSerial.receive()) {
            static unsigned long lastFeedbackPrint = 0;
            if (now - lastFeedbackPrint > 1000) {  // Print feedback every 1s
                lastFeedbackPrint = now;
                Serial.printf("[HOVER] Feedback: speedL=%d speedR=%d bat=%d.%dV temp=%d°C\n",
                              hoverSerial.feedback.speedL_meas,
                              hoverSerial.feedback.speedR_meas,
                              hoverSerial.feedback.batVoltage / 100,
                              abs(hoverSerial.feedback.batVoltage % 100),
                              hoverSerial.feedback.boardTemp / 10);
            }
        }

        // Debug output (every 500ms)
        static unsigned long lastDebugPrint = 0;
        if (now - lastDebugPrint > 500) {
            lastDebugPrint = now;
            if (followMe.isEnabled()) {
                if (dirDetector.isAvailable()) {
                    Serial.printf("[FOLLOW] steer=%4d speed=%4d zone=%s dir=%s rssi_m=%u rssi_s=%u diff=%.0f\n",
                                  cmdSteer, cmdSpeed, followMe.getZoneName(),
                                  dirDetector.getZoneName(),
                                  dirDetector.getMasterRssi(), dirDetector.getSatelliteRssi(),
                                  dirDetector.getFilteredDiff());
                } else {
                    Serial.printf("[FOLLOW] steer=%4d speed=%4d zone=%s rssi=%u(%.0f) sat=%s\n",
                                  cmdSteer, cmdSpeed, followMe.getZoneName(),
                                  followMe.getRawRssi(), followMe.getFilteredRssi(),
                                  dirDetector.isSatelliteOnline() ? "online" : "offline");
                }
            } else {
                Serial.printf("[CMD] steer=%4d speed=%4d turbo=%d estop=%d connected=%d sat=%s\n",
                              cmdSteer, cmdSpeed, turboMode, emergencyStop, gamepadConnected,
                              dirDetector.isSatelliteOnline() ? "online" : "offline");
            }
        }
    }

    // Small delay to prevent WDT issues
    vTaskDelay(1);
}
