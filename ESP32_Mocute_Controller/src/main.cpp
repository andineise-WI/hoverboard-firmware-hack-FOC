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
 */

#include <Arduino.h>
#include <Bluepad32.h>
#include "hoverboard_serial.h"

// ========================== PIN Configuration ==========================
// UART2 pins for hoverboard communication
#define HOVER_RX_PIN    16    // ESP32 RX <- Hoverboard TX
#define HOVER_TX_PIN    17    // ESP32 TX -> Hoverboard RX

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

// ========================== Process Gamepad ==========================

void processGamepad(ControllerPtr ctl) {
    if (ctl == nullptr || !ctl->isConnected() || !ctl->hasData()) {
        return;
    }

    if (!ctl->isGamepad()) {
        return;
    }

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

    // Determine speed limit based on turbo mode
    int16_t speedLimit = turboMode ? SPEED_LIMIT_TURBO : SPEED_LIMIT_NORMAL;
    int16_t steerLimit = STEER_LIMIT;

    if (emergencyStop) {
        // Emergency stop: zero all outputs
        cmdSteer = 0;
        cmdSpeed = 0;
    } else {
        // Map joystick to commands
        // Y-axis: forward is typically negative on most gamepads, so invert
        cmdSpeed = mapJoystick(-axisY, JOYSTICK_DEADZONE, speedLimit);
        cmdSteer = mapJoystick(axisX, JOYSTICK_DEADZONE, steerLimit);
    }

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

// ========================== SETUP ==========================

void setup() {
    // Initialize debug serial
    Serial.begin(115200);
    Serial.println();
    Serial.println("============================================");
    Serial.println("  ESP32 Mocute 052 -> Hoverboard Controller");
    Serial.println("============================================");
    Serial.println();

    // LED setup
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Initialize hoverboard serial
    hoverSerial.begin();
    Serial.printf("[HOVER] Serial initialized at %d baud (RX=%d, TX=%d)\n",
                  HOVER_SERIAL_BAUD, HOVER_RX_PIN, HOVER_TX_PIN);

    // Send initial stop command
    hoverSerial.send(0, 0);

    // Initialize Bluepad32
    Serial.println("[BP32]  Initializing Bluepad32...");
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // Optionally forget previously paired devices to allow new pairing
    // BP32.forgetBluetoothKeys();

    // Enable virtual devices (e.g., mouse, keyboard) - usually not needed for gamepad
    // BP32.enableVirtualDevice(false);

    Serial.println("[BP32]  Ready! Waiting for Mocute 052 gamepad...");
    Serial.println("[BP32]  Turn on the Mocute 052 and set it to Game mode.");
    Serial.println();

    // Initialize controller slots
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        myControllers[i] = nullptr;
    }
}

// ========================== LOOP ==========================

void loop() {
    // Must call this to process Bluepad32 events
    bool dataUpdated = BP32.update();

    if (dataUpdated) {
        // Process all connected controllers
        for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
            if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
                processGamepad(myControllers[i]);
            }
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
            Serial.printf("[CMD] steer=%4d speed=%4d turbo=%d estop=%d connected=%d\n",
                          cmdSteer, cmdSpeed, turboMode, emergencyStop, gamepadConnected);
        }
    }

    // Small delay to prevent WDT issues
    vTaskDelay(1);
}
