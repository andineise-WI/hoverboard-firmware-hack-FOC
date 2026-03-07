// =============================================================================
//  RA4M1 4WD Hoverboard Controller — main.cpp
//
//  Reads PWM signals from a HOTRC DS650 RC receiver, performs 4WD mixing,
//  and sends UART commands to two hoverboard controllers (front + rear).
//
//  Target: Arduino UNO R4 Minima (Renesas RA4M1)
//  UART protocol compatible with:
//      https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Author:  auto-generated scaffold
//  License: GPL-3.0  (matches hoverboard firmware)
// =============================================================================

#include <Arduino.h>
#include "config.h"

// Define PIN_LED if not already defined in config.h
#ifndef PIN_LED
#define PIN_LED 13  // Default to built-in LED on pin 13
#endif

// ─────────────────────────── DATA STRUCTURES ────────────────────────────────

// Command packet sent TO a hoverboard (8 bytes)
typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  steer;
    int16_t  speed;
    uint16_t checksum;
} SerialCommand;

// Feedback packet received FROM a hoverboard (18 bytes)
typedef struct __attribute__((packed)) {
    uint16_t start;
    int16_t  cmd1;
    int16_t  cmd2;
    int16_t  speedR_meas;
    int16_t  speedL_meas;
    int16_t  batVoltage;
    int16_t  boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;

// Per-board state
typedef struct {
    SerialFeedback fb;
    SerialFeedback fbNew;
    uint8_t        fbIdx;
    byte          *fbPtr;
    byte           fbPrevByte;
    uint16_t       fbBufStart;
    bool           fbValid;
    unsigned long  fbLastTime;
} BoardState;

// ──────────────────────── SECOND UART (Board 2) ─────────────────────────────
// Serial1 is used for Board 1 (D0 RX, D1 TX).
// For Board 2 we instantiate a second hardware UART on D11/D12 (SCI0).
// If this does not compile with your core version, see README for alternatives.
UART Board2Serial(PIN_BOARD2_TX, PIN_BOARD2_RX);

// Convenience references
#define BOARD1_SERIAL   Serial1
#define BOARD2_SERIAL   Board2Serial

// ─────────────────────── RC RECEIVER VARIABLES ──────────────────────────────

// Pin array for easy iteration
static const uint8_t rcPins[RC_NUM_CHANNELS] = {
    PIN_RC_CH1, PIN_RC_CH2,
#if RC_NUM_CHANNELS >= 3
    PIN_RC_CH3,
#endif
#if RC_NUM_CHANNELS >= 4
    PIN_RC_CH4,
#endif
};

// Volatile data written from ISRs
static volatile uint32_t rcRiseTime[RC_NUM_CHANNELS];
static volatile uint16_t rcPulseWidth[RC_NUM_CHANNELS];
static volatile bool     rcNewData[RC_NUM_CHANNELS];
static volatile uint32_t rcLastGoodPulse[RC_NUM_CHANNELS];

// ──────────────────────── RC ISR HANDLERS ───────────────────────────────────
// Generic handler — reads the pulse width on CHANGE interrupt.
static void rcISR(uint8_t ch) {
    uint8_t pin = rcPins[ch];
    if (digitalRead(pin) == HIGH) {
        rcRiseTime[ch] = micros();
    } else {
        uint32_t pw = micros() - rcRiseTime[ch];
        if (pw >= 800 && pw <= 2200) {          // plausibility check
            rcPulseWidth[ch]    = (uint16_t)pw;
            rcNewData[ch]       = true;
            rcLastGoodPulse[ch] = millis();
        }
    }
}

// Individual ISR wrappers (attachInterrupt needs a void(*)() )
static void rcISR_CH1() { rcISR(0); }
static void rcISR_CH2() { rcISR(1); }
#if RC_NUM_CHANNELS >= 3
static void rcISR_CH3() { rcISR(2); }
#endif
#if RC_NUM_CHANNELS >= 4
static void rcISR_CH4() { rcISR(3); }
#endif

// ───────────────────────── GLOBAL STATE ─────────────────────────────────────

static BoardState board1, board2;

// Smoothed command outputs (after ramp)
static int16_t cmdSteerFront  = 0;
static int16_t cmdSpeedFront  = 0;
static int16_t cmdSteerRear   = 0;
static int16_t cmdSpeedRear   = 0;

// Timestamps
static unsigned long tSend     = 0;
static unsigned long tDebug    = 0;

// Active drive mode
static uint8_t driveMode = MODE_DEFAULT;

// ── Button toggle state for CH3 (speed limit) and CH4 (drive mode) ──────────
// Speed-limit steps (percent)
static const uint8_t speedLimitSteps[SPEED_LIMIT_NUM_STEPS] = { 25, 50, 75, 100 };
static uint8_t speedLimitStepIdx = SPEED_LIMIT_DEFAULT_STEP;

// Edge detection state
static bool    ch3ButtonPressed   = false;
static bool    ch4ButtonPressed   = false;
static unsigned long ch3LastToggle = 0;
static unsigned long ch4LastToggle = 0;

// ───────────────────────── HELPER FUNCTIONS ─────────────────────────────────

// Constrain to command range
static int16_t clampCmd(int32_t v) {
    if (v < CMD_MIN) return CMD_MIN;
    if (v > CMD_MAX) return CMD_MAX;
    return (int16_t)v;
}

// Apply dead-band around zero
static int16_t applyDeadband(int16_t value, int16_t band) {
    if (value > band)  return value - band;
    if (value < -band) return value + band;
    return 0;
}

// Apply exponential curve.  expo ∈ [0, 100]. 0 = linear, 100 = strong curve.
// Formula: out = (1 − k) * in + k * in³   (normalised to ±1 then scaled back)
static int16_t applyExpo(int16_t value, uint8_t expo) {
    if (expo == 0) return value;
    float k   = (float)expo / 100.0f;
    float v   = (float)value / (float)CMD_MAX;            // normalise to ±1
    float out = (1.0f - k) * v + k * v * v * v;
    return clampCmd((int32_t)(out * (float)CMD_MAX));
}

// Smooth ramping: move current towards target with limited step size
static int16_t rampValue(int16_t current, int16_t target) {
    int16_t diff = target - current;
    // Use faster ramp when decelerating (sign change or moving toward zero)
    int16_t maxStep = RAMP_RATE;
    bool decelerating = (target == 0) ||
                        (current > 0 && target < current) ||
                        (current < 0 && target > current);
    if (decelerating) maxStep = RAMP_RATE_BRAKE;

    if (diff >  maxStep) diff =  maxStep;
    if (diff < -maxStep) diff = -maxStep;
    return current + diff;
}

// Map RC pulse width (µs) to command range [CMD_MIN .. CMD_MAX]
static int16_t mapRC(uint16_t pulseWidth) {
    int32_t pw  = (int32_t)pulseWidth;
    int32_t mid = (int32_t)RC_PULSE_MID;
    int32_t lo  = (int32_t)RC_PULSE_MIN;
    int32_t hi  = (int32_t)RC_PULSE_MAX;

    // Apply dead-band in µs domain
    if (pw > mid - RC_PULSE_DEADBAND && pw < mid + RC_PULSE_DEADBAND)
        return 0;

    int32_t cmd;
    if (pw < mid)
        cmd = (int32_t)CMD_MIN * (mid - pw) / (mid - lo);
    else
        cmd = (int32_t)CMD_MAX * (pw - mid) / (hi - mid);

    return clampCmd(cmd);
}

// ──────────────────── HOVERBOARD UART: SEND ─────────────────────────────────

static void sendHoverCommand(Stream &serial, int16_t steer, int16_t speed) {
    SerialCommand cmd;
    cmd.start    = START_FRAME;
    cmd.steer    = steer;
    cmd.speed    = speed;
    cmd.checksum = (uint16_t)(cmd.start ^ cmd.steer ^ cmd.speed);
    serial.write((uint8_t *)&cmd, sizeof(cmd));
}

// ──────────────────── HOVERBOARD UART: RECEIVE ──────────────────────────────

static void receiveFeedback(Stream &serial, BoardState &bs) {
    while (serial.available()) {
        byte inByte = serial.read();
        bs.fbBufStart = ((uint16_t)inByte << 8) | bs.fbPrevByte;

        if (bs.fbBufStart == START_FRAME) {
            bs.fbPtr    = (byte *)&bs.fbNew;
            *bs.fbPtr++ = bs.fbPrevByte;
            *bs.fbPtr++ = inByte;
            bs.fbIdx    = 2;
        } else if (bs.fbIdx >= 2 && bs.fbIdx < sizeof(SerialFeedback)) {
            *bs.fbPtr++ = inByte;
            bs.fbIdx++;
        }

        if (bs.fbIdx == sizeof(SerialFeedback)) {
            uint16_t cs = (uint16_t)(
                bs.fbNew.start ^ bs.fbNew.cmd1 ^ bs.fbNew.cmd2 ^
                bs.fbNew.speedR_meas ^ bs.fbNew.speedL_meas ^
                bs.fbNew.batVoltage ^ bs.fbNew.boardTemp ^ bs.fbNew.cmdLed);

            if (bs.fbNew.start == START_FRAME && cs == bs.fbNew.checksum) {
                memcpy(&bs.fb, &bs.fbNew, sizeof(SerialFeedback));
                bs.fbValid    = true;
                bs.fbLastTime = millis();
            }
            bs.fbIdx = 0;
        }

        bs.fbPrevByte = inByte;
    }
}

// ────────────────────── BUTTON TOGGLE HELPERS ───────────────────────────────

// Detect rising edge (button press) with debounce.
// Returns true once per press event.
static bool detectButtonPress(uint16_t pw, bool &wasPressed, unsigned long &lastToggle, unsigned long now) {
    if (pw >= BUTTON_PULSE_THRESH && !wasPressed) {
        wasPressed = true;
        if (now - lastToggle >= BUTTON_DEBOUNCE_MS) {
            lastToggle = now;
            return true;   // rising edge detected
        }
    }
    if (pw <= BUTTON_RELEASE_THRESH) {
        wasPressed = false;
    }
    return false;
}

// Cycle CH4 drive mode: NORMAL → SPORT → CRAWL → NORMAL → ...
static void cycleDriveMode() {
    driveMode = (driveMode + 1) % MODE_NUM_MODES;
}

// Cycle CH3 speed-limit step
static void cycleSpeedLimit() {
    speedLimitStepIdx = (speedLimitStepIdx + 1) % SPEED_LIMIT_NUM_STEPS;
}

static uint8_t getSpeedLimitForMode(uint8_t mode) {
    switch (mode) {
        case MODE_SPORT: return SPORT_SPEED_LIMIT;
        case MODE_CRAWL: return CRAWL_SPEED_LIMIT;
        default:         return NORMAL_SPEED_LIMIT;
    }
}

// ═══════════════════════════════ SETUP ═══════════════════════════════════════

void setup() {
    // ── Debug serial (USB) ──────────────────────────────────────────────
#if DEBUG_ENABLE
    Serial.begin(DEBUG_BAUD);
    unsigned long t0 = millis();
    while (!Serial && (millis() - t0 < 2000)) { /* wait up to 2 s */ }
    Serial.println(F("=== RA4M1 4WD Hoverboard Controller ==="));
    Serial.println(F("Initialising ..."));
#endif

    // ── Hoverboard UARTs ────────────────────────────────────────────────
    BOARD1_SERIAL.begin(HOVER_BAUD);
    BOARD2_SERIAL.begin(HOVER_BAUD);

    // ── RC receiver pins & interrupts ───────────────────────────────────
    for (uint8_t i = 0; i < RC_NUM_CHANNELS; i++) {
        pinMode(rcPins[i], INPUT);
        rcPulseWidth[i]    = RC_PULSE_MID;      // safe default
        rcLastGoodPulse[i] = millis();
    }
    attachInterrupt(digitalPinToInterrupt(PIN_RC_CH1), rcISR_CH1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RC_CH2), rcISR_CH2, CHANGE);
#if RC_NUM_CHANNELS >= 3
    attachInterrupt(digitalPinToInterrupt(PIN_RC_CH3), rcISR_CH3, CHANGE);
#endif
#if RC_NUM_CHANNELS >= 4
    attachInterrupt(digitalPinToInterrupt(PIN_RC_CH4), rcISR_CH4, CHANGE);
#endif

    // ── Board state init ────────────────────────────────────────────────
    memset(&board1, 0, sizeof(board1));
    memset(&board2, 0, sizeof(board2));

    // ── LED ─────────────────────────────────────────────────────────────
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    tSend  = millis();
    tDebug = millis();

#if DEBUG_ENABLE
    Serial.println(F("Ready.  Waiting for RC signal ..."));
#endif
}

// ═══════════════════════════════ LOOP ════════════════════════════════════════

void loop() {
    unsigned long now = millis();

    // ── 1. Read feedback from both boards ───────────────────────────────
#if FEEDBACK_ENABLE
  #if FEEDBACK_BOARD1
    receiveFeedback(BOARD1_SERIAL, board1);
  #endif
  #if FEEDBACK_BOARD2
    receiveFeedback(BOARD2_SERIAL, board2);
  #endif
#endif

    // ── 2. Process RC data & send commands at fixed interval ────────────
    if (now - tSend < UART_SEND_INTERVAL) return;
    tSend = now;

    // --- Read RC channels (snapshot volatile data) -----------------------
    noInterrupts();
    uint16_t pw1 = rcPulseWidth[0];   // steering
    uint16_t pw2 = rcPulseWidth[1];   // throttle
    uint16_t pw3 = (RC_NUM_CHANNELS >= 3) ? rcPulseWidth[2] : RC_PULSE_MID;
    uint16_t pw4 = (RC_NUM_CHANNELS >= 4) ? rcPulseWidth[3] : RC_PULSE_MID;
    uint32_t lastCH1 = rcLastGoodPulse[0];
    uint32_t lastCH2 = rcLastGoodPulse[1];
    interrupts();

    // --- Failsafe: no RC signal → stop ──────────────────────────────────
    bool failsafe = false;
    if ((now - lastCH1 > RC_FAILSAFE_TIMEOUT) ||
        (now - lastCH2 > RC_FAILSAFE_TIMEOUT)) {
        failsafe = true;
    }

    // --- Map RC pulse widths to command range ───────────────────────────
    int16_t rawSteer    = failsafe ? 0 : mapRC(pw1);
    int16_t rawThrottle = failsafe ? 0 : mapRC(pw2);

    // --- Dead-band (in command domain) ──────────────────────────────────
    rawSteer    = applyDeadband(rawSteer,    CMD_DEADBAND);
    rawThrottle = applyDeadband(rawThrottle, CMD_DEADBAND);

    // --- Expo curves ────────────────────────────────────────────────────
    rawSteer    = applyExpo(rawSteer,    EXPO_STEERING);
    rawThrottle = applyExpo(rawThrottle, EXPO_THROTTLE);

    // --- Drive mode (CH4 button: tap to cycle) ─────────────────────────
    if (detectButtonPress(pw4, ch4ButtonPressed, ch4LastToggle, now)) {
        cycleDriveMode();
    }
    uint8_t modeLimitPct = getSpeedLimitForMode(driveMode);

    // --- Speed limit via CH3 button (tap to cycle) ──────────────────────
    uint8_t speedLimitPct = 100;
#if SPEED_LIMIT_ENABLE
    if (detectButtonPress(pw3, ch3ButtonPressed, ch3LastToggle, now)) {
        cycleSpeedLimit();
    }
    speedLimitPct = speedLimitSteps[speedLimitStepIdx];
#endif

    // Combine mode limit and speed-limit step (take the smaller)
    uint8_t effectiveLimit = (modeLimitPct < speedLimitPct) ? modeLimitPct : speedLimitPct;

    // --- Apply speed limit ──────────────────────────────────────────────
    int16_t maxCmd = (int16_t)((int32_t)CMD_MAX * effectiveLimit / 100);
    rawThrottle = clampCmd((int32_t)rawThrottle * effectiveLimit / 100);
    rawSteer    = clampCmd((int32_t)rawSteer    * effectiveLimit / 100);
    (void)maxCmd;   // suppress unused warning

    // --- 4WD mixing: compute per-axle steer + speed ─────────────────────
    int16_t targetSteerFront = clampCmd((int32_t)rawSteer * FRONT_STEER_RATIO / 100);
    int16_t targetSpeedFront = rawThrottle;
    int16_t targetSteerRear  = clampCmd((int32_t)rawSteer * REAR_STEER_RATIO  / 100);
    int16_t targetSpeedRear  = rawThrottle;

    // Invert if configured
    if (FRONT_STEER_INVERT) targetSteerFront = -targetSteerFront;
    if (FRONT_SPEED_INVERT) targetSpeedFront = -targetSpeedFront;
    if (REAR_STEER_INVERT)  targetSteerRear  = -targetSteerRear;
    if (REAR_SPEED_INVERT)  targetSpeedRear  = -targetSpeedRear;

    // --- Ramp (smooth acceleration / deceleration) ──────────────────────
    cmdSteerFront = rampValue(cmdSteerFront, targetSteerFront);
    cmdSpeedFront = rampValue(cmdSpeedFront, targetSpeedFront);
    cmdSteerRear  = rampValue(cmdSteerRear,  targetSteerRear);
    cmdSpeedRear  = rampValue(cmdSpeedRear,  targetSpeedRear);

    // --- Send commands to both hoverboards ──────────────────────────────
    sendHoverCommand(BOARD1_SERIAL, cmdSteerFront, cmdSpeedFront);
    sendHoverCommand(BOARD2_SERIAL, cmdSteerRear,  cmdSpeedRear);

    // --- LED indicator ──────────────────────────────────────────────────
    if (failsafe) {
        // Fast blink on failsafe (10 Hz)
        digitalWrite(PIN_LED, (now / 100) % 2);
    } else if (cmdSpeedFront != 0 || cmdSpeedRear != 0) {
        // Solid on when driving
        digitalWrite(PIN_LED, HIGH);
    } else {
        // Slow blink when idle (1 Hz)
        digitalWrite(PIN_LED, (now / 500) % 2);
    }

    // ── 3. Debug output ─────────────────────────────────────────────────
#if DEBUG_ENABLE
    if (now - tDebug >= DEBUG_INTERVAL) {
        tDebug = now;

        Serial.print(F("RC: CH1="));   Serial.print(pw1);
        Serial.print(F(" CH2="));      Serial.print(pw2);
        Serial.print(F(" CH3="));      Serial.print(pw3);
        Serial.print(F(" CH4="));      Serial.print(pw4);

        Serial.print(F(" | Mode="));
        switch (driveMode) {
            case MODE_SPORT: Serial.print(F("SPORT")); break;
            case MODE_CRAWL: Serial.print(F("CRAWL")); break;
            default:         Serial.print(F("NORM "));  break;
        }
        Serial.print(F(" SpdStep="));  Serial.print(speedLimitSteps[speedLimitStepIdx]);
        Serial.print(F("% Lim="));     Serial.print(effectiveLimit);
        Serial.print(F("%"));

        Serial.print(F(" | Front S="));Serial.print(cmdSteerFront);
        Serial.print(F(" T="));        Serial.print(cmdSpeedFront);
        Serial.print(F(" | Rear S=")); Serial.print(cmdSteerRear);
        Serial.print(F(" T="));        Serial.print(cmdSpeedRear);

        if (failsafe) Serial.print(F(" *** FAILSAFE ***"));

#if FEEDBACK_ENABLE && FEEDBACK_BOARD1
        if (board1.fbValid) {
            Serial.print(F(" | B1: V="));
            Serial.print(board1.fb.batVoltage / 100.0f, 1);
            Serial.print(F("V T="));
            Serial.print(board1.fb.boardTemp / 10.0f, 1);
            Serial.print(F("C sL="));
            Serial.print(board1.fb.speedL_meas);
            Serial.print(F(" sR="));
            Serial.print(board1.fb.speedR_meas);
        }
#endif
#if FEEDBACK_ENABLE && FEEDBACK_BOARD2
        if (board2.fbValid) {
            Serial.print(F(" | B2: V="));
            Serial.print(board2.fb.batVoltage / 100.0f, 1);
            Serial.print(F("V T="));
            Serial.print(board2.fb.boardTemp / 10.0f, 1);
            Serial.print(F("C sL="));
            Serial.print(board2.fb.speedL_meas);
            Serial.print(F(" sR="));
            Serial.print(board2.fb.speedR_meas);
        }
#endif
        Serial.println();
    }
#endif
}
